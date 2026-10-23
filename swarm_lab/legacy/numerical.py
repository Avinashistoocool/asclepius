"""Cubic-spline collocation of the normal-chord midpoint equation."""

import time

import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import least_squares

from boundary import cross, inside, intersection_parameters, segment_inside


def normal_chords(points, velocities, polygon):
    speed = np.linalg.norm(velocities, axis=1)
    if np.any(speed < 1e-10):
        raise ValueError("The candidate trajectory has a singular tangent.")
    normals = np.c_[-velocities[:, 1], velocities[:, 0]] / speed[:, None]
    edges = np.roll(polygon, -1, axis=0) - polygon
    rel = polygon[None, :, :] - points[:, None, :]
    den = cross(normals[:, None, :], edges[None, :, :])
    nonparallel = np.abs(den) > 1e-10
    safe = np.where(nonparallel, den, 1.)
    signed = cross(rel, edges[None, :, :]) / safe
    along = cross(rel, normals[:, None, :]) / safe
    valid = nonparallel & (along >= -1e-10) & (along <= 1 + 1e-10)
    positive = np.where(valid & (signed >= -1e-9), signed, np.inf)
    negative = np.where(valid & (signed <= 1e-9), signed, -np.inf)
    ip = np.argmin(positive, axis=1)
    im = np.argmax(negative, axis=1)
    row = np.arange(len(points))
    plus, minus = positive[row, ip], negative[row, im]
    resolved = np.isfinite(plus) & np.isfinite(minus)
    return minus, plus, im, ip, normals, resolved


def basis(knots, parameters, derivative=0):
    return BSpline(knots, np.eye(len(knots) - 4), 3)(parameters, derivative)


def slice_midpoints(polygon, direction, normal, low, high, parameters):
    projection = polygon @ normal
    reference = float(np.mean(projection))
    mids = []
    for t in parameters:
        point = (low + (high - low) * t) * direction + reference * normal
        hits = intersection_parameters(point, normal, polygon)
        hits = np.unique(np.round(hits, 10))
        intervals = [
            (a, b) for a, b in zip(hits[:-1], hits[1:])
            if b - a > 1e-8 and inside(point + .5 * (a + b) * normal, polygon)
        ]
        if not intervals:
            raise ValueError("This scan direction has no connected interior chord.")
        a, b = min(intervals, key=lambda pair: abs(.5 * (pair[0] + pair[1])))
        reference += .5 * (a + b)
        mids.append(reference)
    return np.asarray(mids)


def spline_pieces(spline, direction, normal, low, high):
    breaks = np.unique(spline.t)
    pieces = []
    for a, b in zip(breaks[:-1], breaks[1:]):
        p0 = (low + (high - low) * a) * direction + spline(a) * normal
        p3 = (low + (high - low) * b) * direction + spline(b) * normal
        v0 = (high - low) * direction + spline(a, 1) * normal
        v1 = (high - low) * direction + spline(b, 1) * normal
        controls = np.array([p0, p0 + (b - a) * v0 / 3,
                             p3 - (b - a) * v1 / 3, p3])
        pieces.append({"interval": [float(a), float(b)], "controls": controls.tolist(),
                       "weights": [1., 1., 1., 1.]})
    return pieces


def split_bezier(controls):
    a = (controls[:-1] + controls[1:]) / 2
    b = (a[:-1] + a[1:]) / 2
    c = (b[0] + b[1]) / 2
    return np.array([controls[0], a[0], b[0], c]), np.array([c, b[1], a[2], controls[3]])


def certify_containment(controls, polygon, depth=0):
    if all(inside(p, polygon) for p in controls) and all(
        segment_inside(controls[i], controls[j], polygon)
        for i in range(4) for j in range(i + 1, 4)
    ):
        return True
    if depth >= 16:
        return False
    left, right = split_bezier(controls)
    return certify_containment(left, polygon, depth + 1) and certify_containment(right, polygon, depth + 1)


def nearest_boundary(point, polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    along = np.clip(np.sum((point - polygon) * edges, axis=1) / np.sum(edges ** 2, axis=1), 0, 1)
    candidates = polygon + along[:, None] * edges
    index = np.argmin(np.linalg.norm(candidates - point, axis=1))
    return candidates[index]


def solve_chart(polygon, direction, tolerance, deadline, controls_count, endpoints=None):
    direction = direction / np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]])
    projected = polygon @ direction
    minimum, maximum = float(np.min(projected)), float(np.max(projected))
    span = maximum - minimum
    low, high = minimum + .005 * span, maximum - .005 * span
    if endpoints is not None:
        low, high = float(endpoints[0] @ direction), float(endpoints[1] @ direction)
    knots = np.r_[np.zeros(4), np.linspace(0, 1, controls_count - 2)[1:-1], np.ones(4)]
    parameters = np.unique(np.r_[
        np.linspace(.0001, .9999, controls_count * 5),
        .5 - .5 * np.cos(np.linspace(.002, np.pi - .002, controls_count * 3)),
    ])
    b0, b1 = basis(knots, parameters), basis(knots, parameters, 1)
    target = slice_midpoints(polygon, direction, normal, low, high, parameters)
    y = np.linalg.lstsq(b0, target, rcond=None)[0]
    initial = np.r_[y, low, high]
    free = np.arange(controls_count + 2)
    if endpoints is not None:
        initial[0], initial[controls_count - 1] = endpoints[0] @ normal, endpoints[1] @ normal
        initial[1] = initial[0] + (endpoints[2] @ normal) / (endpoints[2] @ direction) * (high - low) * knots[4] / 3
        initial[controls_count - 2] = initial[controls_count - 1] - (endpoints[3] @ normal) / (endpoints[3] @ direction) * (high - low) * (1 - knots[-5]) / 3
        initial[2:controls_count - 2] = 0
        initial[2:controls_count - 2] = np.linalg.lstsq(
            b0[:, 2:-2], target - b0 @ initial[:controls_count], rcond=None)[0]
        free = np.arange(2, controls_count - 2)
    edges = np.roll(polygon, -1, axis=0) - polygon
    cached = {}
    pjac = np.zeros((len(parameters), controls_count + 2, 2))
    vjac = np.zeros_like(pjac)
    pjac[:, :controls_count] = b0[:, :, None] * normal
    vjac[:, :controls_count] = b1[:, :, None] * normal
    pjac[:, -2] = (1 - parameters)[:, None] * direction
    pjac[:, -1] = parameters[:, None] * direction
    vjac[:, -2] = -direction
    vjac[:, -1] = direction

    def evaluate(values):
        if time.perf_counter() > deadline:
            raise TimeoutError("The numerical search reached its time budget.")
        if "values" in cached and np.array_equal(values, cached["values"]):
            return cached["residual"], cached["jacobian"]
        y = values[:-2]
        low, high = values[-2:]
        heights, slopes = b0 @ y, b1 @ y
        points = (low + (high - low) * parameters)[:, None] * direction + heights[:, None] * normal
        velocities = (high - low) * direction + slopes[:, None] * normal
        lo, hi, il, ih, normals, valid = normal_chords(points, velocities, polygon)
        residual = heights - target
        jacobian = np.c_[b0, np.zeros((len(parameters), 2))]
        indices = np.flatnonzero(valid)
        if len(indices):
            residual[indices] = (lo[indices] + hi[indices]) / 2
            speed = np.linalg.norm(velocities, axis=1)
            tangent = velocities / speed[:, None]
            dv = vjac - tangent[:, None, :] * np.sum(tangent[:, None, :] * vjac, axis=2)[:, :, None]
            dn = np.stack((-dv[:, :, 1], dv[:, :, 0]), axis=2) / speed[:, None, None]
            jacobian[indices] = 0.
            for roots, sites in ((lo, il), (hi, ih)):
                e = edges[sites[indices]]
                denom = cross(normals[indices], e)
                jacobian[indices] += .5 * (
                    -cross(pjac[indices], e[:, None, :])
                    - roots[indices, None] * cross(dn[indices], e[:, None, :])
                ) / denom[:, None]
        endpoint_residuals, endpoint_jacobians = [], []
        for height_index, x_index in ((0, -2), (controls_count - 1, -1)):
            point = values[x_index] * direction + y[height_index] * normal
            difference = point - nearest_boundary(point, polygon)
            distance = np.linalg.norm(difference)
            gradient = difference / distance if distance > 1e-12 else np.zeros(2)
            row = np.zeros(controls_count + 2)
            row[height_index] = gradient @ normal
            row[x_index] = gradient @ direction
            endpoint_residuals.append(20 * distance)
            endpoint_jacobians.append(20 * row)
        residual = np.r_[residual, endpoint_residuals]
        jacobian = np.vstack((jacobian, endpoint_jacobians))
        cached.update(values=values.copy(), residual=residual, jacobian=jacobian)
        return residual, jacobian

    normal_min, normal_max = np.min(polygon @ normal), np.max(polygon @ normal)
    lower = np.r_[np.full(controls_count, normal_min - span * .1), minimum, maximum - .2 * span]
    upper = np.r_[np.full(controls_count, normal_max + span * .1), minimum + .2 * span, maximum]
    initial = np.clip(initial, lower + 1e-7, upper - 1e-7)
    if endpoints is not None:
        initial[-2:] = low, high

    def unpack(values):
        complete = initial.copy()
        complete[free] = values
        return complete

    fit = least_squares(lambda values: evaluate(unpack(values))[0], initial[free],
                        jac=lambda values: evaluate(unpack(values))[1][:, free],
                        bounds=(lower[free], upper[free]), max_nfev=180, ftol=1e-10, xtol=1e-10, gtol=1e-8)
    solution = unpack(fit.x)
    y = solution[:-2].copy()
    low, high = solution[-2:]
    endpoint_errors = []
    for height_index, x_value in ((0, low), (-1, high)):
        point = x_value * direction + y[height_index] * normal
        endpoint_error = np.linalg.norm(point - nearest_boundary(point, polygon))
        endpoint_errors.append(float(endpoint_error))
        if endpoint_error > 1e-5:
            raise ValueError(f"Endpoint residual {endpoint_error:.6f}; midpoint residual {np.max(np.abs(fit.fun[:-2])):.4f}.")
    spline = BSpline(knots, y.copy(), 3)
    check = np.unique(np.r_[np.linspace(.000001, .999999, 2049),
                             .5 - .5 * np.cos(np.linspace(.0001, np.pi - .0001, 1025))])
    path = (low + (high - low) * check)[:, None] * direction + spline(check)[:, None] * normal
    velocity = (high - low) * direction + spline(check, 1)[:, None] * normal
    lo, hi, _, _, normals, resolved = normal_chords(path, velocity, polygon)
    if not np.all(resolved):
        raise ValueError("A candidate leaves the interior or loses a normal-chord intersection.")
    errors = np.abs(lo + hi) / 2
    maximum = float(np.max(errors))
    if maximum > tolerance:
        raise ValueError(f"Best checked midpoint error {maximum:.4f} exceeds tolerance {tolerance:g}.")
    pieces = spline_pieces(spline, direction, normal, low, high)
    if not all(certify_containment(np.asarray(piece["controls"]), polygon) for piece in pieces):
        raise ValueError("Continuous spline containment could not be established.")
    return {
        "pieces": pieces, "maxError": maximum, "rmsError": float(np.sqrt(np.mean(errors ** 2))),
        "checks": len(check), "direction": direction.tolist(),
        "endpointError": max(endpoint_errors),
        "iterations": int(fit.nfev), "solverConverged": bool(fit.success),
    }


def solve_numerically(polygon, tolerance, seconds=18):
    covariance = np.cov(polygon.T)
    _, vectors = np.linalg.eigh(covariance)
    major = vectors[:, -1]
    axes = [major, np.array([-major[1], major[0]]),
            np.array([1., 0.]), np.array([0., 1.])]
    unique = []
    for axis in axes:
        if all(abs(np.dot(axis, previous)) < .999 for previous in unique):
            unique.append(axis)
    deadline = time.perf_counter() + seconds
    failures = []
    edges = np.roll(polygon, -1, axis=0) - polygon
    lengths = np.linalg.norm(edges, axis=1)
    long_edges = set(np.argsort(lengths)[-8:])
    median_length = float(np.median(lengths))
    midpoints = polygon + edges / 2
    inward = np.c_[-edges[:, 1], edges[:, 0]] / lengths[:, None]
    candidates = []
    diagonal = np.linalg.norm(np.ptp(polygon, axis=0))
    for i in range(len(polygon)):
        for j in range(i + 1, len(polygon)):
            axis = midpoints[j] - midpoints[i]
            distance = np.linalg.norm(axis)
            if distance < .5 * diagonal:
                continue
            alignment = min(inward[i] @ axis, -inward[j] @ axis) / distance
            caps = (i in long_edges and j in long_edges and min(lengths[i], lengths[j]) >= 4 * median_length
                    and alignment > .2)
            if caps or alignment > .94:
                candidates.append((1000 * caps + alignment * distance, i, j))
    for _, i, j in sorted(candidates, reverse=True)[:6]:
        a, b = midpoints[i], midpoints[j]
        axis = b - a
        va, vb = inward[i], inward[j]
        for count in (20, 36):
            try:
                return solve_chart(polygon, axis, tolerance, deadline, count, (a, b, va, vb))
            except TimeoutError as error:
                return {"failure": str(error)}
            except ValueError as error:
                failures.append(str(error))
    for count in (20, 36, 56):
        for axis in unique:
            try:
                return solve_chart(polygon, axis, tolerance, deadline, count)
            except TimeoutError as error:
                failures.append(str(error))
                return {"failure": " ".join(failures[-3:])}
            except ValueError as error:
                failures.append(str(error))
    return {"failure": " ".join(failures[-3:])}
