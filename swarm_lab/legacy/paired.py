"""Whole-curve boundary-pair collocation with shape continuation."""

import time

import numpy as np
from scipy.interpolate import BSpline, PPoly
from scipy.linalg import null_space
from scipy.optimize import brentq, least_squares

from smooth import TAU


def graded_knots(count):
    interior = .5 - .5 * np.cos(np.pi * np.linspace(0, 1, count - 2)[1:-1])
    return np.r_[np.zeros(4), interior, np.ones(4)]


def phase_space(count, parameters):
    knots = graded_knots(count)
    identity = BSpline(knots, np.eye(count), 3)
    constraints = np.vstack((identity(0., 1), identity(1., 1), identity(0., 3), identity(1., 3)))
    transform = null_space(constraints / np.linalg.norm(constraints, axis=1)[:, None])
    return knots, transform, identity(parameters) @ transform, identity(parameters, 1) @ transform / np.pi


def pair_values(boundary, spline, parameter, fraction=1.):
    parameter = np.asarray(parameter)
    phase_spline, separation = spline if isinstance(spline, tuple) else (spline, None)
    d = separation(parameter) if separation is not None else np.pi * parameter
    phase = phase_spline(parameter)
    rate = phase_spline(parameter, 1)
    drate = separation(parameter, 1) if separation is not None else np.full_like(parameter, np.pi)
    a, b = boundary.evaluate(phase - d, fraction=fraction), boundary.evaluate(phase + d, fraction=fraction)
    av, bv = boundary.evaluate(phase - d, 1, fraction), boundary.evaluate(phase + d, 1, fraction)
    velocity = .5 * (av * (rate - drate)[..., None] + bv * (rate + drate)[..., None])
    return (a + b) / 2, velocity, b - a, a, b


def fit_full_stage(boundary, previous, count, fraction, deadline):
    breaks = np.unique(graded_knots(count))
    parameters = np.unique(np.concatenate([
        a + (b - a) * np.array([.112701665, .5, .887298335])
        for a, b in zip(breaks[:-1], breaks[1:])
    ]))
    knots, zc, bc, _ = phase_space(count, parameters)
    identity = BSpline(knots, np.eye(count), 3)
    conditions = np.vstack((identity(0.), identity(1.), identity(0., 2), identity(1., 2)))
    zd = null_space(conditions / np.linalg.norm(conditions, axis=1)[:, None])
    linear = np.array([np.mean(knots[i + 1:i + 4]) for i in range(count)]) * np.pi
    bd, bct, bdt = identity(parameters) @ zd, identity(parameters, 1) @ zc, identity(parameters, 1) @ zd
    if isinstance(previous, tuple):
        xc = np.linalg.lstsq(bc, previous[0](parameters), rcond=None)[0]
        xd = np.linalg.lstsq(bd, previous[1](parameters) - np.pi * parameters, rcond=None)[0]
        _, velocity, _, _, _ = pair_values(boundary, previous, parameters, fraction)
        radius = float(np.median(np.linalg.norm(velocity, axis=1) / (np.pi * np.sin(np.pi * parameters))))
    else:
        xc = np.linalg.lstsq(bc, np.full(len(parameters), previous), rcond=None)[0]
        xd, radius = np.zeros(zd.shape[1]), boundary.radius
    nc, nd = len(xc), len(xd)
    initial = np.r_[xc, xd, np.log(max(radius, 1.))]
    cache = {}

    def evaluate(values):
        if time.perf_counter() > deadline:
            raise TimeoutError("Whole-curve continuation reached its time budget.")
        if "values" in cache and np.array_equal(values, cache["values"]):
            return cache["residual"], cache["jacobian"]
        c = bc @ values[:nc]
        d = np.pi * parameters + bd @ values[nc:nc + nd]
        ct, dt = bct @ values[:nc], np.pi + bdt @ values[nc:nc + nd]
        a, b = boundary.evaluate(c - d, fraction=fraction), boundary.evaluate(c + d, fraction=fraction)
        av, bv = boundary.evaluate(c - d, 1, fraction), boundary.evaluate(c + d, 1, fraction)
        aa, ba = boundary.evaluate(c - d, 2, fraction), boundary.evaluate(c + d, 2, fraction)
        q = b - a
        velocity = .5 * ((av + bv) * ct[:, None] + (bv - av) * dt[:, None])
        qc, qd = bv - av, bv + av
        vc = .5 * ((aa + ba) * ct[:, None] + (ba - aa) * dt[:, None])
        vd = .5 * ((ba - aa) * ct[:, None] + (ba + aa) * dt[:, None])
        vct, vdt = .5 * (av + bv), .5 * (bv - av)
        norm = np.maximum(np.linalg.norm(q, axis=1), 1e-8)
        speed = np.maximum(np.linalg.norm(velocity, axis=1), 1e-8)
        scale = np.pi * np.sin(np.pi * parameters)
        numerator = np.sum(q * velocity, axis=1)
        residual = numerator / norm / scale
        partials = []
        for qz, vz in ((qc, vc), (qd, vd)):
            partials.append((np.sum(qz * velocity + q * vz, axis=1)
                             - numerator * np.sum(q * qz, axis=1) / norm ** 2) / norm / scale)
        rct = np.sum(q * vct, axis=1) / norm / scale
        rdt = np.sum(q * vdt, axis=1) / norm / scale
        jp = np.c_[partials[0][:, None] * bc + rct[:, None] * bct,
                   partials[1][:, None] * bd + rdt[:, None] * bdt,
                   np.zeros(len(parameters))]
        radius = np.exp(np.clip(values[-1], -10, 10))
        speed_residual = speed / scale - radius
        speed_partials = [np.sum(velocity * v, axis=1) / speed / scale for v in (vc, vd, vct, vdt)]
        js = np.c_[speed_partials[0][:, None] * bc + speed_partials[2][:, None] * bct,
                   speed_partials[1][:, None] * bd + speed_partials[3][:, None] * bdt,
                   np.full(len(parameters), -radius)]
        violation = np.minimum(d - .000001, 0) + np.maximum(d - np.pi + .000001, 0)
        jv = np.zeros_like(jp)
        jv[:, nc:nc + nd] = 1000 * ((d < .000001) | (d > np.pi - .000001))[:, None] * bd
        combined = np.r_[residual, speed_residual, 1000 * violation]
        jacobian = np.vstack((jp, js, jv))
        cache.update(values=values.copy(), residual=combined, jacobian=jacobian)
        return combined, jacobian

    fit = least_squares(lambda values: evaluate(values)[0], initial,
                        jac=lambda values: evaluate(values)[1], max_nfev=100,
                        ftol=1e-9, xtol=1e-9, gtol=1e-6)
    c = BSpline(knots, zc @ fit.x[:nc], 3)
    d = BSpline(knots, linear + zd @ fit.x[nc:nc + nd], 3)
    return (c, d), float(np.max(np.abs(fit.fun[:len(parameters)]))), int(fit.nfev)


def fit_stage(boundary, phase, count, fraction, deadline):
    parameters = np.unique(np.r_[np.linspace(.0005, .9995, count * 4),
                                 .5 - .5 * np.cos(np.linspace(.012, np.pi - .012, count * 2))])
    knots, transform, b0, b1 = phase_space(count, parameters)
    if isinstance(phase, BSpline):
        coordinates = np.linalg.lstsq(b0, phase(parameters), rcond=None)[0]
    else:
        coordinates = np.linalg.lstsq(b0, np.full(len(parameters), phase), rcond=None)[0]
    d = np.pi * parameters
    cache = {}

    def evaluate(values):
        if time.perf_counter() > deadline:
            raise TimeoutError("Boundary-pair continuation reached its time budget.")
        if "values" in cache and np.array_equal(values, cache["values"]):
            return cache["residual"], cache["jacobian"]
        c, rate = b0 @ values, b1 @ values
        a, b = boundary.evaluate(c - d, fraction=fraction), boundary.evaluate(c + d, fraction=fraction)
        av, bv = boundary.evaluate(c - d, 1, fraction), boundary.evaluate(c + d, 1, fraction)
        aa, ba = boundary.evaluate(c - d, 2, fraction), boundary.evaluate(c + d, 2, fraction)
        chord = b - a
        velocity = .5 * (av * (rate - 1)[:, None] + bv * (rate + 1)[:, None])
        chord_c = bv - av
        velocity_c = .5 * (aa * (rate - 1)[:, None] + ba * (rate + 1)[:, None])
        velocity_rate = .5 * (av + bv)
        norm = np.linalg.norm(chord, axis=1)
        if np.min(norm) < 1e-9:
            raise ValueError("An interior chord collapsed during continuation.")
        denominator = norm * np.sin(d)
        numerator = np.sum(chord * velocity, axis=1)
        residual = numerator / denominator
        dc = ((np.sum(chord_c * velocity + chord * velocity_c, axis=1)
               - numerator * np.sum(chord * chord_c, axis=1) / norm ** 2) / denominator)
        dr = np.sum(chord * velocity_rate, axis=1) / denominator
        jacobian = dc[:, None] * b0 + dr[:, None] * b1
        cache.update(values=values.copy(), residual=residual, jacobian=jacobian)
        return residual, jacobian

    fit = least_squares(lambda values: evaluate(values)[0], coordinates,
                        jac=lambda values: evaluate(values)[1], max_nfev=100,
                        ftol=2e-10, xtol=2e-10, gtol=1e-7)
    spline = BSpline(knots, transform @ fit.x, 3)
    return spline, float(np.max(np.abs(fit.fun))), int(fit.nfev)


def export_phase(spline):
    polynomial = PPoly.from_spline(spline)
    pieces = []
    for i, (a, b) in enumerate(zip(polynomial.x[:-1], polynomial.x[1:])):
        if b > a:
            pieces.append({"interval": [float(a), float(b)], "coefficients": polynomial.c[:, i].tolist()})
    return pieces


def actual_chords(boundary, spline, parameters):
    points, velocity, _, _, _ = pair_values(boundary, spline, parameters)
    speeds = np.linalg.norm(velocity, axis=1)
    if np.any(speeds < 1e-8):
        raise ValueError("The midpoint trajectory has an interior stationary point.")
    tangents = velocity / speeds[:, None]
    normals = np.c_[-tangents[:, 1], tangents[:, 0]]
    negative, positive, root_parameters = [], [], []
    for point, tangent, normal in zip(points, tangents, normals):
        sites, hits = boundary.line_hits(point, tangent)
        signed = (hits - point) @ normal
        below, above = np.flatnonzero(signed < -1e-7), np.flatnonzero(signed > 1e-7)
        if not len(below) or not len(above):
            raise ValueError("A normal chord could not be resolved on both sides of the midpoint.")
        lo, hi = below[np.argmax(signed[below])], above[np.argmin(signed[above])]
        if len(signed) != 2:
            # Multiple hits are allowed, but the containing interval must have odd winding.
            if np.count_nonzero(signed < 0) % 2 != 1:
                raise ValueError("The paired midpoint lies outside the smooth boundary.")
        negative.append(hits[lo])
        positive.append(hits[hi])
        root_parameters.append([sites[lo], sites[hi]])
    negative, positive = np.asarray(negative), np.asarray(positive)
    error = np.linalg.norm((negative + positive) / 2 - points, axis=1)
    return points, negative, positive, error, np.asarray(root_parameters)


def critical_seeds(boundary):
    def stationarity(phase):
        chord = boundary.evaluate(phase + np.pi / 2) - boundary.evaluate(phase - np.pi / 2)
        derivative = boundary.evaluate(phase + np.pi / 2, 1) - boundary.evaluate(phase - np.pi / 2, 1)
        return float(chord @ derivative)

    sample = np.linspace(0, np.pi, 129)
    values = [stationarity(phase) for phase in sample]
    if max(abs(value) for value in values) < 1e-6:
        return [0., np.pi / 2]
    roots = []
    for lo, hi, fl, fh in zip(sample[:-1], sample[1:], values[:-1], values[1:]):
        if abs(fl) < 1e-6:
            roots.append(lo)
        elif fl * fh < 0:
            roots.append(brentq(stationarity, lo, hi))
    roots.sort(key=lambda phase: np.linalg.norm(boundary.evaluate(phase + np.pi / 2) -
                                               boundary.evaluate(phase - np.pi / 2)))
    for phase in (0., np.pi / 2):
        if all(abs(phase - previous) > .05 for previous in roots):
            roots.append(phase)
    return roots[:6]


def solve_pairs(boundary, tolerance, seconds=80):
    from enclosure import verify_intervals

    deadline = time.perf_counter() + seconds
    details = []
    for seed in critical_seeds(boundary):
        phase = seed
        stages = []
        try:
            seed_deadline = min(deadline, time.perf_counter() + 24)
            fraction, step, previous_residual = 0., .05, 0.
            while fraction < 1 - 1e-8:
                target = min(1., fraction + step)
                trial, residual, iterations = fit_full_stage(boundary, phase, 48, target, seed_deadline)
                if residual > max(.5, previous_residual * 5) and step > .05:
                    step /= 2
                    continue
                phase, fraction, previous_residual = trial, target, residual
                stages.append({"fraction": fraction, "residual": residual, "iterations": iterations})
                step = min(.25, step * 1.8)
            for count in (48, 72, 104, 144):
                if count != 48:
                    phase, residual, iterations = fit_full_stage(boundary, phase, count, 1., seed_deadline)
                sample = np.unique(np.r_[np.linspace(.00001, .99999, 193),
                                         .5 - .5 * np.cos(np.linspace(.001, np.pi - .001, 97))])
                try:
                    _, _, _, errors, _ = actual_chords(boundary, phase, sample)
                except ValueError as error:
                    details.append(str(error))
                    continue
                maximum = float(np.max(errors))
                if maximum <= tolerance * .6:
                    remaining = min(35., deadline - time.perf_counter())
                    if remaining <= 0:
                        raise TimeoutError
                    certificate = verify_intervals(boundary, phase, tolerance, seconds=remaining)
                    if certificate["verified"]:
                        return {"spline": phase, "maxError": maximum, "checks": len(sample),
                                "certificate": certificate, "stages": stages, "collocationResidual": residual}
                    details.append(certificate["reason"])
                details.append(f"phase {seed:.2f}, {count} controls: midpoint error {maximum:.5f}")
        except TimeoutError:
            details.append("A continuation branch reached its time budget.")
            if time.perf_counter() >= deadline:
                break
        except ValueError as error:
            details.append(str(error))
    raise ValueError("No interval-verified trajectory was found in this bounded continuation search. "
                     "This does not prove nonexistence. " + "; ".join(details[-3:]))
