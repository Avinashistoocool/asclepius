"""Exact-first normal-chord bisection, with an explicitly numerical fallback."""

import math
import time

import numpy as np
from scipy import ndimage
from skimage.draw import polygon2mask

from boundary import cross, validate_polygon
from exact import analytic_chords, evaluate_pieces, exact_construction
from numerical import normal_chords, solve_numerically
from paired import actual_chords, export_phase, pair_values, solve_pairs
from smooth import SmoothBoundary, TAU


def analyze_smooth(polygon, model, resolution, tolerance, started):
    boundary = SmoothBoundary(polygon, model)
    solution = solve_pairs(boundary, tolerance)
    splines = solution["spline"]
    parameter = np.linspace(0, 1, 801)
    path, _, _, _, _ = pair_values(boundary, splines, parameter)
    centers, a, b, errors, _ = actual_chords(boundary, splines, np.linspace(.002, .998, 81))
    sites = np.arange(1024) * TAU / 1024
    outline = boundary.evaluate(sites)
    first, second = boundary.evaluate(sites, 1), boundary.evaluate(sites, 2)
    curvature = cross(first, second) / np.linalg.norm(first, axis=1) ** 3
    spacing = 600 / (resolution - 1)
    mask = polygon2mask((resolution, resolution), outline[:, ::-1] / spacing)
    heat = ndimage.distance_transform_edt(mask)[::3, ::3] * spacing
    certificate = solution["certificate"]
    return {
        "mode": "numerical",
        "boundaryModel": model,
        "boundary": outline.tolist(),
        "editingHandles": polygon.tolist(),
        "smoothBoundary": boundary.export(),
        "path": path.tolist(),
        "curve": {
            "type": "boundary-pair",
            "phasePieces": export_phase(splines[0]),
            "separationPieces": export_phase(splines[1]),
            "boundary": boundary.export(),
            "formula": "u(t)=c(t)-d(t); v(t)=c(t)+d(t); p(t)=[gamma(u(t))+gamma(v(t))]/2. c and d are piecewise cubics. gamma is the exported smooth boundary function.",
        },
        "chords": [{"center": p.tolist(), "a": q.tolist(), "b": r.tolist(), "error": float(error)}
                   for p, q, r, error in zip(centers, a, b, errors)],
        "curvature": curvature.tolist(),
        "heat": {"width": heat.shape[1], "height": heat.shape[0], "values": np.round(heat, 2).ravel().tolist()},
        "certificate": certificate,
        "continuation": solution["stages"],
        "stats": {
            "length": round(float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))), 3),
            "maxMidpointError": max(float(np.max(errors)), solution["maxError"]),
            "verifiedErrorBound": certificate["bound"],
            "tolerance": tolerance,
            "endpointError": 0.,
            "validationSamples": solution["checks"] + len(errors),
            "verifiedIntervals": certificate["intervals"],
            "concaveVertices": int(np.count_nonzero(curvature < -1e-6)),
            "elapsedMs": round((time.perf_counter() - started) * 1000),
        },
        "warnings": [
            "Midpoints are constructed from two boundary points; whole-curve collocation solves perpendicularity and regular speed.",
            f"Shape continuation completed {len(solution['stages'])} stages; contact parameters can reverse direction and the spatial path need not be a graph.",
            "Fourier coefficients define the actual boundary." if boundary.fourier else
            "Editing handles define a periodic C2 interpolating boundary, not the polygon joining the handles. The displayed smooth outline is the geometry solved.",
            f"Midpoint error is enclosed over {certificate['intervals']} parameter intervals, including endpoint caps.",
            "Interval bounds use floating-point roundoff margins; they are numerical enclosures, not formal exact-arithmetic proofs.",
            "This is a feasible trajectory search, not a proof of uniqueness or global width minimization.",
        ],
    }


def analyze(points, resolution=240, tolerance=.1, model=None):
    started = time.perf_counter()
    polygon = validate_polygon(points)
    if isinstance(resolution, bool) or resolution not in (180, 240, 320):
        raise ValueError("Field resolution must be 180, 240, or 320.")
    if isinstance(tolerance, bool) or not isinstance(tolerance, (float, int)) or not math.isfinite(tolerance) or not .01 <= tolerance <= 1:
        raise ValueError("Numerical midpoint tolerance must be between 0.01 and 1 canvas unit.")
    if model is None or (isinstance(model, dict) and model.get("kind") == "fourier"):
        return analyze_smooth(polygon, model, resolution, tolerance, started)
    construction = exact_construction(polygon, model)
    exact = construction is not None
    if exact:
        model = construction["model"]
    if not exact:
        construction = solve_numerically(polygon, tolerance)
        if "failure" in construction:
            raise ValueError(
                "No validated numerical trajectory was found. This is a bounded local search, not a proof "
                "that no solution exists. Try a gentler outline, the random ribbon family, or a larger "
                f"tolerance. Solver detail: {construction['failure']}"
            )
    pieces = construction["pieces"]
    parameter = np.linspace(0, 1, 801)
    path, velocities = evaluate_pieces(pieces, parameter)
    chord_parameters = np.linspace(.002, .998, 81)
    centers, chord_velocities = evaluate_pieces(pieces, chord_parameters)
    if exact and model["kind"] in ("circle", "ellipse", "crescent"):
        a, b = analytic_chords(centers, chord_velocities, model)
    else:
        lo, hi, _, _, normals, resolved = normal_chords(centers, chord_velocities, polygon)
        if not np.all(resolved):
            raise ValueError("A displayed perpendicular chord could not be resolved.")
        a, b = centers + lo[:, None] * normals, centers + hi[:, None] * normals
    errors = np.linalg.norm((a + b) / 2 - centers, axis=1)
    if not exact and np.max(errors) > tolerance:
        raise ValueError("The display's independent chord samples exceed the requested midpoint tolerance.")
    speed = np.linalg.norm(velocities, axis=1)
    if np.any(speed < 1e-7):
        raise ValueError("The trajectory has a singular tangent and cannot be accepted.")
    spacing = 600 / (resolution - 1)
    mask = polygon2mask((resolution, resolution), polygon[:, ::-1] / spacing)
    if exact and model["kind"] in ("circle", "ellipse", "crescent"):
        yy, xx = np.mgrid[:resolution, :resolution] * spacing - 300
        if model["kind"] == "circle":
            mask = xx ** 2 + yy ** 2 <= 205 ** 2
        elif model["kind"] == "ellipse":
            mask = (xx / (205 * 1.12)) ** 2 + (yy / (205 * .63)) ** 2 <= 1
        else:
            radius = np.hypot(xx, yy)
            theta = np.mod(np.arctan2(yy, xx), 2 * np.pi)
            inner = 80 + 90 * (1 - model.get("depth", .55))
            mask = (radius >= inner) & (radius <= 220) & (theta >= .23 * np.pi) & (theta <= 1.77 * np.pi)
    heat = ndimage.distance_transform_edt(mask)[::3, ::3] * spacing
    incoming, outgoing = polygon - np.roll(polygon, 1, axis=0), np.roll(polygon, -1, axis=0) - polygon
    turning = np.arctan2(cross(incoming, outgoing), np.sum(incoming * outgoing, axis=1))
    curvature = turning / ((np.linalg.norm(incoming, axis=1) + np.linalg.norm(outgoing, axis=1)) / 2)
    if exact:
        warnings = [
            construction["proof"], construction["boundaryDescription"],
            "Exact refers to the declared geometric model. Display coordinates use floating-point arithmetic.",
            "One admissible trajectory is constructed; no global width-minimization claim is made.",
        ]
    else:
        warnings = [
            "C2 piecewise-cubic spline: coefficients are solved from normal-chord midpoint equations, not a skeleton.",
            f"Midpoint error checked at {construction['checks']} off-grid and near-endpoint samples, plus the displayed chords.",
            "Numerical bisection checks are not an exact all-parameter proof. Continuous spline containment uses subdivided Bezier hulls.",
            "The local search uses direction-monotone charts and selected endpoints; failure does not establish nonexistence.",
            "No arbitrary bending weight or claim of globally minimizing the original width functional is used.",
        ]
    return {
        "mode": "exact" if exact else "numerical",
        "boundaryModel": model,
        "boundary": polygon.tolist(),
        "path": path.tolist(),
        "curve": {"type": "piecewise-rational-bezier" if exact else "piecewise-cubic-bezier",
                  "pieces": pieces,
                  "formula": "p(u) = sum[w_i * C(n,i) * u^i * (1-u)^(n-i) * P_i] / sum[w_i * C(n,i) * u^i * (1-u)^(n-i)], 0 <= u <= 1 on each piece."},
        "chords": [{"center": p.tolist(), "a": q.tolist(), "b": r.tolist(), "error": float(error)}
                   for p, q, r, error in zip(centers, a, b, errors)],
        "curvature": curvature.tolist(),
        "heat": {"width": heat.shape[1], "height": heat.shape[0], "values": np.round(heat, 2).ravel().tolist()},
        "stats": {
            "length": round(float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))), 3),
            "maxMidpointError": max(float(np.max(errors)), construction["maxError"]),
            "rmsMidpointError": construction["rmsError"],
            "tolerance": tolerance,
            "endpointError": 0. if exact else construction["endpointError"],
            "validationSamples": construction["checks"] + len(chord_parameters),
            "concaveVertices": int(np.count_nonzero(curvature < -1e-6)),
            "elapsedMs": round((time.perf_counter() - started) * 1000),
        },
        "warnings": warnings,
    }
