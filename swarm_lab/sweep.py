"""Swarm sweep planners compared on the same field.

A sweep is a scalar "arrival time" field T over the region: the formation at
time t is the front {T = t}. Drones are spread evenly along the front, so each
drone's swath (and altitude) equals its share of front length. A front with
several components means the swarm has split into sub-swarms.

Planners:
- mat:       MAT trunk with the perpendicular chord as a straight formation.
- lawnmower: straight line sweeping along the principal axis (splits allowed).
- harmonic:  level sets of a harmonic function, 0 on one end and 1 on the other.
- geodesic:  wavefront of geodesic distance from one end.
"""

import time

import numpy as np
from scipy import sparse
from scipy.interpolate import RegularGridInterpolator
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree
from skimage.draw import polygon as fill_polygon, polygon2mask
from skimage.graph import MCP_Geometric
from skimage.measure import find_contours

from boundary import intersection_parameters, validate_polygon
from complexity import describe
from medial_axis import analyze as medial_analyze, resample

METHODS = {
    "mat": "MAT trunk + perpendicular chord",
    "lawnmower": "Straight sweep line (splits)",
    "harmonic": "Harmonic level-set front",
    "geodesic": "Geodesic wavefront",
}


class Grid:
    def __init__(self, polygon, size):
        self.size, self.spacing = size, 600 / (size - 1)
        self.mask = polygon2mask((size, size), polygon[:, ::-1] / self.spacing)
        rows, cols = np.mgrid[:size, :size]
        self.xy = np.stack((cols, rows), axis=-1) * self.spacing


def end_caps(polygon, trunk):
    samples, sites, perimeter = resample(polygon, 1.)
    tree = cKDTree(samples)
    a, b = tree.query([trunk[0], trunk[-1]])[1]
    if a == b:
        far = np.linalg.norm(samples - samples[a], axis=1)
        b = int(np.argmax(far))
    return samples, sites, perimeter, int(a), int(b)


def harmonic_field(grid, polygon, caps):
    samples, sites, perimeter, a, b = caps
    sa, sb = sites[a], sites[b]
    forward = (sites - sa) % perimeter
    first = (sb - sa) % perimeter
    values = np.where(forward <= first, forward / first, ((sa - sites) % perimeter) / (perimeter - first))
    mask = grid.mask
    padded = np.pad(mask, 1)
    interior = mask & padded[:-2, 1:-1] & padded[2:, 1:-1] & padded[1:-1, :-2] & padded[1:-1, 2:]
    edge = mask & ~interior
    field = np.full(mask.shape, np.nan)
    field[edge] = values[cKDTree(samples).query(grid.xy[edge])[1]]
    index = -np.ones(mask.shape, dtype=int)
    index[interior] = np.arange(np.count_nonzero(interior))
    rows, cols = np.nonzero(interior)
    count = len(rows)
    entries_r, entries_c, entries_v = [np.arange(count)], [np.arange(count)], [np.full(count, 4.)]
    rhs = np.zeros(count)
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nr, nc = rows + dr, cols + dc
        neighbour = index[nr, nc]
        unknown = neighbour >= 0
        entries_r.append(np.arange(count)[unknown])
        entries_c.append(neighbour[unknown])
        entries_v.append(-np.ones(np.count_nonzero(unknown)))
        rhs[~unknown] += field[nr[~unknown], nc[~unknown]]
    matrix = sparse.csr_matrix((np.concatenate(entries_v), (np.concatenate(entries_r), np.concatenate(entries_c))),
                               shape=(count, count))
    field[interior] = spsolve(matrix, rhs)
    return field


def geodesic_field(grid, caps):
    samples, _, _, a, _ = caps
    costs = np.where(grid.mask, 1., np.inf)
    start = np.round(samples[a][::-1] / grid.spacing).astype(int)
    inside = np.argwhere(grid.mask)
    start = inside[np.argmin(np.sum((inside - start) ** 2, axis=1))]
    distance, _ = MCP_Geometric(costs, fully_connected=True).find_costs([tuple(start)])
    field = np.where(grid.mask, distance, np.nan)
    return field / np.nanmax(field)


def lawnmower_field(grid, polygon):
    centred = polygon - polygon.mean(axis=0)
    direction = np.linalg.svd(centred, full_matrices=False)[2][0]
    projection = grid.xy @ direction
    lo, hi = np.min(polygon @ direction), np.max(polygon @ direction)
    return np.where(grid.mask, (projection - lo) / (hi - lo), np.nan)


def mat_field(grid, polygon, path):
    path = np.asarray(path)
    tangent = np.gradient(path, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1), 1e-12)[:, None]
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    ends = []
    for point, n in zip(path, normal):
        hits = intersection_parameters(point, n, polygon)
        below, above = hits[hits < -1e-9], hits[hits > 1e-9]
        lo = np.max(below) if len(below) else 0.
        hi = np.min(above) if len(above) else 0.
        ends.append((point + lo * n, point + hi * n))
    ends = np.asarray(ends)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    times = arc / arc[-1]
    field = np.full(grid.mask.shape, np.inf)
    for i in range(len(path) - 1):
        quad = np.array([ends[i, 0], ends[i, 1], ends[i + 1, 1], ends[i + 1, 0]]) / grid.spacing
        rr, cc = fill_polygon(quad[:, 1], quad[:, 0], grid.mask.shape)
        field[rr, cc] = np.minimum(field[rr, cc], times[i])
    field[~grid.mask] = np.nan
    field[np.isinf(field)] = np.nan
    return field, ends, times


def chord_folds(ends):
    a, b = ends[:-1], ends[1:]
    d1, d2 = a[:, 1] - a[:, 0], b[:, 1] - b[:, 0]
    den = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    rel = b[:, 0] - a[:, 0]
    ok = np.abs(den) > 1e-12
    t = np.where(ok, (rel[:, 0] * d2[:, 1] - rel[:, 1] * d2[:, 0]) / np.where(ok, den, 1), -1)
    u = np.where(ok, (rel[:, 0] * d1[:, 1] - rel[:, 1] * d1[:, 0]) / np.where(ok, den, 1), -1)
    return float(np.mean(ok & (t > .02) & (t < .98) & (u > .02) & (u < .98)))


def decimate(line, limit=90):
    if len(line) <= limit:
        return line
    return line[np.linspace(0, len(line) - 1, limit).round().astype(int)]


def place_drones(components, drones):
    lengths = np.array([np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1)) for c in components])
    total = lengths.sum()
    if total <= 0:
        return [], 0., 0
    share = lengths / total * drones
    allocated = np.floor(share).astype(int)
    for i in np.argsort(share - allocated)[::-1][:drones - allocated.sum()]:
        allocated[i] += 1
    placed, swaths, starved = [], [], 0
    for line, length, count in zip(components, lengths, allocated):
        if count == 0:
            starved += 1
            continue
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]
        targets = (np.arange(count) + .5) / count * length
        xy = np.c_[np.interp(targets, arc, line[:, 0]), np.interp(targets, arc, line[:, 1])]
        swath = length / count
        swaths.append(swath)
        placed.extend([[float(x), float(y), float(swath)] for x, y in xy])
    return placed, max(swaths) if swaths else 0., starved


def summarize(grid, field, frames_fronts, drones, levels):
    covered = np.isfinite(field) & grid.mask
    coverage = np.count_nonzero(covered) / np.count_nonzero(grid.mask)
    filled = np.where(covered, field, np.nanmean(field[covered]) if np.any(covered) else 0.)
    gy, gx = np.gradient(filled, grid.spacing)
    gradient = np.hypot(gx, gy)
    sampler = RegularGridInterpolator((np.arange(grid.size), np.arange(grid.size)), gradient,
                                      bounds_error=False, fill_value=np.nan)
    frames, mission, spreads = [], 0., []
    peak_swath, max_split, starved_frames, lengths = 0., 0, 0, []
    step = levels[1] - levels[0]
    for level, components in zip(levels, frames_fronts):
        placed, swath, starved = place_drones(components, drones)
        peak_swath = max(peak_swath, swath)
        max_split = max(max_split, len(components))
        starved_frames += starved > 0
        if components:
            lengths.append(sum(float(np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1))) for c in components))
            points = np.vstack(components) / grid.spacing
            g = sampler(points[:, ::-1])
            g = g[np.isfinite(g) & (g > 1e-9)]
            if len(g) > 2:
                speed = 1 / g
                mission += np.percentile(speed, 95) * step
                spreads.append(np.percentile(speed, 95) / max(np.percentile(speed, 5), 1e-9))
        frames.append({"t": float(level), "fronts": [decimate(c).round(2).tolist() for c in components],
                       "drones": placed})
    return frames, {
        "coverage": round(100 * coverage, 2),
        "maxSubSwarms": int(max_split),
        "peakSwath": round(peak_swath, 2),
        "meanFrontLength": round(float(np.mean(lengths)), 2) if lengths else 0.,
        "missionLength": round(float(mission), 1),
        "speedSpread": round(float(np.median(spreads)), 2) if spreads else 1.,
        "starvedFrames": int(starved_frames),
    }


def level_fronts(grid, field, levels):
    fronts = []
    image = np.where(np.isfinite(field), field, -1.)
    for level in levels:
        contours = find_contours(image, level, mask=np.isfinite(field))
        lines = [c[:, ::-1] * grid.spacing for c in contours if len(c) > 2]
        fronts.append([c for c in lines if np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1)) > 4 * grid.spacing])
    return fronts


def coverage_image(field, mask, step=2):
    image = np.where(mask, np.where(np.isfinite(field), field, -1.), -2.)[::step, ::step]
    return {"width": image.shape[1], "height": image.shape[0], "values": np.round(image, 3).ravel().tolist()}


def compare(points, drones=12, size=200, prune=.05, progress=lambda f, m: None):
    started = time.perf_counter()
    if isinstance(drones, bool) or not isinstance(drones, int) or not 2 <= drones <= 64:
        raise ValueError("Drone count must be an integer from 2 to 64.")
    polygon = validate_polygon(points)
    progress(.02, "Computing medial axis for the trunk and end caps")
    medial = medial_analyze(polygon.tolist(), prune=prune, progress=lambda f, m: progress(.02 + .13 * f, "MAT: " + m))
    grid = Grid(polygon, size)
    caps = end_caps(polygon, np.asarray(medial["path"]))
    levels = np.linspace(.005, .995, 90)
    results = {}

    progress(.16, "Measuring shape complexity")
    shape = describe(polygon, medial)
    timings = {}
    clock = time.perf_counter()
    progress(.18, "MAT: rasterising perpendicular chords")
    field, ends, times = mat_field(grid, polygon, medial["path"])
    index = np.searchsorted(times, levels).clip(0, len(times) - 1)
    fronts = [[ends[i]] for i in index]
    results["mat"] = (field, fronts)
    mat_extra = {"missionLength": round(float(max(np.sum(np.linalg.norm(np.diff(ends[:, k], axis=0), axis=1))
                                                for k in (0, 1))), 1),
                 "folds": round(100 * chord_folds(ends), 1)}

    timings["mat"] = time.perf_counter() - clock
    clock = time.perf_counter()
    progress(.32, "Lawnmower: projecting onto principal axis")
    field = lawnmower_field(grid, polygon)
    results["lawnmower"] = (field, level_fronts(grid, field, levels))

    timings["lawnmower"] = time.perf_counter() - clock
    clock = time.perf_counter()
    progress(.45, f"Harmonic: solving Laplace equation on {np.count_nonzero(grid.mask)} cells")
    field = harmonic_field(grid, polygon, caps)
    progress(.65, "Harmonic: extracting level-set fronts")
    results["harmonic"] = (field, level_fronts(grid, field, levels))

    timings["harmonic"] = time.perf_counter() - clock
    clock = time.perf_counter()
    progress(.75, "Geodesic: fast marching from start cap")
    field = geodesic_field(grid, caps)
    results["geodesic"] = (field, level_fronts(grid, field, levels))
    timings["geodesic"] = time.perf_counter() - clock

    output = {}
    for k, (name, (field, fronts)) in enumerate(results.items()):
        progress(.85 + .13 * k / len(results), f"Scoring {METHODS[name]}")
        frames, stats = summarize(grid, field, fronts, drones, levels)
        stats["folds"] = 0.
        stats["computeMs"] = round(1000 * timings[name])
        if name == "mat":
            stats.update(mat_extra)
        output[name] = {"label": METHODS[name], "frames": frames, "stats": stats,
                        "coverage": coverage_image(field, grid.mask)}
    progress(1., "Done")
    samples = caps[0]
    return {
        "boundary": polygon.tolist(),
        "trunk": medial["path"],
        "start": samples[caps[3]].tolist(),
        "end": samples[caps[4]].tolist(),
        "drones": drones,
        "complexity": shape,
        "methods": output,
        "elapsedMs": round((time.perf_counter() - started) * 1000),
    }
