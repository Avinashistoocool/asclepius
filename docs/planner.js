// Swarm sweep planners compared on the same field (browser/Node port of sweep.py compare()).
//
// A sweep is a scalar arrival-time field T over the region: the formation at time t is the front {T = t}.
// Drones are spread evenly along the front, so each drone's swath (and altitude) equals its share of front length.
import {
  validatePolygon, intersectionParameters, fillPolygon, lineLength, cumulativeArc, interpAlong, linspace,
  PointIndex, principalAxes, percentile, roundTo,
} from "./geometry.js";
import { medialAnalyze } from "./medial.js";
import { describe } from "./complexity.js";
import { nadirPlan, JUMP } from "./nadir.js";

export const METHODS = {
  mat: "MAT trunk + perpendicular chord",
  lawnmower: "Straight sweep line (splits)",
  harmonic: "Harmonic level-set front",
  geodesic: "Geodesic wavefront",
};

const now = () => (typeof performance !== "undefined" ? performance.now() : Date.now());
const pmod = (a, m) => ((a % m) + m) % m;

class Grid {
  constructor(polygon, size) {
    this.size = size;
    this.spacing = 600 / (size - 1);
    this.mask = new Uint8Array(size * size);
    const scaled = polygon.map(([x, y]) => [x / this.spacing, y / this.spacing]);
    fillPolygon(scaled, size, size, (r, c) => { this.mask[r * size + c] = 1; });
    const cells = [];
    for (let i = 0; i < size * size; i++) if (this.mask[i]) cells.push(i);
    this.cells = Int32Array.from(cells);
  }
  x(i) { return (i % this.size) * this.spacing; }
  y(i) { return Math.floor(i / this.size) * this.spacing; }
}

function endCaps(medial) {
  const { samples, sites, perimeter, sampleIndex, path } = medial;
  const a = sampleIndex.query(path[0][0], path[0][1]);
  let b = sampleIndex.query(path[path.length - 1][0], path[path.length - 1][1]);
  if (a === b) {
    let best = -1;
    samples.forEach((p, i) => { const d = Math.hypot(p[0] - samples[a][0], p[1] - samples[a][1]); if (d > best) { best = d; b = i; } });
  }
  return { samples, sites, perimeter, a, b, index: sampleIndex };
}

function harmonicField(grid, caps) {
  const { samples, sites, perimeter, a, b, index } = caps, S = grid.size, mask = grid.mask;
  const sa = sites[a], sb = sites[b], first = pmod(sb - sa, perimeter);
  const values = Array.from(sites, s => {
    const forward = pmod(s - sa, perimeter);
    return forward <= first ? forward / first : pmod(sa - s, perimeter) / (perimeter - first);
  });
  const field = new Float64Array(S * S).fill(NaN), unknown = new Int32Array(S * S).fill(-1), interior = [];
  const inMask = (r, c) => r >= 0 && c >= 0 && r < S && c < S && mask[r * S + c];
  for (const i of grid.cells) {
    const r = Math.floor(i / S), c = i % S;
    if (inMask(r + 1, c) && inMask(r - 1, c) && inMask(r, c + 1) && inMask(r, c - 1)) { unknown[i] = interior.length; interior.push(i); }
    else field[i] = values[index.query(grid.x(i), grid.y(i))];
  }
  const n = interior.length, rhs = new Float64Array(n), nbr = new Int32Array(4 * n);
  const offsets = [S, -S, 1, -1];
  for (let k = 0; k < n; k++) {
    const i = interior[k];
    for (let d = 0; d < 4; d++) {
      const j = i + offsets[d];
      nbr[4 * k + d] = unknown[j];
      if (unknown[j] < 0) rhs[k] += field[j];
    }
  }
  // Conjugate gradient on the SPD 5-point Laplacian (4 on the diagonal, -1 to unknown neighbours).
  const x = new Float64Array(n), r = Float64Array.from(rhs), p = Float64Array.from(rhs), Ap = new Float64Array(n);
  for (let k = 0; k < n; k++) x[k] = .5;
  const apply = (v, out) => {
    for (let k = 0; k < n; k++) {
      let s = 4 * v[k];
      for (let d = 0; d < 4; d++) { const j = nbr[4 * k + d]; if (j >= 0) s -= v[j]; }
      out[k] = s;
    }
  };
  apply(x, Ap);
  for (let k = 0; k < n; k++) { r[k] = rhs[k] - Ap[k]; p[k] = r[k]; }
  let rr = 0, bb = 0;
  for (let k = 0; k < n; k++) { rr += r[k] * r[k]; bb += rhs[k] * rhs[k]; }
  const tol = 1e-24 * Math.max(bb, 1);
  for (let it = 0; it < 20 * n && rr > tol; it++) {
    apply(p, Ap);
    let pAp = 0;
    for (let k = 0; k < n; k++) pAp += p[k] * Ap[k];
    const alpha = rr / pAp;
    let next = 0;
    for (let k = 0; k < n; k++) { x[k] += alpha * p[k]; r[k] -= alpha * Ap[k]; next += r[k] * r[k]; }
    const beta = next / rr;
    rr = next;
    for (let k = 0; k < n; k++) p[k] = r[k] + beta * p[k];
  }
  for (let k = 0; k < n; k++) field[interior[k]] = x[k];
  return field;
}

class MinHeap {
  constructor() { this.keys = []; this.items = []; }
  get size() { return this.keys.length; }
  push(key, item) {
    const keys = this.keys, items = this.items;
    let i = keys.length;
    keys.push(key); items.push(item);
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (keys[parent] <= key) break;
      keys[i] = keys[parent]; items[i] = items[parent]; i = parent;
    }
    keys[i] = key; items[i] = item;
  }
  pop() {
    const keys = this.keys, items = this.items, top = items[0], topKey = keys[0];
    const lastKey = keys.pop(), lastItem = items.pop(), n = keys.length;
    if (n) {
      let i = 0;
      while (true) {
        let child = 2 * i + 1;
        if (child >= n) break;
        if (child + 1 < n && keys[child + 1] < keys[child]) child++;
        if (keys[child] >= lastKey) break;
        keys[i] = keys[child]; items[i] = items[child]; i = child;
      }
      keys[i] = lastKey; items[i] = lastItem;
    }
    this.lastKey = topKey;
    return top;
  }
}

// 8-connected Dijkstra with unit cost per cell, equivalent to skimage MCP_Geometric(fully_connected=True).
function geodesicField(grid, caps) {
  const { samples, a } = caps, S = grid.size, mask = grid.mask;
  const sr = Math.round(samples[a][1] / grid.spacing), sc = Math.round(samples[a][0] / grid.spacing);
  let start = grid.cells[0], best = Infinity;
  for (const i of grid.cells) {
    const d = (Math.floor(i / S) - sr) ** 2 + (i % S - sc) ** 2;
    if (d < best) { best = d; start = i; }
  }
  const dist = new Float64Array(S * S).fill(Infinity), done = new Uint8Array(S * S), heap = new MinHeap();
  dist[start] = 0; heap.push(0, start);
  const moves = [[-1, -1, Math.SQRT2], [-1, 0, 1], [-1, 1, Math.SQRT2], [0, -1, 1], [0, 1, 1], [1, -1, Math.SQRT2], [1, 0, 1], [1, 1, Math.SQRT2]];
  while (heap.size) {
    const i = heap.pop();
    if (done[i]) continue;
    done[i] = 1;
    const r = Math.floor(i / S), c = i % S, di = dist[i];
    for (const [dr, dc, w] of moves) {
      const nr = r + dr, nc = c + dc;
      if (nr < 0 || nc < 0 || nr >= S || nc >= S) continue;
      const j = nr * S + nc;
      if (!mask[j] || done[j]) continue;
      const nd = di + w;
      if (nd < dist[j]) { dist[j] = nd; heap.push(nd, j); }
    }
  }
  let max = 0;
  for (const i of grid.cells) if (Number.isFinite(dist[i]) && dist[i] > max) max = dist[i];
  const field = new Float64Array(S * S).fill(NaN);
  for (const i of grid.cells) field[i] = Number.isFinite(dist[i]) ? dist[i] / max : NaN;
  return field;
}

function lawnmowerField(grid, polygon) {
  const [dx, dy] = principalAxes(polygon).direction;
  let lo = Infinity, hi = -Infinity;
  for (const [x, y] of polygon) { const p = x * dx + y * dy; if (p < lo) lo = p; if (p > hi) hi = p; }
  const field = new Float64Array(grid.size * grid.size).fill(NaN);
  for (const i of grid.cells) field[i] = (grid.x(i) * dx + grid.y(i) * dy - lo) / (hi - lo);
  return field;
}

function matField(grid, polygon, path) {
  const n = path.length, S = grid.size, ends = [];
  for (let i = 0; i < n; i++) {
    const prev = path[Math.max(i - 1, 0)], next = path[Math.min(i + 1, n - 1)], div = i === 0 || i === n - 1 ? 1 : 2;
    let tx = (next[0] - prev[0]) / div, ty = (next[1] - prev[1]) / div;
    const norm = Math.max(Math.hypot(tx, ty), 1e-12);
    tx /= norm; ty /= norm;
    const nx = -ty, ny = tx, p = path[i];
    let lo = -Infinity, hi = Infinity;
    for (const t of intersectionParameters(p[0], p[1], nx, ny, polygon)) {
      if (t < -1e-9 && t > lo) lo = t;
      if (t > 1e-9 && t < hi) hi = t;
    }
    if (!Number.isFinite(lo)) lo = 0;
    if (!Number.isFinite(hi)) hi = 0;
    ends.push([[p[0] + lo * nx, p[1] + lo * ny], [p[0] + hi * nx, p[1] + hi * ny]]);
  }
  const arc = cumulativeArc(path), times = Array.from(arc, v => v / arc[n - 1]);
  const field = new Float64Array(S * S).fill(Infinity), h = grid.spacing;
  for (let i = 0; i + 1 < n; i++) {
    const quad = [ends[i][0], ends[i][1], ends[i + 1][1], ends[i + 1][0]].map(([x, y]) => [x / h, y / h]);
    const t = times[i];
    fillPolygon(quad, S, S, (r, c) => { const j = r * S + c; if (t < field[j]) field[j] = t; });
  }
  for (let j = 0; j < S * S; j++) if (!grid.mask[j] || !Number.isFinite(field[j])) field[j] = NaN;
  return { field, ends, times };
}

export function chordFolds(ends) {
  let folds = 0;
  for (let i = 0; i + 1 < ends.length; i++) {
    const [a0, a1] = ends[i], [b0, b1] = ends[i + 1];
    const d1x = a1[0] - a0[0], d1y = a1[1] - a0[1], d2x = b1[0] - b0[0], d2y = b1[1] - b0[1];
    const den = d1x * d2y - d1y * d2x;
    if (Math.abs(den) <= 1e-12) continue;
    const rx = b0[0] - a0[0], ry = b0[1] - a0[1];
    const t = (rx * d2y - ry * d2x) / den, u = (rx * d1y - ry * d1x) / den;
    if (t > .02 && t < .98 && u > .02 && u < .98) folds++;
  }
  return ends.length > 1 ? folds / (ends.length - 1) : 0;
}

// Marching squares (skimage.measure.find_contours semantics: value > level is "above", squares with a
// masked corner are skipped, ambiguous saddles are disconnected on the high side).
function findContours(grid, field, level) {
  const S = grid.size, h = grid.spacing, segments = [], points = new Map();
  const frac = (from, to) => (to === from ? 0 : (level - from) / (to - from));
  for (let r = 0; r + 1 < S; r++) {
    for (let c = 0; c + 1 < S; c++) {
      const i = r * S + c, ul = field[i], ur = field[i + 1], ll = field[i + S], lr = field[i + S + 1];
      if (!(Number.isFinite(ul) && Number.isFinite(ur) && Number.isFinite(ll) && Number.isFinite(lr))) continue;
      const kase = (ul > level ? 1 : 0) | (ur > level ? 2 : 0) | (ll > level ? 4 : 0) | (lr > level ? 8 : 0);
      if (kase === 0 || kase === 15) continue;
      const top = 2 * i, bottom = 2 * (i + S), left = 2 * i + 1, right = 2 * (i + 1) + 1;
      const point = key => {
        if (points.has(key)) return;
        if (key === top) points.set(key, [(c + frac(ul, ur)) * h, r * h]);
        else if (key === bottom) points.set(key, [(c + frac(ll, lr)) * h, (r + 1) * h]);
        else if (key === left) points.set(key, [c * h, (r + frac(ul, ll)) * h]);
        else points.set(key, [(c + 1) * h, (r + frac(ur, lr)) * h]);
      };
      const add = (p, q) => { point(p); point(q); segments.push(p, q); };
      switch (kase) {
        case 1: case 14: add(top, left); break;
        case 2: case 13: add(right, top); break;
        case 3: case 12: add(right, left); break;
        case 4: case 11: add(left, bottom); break;
        case 5: case 10: add(top, bottom); break;
        case 6: add(right, top); add(left, bottom); break;
        case 7: case 8: add(right, bottom); break;
        case 9: add(top, left); add(bottom, right); break;
      }
    }
  }
  const links = new Map(), count = segments.length / 2;
  for (let s = 0; s < count; s++) for (const key of [segments[2 * s], segments[2 * s + 1]]) {
    if (!links.has(key)) links.set(key, []);
    links.get(key).push(s);
  }
  const used = new Uint8Array(count), lines = [];
  const trace = (s, fromKey) => {
    const keys = [fromKey];
    let key = fromKey;
    while (s >= 0 && !used[s]) {
      used[s] = 1;
      key = segments[2 * s] === key ? segments[2 * s + 1] : segments[2 * s];
      keys.push(key);
      const next = links.get(key).find(t => !used[t]);
      s = next === undefined ? -1 : next;
    }
    return keys.map(k => points.get(k));
  };
  for (const [key, list] of links) if (list.length === 1 && !used[list[0]]) lines.push(trace(list[0], key));
  for (let s = 0; s < count; s++) if (!used[s]) lines.push(trace(s, segments[2 * s]));
  return lines;
}

function levelFronts(grid, field, levels) {
  return Array.from(levels, level => findContours(grid, field, level)
    .filter(line => line.length > 2 && lineLength(line) > 4 * grid.spacing));
}

function decimate(line, limit = 90) {
  if (line.length <= limit) return line;
  return Array.from(linspace(0, line.length - 1, limit), k => line[Math.round(k)]);
}

function placeDrones(components, drones) {
  const lengths = components.map(lineLength), total = lengths.reduce((a, b) => a + b, 0);
  if (total <= 0) return { placed: [], swath: 0, starved: 0 };
  const share = lengths.map(l => l / total * drones), allocated = share.map(Math.floor);
  const remaining = drones - allocated.reduce((a, b) => a + b, 0);
  const order = share.map((s, i) => [s - allocated[i], i]).sort((p, q) => q[0] - p[0] || q[1] - p[1]);
  for (let k = 0; k < remaining && k < order.length; k++) allocated[order[k][1]]++;
  const placed = [];
  let swath = 0, starved = 0;
  components.forEach((line, k) => {
    const count = allocated[k];
    if (!count) { starved++; return; }
    const arc = cumulativeArc(line), targets = Array.from({ length: count }, (_, j) => (j + .5) / count * lengths[k]);
    const s = lengths[k] / count;
    swath = Math.max(swath, s);
    for (const [x, y] of interpAlong(line, arc, targets)) placed.push([x, y, s]);
  });
  return { placed, swath, starved };
}

function densify(line, spacing = 2) {
  const arc = cumulativeArc(line), total = arc[arc.length - 1];
  return interpAlong(line, arc, linspace(0, total, Math.max(Math.trunc(total / spacing), 2)));
}

const mean = (values) => values.reduce((a, b) => a + b, 0) / values.length;
const median = (values) => percentile(values, 50);

// Order and orient front pieces so the lane coordinate s in [0, 1] varies continuously in time.
function orientFronts(framesFronts) {
  const oriented = [];
  let reference = null;
  for (const components of framesFronts) {
    let pieces = components.map(c => densify(c));
    if (!pieces.length) { oriented.push({ pieces: [], ref: null }); continue; }
    if (!reference) {
      pieces = pieces.map(p => (p[0][0] + p[0][1] <= p[p.length - 1][0] + p[p.length - 1][1] ? p : p.slice().reverse()));
      const key = p => { const m = [mean(p.map(q => q[0])), mean(p.map(q => q[1]))]; return m; };
      pieces = pieces.map(p => [key(p), p]).sort((u, v) => u[0][0] - v[0][0] || u[0][1] - v[0][1]).map(u => u[1]);
    } else {
      const keyed = pieces.map(p => {
        let s = p.map(q => reference.lane[reference.index.query(q[0], q[1])]);
        const quarter = Math.max(Math.floor(p.length / 4), 1);
        if (mean(s.slice(0, quarter)) > mean(s.slice(-quarter))) { p = p.slice().reverse(); s = s.slice().reverse(); }
        return [median(s), p];
      });
      keyed.sort((u, v) => u[0] - v[0]);
      pieces = keyed.map(u => u[1]);
    }
    const arcs = pieces.map(cumulativeArc), total = arcs.reduce((a, arc) => a + arc[arc.length - 1], 0);
    const points = [], lane = [];
    let offset = 0;
    pieces.forEach((p, k) => {
      const arc = arcs[k];
      p.forEach((q, j) => { points.push(q); lane.push((offset + arc[j]) / Math.max(total, 1e-12)); });
      offset += arc[arc.length - 1];
    });
    reference = { index: new PointIndex(points), points, lane: Float64Array.from(lane), total };
    oriented.push({ pieces, ref: reference });
  }
  return oriented;
}

// Assign every swept cell to the drone whose lane interval contains its lane coordinate.
function droneLanes(grid, field, levels, oriented, drones) {
  const S = grid.size, lanes = new Int32Array(S * S).fill(-2), L = levels.length, step = levels[1] - levels[0];
  for (const i of grid.cells) lanes[i] = -1;
  const valid = [];
  oriented.forEach((o, k) => { if (o.ref) valid.push(k); });
  if (!valid.length) return lanes;
  // Snap frames without a front (e.g. the very ends) to the closest frame that has one.
  const snap = Array.from({ length: L }, (_, k) => {
    let best = valid[0];
    for (const v of valid) if (Math.abs(v - k) < Math.abs(best - k)) best = v;
    return best;
  });
  for (const i of grid.cells) {
    const v = field[i];
    if (!Number.isFinite(v)) continue;
    const k = snap[Math.min(Math.max(Math.round((v - levels[0]) / step), 0), L - 1)], ref = oriented[k].ref;
    const s = ref.lane[ref.index.query(grid.x(i), grid.y(i))];
    lanes[i] = Math.min(Math.trunc(s * drones), drones - 1);
  }
  return lanes;
}

// Per frame, each drone sits at the centre of its lane interval on the front (null if no front).
function dronePositions(oriented, drones) {
  return oriented.map(({ ref }) => {
    if (!ref) return null;
    const { lane, points, total } = ref, n = lane.length, frame = [];
    for (let i = 0; i < drones; i++) {
      const target = (i + .5) / drones;
      let lo = 0, hi = n;
      while (lo < hi) { const mid = (lo + hi) >> 1; if (lane[mid] < target) lo = mid + 1; else hi = mid; }
      const j = Math.min(Math.max(lo, 1), n - 1);
      const u = Math.min(Math.max((target - lane[j - 1]) / Math.max(lane[j] - lane[j - 1], 1e-12), 0), 1);
      const a = points[j - 1], b = points[j];
      frame.push([a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), total / drones]);
    }
    return frame;
  });
}

// np.gradient of the field (filled outside the swept region) sampled bilinearly like RegularGridInterpolator.
function gradientSampler(grid, field) {
  const S = grid.size, h = grid.spacing, filled = new Float64Array(S * S);
  let sum = 0, count = 0;
  for (const i of grid.cells) if (Number.isFinite(field[i])) { sum += field[i]; count++; }
  const fill = count ? sum / count : 0;
  for (let i = 0; i < S * S; i++) filled[i] = grid.mask[i] && Number.isFinite(field[i]) ? field[i] : fill;
  const g = new Float64Array(S * S);
  for (let r = 0; r < S; r++) for (let c = 0; c < S; c++) {
    const i = r * S + c;
    const gy = r === 0 ? (filled[i + S] - filled[i]) / h : r === S - 1 ? (filled[i] - filled[i - S]) / h : (filled[i + S] - filled[i - S]) / (2 * h);
    const gx = c === 0 ? (filled[i + 1] - filled[i]) / h : c === S - 1 ? (filled[i] - filled[i - 1]) / h : (filled[i + 1] - filled[i - 1]) / (2 * h);
    g[i] = Math.hypot(gx, gy);
  }
  return (x, y) => {
    const col = x / h, row = y / h;
    if (!(row >= 0 && col >= 0 && row <= S - 1 && col <= S - 1)) return NaN;
    const r0 = Math.min(Math.floor(row), S - 2), c0 = Math.min(Math.floor(col), S - 2), fr = row - r0, fc = col - c0, i = r0 * S + c0;
    return (1 - fr) * ((1 - fc) * g[i] + fc * g[i + 1]) + fr * ((1 - fc) * g[i + S] + fc * g[i + S + 1]);
  };
}

const roundPoint = p => [roundTo(p[0], 2), roundTo(p[1], 2)];

function summarize(grid, field, framesFronts, drones, levels, camera) {
  let covered = 0;
  for (const i of grid.cells) if (Number.isFinite(field[i])) covered++;
  const coverage = covered / grid.cells.length;
  const sampler = gradientSampler(grid, field);
  const oriented = orientFronts(framesFronts);
  const lanes = droneLanes(grid, field, levels, oriented, drones);
  const positions = dronePositions(oriented, drones);
  const paths = Array.from({ length: drones }, (_, i) => positions.filter(f => f).map(f => [f[i][0], f[i][1]]));
  const cell = grid.spacing ** 2;
  const { plans, hits, stats: cameraStats } = nadirPlan(grid, lanes, paths, camera);
  const laneArea = new Float64Array(drones);
  for (const i of grid.cells) if (lanes[i] >= 0) laneArea[lanes[i]]++;
  const perDrone = paths.map((path, i) => {
    let length = 0, transits = 0;
    for (let k = 1; k < path.length; k++) {
      const s = Math.hypot(path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1]);
      if (s > JUMP) transits++; else length += s;
    }
    const altitude = plans[i].altitude;
    return {
      path: path.map(roundPoint),
      area: roundTo(laneArea[i] * cell, 1),
      length: roundTo(length, 1),
      transits,
      heading: plans[i].heading.map(([hx, hy]) => roundTo(Math.atan2(hy, hx) * 180 / Math.PI, 1)),
      altitude: Array.from(altitude, a => roundTo(a, 2)),
      maxAltitude: path.length ? roundTo(Math.max(...altitude), 1) : 0,
    };
  });
  const frameOf = [];
  let seen = -1;
  for (const f of positions) { if (f) seen++; frameOf.push(seen); }
  const areas = perDrone.map(d => d.area);
  const frames = [], spreads = [], frontLengths = [], step = levels[1] - levels[0];
  let mission = 0, peakSwath = 0, maxSplit = 0, starvedFrames = 0;
  levels.forEach((level, k) => {
    const components = framesFronts[k];
    let { placed, swath, starved } = placeDrones(components, drones);
    placed = placed.map(([x, y, s]) => [roundTo(x, 2), roundTo(y, 2), roundTo(s, 2)]);
    if (positions[k]) {
      const j = frameOf[k];
      placed = positions[k].map(([x, y, s], i) => [roundTo(x, 2), roundTo(y, 2), roundTo(s, 2), perDrone[i].heading[j], perDrone[i].altitude[j]]);
    }
    peakSwath = Math.max(peakSwath, swath);
    maxSplit = Math.max(maxSplit, components.length);
    if (starved > 0) starvedFrames++;
    if (components.length) {
      frontLengths.push(components.reduce((a, c) => a + lineLength(c), 0));
      const speeds = [];
      for (const c of components) for (const [x, y] of c) { const g = sampler(x, y); if (Number.isFinite(g) && g > 1e-9) speeds.push(1 / g); }
      if (speeds.length > 2) {
        const fast = percentile(speeds, 95);
        mission += fast * step;
        spreads.push(fast / Math.max(percentile(speeds, 5), 1e-9));
      }
    }
    frames.push({ t: level, fronts: components.map(c => decimate(c).map(roundPoint)), drones: placed });
  });
  const stats = {
    coverage: roundTo(100 * coverage, 2),
    maxSubSwarms: maxSplit,
    peakSwath: roundTo(peakSwath, 2),
    meanFrontLength: frontLengths.length ? roundTo(mean(frontLengths), 2) : 0,
    missionLength: roundTo(mission, 1),
    speedSpread: spreads.length ? roundTo(median(spreads), 2) : 1,
    starvedFrames,
    areaBalance: roundTo(Math.min(...areas) / Math.max(Math.max(...areas), 1e-9), 2),
    longestDronePath: roundTo(Math.max(...perDrone.map(d => d.length)), 1),
    transits: perDrone.reduce((a, d) => a + d.transits, 0),
    ...cameraStats,
  };
  return { frames, stats, perDrone, lanes, hits };
}

function subsample(grid, value, step = 2) {
  const S = grid.size, width = Math.ceil(S / step), values = [];
  for (let r = 0; r < S; r += step) for (let c = 0; c < S; c += step) values.push(value(r * S + c));
  return { width, height: width, values };
}

function checkNumber(name, value, low, high) {
  if (typeof value !== "number" || !Number.isFinite(value) || value < low || value > high)
    throw new Error(`${name} must be between ${low} and ${high}.`);
}

export function compare(points, opts = {}, progress = () => {}) {
  const started = now();
  const { drones = 12, size = 200, prune = .05, hfov = 70, vfov = 55, overlap = .1 } = opts;
  if (!Number.isInteger(drones) || drones < 2 || drones > 64) throw new Error("Drone count must be an integer from 2 to 64.");
  checkNumber("Horizontal FOV", hfov, 10, 150);
  checkNumber("Vertical FOV", vfov, 10, 150);
  checkNumber("Overlap", overlap, 0, 1);
  const camera = { hfov, vfov, overlap };
  const polygon = validatePolygon(points);
  progress(.02, "Computing medial axis for the trunk and end caps");
  const medial = medialAnalyze(polygon, { prune, progress: (f, m) => progress(.02 + .13 * f, "MAT: " + m) });
  const grid = new Grid(polygon, size);
  const caps = endCaps(medial);
  const levels = Array.from(linspace(.005, .995, 90));
  const results = {}, timings = {};

  progress(.16, "Measuring shape complexity");
  const shape = describe(polygon, medial);
  let clock = now();
  progress(.18, "MAT: rasterising perpendicular chords");
  const mat = matField(grid, polygon, medial.path);
  results.mat = {
    field: mat.field,
    fronts: levels.map(level => {
      let lo = 0, hi = mat.times.length;
      while (lo < hi) { const mid = (lo + hi) >> 1; if (mat.times[mid] < level) lo = mid + 1; else hi = mid; }
      return [mat.ends[Math.min(lo, mat.times.length - 1)]];
    }),
  };
  const matExtra = {
    missionLength: roundTo(Math.max(lineLength(mat.ends.map(e => e[0])), lineLength(mat.ends.map(e => e[1]))), 1),
    folds: roundTo(100 * chordFolds(mat.ends), 1),
  };
  timings.mat = now() - clock;

  clock = now();
  progress(.32, "Lawnmower: projecting onto principal axis");
  let field = lawnmowerField(grid, polygon);
  results.lawnmower = { field, fronts: levelFronts(grid, field, levels) };
  timings.lawnmower = now() - clock;

  clock = now();
  progress(.45, `Harmonic: solving Laplace equation on ${grid.cells.length} cells`);
  field = harmonicField(grid, caps);
  progress(.65, "Harmonic: extracting level-set fronts");
  results.harmonic = { field, fronts: levelFronts(grid, field, levels) };
  timings.harmonic = now() - clock;

  clock = now();
  progress(.75, "Geodesic: fast marching from start cap");
  field = geodesicField(grid, caps);
  results.geodesic = { field, fronts: levelFronts(grid, field, levels) };
  timings.geodesic = now() - clock;

  const output = {}, names = Object.keys(results);
  names.forEach((name, k) => {
    progress(.82 + .16 * k / names.length, `Nadir camera plan + coverage check: ${METHODS[name]}`);
    const { field, fronts } = results[name];
    const { frames, stats, perDrone, lanes, hits } = summarize(grid, field, fronts, drones, levels, camera);
    stats.folds = 0;
    stats.computeMs = Math.round(timings[name]);
    if (name === "mat") Object.assign(stats, matExtra);
    output[name] = {
      label: METHODS[name], frames, stats, drones: perDrone,
      coverage: subsample(grid, i => (grid.mask[i] ? (Number.isFinite(field[i]) ? roundTo(field[i], 3) : -1) : -2)),
      lanes: subsample(grid, i => lanes[i]),
      imaged: subsample(grid, i => (grid.mask[i] ? hits[i] : -2)),
    };
  });
  progress(1, "Done");
  return {
    boundary: polygon,
    trunk: medial.path,
    start: caps.samples[caps.a].slice(),
    end: caps.samples[caps.b].slice(),
    drones,
    complexity: shape,
    camera,
    methods: output,
    elapsedMs: Math.round(now() - started),
  };
}
