// Medial axis trunk extraction (port of medial_axis.py analyze(), only the parts compare() needs).
import Delaunator from "./delaunator.js";
import { pointInPolygon, PointIndex, resample, cumulativeArc, interpAlong, linspace, firstHit, polygonArea, principalAxes, lineLength } from "./geometry.js";

class UnionFind {
  constructor(n) { this.parent = Int32Array.from({ length: n }, (_, i) => i); }
  find(i) { const p = this.parent; while (p[i] !== i) { p[i] = p[p[i]]; i = p[i]; } return i; }
  union(a, b) { a = this.find(a); b = this.find(b); if (a === b) return false; this.parent[a] = b; return true; }
}

function treeDistances(adjacency, start) {
  const dist = new Map([[start, 0]]), parent = new Map([[start, -1]]), stack = [start];
  while (stack.length) {
    const u = stack.pop();
    for (const [v, w] of adjacency.get(u)) if (!dist.has(v)) { dist.set(v, dist.get(u) + w); parent.set(v, u); stack.push(v); }
  }
  return { dist, parent };
}

function farthest(nodes, dist) {
  let best = -1, bestNode = nodes[0];
  for (const n of nodes) { const d = dist.has(n) ? dist.get(n) : -1; if (d > best) { best = d; bestNode = n; } }
  return bestNode;
}

function longestPath(adjacency, nodes) {
  const start = farthest(nodes, treeDistances(adjacency, nodes[0]).dist);
  const { dist, parent } = treeDistances(adjacency, start);
  const path = [farthest(nodes, dist)];
  while (path[path.length - 1] !== start) path.push(parent.get(path[path.length - 1]));
  return path.reverse();
}

function gaussianSmooth(points, sigma) {
  const n = points.length, radius = Math.ceil(3 * sigma), kernel = [];
  for (let k = -radius; k <= radius; k++) kernel.push(Math.exp(-k * k / (2 * sigma * sigma)));
  const total = kernel.reduce((a, b) => a + b, 0);
  // Odd reflection about each endpoint keeps the endpoints and the end tangents.
  const at = (i, d) => i < 0 ? 2 * points[0][d] - points[Math.min(-i, n - 1)][d]
    : i >= n ? 2 * points[n - 1][d] - points[Math.max(2 * (n - 1) - i, 0)][d] : points[i][d];
  return points.map((_, i) => {
    let x = 0, y = 0;
    for (let k = -radius; k <= radius; k++) { const w = kernel[k + radius]; x += w * at(i + k, 0); y += w * at(i + k, 1); }
    return [x / total, y / total];
  });
}

function gradient(path) {
  const n = path.length;
  return path.map((p, i) => {
    if (n < 2) return [0, 0];
    if (i === 0) return [path[1][0] - p[0], path[1][1] - p[1]];
    if (i === n - 1) return [p[0] - path[i - 1][0], p[1] - path[i - 1][1]];
    return [(path[i + 1][0] - path[i - 1][0]) / 2, (path[i + 1][1] - path[i - 1][1]) / 2];
  });
}

// Replacement for the scipy smoothing spline: uniform arc-length resampling to 801 points followed by
// Gaussian smoothing, using the strongest smoothing that keeps the trunk inside the polygon.
function smoothPath(points, polygon) {
  points = points.filter((p, i) => i === 0 || Math.hypot(p[0] - points[i - 1][0], p[1] - points[i - 1][1]) > 1e-6);
  if (points.length < 4) return points.map(p => p.slice());
  const arc = cumulativeArc(points), total = arc[arc.length - 1];
  const base = interpAlong(points, arc, linspace(0, total, 801));
  const step = total / 800;
  for (const scale of [5, 1.5]) {
    const sigma = scale / step;
    if (sigma < .3) continue;
    const path = gaussianSmooth(base, Math.min(sigma, 200));
    if (path.every(p => pointInPolygon(p[0], p[1], polygon))) return path;
  }
  return base;
}

function endTangent(path, end) {
  const n = path.length, sign = end === 0 ? -1 : 1, from = end === 0 ? 0 : n - 1;
  let k = from, length = 0;
  while (length < 3 && k - sign >= 0 && k - sign < n) {
    length += Math.hypot(path[k][0] - path[k - sign][0], path[k][1] - path[k - sign][1]);
    k -= sign;
  }
  let dx = path[from][0] - path[k][0], dy = path[from][1] - path[k][1];
  const norm = Math.max(Math.hypot(dx, dy), 1e-12);
  return [dx / norm, dy / norm];
}

export function medialAnalyze(polygon, { prune = .05, extend = true, progress = () => {} } = {}) {
  progress(.08, "Resampling boundary");
  const { samples, sites, perimeter } = resample(polygon, 1);
  progress(.15, `Voronoi diagram of ${samples.length} boundary samples`);
  const coords = new Float64Array(2 * samples.length);
  samples.forEach((p, i) => { coords[2 * i] = p[0]; coords[2 * i + 1] = p[1]; });
  const del = new Delaunator(coords), triangles = del.triangles, halfedges = del.halfedges;
  const triangleCount = triangles.length / 3, vertices = new Array(triangleCount);
  for (let t = 0; t < triangleCount; t++) {
    const a = triangles[3 * t], b = triangles[3 * t + 1], c = triangles[3 * t + 2];
    const ax = coords[2 * a], ay = coords[2 * a + 1], bx = coords[2 * b] - ax, by = coords[2 * b + 1] - ay;
    const cx = coords[2 * c] - ax, cy = coords[2 * c + 1] - ay, d = 2 * (bx * cy - by * cx);
    const b2 = bx * bx + by * by, c2 = cx * cx + cy * cy;
    vertices[t] = Math.abs(d) < 1e-300 ? [Infinity, Infinity] : [ax + (cy * b2 - by * c2) / d, ay + (bx * c2 - cx * b2) / d];
  }

  progress(.40, "Selecting interior Voronoi vertices");
  const inside = vertices.map(([x, y]) => x > 0 && x < 600 && y > 0 && y < 600 && pointInPolygon(x, y, polygon));

  progress(.50, "Pruning medial branches");
  const minArc = Math.max(prune * perimeter, 3), edges = [];
  for (let e = 0; e < halfedges.length; e++) {
    const o = halfedges[e];
    if (o < e) continue; // hull edges (o === -1) are infinite ridges; each interior pair visited once
    const t1 = Math.floor(e / 3), t2 = Math.floor(o / 3);
    if (!inside[t1] || !inside[t2]) continue;
    const g1 = triangles[e], g2 = triangles[e % 3 === 2 ? e - 2 : e + 1];
    let arc = Math.abs(sites[g1] - sites[g2]);
    arc = Math.min(arc, perimeter - arc);
    if (arc > minArc) edges.push([t1, t2, Math.hypot(vertices[t1][0] - vertices[t2][0], vertices[t1][1] - vertices[t2][1]) + 1e-9]);
  }
  const index = new PointIndex(samples);

  progress(.62, "Building medial graph");
  let trunk = [], branches = 0;
  if (edges.length) {
    const components = new UnionFind(triangleCount);
    for (const [a, b] of edges) components.union(a, b);
    const totals = new Map();
    for (const [a, , w] of edges) { const r = components.find(a); totals.set(r, (totals.get(r) || 0) + w); }
    let main = -1, best = -1;
    for (const [r, w] of totals) if (w > best) { best = w; main = r; }
    const forest = new UnionFind(triangleCount), adjacency = new Map();
    for (const [a, b, w] of edges.slice().sort((p, q) => p[2] - q[2])) {
      if (components.find(a) !== main || !forest.union(a, b)) continue;
      if (!adjacency.has(a)) adjacency.set(a, []);
      if (!adjacency.has(b)) adjacency.set(b, []);
      adjacency.get(a).push([b, w]); adjacency.get(b).push([a, w]);
      branches++;
    }
    progress(.72, "Extracting longest medial path");
    const nodes = [...adjacency.keys()].sort((a, b) => a - b);
    trunk = longestPath(adjacency, nodes).map(t => vertices[t].slice());
  }

  let disk = false;
  const trunkLength = trunk.length > 1 ? lineLength(trunk) : 0, area = Math.abs(polygonArea(polygon));
  if (trunkLength < .02 * Math.sqrt(area)) {
    // Disk-like region: the medial axis collapses to (nearly) a point.
    let centers = vertices.filter((_, i) => inside[i]);
    if (!centers.length) { let mx = 0, my = 0; for (const p of polygon) { mx += p[0]; my += p[1]; } centers = [[mx / polygon.length, my / polygon.length]]; }
    let center = centers[0], radius = -1;
    for (const c of centers) { index.query(c[0], c[1]); if (index.distance > radius) { radius = index.distance; center = c; } }
    const [dx, dy] = principalAxes(samples).direction;
    trunk = [[center[0] - dx, center[1] - dy], [center[0] + dx, center[1] + dy]];
    disk = true;
  }

  progress(.80, "Smoothing trunk");
  let path = smoothPath(trunk, polygon);
  if (extend && path.length > 1) {
    progress(.84, "Extending trunk ends to the boundary");
    const pieces = [];
    for (const end of [0, -1]) {
      const p = end === 0 ? path[0] : path[path.length - 1], [tx, ty] = endTangent(path, end);
      const reach = firstHit(p[0], p[1], tx, ty, polygon), steps = Math.max(Math.trunc(reach / 2), 1), piece = [];
      const ts = linspace(0, reach * (1 - 1e-7), steps + 1);
      for (let k = 1; k < ts.length; k++) piece.push([p[0] + ts[k] * tx, p[1] + ts[k] * ty]);
      pieces.push(piece);
    }
    path = pieces[0].reverse().concat(path, pieces[1]);
  }
  progress(.88, "Measuring inscribed disks");
  const radius = path.map(p => { index.query(p[0], p[1]); return index.distance; });
  progress(1, "Done");
  return { path, radius, branches, disk, samples, sites, perimeter, sampleIndex: index, velocity: gradient(path) };
}
