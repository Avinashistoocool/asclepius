// Polygon predicates, rasterisation, resampling and nearest-neighbour lookup (port of boundary.py helpers).

export const cross = (ax, ay, bx, by) => ax * by - ay * bx;

export function polygonArea(poly) {
  let s = 0;
  for (let i = 0, n = poly.length; i < n; i++) {
    const p = poly[i], q = poly[(i + 1) % n];
    s += p[0] * q[1] - p[1] * q[0];
  }
  return s / 2;
}

export function validatePolygon(points) {
  if (!Array.isArray(points)) throw new Error("Boundary coordinates must be numeric pairs.");
  const poly = points.map(p => {
    if (!p || p.length !== 2) throw new Error("Use between 3 and 384 boundary vertices.");
    const x = Number(p[0]), y = Number(p[1]);
    if (typeof p[0] === "boolean" || typeof p[1] === "boolean" || Number.isNaN(x) || Number.isNaN(y))
      throw new Error("Boundary coordinates must be numeric pairs.");
    return [x, y];
  });
  const n = poly.length;
  if (n < 3 || n > 384) throw new Error("Use between 3 and 384 boundary vertices.");
  if (poly.some(([x, y]) => !Number.isFinite(x) || !Number.isFinite(y) || x < 5 || y < 5 || x > 595 || y > 595))
    throw new Error("Keep all vertices within the drawing margin (5 to 595).");
  for (let i = 0; i < n; i++) {
    const a = poly[i], b = poly[(i + 1) % n], z = poly[(i - 1 + n) % n];
    const ex = b[0] - a[0], ey = b[1] - a[1], ix = a[0] - z[0], iy = a[1] - z[1];
    if (Math.hypot(ex, ey) < .1) throw new Error("Adjacent vertices are too close together.");
    if (Math.abs(cross(ix, iy, ex, ey)) < 1e-8 && ix * ex + iy * ey < 0) throw new Error("The boundary doubles back over itself.");
  }
  for (let i = 0; i < n; i++) {
    const a = poly[i], b = poly[(i + 1) % n];
    for (let j = i + 1; j < n; j++) {
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;
      const c = poly[j], d = poly[(j + 1) % n];
      let disjoint = false;
      for (let k = 0; k < 2; k++)
        if (Math.max(Math.min(a[k], b[k]), Math.min(c[k], d[k])) > Math.min(Math.max(a[k], b[k]), Math.max(c[k], d[k])) + 1e-9) disjoint = true;
      if (disjoint) continue;
      const bax = b[0] - a[0], bay = b[1] - a[1], dcx = d[0] - c[0], dcy = d[1] - c[1];
      if (cross(bax, bay, c[0] - a[0], c[1] - a[1]) * cross(bax, bay, d[0] - a[0], d[1] - a[1]) <= 1e-9 &&
          cross(dcx, dcy, a[0] - c[0], a[1] - c[1]) * cross(dcx, dcy, b[0] - c[0], b[1] - c[1]) <= 1e-9)
        throw new Error("The boundary crosses or touches itself. Move its vertices apart.");
    }
  }
  const area = polygonArea(poly);
  if (Math.abs(area) < 100) throw new Error("The shape is too small. Draw a larger closed region.");
  return area > 0 ? poly : poly.slice().reverse();
}

// Ray parameters t where origin + t*direction meets the polygon boundary.
export function intersectionParameters(ox, oy, dx, dy, poly) {
  const hits = [], extra = [], n = poly.length, norm2 = dx * dx + dy * dy;
  for (let i = 0; i < n; i++) {
    const p = poly[i], q = poly[(i + 1) % n];
    const ex = q[0] - p[0], ey = q[1] - p[1], rx = p[0] - ox, ry = p[1] - oy;
    const den = cross(dx, dy, ex, ey);
    if (Math.abs(den) > 1e-10) {
      const t = cross(rx, ry, ex, ey) / den, u = cross(rx, ry, dx, dy) / den;
      if (u >= -1e-9 && u <= 1 + 1e-9) hits.push(t);
    } else if (Math.abs(cross(rx, ry, dx, dy)) < 1e-8 && norm2 > 1e-16) {
      hits.push((rx * dx + ry * dy) / norm2);
      extra.push(((rx + ex) * dx + (ry + ey) * dy) / norm2);
    }
  }
  return hits.concat(extra);
}

export function firstHit(ox, oy, dx, dy, poly) {
  let best = Infinity;
  for (const t of intersectionParameters(ox, oy, dx, dy, poly)) if (t > 1e-9 && t < best) best = t;
  return Number.isFinite(best) ? best : 0;
}

export function pointInPolygon(x, y, poly) {
  let inside = false;
  for (let i = 0, n = poly.length, j = n - 1; i < n; j = i++) {
    const a = poly[j], b = poly[i];
    if ((a[1] > y) !== (b[1] > y) && x < a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])) inside = !inside;
  }
  return inside;
}

// Calls visit(r, c) for integer grid points (col = x, row = y) inside the polygon (crossing rule, like skimage.draw.polygon).
export function fillPolygon(poly, rows, cols, visit) {
  let ymin = Infinity, ymax = -Infinity;
  for (const p of poly) { if (p[1] < ymin) ymin = p[1]; if (p[1] > ymax) ymax = p[1]; }
  const r0 = Math.max(0, Math.ceil(ymin)), r1 = Math.min(rows - 1, Math.floor(ymax)), n = poly.length, xs = [];
  for (let r = r0; r <= r1; r++) {
    xs.length = 0;
    for (let i = 0, j = n - 1; i < n; j = i++) {
      const a = poly[j], b = poly[i];
      if ((a[1] > r) !== (b[1] > r)) xs.push(a[0] + (r - a[1]) * (b[0] - a[0]) / (b[1] - a[1]));
    }
    xs.sort((u, v) => u - v);
    for (let k = 0; k + 1 < xs.length; k += 2) {
      const c0 = Math.max(0, Math.ceil(xs[k])), c1 = Math.min(cols - 1, Math.ceil(xs[k + 1]) - 1);
      for (let c = c0; c <= c1; c++) visit(r, c);
    }
  }
}

export function lineLength(line) {
  let s = 0;
  for (let i = 1; i < line.length; i++) s += Math.hypot(line[i][0] - line[i - 1][0], line[i][1] - line[i - 1][1]);
  return s;
}

export function cumulativeArc(line) {
  const arc = new Float64Array(line.length);
  for (let i = 1; i < line.length; i++) arc[i] = arc[i - 1] + Math.hypot(line[i][0] - line[i - 1][0], line[i][1] - line[i - 1][1]);
  return arc;
}

// np.interp of a polyline at sorted arc-length targets.
export function interpAlong(line, arc, targets) {
  const out = new Array(targets.length), n = line.length;
  let j = 0;
  for (let k = 0; k < targets.length; k++) {
    const t = targets[k];
    if (t <= arc[0]) { out[k] = [line[0][0], line[0][1]]; continue; }
    if (t >= arc[n - 1]) { out[k] = [line[n - 1][0], line[n - 1][1]]; continue; }
    while (j < n - 2 && arc[j + 1] <= t) j++;
    const span = arc[j + 1] - arc[j], u = span > 0 ? (t - arc[j]) / span : 0;
    out[k] = [line[j][0] + u * (line[j + 1][0] - line[j][0]), line[j][1] + u * (line[j + 1][1] - line[j][1])];
  }
  return out;
}

export function linspace(a, b, n) {
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) out[i] = n === 1 ? a : a + (b - a) * i / (n - 1);
  return out;
}

export function resample(poly, spacing) {
  const closed = poly.concat([poly[0]]), arc = cumulativeArc(closed), perimeter = arc[arc.length - 1];
  const count = Math.trunc(Math.min(Math.max(perimeter / spacing, 400), 6000));
  const raw = [];
  for (let i = 0; i < count; i++) raw.push(i * perimeter / count);
  for (let i = 0; i < poly.length; i++) raw.push(arc[i]);
  raw.sort((a, b) => a - b);
  const unique = raw.filter((v, i) => i === 0 || v !== raw[i - 1]);
  const pts = interpAlong(closed, arc, unique), samples = [], sites = [];
  for (let i = 0; i < pts.length; i++) {
    if (i === 0 || Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) > 1e-6) { samples.push(pts[i]); sites.push(unique[i]); }
  }
  return { samples, sites: Float64Array.from(sites), perimeter };
}

export function convexHull(poly) {
  const pts = poly.map(p => p.slice()).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const turn = (o, a, b) => cross(a[0] - o[0], a[1] - o[1], b[0] - o[0], b[1] - o[1]);
  const lower = [], upper = [];
  for (const p of pts) { while (lower.length >= 2 && turn(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop(); lower.push(p); }
  for (let i = pts.length - 1; i >= 0; i--) { const p = pts[i]; while (upper.length >= 2 && turn(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop(); upper.push(p); }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  return { area: Math.abs(polygonArea(hull)), perimeter: lineLength(hull.concat([hull[0]])) };
}

// Principal axes of centred points: singular values and the leading right singular vector.
export function principalAxes(points) {
  let mx = 0, my = 0;
  for (const p of points) { mx += p[0]; my += p[1]; }
  mx /= points.length; my /= points.length;
  let sxx = 0, sxy = 0, syy = 0;
  for (const p of points) { const x = p[0] - mx, y = p[1] - my; sxx += x * x; sxy += x * y; syy += y * y; }
  const tr = sxx + syy, det = sxx * syy - sxy * sxy, disc = Math.sqrt(Math.max(tr * tr / 4 - det, 0));
  const l0 = tr / 2 + disc, l1 = Math.max(tr / 2 - disc, 0);
  let vx, vy;
  if (Math.abs(sxy) > 1e-12 * Math.max(tr, 1)) { vx = sxy; vy = l0 - sxx; } else if (sxx >= syy) { vx = 1; vy = 0; } else { vx = 0; vy = 1; }
  const norm = Math.hypot(vx, vy);
  vx /= norm; vy /= norm;
  // LAPACK sign convention is arbitrary; pick the one whose largest component is negative/positive consistently.
  if (Math.abs(vx) >= Math.abs(vy) ? vx > 0 : vy > 0) { vx = -vx; vy = -vy; }
  return { singular: [Math.sqrt(l0), Math.sqrt(l1)], direction: [vx, vy], mean: [mx, my] };
}

// Exact nearest-neighbour queries on a uniform bucket grid (replacement for cKDTree).
export class PointIndex {
  constructor(points) {
    const n = points.length;
    this.n = n;
    this.xs = new Float64Array(n); this.ys = new Float64Array(n);
    let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
    for (let i = 0; i < n; i++) {
      const x = points[i][0], y = points[i][1];
      this.xs[i] = x; this.ys[i] = y;
      if (x < minx) minx = x; if (x > maxx) maxx = x; if (y < miny) miny = y; if (y > maxy) maxy = y;
    }
    const w = Math.max(maxx - minx, 1e-9), h = Math.max(maxy - miny, 1e-9);
    let cell = Math.max(Math.sqrt(w * h / Math.max(n, 1)) * 1.5, Math.max(w, h) / 256, 1e-6);
    this.cell = cell; this.minx = minx; this.miny = miny;
    this.nx = Math.floor(w / cell) + 1; this.ny = Math.floor(h / cell) + 1;
    const cells = this.nx * this.ny, start = new Int32Array(cells + 1), key = new Int32Array(n);
    for (let i = 0; i < n; i++) {
      const cx = Math.min(this.nx - 1, Math.floor((this.xs[i] - minx) / cell));
      const cy = Math.min(this.ny - 1, Math.floor((this.ys[i] - miny) / cell));
      key[i] = cy * this.nx + cx; start[key[i] + 1]++;
    }
    for (let i = 0; i < cells; i++) start[i + 1] += start[i];
    const fill = start.slice(0, cells), items = new Int32Array(n);
    for (let i = 0; i < n; i++) items[fill[key[i]]++] = i;
    this.start = start; this.items = items;
    this.distance = 0;
  }
  // Returns the index of the nearest point (first one among exact ties); distance in this.distance.
  query(x, y) {
    const { cell, nx, ny, start, items, xs, ys } = this;
    const cx = Math.min(nx - 1, Math.max(0, Math.floor((x - this.minx) / cell)));
    const cy = Math.min(ny - 1, Math.max(0, Math.floor((y - this.miny) / cell)));
    let best = Infinity, bestIndex = -1;
    const maxRing = Math.max(nx, ny);
    for (let k = 0; k <= maxRing; k++) {
      if (bestIndex >= 0) { const bound = (k - 1) * cell; if (bound > 0 && bound * bound >= best) break; }
      const y0 = cy - k, y1 = cy + k, x0 = cx - k, x1 = cx + k;
      for (let gy = Math.max(0, y0); gy <= Math.min(ny - 1, y1); gy++) {
        const edgeRow = gy === y0 || gy === y1, step = edgeRow ? 1 : Math.max(1, x1 - x0);
        for (let gx = x0; gx <= x1; gx += step) {
          if (gx < 0 || gx >= nx) continue;
          const c = gy * nx + gx;
          for (let m = start[c]; m < start[c + 1]; m++) {
            const i = items[m], dx = xs[i] - x, dy = ys[i] - y, d = dx * dx + dy * dy;
            if (d < best || (d === best && i < bestIndex)) { best = d; bestIndex = i; }
          }
        }
      }
    }
    this.distance = Math.sqrt(best);
    return bestIndex;
  }
}

export function percentile(values, p) {
  const s = Float64Array.from(values).sort(), pos = p / 100 * (s.length - 1), lo = Math.floor(pos), hi = Math.ceil(pos);
  return s[lo] + (s[hi] - s[lo]) * (pos - lo);
}

export const roundTo = (x, digits) => { const f = 10 ** digits; const v = Math.round(x * f) / f; return v === 0 ? 0 : v; };
