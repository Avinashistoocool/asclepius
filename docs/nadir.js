// Nadir camera planning: headings, altitudes and verified ground coverage (port of nadir.py).
import { PointIndex } from "./geometry.js";

export const JUMP = 25;

// Unit direction of travel per sample, ignoring transit jumps.
export function headings(path) {
  const n = path.length;
  if (n < 2) return path.map(() => [1, 0]);
  const out = path.map(() => [0, 0]);
  for (let i = 0; i + 1 < n; i++) {
    const dx = path[i + 1][0] - path[i][0], dy = path[i + 1][1] - path[i][1], length = Math.hypot(dx, dy);
    if (length > 1e-6 && length <= JUMP) {
      out[i][0] += dx / length; out[i][1] += dy / length;
      out[i + 1][0] += dx / length; out[i + 1][1] += dy / length;
    }
  }
  let last = [1, 0], first = -1;
  const norm = out.map(v => Math.hypot(v[0], v[1]));
  for (let i = 0; i < n; i++) {
    if (norm[i] > 1e-9) { last = [out[i][0] / norm[i], out[i][1] / norm[i]]; if (first < 0) first = i; }
    out[i] = last.slice();
  }
  if (first > 0) for (let i = 0; i < first; i++) out[i] = out[first].slice();
  return out;
}

function stamp(grid, cx, cy, hx, hy, halfAlong, halfAcross, out) {
  const { spacing: h, size: S } = grid, reach = Math.hypot(halfAlong, halfAcross);
  const c0 = Math.max(Math.floor((cx - reach) / h), 0), r0 = Math.max(Math.floor((cy - reach) / h), 0);
  const c1 = Math.min(Math.ceil((cx + reach) / h), S - 1), r1 = Math.min(Math.ceil((cy + reach) / h), S - 1);
  for (let r = r0; r <= r1; r++) {
    const ly = r * h - cy;
    for (let c = c0; c <= c1; c++) {
      const lx = c * h - cx;
      if (Math.abs(lx * hx + ly * hy) <= halfAlong && Math.abs(ly * hx - lx * hy) <= halfAcross) out[r * S + c] = 1;
    }
  }
}

// Stamp footprints densely enough along the path that consecutive images overlap.
function sweepFootprints(grid, path, plan, tanX, tanY, hits, single) {
  if (!path.length) return;
  single.fill(0);
  const { heading, altitude } = plan;
  for (let k = 0; k < path.length; k++) {
    let alt = Math.max(altitude[k], 1e-3);
    stamp(grid, path[k][0], path[k][1], heading[k][0], heading[k][1], alt * tanY, alt * tanX, single);
    if (k + 1 >= path.length) continue;
    const length = Math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1]);
    if (length > JUMP || length < 1e-9) continue;
    // Conservative: while moving between samples use the lower of the two altitudes.
    alt = Math.max(Math.min(altitude[k], altitude[k + 1]), 1e-3);
    const halfAlong = alt * tanY, halfAcross = alt * tanX;
    const steps = Math.trunc(Math.min(Math.ceil(length / Math.max(Math.min(2 * halfAlong, grid.spacing), grid.spacing / 2)), 64));
    for (let s = 1; s < steps; s++) {
      const u = s / steps;
      let dx = (1 - u) * heading[k][0] + u * heading[k + 1][0], dy = (1 - u) * heading[k][1] + u * heading[k + 1][1];
      const norm = Math.max(Math.hypot(dx, dy), 1e-9);
      dx /= norm; dy /= norm;
      stamp(grid, (1 - u) * path[k][0] + u * path[k + 1][0], (1 - u) * path[k][1] + u * path[k + 1][1], dx, dy, halfAlong, halfAcross, single);
    }
  }
  for (let i = 0; i < hits.length; i++) hits[i] += single[i];
}

export function nadirPlan(grid, lanes, paths, { hfov = 70, vfov = 55, overlap = .1, maxRounds = 8 } = {}) {
  const tanX = Math.tan(hfov * Math.PI / 360), tanY = Math.tan(vfov * Math.PI / 360);
  const { size: S, spacing: h, mask, cells } = grid;
  const owner = new Int32Array(S * S).fill(-1);
  for (const i of cells) owner[i] = lanes[i];
  const trees = paths.map(p => p.length ? new PointIndex(p) : null);
  // Any cell without a lane goes to the drone whose path passes closest.
  for (const i of cells) {
    if (owner[i] >= 0) continue;
    const x = (i % S) * h, y = Math.floor(i / S) * h;
    let best = Infinity;
    trees.forEach((tree, d) => { if (!tree) return; tree.query(x, y); if (tree.distance < best) { best = tree.distance; owner[i] = d; } });
  }
  const pad = h * Math.SQRT2 / 2;
  const plans = paths.map((path, d) => {
    const n = path.length;
    if (!n) return { heading: [], altitude: new Float64Array(0) };
    const heading = headings(path);
    let altitude = new Float64Array(n);
    const gap = [];
    for (let k = 0; k + 1 < n; k++) {
      const g = Math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1]);
      gap.push(g > JUMP ? 0 : g);
    }
    for (const i of cells) {
      if (owner[i] !== d) continue;
      const x = (i % S) * h, y = Math.floor(i / S) * h, j = trees[d].query(x, y);
      const ox = x - path[j][0], oy = y - path[j][1], [hx, hy] = heading[j];
      const along = Math.abs(ox * hx + oy * hy) + pad, across = Math.abs(-ox * hy + oy * hx) + pad;
      // Along-track reach is supplied by motion between samples; only half a step is needed.
      const halfStep = Math.max(j < n - 1 ? gap[j] : 0, j > 0 ? gap[j - 1] : 0) / 2;
      const need = Math.max(across / tanX, Math.max(along - halfStep, 0) / tanY);
      if (need > altitude[j]) altitude[j] = need;
    }
    altitude = altitude.map((_, k) => Math.max(altitude[Math.max(k - 1, 0)], altitude[k], altitude[Math.min(k + 1, n - 1)]));
    // Samples with no assigned cells still need a flyable altitude: interpolate from neighbours.
    const known = [];
    altitude.forEach((a, k) => { if (a > 0) known.push(k); });
    if (known.length && known.length < n) {
      const filled = new Float64Array(n);
      let q = 0;
      for (let k = 0; k < n; k++) {
        if (k <= known[0]) filled[k] = altitude[known[0]];
        else if (k >= known[known.length - 1]) filled[k] = altitude[known[known.length - 1]];
        else {
          while (known[q + 1] < k) q++;
          const a = known[q], b = known[q + 1];
          filled[k] = altitude[a] + (altitude[b] - altitude[a]) * (k - a) / (b - a);
        }
      }
      altitude = filled;
    }
    return { heading, altitude: altitude.map(a => Math.max(a, h) * (1 + overlap)) };
  });

  const hits = new Int32Array(S * S), single = new Uint8Array(S * S);
  let round = 0;
  for (round = 0; round < maxRounds; round++) {
    hits.fill(0);
    paths.forEach((path, d) => sweepFootprints(grid, path, plans[d], tanX, tanY, hits, single));
    const touched = paths.map(() => new Set());
    let gaps = 0;
    for (const i of cells) {
      if (hits[i]) continue;
      gaps++;
      const d = owner[i];
      if (d < 0 || !paths[d].length) continue;
      touched[d].add(trees[d].query((i % S) * h, Math.floor(i / S) * h));
    }
    if (!gaps) break;
    touched.forEach((set, d) => {
      const alt = plans[d].altitude;
      for (const j of [...set].sort((a, b) => a - b))
        for (let k = Math.max(j - 1, 0); k < Math.min(j + 2, alt.length); k++) alt[k] = alt[k] * 1.15 + h;
    });
  }
  let covered = 0, hitSum = 0;
  for (const i of cells) if (hits[i] > 0) { covered++; hitSum += hits[i]; }
  let maxAlt = 0, altSum = 0, altCount = 0;
  for (const p of plans) for (const a of p.altitude) { if (a > maxAlt) maxAlt = a; altSum += a; altCount++; }
  const r = (x, d) => Math.round(x * 10 ** d) / 10 ** d;
  return {
    plans, hits,
    stats: {
      cameraCoverage: r(100 * covered / Math.max(cells.length, 1), 2),
      gapCells: cells.length - covered,
      rounds: Math.min(round + 1, maxRounds),
      maxAltitude: r(maxAlt, 1),
      meanAltitude: r(altCount ? altSum / altCount : 0, 1),
      meanOverlap: r(covered ? hitSum / covered : 0, 2),
    },
  };
}
