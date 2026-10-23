// Shape complexity descriptors (port of complexity.py describe()).
import { convexHull, polygonArea, principalAxes, roundTo, cross } from "./geometry.js";

export function describe(polygon, medial) {
  const n = polygon.length, area = Math.abs(polygonArea(polygon));
  let perimeter = 0, reflex = 0, turningSum = 0;
  for (let i = 0; i < n; i++) {
    const p = polygon[i], q = polygon[(i + 1) % n], z = polygon[(i - 1 + n) % n];
    const ex = q[0] - p[0], ey = q[1] - p[1], ix = p[0] - z[0], iy = p[1] - z[1];
    perimeter += Math.hypot(ex, ey);
    const turning = Math.atan2(cross(ix, iy, ex, ey), ix * ex + iy * ey);
    if (turning < -1e-3) reflex++;
    turningSum += Math.abs(turning);
  }
  const hull = convexHull(polygon);
  const bending = turningSum / (2 * Math.PI);
  const singular = principalAxes(polygon).singular;
  const radius = medial.radius, m = radius.length;
  const inner = m > 20 ? radius.slice(Math.floor(m / 10), m - Math.floor(m / 10)) : radius.slice();
  let necks = 0;
  if (inner.length > 5) {
    const left = [], right = new Array(inner.length);
    let run = -Infinity;
    for (const r of inner) { run = Math.max(run, r); left.push(run); }
    run = -Infinity;
    for (let i = inner.length - 1; i >= 0; i--) { run = Math.max(run, inner[i]); right[i] = run; }
    let previous = false;
    inner.forEach((r, i) => { const pinched = r < .75 * Math.min(left[i], right[i]); if (pinched && !previous) necks++; previous = pinched; });
  }
  const circularity = 4 * Math.PI * area / perimeter ** 2, solidity = area / hull.area, convexity = hull.perimeter / perimeter;
  const elongation = singular[0] / Math.max(singular[1], 1e-9);
  const innerMax = inner.length ? Math.max(...inner) : 0, innerMin = inner.length ? Math.min(...inner) : 0;
  const widthRatio = medial.disk || !inner.length ? 1 : innerMax / Math.max(innerMin, 1e-9);
  const terms = {
    nonConvexity: 1 - solidity,
    boundaryRoughness: Math.min((bending - 1) / 3, 1),
    elongation: Math.min((elongation - 1) / 4, 1),
    widthVariation: Math.min((widthRatio - 1) / 4, 1),
    necks: Math.min(necks / 3, 1),
  };
  const weights = { nonConvexity: .3, boundaryRoughness: .25, elongation: .15, widthVariation: .15, necks: .15 };
  let score = 0;
  for (const k in terms) score += weights[k] * Math.max(terms[k], 0);
  const roundedTerms = {};
  for (const k in terms) roundedTerms[k] = roundTo(Math.max(terms[k], 0), 3);
  return {
    score: roundTo(100 * score, 1),
    area: roundTo(area, 1),
    perimeter: roundTo(perimeter, 1),
    circularity: roundTo(circularity, 3),
    solidity: roundTo(solidity, 3),
    convexity: roundTo(convexity, 3),
    reflexVertices: reflex,
    bending: roundTo(bending, 2),
    elongation: roundTo(elongation, 2),
    maxInscribed: roundTo(Math.max(...radius), 1),
    widthRatio: roundTo(widthRatio, 2),
    necks,
    medialEdges: medial.branches,
    terms: roundedTerms,
  };
}
