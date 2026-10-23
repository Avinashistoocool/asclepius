// Preset field shapes (same as makeShape in compare.html; depth/arms exposed as optional parameters).
export function rng(seed) {
  let a = seed >>> 0;
  return () => { a = (a + 0x6D2B79F5) >>> 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t ^= t + Math.imul(t ^ (t >>> 7), 61 | t); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

export function makeShape(name, seed, depth = .55, n = 5) {
  const out = [];
  if (name === "star") { for (let i = 0; i < 2 * n; i++) { const t = -Math.PI / 2 + i * Math.PI / n, r = i % 2 ? 220 * (1 - depth) : 220; out.push([300 + r * Math.cos(t), 300 + r * Math.sin(t)]); } return out; }
  if (name === "crescent") {
    const inner = 80 + 90 * (1 - depth);
    for (let i = 0; i <= 64; i++) { const t = Math.PI * .23 + i / 64 * Math.PI * 1.54; out.push([300 + 220 * Math.cos(t), 300 + 220 * Math.sin(t)]); }
    for (let i = 64; i >= 0; i--) { const t = Math.PI * .23 + i / 64 * Math.PI * 1.54; out.push([300 + inner * Math.cos(t), 300 + inner * Math.sin(t)]); }
    return out;
  }
  if (name === "neck") { const h = 18 + 65 * (1 - depth); return [[80, 155], [220, 155], [220, 300 - h], [380, 300 - h], [380, 155], [520, 155], [520, 445], [380, 445], [380, 300 + h], [220, 300 + h], [220, 445], [80, 445]]; }
  const random = rng(seed), phases = Array.from({ length: 7 }, () => random() * 2 * Math.PI), amps = Array.from({ length: 7 }, (_, i) => (random() * .6 + .4) / Math.pow(i + 2, 1.3));
  for (let i = 0; i < 160; i++) {
    const t = i / 160 * 2 * Math.PI;
    let r = name === "flower" ? 170 * (1 + depth * .42 * Math.cos(n * t)) : 205;
    if (name === "blob") { r = 160; for (let k = 0; k < 7; k++) r += 95 * .8 * amps[k] * Math.cos((k + 2) * t + phases[k]); }
    out.push([300 + r * Math.cos(t) * (name === "ellipse" ? 1.12 : 1), 300 + r * Math.sin(t) * (name === "ellipse" ? .63 : 1)]);
  }
  return out;
}
