// Node verification: runs the browser planner on preset fields, checks invariants and compares with Python sweep.compare().
// Usage: node docs/verify.mjs   (add --no-python to skip the Python comparison)
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import path from "node:path";
import { compare } from "./planner.js";
import { makeShape } from "./shapes.js";

const here = path.dirname(fileURLToPath(import.meta.url));
const labDir = path.resolve(here, "..", "swarm_lab");
const CASES = [
  ["ellipse", makeShape("ellipse", 0)],
  ["star5 d.55", makeShape("star", 0, .55, 5)],
  ["neck d.8", makeShape("neck", 0, .8)],
  ["crescent d.55", makeShape("crescent", 0, .55)],
  ["blob seed7", makeShape("blob", 7)],
];
const ORDER = ["mat", "lawnmower", "harmonic", "geodesic"];
const KEYS = ["coverage", "peakSwath", "missionLength", "maxSubSwarms", "speedSpread", "areaBalance", "longestDronePath", "transits", "maxAltitude", "meanAltitude", "folds"];
const DRONES = 12;

const failures = [];
const check = (ok, message) => { if (!ok) failures.push(message); };

const js = {};
for (const [name, points] of CASES) {
  const progress = [];
  const result = compare(points, { drones: DRONES }, (f, m) => progress.push([f, m]));
  js[name] = result;
  check(progress.length > 5 && progress.at(-1)[0] === 1, `${name}: progress events missing`);
  check(progress.every((p, i) => i === 0 || p[0] >= progress[i - 1][0] - 1e-12), `${name}: progress not monotonic`);
  for (const key of ["boundary", "trunk", "start", "end", "drones", "camera", "complexity", "elapsedMs", "methods"])
    check(key in result, `${name}: missing ${key}`);
  for (const method of ORDER) {
    const m = result.methods[method], tag = `${name}/${method}`;
    check(m && m.label && m.frames.length === 90, `${tag}: bad frames`);
    const s = m.stats;
    check(s.cameraCoverage === 100, `${tag}: cameraCoverage ${s.cameraCoverage}`);
    check(s.gapCells === 0, `${tag}: gapCells ${s.gapCells}`);
    check(s.coverage >= 99, `${tag}: coverage ${s.coverage}`);
    for (const k of ["computeMs", "cameraCoverage", "gapCells", "rounds", "maxAltitude", "meanAltitude", "meanOverlap", "areaBalance", "longestDronePath",
      "transits", "folds", "coverage", "maxSubSwarms", "peakSwath", "missionLength", "speedSpread", "meanFrontLength", "starvedFrames"])
      check(typeof s[k] === "number" && Number.isFinite(s[k]), `${tag}: stat ${k} = ${s[k]}`);
    const ids = new Set(m.lanes.values.filter(v => v >= 0));
    for (let i = 0; i < DRONES; i++) check(ids.has(i), `${tag}: lane image lacks drone ${i}`);
    check(m.lanes.width * m.lanes.height === m.lanes.values.length && m.coverage.values.length === m.lanes.values.length &&
      m.imaged.values.length === m.lanes.values.length, `${tag}: image sizes`);
    check(m.drones.length === DRONES, `${tag}: drone count`);
    m.drones.forEach((d, i) => {
      check(d.path.length > 0 && d.heading.length === d.path.length && d.altitude.length === d.path.length, `${tag}: drone ${i} array lengths`);
      check(d.altitude.every(a => a > 0), `${tag}: drone ${i} non-positive altitude`);
      for (const k of ["area", "length", "transits", "maxAltitude"]) check(Number.isFinite(d[k]), `${tag}: drone ${i} ${k}`);
    });
    for (const f of m.frames) for (const d of f.drones) check(d.length === 3 || (d.length === 5 && d[4] > 0), `${tag}: frame drone tuple`);
  }
}

let py = null;
if (!process.argv.includes("--no-python")) {
  const script = [
    "import json,sys", "from sweep import compare", "cases=json.load(sys.stdin)", "out={}",
    "for name,pts in cases:",
    "    r=compare(pts,drones=" + DRONES + ")",
    "    out[name]={'elapsedMs':r['elapsedMs'],'score':r['complexity']['score'],'methods':{k:v['stats'] for k,v in r['methods'].items()}}",
    "print(json.dumps(out))",
  ].join("\n");
  for (const exe of ["python", "py"]) {
    const run = spawnSync(exe, ["-c", script], { cwd: labDir, input: JSON.stringify(CASES), encoding: "utf8", maxBuffer: 64 << 20 });
    if (run.status === 0) { py = JSON.parse(run.stdout.trim().split("\n").at(-1)); break; }
    if (run.error?.code !== "ENOENT") { console.warn(`Python comparison failed:\n${run.stderr}`); break; }
  }
}

const pad = (v, w) => String(v).padStart(w);
const cell = (a, b) => (b === undefined ? `${a}` : `${a} / ${b}`);
console.log(`\nJS vs Python (${py ? "js / py" : "js only"}), ${DRONES} drones`);
for (const [name] of CASES) {
  const r = js[name], p = py?.[name];
  console.log(`\n== ${name}: complexity ${cell(r.complexity.score, p?.score)}, elapsed ms ${cell(r.elapsedMs, p?.elapsedMs)}`);
  console.log(["stat".padEnd(17), ...ORDER.map(m => pad(m, 18))].join(""));
  for (const k of KEYS) console.log([k.padEnd(17), ...ORDER.map(m => pad(cell(r.methods[m].stats[k], p?.methods[m][k]), 18))].join(""));
}

// Same-order-of-magnitude report for the shape-driven metrics (informational; the trunk smoothing differs).
if (py) {
  const notes = [];
  for (const [name] of CASES) for (const m of ORDER) for (const k of ["peakSwath", "missionLength", "longestDronePath", "meanAltitude"]) {
    const a = js[name].methods[m].stats[k], b = py[name].methods[m][k];
    if (a > 0 && b > 0 && Math.max(a / b, b / a) > 3) notes.push(`${name}/${m}/${k}: js ${a} vs py ${b}`);
  }
  console.log(notes.length ? `\nMetrics differing by more than 3x:\n  ${notes.join("\n  ")}` : "\nAll compared magnitudes within 3x of Python.");
}

const slowest = Math.max(...Object.values(js).map(r => r.elapsedMs));
console.log(`\nSlowest JS compare: ${slowest} ms`);
check(slowest < 5000, `JS compare too slow: ${slowest} ms`);
if (failures.length) { console.error(`\nFAILED (${failures.length}):\n  ${failures.join("\n  ")}`); process.exit(1); }
console.log("\nAll checks passed.");
