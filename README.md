# Team Asclepius: NIDAR Rescueswarm

This repository contains the software tools for a search-and-rescue drone swarm for the NIDAR challenge. The tools do these tasks:

- They plan the coverage of a field.
- They manage the fleet.
- They find persons with a detector on the drone.

| Path | Contents |
| --- | --- |
| [`swarm_lab/`](swarm_lab/) | Coverage planner: medial axis, a side-by-side comparison of four sweep planners, an area for each drone, flight paths, nadir camera plan |
| [`swarm_lab/MANUAL.md`](swarm_lab/MANUAL.md) | **Field-to-flight manual:** draw the KML, make the missions, set up each drone and upload to it, fly, and RTL |
| [`swarm_lab/ARDUPILOT.md`](swarm_lab/ARDUPILOT.md) | **Fly the plan:** KML/KMZ/.poly ? one ArduCopter `.waypoints` file for each drone (takeoff, synchronised speeds, camera trigger distance, RTL) + optional pymavlink upload with read-back |
| [`swarm_lab/DASHBOARD.md`](swarm_lab/DASHBOARD.md) | **Monitor the flight:** live telemetry dashboard for many drones (map, one card for each drone, alerts, RTL/LOITER buttons, tlogs) |
| `Fleet_manager.py` | Fleet/lane assignment, camera parameters, and dispatch of the delivery drone |
| `docs/` | GitHub Pages version of the comparison page. It is a JS-only port (`planner.js`, `medial.js`, `nadir.js`). `node docs/verify.mjs` compares it with the Python planners |
| [`paper/`](paper/) | Coverage theory paper ([LaTeX](paper/coverage_theory.tex), [Markdown](paper/coverage_theory.md), [PDF](https://avinashistoocool.github.io/asclepius/papers/coverage_theory.pdf)): the mathematics, the solvers that failed and the reasons, and the method that we use. It includes equation checks (`verify_derivations.py`), figures (`figures.py`) and tables (`results.py`). This folder also has the software [technical report](paper/Asclepius_Technical_Report.tex) ([PDF](https://avinashistoocool.github.io/asclepius/papers/Asclepius_Technical_Report.pdf)) |
| `NIDAR_YOLO.py` | YOLO-World open-vocabulary detection (`person`, `tree`, ...) |
| `weights/`, `yolov8s-worldv2.pt` | Model weights (Git LFS) |

**Live comparison page:** <https://avinashistoocool.github.io/asclepius/>. The page runs fully
in your browser.

## Documentation

| Document | PDF | Source |
| --- | --- | --- |
| Planner site (interactive) | <https://avinashistoocool.github.io/asclepius/> | [`docs/`](docs/) |
| Coverage theory paper: *Chord-Bisecting Trajectories and Arrival-Time Sweeps for Nadir-Camera Drone Swarms* | [coverage_theory.pdf](https://avinashistoocool.github.io/asclepius/papers/coverage_theory.pdf) | [`paper/coverage_theory.tex`](paper/coverage_theory.tex) |
| Technical report: software architecture, fleet management, detection, flight interfaces and safety | [Asclepius_Technical_Report.pdf](https://avinashistoocool.github.io/asclepius/papers/Asclepius_Technical_Report.pdf) | [`paper/Asclepius_Technical_Report.tex`](paper/Asclepius_Technical_Report.tex) |

The [GitHub releases](https://github.com/Avinashistoocool/asclepius/releases) also contain the two PDFs.
To make the PDFs again, go into `paper/` and run `pdflatex` two times on each `.tex` file.

## Status (October 2026)

| Area | State |
| --- | --- |
| Coverage planner (`swarm_lab/`) | Operational. Four planners; 100% nadir coverage on all test fields; 8 unit tests pass |
| Browser demo (`docs/`) | Live on GitHub Pages, Obsidian & Lime UI, runs in a Web Worker in 0.8?2.3 s |
| Technical report (`paper/`) | LaTeX + Markdown, 4 figures, 2 result tables, 8/8 equation checks pass |
| Exact chord-bisecting trajectory | Solved for convex fields and symmetric presets. The report shows that, in general, this trajectory does not exist for strongly non-convex fields |
| ArduPilot export | `ardupilot_export.py`: one Mission Planner `.waypoints` file for each drone, KML preview, MAVLink upload. We tested the upload on simulated vehicles |
| Fleet manager / YOLO detection | No change in this round |

**Primary result.** On all test fields, the harmonic planner (`?T = 0`) needs the lowest maximum altitude.
The lawnmower and geodesic planners give the best balance of work between the drones.

**Next steps** (the report gives the details):

- A shooting search for exact centre lines in near-convex fields.
- `p`-Laplacian sweeps that make the worst altitude as low as possible.
- Harmonic fields that balance the work of the drones.
- Limits for turn rate, terrain and GPS for real flight.

---

## Coverage planning: the problem

A swarm of **N drones** flies as a flexible line over a field of any shape. Each drone
has a **nadir (straight-down) camera**. Each drone can fly at its own altitude, thus you can
adjust its ground footprint (swath). The swarm can split around obstacles and
join again later. The objective is to **image all points of the field**. The altitude, the path length and
the difference in work between the drones must stay low.

### Common model: arrival-time fields

We write each planner below as a scalar **arrival time** `T(x)` over the field, scaled
to `[0, 1]`:

- The formation at time `t` is the **front** `{x : T(x) = t}`. Marching squares gives this front.
- If the front has two or more pieces, the swarm has **split** into sub-swarms.
- The drones have equal spaces between them along the front. The swath of each drone is `front length / N`.
- The swarm sweeps a point when the front goes over it. Coverage is the fraction of cells with a finite `T`.

This model puts strategies that are not similar on the same base. Thus, we can compare them with the
same metrics.

---

## Algorithms

### 1. Medial axis transform (MAT): the backbone

The medial axis is the set of centres of maximal inscribed disks (disks that touch the
boundary at two or more points). All simple closed regions have a medial axis. Thus, this step cannot
fail with "no solution found".

1. Resample the boundary at a spacing of about 1 unit (400 to 6000 samples).
2. Calculate the **Voronoi diagram** of the samples. The interior Voronoi vertices are an approximation of the MAT.
3. **Prune** each Voronoi ridge, unless its two generator samples are farther apart along the
   boundary than `prune × perimeter` (Ogniewicz residual). This step removes spurs that come from corners
   and from boundary noise.
4. Make a **minimum spanning tree** of the largest component. Use its **longest path**
   (two Dijkstra passes) as the **trunk**.
5. Smooth the trunk with a cubic spline that stays in the field. Then extend the two ends
   tangentially to the boundary.
6. For shapes similar to a disk, the MAT collapses to a point. For these shapes, use the principal
   axis through the largest inscribed disk as a fallback.

The end points of the trunk define the **start and end caps** for the harmonic and geodesic
planners. The complexity metrics use the inscribed radius along the trunk.

> The MAT has an *equal distance* to the two sides of the boundary. It bisects its perpendicular chord only where
> the two walls are locally parallel. We tried three methods to solve for an exact chord-bisecting trajectory:
> spline collocation, boundary-pair continuation, and ODE shooting.
> The report shows that such a trajectory exists only for convex or near-convex fields.
> For this reason, we relax the sweep to arrival-time fields.

### 2. Sweep planners

| Planner | Arrival time `T(x)` | Advantages | Disadvantages |
| --- | --- | --- | --- |
| **MAT + chord** | Arc length along the trunk at which the perpendicular chord through `x` goes over the point | One rigid line, which never splits | Chords **fold** (cross each other) in bends and lobes; the swarm sweeps some areas backwards or two times |
| **Lawnmower** | Projection onto the principal axis (PCA) | All drones fly at the same speed; balanced areas | Splits around each concavity; the straight line does not follow the shape |
| **Harmonic** | Solution of Laplace's equation `∇²T = 0`, with `T = 0` on the start cap and `T = 1` on the end cap (linear along each side wall) | Smooth fronts, no folds or cusps; follows bends | Fronts come close together in narrow parts and spread in lobes, thus the speeds of the drones change much |
| **Geodesic** | Shortest-path (fast-marching, 8-connected) distance from the start cap | Constant front speed; short missions | Fronts can get cusps and split into many pieces |

Implementation details:

- **Grid:** 200 × 200 cells over a 600 × 600 canvas. Polygon rasterisation gives the mask of the field.
- **Harmonic:** a 5-point Laplacian on interior cells with Dirichlet boundary values. A sparse direct solver solves it.
- **Geodesic:** `skimage.graph.MCP_Geometric` from the cell of the start cap.
- **MAT + chord:** the planner rasterises each quad between two adjacent perpendicular chords and gives it the earlier time.
- **Fronts:** 90 levels from `t = 0.005` to `0.995`. The planner removes pieces that are shorter than 4 cells.

### 3. Areas and flight paths for each drone

1. **Give the fronts a consistent direction in time.** Resample each front. Put its pieces in order
   and flip them, so that their nearest points on the previous front have an increasing lane
   coordinate. Then join the pieces and parameterise them by normalised arc length
   `s ∈ [0, 1]`.
2. **Lanes.** Drone `i` owns `s ∈ [i/N, (i+1)/N)`. Match each swept cell to its
   nearest front frame and nearest front point. The cell gets the lane of that point. This step
   **divides the field** into N coloured areas.
3. **Paths.** Drone `i` flies `s = (i + 0.5)/N` on each front. A step that is longer than 25 units
   is a **transit jump** to a different piece of a split front.

### 4. Nadir camera plan: guaranteed coverage

A nadir camera at altitude `h`, with fields of view `HFOV` and `VFOV`, makes an image of a rectangle.
The centre of the rectangle is below the drone:

```
across-track half-width  = h · tan(HFOV / 2)
along-track  half-length = h · tan(VFOV / 2)
```

The rectangle turns to the **heading** of the drone. The heading is the direction of travel, averaged over
the adjacent path segments (the planner ignores transit jumps).

1. **Initial altitude.** For each cell that drone `i` owns, find the nearest path sample.
   Write the cell in the body frame of that sample (along, across), with a pad of half a cell
   diagonal. The necessary altitude is `max(across / tan(HFOV/2), (along − half step) / tan(VFOV/2))`.
   Take the maximum for each sample, then apply a max filter of 3 samples. For samples that have no
   cells, interpolate the value from the adjacent samples. Then multiply by `(1 + overlap)`.
2. **Verify.** Stamp the footprint at each sample, and also with small spaces between samples (not more than
   one footprint length apart, with the lower of the two altitudes). Mark each cell that has an image.
3. **Repair.** For each field cell that has no image, increase the altitude of the drone that owns it,
   around its nearest sample, by `×1.15 + 1 cell`. Then verify again (up to 8 rounds).

Result: **Nadir coverage = 100%** and **unimaged cells = 0** for all planners. The
test suite makes sure of this result. The altitude is in canvas units. Scale it by the real size of the field.

### 5. Shape complexity

| Metric | Definition |
| --- | --- |
| Circularity | `4πA / P²` (1 for a disk) |
| Solidity | area / convex hull area |
| Convexity | hull perimeter / perimeter |
| Reflex vertices | vertices with a negative turning angle |
| Bending | total absolute turning / 2π (1 for a convex shape) |
| Elongation | ratio of principal singular values |
| Width ratio | max / min inscribed radius along the inner 80% of the trunk |
| Necks | Parts of the trunk where the radius is less than 75% of the widest point on **both** sides |
| **Score (0–100)** | 30% non-convexity + 25% roughness + 15% elongation + 15% width variation + 15% necks |

---

## Comparison metrics

| Metric | Meaning | Better |
| --- | --- | --- |
| Coverage % | Cells that the front sweeps | higher |
| Peak swath | Largest share of the front for one drone. Without a camera model, this value sets the max altitude | lower |
| Mission | Path length of the fastest drone | lower |
| Max sub-swarms | Largest number of front pieces at one time | lower |
| Speed spread | 95th / 5th percentile of drone speed on the front | closer to 1 |
| Chord folds % | MAT only: steps where adjacent chords cross | lower |
| Area balance | Smallest / largest drone area | closer to 1 |
| Longest drone path | Transit jumps are not included | lower |
| Transit jumps | Hops between pieces of a split front | lower |
| Nadir coverage % / unimaged cells | After the camera plan | 100 / 0 |
| Max / mean altitude | From the camera plan | lower |
| Mean image overlap | Average number of drones that make an image of a cell | close to 1 |
| Compute ms | Time that the planner uses | lower |

### Typical results (12 drones)

Maximum altitude for 100% nadir coverage (HFOV 70?, VFOV 55?, 10% overlap; canvas units; a lower value is better):

| Field | MAT + chord | Lawnmower | Harmonic | Geodesic |
| --- | ---: | ---: | ---: | ---: |
| Ellipse | 21.4 | **21.3** | **21.3** | 24.0 |
| Star | 140.7 | 58.4 | **31.0** | 77.0 |
| C-bend | 31.8 | 55.9 | **15.0** | 23.8 |
| Narrow neck | 93.5 | 84.1 | **21.1** | 159.2 |
| Blob | 27.9 | 28.1 | **24.9** | 79.1 |

Area balance (smallest / largest drone area): lawnmower 0.73?0.87, geodesic 0.73?0.86,
harmonic 0.32?0.79, MAT 0.12?0.94. In non-convex fields, MAT chords fold on 23?52% of steps.
To make the tables again, run `py paper/results.py`.

General guide:

- **Harmonic** gives the lowest altitudes and no splits on fields with bends or necks. The disadvantage is that the drone speeds are not equal.
- **Geodesic** and **lawnmower** give a balanced workload and short missions, but they split more.
- **MAT + chord** is reliable only on near-convex or tube-like fields.

---

## Running locally

```powershell
cd swarm_lab
py -m pip install -r requirements.txt
py server.py              # http://127.0.0.1:8765  (MAT lab)
                          # http://127.0.0.1:8765/compare  (planner comparison)
py -m unittest test_medial -v
cd ..
py paper/verify_derivations.py   # equation checks
py paper/figures.py              # report figures
py paper/results.py              # result tables
```

The server sends NDJSON progress events as a stream. The two pages show a live progress bar.
The [GitHub Pages version](https://avinashistoocool.github.io/asclepius/) runs the same
pipeline in the browser without a server.

### `swarm_lab/` files

| File | Function |
| --- | --- |
| `medial_axis.py` | Voronoi MAT, pruning, trunk, inscribed radii |
| `sweep.py` | Arrival-time fields, fronts, lanes, drone paths, metrics |
| `nadir.py` | Headings, altitudes, footprint stamps, coverage repair |
| `complexity.py` | Descriptors of shape complexity |
| `boundary.py` / `exact.py` | Polygon validation and intersections; preset shapes |
| `server.py` | Local HTTP server that sends progress as a stream |
| `ardupilot_export.py` | KML/KMZ/.poly ? ArduCopter missions and MAVLink upload ([guide](swarm_lab/ARDUPILOT.md)) |
| `dashboard.py` | MAVLink telemetry dashboard for many drones on http://127.0.0.1:8090 ([guide](swarm_lab/DASHBOARD.md)) |
| `examples/` | Sample field (`field.kml`) and a fake MAVLink vehicle for upload tests |
| `index.html` / `compare.html` | MAT lab UI / comparison UI |
| `test_medial.py` | Unit tests (MAT, complexity, lane partition, 100% nadir coverage) |
| `test_ardupilot.py` | Exporter tests (projection, mission structure, altitude cap, trigger spacing, coverage) |

---

## Citing

If you use Asclepius or its coverage method, please cite the coverage theory paper. The citation
metadata is in [`CITATION.cff`](CITATION.cff) (GitHub shows it under *Cite this repository*)
and in [`.zenodo.json`](.zenodo.json).

```bibtex
@techreport{asclepius2026coverage,
  author      = {Avinash and {Team Asclepius}},
  title       = {Chord-Bisecting Trajectories and Arrival-Time Sweeps for Nadir-Camera Drone Swarms},
  institution = {Team Asclepius, NIDAR RescueSwarm},
  year        = {2026},
  url         = {https://avinashistoocool.github.io/asclepius/papers/coverage_theory.pdf}
}

@software{asclepius2026,
  author  = {Avinash and {Team Asclepius}},
  title   = {Asclepius: coverage planning, flight export and detection for the NIDAR RescueSwarm drone swarm},
  version = {1.0.0},
  year    = {2026},
  url     = {https://github.com/Avinashistoocool/asclepius}
}
```

## License

The code in this repository is MIT licensed. See [`LICENSE`](LICENSE).

The YOLO-World weights (`yolov8s-worldv2.pt`, `weights/`) are Ultralytics models. Ultralytics distributes them under
[AGPL-3.0](https://github.com/ultralytics/ultralytics/blob/main/LICENSE). The MIT license does not apply to them.
Their use must obey AGPL-3.0 (or an Ultralytics enterprise license).
