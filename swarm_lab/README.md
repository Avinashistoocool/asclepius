# Medial Axis Trajectory Lab

Computes the **medial axis transform (MAT)** of a closed 2D region and extracts a
smooth central trajectory. The MAT always exists, so there is no "no solution found" failure.

## Run (Windows)

```powershell
py server.py
```

Open <http://127.0.0.1:8765>. A progress bar under the canvas shows each stage live
(the server streams newline-delimited JSON progress events).

## Method

1. Resample the polygon boundary at ~1 canvas unit spacing.
2. Voronoi diagram of the samples; keep edges whose both vertices are inside the shape.
3. Prune branches whose two generating samples are closer than `prune x perimeter`
   along the boundary (removes noise spurs from corners/wiggles).
4. Minimum spanning tree of the largest component; the longest path is the **trunk**.
5. Smooth the trunk with a cubic spline (kept inside the shape) and optionally extend
   both ends tangentially to the boundary.
6. Disk-like shapes (MAT collapses to a point) use the principal axis through the
   largest inscribed disk.

Each trunk point also carries its inscribed-disk radius (shown as the moving circle).

## Note on chord bisection

The MAT is *equidistant to the boundary*. That equals "bisects its perpendicular chord"
only where the two walls are locally parallel. The chord-midpoint offset is shown as a
diagnostic, not as an acceptance criterion.

## Tests

```powershell
py -m unittest test_medial -v
```

## Swarm sweep comparison (`/compare`)

Each planner defines an arrival time over the field; the formation at time t is the
front where arrival = t. Drones are spaced evenly along the front, so per-drone swath
(altitude) = front length / drones in that piece. Multiple front pieces = a split.

| Planner | Idea |
| --- | --- |
| MAT + chord | Rigid line on the perpendicular chord of the medial trunk |
| Lawnmower | Straight line along the principal axis; splits around concavities |
| Harmonic | Level sets of a Laplace solution between the two trunk end caps |
| Geodesic | Wavefront of shortest-path distance from the start cap |

Metrics: coverage, peak swath, mission length (fastest drone), max sub-swarms,
speed spread, and MAT chord folds (where consecutive chords cross).

### Shape complexity panel

Area, perimeter, circularity, solidity (area / hull area), convexity, reflex vertices,
total turning / 2pi, elongation (principal-axis ratio), max inscribed radius, width ratio
along the trunk, necks (pinches below 75% of the widest point on both sides), and medial
tree edges. Composite score 0-100: non-convexity 30%, boundary roughness 25%,
elongation 15%, width variation 15%, necks 15%. Each planner card also reports its own
server compute time.

### Drone areas and paths

Each front is oriented consistently over time and given a lane coordinate s in [0, 1].
Drone i owns s in [i/N, (i+1)/N): every swept cell gets the lane of its nearest front
point, giving a partition of the field into N coloured areas. The drone's path is the
centre of its lane (s = (i + 0.5)/N) traced through all frames. Card metrics add area
balance (min/max area), longest drone path, and transit jumps between split pieces.
