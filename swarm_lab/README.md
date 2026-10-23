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

The previous chord-bisection solver lives in `legacy/` for reference.

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

