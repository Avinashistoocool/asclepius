"""Regenerate Tables 3-4 of coverage_theory.md.  Run from the repository root:  py paper/results.py"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "swarm_lab"))

from exact import preset_boundary  # noqa: E402
from medial_axis import analyze  # noqa: E402
from sweep import compare  # noqa: E402
from test_medial import blob  # noqa: E402

FIELDS = [
    ("Ellipse", preset_boundary({"kind": "ellipse"})),
    ("Star", preset_boundary({"kind": "star", "depth": .55, "arms": 5})),
    ("C-bend", preset_boundary({"kind": "crescent", "depth": .55})),
    ("Narrow neck", preset_boundary({"kind": "neck", "depth": .8})),
    ("Blob", blob(1)),
]
ORDER = ["mat", "lawnmower", "harmonic", "geodesic"]
rows = []
for name, polygon in FIELDS:
    result = compare(polygon.tolist(), drones=12, hfov=70., vfov=55., overlap=.1)
    stats = {k: result["methods"][k]["stats"] for k in ORDER}
    offset = analyze(polygon.tolist())["stats"]["meanChordError"]
    rows.append((name, stats, offset, result["elapsedMs"]))


def table(title, key, better):
    print(f"\n{title}\n| Field | MAT + chord | Lawnmower | Harmonic | Geodesic |\n| --- | ---: | ---: | ---: | ---: |")
    for name, stats, _, _ in rows:
        values = [stats[k][key] for k in ORDER]
        best = (min if better < 0 else max)(values)
        print(f"| {name} | " + " | ".join(f"**{v}**" if v == best else f"{v}" for v in values) + " |")


table("Max altitude", "maxAltitude", -1)
table("Area balance", "areaBalance", 1)
print("\n| Field | MAT chord folds % | MAT mean chord offset | nadir coverage (all) | gap cells (all) | max repair rounds | ms |")
for name, stats, offset, ms in rows:
    print(f"| {name} | {stats['mat']['folds']} | {offset:.1f} | {min(s['cameraCoverage'] for s in stats.values())} | "
          f"{sum(s['gapCells'] for s in stats.values())} | {max(s['rounds'] for s in stats.values())} | {ms} |")
