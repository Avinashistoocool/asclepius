import math
import unittest

import numpy as np

from boundary import inside
from exact import preset_boundary
from medial_axis import analyze


def blob(seed, depth=.8):
    rng = np.random.default_rng(seed)
    t = np.arange(160) / 160 * 2 * np.pi
    phases, amps = rng.random(7) * 2 * np.pi, (rng.random(7) * .6 + .4) / np.arange(2, 9) ** 1.3
    r = 160 + sum(95 * depth * amps[k] * np.cos((k + 2) * t + phases[k]) for k in range(7))
    return np.c_[300 + r * np.cos(t), 300 + r * np.sin(t)]


class MedialAxisTests(unittest.TestCase):
    def test_every_preset_and_random_blob_succeeds(self):
        shapes = [preset_boundary({"kind": k, "depth": .55, "arms": 5})
                  for k in ("circle", "ellipse", "star", "flower", "crescent", "neck")]
        shapes += [blob(seed) for seed in range(20)]
        for polygon in shapes:
            result = analyze(polygon.tolist())
            path = np.asarray(result["path"])
            self.assertGreater(result["stats"]["length"], 50)
            self.assertTrue(all(inside(p, polygon) for p in path[::10]))

    def test_ellipse_trunk_is_major_axis(self):
        result = analyze(preset_boundary({"kind": "ellipse"}).tolist())
        path = np.asarray(result["path"])
        self.assertLess(np.max(np.abs(path[:, 1] - 300)), 1.)
        self.assertAlmostEqual(result["stats"]["length"], 2 * 205 * 1.12, delta=2)

    def test_crescent_follows_mid_radius(self):
        result = analyze(preset_boundary({"kind": "crescent", "depth": .55}).tolist(), extend=False)
        radius = np.linalg.norm(np.asarray(result["path"]) - 300, axis=1)
        self.assertLess(np.max(np.abs(radius[50:-50] - (220 + 120.5) / 2)), 2.)

    def test_progress_is_monotone_and_complete(self):
        events = []
        analyze(blob(3).tolist(), progress=lambda f, m: events.append(f))
        self.assertEqual(events[-1], 1.)
        self.assertEqual(events, sorted(events))

    def test_invalid_input(self):
        for kwargs in ({"points": [[100, 100], [500, 500], [100, 500], [500, 100]]},
                       {"points": [[math.nan, 100], [200, 300], [300, 100]]},
                       {"points": preset_boundary({"kind": "circle"}).tolist(), "prune": 2}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                analyze(**kwargs)



if __name__ == "__main__":
    unittest.main()
