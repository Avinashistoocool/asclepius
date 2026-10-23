import math
import unittest

import numpy as np

from boundary import inside, validate_polygon
from exact import evaluate_pieces, preset_boundary, reflection_construction
from geometry import analyze
from numerical import nearest_boundary, normal_chords


def ribbon(amplitude=40, wobble=5):
    t = np.linspace(0, 1, 120)
    path = np.c_[100 + 400 * t, 300 + amplitude * np.sin(np.pi * t) + wobble * np.sin(2 * np.pi * t)]
    velocity = np.c_[np.full_like(t, 400), amplitude * np.pi * np.cos(np.pi * t) + 2 * np.pi * wobble * np.cos(2 * np.pi * t)]
    normal = np.c_[-velocity[:, 1], velocity[:, 0]] / np.linalg.norm(velocity, axis=1)[:, None]
    width = 35 + 5 * np.sin(np.pi * t)
    return np.vstack((path + width[:, None] * normal, (path - width[:, None] * normal)[::-1]))


class ExactConstructionTests(unittest.TestCase):
    def check_preset(self, kind, **parameters):
        model = {"kind": kind, **parameters}
        result = analyze(preset_boundary(model), model=model)
        self.assertEqual(result["mode"], "exact")
        self.assertLess(result["stats"]["maxMidpointError"], 1e-7)
        for chord in result["chords"]:
            center, a, b = np.asarray(chord["center"]), np.asarray(chord["a"]), np.asarray(chord["b"])
            np.testing.assert_allclose((a + b) / 2, center, atol=1e-7)
        return result

    def test_circle_diameter_and_boundary(self):
        result = self.check_preset("circle")
        np.testing.assert_allclose(np.asarray(result["path"])[[0, -1]], [[95, 300], [505, 300]], atol=1e-8)
        for chord in result["chords"]:
            for key in ("a", "b"):
                self.assertAlmostEqual(np.linalg.norm(np.asarray(chord[key]) - 300), 205, places=8)

    def test_ellipse_major_axis(self):
        result = self.check_preset("ellipse")
        self.assertAlmostEqual(result["stats"]["length"], 459.2)
        for chord in result["chords"]:
            for key in ("a", "b"):
                x, y = np.asarray(chord[key]) - 300
                self.assertAlmostEqual((x / 229.6) ** 2 + (y / 129.15) ** 2, 1, places=8)

    def test_annular_sector_is_exact_curved_trajectory(self):
        result = self.check_preset("crescent", depth=.55)
        points, velocity = evaluate_pieces(result["curve"]["pieces"], np.linspace(0, 1, 2347))
        radius = (220 + 120.5) / 2
        np.testing.assert_allclose(np.linalg.norm(points - 300, axis=1), radius, atol=1e-9)
        np.testing.assert_allclose(np.sum((points - 300) * velocity, axis=1), 0, atol=1e-8)
        self.assertAlmostEqual(result["stats"]["length"], radius * 1.54 * np.pi, delta=.01)

    def test_stars_with_negative_turns(self):
        for arms in (3, 5, 8, 12):
            with self.subTest(arms=arms):
                result = self.check_preset("star", arms=arms, depth=.8)
                self.assertEqual(result["stats"]["concaveVertices"], arms)
                expected = 440 if arms % 2 == 0 else 264
                self.assertAlmostEqual(result["stats"]["length"], expected)

    def test_flower_and_neck(self):
        self.check_preset("flower", arms=7, depth=.65)
        self.check_preset("neck", depth=.8)

    def test_exact_cartesian_symmetry_without_preset(self):
        polygon = np.array([[80, 180], [240, 180], [240, 260], [360, 260], [360, 180],
                            [520, 180], [520, 420], [360, 420], [360, 340],
                            [240, 340], [240, 420], [80, 420]], dtype=float)
        result = analyze(polygon)
        self.assertEqual(result["mode"], "exact")
        self.assertEqual(result["boundaryModel"]["kind"], "polygon-reflection")
        self.assertAlmostEqual(result["stats"]["length"], 440)

    def test_almost_symmetric_is_not_claimed_exact(self):
        polygon = np.array([[100, 100], [500, 100.000000001], [500, 500], [100, 500]], dtype=float)
        self.assertIsNone(reflection_construction(polygon))

    def test_edited_preset_metadata_is_rejected(self):
        model = {"kind": "ellipse"}
        polygon = preset_boundary(model)
        polygon[0, 0] -= 1
        with self.assertRaisesRegex(ValueError, "no longer matches"):
            analyze(polygon, model=model)


class NumericalConstructionTests(unittest.TestCase):
    def test_curved_asymmetric_ribbon_and_off_grid_chords(self):
        polygon = validate_polygon(ribbon())
        result = analyze(polygon, tolerance=.05)
        self.assertEqual(result["mode"], "numerical")
        self.assertLessEqual(result["stats"]["maxMidpointError"], .05)
        self.assertGreater(result["stats"]["validationSamples"], 3000)
        parameters = (np.arange(10007) + .314159) / 10008
        points, velocity = evaluate_pieces(result["curve"]["pieces"], parameters)
        lo, hi, _, _, normals, valid = normal_chords(points, velocity, polygon)
        self.assertTrue(np.all(valid))
        self.assertLess(np.max(np.abs(lo + hi) / 2), .05)
        np.testing.assert_allclose(np.sum(normals * velocity, axis=1), 0, atol=1e-9)
        for endpoint in np.asarray(result["path"])[[0, -1]]:
            self.assertLess(np.linalg.norm(endpoint - nearest_boundary(endpoint, polygon)), 1e-7)
        self.assertTrue(all(inside(point, polygon) for point in points[::13]))

    def test_cubic_spline_is_c2_at_joins(self):
        result = analyze(ribbon(20, -3), tolerance=.1)
        pieces = result["curve"]["pieces"]
        for left, right in zip(pieces[:-1], pieces[1:]):
            a, b = np.asarray(left["controls"]), np.asarray(right["controls"])
            dl, dr = np.diff(left["interval"])[0], np.diff(right["interval"])[0]
            np.testing.assert_allclose(a[-1], b[0], atol=1e-7)
            np.testing.assert_allclose(3 * (a[-1] - a[-2]) / dl, 3 * (b[1] - b[0]) / dr, atol=1e-6)
            np.testing.assert_allclose(6 * (a[-1] - 2 * a[-2] + a[-3]) / dl ** 2,
                                       6 * (b[2] - 2 * b[1] + b[0]) / dr ** 2, atol=1e-5)

    def test_reversed_winding(self):
        result = analyze(ribbon(25, 2)[::-1])
        self.assertEqual(result["mode"], "numerical")
        self.assertLess(result["stats"]["maxMidpointError"], .1)

    def test_invalid_boundaries(self):
        for points in (
            [[100, 100], [500, 500], [100, 500], [500, 100]],
            [[100, 100], [100, 100], [300, 300]],
            [[math.nan, 100], [200, 300], [300, 100]],
            [[100, 100], [300, 100], [200, 100], [200, 300]],
        ):
            with self.subTest(points=points), self.assertRaises(ValueError):
                analyze(points)

    def test_invalid_options(self):
        polygon = preset_boundary({"kind": "circle"})
        for options in ({"resolution": 1}, {"tolerance": 0}, {"tolerance": math.nan},
                        {"tolerance": "a"}, {"model": {"kind": "unknown"}}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                analyze(polygon, **options)


if __name__ == "__main__":
    unittest.main()
