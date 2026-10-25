import math
import os
import tempfile
import unittest

import numpy as np

import ardupilot_export as ap

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD = os.path.join(HERE, "examples", "field.kml")


class ArduPilotExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out = tempfile.mkdtemp()
        cls.summary = ap.main([FIELD, "--drones", "3", "--out", cls.out])

    def test_kml_and_projection_round_trip(self):
        _, ring = ap.read_kml(FIELD)
        ring = np.asarray(ring)
        frame = ap.LocalFrame(*ring.mean(axis=0)[::-1])
        back = frame.to_lonlat(frame.to_en(ring))
        np.testing.assert_allclose(back, ring, atol=1e-9)
        canvas = ap.CanvasMap.fit(frame.to_en(ring))
        xy = canvas.to_canvas(frame.to_en(ring))
        self.assertTrue(np.all((xy >= ap.CANVAS_MARGIN - 1e-6) & (xy <= 600 - ap.CANVAS_MARGIN + 1e-6)))

    def test_waypoint_files_are_valid_qgc_wpl(self):
        for entry in self.summary["missions"]:
            items = ap.read_waypoints(os.path.join(self.out, entry["file"]))
            self.assertEqual(len(items), entry["items"])
            self.assertEqual(items[0].command, ap.MAV_CMD_NAV_WAYPOINT)  # home
            self.assertEqual(items[1].command, ap.MAV_CMD_NAV_TAKEOFF)
            self.assertEqual(items[-1].command, ap.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            self.assertTrue(any(i.command == ap.MAV_CMD_DO_SET_CAM_TRIGG_DIST and i.params[0] > 0 for i in items))
            self.assertEqual([i.command for i in items if i.command == ap.MAV_CMD_DO_SET_CAM_TRIGG_DIST][-1],
                             ap.MAV_CMD_DO_SET_CAM_TRIGG_DIST)
            last_trigger = [i for i in items if i.command == ap.MAV_CMD_DO_SET_CAM_TRIGG_DIST][-1]
            self.assertEqual(last_trigger.params[0], 0.)  # camera off before RTL
            for item in items:
                if item.command == ap.MAV_CMD_NAV_WAYPOINT and item.frame == ap.MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    self.assertGreaterEqual(item.alt, 15. - 1e-6)
                    self.assertLessEqual(item.alt, 120. + 1e-6)
                    self.assertTrue(28.5 < item.lat < 28.6 and 77.1 < item.lon < 77.2)

    def test_full_coverage_at_flown_altitudes(self):
        self.assertEqual(self.summary["nadirCoveragePct"], 100.0)
        self.assertFalse(self.summary["capped"])

    def test_altitude_cap_is_respected_and_reported(self):
        out = tempfile.mkdtemp()
        summary = ap.main([FIELD, "--drones", "2", "--max-alt", "20", "--min-alt", "10", "--out", out])
        self.assertTrue(all(m["maxAltM"] <= 20 + 1e-6 for m in summary["missions"]))
        if summary["requiredMaxAltM"] > 20:
            self.assertTrue(summary["capped"])
            self.assertLess(summary["nadirCoveragePct"], 100.0)

    def test_trigger_distance_matches_footprint(self):
        tan_y = math.tan(math.radians(62.22) / 2)
        items = ap.read_waypoints(os.path.join(self.out, self.summary["missions"][0]["file"]))
        triggers = [i.params[0] for i in items if i.command == ap.MAV_CMD_DO_SET_CAM_TRIGG_DIST and i.params[0] > 0]
        alts = [i.alt for i in items if i.command == ap.MAV_CMD_NAV_WAYPOINT and i.alt > 0]
        self.assertLessEqual(max(triggers), 2 * max(alts) * tan_y * (1 - .3) + .1)

    def test_test_mode_single_drone_square_at_home(self):
        out = tempfile.mkdtemp()
        summary = ap.main(["--test", "--home", "77.1930", "28.5466", "--out", out])
        self.assertTrue(summary["test"])
        self.assertEqual(summary["drones"], 1)
        self.assertEqual(len(summary["missions"]), 1)
        self.assertTrue(summary["missions"][0]["file"].startswith("TEST_"))
        self.assertLessEqual(summary["missions"][0]["maxAltM"], 30 + 1e-6)
        self.assertLess(summary["areaHa"], .65)
        self.assertEqual(summary["nadirCoveragePct"], 100.0)
        items = ap.read_waypoints(os.path.join(out, summary["missions"][0]["file"]))
        speeds = [i.params[1] for i in items if i.command == ap.MAV_CMD_DO_CHANGE_SPEED]
        self.assertTrue(speeds and max(speeds) <= 3 + 1e-6)

    def test_test_mode_clips_field_and_limits_drones(self):
        out = tempfile.mkdtemp()
        summary = ap.main([FIELD, "--test", "--drones", "2", "--out", out])
        self.assertEqual(len(summary["missions"]), 2)
        self.assertLessEqual(summary["areaHa"], .64 + 1e-6)
        with self.assertRaises(SystemExit):
            ap.parse([FIELD, "--test", "--drones", "3"])
        with self.assertRaises(SystemExit):
            ap.parse(["--test"])

    def test_sitl_shortcut(self):
        args = ap.parse([FIELD, "--test", "--drones", "2", "--connect", "sitl"])
        self.assertEqual(args.connect, ["tcp:127.0.0.1:5760", "tcp:127.0.0.1:5770"])

    def test_rtl_and_fence_params(self):
        out = tempfile.mkdtemp()
        summary = ap.main([FIELD, "--drones", "3", "--home", "77.1929", "28.5466", "--out", out])
        top = max(m["maxAltM"] for m in summary["missions"])
        rtls = [m["rtlAltM"] for m in summary["missions"]]
        self.assertTrue(all(r > top for r in rtls))
        self.assertEqual(len(set(rtls)), 3)  # distinct return heights per drone
        for i, entry in enumerate(summary["missions"]):
            path = os.path.join(out, entry["file"].replace(".waypoints", ".param"))
            values = dict(line.strip().split(",") for line in open(path) if line[0] != "#")
            self.assertEqual(int(values["RTL_ALT"]), round(entry["rtlAltM"] * 100))
            self.assertGreater(int(values["FENCE_ALT_MAX"]), entry["rtlAltM"])
            self.assertEqual(values["FENCE_ACTION"], "1")
            items = ap.read_waypoints(os.path.join(out, entry["file"]))
            self.assertEqual(items[-1].command, ap.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def test_only_one_path_covers_its_own_area(self):
        out = tempfile.mkdtemp()
        summary = ap.main([FIELD, "--drones", "4", "--only", "3", "--test", "--test-size", "200",
                           "--home", "77.1929", "28.5466", "--out", out])
        self.assertEqual(summary["onlyPath"], 3)
        self.assertEqual(len(summary["missions"]), 1)
        self.assertEqual(summary["missions"][0]["file"], "TEST_field_path03of04.waypoints")
        self.assertEqual(summary["ownAreaCoveragePct"], 100.0)
        self.assertIsNone(summary["minSeparationM"])
        items = ap.read_waypoints(os.path.join(out, summary["missions"][0]["file"]))
        self.assertEqual(items[-1].command, ap.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        with self.assertRaises(SystemExit):
            ap.parse([FIELD, "--drones", "3", "--only", "4"])


if __name__ == "__main__":
    unittest.main()
