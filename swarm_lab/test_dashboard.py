import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

import dashboard

HERE = os.path.dirname(os.path.abspath(__file__))


def free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class DashboardTests(unittest.TestCase):
    """Two fake flying copters -> dashboard -> HTTP API, including an RTL command."""

    @classmethod
    def setUpClass(cls):
        cls.ports = [free_port(), free_port()]
        cls.sim = subprocess.Popen([sys.executable, os.path.join(HERE, "examples", "fake_vehicle.py"),
                                    *map(str, cls.ports), "--telemetry"], stdout=subprocess.PIPE, text=True)
        time.sleep(2)
        cls.web = free_port()
        cls.logs = tempfile.mkdtemp()
        args = ["--connect", *[f"tcp:127.0.0.1:{p}" for p in cls.ports], "--port", str(cls.web),
                "--log-dir", cls.logs, "--cells", "3"]
        cls.stop = threading.Event()
        threading.Thread(target=dashboard.main, args=(args, cls.stop), daemon=True).start()
        deadline = time.time() + 20
        while time.time() < deadline:
            try:
                state = cls.get("/api/state")
                if len(state["drones"]) == 2 and all(d["lat"] for d in state["drones"]):
                    break
            except (urllib.error.URLError, ConnectionError):
                pass
            time.sleep(.5)

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.sim.terminate()
        cls.sim.communicate(timeout=5)

    @classmethod
    def get(cls, path):
        with urllib.request.urlopen(f"http://127.0.0.1:{cls.web}{path}", timeout=5) as response:
            return json.loads(response.read())

    def post(self, body, header=True):
        request = urllib.request.Request(f"http://127.0.0.1:{self.web}/api/command", json.dumps(body).encode(),
                                         {"Content-Type": "application/json", **({"X-Dashboard": "1"} if header else {})})
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            with error:
                return error.code, json.loads(error.read())

    def test_tracks_both_drones_by_sysid(self):
        state = self.get("/api/state")
        self.assertEqual([d["sysid"] for d in state["drones"]], [1, 2])
        for d in state["drones"]:
            self.assertIn(d["mode"], ("AUTO", "RTL"))   # RTL if the command test ran first
            self.assertTrue(d["armed"])
            self.assertEqual(d["gpsFix"], 3)
            self.assertIsNotNone(d["voltage"])
            self.assertAlmostEqual(d["cellV"], d["voltage"] / 3, places=2)
            self.assertIsNotNone(d["home"])
            self.assertIsNotNone(d["vibe"])
            self.assertLess(d["heartbeatAge"], 2)
        self.assertTrue(all(link["packets"] > 0 for link in state["links"]))

    def test_alerts(self):
        args = dashboard.parse(["--cells", "3"])
        hub = dashboard.Hub(dashboard.parse(["--log-dir", ""]))
        drone = dashboard.Drone(7, 0)
        drone.last_heartbeat = time.time()
        s = drone.snapshot(time.time(), args)
        s.update(voltage=10.2, armed=True, mode="AUTO", climb=-2.5, throttle=90, gpsFix=3, sats=12, hdop=.8,
                 vibe=[10, 70, 20], fence={"breach": 1}, flightS=60, altRel=2.)
        keys = {a[0]: a[1] for a in hub.alerts(drone, s, time.time())}
        self.assertEqual(keys["batt"], "crit")        # 3.4 V/cell
        self.assertEqual(keys["sink"], "crit")        # falling in AUTO
        self.assertEqual(keys["thr"], "warn")         # thrust saturation
        self.assertEqual(keys["vibe"], "crit")
        self.assertEqual(keys["fence"], "crit")
        self.assertEqual(keys["low"], "warn")
        drone.last_heartbeat = time.time() - 10
        self.assertEqual([a[0] for a in hub.alerts(drone, s, time.time())], ["link"])

    def test_plan_and_page(self):
        self.assertIn("paths", self.get("/api/plan"))
        with urllib.request.urlopen(f"http://127.0.0.1:{self.web}/", timeout=5) as response:
            self.assertIn(b"SWARM TELEMETRY", response.read())
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(f"http://127.0.0.1:{self.web}/../dashboard.py", timeout=5)
        caught.exception.close()

    def test_commands_are_limited_and_reach_the_vehicle(self):
        self.assertEqual(self.post({"target": "all", "action": "RTL"}, header=False)[0], 403)
        self.assertEqual(self.post({"target": "1", "action": "ARM"})[0], 400)
        code, body = self.post({"target": "2", "action": "RTL"})
        self.assertEqual((code, body["sent"]), (200, [2]))
        deadline = time.time() + 5
        while time.time() < deadline and self.get("/api/state")["drones"][1]["mode"] != "RTL":
            time.sleep(.3)
        state = self.get("/api/state")
        self.assertEqual(state["drones"][1]["mode"], "RTL")
        self.assertEqual(state["drones"][0]["mode"], "AUTO")   # only the target drone


if __name__ == "__main__":
    unittest.main()
