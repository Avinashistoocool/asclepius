"""Minimal fake ArduCopter vehicles: MAVLink mission, fence and parameter protocols.

Use it to test mission uploads without hardware or SITL. Each vehicle is a TCP server, like ArduPilot SITL (tcp:127.0.0.1:5760):

    py examples/fake_vehicle.py 14550 14560          # two vehicles, system ids 1 and 2
    py ardupilot_export.py examples/field.kml --drones 2 \
        --connect tcp:127.0.0.1:14550 tcp:127.0.0.1:14560

Add --telemetry to make them "fly" (armed, AUTO, along the uploaded mission or a circle) and stream
position, battery, GPS, EKF, vibration and mission progress, for testing dashboard.py:

    py examples/fake_vehicle.py 5760 5770 --telemetry
    py dashboard.py --connect sitl --drones 2
"""

import math
import os
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")  # like ArduPilot: needed for fence (mission_type) messages

from pymavlink import mavutil


HOME = (28.5466, 77.1930)
MODES = mavutil.mode_mapping_byname(mavutil.mavlink.MAV_TYPE_QUADROTOR)
EARTH = 6378137.0


class Flight:
    """Very small kinematic copter: enough to exercise a telemetry dashboard, not a flight model."""

    def __init__(self, sysid, stores):
        self.sysid, self.stores = sysid, stores
        self.home = (HOME[0] + (sysid - 1) * 4e-5, HOME[1])
        self.lat, self.lon, self.alt = *self.home, 0.
        self.mode, self.armed, self.seq, self.t0 = 3, True, 1, time.time()
        self.voltage, self.mah, self.speed, self.climb, self.heading = 12.4, 0., 0., 0., 0.

    def targets(self):
        items = [i for s, i in sorted(self.stores[0].items()) if s > 0 and i.command in (16, 82) and (i.x or i.y)]
        if items:
            return [(i.seq, i.x / 1e7, i.y / 1e7, i.z) for i in items]
        return [(n + 1, self.home[0] + 3e-4 * math.cos(n * math.pi / 6), self.home[1] + 3.4e-4 * math.sin(n * math.pi / 6),
                 12. + 3 * self.sysid) for n in range(12)]

    def step(self, dt):
        name = next((k for k, v in MODES.items() if v == self.mode), "?")
        target, cruise = None, 3.
        if name == "AUTO":
            route = self.targets()
            current = next((t for t in route if t[0] >= self.seq), None)
            if current is None:
                self.mode, name = MODES["RTL"], "RTL"
            else:
                self.seq, target = current[0], current[1:]
                if self.distance(*target[:2]) < 2 and abs(self.alt - target[2]) < 1:
                    later = [t for t in route if t[0] > self.seq]
                    self.seq = later[0][0] if later else self.seq + 1000
        if name == "RTL":
            target = (*self.home, 0. if self.distance(*self.home) < 1 else max(self.alt, 15.))
        elif name == "LAND":
            target = (self.lat, self.lon, 0.)
        if target is None:                      # LOITER, BRAKE, GUIDED...: hold position
            self.speed, self.climb = 0., 0.
        else:
            north = math.radians(target[0] - self.lat) * EARTH
            east = math.radians(target[1] - self.lon) * EARTH * math.cos(math.radians(self.lat))
            horizontal = math.hypot(north, east)
            move = min(horizontal, cruise * dt)
            if horizontal > .01:
                self.heading = math.degrees(math.atan2(east, north)) % 360
                self.lat += math.degrees(move * north / horizontal / EARTH)
                self.lon += math.degrees(move * east / horizontal / EARTH / math.cos(math.radians(self.lat)))
            rise = max(-1.5 * dt, min(2 * dt, target[2] - self.alt))
            self.alt += rise
            self.speed, self.climb = move / dt, rise / dt
        if self.armed and self.alt < .1 and (name == "LAND" or (name == "RTL" and self.distance(*self.home) < 1.5)):
            self.armed, self.alt = False, 0.
        current = 14. if self.armed else .5
        self.mah += current * dt / 3.6
        self.voltage = max(9.6, 12.5 - self.mah / 500 - (.35 if self.armed else 0))
        return current

    def distance(self, lat, lon):
        north = math.radians(lat - self.lat) * EARTH
        east = math.radians(lon - self.lon) * EARTH * math.cos(math.radians(self.lat))
        return math.hypot(north, east)

    def send(self, link, current, beat):
        m, t = link.mav, int((time.time() - self.t0) * 1000) & 0xFFFFFFFF
        if beat:
            base = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
            m.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                             base, self.mode, mavutil.mavlink.MAV_STATE_ACTIVE if self.armed else 3)
            m.home_position_send(int(self.home[0] * 1e7), int(self.home[1] * 1e7), 216000, 0, 0, 0,
                                 [1, 0, 0, 0], 0, 0, 0)
            m.ekf_status_report_send(0x1FF, .05, .08, .1, .04, 0)
            m.vibration_send(0, 8 + 4 * math.sin(t / 900), 9., 14., 0, 0, 0)
            m.gps_raw_int_send(0, 3, int(self.lat * 1e7), int(self.lon * 1e7), 216000, 80, 120, 0, 0, 14)
            m.fence_status_send(0, 0, 0, 0)
            m.mission_current_send(self.seq if self.seq < 1000 else 0, max(len(self.stores[0]), 1))
        north = math.cos(math.radians(self.heading)) * self.speed
        east = math.sin(math.radians(self.heading)) * self.speed
        m.global_position_int_send(t, int(self.lat * 1e7), int(self.lon * 1e7), int((216 + self.alt) * 1000),
                                   int(self.alt * 1000), int(north * 100), int(east * 100), int(-self.climb * 100),
                                   int(self.heading * 100))
        m.vfr_hud_send(self.speed, self.speed, int(self.heading), 48 if self.armed else 0, 216 + self.alt, self.climb)
        m.attitude_send(t, 0., math.radians(-3 * self.speed), math.radians(self.heading), 0, 0, 0)
        m.sys_status_send(0x2F, 0x2F, 0x2F, 300, int(self.voltage * 1000), int(current * 100),
                          max(0, int(100 - self.mah / 22)), 0, 0, 0, 0, 0, 0)
        m.nav_controller_output_send(0, 0, int(self.heading), 0, 5, 0., 0., 0.)


def vehicle(port, sysid, stop, telemetry=False):
    link = mavutil.mavlink_connection(f"tcpin:127.0.0.1:{port}", source_system=sysid, source_component=1)
    stores, expected, params, last_beat = {0: {}, 1: {}}, {0: 0, 1: 0}, {}, 0.
    names = {0: "mission", 1: "fence"}
    flight = Flight(sysid, stores) if telemetry else None
    last_step = time.time()
    while not stop.is_set():
        now = time.time()
        if flight is None and now - last_beat > .5:
            last_beat = now
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
        if flight is not None and now - last_step >= .25:
            beat = now - last_beat > .5
            last_beat = now if beat else last_beat
            flight.send(link, flight.step(now - last_step), beat)
            last_step = now
        message = link.recv_match(blocking=True, timeout=.1)
        if message is None:
            continue
        kind, gcs = message.get_type(), (message.get_srcSystem(), message.get_srcComponent())
        kind_type = getattr(message, "mission_type", 0)
        if kind == "MISSION_COUNT":
            expected[kind_type], stores[kind_type] = message.count, {}
            link.mav.mission_request_int_send(*gcs, 0, kind_type)
        elif kind == "MISSION_ITEM_INT":
            stores[kind_type][message.seq] = message
            if message.seq + 1 < expected[kind_type]:
                link.mav.mission_request_int_send(*gcs, message.seq + 1, kind_type)
            else:
                link.mav.mission_ack_send(*gcs, mavutil.mavlink.MAV_MISSION_ACCEPTED, kind_type)
                print(f"[vehicle {sysid}] stored {len(stores[kind_type])} {names[kind_type]} items", flush=True)
        elif kind == "MISSION_REQUEST_LIST":
            link.mav.mission_count_send(*gcs, len(stores[kind_type]), kind_type)
        elif kind == "MISSION_REQUEST_INT" and message.seq in stores[kind_type]:
            item = stores[kind_type][message.seq]
            link.mav.mission_item_int_send(*gcs, item.seq, item.frame, item.command, item.current, item.autocontinue,
                                           item.param1, item.param2, item.param3, item.param4,
                                           item.x, item.y, item.z, kind_type)
        elif kind == "PARAM_SET":
            name = message.param_id.rstrip("\x00") if isinstance(message.param_id, str) else message.param_id
            params[name] = message.param_value
            link.mav.param_value_send(name.encode(), message.param_value, message.param_type, len(params), 0)
            print(f"[vehicle {sysid}] {name} = {message.param_value:g}", flush=True)
        elif kind == "COMMAND_LONG" and message.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and flight:
            flight.mode = int(message.param2)
            link.mav.command_ack_send(message.command, 0)
            link.mav.statustext_send(6, f"Mode changed to {int(message.param2)}".encode())
            print(f"[vehicle {sysid}] mode -> {int(message.param2)}", flush=True)
        elif kind == "COMMAND_LONG":
            link.mav.command_ack_send(message.command, 0)


if __name__ == "__main__":
    stop = threading.Event()
    telemetry = "--telemetry" in sys.argv
    ports = [int(p) for p in sys.argv[1:] if p != "--telemetry"]
    for index, port in enumerate(ports):
        threading.Thread(target=vehicle, args=(port, index + 1, stop, telemetry), daemon=True).start()
    print(f"Fake vehicles listening on TCP ports {' '.join(map(str, ports))}"
          f"{' (flying, streaming telemetry)' if telemetry else ''}. Ctrl+C to stop.", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()
