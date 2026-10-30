"""Multi-drone telemetry dashboard for ArduPilot (MAVLink) swarms.

Listens to one or more MAVLink links, tracks every autopilot by SYSID, raises alerts and serves a live
web page (map + per-drone cards + alert log) on http://127.0.0.1:8090.

    py dashboard.py --connect udpin:0.0.0.0:14550                      # Mission Planner / MAVProxy forwarding
    py dashboard.py --connect sitl --drones 2 --mission missions        # SITL, with the planned paths drawn
    py dashboard.py --connect COM5,57600                                # telemetry radio (if nothing else uses it)

Every received packet is saved to logs/<time>_linkN.tlog (replay it in Mission Planner).
See DASHBOARD.md for the full guide.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "dashboard_static")
GCS_SYSID = 254          # not 255: the dashboard must not keep ArduPilot's GCS failsafe (SYSID_MYGCS) alive
GCS_COMPID = 191
EARTH_RADIUS = 6378137.0
SITL_PORTS = 5760
COMMANDS = ("RTL", "LOITER", "BRAKE", "LAND")   # safe "stop the mission" actions only; never arm/takeoff/AUTO
LANDING_MODES = {"LAND", "RTL", "SMART_RTL", "AUTO_RTL"}
CLEAR_DELAY = 3.
HOLD_MODES = {"AUTO", "GUIDED", "LOITER", "ALT_HOLD", "POSHOLD", "BRAKE", "CIRCLE"}
SENSOR_NAMES = {bit: name.replace("MAV_SYS_STATUS_", "").replace("SENSOR_", "")
                for bit, name in ((e, v.name) for e, v in mavutil.mavlink.enums["MAV_SYS_STATUS_SENSOR"].items())
                if bit & (bit - 1) == 0}


def distance_m(lat1, lon1, lat2, lon2):
    north = math.radians(lat2 - lat1) * EARTH_RADIUS
    east = math.radians(lon2 - lon1) * EARTH_RADIUS * math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(east, north)


# --------------------------------------------------------------------------- per-drone state
class Drone:
    def __init__(self, sysid, link_index):
        self.sysid, self.link_index = sysid, link_index
        self.s = {"sysid": sysid, "link": link_index, "mode": "?", "armed": False, "vehicle": "?",
                  "lat": None, "lon": None, "altRel": None, "altMsl": None, "heading": None, "groundSpeed": None,
                  "climb": None, "throttle": None, "roll": None, "pitch": None, "voltage": None, "current": None,
                  "remaining": None, "consumedMah": None, "gpsFix": None, "sats": None, "hdop": None,
                  "ekfFlags": None, "ekf": None, "vibe": None, "clip": None, "wpSeq": None, "wpTotal": None,
                  "wpDist": None, "altError": None, "xtrack": None, "home": None, "fence": None, "rssi": None,
                  "remRssi": None, "dropRate": None, "unhealthy": [], "lastAck": None, "armedAt": None}
        self.last_heartbeat = 0.
        self.first_seen = time.time()
        self.track = deque(maxlen=3000)
        self.history = deque(maxlen=600)            # (t, altRel, voltage, climb, throttle) ~2 Hz for 5 min
        self.messages = deque(maxlen=40)
        self.clip_prev, self.clip_rising_until = None, 0.
        self.last_hist = 0.
        self.requested = False

    def snapshot(self, now, args):
        s = dict(self.s)
        s["heartbeatAge"] = round(now - self.last_heartbeat, 1) if self.last_heartbeat else None
        s["cellV"] = round(s["voltage"] / args.cells, 3) if s["voltage"] else None
        if s["home"] and s["lat"] is not None:
            s["homeDist"] = round(distance_m(s["lat"], s["lon"], s["home"]["lat"], s["home"]["lon"]), 1)
        else:
            s["homeDist"] = None
        s["flightS"] = round(now - s["armedAt"]) if s["armed"] and s["armedAt"] else None
        s["track"] = list(self.track)[-600:]
        s["history"] = list(self.history)
        s["messages"] = list(self.messages)
        return s


class Hub:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.drones: dict[int, Drone] = {}
        self.links = []
        self.link_status = []
        self.events = deque(maxlen=300)             # alert transitions and STATUSTEXT, newest last
        self.active = {}                            # (sysid, key) -> (level, text) currently raised
        self.event_log = None
        if args.log_dir:
            os.makedirs(args.log_dir, exist_ok=True)
            self.event_log = open(os.path.join(args.log_dir, time.strftime("%Y%m%d_%H%M%S") + "_events.csv"),
                                  "a", buffering=1, encoding="utf-8")
            self.event_log.write("time,sysid,level,event\n")

    # ------------------------------------------------------------------ events
    def log(self, sysid, level, text):
        entry = {"t": time.time(), "sysid": sysid, "level": level, "text": text}
        self.events.append(entry)
        if self.event_log:
            self.event_log.write(f"{time.strftime('%H:%M:%S')},{sysid},{level},\"{text.replace(chr(34), '')}\"\n")
        if level in ("crit", "warn"):
            print(f"[{time.strftime('%H:%M:%S')}] drone {sysid} {level.upper()}: {text}", flush=True)

    # ------------------------------------------------------------------ MAVLink input
    def handle(self, link_index, link, msg):
        kind = msg.get_type()
        if kind == "BAD_DATA":
            return
        sysid, compid = msg.get_srcSystem(), msg.get_srcComponent()
        if sysid in (GCS_SYSID, 255) or sysid == 0:
            return
        now = time.time()
        if kind == "HEARTBEAT":
            if compid != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 or msg.type == mavutil.mavlink.MAV_TYPE_GCS \
                    or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                return
        with self.lock:
            drone = self.drones.get(sysid)
            if drone is None:
                if kind != "HEARTBEAT":
                    return
                drone = self.drones[sysid] = Drone(sysid, link_index)
                self.log(sysid, "info", f"new vehicle on link {link_index + 1} ({self.link_status[link_index]['url']})")
            elif drone.link_index != link_index and kind == "HEARTBEAT" and now - drone.last_heartbeat < 2:
                self.log(sysid, "warn", "two vehicles share this SYSID_THISMAV; give each drone a unique id")
                return
            s = drone.s
            if kind == "HEARTBEAT":
                drone.link_index = s["link"] = link_index
                drone.last_heartbeat = now
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode = mavutil.mode_string_v10(msg)
                if mode != s["mode"] and s["mode"] != "?":
                    self.log(sysid, "info", f"mode {s['mode']} -> {mode}")
                if armed != s["armed"]:
                    self.log(sysid, "info", "ARMED" if armed else "DISARMED")
                    s["armedAt"] = now if armed else None
                s["mode"], s["armed"], s["mavType"] = mode, armed, msg.type
                s["vehicle"] = mavutil.mavlink.enums["MAV_TYPE"][msg.type].name.replace("MAV_TYPE_", "")
                if not drone.requested:
                    drone.requested = True
                    self.request_streams(link, sysid, compid)
            elif kind == "GLOBAL_POSITION_INT":
                s["lat"], s["lon"] = msg.lat / 1e7, msg.lon / 1e7
                s["altRel"], s["altMsl"] = msg.relative_alt / 1000, msg.alt / 1000
                s["heading"] = msg.hdg / 100 if msg.hdg != 65535 else s["heading"]
                if msg.lat or msg.lon:
                    if not drone.track or distance_m(*drone.track[-1][:2], s["lat"], s["lon"]) > .5:
                        drone.track.append((round(s["lat"], 7), round(s["lon"], 7), round(s["altRel"], 1)))
                if now - drone.last_hist >= .5:
                    drone.last_hist = now
                    drone.history.append((round(now, 1), round(s["altRel"], 2), s["voltage"], s["climb"],
                                          s["throttle"]))
            elif kind == "VFR_HUD":
                s["groundSpeed"], s["climb"], s["throttle"] = round(msg.groundspeed, 2), round(msg.climb, 2), msg.throttle
                s["heading"] = msg.heading
            elif kind == "ATTITUDE":
                s["roll"], s["pitch"] = round(math.degrees(msg.roll), 1), round(math.degrees(msg.pitch), 1)
            elif kind == "SYS_STATUS":
                s["voltage"] = msg.voltage_battery / 1000 if msg.voltage_battery not in (0, 65535) else None
                s["current"] = msg.current_battery / 100 if msg.current_battery >= 0 else None
                s["remaining"] = msg.battery_remaining if msg.battery_remaining >= 0 else None
                s["dropRate"] = msg.drop_rate_comm / 100
                bad = msg.onboard_control_sensors_present & msg.onboard_control_sensors_enabled \
                    & ~msg.onboard_control_sensors_health
                s["unhealthy"] = [SENSOR_NAMES.get(1 << b, f"bit{b}") for b in range(32) if bad >> b & 1]
            elif kind == "BATTERY_STATUS" and msg.id == 0:
                if msg.current_consumed >= 0:
                    s["consumedMah"] = msg.current_consumed
            elif kind == "GPS_RAW_INT":
                s["gpsFix"], s["sats"] = msg.fix_type, msg.satellites_visible if msg.satellites_visible != 255 else None
                s["hdop"] = round(msg.eph / 100, 2) if msg.eph != 65535 else None
            elif kind == "EKF_STATUS_REPORT":
                s["ekfFlags"] = msg.flags
                s["ekf"] = {"vel": round(msg.velocity_variance, 2), "posH": round(msg.pos_horiz_variance, 2),
                            "posV": round(msg.pos_vert_variance, 2), "compass": round(msg.compass_variance, 2),
                            "terrain": round(msg.terrain_alt_variance, 2)}
            elif kind == "VIBRATION":
                s["vibe"] = [round(msg.vibration_x, 1), round(msg.vibration_y, 1), round(msg.vibration_z, 1)]
                clip = [msg.clipping_0, msg.clipping_1, msg.clipping_2]
                if drone.clip_prev is not None and sum(clip) > sum(drone.clip_prev):
                    drone.clip_rising_until = now + 5
                drone.clip_prev, s["clip"] = clip, clip
            elif kind == "MISSION_CURRENT":
                s["wpSeq"] = msg.seq
                s["wpTotal"] = msg.total if msg.total != 65535 else s["wpTotal"]
            elif kind == "NAV_CONTROLLER_OUTPUT":
                s["wpDist"], s["altError"], s["xtrack"] = msg.wp_dist, round(msg.alt_error, 2), round(msg.xtrack_error, 2)
            elif kind == "HOME_POSITION":
                s["home"] = {"lat": msg.latitude / 1e7, "lon": msg.longitude / 1e7, "alt": msg.altitude / 1000}
            elif kind == "FENCE_STATUS":
                if msg.breach_status and not (s["fence"] or {}).get("breach"):
                    self.log(sysid, "crit", f"FENCE BREACH (type {msg.breach_type}); ArduPilot fence action engaged")
                s["fence"] = {"breach": msg.breach_status, "count": msg.breach_count, "type": msg.breach_type}
            elif kind == "RADIO_STATUS":
                s["rssi"], s["remRssi"] = msg.rssi, msg.remrssi
            elif kind == "COMMAND_ACK":
                result = mavutil.mavlink.enums["MAV_RESULT"][msg.result].name.replace("MAV_RESULT_", "")
                s["lastAck"] = {"t": now, "command": msg.command, "result": result}
                if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    self.log(sysid, "info" if msg.result == 0 else "warn", f"mode command {result}")
            elif kind == "STATUSTEXT":
                text = msg.text.rstrip("\x00") if isinstance(msg.text, str) else str(msg.text)
                level = "crit" if msg.severity <= 2 else "warn" if msg.severity <= 4 else "info"
                drone.messages.append({"t": now, "severity": msg.severity, "text": text})
                self.log(sysid, level, text)

    def request_streams(self, link, sysid, compid):
        # Ask for ~4 Hz telemetry. Harmless if Mission Planner already set the SRx rates.
        link.mav.request_data_stream_send(sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_ALL, self.args.rate, 1)
        for message_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION, 2),
                               (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
                               (mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, .2),
                               (mavutil.mavlink.MAVLINK_MSG_ID_FENCE_STATUS, 1)):
            link.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       message_id, 1e6 / hz, 0, 0, 0, 0, 0)
        link.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)

    def reader(self, index, url, stop):
        status = self.link_status[index]
        while not stop.is_set():
            try:
                link = mavutil.mavlink_connection(url, source_system=GCS_SYSID, source_component=GCS_COMPID,
                                                  autoreconnect=True)
            except Exception as error:   # serial port busy, TCP refused...
                status["error"] = str(error)
                time.sleep(2)
                continue
            if self.args.log_dir:
                link.setup_logfile(os.path.join(self.args.log_dir,
                                                time.strftime("%Y%m%d_%H%M%S") + f"_link{index + 1}.tlog"), "ab")
            self.links[index] = link
            status["error"], last_beat = None, 0.
            while not stop.is_set():
                now = time.time()
                if now - last_beat > 1:
                    last_beat = now
                    if link.logfile:
                        link.logfile.flush()   # keep the tlog on disk even if the laptop dies mid-flight
                    try:
                        link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                0, 0, 0)
                    except OSError as error:
                        status["error"] = str(error)
                try:
                    msg = link.recv_match(blocking=True, timeout=.5)
                except Exception as error:
                    status["error"] = str(error)
                    time.sleep(.5)
                    continue
                if msg is not None:
                    status["packets"] += 1
                    status["last"] = time.time()
                    self.handle(index, link, msg)
            try:
                link.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ commands
    def command(self, target, action):
        if self.args.read_only:
            raise PermissionError("dashboard started with --read-only")
        if action not in COMMANDS:
            raise ValueError(f"unsupported action {action}; allowed: {', '.join(COMMANDS)}")
        with self.lock:
            drones = list(self.drones.values()) if target == "all" else [self.drones[int(target)]]
        sent = []
        for drone in drones:
            link = self.links[drone.link_index]
            mapping = mavutil.mode_mapping_byname(drone.s.get("mavType", mavutil.mavlink.MAV_TYPE_QUADROTOR))
            if link is None or action not in mapping:
                continue
            for _ in range(2):   # radios drop packets; a repeated mode change is harmless
                link.mav.command_long_send(drone.sysid, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                                           mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[action],
                                           0, 0, 0, 0, 0)
            self.log(drone.sysid, "warn", f"operator command: {action}")
            sent.append(drone.sysid)
        return sent

    # ------------------------------------------------------------------ alerts
    def alerts(self, d, s, now):
        a, out = self.args, []
        age = now - d.last_heartbeat
        if age > a.link_timeout:
            out.append(("link", "crit", f"no heartbeat for {age:.0f} s (link lost)"))
            return out
        armed = s["armed"]
        if s["voltage"]:
            cell = s["voltage"] / a.cells
            if cell < a.cell_crit:
                out.append(("batt", "crit", f"battery {s['voltage']:.2f} V ({cell:.2f} V/cell): land now"))
            elif cell < a.cell_warn:
                out.append(("batt", "warn", f"battery {s['voltage']:.2f} V ({cell:.2f} V/cell) low"))
        if s["remaining"] is not None and s["remaining"] < a.batt_pct:
            out.append(("battpct", "warn", f"battery {s['remaining']}% remaining"))
        if s["gpsFix"] is not None and s["gpsFix"] < 3:
            out.append(("gps", "crit" if armed else "warn", f"no 3D GPS fix (fix type {s['gpsFix']})"))
        elif s["sats"] is not None and s["sats"] < a.min_sats:
            out.append(("sats", "warn", f"only {s['sats']} satellites"))
        if s["hdop"] is not None and s["hdop"] > a.max_hdop:
            out.append(("hdop", "warn", f"HDOP {s['hdop']}"))
        if s["ekf"]:
            worst = max(s["ekf"]["vel"], s["ekf"]["posH"], s["ekf"]["posV"], s["ekf"]["compass"])
            if worst >= .8:
                out.append(("ekf", "crit", f"EKF variance {worst:.2f} (failsafe level)"))
            elif worst >= .5:
                out.append(("ekf", "warn", f"EKF variance {worst:.2f}"))
        if s["ekfFlags"] is not None and s["ekfFlags"] & mavutil.mavlink.ESTIMATOR_GPS_GLITCH:
            out.append(("glitch", "crit", "EKF reports GPS glitch"))
        if s["vibe"]:
            peak = max(s["vibe"])
            if peak > a.vibe_crit:
                out.append(("vibe", "crit", f"vibration {peak:.0f} m/s/s: altitude estimate unreliable"))
            elif peak > a.vibe_warn:
                out.append(("vibe", "warn", f"vibration {peak:.0f} m/s/s"))
        if now < d.clip_rising_until:
            out.append(("clip", "warn", "accelerometer clipping (vibration saturating the IMU)"))
        if (s["fence"] or {}).get("breach"):
            out.append(("fence", "crit", "outside the geofence"))
        if armed and s["throttle"] is not None and s["throttle"] >= a.throttle_warn:
            out.append(("thr", "warn", f"throttle {s['throttle']}%: near thrust limit, ArduPilot will trade altitude"))
        if armed and s["climb"] is not None and s["mode"] in HOLD_MODES and s["climb"] < -a.sink_warn:
            out.append(("sink", "crit", f"descending {-s['climb']:.1f} m/s in {s['mode']} (unexpected sink)"))
        if armed and s["mode"] == "AUTO" and s["altError"] is not None and abs(s["altError"]) > a.alt_dev:
            out.append(("alterr", "warn", f"{s['altError']:+.1f} m from target altitude"))
        if armed and s["altRel"] is not None and s["mode"] not in LANDING_MODES and (s["flightS"] or 0) > 15 \
                and s["altRel"] < a.low_alt:
            out.append(("low", "warn", f"only {s['altRel']:.1f} m above home"))
        if s["unhealthy"]:
            out.append(("sensors", "warn", "unhealthy: " + ", ".join(s["unhealthy"][:4])))
        if s["dropRate"] and s["dropRate"] > 10:
            out.append(("drop", "warn", f"telemetry drop rate {s['dropRate']:.0f}%"))
        if s.get("minSep") is not None and s["minSep"] < a.min_sep:
            out.append(("sep", "crit", f"{s['minSep']:.1f} m from drone {s['minSepTo']}"))
        recent = [m for m in d.messages if m["severity"] <= 3 and now - m["t"] < 20]
        if recent:
            out.append(("text", "crit" if recent[-1]["severity"] <= 2 else "warn", recent[-1]["text"]))
        return out

    def state(self):
        now = time.time()
        with self.lock:
            snaps = {sysid: d.snapshot(now, self.args) for sysid, d in self.drones.items()}
            for sysid, s in snaps.items():
                best, to = None, None
                for other, o in snaps.items():
                    if other != sysid and s["lat"] is not None and o["lat"] is not None and s["armed"] and o["armed"]:
                        gap = math.hypot(distance_m(s["lat"], s["lon"], o["lat"], o["lon"]),
                                         (s["altRel"] or 0) - (o["altRel"] or 0))
                        if best is None or gap < best:
                            best, to = gap, other
                s["minSep"], s["minSepTo"] = (round(best, 1) if best is not None else None), to
            for sysid, s in snaps.items():
                found = self.alerts(self.drones[sysid], s, now)
                s["alerts"] = [{"key": k, "level": lvl, "text": t} for k, lvl, t in found]
                keys = {(sysid, k) for k, _, _ in found}
                for k, lvl, text in found:
                    if self.active.get((sysid, k), (None,))[0] != lvl:
                        self.log(sysid, lvl, text)
                    self.active[(sysid, k)] = (lvl, text, now)
                # An alert clears only after CLEAR_DELAY s without it, so a sagging battery does not flap.
                for key in [key for key in self.active if key[0] == sysid and key not in keys]:
                    lvl, text, seen = self.active[key]
                    if now - seen > CLEAR_DELAY:
                        del self.active[key]
                        self.log(sysid, "ok", f"cleared: {text}")
                    else:
                        s["alerts"].append({"key": key[1], "level": lvl, "text": text})
            links = [dict(l, age=round(now - l["last"], 1) if l["last"] else None) for l in self.link_status]
            return {"t": now, "drones": [snaps[k] for k in sorted(snaps)], "links": links,
                    "events": list(self.events)[-80:], "readOnly": self.args.read_only,
                    "cells": self.args.cells, "commands": list(COMMANDS)}


# --------------------------------------------------------------------------- planned mission overlay
def read_wpl(path):
    points = []
    with open(path) as handle:
        if not handle.readline().startswith("QGC WPL"):
            return points
        for line in handle:
            f = line.split("\t")
            if len(f) == 12 and int(f[0]) > 0 and int(f[3]) in (16, 82) and (float(f[8]) or float(f[9])):
                points.append((float(f[8]), float(f[9]), float(f[10])))
    return points


def load_plan(mission_dir, field_path):
    plan = {"field": None, "fence": None, "paths": []}
    if field_path or mission_dir:
        try:
            from ardupilot_export import read_kml
            if field_path:
                _, ring = read_kml(field_path)
            else:
                kmls = sorted(glob.glob(os.path.join(mission_dir, "*_plan.kml")))
                ring = read_kml(kmls[0], placemark="Field")[1] if kmls else None
            if ring:
                plan["field"] = [(lat, lon) for lon, lat in ring]
        except Exception as error:
            print(f"warning: could not read the field outline: {error}", file=sys.stderr)
    if mission_dir:
        polys = sorted(glob.glob(os.path.join(mission_dir, "*_fence.poly")))
        if polys:
            with open(polys[0]) as handle:
                plan["fence"] = [tuple(map(float, line.split()[:2])) for line in handle
                                 if line.strip() and not line.startswith("#")]
        for path in sorted(glob.glob(os.path.join(mission_dir, "*.waypoints"))):
            name = os.path.basename(path)
            if name.endswith("_fence.waypoints"):
                continue
            match = re.search(r"drone(\d+)", name)
            sysid = int(match.group(1)) if match else 1   # --only files are flown as drone 1
            plan["paths"].append({"sysid": sysid, "file": name, "points": read_wpl(path)})
    return plan


# --------------------------------------------------------------------------- web server
def make_handler(hub, plan):
    files = {"/": ("index.html", "text/html; charset=utf-8")}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, kind="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/api/state":
                return self._send(200, hub.state())
            if path == "/api/plan":
                return self._send(200, plan)
            if path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(b"data: " + json.dumps(hub.state()).encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(1 / hub.args.ui_rate)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return
            name, kind = files.get(path, (path.lstrip("/"), None))
            full = os.path.normpath(os.path.join(STATIC, name))
            if not full.startswith(STATIC) or not os.path.isfile(full):
                return self._send(404, {"error": "not found"})
            if kind is None:
                kind = {".js": "text/javascript", ".css": "text/css", ".png": "image/png"}.get(
                    os.path.splitext(full)[1], "application/octet-stream")
            with open(full, "rb") as handle:
                self._send(200, handle.read(), kind)

        def do_POST(self):
            if self.path != "/api/command":
                return self._send(404, {"error": "not found"})
            if self.headers.get("X-Dashboard") != "1":   # blocks cross-site form posts
                return self._send(403, {"error": "missing header"})
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                sent = hub.command(str(body.get("target")), str(body.get("action")))
                self._send(200, {"sent": sent})
            except PermissionError as error:
                self._send(403, {"error": str(error)})
            except (KeyError, ValueError) as error:
                self._send(400, {"error": str(error)})

    return Handler


def expand_connect(urls, drones):
    if urls == ["sitl"]:
        return [f"tcp:127.0.0.1:{SITL_PORTS + 10 * i}" for i in range(drones)]
    return urls


def parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", nargs="+", default=["udpin:0.0.0.0:14550"],
                   help="MAVLink links (one per radio/forward). udpin:0.0.0.0:14550, tcp:HOST:PORT, COM5,57600, "
                        "or 'sitl' (tcp:127.0.0.1:5760, 5770, ...). Several drones may share one link.")
    p.add_argument("--drones", type=int, default=1, help="number of SITL instances when --connect sitl")
    p.add_argument("--mission", help="exporter output folder: draws field, fence and planned paths")
    p.add_argument("--field", help="KML/KMZ/.poly field outline (if not taken from --mission)")
    p.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to open the page from other laptops/phones")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--read-only", action="store_true", help="hide and refuse the RTL/LOITER/BRAKE/LAND buttons")
    p.add_argument("--log-dir", default=os.path.join(HERE, "logs"), help="tlog + events.csv folder ('' = off)")
    p.add_argument("--rate", type=int, default=4, help="telemetry stream rate requested from each drone (Hz)")
    p.add_argument("--ui-rate", type=float, default=4, help="page refresh rate (Hz)")
    g = p.add_argument_group("alert thresholds")
    g.add_argument("--cells", type=int, default=3, help="battery cells in series (3S = 3)")
    g.add_argument("--cell-warn", type=float, default=3.6, help="V per cell under load: warning")
    g.add_argument("--cell-crit", type=float, default=3.45, help="V per cell under load: critical")
    g.add_argument("--batt-pct", type=int, default=30, help="warn below this battery percentage")
    g.add_argument("--min-sats", type=int, default=8)
    g.add_argument("--max-hdop", type=float, default=2.0)
    g.add_argument("--vibe-warn", type=float, default=30)
    g.add_argument("--vibe-crit", type=float, default=60)
    g.add_argument("--throttle-warn", type=int, default=80, help="throttle %% that signals thrust saturation")
    g.add_argument("--sink-warn", type=float, default=1.5, help="m/s of descent in AUTO/LOITER/ALT_HOLD that alarms")
    g.add_argument("--alt-dev", type=float, default=3, help="m away from the AUTO target altitude")
    g.add_argument("--low-alt", type=float, default=4, help="m above home considered too low in flight")
    g.add_argument("--min-sep", type=float, default=10, help="m between armed drones")
    g.add_argument("--link-timeout", type=float, default=3, help="s without heartbeat = link lost")
    args = p.parse_args(argv)
    args.connect = expand_connect(args.connect, args.drones)
    return args


def main(argv=None, stop=None):
    args = parse(argv)
    hub = Hub(args)
    plan = load_plan(args.mission, args.field)
    stop = stop or threading.Event()
    threading.Thread(target=lambda: (stop.wait(), server.shutdown()), daemon=True).start()
    for index, url in enumerate(args.connect):
        hub.links.append(None)
        hub.link_status.append({"url": url, "packets": 0, "last": None, "error": None})
        threading.Thread(target=hub.reader, args=(index, url, stop), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(hub, plan))
    server.daemon_threads = True
    print(f"Dashboard on http://{'127.0.0.1' if args.host == '0.0.0.0' else args.host}:{args.port}  "
          f"(links: {', '.join(args.connect)}; {'read-only' if args.read_only else 'RTL/LOITER/BRAKE/LAND enabled'})",
          flush=True)
    if plan["paths"] or plan["field"]:
        print(f"Plan overlay: field={'yes' if plan['field'] else 'no'}, fence={'yes' if plan['fence'] else 'no'}, "
              f"{len(plan['paths'])} path(s)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        if hub.event_log:
            hub.event_log.close()


if __name__ == "__main__":
    main()
