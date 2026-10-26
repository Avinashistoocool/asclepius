"""Minimal fake ArduCopter vehicles: MAVLink mission, fence and parameter protocols.

Use it to test mission uploads without hardware or SITL. Each vehicle is a TCP server, like ArduPilot SITL (tcp:127.0.0.1:5760):

    py examples/fake_vehicle.py 14550 14560          # two vehicles, system ids 1 and 2
    py ardupilot_export.py examples/field.kml --drones 2 \
        --connect tcp:127.0.0.1:14550 tcp:127.0.0.1:14560
"""

import os
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")  # like ArduPilot: needed for fence (mission_type) messages

from pymavlink import mavutil


def vehicle(port, sysid, stop):
    link = mavutil.mavlink_connection(f"tcpin:127.0.0.1:{port}", source_system=sysid, source_component=1)
    stores, expected, params, last_beat = {0: {}, 1: {}}, {0: 0, 1: 0}, {}, 0.
    names = {0: "mission", 1: "fence"}
    while not stop.is_set():
        if time.time() - last_beat > .5:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
            last_beat = time.time()
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


if __name__ == "__main__":
    stop = threading.Event()
    for index, port in enumerate(int(p) for p in sys.argv[1:]):
        threading.Thread(target=vehicle, args=(port, index + 1, stop), daemon=True).start()
    print(f"Fake vehicles listening on TCP ports {' '.join(sys.argv[1:])}. Ctrl+C to stop.", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()
