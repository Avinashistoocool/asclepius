# Swarm telemetry dashboard

`dashboard.py` is a browser dashboard for any number of ArduPilot drones. It shows a live map, one card per
drone, alerts with sound, an event log, altitude and battery charts, and confirmed RTL / LOITER / BRAKE / LAND
buttons. It does not replace Mission Planner. Keep Mission Planner (MP) as the primary GCS, and use the
dashboard as the "whole swarm at a glance" screen.

```
drone 1 ─radio─┐                                     ┌─> Mission Planner (main GCS, arming, AUTO)
drone 2 ─radio─┼─> laptop ─ MP "MAVLink forwarding" ─┤
drone N ─radio─┘                                     └─> UDP 14550 ─> dashboard.py ─> http://127.0.0.1:8090
```

## 1. Start it

```powershell
cd swarm_lab
py -m pip install -r requirements.txt          # pymavlink (already needed by the exporter)
py dashboard.py --connect udpin:0.0.0.0:14550 --mission missions_test
```

Open **http://127.0.0.1:8090**. `--mission` is optional: it points at the exporter output folder, so the field,
the red geofence and each planned path (dashed, in the drone's colour) are drawn under the live tracks.

| Situation | `--connect` |
|---|---|
| Mission Planner already owns the radios (normal case) | `udpin:0.0.0.0:14550` (see section 2) |
| Several MP forwards / MAVProxy outputs | `udpin:0.0.0.0:14550 udpin:0.0.0.0:14551` |
| One telemetry radio, MP **not** using the COM port | `COM5,57600` |
| SITL, N drones | `sitl --drones N` (tcp 5760, 5770, ...) |
| A drone with a companion computer / WiFi bridge | `tcp:192.168.4.1:5760` or `udpin:0.0.0.0:14555` |

Several drones can share one link. Drones are told apart by **`SYSID_THISMAV`**, so every drone **must** have a
unique value (1, 2, 3, ..., matching the exporter's `drone01`, `drone02`, ...). If two drones share an id,
the dashboard warns you.

## 2. Mission Planner forwarding (once per session)

1. Connect every drone in MP as usual (one COM port per radio, or use the multi-link "Connect" dropdown).
2. Press **Ctrl+F**, then click **MAVLink** (sometimes labelled "Mavlink Mirror"). Choose **UDP Client**,
   host `127.0.0.1`, port `14550`, and tick **Write access** if you want the dashboard buttons to work.
3. Do this once for each connected drone (each connection has its own mirror), or forward them all to the same port.

With MAVProxy: `mavproxy.py --master=COM5,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551`
(then point MP at 14551).

The dashboard asks each drone for about 4 Hz telemetry (`--rate`). Over a weak 57 600 baud link with many drones,
keep MP's own rates (CONFIG > Planner > Telemetry rates) at 2 to 4 Hz, or use `--rate 2`.

## 3. What each card shows

| Field | Source | Why it matters |
|---|---|---|
| mode, ARMED/SAFE, flight time | HEARTBEAT | Is the drone doing what you think? |
| alt rel, climb, alt err | GLOBAL_POSITION_INT, VFR_HUD, NAV_CONTROLLER_OUTPUT | Altitude above home; sink rate; distance from the commanded altitude |
| speed, heading, roll/pitch | VFR_HUD, ATTITUDE | Hard manoeuvres (large roll/pitch) cost lift |
| throttle | VFR_HUD | **Above 80% means thrust saturation**: ArduPilot keeps the attitude and gives up altitude |
| battery, per cell, current, used, % | SYS_STATUS, BATTERY_STATUS | Voltage under load; per-cell value uses `--cells` (default 3 for 3S) |
| gps fix/sats, hdop | GPS_RAW_INT | Need 3D fix, ≥ 8 sats, HDOP < 2 |
| ekf var | EKF_STATUS_REPORT | Max variance; 0.8 triggers ArduPilot's EKF failsafe |
| vibe | VIBRATION | > 30 m/s/s degrades altitude hold; > 60 m/s/s is dangerous |
| waypoint, to wp, progress bar | MISSION_CURRENT, NAV_CONTROLLER_OUTPUT | Mission progress |
| home | HOME_POSITION | Distance to home (compare with FENCE_RADIUS) |
| link | heartbeat age | Turns grey and alarms after 3 s of silence |

The bottom-left chart plots the selected drone's altitude and battery voltage over the last 5 minutes.
A falling altitude line that coincides with a voltage dip is the signature of brown-out or thrust saturation.

## 4. Alerts

Critical alerts turn the card red, show in the red banner and beep once. Warnings are amber. Every change is
written to the event log and to `logs/<time>_events.csv`.

| Alert | Default | Option |
|---|---|---|
| Battery per cell warn / critical | 3.60 / 3.45 V | `--cell-warn`, `--cell-crit`, `--cells` |
| Battery % | < 30% | `--batt-pct` |
| GPS | no 3D fix (critical when armed), < 8 sats, HDOP > 2 | `--min-sats`, `--max-hdop` |
| EKF | variance ≥ 0.5 warn, ≥ 0.8 critical, GPS glitch flag | |
| Vibration / clipping | > 30 warn, > 60 critical, clip counters rising | `--vibe-warn`, `--vibe-crit` |
| Throttle saturation | ≥ 80% while armed | `--throttle-warn` |
| Unexpected sink | descending > 1.5 m/s in AUTO/GUIDED/LOITER/ALT_HOLD/POSHOLD | `--sink-warn` |
| Altitude error in AUTO | > 3 m from target | `--alt-dev` |
| Too low | < 4 m above home after 15 s armed (not in RTL/LAND) | `--low-alt` |
| Fence breach | FENCE_STATUS | |
| Drones too close | < 10 m (3D) between armed drones | `--min-sep` |
| Link lost | no heartbeat for 3 s | `--link-timeout` |
| Unhealthy sensors, drop rate > 10%, autopilot STATUSTEXT severity ≤ ERROR | | |

For a 3S pack: 3.60 V/cell = 10.8 V and 3.45 V/cell = 10.35 V under load. Match these to your `BATT_LOW_VOLT` /
`BATT_CRT_VOLT` so the dashboard warns you just before ArduPilot acts.

The dashboard **only observes and alerts**. The real protection is still the on-board failsafes (fence, RTL_ALT,
battery, radio and GCS failsafes), because they work even when the laptop or the link dies. The dashboard uses
GCS system id 254, so it does **not** keep a GCS failsafe from triggering when Mission Planner (255) is gone.

## 5. Buttons

Each card has **RTL, LOITER, BRAKE, LAND**, and the header has **LOITER ALL / RTL ALL**. Every button asks for
confirmation and sends `MAV_CMD_DO_SET_MODE` twice (radios drop packets). Arm, takeoff and AUTO are
deliberately not offered: start missions from Mission Planner. Use `--read-only` to hide the buttons. If
MP's forwarding does not have write access, the commands silently go nowhere.

The page binds to `127.0.0.1` by default. `--host 0.0.0.0` lets a teammate's laptop or phone on the same WiFi
watch it, but it also lets them press RTL. Combine `--host 0.0.0.0` with `--read-only` unless you trust the network.

## 6. Logs

* `logs/<time>_linkN.tlog`: every MAVLink packet. Open it in MP (**Telemetry Logs > Tlog > Playback** or
  **Graph log**) to replay a flight.
* `logs/<time>_events.csv`: alerts, mode changes, arming, STATUSTEXT and operator commands.
* The on-board **DataFlash .bin** (MP: **DataFlash Logs > Download**) is still the best for diagnosing crashes:
  CTUN (Alt, DAlt, ThO), VIBE, BAT, RATE, ATT, MOTB (ThLimit), PM, ERR, EV.

## 7. Try it without drones

```powershell
py examples\fake_vehicle.py 5760 5770 --telemetry          # two fake copters flying a circle in AUTO
py ardupilot_export.py examples\field.kml --drones 2 --connect sitl --yes   # optional: they then fly the plan
py dashboard.py --connect sitl --drones 2 --mission missions
py -m unittest test_dashboard
```

Real SITL works the same way: `sim_vehicle.py -v ArduCopter -I0 --sysid 1` and `-I1 --sysid 2`, then
`py dashboard.py --connect sitl --drones 2`.

## 8. Ready-made alternatives

* **Mission Planner**: *Ctrl+F > Swarm* shows all connected vehicles on one map; the HUD shows one drone at a time.
* **QGroundControl** has a multi-vehicle list with a per-vehicle selector.
* **MAVProxy** with `module load map console` and `vehicle N` to switch.
* **Grafana + InfluxDB** (via mavlink2rest or a pymavlink bridge) for long-term dashboards.

This dashboard exists because these GCS tools focus on one vehicle at a time, have no swarm-wide alerts
tailored to this mission (throttle saturation, sink in AUTO, separation), and do not overlay the exporter's
per-drone plan.
