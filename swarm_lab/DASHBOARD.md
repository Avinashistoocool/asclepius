# Swarm telemetry dashboard

`dashboard.py` is a browser dashboard for ArduPilot drones, with no limit on the number of drones. It shows these items:

- A live map and one card for each drone.
- Alerts with sound, and an event log.
- Charts of altitude and battery.
- RTL / LOITER / BRAKE / LAND buttons with confirmation.

The dashboard does not replace Mission Planner. Keep Mission Planner (MP) as the primary GCS. Use the
dashboard as the screen that shows the full swarm at one time.

```
drone 1 ─radio─┐                                     ┌─> Mission Planner (main GCS, arming, AUTO)
drone 2 ─radio─┼─> laptop ─ MP "MAVLink forwarding" ─┤
drone N ─radio─┘                                     └─> UDP 14550 ─> dashboard.py ─> http://127.0.0.1:8090
```

## 1. Start the dashboard

```powershell
cd swarm_lab
py -m pip install -r requirements.txt          # pymavlink (already needed by the exporter)
py dashboard.py --connect udpin:0.0.0.0:14550 --mission missions_test
```

Open **http://127.0.0.1:8090**. `--mission` is optional. It points to the output folder of the exporter. With it, the dashboard draws the field,
the red geofence and each planned path (dashed, in the colour of the drone) below the live tracks.

| Situation | `--connect` |
|---|---|
| Mission Planner already uses the radios (usual case) | `udpin:0.0.0.0:14550` (refer to section 2) |
| Two or more MP forwards / MAVProxy outputs | `udpin:0.0.0.0:14550 udpin:0.0.0.0:14551` |
| One telemetry radio, MP does **not** use the COM port | `COM5,57600` |
| SITL, N drones | `sitl --drones N` (tcp 5760, 5770, ...) |
| A drone with a companion computer / WiFi bridge | `tcp:192.168.4.1:5760` or `udpin:0.0.0.0:14555` |

Two or more drones can use the same link. The dashboard identifies each drone by its **`SYSID_THISMAV`**. Thus, each drone **must** have a
unique value (1, 2, 3, ..., the same as `drone01`, `drone02`, ... from the exporter). If two drones have the same id,
the dashboard gives a warning.

## 2. Mission Planner forwarding (one time for each session)

1. Connect each drone in MP as usual (one COM port for each radio, or use the multi-link "Connect" dropdown).
2. Push **Ctrl+F**, then click **MAVLink** (the label is sometimes "Mavlink Mirror"). Select **UDP Client**,
   host `127.0.0.1`, port `14550`.
3. If you want the dashboard buttons to operate, select **Write access**.
4. Do these steps one time for each connected drone (each connection has its own mirror). As an alternative, forward all of them to the same port.

With MAVProxy: `mavproxy.py --master=COM5,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551`
(then set MP to 14551).

The dashboard asks each drone for telemetry at about 4 Hz (`--rate`). On a weak 57 600 baud link with many drones,
keep the rates of MP (CONFIG > Planner > Telemetry rates) at 2 to 4 Hz, or use `--rate 2`.

## 3. Contents of each card

| Field | Source | Why it is important |
|---|---|---|
| mode, ARMED/SAFE, flight time | HEARTBEAT | Shows if the drone does the task that you expect |
| alt rel, climb, alt err | GLOBAL_POSITION_INT, VFR_HUD, NAV_CONTROLLER_OUTPUT | Altitude above home; sink rate; distance from the commanded altitude |
| speed, heading, roll/pitch | VFR_HUD, ATTITUDE | Sudden manoeuvres (large roll/pitch) decrease lift |
| throttle | VFR_HUD | **A value above 80% shows thrust saturation**: ArduPilot keeps the attitude and lets the altitude decrease |
| battery, per cell, current, used, % | SYS_STATUS, BATTERY_STATUS | Voltage under load; the value for each cell uses `--cells` (default 3 for 3S) |
| gps fix/sats, hdop | GPS_RAW_INT | Necessary: 3D fix, ≥ 8 sats, HDOP < 2 |
| ekf var | EKF_STATUS_REPORT | Max variance; 0.8 triggers the EKF failsafe of ArduPilot |
| vibe | VIBRATION | > 30 m/s/s makes altitude hold worse; > 60 m/s/s is dangerous |
| waypoint, to wp, progress bar | MISSION_CURRENT, NAV_CONTROLLER_OUTPUT | Mission progress |
| home | HOME_POSITION | Distance to home (compare with FENCE_RADIUS) |
| link | heartbeat age | Becomes grey and gives an alarm after 3 s of silence |

The chart at the bottom left shows the altitude and battery voltage of the selected drone for the last 5 minutes.
If the altitude line decreases at the same time as the voltage, this is a sign of brown-out or thrust saturation.

## 4. Alerts

A critical alert makes the card red, shows in the red banner and gives one beep. Warnings are amber. The dashboard writes each change
to the event log and to `logs/<time>_events.csv`.

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

For a 3S pack: 3.60 V/cell = 10.8 V and 3.45 V/cell = 10.35 V under load. Set these values to agree with your `BATT_LOW_VOLT` /
`BATT_CRT_VOLT`. Then the dashboard gives a warning a short time before ArduPilot does an action.

> **CAUTION:** Do not use the dashboard as the primary protection. It **only monitors and gives alerts**.
> The real protection is the on-board failsafes (fence, RTL_ALT, battery, radio and GCS failsafes). These failsafes operate also when the laptop or the link stops.
> The dashboard uses GCS system id 254. Thus, it does **not** prevent a GCS failsafe when Mission Planner (255) is not available.

## 5. Buttons

Each card has **RTL, LOITER, BRAKE, LAND**. The header has **LOITER ALL / RTL ALL**. Each button asks for
confirmation and sends `MAV_CMD_DO_SET_MODE` two times (radios can lose packets). The dashboard intentionally does not have buttons for arm, takeoff and AUTO.
Start the missions from Mission Planner. To hide the buttons, use `--read-only`. If
the forwarding of MP does not have write access, the commands go nowhere and you get no error.

The page binds to `127.0.0.1` by default. With `--host 0.0.0.0`, a laptop or phone of a team member on the same WiFi
can see the page, but the team member can also push RTL.

> **WARNING:** Use `--host 0.0.0.0` together with `--read-only`, unless you trust the network. Other persons on the network can send RTL commands to the drones.

## 6. Logs

* `logs/<time>_linkN.tlog`: all MAVLink packets. To replay a flight, open this file in MP (**Telemetry Logs > Tlog > Playback** or
  **Graph log**).
* `logs/<time>_events.csv`: alerts, mode changes, arming, STATUSTEXT and operator commands.
* The on-board **DataFlash .bin** (MP: **DataFlash Logs > Download**) is still the best log to find the cause of a crash:
  CTUN (Alt, DAlt, ThO), VIBE, BAT, RATE, ATT, MOTB (ThLimit), PM, ERR, EV.

## 7. Test the dashboard without drones

```powershell
py examples\fake_vehicle.py 5760 5770 --telemetry          # two fake copters flying a circle in AUTO
py ardupilot_export.py examples\field.kml --drones 2 --connect sitl --yes   # optional: they then fly the plan
py dashboard.py --connect sitl --drones 2 --mission missions
py -m unittest test_dashboard
```

Real SITL operates in the same way: `sim_vehicle.py -v ArduCopter -I0 --sysid 1` and `-I1 --sysid 2`, then
`py dashboard.py --connect sitl --drones 2`.

## 8. Alternative tools

* **Mission Planner**: *Ctrl+F > Swarm* shows all connected vehicles on one map. The HUD shows one drone at a time.
* **QGroundControl** has a list of vehicles with a selector for each vehicle.
* **MAVProxy** with `module load map console`, and `vehicle N` to change the vehicle.
* **Grafana + InfluxDB** (through mavlink2rest or a pymavlink bridge) for long-term dashboards.

We made this dashboard for these reasons:

- These GCS tools show one vehicle at a time.
- They do not have alerts for the full swarm that are specific to this mission (throttle saturation, sink in AUTO, separation).
- They do not show the plan of each drone from the exporter.
