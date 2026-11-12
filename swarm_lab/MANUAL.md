# Field-to-flight manual

This manual gives the full procedure for a mission:

1. Draw the field.
2. Make the missions.
3. Examine the missions.
4. Fly the drones.
5. Use RTL to bring each drone back home.

Run the commands from the `swarm_lab/` folder in PowerShell. For details on each option, refer to
[ARDUPILOT.md](ARDUPILOT.md). To monitor all drones during the flight, refer to [DASHBOARD.md](DASHBOARD.md).

```
 1 Draw field (KML)  →  2 Generate missions  →  3 Check preview  →  4 SITL rehearsal
 →  5 Set up drones  →  6 Upload  →  7 Pre-flight  →  8 Fly (AUTO)  →  9 RTL & land  →  10 After flight
```

---

## 0. One-time setup

```powershell
cd swarm_lab
py -m pip install -r requirements.txt
py -m unittest test_ardupilot          # should print OK
```

You also need these items:

- **Mission Planner** (Windows), and **ArduCopter 4.3 or newer** on each drone.
- One telemetry link for each drone (USB, a SiK radio, or Wi-Fi/UDP).
- One **RC transmitter for each drone**, with a safety pilot.
- A camera that points down and that ArduPilot can trigger (servo, relay, or MAVLink camera).

---

## 1. Get the field boundary (KML)

Use **one** of the methods that follow. The planner uses the **largest polygon** in the file.
The planner ignores holes.

### A. Google Earth Pro (desktop, most accurate)

1. Zoom to the field. Click **Add Polygon** (toolbar), or push *Ctrl+Shift+G*.
2. Give the polygon a name (for example, `Search area`). Then click around the boundary.
3. Keep obstacles **outside** the polygon (for example, buildings, power lines and trees).
4. Click **OK**.
5. In *Places*, right-click the polygon and select **Save Place As…**.
6. Save it as `field.kml` (or `.kmz`).

### B. Google Earth (web)

**Projects → New project → Draw line or shape** → close the shape → **⋮ → Export as KML file**.

### C. Google My Maps

**Draw a line → Add line or shape** → close the shape → menu **⋮ → Export to KML/KMZ**.
Select **"Export to a .KML file"**, or keep KMZ. The two formats operate correctly.

### D. Mission Planner (no internet necessary in the field if the maps are in the cache)

**PLAN** tab → right-click the map → **Draw Polygon → Add Polygon Point** around the field
→ right-click → **Draw Polygon → Save polygon** → `field.poly`. Use the `.poly` file
at each location where this manual shows `field.kml`.

### Find the launch (home) point

The `--home` option uses **longitude first, then latitude**:

- **Google Maps:** right-click the take-off point to copy `lat, lon`, for example `28.5466, 77.1930`.
  Then use `--home 77.1930 28.5466`.
- **Mission Planner:** the coordinates below the mouse at the bottom of the PLAN map.

Select a home point that is clear, flat and **outside the field or at its edge**. Each
drone goes back to this point on RTL.

> **WARNING:** Always use `--home` for real flights. If you do not use `--home`, the tool uses the field centroid for item 0 and for the geofence.

---

## 2. Make the missions

### First flights: test mode (one drone)

```powershell
py ardupilot_export.py field.kml --test --home 77.1930 28.5466
```

Then increase to two drones:

```powershell
py ardupilot_export.py field.kml --test --drones 2 --home 77.1930 28.5466
```

`--test` flies the **real field** from your KML with safe settings for a first flight:

- 10 to 30 m altitude.
- A maximum speed of 3 m/s.
- 8 m vertical separation.
- A confirmation prompt before the upload.

To fly only a part of a large field, add `--test-size M`. This option keeps an M-metre square around
home. To do a test without a KML file, use `py ardupilot_export.py --test --home LON LAT`
(an 80 m square at home).

### Boundary fence: RTL if the drone touches the edge of the field

Each run makes an **inclusion fence** from the KML boundary. The fence is `--fence-buffer` (default 5 m) outside the boundary,
and it includes a corridor to home. **If the drone crosses the fence, it goes back home immediately.** The tool does these steps with the fence:

- It writes the fence to `*_fence.waypoints` and `*_fence.poly` (Mission Planner: **PLAN ? Fence ?
  Load from file**). It draws the fence in red in `*_plan.kml`.
- With `--connect`, it uploads the fence and reads it back automatically, together with `FENCE_TYPE = 7`
  (altitude + circle + polygon) and `FENCE_ACTION = 1` (RTL).
- It compares the fence with the planned path. If the path comes within `accept-radius + 3` m of the
  fence, the tool tells you to increase `--fence-buffer`. This prevents a false RTL because of GPS drift.

To disable the fence, use `--no-boundary-fence`. The fence needs MAVLink 2 on the telemetry port
(`SERIALn_PROTOCOL = 2`). If MAVLink 2 is not available, the upload stops and does not overwrite the mission.

### One drone that flies one path (covers only its own area)

Divide the field into K paths, as for the full swarm, and let **one** drone fly only
one of them. The drone makes images of **all of its own area** and ignores the other areas. This is
the most realistic test with one drone, because the drone flies the same path as
in the real swarm.

```powershell
py ardupilot_export.py field.kml --drones 4 --only 2 --test --home 77.1930 28.5466
```

```
Paths in this 4-drone plan (pick one with --only N):
  path 1:   204 m, alt 10-10 m, ~28 photos, 0 transit(s)
  path 2:   270 m, alt 10-13 m, ~38 photos, 0 transit(s)  <== flying
  ...
Path 2: own area 0.99 ha (25% of the field); imaged by this drone alone: 100.0%
```

- `--drones 4` sets the number of paths in the field. `--only 2` selects the path to fly.
- **Path numbers.** The numbers go across the drone line, from one edge of the
  field (path 1) to the opposite edge (path K). Path 1 is near one side wall, and path K is near the other side wall.
  The middle paths go through the centre. To select the centre path, use `--only middle`.
  For an even K, this is the path immediately to the left of the centre, for example path 2 of 4. The
  `*_plan.kml` preview shows the selected path and its yellow area.
- Output: one `TEST_field_path02of04.waypoints` file and its `.param` file. The
  `*_plan.kml` preview shows that path and its own area (yellow).
- The tool makes sure that the drone makes images of **100% of its own area**. If not, the tool gives a warning.
- The tool does not apply an altitude stagger. The drone flies at its own speed and ends with RTL.
- Without `--test`, the drone flies the same path over the full field, with the usual limits.

### Full survey

```powershell
py ardupilot_export.py field.kml --drones 4 --home 77.1930 28.5466 --hfov 147 --vfov 62.22 --max-alt 120
```

To let the tool add drones until the plan is below `--max-alt`, add `--auto-drones`.

### Output files (in `missions_test/` or `missions/`)

| File | Use |
| --- | --- |
| `*_drone01.waypoints`, `*_drone02.waypoints`, … | The mission for each drone. Upload it in step 6 |
| `*_drone01.param`, … | RTL altitude and geofence for each drone. Load it in step 5 |
| `*_plan.kml` | Open it in Google Earth and examine each path in 3D (step 3) |
| `*_summary.json` | Values to record: coverage, altitudes, RTL heights, time, spacing |

Read the console output before you continue:

```
Nadir coverage at flown altitudes: 100.0%  (max altitude 24.9 m, cap 30 m)
Estimated synchronised mission time: 1.0 min; closest 3D approach between drones: 8.2 m
  TEST_field_drone01.waypoints: 20 items, 93 m, alt 10-11 m, ~15 photos, RTL at 30 m, fence 90 m / 40 m
  TEST_field_drone02.waypoints: 25 items, 176 m, alt 18-25 m, ~22 photos, RTL at 38 m, fence 90 m / 48 m
```

- **The coverage must be 100%.** If it is lower, the altitude cap is too low. Add a drone, or
  decrease `--test-size`.
- **WARNING:** Do not fly until you correct the cause of each `warning:` line.

---

## 3. Examine the plan

1. Open `*_plan.kml` in Google Earth.
2. Make sure that each coloured path stays in the field.
3. Make sure that each path is clear of trees, poles and wires at its altitude.
4. Make sure that home is at the location where you will stand.
5. In Mission Planner, select **PLAN → Load WP File**, then open each `.waypoints` file.
6. Make sure that the list:
   - starts with **TAKEOFF**;
   - contains **DO_CHANGE_SPEED** and **DO_SET_CAM_TRIGG_DIST** entries;
   - contains **WAYPOINT** entries at the expected altitudes;
   - ends with **DO_SET_CAM_TRIGG_DIST 0** (camera off), then **RETURN_TO_LAUNCH**.

---

## 4. Do a rehearsal in SITL (strongly recommended)

Start one simulated copter for each drone at your home location. SITL uses latitude first:

```powershell
sim_vehicle.py -v ArduCopter -I0 --sysid 1 -l 28.5466,77.1930,0,0
sim_vehicle.py -v ArduCopter -I1 --sysid 2 -l 28.5466,77.1930,0,0
```

Then run the command that you will use in the field, with `--connect sitl` (TCP ports
5760, 5770, …):

```powershell
py ardupilot_export.py field.kml --test --drones 2 --home 77.1930 28.5466 --connect sitl
```

In Mission Planner, connect to `TCP 127.0.0.1:5760` (and `:5770`). Then do steps 7 to 9
as in the field. Make sure that each drone is **landed and disarmed at home** at the end.

> If SITL is not available, use `py examples/fake_vehicle.py 15760 15770` with
> `--connect tcp:127.0.0.1:15760 tcp:127.0.0.1:15770`. This checks only the upload, not the flight.

---

## 5. Set up each drone (one time for each drone)

Connect each drone to Mission Planner on its own link. Select the COM port and baud rate,
then click **CONNECT**.

1. **Unique ID.** In **CONFIG → Full Parameter List**, set `SYSID_THISMAV` = 1, 2, 3, …
   to agree with the drone number. Write the parameters and reboot.
2. **RTL and geofence.** In **CONFIG → Full Parameter List → Load from file**, select
   `*_droneNN.param` for *that* drone. Then click **Write Params**. This step sets these parameters:

   | Parameter | Value | Reason |
   | --- | --- | --- |
   | `RTL_ALT` | above the highest survey altitude, **different for each drone** | The drones climb to different heights when they go back. Thus, they cannot collide on the way home |
   | `RTL_ALT_FINAL` | 0 | Land at home |
   | `FENCE_ENABLE`, `FENCE_TYPE` 3 | on, altitude + circle | Hard limit around home |
   | `FENCE_RADIUS`, `FENCE_ALT_MAX` | farthest waypoint + 30 m; RTL altitude + 10 m | A breach triggers… |
   | `FENCE_ACTION` | 1 | …**RTL** |

3. **Failsafes.** Set these parameters manually one time. Each parameter makes the drone go back home automatically:

   | Parameter | Set to | Effect |
   | --- | --- | --- |
   | `FS_THR_ENABLE` | 1 | RC signal lost → **RTL** |
   | `BATT_FS_LOW_ACT` | 2 | Low battery → **RTL** (also set `BATT_LOW_VOLT` or `BATT_LOW_MAH`) |
   | `BATT_FS_CRT_ACT` | 1 | Critical battery → **Land** |
   | `FS_GCS_ENABLE` | 1 (optional) | Telemetry lost → **RTL** |
   | `AUTO_OPTIONS` | 3 | Arm in AUTO, and take off without a throttle increase |

4. **Camera.** Set `CAM1_TYPE` for your trigger type (1 servo, 2 relay, 5 MAVLink). Point
   the gimbal straight down (`MNT1_*`, pitch −90°), or attach the camera in a fixed nadir position.
5. **Flight modes.** On the RC, make sure that one switch position is **RTL** and one other position is
   **LOITER** (`FLTMODE1`…`FLTMODE6`, `FLTMODE_CH`).

---

## 6. Upload the missions

**Option A: from this tool (all drones at one time, with read-back):**

```powershell
# Telemetry radios: one connection per drone, in drone order
py ardupilot_export.py field.kml --test --drones 2 --home 77.1930 28.5466 --connect COM5,57600 COM6,57600
```

In test mode, the tool prints a checklist and asks `Upload …? [y/N]`. The tool **reads back each mission and compares it
item by item**. If the items are not the same, the upload stops with an error.

> **CAUTION:** Close the Mission Planner connection to a COM port before the tool uses the port. Only one program at a time
> can open a serial port. As an alternative, use the MAVLink
> forwarding of Mission Planner with `--connect udpin:0.0.0.0:14550`.

**Option B: from Mission Planner (one drone at a time):**

1. Connect to the drone.
2. Select **PLAN → Load WP File** → `*_droneNN.waypoints` → **Write WPs**.
3. To confirm, select **Read WPs**.

> **WARNING:** Make sure that **drone N has mission N**. The missions have different paths and altitudes.

---

## 7. Pre-flight checklist (each drone)

- [ ] Correct `.waypoints` loaded (**Read WPs** shows the expected item count, as in the summary).
- [ ] Correct `.param` loaded (`RTL_ALT` is the same as `rtlAltM` × 100 in the summary).
- [ ] Battery full. Battery failsafe set.
- [ ] GPS 3D fix, HDOP < 1.2, at least 10 satellites. **Home set at the take-off point** (the Mission Planner map shows it).
- [ ] Fence enabled. No pre-arm errors in the HUD/Messages of Mission Planner.
- [ ] Camera triggers: on the ground, **Actions → Trigger Camera NOW** takes a photo.
- [ ] Props clear. One safety pilot for each drone, with a hand on the **mode switch** (RTL/LOITER).
- [ ] A minimum of 5 m between the drones on the ground. Airspace and local permissions are available.

---

## 8. Fly

Do these steps for each drone. Start with **drone 1**. With two or more drones, start them within a few
seconds of each other. The speeds are synchronised, thus the drone line stays together.

1. In Mission Planner, select the drone (vehicle selector at the top right, or its own window).
2. Select **DATA → Actions**. Select **Auto** → **Set Mode**. Then select **Arm/Disarm**.
   - With `AUTO_OPTIONS = 3`, the drone arms in AUTO and takes off immediately.
   - If not, arm in **LOITER** and change to **AUTO**. Then increase the throttle for a short time to start.
3. Monitor the drone. It takes off and flies its path, and the photo count increases.

### Height failsafe: `--RTLheight`

To set a hard ceiling, add `--RTLheight M` to a command. **If a drone climbs above M
metres over home, the drone immediately goes back home (RTL). The cause of the climb is not important (GPS or baro error, a bad command, a flyaway).**

```powershell
py ardupilot_export.py field.kml --drones 4 --only middle --test --home 77.1930 28.5466 --RTLheight 25 --connect COM5,57600
```

- This option sets `FENCE_ALT_MAX = M`, `FENCE_ACTION = 1` (RTL), `FENCE_TYPE = 3` (altitude
  ceiling + circle around home) and `FENCE_ENABLE = 1`.
- The survey altitudes stay below M. The `RTL_ALT` of each drone is between the
  survey and M. Thus, the return flight does not go through the ceiling again.
- The same fence also triggers RTL if the drone goes more than `FENCE_RADIUS` from home
  (the farthest waypoint + 30 m).
- With `--connect`, the tool **writes these parameters to the drone and confirms them** after the
  mission upload. To upload only the mission, use `--no-params`. The `.param` file also contains the values.
- ArduPilot enforces the fence only when the drone is armed with a valid home. Before take-off, examine
  **Fence** on the HUD or in Messages.

> **WARNING:** If you see a problem, set the mode to LOITER (the drone stops in its position) or **RTL** on the RC.
> You can continue a mission with AUTO.

---

## 9. Return to launch (RTL) and landing

Each mission **ends with RETURN_TO_LAUNCH**. Thus, you do not need to do an action after the
last waypoint. On RTL, the drone does these steps:

1. It sets the camera off (the tool sets the trigger distance to 0 immediately before).
2. It climbs to its own `RTL_ALT`. Each drone uses a different height. Thus, drones that go back
   at the same time have a vertical separation.
3. It flies straight home, descends, and **lands at home** (`RTL_ALT_FINAL = 0`).
4. It disarms automatically after the landing.

RTL also starts **automatically** for these events:

| Event | Trigger |
| --- | --- |
| Mission finished | Last mission item |
| Geofence breached: outside the field boundary fence, above `--RTLheight` / `FENCE_ALT_MAX`, or beyond `FENCE_RADIUS` | `FENCE_ACTION = 1` |
| RC link lost | `FS_THR_ENABLE = 1` |
| Battery low | `BATT_FS_LOW_ACT = 2` |
| Telemetry lost (if enabled) | `FS_GCS_ENABLE = 1` |

**Manual RTL**, at any time:

- **RC:** set the mode switch to **RTL**.
- **Mission Planner:** **DATA → Actions → RTL**, or **Set Mode → RTL**.
- **All drones at one time:** select each vehicle and push **RTL**. Each drone climbs to
  its own `RTL_ALT` first, thus the sequence is not important.

> **WARNING:** Keep persons away from home during the landing. A drone can land at the wrong position.
> If a drone does not land at the target, take control in LOITER and land it manually.

---

## 10. After the flight

1. Download the flight logs: **DATA → DataFlash Logs → Download DataFlash Log Via Mavlink**.
2. Geotag the photos: open **Ctrl+F → Geo ref images**, and select the log and the photo folder.
3. Examine the coverage. The footprints of your photos must cover the field, as in the `*_plan.kml` preview.
4. Increase the scale in this sequence: test with 1 drone → test with 2 drones → full field with `--drones N` (without `--test`).

---

## Troubleshooting

| Symptom | Correction |
| --- | --- |
| `No polygon found in the KML` | The KML has only pins or lines. Draw the field again as a **polygon** |
| `Test area … does not overlap the field` | `--home` is far from the field. Make sure that the coordinate sequence is **LON LAT**, or increase `--test-size` |
| `Coverage at capped altitudes: <100%` | Add a drone, increase `--max-alt` within the legal limits, or decrease `--test-size` |
| `warning: drones pass close to each other` | Increase `--alt-stagger`, or launch the drones a few seconds apart |
| `No vehicle heartbeat on …` | The COM port, baud rate or IP is incorrect, or Mission Planner holds the port. Examine the link with Mission Planner first |
| `Vehicle … rejected the mission` | The firmware is too old, or the mission is too large for the board. Update ArduCopter, or decrease `--grid` |
| `Read-back mismatch` | Radio dropout. Do the upload again, or upload from Mission Planner (option B) |
| The drone does not arm in AUTO | Set `AUTO_OPTIONS = 3`, or arm in LOITER and then change to AUTO |
| No photos | Examine `CAM1_TYPE`, the trigger wires, and the settings of the camera (auto-focus, SD card) |
| Incorrect landing point | The drone sets home when you arm it. Arm it at the actual take-off point |
