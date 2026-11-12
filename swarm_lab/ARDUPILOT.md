# Fly the plan with ArduPilot and Mission Planner

> If you are a new user, start with the step-by-step [MANUAL.md](MANUAL.md). This page is the reference for the options.

`swarm_lab/ardupilot_export.py` changes a coverage plan into one ArduCopter mission for each
drone. You can load the missions in Mission Planner, or you can upload them directly through MAVLink.

```
KML/KMZ/.poly  ──►  planner (harmonic / lawnmower / geodesic / MAT)  ──►  per-drone
field boundary      lanes, paths, headings, nadir altitudes              .waypoints files
                                                                         + KML preview
                                                                         + summary.json
                                                                         + optional MAVLink upload
```

## 1. Install

```powershell
cd swarm_lab
py -m pip install -r requirements.txt     # includes pymavlink
```

## 2. Draw the field

- **Google Earth:** select *Add → Polygon* and draw the search area. Then select *Save Place As → .kml/.kmz*.
- **Mission Planner:** in *Flight Plan*, right-click → *Draw Polygon*. Then select *Save Polygon*
  (`.poly`).

If the file contains two or more polygons, the tool uses the largest. To select one by name, use
`--placemark "Name"`. The tool ignores holes (inner boundaries).

## 3. Make the missions

### Test mode (start here)

For the first flights with one or two drones, add `--test`:

```powershell
# One drone over an 80 m square centred on where you stand (no field file needed)
py ardupilot_export.py --test --home 77.1930 28.5466

# One or two drones over the real field, clipped to an 80 m square around its centre
py ardupilot_export.py field.kml --test
py ardupilot_export.py field.kml --test --drones 2

# Test in SITL (expands to tcp:127.0.0.1:5760, :5770, ...)
py ardupilot_export.py field.kml --test --drones 2 --connect sitl

# Real drone over its telemetry radio
py ardupilot_export.py field.kml --test --connect COM5,57600
```

`--test` changes the default values to safe values for a first flight. You can override each of them,
for example with `--max-alt 40`:

| Setting | Normal | `--test` |
| --- | --- | --- |
| Drones | 4 | **1** (only 1 or 2 allowed) |
| Max / min altitude | 120 / 15 m | **30 / 10 m** |
| Max speed | 8 m/s | **3 m/s** |
| Altitude stagger between drones | 5 m | **8 m** |
| Separation warning | 5 m | **8 m** |
| Area | whole field | **whole field** (optional `--test-size M` square around `--home`) |
| Output | `missions/` | **`missions_test/`**, files prefixed `TEST_` |
| Upload | immediate | **checklist and a `y/N` prompt** (skip it with `--yes`) |

**Fly one path of the swarm with one drone:** `--drones K --only N` makes a plan with K paths, but exports
only path N. The drone covers all of its own area (the tool checks this) and ignores the other areas:

```powershell
py ardupilot_export.py field.kml --drones 4 --only 2 --test --home 77.1930 28.5466
```

Recommended sequence:

1. `--test` in SITL.
2. `--test` with one real drone, and a pilot ready on the RC. Then use `--drones K --only N`
   to fly one real path of the swarm.
3. `--test --drones 2`.
4. A full field without `--test`.

### Full missions

```powershell
py ardupilot_export.py field.kml --drones 6 --hfov 147 --vfov 62.22 --max-alt 120 --out missions
```

Output in `missions/`:

| File | Use |
| --- | --- |
| `field_drone01.waypoints` … | One mission for each drone (`QGC WPL 110`). To load it in Mission Planner, use *Flight Plan → Load WP File* |
| `field_plan.kml` | Open it in Google Earth to examine each path in 3D before the flight |
| `field_fence.waypoints`, `field_fence.poly` | Boundary inclusion fence (KML edge + `--fence-buffer`). Breach ? RTL |
| `field_drone01.param` ? | Recommended `RTL_ALT` (different for each drone), `RTL_ALT_FINAL`, and geofence (`FENCE_*`, action RTL) |
| `field_summary.json` | Necessary altitude, coverage at the flight altitudes, mission time, closest approach between drones, and the length and photo count of each drone |

### Contents of each mission

| # | Command | Notes |
| --- | --- | --- |
| 0 | Home | The autopilot replaces it with the actual home position |
| 1 | `NAV_TAKEOFF` | To the first survey altitude |
| | `DO_CHANGE_SPEED` | Max `--speed`. Synchronisation is on by default. Then the tool sets the speed of each leg so that all drones reach the same front at the same time. This keeps the formation line together |
| | `DO_SET_CAM_TRIGG_DIST` | Photo spacing = `2·h·tan(VFOV/2)·(1 − front overlap)`. The tool updates it each time the altitude changes. It sets it to 0 (camera off) on transit hops and before RTL |
| | `NAV_WAYPOINT` | Simplified path (within `--path-tolerance` m), with altitude **relative to home**. Each waypoint that stays flies at the highest altitude that a replaced sample needs. Thus, the simplification never decreases coverage |
| last | `NAV_RETURN_TO_LAUNCH` | |

### Altitudes, coverage and safety

- The planner selects the lowest altitude that still makes an image of each cell of the area of the drone
  (refer to the report). After the conversion to metres, the exporter does these steps:
  1. It clips the altitudes to `[--min-alt, --max-alt]`.
  2. It adds `drone index × --alt-stagger` (default 5 m), so that drones that come together have a
     vertical separation. A higher flight never decreases coverage.
  3. It **checks the nadir coverage again at the final altitudes** and prints it.
- If the field needs more than `--max-alt`, the exporter gives a warning and shows the coverage that
  it gets. Add drones, or use `--auto-drones`. This option adds drones automatically
  until the plan is below the cap.
- The tool shows the closest 3D approach between two drones, with the assumption that they fly in sync.
  If it is less than `--min-sep` (default 5 m), increase `--alt-stagger`.
- Altitudes are **AGL relative to home**. For terrain that is not level, enable terrain following in
  ArduPilot (`TERRAIN_ENABLE=1`, and change the waypoint frames to terrain), or add a margin.

### Useful options

| Option | Default | Meaning |
| --- | --- | --- |
| `--planner` | `harmonic` | `harmonic` gives the lowest altitude; `lawnmower` and `geodesic` give the best balance of work; `mat` |
| `--hfov`, `--vfov` | 147, 62.22 | Camera across-track and along-track fields of view (degrees) |
| `--overlap` | 0.1 | Side-overlap margin on altitude |
| `--front-overlap` | 0.3 | Forward overlap between adjacent photos |
| `--speed`, `--min-speed` | 8, 1.5 m/s | Ground-speed limits |
| `--no-sync` | | All drones fly at `--speed`. The missions are shorter, but the formation line does not stay together |
| `--accept-radius` | 2 m | `WPNAV` acceptance radius for each waypoint |
| `--home LON LAT` | centroid | Launch point: item 0, fence centre, test square |
| `--RTLheight M` | off | **Height failsafe:** sets `FENCE_ALT_MAX=M` with action RTL. Thus, a drone above M m goes back home. Survey and RTL altitudes stay below M. `--connect` sends the value to the drone |
| `--only N` / `--only middle` | | Fly only path N of a `--drones K` plan, with one drone. The numbers of the paths go from one edge of the drone line to the other |
| `--smooth spline\|straight` | spline | `spline`: survey legs use `NAV_SPLINE_WAYPOINT`, thus the copter flies one smooth curve through them. `straight`: plain waypoints |
| `--min-spacing M` | 5 m | Minimum distance between survey waypoints. Points that are closer cause kinks in the splines and sudden movements of the copter |
| `--fence-buffer M` | 5 m | The boundary inclusion fence is M metres outside the KML edge. If the drone crosses it, RTL starts |
| `--no-boundary-fence` | | Do not make or upload the boundary fence |
| `--test-size M` | off | Fly only an M-metre square of the field around `--home` |
| `--no-params` | | With `--connect`, upload the mission without the RTL/fence parameters |
| `--rtl-alt` | survey max + `--rtl-margin` | RTL altitude for drone 1. Each subsequent drone adds `--alt-stagger` |
| `--rtl-margin`, `--fence-margin` | 5 m, 30 m | Added height above the survey for RTL; added distance beyond the farthest waypoint for the fence |

## 4. Upload to the drones (pymavlink)

Give one MAVLink connection for each drone, in the sequence of the drones:

```powershell
# Telemetry radios
py ardupilot_export.py field.kml --drones 3 --connect COM5,57600 COM6,57600 COM7,57600

# SITL (each instance on its own TCP port; or just: --connect sitl)
py ardupilot_export.py field.kml --drones 3 --connect tcp:127.0.0.1:5760 tcp:127.0.0.1:5770 tcp:127.0.0.1:5780

# Mission Planner MAVLink forwarding / companion computers
py ardupilot_export.py field.kml --drones 2 --connect udpin:0.0.0.0:14550 udpin:0.0.0.0:14551
```

Then the tool writes the RTL and geofence parameters of each drone (`RTL_ALT`, `FENCE_*`, including `--RTLheight`)
with `PARAM_SET`, and the drone confirms each value. The tool sends each mission
with the MAVLink mission protocol (`MISSION_COUNT` →
`MISSION_REQUEST_INT` → `MISSION_ITEM_INT` → `MISSION_ACK`). Then it **reads the mission back and compares it
item by item**. Give each drone a unique `SYSID_THISMAV`, so that Mission Planner can show
all of them together.

To do a test without hardware, run the fake vehicles that use the mission protocol:

```powershell
py examples/fake_vehicle.py 15760 15770
py ardupilot_export.py examples/field.kml --drones 2 --connect tcp:127.0.0.1:15760 tcp:127.0.0.1:15770
```

## 5. Fly

1. In Mission Planner, connect to each drone (*Ctrl+L* or the connection dropdown).
   Examine its mission on *Flight Plan → Read*.
2. Do the pre-arm checks: GPS lock, fence, battery failsafe, and `RTL_ALT` above the highest mission
   altitude plus the stagger.
3. Camera: set `CAM1_TYPE` (servo/relay/MAVLink), so that `DO_SET_CAM_TRIGG_DIST`
   triggers the shutter. Point the gimbal at nadir (`MNT1_*`, or a `DO_MOUNT_CONTROL`
   pitch of −90).
4. Arm the drones and set them to **AUTO** at the same time (for example, from the Mission Planner
   *Swarm* screen, or with a script). Then the synchronised speeds keep the formation line.

> **WARNING:** Examine `field_plan.kml` and the summary before each flight. Keep a pilot on
> each RC link. Do a simulation in SITL first, with the real parameter files. The planner checks
> nadir coverage and the distance between drones geometrically only. It does **not** model wind,
> GPS error, obstacles above ground, or airspace rules.

## Tests

```powershell
py -m unittest test_ardupilot -v
```

The tests examine these items:

- The KML/projection round trip.
- The `.waypoints` structure (home, takeoff, camera on and off, RTL).
- The altitude limits.
- The trigger distance, compared with the size of the footprint.
- 100% coverage at the flight altitudes.
