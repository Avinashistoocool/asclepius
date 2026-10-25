# Flying the plan with ArduPilot and Mission Planner

`swarm_lab/ardupilot_export.py` converts a coverage plan into one ArduCopter mission per
drone. You can load the missions in Mission Planner or upload them directly over MAVLink.

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

- **Google Earth:** *Add → Polygon*, trace the search area, then *Save Place As → .kml/.kmz*.
- **Mission Planner:** in *Flight Plan*, right-click → *Draw Polygon*, then *Save Polygon*
  (`.poly`).

If the file contains several polygons, the largest is used. To choose one by name, use
`--placemark "Name"`. Holes (inner boundaries) are ignored.

## 3. Generate missions

```powershell
py ardupilot_export.py field.kml --drones 6 --hfov 147 --vfov 62.22 --max-alt 120 --out missions
```

Output in `missions/`:

| File | Use |
| --- | --- |
| `field_drone01.waypoints` … | One mission per drone (`QGC WPL 110`). Load it in Mission Planner with *Flight Plan → Load WP File* |
| `field_plan.kml` | Open in Google Earth to check every path in 3D before flying |
| `field_summary.json` | Required altitude, coverage at the flown altitudes, mission time, closest approach between drones, and per-drone length and photo count |

### What each mission contains

| # | Command | Notes |
| --- | --- | --- |
| 0 | Home | Replaced by the autopilot with the actual home position |
| 1 | `NAV_TAKEOFF` | To the first survey altitude |
| | `DO_CHANGE_SPEED` | Max `--speed`. With synchronisation on (the default), each leg's speed is set so that every drone reaches the same front at the same time, which keeps the formation line together |
| | `DO_SET_CAM_TRIGG_DIST` | Photo spacing = `2·h·tan(VFOV/2)·(1 − front overlap)`. It is updated whenever the altitude changes, and set to 0 (camera off) on transit hops and before RTL |
| | `NAV_WAYPOINT` | Simplified path (within `--path-tolerance` m), with altitude **relative to home**. Each kept waypoint flies at the highest altitude required by any sample it replaces, so simplification never reduces coverage |
| last | `NAV_RETURN_TO_LAUNCH` | |

### Altitudes, coverage and safety

- The planner chooses the lowest altitude that still images every cell of the drone's
  area (see the report). After converting to metres, the exporter:
  1. clips altitudes to `[--min-alt, --max-alt]`;
  2. adds `drone index × --alt-stagger` (default 5 m) so that converging drones are
     vertically separated. Flying higher never reduces coverage;
  3. **re-checks nadir coverage at the final altitudes** and prints it.
- If the field needs more than `--max-alt`, the exporter warns and reports the coverage that
  is actually achieved. Add drones, or use `--auto-drones` to add drones automatically
  until the plan fits under the cap.
- The closest 3D approach between any two drones is reported, assuming they fly in sync.
  If it falls below `--min-sep` (default 5 m), increase `--alt-stagger`.
- Altitudes are **AGL relative to home**. For uneven terrain, enable terrain following in
  ArduPilot (`TERRAIN_ENABLE=1`, and change waypoint frames to terrain) or add margin.

### Useful options

| Option | Default | Meaning |
| --- | --- | --- |
| `--planner` | `harmonic` | `harmonic` gives the lowest altitude; `lawnmower` and `geodesic` balance workload best; `mat` |
| `--hfov`, `--vfov` | 147, 62.22 | Camera across-track and along-track fields of view (degrees) |
| `--overlap` | 0.1 | Side-overlap margin on altitude |
| `--front-overlap` | 0.3 | Forward overlap between consecutive photos |
| `--speed`, `--min-speed` | 8, 1.5 m/s | Ground-speed limits |
| `--no-sync` | | Every drone flies at `--speed`. Missions are shorter, but the formation line is not kept |
| `--accept-radius` | 2 m | `WPNAV` acceptance radius per waypoint |
| `--home LON LAT` | centroid | Coordinates written into item 0 |

## 4. Upload to the drones (pymavlink)

Give one MAVLink connection per drone, in drone order:

```powershell
# Telemetry radios
py ardupilot_export.py field.kml --drones 3 --connect COM5,57600 COM6,57600 COM7,57600

# SITL (each instance on its own TCP port)
py ardupilot_export.py field.kml --drones 3 --connect tcp:127.0.0.1:5760 tcp:127.0.0.1:5770 tcp:127.0.0.1:5780

# Mission Planner MAVLink forwarding / companion computers
py ardupilot_export.py field.kml --drones 2 --connect udpin:0.0.0.0:14550 udpin:0.0.0.0:14551
```

Each mission is sent with the MAVLink mission protocol (`MISSION_COUNT` →
`MISSION_REQUEST_INT` → `MISSION_ITEM_INT` → `MISSION_ACK`), then **read back and compared
item by item**. Give each drone a unique `SYSID_THISMAV` so that Mission Planner can show
all of them together.

To test without hardware, run the fake vehicles that implement the mission protocol:

```powershell
py examples/fake_vehicle.py 15760 15770
py ardupilot_export.py examples/field.kml --drones 2 --connect tcp:127.0.0.1:15760 tcp:127.0.0.1:15770
```

## 5. Fly

1. In Mission Planner, connect to each drone (*Ctrl+L* or the connection dropdown) and
   check its mission on *Flight Plan → Read*.
2. Pre-arm checks: GPS lock, fence, battery failsafe, `RTL_ALT` above the highest mission
   altitude plus the stagger.
3. Camera: set `CAM1_TYPE` (servo/relay/MAVLink) so that `DO_SET_CAM_TRIGG_DIST`
   triggers the shutter. Point the gimbal at nadir (`MNT1_*`, or a `DO_MOUNT_CONTROL`
   pitch of −90).
4. Arm the drones and switch them to **AUTO** together (e.g. from the Mission Planner
   *Swarm* screen, or with a script) so that the synchronised speeds keep the formation line.

> **Safety.** Always review `field_plan.kml` and the summary first, and keep a pilot on
> each RC link. Simulate first in SITL with the real parameter files. The planner checks
> nadir coverage and drone-to-drone spacing geometrically. It does **not** model wind,
> GPS error, obstacles above ground, or airspace rules.

## Tests

```powershell
py -m unittest test_ardupilot -v
```

The tests cover the KML/projection round trip, the `.waypoints` structure (home, takeoff,
camera on and off, RTL), altitude limits, the trigger distance against the footprint size,
and 100% coverage at the flown altitudes.
