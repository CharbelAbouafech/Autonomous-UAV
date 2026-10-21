# Autonomous UAV

Software for our entry in the C-UASC competition. A Raspberry Pi 5 companion computer talks to a Pixhawk running PX4 and flies three missions with no human input after launch: waypoint navigation, a circuit time trial and object localization. It's Python (asyncio) on top of MAVSDK, with an Ultralytics YOLO model for the camera.

## Missions

### Waypoint navigation

- Up to 7 GPS waypoints, given as an unordered list on competition day.
- Before flying, the route is optimized. We try every ordering (7! = 5,040 of them), measured from the drone's current GPS position, and keep the shortest. Set `optimize_route` to `false` to fly the order in the config.
- The mission is uploaded to PX4 before arming. PX4 then flies it in mission mode and handles takeoff, position control and waypoint acceptance.
- After the last waypoint the drone returns to launch (`rtl_after`, on by default).
- Time limit: 540 s in the config, under the 10-minute competition limit. If it runs out, the drone returns to launch.
- Scored by a judge-provided GPS data logger.
- If the drone breaches the inner geofence, it flies back to the last waypoint it completed, skips the waypoint it was heading for, and carries on with the next one.

### Circuit time trial

- Uses the same unordered waypoint list (2 to 7 waypoints) for one timed run as fast as possible: 5 m/s, every waypoint set to fly-through, 4 m acceptance radius. The competition requires passing within 4 m of each waypoint.
- The route is optimized the same way as in waypoint navigation. The log prints the chosen start and end waypoints so they can be reported to the judges before the flight.
- The timer starts when PX4 reports the first waypoint reached and stops at the last. Checkpoint and leg times are saved in the result file.
- The drone lands after the last waypoint.

### Object localization

- The search area comes from 3 to 4 large ground control points (about 1.2 m) given as GPS corners, plus a 3 m margin. The targets are smaller (about 0.6 m) survey-style markers with black numbers on white.
- The drone flies to the starting corner, then flies a lawnmower pattern in offboard mode using NED velocity commands: 1.5 m/s with legs 2 m apart.
- A second asyncio task grabs camera frames and runs YOLO on every 5th one (confidence 0.3). Each detection is tagged with the drone's GPS position and altitude. `ObjectManager` merges repeated sightings into unique targets.
- Altitude floor: 25 ft (7.62 m) AGL. If the drone drops below it, the search loop commands a climb at 0.5 m/s. Below 20 ft (6.1 m) the run is flagged `zero_score_altitude_breach`, since the competition scores zero there.
- Time limit: 540 s, then the search stops and the drone lands.

## Quick start

SSH into the Pi and run:

```
# put real GPS coordinates in the config first
nano config/waypoint_nav.json

python main.py waypoint_nav
python main.py time_trial
python main.py object_localization

# custom config, or a simulator connection
python main.py waypoint_nav --config config/custom.json
python main.py waypoint_nav --address udpin://0.0.0.0:14540
```

The default connection is `serial:///dev/ttyAMA0:921600`. After every run a JSON result is written to `logs/` (created automatically). The `time_trial` and `object_localization` configs in the repo use coordinates around the simulator home point, so they run in SITL as they are. Replace them on competition day.

## How it's organized

```
main.py             picks a mission and connects to the drone
   |
BaseMission         lifecycle and result logging
   |                (WaypointNav, TimeTrial, ObjectLocalization)
DroneController     MAVSDK wrapper: pre-flight check, geofence, monitors, abort
   |
Pixhawk (PX4)       MAVLink over UART
```

Waypoint navigation and the time trial use PX4's mission mode. Object localization uses offboard mode with NED velocity commands.

### Mission lifecycle

Every mission goes through the same steps:

1. **Pre-flight check.** Fails if the battery is under 30% (it warns under 40%), there's no global position or home position, the accelerometer or gyro isn't calibrated, or the vehicle isn't armable.
2. **Pre-execute.** Arm, take off and start the safety monitors.
3. **Execute.** The mission itself.
4. **Post-execute.** Land and stop the monitors.
5. **Save the result.** `logs/<mission>_<timestamp>.json` holds the success flag, elapsed time, abort reason and mission data.

Any unhandled exception triggers the emergency abort.

### Safety

| Layer | What it does |
|-------|--------------|
| Pre-flight gate | Blocks launch if any of the checks above fail. |
| Battery monitor | Background task. Triggers the emergency abort below 30%. |
| Flight-mode monitor | Detects a PX4 failsafe: an RTL or land that we didn't ask for sets a flag that stops the mission loops. |
| Outer geofence | A breach more than 2 m outside the polygon (the buffer absorbs GPS jitter) triggers an immediate return to launch. The polygon is also uploaded to PX4 so QGroundControl draws it. |
| Inner geofence (optional) | A breach holds the drone and sends it back to the last position recorded inside the inner polygon. Waypoint navigation replaces this with its own recovery, described above. |
| Altitude floor | Object localization only, see above. |
| Mission timeout | 540 s. Waypoint navigation returns to launch. The other two stop and land. |
| In-loop check | Every mission loop calls `is_safe_to_continue()`, which is false after a failsafe, a critical battery or an outer fence breach. |
| Emergency abort | Stop offboard, then RTL, then land, then kill the motors, moving to the next step only if the previous call fails. |
| LiDAR | A TF Mini on `/dev/ttyAMA3` logs the height above ground while airborne. It's for monitoring only. |

When it connects, `DroneController` also sets these PX4 parameters: `COM_RC_IN_MODE = 4` (no RC input required), `MPC_ACC_HOR = 1.5` and `MPC_JERK_AUTO = 3.0` (gentler acceleration), `COM_HOME_IN_AIR = 0` (home is the launch point) and `GF_ACTION = 0` (PX4's own fence action is off, because our monitor handles breaches).

## Configuration

Mission parameters live in JSON files in `config/`.

**Waypoint navigation** (`waypoint_nav.json`, 1 to 7 waypoints)

| Field | Meaning | Default |
|-------|---------|---------|
| `altitude_m` | flight altitude | 5.0 |
| `speed_m_s` | cruise speed | 2.0 |
| `acceptance_radius_m` | waypoint reached when this close | 1.0 |
| `optimize_route` | reorder waypoints for the shortest route | true |
| `rtl_after` | return to launch after the last waypoint | true |
| `timeout_s` | mission time limit | 540 |

Each waypoint needs `lat` and `lon`. It can also set `alt_m`, `label`, `speed_m_s`, `fly_through` and `loiter_s`.

**Circuit time trial** (`time_trial.json`, 2 to 7 waypoints)

| Field | Meaning | Default |
|-------|---------|---------|
| `altitude_m` | flight altitude | 5.0 |
| `speed_m_s` | speed | 5.0 |
| `acceptance_radius_m` | waypoint reached when this close | 4.0 |
| `optimize_route` | reorder waypoints for the shortest route | true |
| `timeout_s` | mission time limit | 540 |

**Object localization** (`object_localization.json`)

| Field | Meaning | Default |
|-------|---------|---------|
| `altitude_m` | search altitude | 8.0 |
| `min_altitude_m` | altitude floor (25 ft) | 7.62 |
| `search_speed_m_s` | lawnmower speed | required |
| `leg_spacing_m` | distance between passes | 2.0 |
| `search_margin_m` | margin around the GCP corners | 3.0 |
| `timeout_s` | search time limit | 540 |
| `search_area` | 3 or more GPS corners | required unless a grid size is set |
| `yolo_model_path` | model weights | `camera/yolo26n.pt` |
| `yolo_confidence` | detection threshold | 0.3 |
| `yolo_classes` | class filter, `null` for all | null |
| `detect_every_n_frames` | run YOLO on every Nth frame | 5 |

Instead of `search_area` you can give `grid_width_m` and `grid_length_m`. The config is rejected if `altitude_m` is below `min_altitude_m`.

**Geofence** (`geofence.json`) has an `outer` polygon and an optional `inner` polygon, each a list of `{"lat", "lon"}` points. The older single-polygon format (`"geofence": [...]`) still loads.

## Hardware

- Raspberry Pi 5 (companion computer)
- Pixhawk running PX4. MAVLink goes over UART at `/dev/ttyAMA0`, 921600 baud, through MAVSDK-Python.
- Raspberry Pi camera through Picamera2, 640x480 at 30 fps. It falls back to an OpenCV webcam if Picamera2 isn't available.
- TF Mini LiDAR on `/dev/ttyAMA3`, 115200 baud
- Ultralytics YOLO nano
