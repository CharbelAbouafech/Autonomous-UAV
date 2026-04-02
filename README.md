# Autonomous UAV

Fully autonomous UAV system running on a Raspberry Pi 5 companion computer with a Pixhawk flight controller (PX4). Three competition missions run entirely without human intervention after launch.

---

## Missions

### 1. Waypoint Navigation
Fly a sequence of GPS waypoints autonomously. Uses PX4's built-in mission mode — the flight controller handles position control, wind compensation, and waypoint acceptance natively.

### 2. Time Trial
Timed waypoint course. Same PX4 mission mode but optimized for speed — all waypoints are fly-through (no stopping), larger acceptance radius, higher speed. Records per-leg and total times.

### 3. Object Localization
Fly a lawnmower search pattern while running YOLOv8 object detection on camera frames. When an object is detected, the drone's GPS position is logged. Nearby detections are deduplicated to produce unique object positions.

---

## Usage

SSH into the Pi and run:

```bash
# Edit config files with real GPS coordinates first
nano config/waypoint_nav.json

# Run a mission
python main.py waypoint_nav
python main.py time_trial
python main.py object_localization

# Custom config or SITL testing
python main.py waypoint_nav --config config/custom.json
python main.py waypoint_nav --address udpin://0.0.0.0:14540
```

Mission results are saved as JSON in the `logs/` directory after each run.

---

## Project Structure

```
Autonomous-UAV/
├── main.py                      # CLI entry point — pick and run a mission
├── drone_controller.py          # Core drone control — MAVSDK, offboard, PID, safety
├── controllers/
│   └── pid_controller.py        # PID controller (error → velocity)
├── camera/
���   ├── tracking.py              # Hybrid tracker (YOLOv8 + OpenCV)
│   ├── track_and_grab.py        # Old grab mission (deprecated)
│   └── cam_movement.py          # Old movement wrappers (deprecated)
├── missions/
│   ├── base_mission.py          # Mission lifecycle base class
│   ├── waypoint_nav.py          # Waypoint Navigation mission
│   ├── time_trial.py            # Time Trial mission
│   └── object_localization.py   # Object Localization mission
├── config/
���   ├── waypoint_nav.json        # Waypoint nav config (GPS coords + params)
│   ├── time_trial.json          # Time trial config
│   └── object_localization.json # Object localization config
├── tests/
│   └── flight_tests.py          # Standalone hover/velocity/PID flight tests
├── logs/                        # Mission result JSONs (gitignored)
└── no_longer_needed/            # Deprecated LiDAR + claw code
```

---

## System Architecture

- **Flight Controller** — Pixhawk running PX4, handles stabilization and motor control
- **Companion Computer** — Raspberry Pi 5, runs autonomy and vision code
- **Camera** — Raspberry Pi Camera (Picamera2), 1280x720, YOLOv8 nano for detection
- **Communication** — MAVLink over UART (`/dev/ttyAMA0`) via MAVSDK-Python
- **Control** — PX4 mission mode for waypoint missions, offboard mode for search patterns

---

## Mission Lifecycle

Every mission follows the same lifecycle enforced by `BaseMission`:

1. **Pre-flight check** — battery, GPS, IMU, gyro, armability
2. **Pre-execute** — arm, takeoff, start safety monitors
3. **Execute** — mission-specific logic
4. **Post-execute** — land, stop monitors
5. **Save result** — JSON log to `logs/`

If anything fails, the emergency abort cascade triggers: stop offboard → RTL → land → kill motors.

---

## Safety

| Layer | What it does |
|-------|-------------|
| Pre-flight gate | Blocks launch if battery/GPS/IMU checks fail |
| Battery monitor | Background task, aborts at 30%, warns at 40% |
| Flight mode monitor | Detects PX4 failsafe (unexpected RTL/LAND) |
| In-loop checks | Every mission loop checks `is_safe_to_continue()` |
| Emergency abort | Cascading fallback: stop → RTL → land → kill |

---

## Configuration

All mission parameters live in JSON config files under `config/`. Edit the GPS coordinates before flying — templates ship with placeholder `0.0` values.

**Waypoint Nav / Time Trial config fields:**
- `altitude_m` — flight altitude
- `speed_m_s` — cruise speed
- `acceptance_radius_m` — how close to waypoint counts as "reached"
- `waypoints` — list of `{lat, lon, alt_m, label, fly_through}`
- `rtl_after` — return to launch after last waypoint (waypoint nav only)

**Object Localization config fields:**
- `grid_width_m`, `grid_length_m` — search area dimensions
- `leg_spacing_m` — distance between parallel passes
- `search_speed_m_s` — flight speed during search
- `yolo_model_path`, `yolo_confidence`, `yolo_classes` — YOLO detection params
- `detect_every_n_frames` — run YOLO every N frames (performance tuning)

---

## PID Configuration

**Position control (meters → m/s):**
- kp=0.5, ki=0.01, kd=0.3 | max output 0.5 m/s

**Vision control (pixels → m/s):**
- kp=0.003, ki=0.0001, kd=0.02

---

## Tech Stack

- **Language:** Python (asyncio)
- **Flight Stack:** PX4
- **SDK:** MAVSDK-Python
- **Vision:** YOLOv8 (Ultralytics), OpenCV
- **Hardware:** Pixhawk, Raspberry Pi 5, Pi Camera
