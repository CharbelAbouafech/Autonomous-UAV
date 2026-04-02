# Autonomous UAV

Fully autonomous UAV system for the **C-UASC** competition. Runs on a Raspberry Pi 5 companion computer with a Pixhawk flight controller (PX4). Three competition missions run entirely without human intervention after launch.

---

## Missions

### 1. Waypoint Navigation
Fly up to 7 GPS waypoints provided as an unordered list on competition day. The system optimizes the route (brute-force TSP over all permutations) to minimize total distance, then flies it using PX4 mission mode.

- Waypoints given day-of as unordered lat/lon/alt
- Route automatically optimized from drone's current position
- 10-minute time limit with auto-RTL on timeout
- Scored by judge-provided GPS data logger

### 2. Circuit Time Trial
Fly an ordered circuit of up to 7 waypoints as fast as possible. The drone flies the circuit up to 3 times and reports the fastest valid lap.

- Waypoints given day-of as ordered lat/lon/alt
- Up to 3 laps — fastest valid lap used for scoring
- Must pass within 2m of each waypoint or lap is invalidated
- Per-lap and per-checkpoint timing recorded

### 3. Object Localization
Search a defined area for ground targets using a lawnmower pattern and YOLO detection. Targets are survey-style ground control points (~0.6m squares) with black numbers on white sections.

- Search area defined by 3-4 large GCPs (~1.2m) provided as GPS coordinates
- Minimum altitude: 20ft (6.1m) AGL — actively enforced during flight
- 10-minute time limit
- Detections deduplicated by GPS proximity

---

## Quick Start

SSH into the Pi and run:

```bash
# Install dependencies
pip install -r requirements.txt

# Edit config with real GPS coordinates (provided on competition day)
nano config/waypoint_nav.json

# Run a mission
python main.py waypoint_nav
python main.py time_trial
python main.py object_localization

# Custom config or SITL testing
python main.py waypoint_nav --config config/custom.json
python main.py waypoint_nav --address udpin://0.0.0.0:14540
```

Results are saved as JSON in `logs/` after each run.

---

## Project Structure

```
Autonomous-UAV/
├── main.py                      # CLI entry point
├── drone_controller.py          # MAVSDK wrapper — offboard, mission, PID, safety
├── requirements.txt             # Python dependencies
├── controllers/
│   └── pid_controller.py        # PID controller (error -> velocity)
├── camera/
│   └── tracking.py              # Hybrid tracker (YOLOv8 + OpenCV)
├── missions/
│   ├── base_mission.py          # Mission lifecycle base class
│   ├── waypoint_nav.py          # Waypoint Navigation (route-optimized)
│   ├── time_trial.py            # Circuit Time Trial (multi-lap)
│   └── object_localization.py   # Object Localization (lawnmower + YOLO)
├── config/
│   ├── waypoint_nav.json        # Waypoint nav params + GPS placeholders
│   ├── time_trial.json          # Time trial params + GPS placeholders
│   └── object_localization.json # Localization params + search area
├── tests/
│   └── flight_tests.py          # Hover, velocity, PID flight tests
├── logs/                        # Mission result JSONs (gitignored)
└── no_longer_needed/            # Deprecated code (LiDAR, claw, old camera)
```

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│                   main.py                       │
│              (CLI + mission launcher)            │
└──────────────────────┬──────────────────────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
   ┌────────────┐ ┌─────────┐ ┌──────────────┐
   │ WaypointNav│ │TimeTrial│ │ObjectLocaliz.│
   │  Mission   │ │ Mission │ │   Mission    │
   └─────┬──────┘ └────┬────┘ └──────┬───────┘
         │             │             │
         └─────────────┼─────────────┘
                       ▼
              ┌─────────────────┐
              │  BaseMission    │
              │  (lifecycle +   │
              │   safety)       │
              └────────┬────────┘
                       ▼
              ┌─────────────────┐      ┌──────────────┐
              │DroneController  │◄────►│ PIDController │
              │ (MAVSDK wrapper)│      └──────────────┘
              └────────┬────────┘
                       ▼
                ┌─────────────┐
                │  Pixhawk    │
                │  (PX4)      │
                └─────────────┘
```

- **Waypoint Nav / Time Trial** use PX4 mission mode — the flight controller handles GPS navigation natively
- **Object Localization** uses offboard mode with NED velocity commands for the search pattern + concurrent YOLO detection

---

## Mission Lifecycle

Every mission follows the same lifecycle enforced by `BaseMission`:

1. **Pre-flight check** — battery, GPS, IMU, gyro, armability
2. **Pre-execute** — arm, takeoff, start safety monitors
3. **Execute** — mission-specific logic
4. **Post-execute** — land, stop monitors
5. **Save result** — JSON log to `logs/`

If anything fails, the emergency abort cascade triggers: stop offboard -> RTL -> land -> kill motors.

---

## Safety

| Layer | What it does |
|-------|-------------|
| Pre-flight gate | Blocks launch if battery/GPS/IMU checks fail |
| Battery monitor | Background task, aborts at 30%, warns at 40% |
| Flight mode monitor | Detects PX4 failsafe (unexpected RTL/LAND) |
| Altitude floor | Object localization enforces 20ft AGL minimum |
| Mission timeout | Auto-RTL/land if mission exceeds time limit |
| In-loop checks | Every mission loop checks `is_safe_to_continue()` |
| Emergency abort | Cascading fallback: stop -> RTL -> land -> kill |

---

## Configuration

All mission parameters live in JSON config files under `config/`. GPS coordinates are placeholders (`0.0`) — fill them on competition day.

**Waypoint Navigation:**
| Field | Description | Default |
|-------|-------------|---------|
| `altitude_m` | Flight altitude (meters) | 5.0 |
| `speed_m_s` | Cruise speed | 2.0 |
| `acceptance_radius_m` | Waypoint reach threshold | 1.0 |
| `optimize_route` | Enable TSP route optimization | true |
| `timeout_s` | Mission timeout (seconds) | 540 |
| `rtl_after` | Return to launch after last waypoint | true |

**Circuit Time Trial:**
| Field | Description | Default |
|-------|-------------|---------|
| `altitude_m` | Flight altitude (meters) | 5.0 |
| `speed_m_s` | Circuit speed | 5.0 |
| `acceptance_radius_m` | Waypoint proximity threshold | 2.0 |
| `max_laps` | Maximum laps to fly | 3 |
| `timeout_s` | Mission timeout (seconds) | 540 |

**Object Localization:**
| Field | Description | Default |
|-------|-------------|---------|
| `altitude_m` | Search altitude (meters) | 8.0 |
| `min_altitude_m` | Hard floor (20ft AGL) | 6.1 |
| `search_speed_m_s` | Lawnmower speed | 1.5 |
| `leg_spacing_m` | Distance between passes | 2.0 |
| `search_area` | GCP corner coordinates (3-4 points) | — |
| `timeout_s` | Search timeout (seconds) | 540 |
| `yolo_confidence` | YOLO detection threshold | 0.3 |

---

## Hardware

- **Flight Controller** — Pixhawk running PX4
- **Companion Computer** — Raspberry Pi 5
- **Camera** — Raspberry Pi Camera (Picamera2), 1280x720
- **Communication** — MAVLink over UART (`/dev/ttyAMA0:921600`) via MAVSDK-Python
- **Detection** — YOLOv8 nano model

---

## Tech Stack

- **Language:** Python 3 (asyncio)
- **Flight SDK:** MAVSDK-Python
- **Vision:** Ultralytics YOLOv8 + OpenCV
- **Flight Stack:** PX4
