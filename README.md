# Autonomous Delivery Drone

UAV Auton is an autonomous UAV project building toward a vision-guided delivery system. The drone uses a Pixhawk flight controller with PX4, controlled via MAVSDK from a Raspberry Pi companion computer.

---

## Features

- Autonomous takeoff, navigation, and landing
- PID-tuned position control (NED frame velocity commands)
- YOLOv8 object detection with hybrid OpenCV tracking
- Pre-flight safety checks (battery, GPS, IMU, gyro, armability)
- In-flight monitoring (battery watchdog, flight mode failsafe)
- Emergency abort with cascading fallback (stop offboard → RTL → land → kill)

---

## Project Structure

```
Autonomous-UAV/
├── drone_controller.py   # Main drone control — MAVSDK, offboard, PID, safety
├── pid_controller.py     # PID controller (error → velocity)
├── camera/
│   ├── main.py           # Vision pipeline entry point (YOLO + tracking)
│   ├── tracking.py       # Hybrid tracker (YOLOv8 + OpenCV)
│   └── cam_movement.py   # Async drone movement wrappers for camera control
└── README.md
```

---

## System Architecture

- **Flight Controller** – Pixhawk running PX4, handles stabilization and motor control
- **Companion Computer** – Raspberry Pi, runs autonomy and vision code
- **Camera** – Raspberry Pi Camera (Picamera2), 1280x720, YOLOv8 nano for detection
- **Communication** – MAVLink over UART (`/dev/ttyAMA0`)
- **Control** – Offboard mode with NED frame velocity commands via MAVSDK

---

## PID Configuration

**Position control (meters → m/s):**
- kp=0.5, ki=0.01, kd=0.3 | max output 0.5 m/s
- Settles under 0.3m in ~4.5s at 2m waypoint distances

**Vision control (pixels → m/s):**
- kp=0.003, ki=0.0001, kd=0.02

---

## Safety Limits

| Parameter | Value |
|-----------|-------|
| Test altitude | 3m |
| Max test speed | 0.3 m/s |
| Min battery | 30% (abort) |
| Battery warning | 40% |

---

## Tech Stack

- **Language:** Python (asyncio)
- **Flight Stack:** PX4
- **SDK:** MAVSDK-Python
- **Vision:** YOLOv8 (Ultralytics), OpenCV
- **Hardware:** Pixhawk, Raspberry Pi, Pi Camera

