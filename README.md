# UAV Auton – Autonomous Drone System

UAV Auton is an autonomous unmanned aerial vehicle (UAV) project focused on intelligent navigation, mission execution, and real-time control. The main components for our Drone include the Pixhawk series flight controller, utilizing MAVSDK for Autonomous functions.

---

## 🚀 Features

- Autonomous takeoff, navigation, and landing  
- Waypoint-based mission planning  

---

## 🧠 System Architecture

The UAV Auton system is composed of:

- **Flight Controller** – Handles low-level stabilization and motor control  
- **Autonomy Module** – High-level decision-making and navigation logic  
- **Sensors** – GPS, IMU, barometer, and optional vision/LiDAR sensors  
- **Ground Control Station (GCS)** – Monitoring, configuration, and mission upload  

---

## 🛠️ Tech Stack

- **Programming Language:** Python
- **Flight Stack:** PX4 / ArduPilot *(or custom)*  
- **Middleware:** ROS / MAVLink  
- **Hardware:** Custom UAV frame, ESCs, motors, onboard computer (e.g., Raspberry Pi, Jetson)  

