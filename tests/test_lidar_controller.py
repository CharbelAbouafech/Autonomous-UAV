#!/usr/bin/env python3
"""Test the LidarController background reader."""

import time
from lidar_controller import LidarController

lidar = LidarController()
lidar.start()

try:
    print("Reading LiDAR (Ctrl+C to stop)...\n")
    while True:
        dist_m, valid = lidar.get_distance()
        dist_cm, _ = lidar.get_distance_cm()

        if valid:
            print(f"Distance: {dist_cm} cm ({dist_m:.2f} m)")
        else:
            print("No valid signal")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    lidar.stop()
