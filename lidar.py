#!/usr/bin/env python3
"""
TF Mini LiDAR — importable class + standalone reader.

TF Mini frame format (9 bytes):
    [0]  0x59  header
    [1]  0x59  header
    [2]  Dist_L
    [3]  Dist_H
    [4]  Strength_L
    [5]  Strength_H
    [6]  Reserved
    [7]  Quality
    [8]  Checksum  (sum of bytes 0–7, low byte)
"""

import serial

TFMINI_PORT     = '/dev/ttyAMA3'
TFMINI_BAUDRATE = 115200


class TFMini:
    def __init__(self, port=TFMINI_PORT, baudrate=TFMINI_BAUDRATE):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)

    def read(self):
        """
        Block until one valid frame is received.
        Returns (distance_mm, strength) or (None, None) on timeout/error.
        """
        while True:
            if self.ser.read() != b'\x59':
                continue
            if self.ser.read() != b'\x59':
                continue

            data = self.ser.read(7)
            if len(data) != 7:
                return None, None

            checksum = (0x59 + 0x59 + sum(data[:7])) & 0xFF
            if checksum != data[6]:
                continue

            dist     = data[0] + (data[1] << 8)
            strength = data[2] + (data[3] << 8)
            return dist, strength

    def close(self):
        self.ser.close()


# ── Standalone test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    THRESHOLD_MM = 10

    sensor = TFMini()
    print("TF Mini LiDAR reading — Ctrl+C to stop\n")
    try:
        while True:
            distance, strength = sensor.read()
            if distance is not None:
                print(f"Distance: {distance:>5} mm | Strength: {strength}")
                if distance <= THRESHOLD_MM:
                    print("  !! TOO CLOSE !!")
    except KeyboardInterrupt:
        sensor.close()
        print("\nStopped")
