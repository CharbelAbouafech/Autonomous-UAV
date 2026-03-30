#!/usr/bin/env python3
"""
TFmini Plus LiDAR controller for precise AGL height measurement.

Reads distance over UART (/dev/ttyAMA3, 115200 baud) in a background thread.
Exposes get_distance() for integration with the async drone controller.

TFmini Plus frame format (9 bytes):
  [0x59][0x59][Dist_L][Dist_H][Str_L][Str_H][Temp_L][Temp_H][Checksum]
"""

import logging
import threading
import time

import serial

logger = logging.getLogger(__name__)

HEADER_BYTE = 0x59
FRAME_SIZE = 9
DEFAULT_PORT = "/dev/ttyAMA3"
DEFAULT_BAUD = 115200
MIN_STRENGTH = 100
MAX_STRENGTH = 65535


class LidarController:
    """TFmini Plus reader with background thread and checksum validation."""

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud

        self._serial = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        self._distance_cm = 0
        self._strength = 0
        self._valid = False
        self._last_update = 0.0

    def start(self):
        """Open serial port and start background reader."""
        logger.info(f"LiDAR: opening {self.port} at {self.baud} baud")
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=0.1,
        )
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("LiDAR: background reader started")

    def stop(self):
        """Stop reader and close serial port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info("LiDAR: stopped")

    def _read_loop(self):
        """Background thread: read and parse TFmini Plus frames."""
        buf = bytearray()

        while self._running:
            try:
                raw = self._serial.read(self._serial.in_waiting or 1)
                if not raw:
                    continue
                buf.extend(raw)

                while len(buf) >= FRAME_SIZE:
                    # Find header 0x59 0x59
                    header_idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == HEADER_BYTE and buf[i + 1] == HEADER_BYTE:
                            header_idx = i
                            break

                    if header_idx < 0:
                        buf = buf[-1:]
                        break

                    if header_idx > 0:
                        buf = buf[header_idx:]

                    if len(buf) < FRAME_SIZE:
                        break

                    frame = buf[:FRAME_SIZE]
                    buf = buf[FRAME_SIZE:]

                    # Verify checksum
                    checksum = sum(frame[:8]) & 0xFF
                    if checksum != frame[8]:
                        continue

                    distance_cm = frame[2] | (frame[3] << 8)
                    strength = frame[4] | (frame[5] << 8)
                    valid = MIN_STRENGTH <= strength < MAX_STRENGTH

                    with self._lock:
                        self._distance_cm = distance_cm
                        self._strength = strength
                        self._valid = valid
                        self._last_update = time.time()

            except serial.SerialException as e:
                logger.error(f"LiDAR serial error: {e}")
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"LiDAR read error: {e}")
                time.sleep(0.01)

    def get_distance(self):
        """
        Get latest reading.

        Returns:
            (distance_m, valid): distance in meters, and whether signal is reliable.
            Returns (0.0, False) if no data or stale (>0.5s).
        """
        with self._lock:
            age = time.time() - self._last_update if self._last_update > 0 else float("inf")
            if age > 0.5:
                return 0.0, False
            return self._distance_cm / 100.0, self._valid

    def get_distance_cm(self):
        """Get latest distance in centimeters. Returns (distance_cm, valid)."""
        with self._lock:
            age = time.time() - self._last_update if self._last_update > 0 else float("inf")
            if age > 0.5:
                return 0, False
            return self._distance_cm, self._valid

    @property
    def is_running(self):
        return self._running and self._thread is not None and self._thread.is_alive()
