#!/usr/bin/env python3
"""
Object Localization Mission.

Flies a lawnmower search pattern in offboard mode while running YOLO
detection on camera frames. When an object is detected, the drone's
GPS position is logged. Detections are deduplicated by proximity.

Two concurrent asyncio tasks handle flight and detection independently:
  - Flight task: NED velocity commands for the search pattern
  - Detection task: camera capture + YOLO inference + GPS tagging

Config example (config/object_localization.json):
    {
        "altitude_m": 4.0,
        "search_speed_m_s": 1.0,
        "grid_width_m": 10.0,
        "grid_length_m": 10.0,
        "leg_spacing_m": 3.0,
        "yolo_model_path": "camera/yolo26n.pt",
        "yolo_confidence": 0.3,
        "yolo_classes": null,
        "detect_every_n_frames": 5
    }
"""

import asyncio
import logging
import math
import time
from pathlib import Path

from missions.base_mission import BaseMission

logger = logging.getLogger(__name__)


class ObjectLocalizationMission(BaseMission):

    @property
    def name(self):
        return "object_localization"

    def _validate_config(self, config):
        required = ["altitude_m", "search_speed_m_s", "grid_width_m", "grid_length_m"]
        for key in required:
            if key not in config:
                raise ValueError(f"Config missing required key: '{key}'")

    async def pre_execute(self):
        """Arm, takeoff, then enter offboard mode for velocity control."""
        altitude = self.config.get("altitude_m", 4.0)
        await self.controller.arm_and_takeoff(altitude)
        self.controller.start_monitors()
        await self.controller.start_offboard()

    def _generate_lawnmower_legs(self):
        """Generate a lawnmower (boustrophedon) search pattern.

        Returns a list of (north_vel, east_vel, duration_s) tuples.
        The pattern alternates north/south passes, shifting east between each.
        """
        speed = self.config["search_speed_m_s"]
        length = self.config["grid_length_m"]
        width = self.config["grid_width_m"]
        spacing = self.config.get("leg_spacing_m", 3.0)

        leg_duration = length / speed
        shift_duration = spacing / speed
        num_legs = max(1, int(width / spacing))

        legs = []
        direction = 1  # 1 = north, -1 = south

        for i in range(num_legs):
            # Main pass (north or south)
            legs.append((speed * direction, 0.0, leg_duration))
            direction *= -1

            # Shift east between passes (skip after last leg)
            if i < num_legs - 1:
                legs.append((0.0, speed, shift_duration))

        return legs

    async def _fly_search_pattern(self):
        """Execute the lawnmower search pattern via NED velocity commands."""
        legs = self._generate_lawnmower_legs()
        logger.info(f"Search pattern: {len(legs)} legs")

        for i, (north_vel, east_vel, duration_s) in enumerate(legs):
            if not self.controller.is_safe_to_continue():
                logger.warning("Safety abort during search pattern")
                return

            direction = "shift E" if east_vel > 0 else ("N" if north_vel > 0 else "S")
            logger.info(f"  Leg {i + 1}/{len(legs)}: {direction} for {duration_s:.1f}s")

            end_time = time.time() + duration_s
            while time.time() < end_time:
                if not self.controller.is_safe_to_continue():
                    return
                await self.controller.set_velocity_ned(north=north_vel, east=east_vel)
                await asyncio.sleep(0.1)  # 10 Hz control loop

        await self.controller.hover()
        logger.info("Search pattern complete")

    async def _run_detection(self, detections, stop_event):
        """Capture camera frames, run YOLO, log detections with GPS position.

        Runs concurrently with the flight task. Appends to the shared
        detections list when objects are found.
        """
        from ultralytics import YOLO
        from camera.tracking import HybridTracker, cam

        model_path = self.config.get("yolo_model_path", "camera/yolo26n.pt")
        confidence = self.config.get("yolo_confidence", 0.3)
        classes = self.config.get("yolo_classes", None)
        detect_every = self.config.get("detect_every_n_frames", 5)

        # Resolve model path relative to project root
        project_root = Path(__file__).resolve().parent.parent
        full_model_path = project_root / model_path

        model = YOLO(str(full_model_path))
        ht = HybridTracker(detect_every=detect_every, conf=confidence, classes=classes)

        # Camera init — use Picamera2 on Pi, fall back to OpenCV webcam
        picam2 = None
        cap = None
        try:
            from picamera2 import Picamera2
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(
                main={"size": (1280, 720), "format": "RGB888"}
            )
            picam2.configure(config)
            picam2.start()
            await asyncio.sleep(2)  # camera warmup
            logger.info("Camera: Picamera2 initialized")
        except ImportError:
            import cv2
            cap = cv2.VideoCapture(0)
            logger.info("Camera: OpenCV webcam fallback")

        frame_count = 0
        try:
            while not stop_event.is_set():
                # Capture frame
                if picam2:
                    frame = picam2.capture_array()
                elif cap:
                    ret, frame = cap.read()
                    if not ret:
                        await asyncio.sleep(0.05)
                        continue
                else:
                    break

                bbox, err_px, frame = cam(frame, model, ht)
                frame_count += 1

                if bbox is not None:
                    lat, lon, alt = await self.controller.get_gps_position()
                    x, y, w, h = bbox
                    detection = {
                        "timestamp": time.time(),
                        "lat": lat,
                        "lon": lon,
                        "alt_m": alt,
                        "bbox": [float(x), float(y), float(w), float(h)],
                        "frame": frame_count,
                    }
                    detections.append(detection)
                    logger.info(
                        f"  DETECTED object at ({lat:.6f}, {lon:.6f}) "
                        f"alt={alt:.1f}m [frame {frame_count}]"
                    )

                # Yield to event loop so flight commands can run
                await asyncio.sleep(0)

        finally:
            if picam2:
                picam2.stop()
            if cap:
                cap.release()
            logger.info(f"Detection stopped after {frame_count} frames")

    @staticmethod
    def _deduplicate_detections(detections, min_distance_m=2.0):
        """Merge detections within min_distance_m of each other.

        Uses greedy clustering: each new detection is either merged into
        an existing cluster or starts a new one. Approximation:
        1 degree lat ~ 111,139m, 1 degree lon ~ 111,139m * cos(lat).
        """
        if not detections:
            return []

        clusters = []
        for det in detections:
            merged = False
            for cluster in clusters:
                dlat = (det["lat"] - cluster["lat"]) * 111139
                dlon = (det["lon"] - cluster["lon"]) * 111139 * math.cos(math.radians(det["lat"]))
                dist = math.sqrt(dlat ** 2 + dlon ** 2)
                if dist < min_distance_m:
                    merged = True
                    break
            if not merged:
                clusters.append({
                    "lat": det["lat"],
                    "lon": det["lon"],
                    "alt_m": det["alt_m"],
                    "first_seen": det["timestamp"],
                })
        return clusters

    async def execute(self):
        """Fly search pattern while running YOLO detection concurrently."""
        detections = []
        stop_event = asyncio.Event()

        # Launch detection as a background task
        detection_task = asyncio.create_task(
            self._run_detection(detections, stop_event)
        )

        # Fly the search pattern (blocks until complete)
        await self._fly_search_pattern()

        # Signal detection to stop and wait for cleanup
        stop_event.set()
        try:
            await asyncio.wait_for(detection_task, timeout=5.0)
        except asyncio.TimeoutError:
            detection_task.cancel()
            logger.warning("Detection task timed out during shutdown")

        # Deduplicate nearby detections
        unique = self._deduplicate_detections(detections, min_distance_m=2.0)

        logger.info(f"\nDetections: {len(detections)} raw, {len(unique)} unique objects")
        for i, obj in enumerate(unique):
            logger.info(f"  Object {i + 1}: ({obj['lat']:.6f}, {obj['lon']:.6f})")

        self.result.data["raw_detections"] = len(detections)
        self.result.data["unique_objects"] = len(unique)
        self.result.data["objects"] = unique

    async def post_execute(self):
        """Stop offboard mode, then land."""
        try:
            await self.controller.drone.offboard.stop()
        except Exception:
            pass
        await self.controller.land()
        await asyncio.sleep(10)
        self.controller.stop_monitors()
