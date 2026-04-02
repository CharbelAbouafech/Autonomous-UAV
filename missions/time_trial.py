#!/usr/bin/env python3
"""
Time Trial Mission.

Timed waypoint course — fly through checkpoints as fast as possible.
Uses PX4 mission mode with all waypoints set to fly-through and higher speed.
Records per-leg and total elapsed times.

Config example (config/time_trial.json):
    {
        "altitude_m": 5.0,
        "speed_m_s": 5.0,
        "acceptance_radius_m": 2.0,
        "waypoints": [
            {"lat": 33.12345, "lon": -117.12345, "label": "Start"},
            {"lat": 33.12350, "lon": -117.12350, "label": "Gate 1"},
            {"lat": 33.12345, "lon": -117.12345, "label": "Finish"}
        ]
    }
"""

import asyncio
import logging
import time

from mavsdk.mission import MissionItem

from missions.base_mission import BaseMission

logger = logging.getLogger(__name__)


class TimeTrialMission(BaseMission):

    @property
    def name(self):
        return "time_trial"

    def _validate_config(self, config):
        if "waypoints" not in config or len(config["waypoints"]) < 2:
            raise ValueError("Time trial needs at least 2 waypoints (start + finish)")
        for i, wp in enumerate(config["waypoints"]):
            if "lat" not in wp or "lon" not in wp:
                raise ValueError(f"Waypoint {i} missing 'lat' or 'lon'")

    def _build_mission_items(self):
        """All waypoints are fly-through for maximum speed."""
        items = []
        default_alt = self.config.get("altitude_m", 5.0)
        speed = self.config.get("speed_m_s", 5.0)
        acceptance = self.config.get("acceptance_radius_m", 2.0)

        for i, wp in enumerate(self.config["waypoints"]):
            item = MissionItem(
                latitude_deg=wp["lat"],
                longitude_deg=wp["lon"],
                relative_altitude_m=wp.get("alt_m", default_alt),
                speed_m_s=speed,
                is_fly_through=True,
                gimbal_pitch_deg=float("nan"),
                gimbal_yaw_deg=float("nan"),
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=0.0,
                camera_photo_interval_s=float("nan"),
                acceptance_radius_m=acceptance,
                yaw_deg=float("nan"),
                camera_photo_distance_m=float("nan"),
            )
            items.append(item)

        return items

    async def pre_execute(self):
        """Mission mode handles arm + takeoff. We just start monitors."""
        self.controller.start_monitors()

    async def execute(self):
        """Upload course, start mission, record checkpoint times."""
        mission_items = self._build_mission_items()
        waypoints = self.config["waypoints"]
        logger.info(f"Time trial course: {len(mission_items)} checkpoints")

        for i, wp in enumerate(waypoints):
            logger.info(f"  {wp.get('label', f'WP{i}')}: ({wp['lat']:.6f}, {wp['lon']:.6f})")

        await self.controller.upload_and_run_mission(mission_items, rtl_after=False)

        # Track checkpoint times as mission progresses
        checkpoint_times = []
        start_time = time.time()
        last_index = -1

        async for progress in self.controller.drone.mission.mission_progress():
            current = progress.current
            total = progress.total

            # New checkpoint reached
            if current != last_index and current > 0:
                now = time.time()
                elapsed = now - start_time

                # Get label for the checkpoint we just passed
                wp_index = min(current - 1, len(waypoints) - 1)
                label = waypoints[wp_index].get("label", f"WP{wp_index}")

                prev_elapsed = checkpoint_times[-1]["elapsed"] if checkpoint_times else 0
                leg_time = elapsed - prev_elapsed

                checkpoint_times.append({
                    "checkpoint": label,
                    "elapsed": round(elapsed, 3),
                    "leg_time": round(leg_time, 3),
                })
                logger.info(f"  Checkpoint '{label}': {elapsed:.2f}s (leg: {leg_time:.2f}s)")

                last_index = current

            if current >= total:
                break

        total_time = time.time() - start_time
        logger.info(f"\n  === TOTAL TIME: {total_time:.2f}s ===")

        self.result.data["total_time_s"] = round(total_time, 3)
        self.result.data["checkpoints"] = checkpoint_times

    async def post_execute(self):
        """Land at the finish line."""
        await self.controller.land()
        await asyncio.sleep(10)
        self.controller.stop_monitors()
