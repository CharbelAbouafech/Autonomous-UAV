#!/usr/bin/env python3
"""
Circuit Time Trial Mission.

Timed waypoint circuit — fly through ordered checkpoints as fast as possible.
Uses PX4 mission mode with all waypoints set to fly-through and higher speed.
The circuit is flown up to 3 times; the fastest valid lap wins.

Competition rules:
    - Up to 7 circuit waypoints provided as ORDERED lat/lon/alt on competition day
    - UAS flies waypoints in order
    - Maximum of 3 full circuits to set a time
    - Must pass within 4m of each waypoint (judge GPS logger) or lap is invalidated
    - Fastest valid lap used for scoring

Config example (config/time_trial.json):
    {
        "altitude_m": 5.0,
        "speed_m_s": 5.0,
        "acceptance_radius_m": 2.0,
        "max_laps": 3,
        "timeout_s": 540,
        "waypoints": [
            {"lat": 33.12345, "lon": -117.12345, "label": "Gate 1"},
            {"lat": 33.12350, "lon": -117.12350, "label": "Gate 2"}
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
            raise ValueError("Time trial needs at least 2 waypoints")
        if len(config["waypoints"]) > 7:
            raise ValueError("Competition allows a maximum of 7 circuit waypoints")
        for i, wp in enumerate(config["waypoints"]):
            if "lat" not in wp or "lon" not in wp:
                raise ValueError(f"Waypoint {i} missing 'lat' or 'lon'")

    def _build_lap_items(self):
        """Build MissionItems for a single lap (all fly-through)."""
        items = []
        default_alt = self.config.get("altitude_m", 5.0)
        speed = self.config.get("speed_m_s", 5.0)
        acceptance = self.config.get("acceptance_radius_m", 2.0)

        for wp in self.config["waypoints"]:
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
                vehicle_action=MissionItem.VehicleAction.NONE,
            )
            items.append(item)

        return items

    def _build_multi_lap_items(self, num_laps):
        """Build MissionItems for multiple laps by repeating the circuit."""
        single_lap = self._build_lap_items()
        return single_lap * num_laps

    async def pre_execute(self):
        """Mission mode handles arm + takeoff. We just start monitors."""
        self.controller.start_monitors()

    async def execute(self):
        """Upload multi-lap course, fly it, and track per-lap times."""
        waypoints = self.config["waypoints"]
        max_laps = self.config.get("max_laps", 3)
        timeout = self.config.get("timeout_s", 540)
        wps_per_lap = len(waypoints)

        logger.info(f"Circuit time trial: {wps_per_lap} waypoints x {max_laps} laps")
        for i, wp in enumerate(waypoints):
            logger.info(f"  {wp.get('label', f'WP{i}')}: ({wp['lat']:.6f}, {wp['lon']:.6f})")

        # Build and upload the full multi-lap mission
        mission_items = self._build_multi_lap_items(max_laps)
        logger.info(f"Uploading {len(mission_items)} total mission items ({max_laps} laps)...")
        await self.controller.upload_and_run_mission(mission_items, rtl_after=False)

        # Track checkpoint times as mission progresses
        lap_times = []          # list of {lap, time_s, checkpoints: [...]}
        current_lap = 0
        lap_start = time.time()
        lap_checkpoints = []
        last_index = -1

        async def monitor_progress():
            nonlocal current_lap, lap_start, lap_checkpoints, last_index

            async for progress in self.controller.drone.mission.mission_progress():
                current = progress.current
                total = progress.total

                if current != last_index and current > 0:
                    now = time.time()

                    # Which waypoint within the current lap?
                    wp_in_lap = (current - 1) % wps_per_lap
                    lap_num = (current - 1) // wps_per_lap

                    # New lap started
                    if lap_num > current_lap:
                        lap_time = now - lap_start
                        lap_times.append({
                            "lap": current_lap + 1,
                            "time_s": round(lap_time, 3),
                            "checkpoints": list(lap_checkpoints),
                        })
                        logger.info(f"\n  === LAP {current_lap + 1} TIME: {lap_time:.2f}s ===\n")
                        current_lap = lap_num
                        lap_start = now
                        lap_checkpoints = []

                    # Log this checkpoint
                    label = waypoints[wp_in_lap].get("label", f"WP{wp_in_lap}")
                    elapsed_in_lap = now - lap_start
                    prev = lap_checkpoints[-1]["elapsed"] if lap_checkpoints else 0
                    leg_time = elapsed_in_lap - prev

                    lap_checkpoints.append({
                        "checkpoint": label,
                        "elapsed": round(elapsed_in_lap, 3),
                        "leg_time": round(leg_time, 3),
                    })
                    logger.info(
                        f"  Lap {current_lap + 1} | {label}: "
                        f"{elapsed_in_lap:.2f}s (leg: {leg_time:.2f}s)"
                    )

                    last_index = current

                if current >= total:
                    # Record final lap
                    lap_time = time.time() - lap_start
                    lap_times.append({
                        "lap": current_lap + 1,
                        "time_s": round(lap_time, 3),
                        "checkpoints": list(lap_checkpoints),
                    })
                    logger.info(f"\n  === LAP {current_lap + 1} TIME: {lap_time:.2f}s ===\n")
                    break

        try:
            await asyncio.wait_for(monitor_progress(), timeout=timeout)
        except asyncio.TimeoutError:
            logger.error(f"Time trial timed out after {timeout}s!")
            # Record whatever lap was in progress
            if lap_checkpoints:
                lap_time = time.time() - lap_start
                lap_times.append({
                    "lap": current_lap + 1,
                    "time_s": round(lap_time, 3),
                    "checkpoints": list(lap_checkpoints),
                    "incomplete": True,
                })

        # Find best lap (incomplete laps excluded)
        valid_laps = [lap for lap in lap_times if not lap.get("incomplete", False)]
        best_lap = min(valid_laps, key=lambda x: x["time_s"]) if valid_laps else None

        if best_lap:
            logger.info(f"  BEST LAP: Lap {best_lap['lap']} — {best_lap['time_s']:.2f}s")
        else:
            logger.warning("  No valid laps completed!")

        self.result.data["laps"] = lap_times
        self.result.data["best_lap"] = best_lap
        self.result.data["total_laps_completed"] = len(valid_laps)

    async def post_execute(self):
        """Land after completing laps."""
        await self.controller.land()
        await asyncio.sleep(10)
        self.controller.stop_monitors()
