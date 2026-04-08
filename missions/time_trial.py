#!/usr/bin/env python3
"""
Circuit Time Trial Mission.

Timed single-run through all waypoints as fast as possible.
Uses PX4 mission mode with all waypoints set to fly-through.

Competition rules:
    - Same 7 unordered waypoints as Waypoint Navigation, provided on competition day
    - Teams choose their own route (route optimization recommended)
    - A start and end waypoint must be provided to judges before flight
    - Time starts when UAS passes within 4m of start waypoint
    - Time ends when UAS passes within 4m of end waypoint
    - Each waypoint must be passed within 4m or run is invalidated
    - Fastest valid run used for scoring (12 pts fastest, 8 pts slowest, linear)

Config example (config/time_trial.json):
    {
        "altitude_m": 5.0,
        "speed_m_s": 5.0,
        "acceptance_radius_m": 4.0,
        "optimize_route": true,
        "timeout_s": 540,
        "waypoints": [
            {"lat": 33.12345, "lon": -117.12345, "label": "Gate 1"},
            {"lat": 33.12350, "lon": -117.12350, "label": "Gate 2"}
        ]
    }
"""

import asyncio
import itertools
import logging
import math
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
            raise ValueError("Competition allows a maximum of 7 waypoints")
        for i, wp in enumerate(config["waypoints"]):
            if "lat" not in wp or "lon" not in wp:
                raise ValueError(f"Waypoint {i} missing 'lat' or 'lon'")

    @staticmethod
    def _gps_distance_m(wp1, wp2):
        """Approximate distance in meters between two GPS waypoints."""
        dlat = (wp1["lat"] - wp2["lat"]) * 111139
        dlon = (wp1["lon"] - wp2["lon"]) * 111139 * math.cos(math.radians(wp1["lat"]))
        return math.sqrt(dlat ** 2 + dlon ** 2)

    def _optimize_route(self, waypoints, start_pos=None):
        """Find shortest route through all waypoints (brute-force permutations).

        For <=7 waypoints (7! = 5040), brute-force all permutations.
        Optionally accounts for drone's current position as the start.
        """
        n = len(waypoints)
        if n <= 1:
            return list(range(n)), 0.0

        def route_distance(order):
            dist = 0.0
            if start_pos:
                dist += self._gps_distance_m(start_pos, waypoints[order[0]])
            for i in range(len(order) - 1):
                dist += self._gps_distance_m(waypoints[order[i]], waypoints[order[i + 1]])
            return dist

        best_order = None
        best_dist = float("inf")
        for perm in itertools.permutations(range(n)):
            d = route_distance(perm)
            if d < best_dist:
                best_dist = d
                best_order = perm

        return list(best_order), best_dist

    def _build_mission_items(self, waypoints):
        """Convert waypoint list to MAVSDK MissionItems (all fly-through)."""
        items = []
        default_alt = self.config.get("altitude_m", 5.0)
        speed = self.config.get("speed_m_s", 5.0)
        acceptance = self.config.get("acceptance_radius_m", 4.0)

        for wp in waypoints:
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

    async def pre_execute(self):
        """Optimize route, upload mission, report to judges, arm and S-curve ramp."""
        waypoints = list(self.config["waypoints"])

        if self.config.get("optimize_route", True):
            start_pos = None
            try:
                lat, lon, alt = await self.controller.get_gps_position()
                start_pos = {"lat": lat, "lon": lon, "alt_m": alt}
            except Exception:
                logger.warning("Could not get GPS position for route optimization")

            order, total_dist = self._optimize_route(waypoints, start_pos)
            waypoints = [waypoints[i] for i in order]
            logger.info(f"Route optimized: order={order}, estimated distance={total_dist:.0f}m")
        else:
            logger.info("Route optimization disabled, using config order")

        self._ordered_waypoints = waypoints

        # Report start/end to judges before arming
        start_wp = waypoints[0]
        end_wp = waypoints[-1]
        logger.info(
            f"\n  *** REPORT TO JUDGES ***\n"
            f"  Start: {start_wp.get('label', 'WP1')} ({start_wp['lat']:.6f}, {start_wp['lon']:.6f})\n"
            f"  End:   {end_wp.get('label', f'WP{len(waypoints)}')} ({end_wp['lat']:.6f}, {end_wp['lon']:.6f})\n"
            f"  *** END REPORT ***\n"
        )
        for i, wp in enumerate(waypoints):
            logger.info(f"  {i + 1}. {wp.get('label', f'WP{i + 1}')}: ({wp['lat']:.6f}, {wp['lon']:.6f})")

        mission_items = self._build_mission_items(waypoints)
        logger.info(f"Uploading {len(mission_items)} waypoints before arming...")
        await self.controller.upload_mission(mission_items, rtl_after=False)

        await self.controller.arm_and_fly(
            first_waypoint=waypoints[0],
            altitude=self.config.get("altitude_m", 5.0),
            speed_m_s=self.config.get("speed_m_s", 5.0),
            ramp_time_s=self.config.get("ramp_time_s", 3.0),
        )
        self.controller.start_monitors()

    async def execute(self):
        """Start pre-uploaded mission, time the run, and monitor until complete."""
        timeout = self.config.get("timeout_s", 540)
        waypoints = self._ordered_waypoints
        start_wp = waypoints[0]
        end_wp = waypoints[-1]

        await self.controller.start_mission()
        logger.info("Mission started — timing run...")

        run_start = None
        run_end = None
        checkpoints = []
        last_index = -1

        async def monitor_progress():
            nonlocal run_start, run_end, last_index

            async for progress in self.controller.drone.mission.mission_progress():
                current = progress.current
                total = progress.total

                if total == 0:
                    continue

                if current != last_index and current > 0:
                    self.controller._current_mission_index = current
                    now = time.time()

                    if current == 1:
                        run_start = now
                        logger.info(f"  Timer STARTED at {start_wp.get('label', 'WP1')}")

                    if run_start is not None:
                        elapsed = now - run_start
                        label = waypoints[current - 1].get("label", f"WP{current}")
                        prev_elapsed = checkpoints[-1]["elapsed"] if checkpoints else 0.0
                        leg_time = elapsed - prev_elapsed
                        checkpoints.append({
                            "waypoint": label,
                            "elapsed": round(elapsed, 3),
                            "leg_time": round(leg_time, 3),
                        })
                        logger.info(f"  {label}: {elapsed:.2f}s (leg: {leg_time:.2f}s)")

                    last_index = current

                if not self.controller.is_safe_to_continue():
                    logger.warning("Safety check failed — stopping time trial")
                    return

                if current >= total:
                    if run_start is not None:
                        run_end = time.time()
                        logger.info(f"\n  === RUN TIME: {run_end - run_start:.2f}s ===\n")
                    break

        try:
            await asyncio.wait_for(monitor_progress(), timeout=timeout)
        except asyncio.TimeoutError:
            logger.error(f"Time trial timed out after {timeout}s!")
            self.result.data["timed_out"] = True

        run_time = (run_end - run_start) if (run_start and run_end) else None
        self.result.data["run_time_s"] = round(run_time, 3) if run_time else None
        self.result.data["checkpoints"] = checkpoints
        self.result.data["start_waypoint"] = start_wp.get("label", "WP1")
        self.result.data["end_waypoint"] = end_wp.get("label", f"WP{len(waypoints)}")
        self.result.data["route_optimized"] = self.config.get("optimize_route", True)

        if run_time:
            logger.info(f"  Final run time: {run_time:.2f}s")
        else:
            logger.warning("  Run did not complete — no valid time recorded")

    async def post_execute(self):
        """Land after completing the run."""
        await self.controller.land()
        logger.info("-- Waiting for drone to land...")
        async for in_air in self.controller.drone.telemetry.in_air():
            if not in_air:
                logger.info("-- Drone is on the ground")
                break
        await asyncio.sleep(2)
        self.controller.stop_monitors()
