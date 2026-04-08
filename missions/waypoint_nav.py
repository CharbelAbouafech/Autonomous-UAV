#!/usr/bin/env python3
"""
Waypoint Navigation Mission.

Flies a sequence of GPS waypoints using PX4's built-in mission mode.
PX4 handles position control, wind compensation, and waypoint acceptance
natively — much more reliable than custom velocity control over GPS.

Competition rules:
    - Up to 7 waypoints provided as UNORDERED lat/lon/alt on competition day
    - UAS determines its own flight path (route optimization)
    - Must complete in under 10 minutes
    - Scored by judge-provided GPS data logger

Config (config/waypoint_nav.json):
    {
        "altitude_m": 5.0,
        "speed_m_s": 2.0,
        "acceptance_radius_m": 1.0,
        "rtl_after": true,
        "optimize_route": true,
        "timeout_s": 540,
        "waypoints": [
            {"lat": 33.12345, "lon": -117.12345, "alt_m": 5.0, "label": "WP1"},
            {"lat": 33.12346, "lon": -117.12346, "alt_m": 5.0, "label": "WP2"}
        ]
    }
"""

import asyncio
import itertools
import logging
import math

from mavsdk.mission import MissionItem

from missions.base_mission import BaseMission

logger = logging.getLogger(__name__)


class WaypointNavMission(BaseMission):

    @property
    def name(self):
        return "waypoint_nav"

    def _validate_config(self, config):
        if "waypoints" not in config or len(config["waypoints"]) == 0:
            raise ValueError("Config must have a non-empty 'waypoints' list")
        if len(config["waypoints"]) > 7:
            raise ValueError("Competition allows a maximum of 7 waypoints")
        for i, wp in enumerate(config["waypoints"]):
            if "lat" not in wp or "lon" not in wp:
                raise ValueError(f"Waypoint {i} missing 'lat' or 'lon'")

    @staticmethod
    def _gps_distance_m(wp1, wp2):
        dlat = (wp1["lat"] - wp2["lat"]) * 111139
        dlon = (wp1["lon"] - wp2["lon"]) * 111139 * math.cos(math.radians(wp1["lat"]))
        dalt = wp1.get("alt_m", 0) - wp2.get("alt_m", 0)
        return math.sqrt(dlat ** 2 + dlon ** 2 + dalt ** 2)

    def _optimize_route(self, waypoints, start_pos=None):
        """Brute-force shortest route through all waypoints (7! = 5040 max)."""
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

        best_order, best_dist = None, float("inf")
        for perm in itertools.permutations(range(n)):
            d = route_distance(perm)
            if d < best_dist:
                best_dist = d
                best_order = perm

        return list(best_order), best_dist

    def _build_mission_items(self, waypoints):
        default_alt = self.config.get("altitude_m", 5.0)
        default_speed = self.config.get("speed_m_s", 2.0)
        acceptance = self.config.get("acceptance_radius_m", 1.0)
        items = []

        for i, wp in enumerate(waypoints):
            items.append(MissionItem(
                latitude_deg=wp["lat"],
                longitude_deg=wp["lon"],
                relative_altitude_m=wp.get("alt_m", default_alt),
                speed_m_s=wp.get("speed_m_s", default_speed),
                is_fly_through=wp.get("fly_through", False),
                gimbal_pitch_deg=float("nan"),
                gimbal_yaw_deg=float("nan"),
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=wp.get("loiter_s", 0.0),
                camera_photo_interval_s=float("nan"),
                acceptance_radius_m=acceptance,
                yaw_deg=float("nan"),
                camera_photo_distance_m=float("nan"),
                vehicle_action=MissionItem.VehicleAction.NONE,
            ))
            label = wp.get("label", f"WP{i + 1}")
            logger.info(f"  {label}: ({wp['lat']:.6f}, {wp['lon']:.6f}) alt={wp.get('alt_m', default_alt)}m")

        return items

    async def pre_execute(self):
        """Optimize route, upload mission to PX4, then arm.

        Uploading before arming guarantees the plan is in PX4 before any
        flight commands are issued — prevents 'no valid mission' errors.
        """
        waypoints = list(self.config["waypoints"])

        if self.config.get("optimize_route", True):
            start_pos = None
            try:
                lat, lon, alt = await self.controller.get_gps_position()
                start_pos = {"lat": lat, "lon": lon, "alt_m": alt}
            except Exception:
                logger.warning("Could not get GPS for route optimization, ignoring start position")

            order, total_dist = self._optimize_route(waypoints, start_pos)
            waypoints = [waypoints[i] for i in order]
            logger.info(f"Route optimized: order={order}, est. distance={total_dist:.0f}m")
        else:
            logger.info("Route optimization disabled, using config order")

        self._ordered_waypoints = waypoints
        mission_items = self._build_mission_items(waypoints)
        rtl_after = self.config.get("rtl_after", True)
        logger.info(f"Uploading {len(mission_items)} waypoints before arming...")
        await self.controller.upload_mission(mission_items, rtl_after=rtl_after)

        await self.controller.arm_and_fly(
            first_waypoint=waypoints[0],
            altitude=self.config.get("altitude_m", 5.0),
            speed_m_s=self.config.get("speed_m_s", 2.0),
            ramp_time_s=self.config.get("ramp_time_s", 3.0),
        )
        self.controller.start_monitors()

    async def execute(self):
        """Start pre-uploaded mission and monitor until complete."""
        timeout = self.config.get("timeout_s", 540)

        await self.controller.start_mission()
        logger.info(f"Mission started — monitoring progress (timeout: {timeout}s)...")

        try:
            await asyncio.wait_for(self._monitor_until_done(), timeout=timeout)
        except asyncio.TimeoutError:
            logger.error(f"Mission timed out after {timeout}s!")
            self.result.data["timed_out"] = True
            self.controller._rtl_intentional = True
            await self.controller.drone.action.return_to_launch()

        self.result.data["waypoints_count"] = len(self.config["waypoints"])
        self.result.data["route_optimized"] = self.config.get("optimize_route", True)

    async def _monitor_until_done(self):
        async for progress in self.controller.drone.mission.mission_progress():
            if progress.total == 0:
                continue
            self.controller._current_mission_index = progress.current
            logger.info(f"  Mission progress: {progress.current}/{progress.total}")
            if not self.controller.is_safe_to_continue():
                logger.warning("Safety check failed — stopping mission")
                return
            if progress.current == progress.total:
                logger.info("-- Mission complete")
                return

    async def post_execute(self):
        """RTL is handled by PX4 if rtl_after=true. Wait for landing then stop monitors."""
        if not self.config.get("rtl_after", True):
            await self.controller.land()
        logger.info("-- Waiting for drone to land...")
        async for in_air in self.controller.drone.telemetry.in_air():
            if not in_air:
                logger.info("-- Drone on the ground")
                break
        await asyncio.sleep(2)
        self.controller.stop_monitors()
