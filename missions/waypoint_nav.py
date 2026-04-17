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

    async def _handle_inner_breach(self):
        """Inner fence breach handler: return to last completed waypoint, skip the bad one."""
        bad_index = self._current_progress
        logger.warning(f"Inner fence breach — returning to last waypoint, skipping WP index {bad_index}")

        try:
            await self.controller.drone.action.hold()
            await asyncio.sleep(0.5)
        except Exception:
            pass

        # Return to last completed waypoint (or home if none completed yet)
        if self._last_wp_position:
            lat, lon, alt_amsl = self._last_wp_position
            logger.info(f"Returning to last waypoint: ({lat:.5f}, {lon:.5f})")
            await self.controller.drone.action.goto_location(lat, lon, alt_amsl, float("nan"))

            # Wait until back inside inner fence
            for _ in range(30):
                await asyncio.sleep(1)
                try:
                    cur_lat, cur_lon, _ = await self.controller.get_gps_position()
                    inner = self.controller._inner_geofence_polygon
                    if inner and self.controller._point_in_polygon(cur_lat, cur_lon, inner):
                        logger.info("Back inside inner fence")
                        break
                except Exception:
                    pass
        else:
            logger.warning("No completed waypoint to return to — holding")
            await asyncio.sleep(5)

        # Skip the bad waypoint and resume mission from next
        skip_to = bad_index + 1
        if skip_to < len(self._mission_items):
            logger.info(f"Resuming mission from WP index {skip_to}")
            await asyncio.sleep(0.5)
            await self.controller.drone.mission.set_current_mission_item(skip_to)
            await asyncio.sleep(0.5)
            await self.controller.start_mission()
            self.controller._inner_breach_recovering = False  # allow future breach detection
        else:
            logger.info("Bad WP was the last — landing now")
            self._breach_triggered_land = True  # signals _monitor_until_done to exit
            await self.controller.land()
            # leave _inner_breach_recovering = True to prevent re-trigger during landing

    async def pre_execute(self):
        """Optimize route, upload mission to PX4, then arm.

        Uploading before arming guarantees the plan is in PX4 before any
        flight commands are issued — prevents 'no valid mission' errors.
        """
        waypoints = list(self.config["waypoints"])

        if self.config.get("optimize_route", True):
            # Use current GPS position as route start for accurate optimization
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

        mission_items = self._build_mission_items(waypoints)
        rtl_after = self.config.get("rtl_after", True)
        self._mission_items = mission_items
        self._current_progress = 0
        self._last_wp_position = None  # (lat, lon, alt_amsl) of last completed waypoint
        self._breach_triggered_land = False
        self.controller._inner_breach_callback = self._handle_inner_breach

        logger.info(f"Uploading {len(mission_items)} waypoints before arming...")
        await self.controller.upload_mission(mission_items, rtl_after=rtl_after)
        await asyncio.sleep(2)  # let QGC sync the plan before arming

        logger.info("-- Arming")
        await self.controller.drone.action.arm()
        self.controller.start_monitors()

    async def execute(self):
        """Start mission and monitor. PX4 handles takeoff and navigation simultaneously."""
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
            logger.info(f"  Mission progress: {progress.current}/{progress.total}")

            # Track current target WP index for breach handler
            self._current_progress = progress.current

            # Record position when a waypoint is completed
            if progress.current > 0 and progress.current > getattr(self, "_last_recorded_progress", 0):
                self._last_recorded_progress = progress.current
                try:
                    async for pos in self.controller.drone.telemetry.position():
                        self._last_wp_position = (pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
                        break
                except Exception:
                    pass

            if self._breach_triggered_land:
                logger.info("Landing triggered by inner fence breach — exiting monitor")
                return
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
