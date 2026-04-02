#!/usr/bin/env python3
"""
Waypoint Navigation Mission.

Flies a sequence of GPS waypoints using PX4's built-in mission mode.
PX4 handles position control, wind compensation, and waypoint acceptance
natively — much more reliable than custom PID over GPS coordinates.

Config example (config/waypoint_nav.json):
    {
        "altitude_m": 5.0,
        "speed_m_s": 2.0,
        "acceptance_radius_m": 1.0,
        "rtl_after": true,
        "waypoints": [
            {"lat": 33.12345, "lon": -117.12345, "alt_m": 5.0, "fly_through": true},
            {"lat": 33.12346, "lon": -117.12346, "alt_m": 5.0, "fly_through": false}
        ]
    }
"""

import asyncio
import logging

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
        for i, wp in enumerate(config["waypoints"]):
            if "lat" not in wp or "lon" not in wp:
                raise ValueError(f"Waypoint {i} missing 'lat' or 'lon'")

    def _build_mission_items(self):
        """Convert config waypoints to MAVSDK MissionItem list."""
        items = []
        default_alt = self.config.get("altitude_m", 5.0)
        default_speed = self.config.get("speed_m_s", 2.0)
        acceptance = self.config.get("acceptance_radius_m", 1.0)

        for i, wp in enumerate(self.config["waypoints"]):
            item = MissionItem(
                latitude_deg=wp["lat"],
                longitude_deg=wp["lon"],
                relative_altitude_m=wp.get("alt_m", default_alt),
                speed_m_s=wp.get("speed_m_s", default_speed),
                is_fly_through=wp.get("fly_through", True),
                gimbal_pitch_deg=float("nan"),
                gimbal_yaw_deg=float("nan"),
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=wp.get("loiter_s", 0.0),
                camera_photo_interval_s=float("nan"),
                acceptance_radius_m=acceptance,
                yaw_deg=float("nan"),
                camera_photo_distance_m=float("nan"),
            )
            items.append(item)
            label = wp.get("label", f"WP{i}")
            logger.info(f"  {label}: ({wp['lat']:.6f}, {wp['lon']:.6f}) alt={wp.get('alt_m', default_alt)}m")

        return items

    async def pre_execute(self):
        """Mission mode handles arm + takeoff via PX4. We just start monitors."""
        self.controller.start_monitors()

    async def execute(self):
        """Upload waypoints to PX4 and fly the mission."""
        mission_items = self._build_mission_items()
        logger.info(f"Uploading {len(mission_items)} waypoints...")

        rtl_after = self.config.get("rtl_after", True)
        await self.controller.upload_and_run_mission(mission_items, rtl_after=rtl_after)

        logger.info("Monitoring mission progress...")
        await self.controller.wait_for_mission_complete()

        self.result.data["waypoints_count"] = len(mission_items)

    async def post_execute(self):
        """If rtl_after is set, PX4 handles RTL. Otherwise land manually."""
        if not self.config.get("rtl_after", True):
            await self.controller.land()
        # Wait for the vehicle to be on the ground
        await asyncio.sleep(10)
        self.controller.stop_monitors()
