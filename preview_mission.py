#!/usr/bin/env python3
"""
Preview mission plan in QGC before executing.

Connects to PX4, uploads the geofence and waypoint plan so they appear
on the QGC map, then waits. Run main.py separately to actually fly.

Usage:
    python preview_mission.py
    python preview_mission.py --config config/waypoint_nav.json
    python preview_mission.py --address udpin://0.0.0.0:14540
"""

import argparse
import asyncio
import itertools
import json
import logging
import math
from pathlib import Path

from mavsdk import System
from mavsdk.geofence import FenceType, GeofenceData, Point, Polygon
from mavsdk.mission import MissionItem, MissionPlan

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

DEFAULT_ADDRESS = "udpin://0.0.0.0:14540"
DEFAULT_CONFIG = "config/waypoint_nav.json"
GEOFENCE_CONFIG = "config/geofence.json"


def _gps_distance_m(wp1, wp2):
    dlat = (wp1["lat"] - wp2["lat"]) * 111139
    dlon = (wp1["lon"] - wp2["lon"]) * 111139 * math.cos(math.radians(wp1["lat"]))
    return math.sqrt(dlat ** 2 + dlon ** 2)


def optimize_route(waypoints):
    """Brute-force shortest route (same logic as WaypointNavMission)."""
    n = len(waypoints)
    if n <= 1:
        return list(range(n))

    best_order, best_dist = None, float("inf")
    for perm in itertools.permutations(range(n)):
        dist = sum(
            _gps_distance_m(waypoints[perm[i]], waypoints[perm[i + 1]])
            for i in range(n - 1)
        )
        if dist < best_dist:
            best_dist, best_order = dist, perm

    logger.info(f"Optimized route order: {list(best_order)}, est. distance: {best_dist:.0f}m")
    return list(best_order)


def build_mission_items(waypoints, config):
    default_alt = config.get("altitude_m", 5.0)
    default_speed = config.get("speed_m_s", 2.0)
    acceptance = config.get("acceptance_radius_m", 1.0)
    items = []
    for i, wp in enumerate(waypoints):
        items.append(MissionItem(
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
            vehicle_action=MissionItem.VehicleAction.NONE,
        ))
        label = wp.get("label", f"WP{i}")
        logger.info(f"  {label}: ({wp['lat']:.6f}, {wp['lon']:.6f}) alt={wp.get('alt_m', default_alt)}m")
    return items


async def main(args):
    config_path = Path(args.config)
    if not config_path.exists():
        logger.error(f"Config not found: {config_path}")
        return

    with open(config_path) as f:
        config = json.load(f)

    geofence_path = Path(GEOFENCE_CONFIG)
    if not geofence_path.exists():
        logger.error(f"Geofence config not found: {geofence_path}")
        return

    with open(geofence_path) as f:
        geofence_polygon = json.load(f)["geofence"]

    drone = System()
    logger.info(f"Connecting to {args.address}...")
    await drone.connect(system_address=args.address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info("-- Connected")
            break

    logger.info("Waiting for global position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            logger.info("-- Position OK")
            break

    await drone.param.set_param_int("COM_RC_IN_MODE", 4)

    # Upload geofence
    points = [Point(pt["lat"], pt["lon"]) for pt in geofence_polygon]
    polygon = Polygon(points, FenceType.INCLUSION)
    await drone.geofence.upload_geofence(GeofenceData([polygon], []))
    logger.info(f"-- Geofence uploaded ({len(points)} vertices)")

    # Build and upload mission plan
    waypoints = list(config["waypoints"])
    if config.get("optimize_route", True):
        order = optimize_route(waypoints)
        waypoints = [waypoints[i] for i in order]

    logger.info(f"Building mission plan ({len(waypoints)} waypoints)...")
    items = build_mission_items(waypoints, config)

    await drone.mission.set_return_to_launch_after_mission(config.get("rtl_after", True))
    await drone.mission.upload_mission(MissionPlan(items))
    logger.info("-- Mission plan uploaded — visible in QGC")
    logger.info("")
    logger.info("Geofence and waypoints are now visible in QGC.")
    logger.info("Run 'python main.py waypoint_nav' when ready to fly.")
    logger.info("Press Ctrl+C to disconnect.")

    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preview mission in QGC before flying")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="Waypoint config JSON")
    parser.add_argument("--address", default=DEFAULT_ADDRESS, help="MAVSDK connection address")
    args = parser.parse_args()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        logger.info("-- Disconnected")
