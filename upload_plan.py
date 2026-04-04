#!/usr/bin/env python3
"""
Upload mission plan and geofence to PX4 for visualization in QGC.
Does NOT arm or fly — purely for previewing the plan on the map.

Usage:
    python upload_plan.py waypoint_nav
    python upload_plan.py time_trial
    python upload_plan.py waypoint_nav --address udpin://0.0.0.0:14540
"""

import argparse
import asyncio
import itertools
import json
import logging
import math
from pathlib import Path

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s]: %(message)s")
logger = logging.getLogger(__name__)

DEFAULT_CONFIGS = {
    "waypoint_nav": "config/waypoint_nav.json",
    "time_trial":   "config/time_trial.json",
}

DEFAULT_ADDRESS = "udpin://0.0.0.0:14540"


def load_config(mission):
    path = Path(DEFAULT_CONFIGS[mission])
    with open(path) as f:
        return json.load(f)


def gps_distance_m(wp1, wp2):
    dlat = (wp1["lat"] - wp2["lat"]) * 111139
    dlon = (wp1["lon"] - wp2["lon"]) * 111139 * math.cos(math.radians(wp1["lat"]))
    return math.sqrt(dlat ** 2 + dlon ** 2)


def optimize_route(waypoints):
    n = len(waypoints)
    best_order, best_dist = None, float("inf")
    for perm in itertools.permutations(range(n)):
        dist = sum(gps_distance_m(waypoints[perm[i]], waypoints[perm[i+1]]) for i in range(n-1))
        if dist < best_dist:
            best_dist, best_order = dist, perm
    return [waypoints[i] for i in best_order], best_order, best_dist


def build_mission_items(waypoints, config, fly_through=False):
    items = []
    default_alt = config.get("altitude_m", 5.0)
    speed = config.get("speed_m_s", 2.0)
    acceptance = config.get("acceptance_radius_m", 1.0)
    for wp in waypoints:
        items.append(MissionItem(
            latitude_deg=wp["lat"],
            longitude_deg=wp["lon"],
            relative_altitude_m=wp.get("alt_m", default_alt),
            speed_m_s=speed,
            is_fly_through=fly_through,
            gimbal_pitch_deg=float("nan"),
            gimbal_yaw_deg=float("nan"),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=float("nan"),
            acceptance_radius_m=acceptance,
            yaw_deg=float("nan"),
            camera_photo_distance_m=float("nan"),
            vehicle_action=MissionItem.VehicleAction.NONE,
        ))
    return items


async def run(args):
    drone = System()
    logger.info(f"Connecting to {args.address}...")
    await drone.connect(system_address=args.address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info("Connected!")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            logger.info("GPS OK")
            break

    # Upload geofence
    from mavsdk.geofence import FenceType, GeofenceData, Point, Polygon
    geofence_path = Path("config/geofence.json")
    with open(geofence_path) as f:
        geofence_data = json.load(f)["geofence"]
    points = [Point(pt["lat"], pt["lon"]) for pt in geofence_data]
    polygon = Polygon(points, FenceType.INCLUSION)
    await drone.geofence.upload_geofence(GeofenceData([polygon], []))
    logger.info(f"Geofence uploaded ({len(points)} vertices)")

    # Load and upload mission
    config = load_config(args.mission)
    waypoints = config["waypoints"]
    fly_through = args.mission == "time_trial"

    if args.mission == "waypoint_nav" and config.get("optimize_route", True):
        waypoints, order, dist = optimize_route(waypoints)
        logger.info(f"Route optimized: order={list(order)}, distance={dist:.0f}m")

    items = build_mission_items(waypoints, config, fly_through=fly_through)
    plan = MissionPlan(items)
    await drone.mission.upload_mission(plan)
    logger.info(f"Mission uploaded: {len(items)} waypoints — visible in QGC Plan view")


def main():
    parser = argparse.ArgumentParser(description="Upload plan to QGC without flying")
    parser.add_argument("mission", choices=list(DEFAULT_CONFIGS.keys()))
    parser.add_argument("--address", default=DEFAULT_ADDRESS)
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
