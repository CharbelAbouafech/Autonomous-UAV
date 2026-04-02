#!/usr/bin/env python3
"""
Autonomous UAV Mission Launcher.

Usage (via SSH on Raspberry Pi):
    python main.py waypoint_nav
    python main.py time_trial
    python main.py object_localization
    python main.py waypoint_nav --config config/custom_waypoints.json
    python main.py waypoint_nav --address serial:///dev/ttyAMA0:921600
"""

import argparse
import asyncio
import logging
import sys

from drone_controller import DroneController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)

DEFAULT_CONFIGS = {
    "waypoint_nav": "config/waypoint_nav.json",
    "time_trial": "config/time_trial.json",
    "object_localization": "config/object_localization.json",
}

DEFAULT_ADDRESS = "serial:///dev/ttyAMA0:921600"


def get_mission_class(mission_name):
    """Import and return the mission class for the given name."""
    if mission_name == "waypoint_nav":
        from missions.waypoint_nav import WaypointNavMission
        return WaypointNavMission
    elif mission_name == "time_trial":
        from missions.time_trial import TimeTrialMission
        return TimeTrialMission
    elif mission_name == "object_localization":
        from missions.object_localization import ObjectLocalizationMission
        return ObjectLocalizationMission
    else:
        raise ValueError(f"Unknown mission: {mission_name}")


async def run(args):
    """Connect to drone and run the selected mission."""
    controller = DroneController()
    await controller.connect(system_address=args.address)

    MissionClass = get_mission_class(args.mission)
    config_path = args.config or DEFAULT_CONFIGS[args.mission]

    mission = MissionClass(controller, config_path)
    result = await mission.run()

    if result.success:
        logger.info(f"\nMission '{args.mission}' completed in {result.elapsed_seconds:.1f}s")
    else:
        logger.error(f"\nMission '{args.mission}' failed: {result.abort_reason}")

    return 0 if result.success else 1


def main():
    parser = argparse.ArgumentParser(description="Autonomous UAV Mission Launcher")
    parser.add_argument(
        "mission",
        choices=["waypoint_nav", "time_trial", "object_localization"],
        help="Mission to execute",
    )
    parser.add_argument(
        "--config", type=str, default=None,
        help="Path to mission config JSON (overrides default)",
    )
    parser.add_argument(
        "--address", type=str, default=DEFAULT_ADDRESS,
        help=f"MAVSDK connection address (default: {DEFAULT_ADDRESS})",
    )
    args = parser.parse_args()

    exit_code = asyncio.run(run(args))
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
