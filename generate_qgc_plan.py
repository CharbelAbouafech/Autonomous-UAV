#!/usr/bin/env python3
"""Generate a QGroundControl .plan file from config files.

Usage:
    python generate_qgc_plan.py
    Then open mission.plan in QGC: File > Open, or drag-and-drop onto the Plan view.
"""

import json
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent / "config"
OUTPUT = Path(__file__).resolve().parent / "mission.plan"

# Must match PX4_HOME_LAT/LON/ALT in run_sitl.sh
SITL_HOME = (35.0503, -118.1505, 830.0)


def load_json(name):
    with open(CONFIG_DIR / name) as f:
        return json.load(f)


def build_plan():
    wp_cfg = load_json("waypoint_nav.json")
    fence_cfg = load_json("geofence.json")

    # home_lat = wp_cfg["waypoints"][0]["lat"]
    # home_lon = wp_cfg["waypoints"][0]["lon"]
    # home_alt = wp_cfg.get("altitude_m", 5.0)
    home_lat, home_lon, home_alt = SITL_HOME

    # Mission items (MAVLink command 16 = NAV_WAYPOINT)
    items = []
    for i, wp in enumerate(wp_cfg["waypoints"]):
        items.append({
            "AMSLAltAboveTerrain": None,
            "Altitude": wp.get("alt_m", home_alt),
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,
            "doJumpId": i + 1,
            "frame": 3,
            "params": [
                0,
                0,
                0,
                None,
                wp["lat"],
                wp["lon"],
                wp.get("alt_m", home_alt)
            ],
            "type": "SimpleItem"
        })

    # Geofence polygons
    polygons = []

    outer = fence_cfg.get("outer", fence_cfg.get("geofence", []))
    polygons.append({
        "inclusion": True,
        "polygon": [[pt["lat"], pt["lon"]] for pt in outer],
        "version": 1
    })

    inner = fence_cfg.get("inner")
    if inner:
        polygons.append({
            "inclusion": True,
            "polygon": [[pt["lat"], pt["lon"]] for pt in inner],
            "version": 1
        })

    plan = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "version": 1,
        "mission": {
            "cruiseSpeed": wp_cfg.get("speed_m_s", 2.0),
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": wp_cfg.get("speed_m_s", 2.0),
            "items": items,
            "plannedHomePosition": [home_lat, home_lon, home_alt],
            "vehicleType": 2,
            "version": 2
        },
        "geoFence": {
            "circles": [],
            "polygons": polygons,
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        }
    }

    with open(OUTPUT, "w") as f:
        json.dump(plan, f, indent=2)

    print(f"Written: {OUTPUT}")
    print(f"  {len(items)} waypoints")
    print(f"  Outer fence: {len(outer)} vertices")
    if inner:
        print(f"  Inner fence: {len(inner)} vertices")
    print("\nOpen in QGC: Plan view → File icon (top-left) → Open, select mission.plan")


if __name__ == "__main__":
    build_plan()
