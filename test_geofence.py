#!/usr/bin/env python3
"""
Geofence breach test — runs in SITL.

Test sequence:
  1. Takeoff inside inner fence  →  last_safe_position recorded
  2. Fly OUTSIDE inner fence (inside outer)  →  hold + return to safe pos
  3. Confirm drone returned, then land
  4. [--outer] Fly OUTSIDE outer fence  →  RTL

Start SITL first:
    PX4_HOME_LAT=35.0503 PX4_HOME_LON=-118.1505 PX4_HOME_ALT=830.0 make px4_sitl gz_x500

Run:
    python test_geofence.py           # inner breach only
    python test_geofence.py --outer   # inner + outer
"""

import argparse
import asyncio
import logging

from drone_controller import DroneController

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# Inner fence: lat 35.049–35.0515, lon -118.1515 to -118.1495
SAFE_POS       = (35.0503, -118.1505, 838.0)   # home — center of inner fence
OUTSIDE_INNER  = (35.0530, -118.1505, 838.0)   # ~170m north of inner fence, inside outer
OUTSIDE_OUTER  = (35.0550, -118.1505, 838.0)   # north of outer fence (35.0532)


async def fly_to(dc, label, lat, lon, alt_amsl, wait_s=10):
    logger.info(f"\n>>> Sending drone to: {label} ({lat:.4f}, {lon:.4f})")
    await dc.drone.action.goto_location(lat, lon, alt_amsl, float("nan"))
    await asyncio.sleep(wait_s)


async def wait_for_recovery(dc, timeout_s=20):
    """Block until drone is back inside inner fence or timeout."""
    logger.info(f"Waiting up to {timeout_s}s for drone to re-enter inner fence...")
    for _ in range(timeout_s):
        await asyncio.sleep(1)
        if dc._inner_geofence_polygon is None:
            break
        try:
            lat, lon, _ = await dc.get_gps_position()
            if dc._point_in_polygon(lat, lon, dc._inner_geofence_polygon):
                logger.info(f"  Drone back inside inner fence at ({lat:.5f}, {lon:.5f})")
                return True
        except Exception:
            pass
    logger.warning("  Recovery timeout — drone may not have returned")
    return False


async def run_test(test_outer=False):
    dc = DroneController()
    await dc.connect()

    if not await dc.pre_flight_check():
        logger.error("Pre-flight check failed — aborting")
        return

    await dc.arm_and_takeoff(altitude=5.0)
    dc.start_monitors()
    await asyncio.sleep(4)  # let monitors settle and record last_safe_position

    logger.info(f"  last_safe_position after takeoff: {dc._last_safe_position}")

    # ── TEST 1: Confirm inside inner fence ───────────────────────────────────
    logger.info("\n" + "="*60)
    logger.info("TEST 1: Flying to center of inner fence")
    logger.info("Expected: last_safe_position continuously updated, no breach")
    logger.info("="*60)
    await fly_to(dc, "Inner fence center", *SAFE_POS, wait_s=5)
    logger.info(f"  last_safe_position = {dc._last_safe_position}")

    # ── TEST 2: Breach inner fence ────────────────────────────────────────────
    logger.info("\n" + "="*60)
    logger.info("TEST 2: Flying OUTSIDE inner fence (inside outer)")
    logger.info("Expected: hold + goto last safe position")
    logger.info("="*60)
    await fly_to(dc, "Outside inner fence", *OUTSIDE_INNER, wait_s=6)

    if dc._inner_breach_recovering:
        logger.info("  PASS: inner breach detected")
    else:
        logger.warning("  NOTE: _inner_breach_recovering not yet set (may still be in progress)")

    recovered = await wait_for_recovery(dc, timeout_s=20)
    if recovered:
        logger.info("  PASS: drone recovered to inner fence")
    else:
        logger.warning("  FAIL: drone did not recover")

    # ── TEST 3 (optional): Breach outer fence ─────────────────────────────────
    if test_outer:
        logger.info("\n" + "="*60)
        logger.info("TEST 3: Flying OUTSIDE outer fence")
        logger.info("Expected: CRITICAL log + RTL")
        logger.info("="*60)

        # Suspend inner fence so recovery doesn't fight the outer fence flight
        dc._inner_geofence_polygon = None
        dc._inner_breach_recovering = False
        logger.info("  (Inner fence suspended for outer breach test)")

        await asyncio.sleep(2)
        logger.info("\n>>> Sending drone to: Outside outer fence (35.0550, -118.1505)")
        await dc.drone.action.goto_location(*OUTSIDE_OUTER, float("nan"))

        # Poll until outer breach detected or timeout
        logger.info("Waiting for outer fence breach (up to 90s)...")
        for _ in range(90):
            await asyncio.sleep(1)
            if dc._outside_geofence:
                logger.info("  PASS: outer breach detected, RTL triggered")
                break
        else:
            logger.warning("  FAIL: outer breach not detected in 90s — forcing RTL")
            dc._rtl_intentional = True
            await dc.drone.action.return_to_launch()

        logger.info("Waiting for RTL + landing...")
        async for in_air in dc.drone.telemetry.in_air():
            if not in_air:
                logger.info("  Drone landed")
                break
    else:
        logger.info("\nLanding...")
        await dc.land()
        async for in_air in dc.drone.telemetry.in_air():
            if not in_air:
                break

    dc.stop_monitors()
    logger.info("\n=== Geofence test complete ===")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--outer", action="store_true", help="Also test outer fence breach (triggers RTL)")
    args = parser.parse_args()
    asyncio.run(run_test(test_outer=args.outer))
