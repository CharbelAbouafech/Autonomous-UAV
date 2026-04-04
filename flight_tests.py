#!/usr/bin/env python3
"""
Standalone flight tests extracted from drone_controller.py.

Usage (SITL):  python tests/flight_tests.py
Usage (real):  python tests/flight_tests.py --address serial:///dev/ttyAMA0:921600
"""

import asyncio
import argparse
import logging
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drone_controller import DroneController, SAFE_ALTITUDE, MAX_TEST_SPEED
from controllers.pid_controller import PIDController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def test_offboard_hover(controller):
    """Test 1: Hover in place"""
    logger.info("\n=== TEST 1: HOVER IN PLACE (5s) ===\n")

    logger.info("-- Hovering for 5 seconds...")
    for i in range(50):  # 50 * 0.1s = 5s
        if not controller.is_safe_to_continue():
            logger.warning("-- Safety abort during hover")
            break
        await controller.hover()
        await asyncio.sleep(0.1)

    logger.info("-- Test 1 complete.")


async def test_velocity_commands(controller):
    """Test 2: Simple velocity commands (reduced speed)"""
    logger.info("\n=== TEST 2: VELOCITY COMMANDS (0.3 m/s, 3s each) ===\n")

    await controller.ensure_airborne()

    # Move forward
    logger.info("-- Moving forward for 3 seconds...")
    for i in range(30):
        if not controller.is_safe_to_continue():
            break
        await controller.set_velocity(forward=MAX_TEST_SPEED)
        await asyncio.sleep(0.1)

    # Hover to stop
    logger.info("-- Hovering...")
    for i in range(30):
        if not controller.is_safe_to_continue():
            break
        await controller.hover()
        await asyncio.sleep(0.1)

    # Move right
    logger.info("-- Moving right for 3 seconds...")
    for i in range(30):
        if not controller.is_safe_to_continue():
            break
        await controller.set_velocity(right=MAX_TEST_SPEED)
        await asyncio.sleep(0.1)

    # Hover to stop
    logger.info("-- Hovering...")
    for i in range(30):
        if not controller.is_safe_to_continue():
            break
        await controller.hover()
        await asyncio.sleep(0.1)

    logger.info("-- Test 2 complete.")


async def test_pid_position(controller):
    """Test 3: PID position control with real telemetry (reduced distances)."""
    logger.info("\n=== TEST 3: PID POSITION CONTROL (2m waypoints, 0.5 m/s max) ===\n")

    await controller.ensure_airborne()

    # Position PID controllers (meters -> m/s)
    pid_north = PIDController(kp=0.5, ki=0.01, kd=0.3, max_output=0.5, min_output=-0.5)
    pid_east = PIDController(kp=0.5, ki=0.01, kd=0.3, max_output=0.5, min_output=-0.5)

    start_n, start_e, start_d = await controller.get_position_ned()
    logger.info(f"-- Start position: N={start_n:.2f} E={start_e:.2f} D={start_d:.2f}")

    waypoints = [
        (2.0, 0.0, "2m North"),
        (2.0, 2.0, "2m North + 2m East"),
        (0.0, 0.0, "Return to start"),
    ]

    settle_threshold = 0.3  # meters
    control_hz = 10
    dt = 1.0 / control_hz

    for target_n_offset, target_e_offset, label in waypoints:
        if not controller.is_safe_to_continue():
            logger.warning("-- Safety abort, skipping remaining waypoints")
            break

        target_n = start_n + target_n_offset
        target_e = start_e + target_e_offset

        logger.info(f"\n-- Flying to: {label} (N={target_n:.1f}, E={target_e:.1f})")

        pid_north.reset()
        pid_east.reset()

        settled_count = 0
        max_iterations = 150  # 15 seconds
        peak_error = 0.0
        settled_at = None

        for i in range(max_iterations):
            if not controller.is_safe_to_continue():
                logger.warning("-- Safety abort during PID control")
                break

            cur_n, cur_e, cur_d = await controller.get_position_ned()

            err_n = target_n - cur_n
            err_e = target_e - cur_e
            total_error = math.sqrt(err_n ** 2 + err_e ** 2)

            if i > 0:
                peak_error = max(peak_error, total_error)

            vel_north = pid_north.update(err_n)
            vel_east = pid_east.update(err_e)

            await controller.set_velocity_ned(north=vel_north, east=vel_east)

            if i % 5 == 0:
                logger.info(
                    f"  [{i * dt:5.1f}s] pos=({cur_n:6.2f}, {cur_e:6.2f}) "
                    f"err=({err_n:+6.2f}, {err_e:+6.2f}) "
                    f"dist={total_error:5.2f}m "
                    f"vel=({vel_north:+5.2f}, {vel_east:+5.2f})"
                )

            if total_error < settle_threshold:
                settled_count += 1
                if settled_count >= 10 and settled_at is None:
                    settled_at = i * dt
                    logger.info(f"  >> SETTLED at {settled_at:.1f}s (error={total_error:.3f}m)")
            else:
                settled_count = 0

            if settled_count >= 20:
                break

            await asyncio.sleep(dt)

        final_n, final_e, _ = await controller.get_position_ned()
        final_err = math.sqrt((target_n - final_n) ** 2 + (target_e - final_e) ** 2)

        logger.info(f"\n  --- Results for '{label}' ---")
        logger.info(f"  Final error:   {final_err:.3f}m")
        logger.info(f"  Peak error:    {peak_error:.3f}m")
        if settled_at:
            logger.info(f"  Settling time: {settled_at:.1f}s")
        else:
            logger.info(f"  Settling time: DID NOT SETTLE (threshold={settle_threshold}m)")
        logger.info(f"  PID gains:     kp={pid_north.kp} ki={pid_north.ki} kd={pid_north.kd}")

        for _ in range(20):
            await controller.hover()
            await asyncio.sleep(0.1)

    logger.info("\n-- Test 3 complete.")


async def main(address):
    """Run all flight tests with safety checks."""
    print("\n" + "=" * 60)
    print("DRONE SAFE REAL-WORLD TEST")
    print(f"  Altitude: {SAFE_ALTITUDE}m | Max speed: {MAX_TEST_SPEED} m/s")
    print("=" * 60)
    print("\nTests:")
    print("  1. Hover in place (5s)")
    print("  2. Velocity commands (0.3 m/s, 3s each)")
    print("  3. PID position control (2m waypoints)")
    print()

    controller = DroneController()
    await controller.connect(system_address=address)

    if not await controller.pre_flight_check():
        logger.error("Pre-flight check FAILED. Aborting.")
        return

    try:
        controller.start_monitors()
        await controller.arm_and_takeoff()
        await controller.start_offboard()

        if controller.is_safe_to_continue():
            percent, voltage = await controller.check_battery()
            logger.info(f"Battery: {percent:.0f}% ({voltage:.1f}V)")
            await test_offboard_hover(controller)

        if controller.is_safe_to_continue():
            percent, voltage = await controller.check_battery()
            logger.info(f"Battery: {percent:.0f}% ({voltage:.1f}V)")
            await test_velocity_commands(controller)

        if controller.is_safe_to_continue():
            percent, voltage = await controller.check_battery()
            logger.info(f"Battery: {percent:.0f}% ({voltage:.1f}V)")
            await test_pid_position(controller)

        logger.info("\n-- All tests complete, landing...")
        await controller.land()
        await asyncio.sleep(10)

    except Exception as e:
        logger.error(f"EXCEPTION: {e}")
        await controller.emergency_abort()
    finally:
        controller.stop_monitors()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone flight tests")
    parser.add_argument(
        "--address", default="udpin://0.0.0.0:14540",
        help="MAVSDK connection address"
    )
    args = parser.parse_args()
    asyncio.run(main(args.address))
