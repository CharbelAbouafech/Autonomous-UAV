#!/usr/bin/env python3

import asyncio
import logging
import math

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from pid_controller import PIDController

# Enable INFO level logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DroneController:
    """Wrapper for MAVSDK drone with PID-based velocity control"""

    def __init__(self):
        self.drone = System()
        # PID controllers for each axis
        # These translate pixel errors → velocity commands
        self.pid_forward = PIDController(kp=0.003, ki=0.0001, kd=0.02)  # forward/back
        self.pid_right = PIDController(kp=0.003, ki=0.0001, kd=0.02)  # left/right
        self.pid_altitude = PIDController(kp=0.002, ki=0.0, kd=0.01)  # up/down

        self.max_forward_speed = 1.0  # m/s
        self.max_right_speed = 1.0  # m/s
        self.max_altitude_speed = 0.5  # m/s (slow descent for safety)

    async def connect(self, system_address="udpin://0.0.0.0:14540"):
        """Connect to drone"""
        logger.info(f"Connecting to drone at {system_address}...")
        await self.drone.connect(system_address=system_address)

        logger.info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logger.info("-- Connected to drone!")
                break

        logger.info("Waiting for global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                logger.info("-- Global position estimate OK")
                break

    async def arm_and_takeoff(self, altitude=5.0):
        """Arm and take off to specified altitude"""
        logger.info("-- Arming")
        await self.drone.action.arm()

        logger.info(f"-- Taking off to {altitude}m")
        await self.drone.action.takeoff()

        # Wait for takeoff to complete (rough estimate)
        await asyncio.sleep(altitude + 2)

    async def start_offboard(self):
        """Start offboard mode and set initial velocity"""
        logger.info("-- Setting initial setpoint")
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        logger.info("-- Starting offboard mode")
        try:
            await self.drone.offboard.start()
            logger.info("-- Offboard mode started!")
        except OffboardError as error:
            logger.error(f"Starting offboard failed: {error._result.result}")
            logger.info("-- Disarming")
            await self.drone.action.disarm()
            raise

    async def set_velocity(self, forward=0.0, right=0.0, down=0.0, yaw_rate=0.0):
        """
        Send velocity command to drone.

        Args:
            forward: Forward velocity (m/s), positive = forward
            right: Right velocity (m/s), positive = right
            down: Downward velocity (m/s), positive = DOWN
            yaw_rate: Yaw rate (rad/s), positive = clockwise
        """
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward, right, down, yaw_rate)
        )

    async def hover(self):
        """Stop all movement"""
        await self.set_velocity(0.0, 0.0, 0.0, 0.0)

    async def get_position_ned(self):
        """Get current NED position (single reading)"""
        async for pos_vel in self.drone.telemetry.position_velocity_ned():
            return (
                pos_vel.position.north_m,
                pos_vel.position.east_m,
                pos_vel.position.down_m,
            )

    async def land(self):
        """Land the drone"""
        logger.info("-- Landing")
        await self.drone.action.land()

    # ===== VISION-BASED CONTROL =====
    # These methods will be called with pixel errors from your camera system

    def get_velocity_from_errors(self, ex_px, ey_px, distance_cm=None):
        """
        Convert pixel errors to velocity commands using PID.

        Args:
            ex_px: Horizontal pixel error (positive = object right of center)
            ey_px: Vertical pixel error (positive = object below center)
            distance_cm: Distance to object (optional, for descent control)

        Returns:
            (forward, right, down): velocity tuple
        """
        # Lateral (left/right) control
        right_vel = self.pid_right.update(ex_px)
        right_vel = max(-self.max_right_speed, min(self.max_right_speed, right_vel))

        # Forward/backward control
        # Negative ey_px = object above center = move forward to get underneath
        forward_vel = self.pid_forward.update(-ey_px)  # Note: negative!
        forward_vel = max(-self.max_forward_speed, min(self.max_forward_speed, forward_vel))

        # Vertical (up/down) control - default: hover (zero descent)
        down_vel = 0.0

        return forward_vel, right_vel, down_vel

    async def center_on_object(self, ex_px, ey_px, centering_threshold=20.0):
        """
        Move drone to center on object.
        Blocks until centered or error exceeds timeout.

        Args:
            ex_px: Horizontal pixel error
            ey_px: Vertical pixel error
            centering_threshold: Pixel threshold for "centered"
        """
        logger.info(f"Centering on object (threshold: {centering_threshold}px)...")

        centered = False
        max_iterations = 200  # ~20 seconds at 10 Hz
        iterations = 0

        while not centered and iterations < max_iterations:
            # Update PID and get velocity
            forward, right, down = self.get_velocity_from_errors(ex_px, ey_px)

            # Send command
            await self.set_velocity(forward, right, down)

            # Check if centered
            if abs(ex_px) < centering_threshold and abs(ey_px) < centering_threshold:
                centered = True
                logger.info("-- Object centered!")
            else:
                logger.info(f"  Centering... ex={ex_px:.1f}px ey={ey_px:.1f}px")

            await asyncio.sleep(0.1)  # 10 Hz control loop
            iterations += 1

        if not centered:
            logger.warning("-- Centering timeout!")

        return centered


async def test_offboard_hover(controller):
    """Test 1: Hover in place"""
    logger.info("\n=== TEST 1: HOVER IN PLACE ===\n")

    logger.info("-- Hovering for 10 seconds...")
    for i in range(100):  # 100 * 0.1s = 10s
        await controller.hover()
        await asyncio.sleep(0.1)

    logger.info("-- Test 1 complete.")


async def test_velocity_commands(controller):
    """Test 2: Simple velocity commands"""
    logger.info("\n=== TEST 2: VELOCITY COMMANDS ===\n")

    # Move forward
    logger.info("-- Moving forward for 5 seconds...")
    for i in range(50):
        await controller.set_velocity(forward=0.5)
        await asyncio.sleep(0.1)

    # Hover to stop
    logger.info("-- Hovering...")
    for i in range(50):
        await controller.hover()
        await asyncio.sleep(0.1)

    # Move right
    logger.info("-- Moving right for 5 seconds...")
    for i in range(50):
        await controller.set_velocity(right=0.5)
        await asyncio.sleep(0.1)

    # Hover to stop
    logger.info("-- Hovering...")
    for i in range(50):
        await controller.hover()
        await asyncio.sleep(0.1)

    logger.info("-- Test 2 complete.")


async def test_pid_position(controller):
    """Test 3: Real PID tuning using telemetry feedback.

    Flies the drone to target offsets and measures how well the PID
    converges — logs overshoot, oscillation, and settling time.
    """
    logger.info("\n=== TEST 3: PID POSITION CONTROL (REAL TELEMETRY) ===\n")

    await controller.arm_and_takeoff(altitude=5)
    await controller.start_offboard()

    # Position PID controllers (meters → m/s)
    pid_north = PIDController(kp=0.5, ki=0.01, kd=0.3, max_output=1.0, min_output=-1.0)
    pid_east = PIDController(kp=0.5, ki=0.01, kd=0.3, max_output=1.0, min_output=-1.0)

    # Get starting position
    start_n, start_e, start_d = await controller.get_position_ned()
    logger.info(f"-- Start position: N={start_n:.2f} E={start_e:.2f} D={start_d:.2f}")

    # Waypoints to test PID response (offset from start in meters)
    waypoints = [
        (5.0, 0.0, "5m North"),
        (5.0, 5.0, "5m North + 5m East"),
        (0.0, 0.0, "Return to start"),
    ]

    settle_threshold = 0.3  # meters — "close enough"
    control_hz = 10
    dt = 1.0 / control_hz

    for target_n_offset, target_e_offset, label in waypoints:
        target_n = start_n + target_n_offset
        target_e = start_e + target_e_offset

        logger.info(f"\n-- Flying to: {label} (N={target_n:.1f}, E={target_e:.1f})")

        pid_north.reset()
        pid_east.reset()

        settled_count = 0
        max_iterations = 200  # 20 seconds
        peak_error = 0.0
        settled_at = None

        for i in range(max_iterations):
            # Read real position
            cur_n, cur_e, cur_d = await controller.get_position_ned()

            # Compute errors
            err_n = target_n - cur_n
            err_e = target_e - cur_e
            total_error = math.sqrt(err_n ** 2 + err_e ** 2)

            # Track peak error for overshoot detection
            if i > 0:
                peak_error = max(peak_error, total_error)

            # PID outputs
            vel_forward = pid_north.update(err_n)
            vel_right = pid_east.update(err_e)

            await controller.set_velocity(forward=vel_forward, right=vel_right)

            # Log every 5 iterations (0.5s)
            if i % 5 == 0:
                logger.info(
                    f"  [{i * dt:5.1f}s] pos=({cur_n:6.2f}, {cur_e:6.2f}) "
                    f"err=({err_n:+6.2f}, {err_e:+6.2f}) "
                    f"dist={total_error:5.2f}m "
                    f"vel=({vel_forward:+5.2f}, {vel_right:+5.2f})"
                )

            # Check if settled
            if total_error < settle_threshold:
                settled_count += 1
                if settled_count >= 10 and settled_at is None:
                    settled_at = i * dt
                    logger.info(f"  >> SETTLED at {settled_at:.1f}s (error={total_error:.3f}m)")
            else:
                settled_count = 0

            # Stop early if settled for 2 seconds
            if settled_count >= 20:
                break

            await asyncio.sleep(dt)

        # Report results for this waypoint
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

        # Brief hover between waypoints
        for _ in range(20):
            await controller.hover()
            await asyncio.sleep(0.1)

    logger.info("\n-- Test complete, landing...")
    await controller.land()
    await asyncio.sleep(5)


async def main():
    """Run tests"""
    print("\n" + "=" * 60)
    print("DRONE OFFBOARD CONTROL TESTS")
    print("=" * 60)
    print("\nAvailable tests:")
    print("  1. Takeoff and hover")
    print("  2. Velocity commands (forward, right)")
    print("  3. PID position control (real telemetry)")
    print("\nRunning all tests...\n")

    controller = DroneController()
    await controller.connect()

    try:
        await controller.arm_and_takeoff(altitude=5)
        await controller.start_offboard()

        await test_offboard_hover(controller)
        await test_velocity_commands(controller)
        await test_pid_position(controller)

        logger.info("\n-- All tests complete, landing...")
        await controller.land()
        await asyncio.sleep(10)
    except Exception as e:
        logger.error(f"Error: {e}")
        await controller.land()


if __name__ == "__main__":
    asyncio.run(main())