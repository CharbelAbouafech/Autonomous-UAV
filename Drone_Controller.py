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


async def test_offboard_hover():
    """Test 1: Take off and hover in place"""
    controller = DroneController()
    await controller.connect()

    logger.info("\n=== TEST 1: TAKEOFF AND HOVER ===\n")

    await controller.arm_and_takeoff(altitude=5)
    await controller.start_offboard()

    logger.info("-- Hovering for 10 seconds...")
    for i in range(100):  # 100 * 0.1s = 10s
        await controller.hover()
        await asyncio.sleep(0.1)

    logger.info("-- Test complete, landing...")
    await controller.land()
    await asyncio.sleep(5)


async def test_velocity_commands():
    """Test 2: Simple velocity commands"""
    controller = DroneController()
    await controller.connect()

    logger.info("\n=== TEST 2: VELOCITY COMMANDS ===\n")

    await controller.arm_and_takeoff(altitude=5)
    await controller.start_offboard()

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

    logger.info("-- Test complete, landing...")
    await controller.land()
    await asyncio.sleep(5)


async def test_pid_centering():
    """Test 3: Simulate pixel errors and test PID centering"""
    controller = DroneController()
    await controller.connect()

    logger.info("\n=== TEST 3: PID CENTERING (SIMULATED) ===\n")

    await controller.arm_and_takeoff(altitude=5)
    await controller.start_offboard()

    # Simulate object offset from center
    simulated_errors = [
        (100.0, -50.0),  # Object right and above
        (80.0, -40.0),
        (60.0, -30.0),
        (40.0, -20.0),
        (20.0, -10.0),
        (10.0, -5.0),
        (5.0, -2.0),
        (0.0, 0.0),  # Centered
    ]

    for ex, ey in simulated_errors:
        forward, right, down = controller.get_velocity_from_errors(ex, ey)
        logger.info(
            f"Error: ({ex:6.1f}px, {ey:6.1f}px) → Velocity: (F:{forward:6.3f}, R:{right:6.3f}, D:{down:6.3f} m/s)")

        # Send command
        await controller.set_velocity(forward, right, down)
        await asyncio.sleep(0.5)

    # Hover
    logger.info("-- Holding position...")
    for i in range(20):
        await controller.hover()
        await asyncio.sleep(0.1)

    logger.info("-- Test complete, landing...")
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
    print("  3. PID centering (simulated pixel errors)")
    print("\nRunning Test 1 (safest)...\n")

    try:
        await test_offboard_hover()
        # Uncomment below to run other tests
        # await test_velocity_commands()
        # await test_pid_centering()
    except Exception as e:
        logger.error(f"Error: {e}")


if __name__ == "__main__":
    asyncio.run(main())