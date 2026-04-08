#!/usr/bin/env python3

import asyncio
import json
import logging
import math
from pathlib import Path

from mavsdk import System
from mavsdk.geofence import FenceType, GeofenceData, Point, Polygon
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import FlightMode

from controllers.pid_controller import PIDController

# Enable INFO level logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Safety constants
SAFE_ALTITUDE = 3.0          # meters — low altitude for real drone testing
BATTERY_MIN_PERCENT = 30     # abort if below this
BATTERY_WARN_PERCENT = 40    # warn if below this
MAX_TEST_SPEED = 0.3         # m/s — reduced for real drone


class DroneController:
    """Wrapper for MAVSDK drone with PID-based velocity control"""

    def __init__(self):
        self.drone = System()
        # PID controllers for each axis
        # These translate pixel errors → velocity commands
        self.pid_forward = PIDController(kp=0.003, ki=0.0001, kd=0.02)  # forward/back
        self.pid_right = PIDController(kp=0.003, ki=0.0001, kd=0.02)  # left/right
        self.pid_altitude = PIDController(kp=0.002, ki=0.0, kd=0.01)  # up/down

        self.max_forward_speed = 0.5  # m/s
        self.max_right_speed = 0.5  # m/s
        self.max_altitude_speed = 0.3  # m/s (slow descent for safety)

        # Safety state
        self._failsafe_triggered = False
        self._battery_critical = False
        self._landing_intentional = False
        self._rtl_intentional = False
        self._outside_geofence = False
        self._monitor_tasks = []

        # Load geofence polygon
        geofence_path = Path(__file__).resolve().parent / "config" / "geofence.json"
        with open(geofence_path, "r") as f:
            self._geofence_polygon = json.load(f)["geofence"]
        logger.info(f"Geofence loaded: {len(self._geofence_polygon)} vertices")

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

        # Allow autonomous modes without RC/joystick input (required for SITL)
        await self.drone.param.set_param_int("COM_RC_IN_MODE", 4)
        logger.info("-- COM_RC_IN_MODE set to 4 (no RC input required)")

        await self.upload_geofence()

    async def upload_geofence(self):
        """Upload geofence polygon to PX4 so QGC displays it on the map."""
        points = [
            Point(pt["lat"], pt["lon"]) for pt in self._geofence_polygon
        ]
        polygon = Polygon(points, FenceType.INCLUSION)
        await self.drone.geofence.upload_geofence(GeofenceData([polygon], []))
        logger.info("-- Geofence uploaded to PX4 (visible in QGC)")

    async def pre_flight_check(self):
        """Verify drone is safe to fly. Returns True if all checks pass."""
        logger.info("\n-- PRE-FLIGHT CHECK --")
        passed = True

        # Battery
        async for battery in self.drone.telemetry.battery():
            percent = battery.remaining_percent * 100
            voltage = battery.voltage_v
            logger.info(f"  Battery: {percent:.0f}% ({voltage:.1f}V)")
            if percent < BATTERY_MIN_PERCENT:
                logger.error(f"  FAIL: Battery too low ({percent:.0f}% < {BATTERY_MIN_PERCENT}%)")
                passed = False
            elif percent < BATTERY_WARN_PERCENT:
                logger.warning(f"  WARN: Battery below {BATTERY_WARN_PERCENT}%")
            else:
                logger.info("  Battery: OK")
            break

        # Health checks
        async for health in self.drone.telemetry.health():
            if not health.is_global_position_ok:
                logger.error("  FAIL: No global position (GPS)")
                passed = False
            else:
                logger.info("  GPS: OK")
            if not health.is_home_position_ok:
                logger.error("  FAIL: Home position not set")
                passed = False
            else:
                logger.info("  Home position: OK")
            if not health.is_accelerometer_calibration_ok:
                logger.error("  FAIL: Accelerometer not calibrated")
                passed = False
            else:
                logger.info("  Accelerometer: OK")
            if not health.is_gyrometer_calibration_ok:
                logger.error("  FAIL: Gyroscope not calibrated")
                passed = False
            else:
                logger.info("  Gyroscope: OK")
            if not health.is_armable:
                logger.error("  FAIL: Drone not armable")
                passed = False
            else:
                logger.info("  Armable: OK")
            break

        if passed:
            logger.info("-- PRE-FLIGHT CHECK PASSED --\n")
        else:
            logger.error("-- PRE-FLIGHT CHECK FAILED --\n")
        return passed

    async def check_battery(self):
        """Quick battery check. Returns (percent, voltage)."""
        async for battery in self.drone.telemetry.battery():
            percent = battery.remaining_percent * 100
            voltage = battery.voltage_v
            if percent < BATTERY_MIN_PERCENT:
                logger.critical(f"BATTERY CRITICAL: {percent:.0f}%")
            elif percent < BATTERY_WARN_PERCENT:
                logger.warning(f"Battery low: {percent:.0f}%")
            return percent, voltage

    async def emergency_abort(self):
        """Emergency abort: stop offboard → RTL → land → kill (cascading fallback)."""
        logger.critical("!!! EMERGENCY ABORT !!!")
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass
        try:
            logger.info("-- Attempting Return to Launch...")
            await self.drone.action.return_to_launch()
        except Exception:
            try:
                logger.info("-- RTL failed, attempting land...")
                await self.drone.action.land()
            except Exception:
                logger.critical("-- Land failed, KILLING MOTORS")
                await self.drone.action.kill()

    def is_safe_to_continue(self):
        """Check if it's safe to continue testing."""
        return not self._failsafe_triggered and not self._battery_critical and not self._outside_geofence

    @staticmethod
    def _point_in_polygon(lat, lon, polygon):
        """Ray-casting algorithm to check if (lat, lon) is inside a polygon."""
        n = len(polygon)
        inside = False
        j = n - 1
        for i in range(n):
            lat_i, lon_i = polygon[i]["lat"], polygon[i]["lon"]
            lat_j, lon_j = polygon[j]["lat"], polygon[j]["lon"]
            if ((lon_i > lon) != (lon_j > lon)) and \
               (lat < (lat_j - lat_i) * (lon - lon_i) / (lon_j - lon_i) + lat_i):
                inside = not inside
            j = i
        return inside

    @staticmethod
    def _distance_to_polygon_m(lat, lon, polygon):
        """Approximate distance in meters from point to nearest polygon edge.
        Uses flat-earth approximation (valid for small areas)."""
        min_dist = float("inf")
        n = len(polygon)
        cos_lat = math.cos(math.radians(lat))
        for i in range(n):
            j = (i + 1) % n
            # Convert polygon edge endpoints to local meters relative to the point
            ax = (polygon[i]["lat"] - lat) * 111139
            ay = (polygon[i]["lon"] - lon) * 111139 * cos_lat
            bx = (polygon[j]["lat"] - lat) * 111139
            by = (polygon[j]["lon"] - lon) * 111139 * cos_lat
            dx, dy = bx - ax, by - ay
            len_sq = dx * dx + dy * dy
            if len_sq == 0:
                dist = math.sqrt(ax * ax + ay * ay)
            else:
                t = max(0.0, min(1.0, (-ax * dx + -ay * dy) / len_sq))
                nearest_x = ax + t * dx
                nearest_y = ay + t * dy
                dist = math.sqrt(nearest_x * nearest_x + nearest_y * nearest_y)
            min_dist = min(min_dist, dist)
        return min_dist

    async def _monitor_geofence(self, breach_buffer_m=2.0):
        """Background task: land immediately if drone leaves geofence polygon.

        Args:
            breach_buffer_m: Only trigger if drone is more than this many meters
                             outside the polygon. Prevents false positives from
                             GPS jitter near the boundary.
        """
        try:
            async for pos in self.drone.telemetry.position():
                if self._outside_geofence:
                    break
                lat = pos.latitude_deg
                lon = pos.longitude_deg
                if not self._point_in_polygon(lat, lon, self._geofence_polygon):
                    dist = self._distance_to_polygon_m(lat, lon, self._geofence_polygon)
                    if dist < breach_buffer_m:
                        continue  # GPS jitter near boundary, not a real breach
                    logger.critical(
                        f"GEOFENCE BREACH at ({lat:.6f}, {lon:.6f}) "
                        f"{dist:.1f}m outside — landing immediately!"
                    )
                    self._outside_geofence = True
                    self._landing_intentional = True
                    try:
                        await self.drone.offboard.stop()
                    except Exception:
                        pass
                    await self.drone.action.land()
                    break
        except asyncio.CancelledError:
            pass

    async def _monitor_flight_mode(self):
        """Background task: detect PX4 failsafe."""
        try:
            async for mode in self.drone.telemetry.flight_mode():
                if mode in (FlightMode.RETURN_TO_LAUNCH, FlightMode.LAND):
                    if not self._failsafe_triggered and not self._landing_intentional and not self._rtl_intentional:
                        logger.warning(f"!! PX4 FAILSAFE: flight mode changed to {mode}")
                        self._failsafe_triggered = True
        except asyncio.CancelledError:
            pass

    async def _monitor_battery(self):
        """Background task: continuous battery watch."""
        try:
            async for battery in self.drone.telemetry.battery():
                percent = battery.remaining_percent * 100
                if percent < BATTERY_MIN_PERCENT and not self._battery_critical:
                    logger.critical(f"BATTERY CRITICAL: {percent:.0f}% — aborting!")
                    self._battery_critical = True
                    await self.emergency_abort()
        except asyncio.CancelledError:
            pass

    def start_monitors(self):
        """Start background battery, flight mode, and geofence monitoring."""
        self._monitor_tasks = [
            asyncio.ensure_future(self._monitor_flight_mode()),
            asyncio.ensure_future(self._monitor_battery()),
            asyncio.ensure_future(self._monitor_geofence()),
        ]
        logger.info("-- Safety monitors started")

    def stop_monitors(self):
        """Stop background monitoring tasks."""
        for task in self._monitor_tasks:
            task.cancel()
        self._monitor_tasks = []
        logger.info("-- Safety monitors stopped")

    async def arm_and_takeoff(self, altitude=SAFE_ALTITUDE):
        """Arm and take off to specified altitude"""
        logger.info(f"-- Setting takeoff altitude to {altitude}m")
        await self.drone.action.set_takeoff_altitude(altitude)

        logger.info("-- Arming")
        await self.drone.action.arm()

        logger.info(f"-- Taking off to {altitude}m")
        await self.drone.action.takeoff()

        # Wait until drone is actually in the air
        logger.info("-- Waiting for drone to be airborne...")
        async for in_air in self.drone.telemetry.in_air():
            if in_air:
                logger.info("-- Drone is airborne!")
                break

        # Give it time to reach target altitude and stabilize
        await asyncio.sleep(5)

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

    async def set_velocity_ned(self, north=0.0, east=0.0, down=0.0, yaw_deg=0.0):
        """Send NED velocity command (world frame)"""
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(north, east, down, yaw_deg)
        )

    async def hover(self):
        """Stop all movement"""
        await self.set_velocity(0.0, 0.0, 0.0, 0.0)

    async def is_in_air(self):
        """Check if drone is currently flying"""
        async for in_air in self.drone.telemetry.in_air():
            return in_air

    async def ensure_airborne(self, altitude=SAFE_ALTITUDE):
        """Re-arm and takeoff if the drone has landed"""
        if not await self.is_in_air():
            logger.info("-- Drone has landed, re-arming and taking off...")
            await asyncio.sleep(3)  # Wait for disarm to complete
            await self.arm_and_takeoff(altitude)
            await self.start_offboard()

    async def get_position_ned(self):
        """Get current NED position (single reading)"""
        async for pos_vel in self.drone.telemetry.position_velocity_ned():
            return (
                pos_vel.position.north_m,
                pos_vel.position.east_m,
                pos_vel.position.down_m,
            )

    async def get_gps_position(self):
        """Get current GPS position. Returns (lat_deg, lon_deg, relative_alt_m)."""
        async for pos in self.drone.telemetry.position():
            return (pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m)

    async def get_heading(self):
        """Get current heading in degrees (0=North, clockwise)."""
        async for heading in self.drone.telemetry.heading():
            return heading.heading_deg

    async def upload_and_run_mission(self, mission_items, rtl_after=False):
        """Upload a list of MissionItems to PX4 and start the mission.

        Args:
            mission_items: list of MissionItem objects
            rtl_after: return to launch after final waypoint
        """
        plan = MissionPlan(mission_items)
        await self.drone.mission.set_return_to_launch_after_mission(rtl_after)
        if rtl_after:
            self._rtl_intentional = True

        logger.info("Uploading mission...")
        await self.drone.mission.upload_mission(plan)
        logger.info("Mission uploaded, starting...")

        await self.drone.mission.start_mission()
        logger.info("Mission started")

    async def wait_for_mission_complete(self):
        """Block until PX4 mission is finished. Logs progress updates."""
        async for progress in self.drone.mission.mission_progress():
            if progress.total == 0:
                continue  # mission not yet loaded, skip 0/0 reads
            logger.info(f"  Mission progress: {progress.current}/{progress.total}")
            if progress.current == progress.total:
                logger.info("-- Mission complete")
                return progress

    async def land(self):
        """Land the drone"""
        logger.info("-- Landing")
        self._landing_intentional = True
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


