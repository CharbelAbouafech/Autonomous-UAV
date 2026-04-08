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

        # Dual geofence state
        # Hard geofence: inner boundary (inset from soft). Breach → return to prev waypoint, resume.
        # Soft geofence: competition boundary. Breach → RTL, mission over.
        self._hard_geofence_breach = False
        self._current_mission_index = 0   # updated by waypoint_nav mission monitor
        self.hard_geofence_margin_m = 15.0  # meters inset from soft geofence boundary

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

        # Smooth acceleration — prevents lurching when mission mode engages
        await self.drone.param.set_param_float("MPC_ACC_HOR", 1.5)   # default 3.0 m/s²
        await self.drone.param.set_param_float("MPC_JERK_AUTO", 3.0)  # default 8.0 m/s³
        logger.info("-- Acceleration limits set for smooth flight")

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

    async def _monitor_geofence(self, gps_jitter_buffer_m=2.0):
        """Background task: dual-layer geofence enforcement.

        Hard geofence (inner boundary — inset by hard_geofence_margin_m from soft edge):
            Breach → pause mission, return to previous waypoint, resume.
            Drone stays in the air and continues the mission.

        Soft geofence (competition boundary — the actual polygon):
            Breach → RTL immediately. Mission is over.
        """
        try:
            async for pos in self.drone.telemetry.position():
                if self._outside_geofence:
                    break

                lat = pos.latitude_deg
                lon = pos.longitude_deg
                dist_to_edge = self._distance_to_polygon_m(lat, lon, self._geofence_polygon)
                inside = self._point_in_polygon(lat, lon, self._geofence_polygon)

                # --- Soft geofence: outside the polygon ---
                if not inside:
                    if dist_to_edge < gps_jitter_buffer_m:
                        continue  # GPS jitter near boundary, ignore
                    logger.critical(
                        f"SOFT GEOFENCE BREACH at ({lat:.6f}, {lon:.6f}) "
                        f"{dist_to_edge:.1f}m outside — RTL!"
                    )
                    self._outside_geofence = True
                    self._rtl_intentional = True
                    try:
                        await self.drone.offboard.stop()
                    except Exception:
                        pass
                    await self.drone.action.return_to_launch()
                    break

                # --- Hard geofence: inside polygon but within margin of edge ---
                if inside and dist_to_edge < self.hard_geofence_margin_m and not self._hard_geofence_breach:
                    logger.warning(
                        f"HARD GEOFENCE WARNING at ({lat:.6f}, {lon:.6f}) "
                        f"{dist_to_edge:.1f}m from boundary — returning to previous waypoint"
                    )
                    self._hard_geofence_breach = True
                    await self._return_to_previous_waypoint()
                    self._hard_geofence_breach = False  # reset after recovery

        except asyncio.CancelledError:
            pass

    async def _return_to_previous_waypoint(self):
        """Pause mission, go back to previous waypoint, resume."""
        try:
            prev_index = max(0, self._current_mission_index - 1)
            logger.info(f"-- Returning to waypoint {prev_index}...")
            await self.drone.mission.pause_mission()
            await asyncio.sleep(0.5)
            await self.drone.mission.set_current_mission_item(prev_index)
            await self.drone.mission.start_mission()
            logger.info(f"-- Mission resumed from waypoint {prev_index}")
        except Exception as e:
            logger.error(f"Failed to return to previous waypoint: {e}")

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

        # Wait until drone reaches target altitude and vertical velocity is near zero
        logger.info(f"-- Climbing to {altitude}m, waiting for stabilization...")
        ALTITUDE_THRESHOLD = 0.3   # meters from target
        VELOCITY_THRESHOLD = 0.1   # m/s vertical — "settled"
        STABLE_DURATION = 1.5      # seconds it must stay stable before proceeding
        stable_since = None

        async for pos_vel in self.drone.telemetry.position_velocity_ned():
            alt_ok = abs(-pos_vel.position.down_m - altitude) < ALTITUDE_THRESHOLD
            vel_ok = abs(pos_vel.velocity.down_m_s) < VELOCITY_THRESHOLD

            if alt_ok and vel_ok:
                if stable_since is None:
                    stable_since = asyncio.get_event_loop().time()
                elif asyncio.get_event_loop().time() - stable_since >= STABLE_DURATION:
                    logger.info(f"-- Stabilized at {-pos_vel.position.down_m:.1f}m, proceeding")
                    break
            else:
                stable_since = None

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

    async def arm_and_fly(self, first_waypoint=None, altitude=SAFE_ALTITUDE,
                          speed_m_s=2.0, ramp_time_s=3.0):
        """Arm then S-curve ramp: climb to altitude while accelerating toward first_waypoint.

        If first_waypoint is provided, the drone climbs and accelerates toward it
        simultaneously using an S-curve profile. By the time this returns, the drone
        is at altitude and cruise speed — call start_mission() immediately after for
        a smooth, jolt-free transition into mission mode.

        If first_waypoint is None, climbs vertically and hovers at altitude.

        Args:
            first_waypoint: dict with 'lat'/'lon', or None for vertical climb only
            altitude:       target altitude in meters
            speed_m_s:      cruise speed to ramp up to
            ramp_time_s:    duration of the S-curve ramp
        """
        logger.info("-- Arming")
        await self.drone.action.arm()

        climb_speed = 1.5  # m/s upward

        if first_waypoint is not None:
            lat, lon, _ = await self.get_gps_position()
            dlat = first_waypoint["lat"] - lat
            dlon = (first_waypoint["lon"] - lon) * math.cos(math.radians(lat))
            bearing = math.atan2(dlon, dlat)
            yaw_deg = math.degrees(bearing) % 360
            north_unit = math.cos(bearing)
            east_unit = math.sin(bearing)
            logger.info(
                f"-- S-curve ramp: {ramp_time_s}s to {speed_m_s}m/s toward first waypoint, "
                f"climbing to {altitude}m simultaneously"
            )
        else:
            yaw_deg, north_unit, east_unit = 0.0, 0.0, 0.0
            logger.info(f"-- Vertical climb to {altitude}m")

        # Set zero setpoint before starting offboard (PX4 requirement)
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
        )
        await self.drone.offboard.start()

        dt = 0.05  # 20 Hz
        steps = int(ramp_time_s / dt)
        current_alt = 0.0

        for i in range(1, steps + 1):
            t = i / steps
            v = speed_m_s * 0.5 * (1 - math.cos(math.pi * t))  # S-curve
            v_down = -climb_speed if current_alt < altitude * 0.95 else 0.0

            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north_unit * v, east_unit * v, v_down, yaw_deg)
            )

            if i % 10 == 0:  # sample altitude every 0.5s
                try:
                    _, _, current_alt = await self.get_gps_position()
                except Exception:
                    pass

            await asyncio.sleep(dt)

        # If still below target altitude, hold cruise speed and finish climbing
        _, _, current_alt = await self.get_gps_position()
        if current_alt < altitude * 0.95:
            logger.info("-- Holding cruise speed, finishing climb...")
            async for pos in self.drone.telemetry.position():
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(north_unit * speed_m_s, east_unit * speed_m_s, -climb_speed, yaw_deg)
                )
                if pos.relative_altitude_m >= altitude * 0.95:
                    break

        logger.info(f"-- At {altitude}m and {speed_m_s}m/s — ready for mission")
        await self.drone.offboard.stop()
        await asyncio.sleep(0.3)

    async def upload_mission(self, mission_items, rtl_after=False):
        """Upload mission plan to PX4 without starting it.

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
        logger.info("-- Mission uploaded")

    async def start_mission(self):
        """Start a previously uploaded mission."""
        await self.drone.mission.start_mission()
        logger.info("-- Mission started")

    async def upload_and_run_mission(self, mission_items, rtl_after=False):
        """Upload a list of MissionItems to PX4 and start the mission."""
        await self.upload_mission(mission_items, rtl_after=rtl_after)
        await self.start_mission()

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


