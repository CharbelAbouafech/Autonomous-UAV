#!/usr/bin/env python3
import asyncio
from mavsdk import System                                               # type: ignore
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed         # type: ignore


async def start_drone():
    """Connects to drone and takesoff. Also prepares Offboard controls."""

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(5)

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))  

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
        return drone
    except OffboardError as error:
        print(
            f"Starting offboard mode failed with error code: \
              {error._result.result}"
        )
        print("-- Disarming")
        await drone.action.disarm()

# Move forward/backward
async def forward(speed, drone):
    
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(speed, 0.0, 0.0, 0.0))

# Move right/left
async def right(speed, drone):
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed( 0.0, speed, 0.0, 0.0))

# Drone altitude
async def altitude(alt, drone):
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed( 0.0, 0.0, alt, 0.0))

async def reset(drone):
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed( 0.0, 0.0, 0.0, 0.0))
