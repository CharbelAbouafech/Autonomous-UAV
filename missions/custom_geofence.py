import asyncio
from mavsdk import System
from mavsdk.geofence import Point, Polygon, FenceType, GeofenceData, Circle

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break
    
    print("Setting geofence action to RTL...")
    await drone.param.set_param_int("GF_ACTION", 3)

    # Your geofence points
    p1 = Point(35.053222, -118.151900)
    p2 = Point(35.050231, -118.151981)
    p3 = Point(35.050264, -118.153483)
    p4 = Point(35.047106, -118.153542)
    p5 = Point(35.047092, -118.150531)
    p6 = Point(35.049014, -118.150539)
    p7 = Point(35.049011, -118.148583)
    p8 = Point(35.053200, -118.148542)

    # Create polygon
    polygon = Polygon(
        [p1, p2, p3, p4, p5, p6, p7, p8],
        FenceType.INCLUSION
    )

    # Dummy circle (required by API)
    circle = Circle(Point(0, 0), 0, FenceType.INCLUSION)

    geofenceData = GeofenceData([polygon], [circle])

    print("Uploading geofence...")
    await drone.geofence.upload_geofence(geofenceData)
    print("Geofence uploaded!")

    # Arm & takeoff
    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)


if __name__ == "__main__":
    asyncio.run(run())