import asyncio
import cv2
from picamera2 import Picamera2            # type: ignore
from ultralytics import YOLO
from cam_movement import start_drone, right, forward, altitude, reset
from tracking import cam, HybridTracker  # your new file
import time

err_min = 20

async def main():
    model = YOLO("yolo26n.pt")

    # Laptop cam video feed
    # cap = cv2.VideoCapture(0)

    # Picamera video feed
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (1280, 720), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    

    ht = HybridTracker(detect_every=10, conf=0.05, classes=[67])

    # drone = await start_drone()

    while True:
        frame = picam2.capture_array()
        bbox, err_px, frame = cam(frame, model, ht)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        if err_px is None:
            #await reset(drone)
            continue

        ex_px, ey_px = err_px

        if abs(ex_px) > err_min:
            print("left" if ex_px<0 else "right")
            # await right(-1 if ex_px > 0 else 1, drone)
        elif abs(ey_px) > err_min:
            print("forward" if ey_px<0 else "backward")
            # await forward(1 if ey_px < 0 else -1, drone)
        elif abs(ey_px) < err_min and abs(ex_px) < err_min:
            print("Centered")
            
            # await altitude(-1, drone)


    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())