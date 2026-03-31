import asyncio
import cv2
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# from picamera2 import Picamera2  # type: ignore
from ultralytics import YOLO
# from cam_movement import start_drone, right, forward, altitude, reset
from tracking import cam, HybridTracker
# from controllers.lidar_controller import LidarController
# from controllers.claw_controller import ClawController
import time

USE_PI = True  # Set True when running on Raspberry Pi

err_min = 20
GRAB_HEIGHT_CM = 15


async def main():
    model = YOLO("yolo26n.pt")

    if USE_PI:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (1280, 720), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(2)
    else:
        cap = cv2.VideoCapture(0)

    ht = HybridTracker(detect_every=10, conf=0.05, classes=[41])

    lidar = None
    claw = None
    if USE_PI:
        from controllers.lidar_controller import LidarController
        from controllers.claw_controller import ClawController
        lidar = LidarController()
        lidar.start()
        claw = ClawController()
        claw.start()

    grabbed = False

    # drone = await start_drone()

    try:
        while True:
            if USE_PI:
                frame = picam2.capture_array()
            else:
                ret, frame = cap.read()
                if not ret:
                    break

            bbox, err_px, frame = cam(frame, model, ht)

            if lidar:
                dist_cm, lidar_valid = lidar.get_distance_cm()
            else:
                lidar_valid = False
                dist_cm = 0

            lidar_text = f"ALT: {dist_cm:.1f} cm" if lidar_valid else "ALT: N/A"
            claw_text = "CLAW: CLOSED" if grabbed else "CLAW: OPEN"
            claw_color = (0, 0, 255) if grabbed else (0, 255, 0)

            cv2.putText(frame, lidar_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, claw_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, claw_color, 2)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if grabbed:
                continue

            if err_px is None:
                # await reset(drone)
                continue

            ex_px, ey_px = err_px

            if abs(ex_px) > err_min:
                print("left" if ex_px < 0 else "right")
                # await right(-1 if ex_px > 0 else 1, drone)
            elif abs(ey_px) > err_min:
                print("forward" if ey_px < 0 else "backward")
                # await forward(1 if ey_px < 0 else -1, drone)
            elif abs(ey_px) < err_min and abs(ex_px) < err_min:
                if lidar_valid and dist_cm <= GRAB_HEIGHT_CM:
                    print(f"Target in range ({dist_cm} cm) — grabbing!")
                    claw.close()
                    grabbed = True
                    # await altitude(1, drone)  # Ascend after grab
                elif lidar_valid:
                    print(f"Centered — descending (alt: {dist_cm} cm)")
                    # await altitude(-0.5, drone)
                else:
                    print("Centered")
    finally:
        if USE_PI:
            picam2.stop()
        else:
            cap.release()
        cv2.destroyAllWindows()
        if claw:
            claw.stop()
        if lidar:
            lidar.stop()


if __name__ == "__main__":
    asyncio.run(main())