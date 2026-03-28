import asyncio
import cv2
from ultralytics import YOLO
from tracking import cam, HybridTracker
import time

err_min = 20


async def main():
    model = YOLO("yolov8n.pt")  # Changed to standard yolov8n (more likely available)

    # Laptop camera feed
    cap = cv2.VideoCapture(0)

    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    ht = HybridTracker(detect_every=10, conf=0.05, classes=[67]) # Detect all classes, higher conf
    # drone = await start_drone()

    frame_count = 0

    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            print("Error: Failed to read frame")
            break

        frame_count += 1
        bbox, err_px, frame = cam(frame, model, ht)

        # Print FPS every 30 frames
        if frame_count % 30 == 0:
            print(f"Frame {frame_count}")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        if err_px is None:
            # await reset(drone)
            continue

        ex_px, ey_px = err_px

        if abs(ex_px) > err_min:
            direction = "left" if ex_px < 0 else "right"
            print(f"{direction} | error: {ex_px:.1f}px")
            # await right(1 if ex_px > 0 else -1, drone)
        elif abs(ey_px) > err_min:
            direction = "forward" if ey_px < 0 else "backward"
            print(f"{direction} | error: {ey_px:.1f}px")
            # await forward(-1 if ey_px < 0 else 1, drone)
        elif abs(ey_px) < err_min and abs(ex_px) < err_min:
            print("Centered")
            # await altitude(1, drone)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    asyncio.run(main())