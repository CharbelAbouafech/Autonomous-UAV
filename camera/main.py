from ultralytics import YOLO
from collections import defaultdict
import cv2
import numpy as np
import asyncio

from object_detection import cam
from cam_movement import start_drone, forward, right, altitude, reset


# from drone_movement import start_drone, forward, right, altitude
# Load the YOLO26 model
model = YOLO("yolo26n.pt")
loc =[]
err_min = 20

    # Open the video file
video_path = 0
cap = cv2.VideoCapture(video_path)


async def main():

    # Store the track history
    track_history = defaultdict(lambda: [])
    drone = await start_drone()
    


    while cap.isOpened():
        loc = cam(cap, model, track_history,  classes=[67], conf=0.05)
        # print(loc)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        if not loc:
            continue
        
        target = max(loc, key=lambda d: d["confidence"])
        ex_px, ey_px = target["err_px"]

        if abs(ex_px) > err_min:
            if ex_px > 0:
                print('Move Left')
                await right(-1, drone)
            else:
                print('Move Right')
                await right(1, drone)

        if abs(ey_px) >err_min:
            if ey_px < 0:
                print('Move Forward')
                await forward(1, drone)
            else:
                print('Move Backward')
                await forward(-1, drone)
        
        if abs(ex_px) < err_min and abs(ey_px) < err_min:
            print('decend')
            await altitude(-1, drone)




        

if __name__ == "__main__":
    asyncio.run(main())