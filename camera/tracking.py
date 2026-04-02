import cv2
import numpy as np


class HybridTracker:
    def __init__(self, detect_every=10, conf=0.2, classes=None):
        # Open tracker object and stores the current bounding box using (x,y,w,h)
        # Then counts how many frames since the last detection and runs object detection
        # every Nth frame
        self.tracker = None
        self.bbox = None  # (x,y,w,h)
        self.frames_since_detect = 0
        self.detect_every = detect_every
        self.conf = conf  # Min confidence
        self.classes = classes

    def _create_tracker(self):
        # Try to access the legacy tracker module if it exists
        legacy = getattr(cv2, "legacy", None)

        # Check both cv2,Legacy and cv2 because tracker APIs differ by opencv version
        for mod in (legacy, cv2):
            if mod is None:
                continue
            # Try creating one of several supported tracker types
            for name in ("TrackerCSRT_create", "TrackerKCF_create", "TrackerMOSSE_create"):
                fn = getattr(mod, name, None)
                if fn:
                    # Return the first available tracker contructor found
                    return fn()

        # If none of the trackers exist, opencv contrib is probably missing
        raise RuntimeError("No OpenCV tracker available (need opencv-contrib).")

    def _init_tracker(self, frame, bbox):
        # create a fresh tracker instance and initializes the tracker the current frame and bounding box
        self.tracker = self._create_tracker()
        self.tracker.init(frame, bbox)
        self.bbox = bbox

        # Reset the detection counter
        self.frames_since_detect = 0


def cam(frame, model, ht: HybridTracker):
    # If frame capture/reading failed, return empty and the original frame
    if frame is None or frame.size == 0:
        return None, None, frame

    # 1) Fast track step
    # Try updating the existing tracker before running a full YOLO detection
    tracked_ok = False
    if ht.tracker is not None:
        tracked_ok, bbox = ht.tracker.update(frame)

        if tracked_ok:
            # Save updated tracker bounding box as floats: (x,y,w,h)
            ht.bbox = tuple(map(float, bbox))
        else:
            # Tracking failed, so the tracker state is cleared
            ht.tracker = None
            ht.bbox = None

    # 2) Decide whether to run YOLO detection
    # Run detection if no tracker exists or enough frames have passed
    need_detect = (ht.tracker is None) or (ht.frames_since_detect >= ht.detect_every)

    det_bbox = None
    if need_detect:

        # Reset detection frame counter since a detection has passed
        ht.frames_since_detect = 0

        # Run YOLO on the current frame
        res = model(frame, verbose=False, conf=ht.conf, classes=ht.classes)[0]

        # pick the highest confidence dectected box
        best = None
        best_conf = 0.0
        for b in res.boxes:
            conf = float(b.conf.item())
            if conf > best_conf:
                # Get box as [x1, y1, x2, y2]
                xyxy = b.xyxy[0].cpu().numpy()
                best = xyxy
                best_conf = conf

        # Convert best detection from corner format to tracker format
        if best is not None:
            x1, y1, x2, y2 = best
            x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)

            # Only accept valid boxes with positive width and height and
            # initialize the tracker using the detected object
            if w > 0 and h > 0:
                det_bbox = (x, y, w, h)
                ht._init_tracker(frame, det_bbox)

    # count the frames since the last detection
    ht.frames_since_detect += 1

    # 3) Choose final bbox
    bbox = ht.bbox if ht.bbox is not None else det_bbox

    # 4) Compute pixel error if bbox exists
    err_px = None
    if bbox is not None:
        x, y, w, h = bbox

        # compute center of bounding box
        cx = x + w / 2.0
        cy = y + h / 2.0

        # Get frame height and width
        H, W = frame.shape[:2]

        # Error/Offset of object center from frame center
        err_px = (cx - W / 2.0, cy - H / 2.0)

        # visualize
        x, y, w, h = map(int, bbox)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow("Hybrid Track", frame)
    return bbox, err_px, frame