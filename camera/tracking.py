import cv2
import numpy as np

class HybridTracker:
    def __init__(self, detect_every=10, conf=0.2, classes=None,max_missed=2, match_iou_thresh=0.2):
        # Open tracker object and stores the current bounding box using (x,y,w,h)
        # Then counts how many frames since the last detection and runs object detection
        # every Nth frame
        self.tracker = None
        self.bbox = None  # (x,y,w,h)

        self.frames_since_detect = 0
        self.detect_every = detect_every
        self.conf = conf  # Min confidence
        self.classes = classes

        self.missed_frames = 0
        self.target_lost = True
        self.max_missed = max_missed
        self.match_iou_thresh = match_iou_thresh




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

def bbox_iou(a, b):
    # Compute IoU(Intersection over union) between two boxes in (x, y, w, h) format
    # IoU measures how much the boxes overlap:
    #   1.0 = perfect overlap
    #   0.0 = no overlap
    # This is used to check whether the tracker box and a detected box
    # refer to the same target

    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    ax2, ay2 = ax + aw, ay + ah
    bx2, by2 = bx + bw, by + bh

    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih

    union = (aw * ah) + (bw * bh) - inter
    return inter / union if union > 0 else 0.0

def best_matching_detection(tracker_bbox, detections, iou_thresh=0.2):
    # Compare the current tracker box against all detected boxes and
    # choose the detection with the highest IoU
    # If the best overlap is above the threshold, treat it as a valid match
    # Otherwise return no match
    
    best_bbox = None
    best_conf = 0.0
    best_iou = 0.0

    for det_bbox, conf in detections:
        iou = bbox_iou(tracker_bbox, det_bbox)
        if iou > best_iou:
            best_iou = iou
            best_bbox = det_bbox
            best_conf = conf

    if best_iou >= iou_thresh:
        return best_bbox, best_conf, best_iou

    return None, None, 0.0


def process_frame(frame, model, ht: HybridTracker):
    # Detect the target then gives the bouding box to the tracker
    # Computes the target offset from the center and returns it

    # If frame capture/reading failed, return empty and the original frame
    if frame is None or frame.size == 0:
        return None, None, frame

    # 1) Fast track step
    # Try updating the existing tracker before running a full YOLO detection
    source = None
    tracker_bbox = None
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
    detect_interval = 1 if ht.missed_frames > 0 else ht.detect_every
    need_detect = (ht.tracker is None) or (ht.frames_since_detect >= ht.detect_every)

    detections = []
    det_bbox = None
    if need_detect:

        # Reset detection frame counter since a detection has passed
        ht.frames_since_detect = 0

        # Run YOLO on the current frame
        res = model(frame, verbose=False, conf=ht.conf, classes=ht.classes)[0]

        # pick the highest confidence dectected box
        for b in res.boxes:
            conf = float(b.conf.item())
            x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()

            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)

            if w > 0 and h > 0:
                detections.append(((x,y,w,h), conf))


        if tracker_bbox is not None and detections:
            match_bbox, _, _ = best_matching_detection(
                tracker_bbox,
                detections,
                iou_thresh=ht.match_iou_thresh,
            )
            
            if match_bbox is not None:
                ht._init_tracker(frame, match_bbox)
                ht.bbox = match_bbox
                ht.missed_frames = 0
                ht.target_lost = False
                source = "track+detect"
            else:
                ht.missed_frames += 1

                if ht.missed_frames >= ht.max_missed:
                    ht.tracker = None
                    ht.bbox = None
                    ht.target_lost = True
                    source = "lost"
                else:
                    ht.bbox = tracker_bbox
                    ht.target_lost = False
                    source = "uncertain"
        
        # 4 No tracker, but detector found something -> acquire target
        elif detections:
            best_det_bbox, _ = max(detections, key=lambda d: d[1])
            ht._init_tracker(frame, best_det_bbox)
            ht.bbox = best_det_bbox
            ht.missed_frames = 0
            ht.target_lost = False
            source = "detect"

        # 5) No tracker and no detection -> lost
        else:
            ht.missed_frames += 1
            ht.tracker = None
            ht.bbox = None
            ht.target_lost = True
            source = "lost"

    else:
        # No detection this frame, trust tracker for now
        ht.frames_since_detect += 1

        if tracker_bbox is not None:
            ht.bbox = tracker_bbox
            ht.target_lost = False
            source = "track"
        else:
            ht.missed_frames += 1
            ht.bbox = None
            ht.target_lost = True
            source = "lost"


    # 6) Output only valid target data
    bbox = None if ht.target_lost else ht.bbox

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

        return bbox, err_px, frame, source

def debug_vis(frame, bbox=None, err_px=None, source=None, missed_frames=0, target_lost=False):
    # Debug visual for testing

    vis = frame.copy()

    if bbox is not None:
        x, y, w, h = map(int, bbox)
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)

    if err_px is not None:
        H, W = vis.shape[:2]
        cx = int(W / 2 + err_px[0])
        cy = int(H / 2 + err_px[1])

        cv2.circle(vis, (W // 2, H // 2), 5, (255, 0, 0), -1)
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(vis, (W // 2, H // 2), (cx, cy), (255, 255, 0), 2)

    status = f"Source: {source}  Missed: {missed_frames}  Lost: {target_lost}"
    cv2.putText(
        vis,
        status,
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    return vis