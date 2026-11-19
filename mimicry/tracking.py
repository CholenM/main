"""CPU hand recognition on the latest raw frame, independent of the browser."""

import math
import time
from collections import deque
from threading import Condition, Event, Thread

import cv2

from .assets import MODEL

LOST = {"hand": "lost"}
GESTURE_CONFIDENCE = .55
FINGERS = ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20))


def visibly_extended(landmarks):
    """Conservative fallback for an otherwise unclassified, clearly open hand."""
    if len(landmarks) != 21:
        return False
    points = [(p.x, p.y, p.z) for p in landmarks]
    scale = math.dist(points[0], points[9])
    if not math.isfinite(scale) or scale < .01:
        return False
    for finger in FINGERS:
        joints = [points[index] for index in finger]
        vectors = [tuple(y-x for x, y in zip(a, b)) for a, b in zip(joints, joints[1:])]
        for u, v in zip(vectors, vectors[1:]):
            length_product = math.hypot(*u) * math.hypot(*v)
            if not length_product > 0 or not math.fsum(x*y for x, y in zip(u, v)) > .82 * length_product:
                return False
        if not math.dist(points[0], joints[-1]) > math.dist(points[0], joints[1]) + .15 * scale:
            return False
    return True


def classify(result):
    if len(result.hand_landmarks) != 1:
        return {"hand": "lost", "reason": "multiple"} if result.hand_landmarks else LOST
    gesture = result.gestures[0][0] if result.gestures and result.gestures[0] else None
    label = gesture.category_name if gesture else "None"
    confident = bool(gesture and gesture.score >= GESTURE_CONFIDENCE)
    if label == "Closed_Fist":
        return {"hand": "fist"} if confident else LOST
    if not (label == "Open_Palm" and confident):
        world = getattr(result, "hand_world_landmarks", [])
        if label not in ("None", "Open_Palm") or len(world) != 1 or not visibly_extended(world[0]):
            return LOST
    hand = result.hand_landmarks[0]
    palm = [hand[i] for i in (0, 5, 9, 13, 17)]
    point = [math.fsum(getattr(p, axis) for p in palm) / len(palm) for axis in ("x", "y")]
    return {"hand": "open", "point": point} if all(0 <= n <= 1 for n in point) else LOST


class Tracking:
    def __init__(self, camera, *, on_observation):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision

        self.camera, self.mp = camera, mp
        self.on_observation = on_observation
        self.changed, self.closed = Condition(), Event()
        self.frame, self.error = None, None
        self.finished = deque(maxlen=30)
        self.inference_ms = 0
        self.recognizer = vision.GestureRecognizer.create_from_options(
            vision.GestureRecognizerOptions(
                base_options=BaseOptions(model_asset_path=str(MODEL), delegate=BaseOptions.Delegate.CPU),
                running_mode=vision.RunningMode.VIDEO, num_hands=2, min_hand_detection_confidence=.55,
                min_hand_presence_confidence=.55, min_tracking_confidence=.65))
        self.thread = Thread(target=self.run, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.closed.set()
        with self.changed:
            self.changed.notify_all()
        self.thread.join()
        self.recognizer.close()

    def next_frame(self, after=0):
        with self.changed:
            ready = self.changed.wait_for(
                lambda: (self.frame is not None and self.frame[0] > after) or self.error or
                self.camera.error or self.closed.is_set(), timeout=.20)
            return self.frame if ready and not (self.error or self.camera.error or self.closed.is_set()) else None

    def diagnostics(self):
        with self.changed:
            now = time.monotonic()
            # Count completions in a rolling second (including gaps).
            fps = sum(stamp > now - 1 for stamp in self.finished)
            return {"fps": fps, "inference_ms": self.inference_ms,
                    "age_ms": round((now - self.frame[2]) * 1000) if self.frame else None}

    def run(self):
        previous = 0
        try:
            while not self.closed.is_set():
                raw = self.camera.next_raw(previous, timeout=.12)
                if raw is None:
                    if self.camera.error or self.camera.closed.is_set():
                        break
                    continue
                seq, image, stamp = raw
                previous = seq
                width = min(640, image.shape[1])
                height = round(width * image.shape[0] / image.shape[1])
                preview = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
                rgb = cv2.cvtColor(preview, cv2.COLOR_BGR2RGB)
                started = time.monotonic()
                result = self.recognizer.recognize_for_video(
                    self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), int(stamp * 1000))
                elapsed = round((time.monotonic() - started) * 1000)
                observation = classify(result)
                self.on_observation(seq, stamp, observation)
                # Keep the operator preview light on the Wi-Fi link without
                # reducing the resolution used for recognition.
                if width > 480:
                    preview = cv2.resize(preview, (480, round(height * 480 / width)),
                                         interpolation=cv2.INTER_AREA)
                ok, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 64])
                if not ok:
                    continue
                with self.changed:
                    self.frame = seq, jpeg.tobytes(), stamp, observation
                    self.finished.append(time.monotonic())
                    self.inference_ms = elapsed
                    self.changed.notify_all()
        except (cv2.error, RuntimeError, ValueError) as exc:
            with self.changed:
                self.error = f"Hand tracking stopped: {exc}"
                self.changed.notify_all()
