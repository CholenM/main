"""Local dashboard, single-owner USB camera and snapshot API."""

import csv
import json
import mimetypes
import time
from http.server import BaseHTTPRequestHandler
from threading import Condition, Event, Lock, Thread
from urllib.parse import urlsplit

import cv2

from . import fixture, scan
from robot.client import RobotError

FILES = ("raw.png", "rectified.png", "overlay.png", "report.md", "detections.csv",
         "errors.csv", "ground_truth.csv", "registration.json", "job.json")


class LiveCamera:
    def __init__(self, device):
        self.device = device
        self.changed = Condition()
        self.closed = Event()
        self.frame = None
        self.jpeg = None
        self.sequence = 0
        self.error = None
        self.worker = Thread(target=self.read, daemon=True)

    def __enter__(self):
        self.worker.start()
        return self

    def __exit__(self, *_):
        self.closed.set()
        with self.changed:
            self.changed.notify_all()
        self.worker.join(timeout=3)

    def read(self):
        cap = None
        try:
            cap = scan.open_camera(self.device)
            next_preview = 0
            while not self.closed.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    raise scan.ScanError("Camera stopped returning frames. Check USB and restart the dashboard.")
                now = time.monotonic()
                with self.changed:
                    self.frame = frame
                if now >= next_preview:
                    width = min(640, frame.shape[1])
                    height = round(width * frame.shape[0] / frame.shape[1])
                    preview = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
                    ok, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 78])
                    if ok:
                        with self.changed:
                            self.jpeg = jpeg.tobytes()
                            self.sequence += 1
                            self.changed.notify_all()
                    next_preview = now + .085
        except (scan.ScanError, cv2.error) as exc:
            with self.changed:
                self.error = str(exc)
                self.changed.notify_all()
        finally:
            if cap is not None:
                cap.release()

    def snapshot(self):
        with self.changed:
            if self.error or self.frame is None:
                raise scan.ScanError(self.error or "Waiting for the USB camera.")
            return self.frame.copy()

    def preview(self, previous):
        with self.changed:
            ready = self.changed.wait_for(
                lambda: self.sequence != previous or self.error or self.closed.is_set(), timeout=5)
            if not ready or self.error or self.closed.is_set():
                return None
            return self.sequence, self.jpeg


class Session:
    def __init__(self, args, replay):
        self.args = args
        self.replay = replay
        self.camera = None
        self.guard = Lock()
        self.last = None
        self.workflow = None

    def state(self):
        camera = self.camera
        return {"mode": "replay" if self.replay is not None else "live",
                "ready": self.replay is not None or bool(camera and camera.frame is not None and not camera.error),
                "camera_error": camera.error if camera else None, "busy": self.guard.locked(),
                "verify": self.args.verify,
                "last": self.last[1] if self.last else None,
                "workflow": self.workflow.describe() if self.workflow else None}

    def measure(self):
        if not self.guard.acquire(blocking=False):
            raise BlockingIOError("A scan is already running.")
        try:
            frame = self.replay if self.replay is not None else self.camera.snapshot()
            directory, passed = scan.run(self.args, frame)

            def rows(name):
                path = directory / name
                if not path.is_file():
                    return []
                with path.open(newline="") as file:
                    return list(csv.DictReader(file))

            points, errors = rows("detections.csv"), rows("errors.csv")
            radial = [float(row["radial_mm"]) for row in errors if row["radial_mm"]]
            report = (directory / "report.md").read_text()
            files = {name: f"/run/{directory.name}/{name}" for name in FILES
                     if (directory / name).is_file()}
            result = {"id": directory.name, "status": report.splitlines()[0].split(": ", 1)[-1].upper(),
                      "passed": passed, "detected": len(points), "points": points, "errors": errors,
                      "expected": sum(row["status"] != "EXTRA" for row in errors) if errors else None,
                      "mean_mm": sum(radial)/len(radial) if radial else None,
                      "max_mm": max(radial) if radial else None,
                      "message": report.split("\n\n", 1)[-1].strip() if not passed and not points else "",
                      "files": files}
            self.last = directory, result
            return result
        finally:
            self.guard.release()


class Dashboard(BaseHTTPRequestHandler):
    def __init__(self, *args, session, **kwargs):
        self.session = session
        super().__init__(*args, **kwargs)

    def log_request(self, code="-", size="-"):
        if urlsplit(self.path).path != "/api/state":
            super().log_request(code, size)

    def send_data(self, data, mime, status=200):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def json(self, data, status=200):
        self.send_data(json.dumps(data).encode(), "application/json", status)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            self.send_data((fixture.ROOT / "vision" / "dashboard.html").read_bytes(),
                           "text/html; charset=utf-8")
        elif path == "/api/state":
            if self.session.workflow:
                self.session.workflow.poll()
            self.json(self.session.state())
        elif path == "/stream.mjpg" and self.session.camera:
            self.stream(self.session.camera)
        elif path.startswith("/run/"):
            workflow = self.session.workflow
            if workflow:
                with workflow.condition:
                    directory = workflow.directory
                if directory and path == f"/run/{directory.name}/plan.json":
                    plan = directory / "plan.json"
                    if plan.is_file():
                        self.send_data(plan.read_bytes(), "application/json")
                        return
            last = self.session.last
            if last:
                directory, result = last
                filename = path.rsplit("/", 1)[-1]
                if result["files"].get(filename) == path:
                    self.send_data((directory / filename).read_bytes(),
                                   mimetypes.guess_type(filename)[0] if filename.endswith(
                                       (".png", ".json", ".csv")) else "text/plain; charset=utf-8")
                    return
            self.send_error(404)
        else:
            self.send_error(404)

    def stream(self, camera):
        first = camera.preview(0)
        if first is None:
            self.send_error(503, "Camera preview unavailable")
            return
        self.connection.settimeout(5)
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        item = first
        try:
            while item is not None and not camera.closed.is_set():
                sequence, jpeg = item
                self.wfile.write(f"--frame\r\nContent-Type: image/jpeg\r\n"
                                 f"Content-Length: {len(jpeg)}\r\n\r\n".encode() + jpeg + b"\r\n")
                item = camera.preview(sequence)
        except OSError:
            pass  # Browser switched views or disconnected.

    def do_POST(self):
        workflow = self.session.workflow
        if self.path != "/api/scan" and not (workflow and self.path.startswith("/api/robot/")):
            self.send_error(404)
            return
        origin = self.headers.get("Origin")
        if workflow and not workflow.simulated:
            port = self.server.server_port
            hosts = {f"localhost:{port}", f"127.0.0.1:{port}"}
            if self.headers.get("Host") not in hosts:
                self.json({"error": "Physical workflow is available only through loopback."}, 403)
                return
        if origin and urlsplit(origin).netloc != self.headers.get("Host"):
            self.json({"error": "Cross-origin scan requests are not allowed."}, 403)
            return
        try:
            if self.path.startswith("/api/robot/"):
                action = self.path.removeprefix("/api/robot/")
                if action == "stop":
                    workflow.stop()
                elif action == "execute":
                    try:
                        size = int(self.headers.get("Content-Length", "0"))
                        if not 1 <= size <= 256:
                            raise ValueError
                        review_id = json.loads(self.rfile.read(size))["review_id"]
                    except (ValueError, TypeError, KeyError):
                        self.json({"error": "Provide the current review_id."}, 400)
                        return
                    workflow.launch(action, review_id)
                else:
                    workflow.launch(action)
                result = workflow.describe()
            else:
                if workflow:
                    raise RobotError("Use Start to home, park and scan in robot workflow mode.")
                result = self.session.measure()
        except BlockingIOError as exc:
            self.json({"error": str(exc)}, 409)
        except scan.ScanError as exc:
            self.json({"error": str(exc)}, 503)
        except RobotError as exc:
            self.json({"error": str(exc)}, 409)
        except OSError as exc:
            self.json({"error": str(exc)}, 500)
        else:
            self.json(result)
