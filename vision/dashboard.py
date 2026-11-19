"""Local dashboard, single-owner USB camera and snapshot API."""

import csv
import json
import mimetypes
import time
from copy import copy
from http.server import BaseHTTPRequestHandler
from threading import Condition, Event, Lock, Thread
from urllib.parse import urlsplit

import cv2

from . import fixture, scan
from robot.client import RobotError
from robot.home_trace import MIME as HOME_TRACE_MIME

FILES = ("raw.png", "rectified.png", "overlay.png", "report.md", "detections.csv",
         "errors.csv", "ground_truth.csv", "registration.json", "job.json")


class LiveCamera:
    def __init__(self, device, *, encode_preview=True):
        self.device = device
        self.encode_preview = encode_preview
        self.changed = Condition()
        self.closed = Event()
        self.frame = None
        self.frame_at = None
        self.frame_sequence = 0
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
            cap = (scan.open_camera(self.device) if self.encode_preview else
                   scan.open_camera(self.device, size=(1280, 720), fps=30, buffer_size=1))
            next_preview = 0
            while not self.closed.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    raise scan.ScanError("Camera stopped returning frames. Check USB and restart the dashboard.")
                now = time.monotonic()
                with self.changed:
                    self.frame = frame
                    if not self.encode_preview:
                        self.frame_at = now
                        self.frame_sequence += 1
                        self.changed.notify_all()
                if self.encode_preview and now >= next_preview:
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

    def next_raw(self, previous, timeout):
        """Wait for the newest raw frame; intermediate frames are not queued."""
        with self.changed:
            ready = self.changed.wait_for(
                lambda: self.frame_sequence != previous or self.error or self.closed.is_set(),
                timeout)
            if not ready or self.error or self.closed.is_set():
                return None
            return self.frame_sequence, self.frame, self.frame_at


class Session:
    def __init__(self, args, replay, demo=False):
        self.args = args
        self.replay = replay
        self.camera = None
        self.guard = Lock()
        self.last = None
        self.workflow = None
        self.demo_frame = replay if demo else None
        self.connector = None

    def connect(self, data):
        if self.connector is None:
            raise RobotError("Hardware setup is not available on this page.")
        with self.guard:
            if self.workflow is not None:
                raise RobotError("A controller is already connected for this dashboard session.")
            self.workflow = self.connector(data)

    def diagnostic_frame(self):
        if self.camera is not None:
            try:
                return self.camera.snapshot()
            except scan.ScanError:
                pass
        return None

    def state(self):
        camera = self.camera
        return {"mode": "replay" if self.replay is not None else "live",
                "ready": self.replay is not None or bool(camera and camera.frame is not None and not camera.error),
                "camera_error": camera.error if camera else None, "busy": self.guard.locked(),
                "verify": self.args.verify,
                "can_connect": self.connector is not None and self.workflow is None,
                "demo_source": ("demo" if self.replay is not None else "live")
                if self.demo_frame is not None else None,
                "last": self.last[1] if self.last else None,
                "workflow": self.workflow.describe() if self.workflow else None}

    def measure(self):
        if not self.guard.acquire(blocking=False):
            raise BlockingIOError("A scan is already running.")
        try:
            if self.replay is None and self.camera is None:
                raise scan.ScanError("No live camera is available.")
            frame = self.replay if self.replay is not None else self.camera.snapshot()
            options = self.args
            if self.replay is self.demo_frame and self.demo_frame is not None:
                options = copy(self.args)
                options.replay = "built-in fixture"
            directory, passed = scan.run(options, frame)

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
    def __init__(self, *args, session, simulation=None, **kwargs):
        self.primary, self.simulation = session, simulation
        self.session = self.primary
        super().__init__(*args, **kwargs)

    def select_view(self):
        path = urlsplit(self.path).path
        self.session = self.primary
        if self.simulation and (path == "/sim" or path.startswith("/sim/")):
            self.session = self.simulation
            return path.removeprefix("/sim") or "/"
        return path

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

    def robot_body(self):
        if self.headers.get_content_type() != "application/json":
            raise ValueError
        size = int(self.headers.get("Content-Length", "0"))
        if not 1 <= size <= 1024:
            raise ValueError
        data = json.loads(self.rfile.read(size))
        if not isinstance(data, dict):
            raise ValueError
        return data

    def physical_host_allowed(self):
        port = self.server.server_port
        return self.headers.get("Host") in (f"localhost:{port}", f"127.0.0.1:{port}")

    def do_GET(self):
        path = self.select_view()
        if path == "/" or (path == "/manual" and (self.session.workflow or self.session.connector)):
            self.send_data((fixture.ROOT / "vision" / "dashboard.html").read_bytes(),
                           "text/html; charset=utf-8")
        elif path == "/api/state":
            if self.session.workflow:
                self.session.workflow.poll()
            self.json(self.session.state() | {"has_simulation": self.simulation is not None})
        elif path == "/api/robot/poses.csv" and self.session.workflow:
            workflow = self.session.workflow
            if not workflow.simulated and not self.physical_host_allowed():
                self.json({"error": "Physical workflow is available only through loopback."}, 403)
                return
            try:
                data = workflow.pose_csv()
            except (OSError, ValueError, RobotError):
                self.json({"error": "The pose log could not be exported."}, 500)
            else:
                self.send_data(data.encode(), "text/csv; charset=utf-8")
        elif path.startswith("/homing/") and self.session.workflow:
            workflow = self.session.workflow
            if not workflow.simulated and not self.physical_host_allowed():
                self.json({"error": "Physical workflow is available only through loopback."}, 403)
                return
            with workflow.condition:
                bundle = workflow.home_trace
            if bundle:
                directory, summary = bundle
                filename = path.rsplit("/", 1)[-1]
                if filename in summary["files"] and path == f"/homing/{summary['id']}/{filename}":
                    try:
                        data = (directory / filename).read_bytes()
                    except OSError:
                        self.send_error(404)
                    else:
                        self.send_data(data, HOME_TRACE_MIME[filename])
                    return
            self.send_error(404)
        elif path == "/stream.mjpg" and self.session.camera:
            self.stream(self.session.camera)
        elif path == "/demo.jpg" and self.session.demo_frame is not None:
            ok, image = cv2.imencode(".jpg", self.session.demo_frame)
            if ok:
                self.send_data(image.tobytes(), "image/jpeg")
            else:
                self.send_error(503)
        elif path.startswith("/run/"):
            workflow = self.session.workflow
            if workflow:
                with workflow.condition:
                    directory = workflow.directory
                filename = path.rsplit("/", 1)[-1]
                if (directory and filename in ("plan.json", "dry_run.json")
                        and path == f"/run/{directory.name}/{filename}"):
                    plan = directory / filename
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
        path = self.select_view()
        workflow = self.session.workflow
        if path not in ("/api/scan", "/api/connect") and not (workflow and path.startswith("/api/robot/")):
            self.send_error(404)
            return
        origin = self.headers.get("Origin")
        if (path == "/api/connect" or workflow and not workflow.simulated) and not self.physical_host_allowed():
            self.json({"error": "Physical workflow is available only through loopback."}, 403)
            return
        if origin and urlsplit(origin).netloc != self.headers.get("Host"):
            self.json({"error": "Cross-origin scan requests are not allowed."}, 403)
            return
        try:
            if path == "/api/connect":
                self.session.connect(self.robot_body())
                result = {"connected": True}
            elif path.startswith("/api/robot/"):
                action = path.removeprefix("/api/robot/")
                if action == "stop":
                    workflow.stop()
                elif action in ("execute", "jog", "z-jog", "nudge", "record-pose", "save-park",
                                "return-park", "drill", "virtual-z", "virtual-drill", "source"):
                    try:
                        data = self.robot_body()
                        if action == "execute" and not isinstance(data["review_id"], str):
                            raise ValueError
                    except (ValueError, TypeError, KeyError):
                        self.json({"error": "Invalid JSON for this robot action."}, 400)
                        return
                    workflow.launch(action, data.get("review_id"), data)
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
        except (TypeError, ValueError, KeyError):
            self.json({"error": "Invalid request body."}, 400)
        else:
            self.json(result)
