"""Standalone loopback dashboard for hand-driven J1/Z jogging."""

import argparse
import json
import os
import re
import secrets
import signal
import subprocess
import sys
from contextlib import ExitStack
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot import sim
from robot.client import RobotClient, RobotError
from vision import scan
from vision.cli import camera_lock, exclusive
from vision.dashboard import LiveCamera

from . import assets
from .control import Control, Z_CAP
from .tracking import Tracking

ACTIONS = {"center", "arm", "disarm", "zero", "home", "reset", "estop"}


class Dashboard(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    disable_nagle_algorithm = True
    timeout = 5

    def __init__(self, *args, control, tracking, secret, **kwargs):
        self.control, self.tracking, self.secret = control, tracking, secret
        super().__init__(*args, **kwargs)

    def log_message(self, *_):
        pass

    def reply(self, code, data=b"", mime="application/json", **headers):
        self.send_response(code)
        self.send_header("Content-Type", mime)
        if code != 204:
            self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy",
                         "default-src 'self'; "
                         "style-src 'self' 'unsafe-inline'; frame-ancestors 'none'; base-uri 'none'")
        for name, value in headers.items():
            self.send_header(name, str(value))
        self.end_headers()
        if data:
            self.wfile.write(data)

    def json(self, code, data):
        self.reply(code, json.dumps(data).encode())

    def request_owner(self):
        host = self.headers.get("Host", "")
        if not re.fullmatch(r"(?:localhost|127\.0\.0\.1)(?::[0-9]{1,5})?", host):
            raise ValueError("The dashboard is available only on loopback.")
        return self.headers.get("X-Mimicry-Client", "")

    def authorize(self, owner):
        if (not re.fullmatch(r"[0-9a-f-]{36}", owner) or
                self.headers.get("X-Mimicry-Key") != self.secret):
            raise PermissionError("Reload the Mimicry dashboard.")

    def do_GET(self):
        try:
            owner = self.request_owner()
            if self.path == "/":
                page = (assets.ROOT / "dashboard.html").read_text()
                page = page.replace("__MIMICRY_KEY__", self.secret)
                page = page.replace("__MIMICRY_VIRTUAL__", "1" if self.tracking is None else "0")
                self.reply(200, page.encode(), "text/html; charset=utf-8")
            elif self.path == "/ui.mjs":
                self.reply(200, (assets.ROOT / "ui.mjs").read_bytes(), "text/javascript")
            elif self.path == "/api/status":
                self.authorize(owner)
                status = self.control.describe(owner)
                status["camera_error"] = (self.tracking.camera.error or self.tracking.error
                                          if self.tracking else None)
                status["tracking"] = self.tracking.diagnostics() if self.tracking else None
                self.json(200, status)
            elif match := re.fullmatch(r"/api/frame(?:\?after=([0-9]{1,18}))?", self.path):
                self.authorize(owner)
                after = int(match[1] or 0)
                frame = self.control.frame(self.tracking.next_frame(after) if self.tracking else None,
                                           virtual=self.tracking is None)
                if frame is None:
                    self.reply(204)
                else:
                    sequence, jpeg, _, observation = frame
                    self.reply(200, jpeg, "image/jpeg" if jpeg else "application/octet-stream",
                               **{"X-Frame": sequence, "X-Hand-Observation": json.dumps(observation)})
            else:
                self.json(404, {"error": "Not found."})
        except (ValueError, PermissionError) as exc:
            self.json(403, {"error": str(exc)})
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        body_read = False
        try:
            owner = self.request_owner()
            self.authorize(owner)
            size = int(self.headers.get("Content-Length", "0"))
            if not 1 <= size <= 512 or self.headers.get_content_type() != "application/json":
                raise ValueError("Expected a short JSON request.")
            raw = self.rfile.read(size)
            body_read = True
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError("Expected a JSON object.")
            if self.path == "/api/presence" and self.tracking is not None:
                self.control.presence(owner, data.get("frame"))
            elif self.path == "/api/observe" and self.tracking is None:
                self.control.observe(owner, data)
            elif self.path.startswith("/api/") and self.path[5:] in ACTIONS:
                self.control.action(owner, self.path[5:])
            else:
                self.json(404, {"error": "Not found."})
                return
            self.json(200, {"ok": True})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except PermissionError as exc:
            self.json(403, {"error": str(exc)})
        except (ValueError, RobotError) as exc:
            self.json(409, {"error": str(exc)})
        finally:
            # A rejected request can still leave bytes on a keep-alive connection.
            if not body_read:
                self.close_connection = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default=os.environ.get("SCARA_CAMERA", "/dev/video0"))
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--robot-url", help="Physical controller (default: local simulator)")
    parser.add_argument("--reverse-j1", action="store_true", help="Map screen right to negative J1")
    parser.add_argument("--no-camera", action="store_true", help="Simulator-only keyboard joystick")
    parser.add_argument("--install-model", action="store_true", help="Install MediaPipe and the hand model, then exit")
    args = parser.parse_args()
    if args.no_camera and args.robot_url:
        parser.error("--no-camera is simulator-only")
    try:
        if args.install_model:
            assets.install()
            print(f"MediaPipe installed; model: {assets.MODEL}")
            return 0
        if not args.no_camera and not assets.MODEL.is_file():
            raise RobotError("Run ./startmimicry.sh --install-model first (one-time download), "
                             "or use --no-camera to try the keyboard simulator.")
        signal.signal(signal.SIGTERM, interrupt)
        with ExitStack() as stack:
            stack.enter_context(exclusive(".viewer.lock", "Stop the drilling or Mimicry dashboard first."))
            if args.robot_url:
                url, token = args.robot_url, os.environ.get("SCARA_ROBOT_TOKEN", "")
                if not token:
                    raise RobotError("Set SCARA_ROBOT_TOKEN for the physical controller.")
            else:
                url, token = stack.enter_context(sim.server(controller=sim.Controller(
                    mimic_z_max=Z_CAP, move_seconds=0.07)))
            control = Control(RobotClient(url, token), not args.robot_url, args.reverse_j1,
                              live=not args.no_camera)
            tracking = None
            if not args.no_camera:
                stack.enter_context(camera_lock())
                camera = stack.enter_context(LiveCamera(args.camera, encode_preview=False))
                tracking = stack.enter_context(Tracking(camera, on_observation=control.observe_live))
            handler = partial(Dashboard, control=control, tracking=tracking, secret=secrets.token_hex(32))
            with ThreadingHTTPServer(("127.0.0.1", args.port), handler) as dashboard:
                print(f"Mimicry: http://127.0.0.1:{dashboard.server_port}/ "
                      f"({'hardware' if args.robot_url else 'simulator'}, Ctrl-C to stop)", flush=True)
                try:
                    dashboard.serve_forever()
                finally:
                    control.close()
    except KeyboardInterrupt:
        return 0
    except ImportError as exc:
        print(f"Mimicry: {exc}. Run ./startmimicry.sh --install-model.", file=sys.stderr)
        return 1
    except (RobotError, scan.ScanError, OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"Mimicry: {exc}", file=sys.stderr)
        return 1
    return 0


def interrupt(*_):
    raise KeyboardInterrupt


if __name__ == "__main__":
    sys.exit(main())
