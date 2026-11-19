"""HTTP simulator for the same bounded-Z and drill-relay controller contract."""

import argparse
import json
import os
import time
import uuid
from contextlib import contextmanager
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock, Thread

from .client import AXES, VERSION

BOUNDS = {"z": (-2000, 2000), "j1": (-10000, 10000), "j2": (-10000, 10000)}


class Controller:
    def __init__(self, move_seconds=0.20, lease_seconds=2.0, drill_seconds=2.5, mimic_z_max=0):
        self.lock = Lock()
        self.boot = uuid.uuid4().hex
        self.command = 0
        self.pos = dict.fromkeys(AXES, 0)
        self.target = self.pos.copy()
        self.state, self.homed, self.fault = "IDLE", False, None
        self.drill = False
        self.drill_until = 0
        self.z_bottom = False
        self.finish = self.lease = 0
        self.nudge_remaining = 30
        self.move_seconds, self.lease_seconds = move_seconds, lease_seconds
        self.drill_seconds = drill_seconds
        self.mimic_z_max = mimic_z_max
        self.z_referenced = self.z_must_rezero = self.mimic_chunk = False

    def handle(self, path, body=None):
        with self.lock:
            now = time.monotonic()
            if self.state in ("HOMING", "MOVING"):
                if self.lease <= min(now, self.finish):
                    self.halt("FAULT", "HEARTBEAT")
                elif self.finish <= now:
                    self.pos = self.target.copy()
                    if self.state == "HOMING":
                        self.homed = True
                    self.state = "IDLE"
                    self.mimic_chunk = False
            if self.drill and self.drill_until <= now:
                self.drill = False
            bottom = self.z_bottom or (self.mimic_z_max > 0 and self.pos["z"] <= 0)
            bounds = BOUNDS | ({"z": (0, self.mimic_z_max)} if self.z_referenced else {})
            if path == "/status" and body is None:
                return 200, {"fw": VERSION, "boot": self.boot, "command_id": self.command,
                             "state": self.state, "homed": self.homed, "pos": self.pos.copy(),
                             "target": self.target.copy(), "limits": {"z": int(not bottom), "j1": 1, "j2": 1},
                             "spindle": self.drill, "spindle_gpio": 23,
                             "drill_lease_ms": 2500, "fault": self.fault, "commissioned": True,
                               "simulator": True, "z_mode": "bounded", "bounds": bounds,
                               "mimicry": {"enabled": self.mimic_z_max > 0,
                                           "z_referenced": self.z_referenced, "z_max_steps": self.mimic_z_max},
                              "prehome_nudge": {"enabled": True, "step": 10,
                                                "remaining": self.nudge_remaining},
                              "home_axis": "j1" if self.state == "HOMING" else None,
                              "home_phase": "SEEK" if self.state == "HOMING" else None}
            if body is None:
                return 404, {"error": "UNKNOWN_ENDPOINT"}
            if path == "/heartbeat":
                if (body.get("boot"), body.get("command_id")) != (self.boot, self.command):
                    return 409, {"error": "COMMAND_CHANGED"}
                if self.state in ("HOMING", "MOVING") and not self.mimic_chunk:
                    self.lease = now + self.lease_seconds
                if self.drill:
                    self.drill_until = now + self.drill_seconds
                return 200, self.ack()
            if path == "/drill" and type(body.get("on")) is bool:
                if not body["on"]:
                    self.drill = False
                    self.drill_until = 0
                elif body.get("boot") != self.boot:
                    return 409, {"error": "COMMAND_CHANGED"}
                elif self.state != "IDLE":
                    return 409, {"error": "INVALID_STATE_OR_COMMAND"}
                else:
                    self.drill = True
                    self.drill_until = now + self.drill_seconds
                return 200, self.ack()
            if path == "/spindle" and type(body.get("on")) is bool:
                self.drill = False
                return (409, {"error": "SPINDLE_DISABLED"}) if body["on"] else (200, self.ack())
            if path == "/stop" and self.state not in ("FAULT", "ESTOP"):
                self.halt("HOLD", "STOP")
                return 200, self.ack()
            if path == "/estop":
                self.halt("ESTOP", "ESTOP")
                return 200, self.ack()
            if path == "/reset" and self.state in ("HOLD", "FAULT", "ESTOP"):
                self.halt("IDLE", None)
                return 200, self.ack()
            if path in ("/z-zero", "/mimic-jog") and self.state == "IDLE":
                if (body.get("boot"), body.get("command_id")) != (self.boot, self.command):
                    return 409, {"error": "COMMAND_CHANGED"}
                if not self.mimic_z_max or self.drill:
                    return 409, {"error": "MIMICRY_DISABLED" if not self.mimic_z_max else "DRILL_ACTIVE"}
                if path == "/z-zero":
                    if not bottom:
                        return 409, {"error": "Z_BOTTOM_SWITCH_NOT_ACTIVE"}
                    self.pos["z"] = self.target["z"] = 0
                    self.z_referenced, self.z_must_rezero, self.fault = True, False, None
                    self.command = (self.command + 1) % 2**32
                    return 200, self.ack()
                axis, delta = body.get("axis"), body.get("delta")
                if (axis not in ("j1", "z") or type(delta) is not int or
                        not 1 <= abs(delta) <= (30 if axis == "z" else 8)):
                    return 400, {"error": "INVALID_MIMIC_JOG"}
                if not self.z_referenced:
                    return 409, {"error": "Z_NOT_REFERENCED"}
                target = self.pos[axis] + delta
                if not bounds[axis][0] <= target <= bounds[axis][1]:
                    return 409, {"error": "MIMIC_TRAVEL_LIMIT"}
                if axis == "j1" and (not self.homed or bottom):
                    return 409, {"error": "NOT_HOMED_OR_SAFE_Z"}
                if axis == "z" and bottom and delta < 0:
                    return 409, {"error": "Z_BOTTOM_LIMIT"}
                self.target = self.pos | {axis: target}
                self.state, self.fault, self.mimic_chunk = "MOVING", None, True
                self.finish, self.lease = now+self.move_seconds, now+0.25
                self.command = (self.command + 1) % 2**32
                return 202, self.ack()
            if path in ("/home", "/move", "/nudge") and self.state == "IDLE":
                if (body.get("boot"), body.get("command_id")) != (self.boot, self.command):
                    return 409, {"error": "COMMAND_CHANGED"}
                if path == "/home":
                    if self.mimic_z_max and bottom:
                        return 409, {"error": "Z_BOTTOM_LIMIT"}
                    self.state, self.homed = "HOMING", False
                    self.drill, self.drill_until = False, 0
                    self.target = self.pos | {"j1": 0, "j2": 0}
                elif path == "/nudge":
                    axis, delta = body.get("axis"), body.get("delta")
                    if (axis not in ("j1", "j2") or type(delta) is not int or
                            abs(delta) != 10 or body.get("clearance_confirmed") is not True):
                        return 400, {"error": "INVALID_NUDGE_OR_CLEARANCE"}
                    if self.homed:
                        return 409, {"error": "ALREADY_HOMED"}
                    if self.nudge_remaining < 10:
                        return 409, {"error": "NUDGE_BUDGET_EXHAUSTED"}
                    self.nudge_remaining -= 10
                    self.drill, self.drill_until = False, 0
                    self.state = "MOVING"
                    self.target = self.pos | {axis: self.pos[axis] + delta}
                else:
                    axis, target, feed = (body.get(k) for k in ("axis", "to", "feed"))
                    if (axis not in AXES or type(target) is not int or type(feed) is not int
                            or not 1000 <= feed <= 20000
                            or not bounds[axis][0] <= target <= bounds[axis][1]):
                        return 400, {"error": "INVALID_TARGET_OR_FEED"}
                    if axis != "z" and not self.homed:
                        return 409, {"error": "NOT_HOMED_OR_SAFE_Z"}
                    if axis == "z" and self.z_must_rezero:
                        return 409, {"error": "Z_NOT_REFERENCED"}
                    self.drill, self.drill_until = False, 0
                    if axis == "z" and self.z_bottom and target < self.pos["z"]:
                        self.state, self.fault = "IDLE", "Z_BOTTOM_LIMIT"
                        self.target = self.pos.copy()
                    else:
                        self.state, self.fault = "MOVING", None
                        self.target = self.pos | {axis: target}
                self.command = (self.command + 1) % 2**32
                self.finish, self.lease = now+self.move_seconds, now+self.lease_seconds
                return 202, self.ack()
            return 409, {"error": "INVALID_STATE_OR_COMMAND"}

    def ack(self):
        return {"boot": self.boot, "command_id": self.command}

    def halt(self, state, fault):
        self.command = (self.command + 1) % 2**32
        self.state, self.fault, self.homed = state, fault, False
        self.drill, self.drill_until = False, 0
        self.target = self.pos.copy()
        self.z_must_rezero |= self.z_referenced
        self.z_referenced = self.mimic_chunk = False


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, controller, token, **kwargs):
        self.controller, self.token = controller, token
        super().__init__(*args, **kwargs)

    def log_message(self, *_):
        pass

    def respond(self, status, value):
        data = json.dumps(value).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def dispatch(self, post):
        if self.headers.get("X-SCARA-Token") != self.token:
            self.respond(403, {"error": "UNAUTHORIZED"})
            return
        body = None
        if post:
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 1 <= length <= 512:
                    raise ValueError
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError
            except (ValueError, TypeError):
                self.respond(400, {"error": "INVALID_JSON"})
                return
        self.respond(*self.controller.handle(self.path, body))

    def do_GET(self):
        self.dispatch(False)

    def do_POST(self):
        self.dispatch(True)


@contextmanager
def server(port=0, controller=None):
    token = os.urandom(16).hex()
    handler = partial(Handler, controller=controller or Controller(), token=token)
    with ThreadingHTTPServer(("127.0.0.1", port), handler) as http:
        thread = Thread(target=http.serve_forever, daemon=True)
        thread.start()
        try:
            yield f"http://127.0.0.1:{http.server_port}", token
        finally:
            http.shutdown()
            thread.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    options = parser.parse_args()
    with server(options.port) as (url, token):
        print(f"Simulator: {url}, X-SCARA-Token: {token}", flush=True)
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            pass
