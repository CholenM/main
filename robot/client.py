"""Versioned HTTP transport; never retry an ambiguous motion command."""

import json
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

AXES = ("z", "j1", "j2")
VERSION = "scara-slave/1.0"


class RobotError(Exception):
    pass


class MotionStopped(RobotError):
    """A motion ended on a normal end-of-travel contact, not a latched fault."""


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class RobotClient:
    def __init__(self, url, token, timeout=0.6):
        parts = urlsplit(url)
        if (parts.scheme != "http" or not parts.hostname or parts.username or parts.password
                or parts.path not in ("", "/") or parts.query or parts.fragment):
            raise RobotError("Use a controller URL such as http://scara.local or http://192.168.1.10.")
        self.url = url.rstrip("/")
        self.token = token
        self.timeout = timeout
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def request(self, path, payload=None):
        headers = {"X-SCARA-Token": self.token}
        data = None if payload is None else json.dumps(payload).encode()
        if data is not None:
            headers["Content-Type"] = "application/json"
        try:
            with self.opener.open(Request(self.url + "/" + path, data, headers),
                                  timeout=self.timeout) as response:
                value = json.loads(response.read(8192))
        except HTTPError as exc:
            try:
                message = json.loads(exc.read(1024))["error"]
            except (OSError, KeyError, ValueError, TypeError):
                message = exc.reason
            raise RobotError(f"{path}: {exc.code} {message}") from exc
        except (URLError, OSError, ValueError) as exc:
            raise RobotError(f"{path}: controller unavailable ({exc})") from exc
        if not isinstance(value, dict):
            raise RobotError(f"{path}: invalid controller response")
        return value

    def status(self):
        status = self.request("status")
        if (status.get("fw") != VERSION or not isinstance(status.get("boot"), str)
                or type(status.get("command_id")) is not int
                or type(status.get("homed")) is not bool
                or type(status.get("commissioned")) is not bool
                or type(status.get("simulator")) is not bool
                or not isinstance(status.get("pos"), dict)
                or any(type(status["pos"].get(axis)) is not int for axis in AXES)
                or status.get("state") not in ("IDLE", "HOMING", "MOVING", "RECOVERING",
                                               "HOLD", "FAULT", "ESTOP")
                or type(status.get("spindle")) is not bool):
            raise RobotError("Unsupported firmware/status response.")
        return status

    def motion(self, path, payload, cancelled, timeout=120, expected_boot=None,
               expected_command=None, observe=None):
        check_cancelled(cancelled)
        before = self.status()
        if expected_boot is not None and before["boot"] != expected_boot:
            raise RobotError("Controller restarted before motion.")
        if expected_command is not None and before["command_id"] != expected_command:
            raise RobotError("The controller command changed before motion.")
        check_cancelled(cancelled)
        ack = self.request(path, {**payload, "boot": before["boot"],
                                  "command_id": before["command_id"]})
        boot, command = ack.get("boot"), ack.get("command_id")
        if (boot != before["boot"] or type(command) is not int or
                command != (before["command_id"] + 1) % 2**32):
            raise RobotError("Controller restarted or did not acknowledge the motion command.")
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            check_cancelled(cancelled)
            status = self.status()
            if observe is not None:
                observe(status)
            if status["boot"] != boot:
                raise RobotError("Controller restarted during motion.")
            if status["state"] in ("HOLD", "FAULT", "ESTOP"):
                raise RobotError(f"Controller {status['state']}: {status.get('fault') or 'no detail'}")
            if status["command_id"] != command:
                raise RobotError("The active motion command changed.")
            if status["state"] == "IDLE":
                if path == "nudge":
                    axis = payload["axis"]
                    reached = (not status["homed"] and
                               status["pos"][axis] == before["pos"][axis] + payload["delta"])
                else:
                    reached = (status["homed"] or payload.get("axis") == "z") and (
                        path != "move" or status["pos"][payload["axis"]] == payload["to"])
                if not reached:
                    if status.get("fault") == "Z_BOTTOM_LIMIT":
                        raise MotionStopped("Z stopped at the bottom switch.")
                    raise RobotError("Controller did not reach the requested datum/target.")
                return status
            if status["state"] not in ("HOMING", "MOVING"):
                raise RobotError(f"Controller {status['state']}: {status.get('fault') or 'no detail'}")
            self.request("heartbeat", {"boot": boot, "command_id": command})
            cancelled.wait(0.12)
        raise RobotError(f"{path}: completion timeout")


def check_cancelled(event):
    if event.is_set():
        raise RobotError("Workflow cancelled.")
