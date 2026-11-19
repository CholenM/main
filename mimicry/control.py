"""Translate fresh single-hand observations into one bounded jog at a time."""

import math
import time
from collections import OrderedDict
from threading import Event, Lock, Thread

from robot.client import RobotError

MAX_SAMPLE_AGE = 0.40
SETUP_WINDOW = 8.0
DEADZONE = 0.09
ENTER_ZONE = 0.12
J1_CHUNK, Z_CHUNK, Z_CAP = 8, 30, 136062


def direction(center, point, current=None):
    dx, dy = (point[i] - center[i] for i in range(2))
    distance = max(abs(dx), abs(dy))
    if distance <= DEADZONE:
        return None
    next_direction = ("right" if dx > 0 else "left") if abs(dx) > abs(dy) else (
        "down" if dy > 0 else "up")
    return next_direction if distance >= ENTER_ZONE or current == next_direction else None


def parse_palm(data):
    if data.get("hand") not in ("open", "fist", "lost"):
        raise ValueError("Invalid hand observation.")
    if data["hand"] != "open":
        return None
    xy = data.get("point")
    if (not isinstance(xy, list) or len(xy) != 2 or
            any(type(n) not in (int, float) or not 0 <= n <= 1 for n in xy)):
        raise ValueError("Invalid palm coordinates.")
    return tuple(xy)


class Control:
    def __init__(self, client, simulated, reverse_j1=False, clock=time.monotonic):
        self.client, self.simulated, self.clock = client, simulated, clock
        self.j1_right = -1 if reverse_j1 else 1
        self.lock = Lock()
        self.frames = OrderedDict()
        self.virtual_sequence = 0
        self.owner = self.center = None
        self.sample_sequence, self.sample_at = -1, -math.inf
        self.armed = False
        self.expected = self.intent = None
        self.same_direction = 0
        self.pending = self.setup_palm = None
        self.message = "Show one open palm, then set the joystick center."
        self.home_thread = None
        self.home_cancel = Event()
        self.stopping = Event()

    def frame(self, preview=None, virtual=False):
        with self.lock:
            if virtual:
                self.virtual_sequence += 1
                result = (self.virtual_sequence, b"", self.clock(), None)
            else:
                result = preview
            if result is None or self.clock() - result[2] > MAX_SAMPLE_AGE:
                return None
            self.frames[result[0]] = result[2], result[3]
            while len(self.frames) > 24:
                self.frames.popitem(last=False)
            return result

    def disarm(self, message):
        self.armed = False
        self.pending = self.setup_palm = None
        self.intent, self.same_direction = None, 0
        self.expected = None
        self.message = message

    def expire(self):
        if self.armed and self.clock() - self.sample_at > MAX_SAMPLE_AGE:
            self.disarm("Tracking timed out. Return to center and arm again.")
        if self.pending and self.clock() - self.pending[1] >= SETUP_WINDOW:
            self.disarm("No steady open palm detected. Show your whole hand and try again.")

    def setup_sample(self, point, stamp):
        action, requested_at, expected = self.pending
        if stamp < requested_at:
            return
        if (point is None or (action == "arm" and
                max(abs(a-b) for a, b in zip(self.center, point)) > DEADZONE)):
            self.setup_palm = None
            return
        previous, self.setup_palm = self.setup_palm, (point, stamp)
        if (previous is None or stamp - previous[1] > MAX_SAMPLE_AGE or
                max(abs(a-b) for a, b in zip(previous[0], point)) > DEADZONE / 2):
            return
        self.pending = self.setup_palm = None
        if action == "center":
            self.center = point
            self.message = "Center set. Keep your palm inside the box and select Arm Gestures."
        else:
            self.expected = expected
            self.armed = True
            self.message = "Armed — move your open palm away from center."

    def home_busy(self):
        return self.home_thread is not None and self.home_thread.is_alive()

    @staticmethod
    def controller_id(status):
        return status["boot"], status["command_id"]

    def checked_status(self):
        status = self.client.status()
        if status["simulator"] != self.simulated:
            raise RobotError("Controller does not match the dashboard mode.")
        return status

    @staticmethod
    def require_ready(status, referenced=False):
        mimic = status.get("mimicry", {})
        upper = mimic.get("z_max_steps")
        if (not status["commissioned"] or mimic.get("enabled") is not True or
                type(upper) is not int or not 0 < upper <= Z_CAP):
            raise RobotError("Mimicry is not enabled in the commissioned controller profile.")
        if status["spindle"]:
            raise RobotError("Drill relay is active. Motion is disabled.")
        if referenced and mimic.get("z_referenced") is not True:
            raise RobotError("Bring Z to the bottom switch and select Set Z Zero.")
        if status["state"] not in ("IDLE", "MOVING"):
            raise RobotError(f"Controller is {status['state']}.")

    def command(self, path, status, **payload):
        boot, command = self.controller_id(status)
        ack = self.client.request(path, dict(payload, boot=boot, command_id=command))
        expected = boot, (command + 1) % 2**32
        if (ack.get("boot"), ack.get("command_id")) != expected:
            raise RobotError("Controller did not acknowledge the expected command.")
        return expected

    def observe(self, owner, data):
        with self.lock:
            self.expire()
            if owner != self.owner:
                if self.armed or self.pending or self.clock() - self.sample_at <= MAX_SAMPLE_AGE:
                    raise RobotError("Another dashboard tab is active.")
                self.owner, self.center, self.sample_sequence = owner, None, -1
            sequence = data.get("frame")
            if type(sequence) is not int:
                raise ValueError("Missing frame sequence.")
            stamp, measurement = self.frames.get(sequence, (-math.inf, None))
            if sequence <= self.sample_sequence or self.clock() - stamp > MAX_SAMPLE_AGE:
                return
            observation = measurement if measurement is not None else data
            point = parse_palm(observation)
            self.sample_sequence, self.sample_at = sequence, stamp
            if self.pending:
                if not self.stopping.is_set():
                    self.setup_sample(point, stamp)
                return
            if not self.armed:
                return
            if point is None:
                self.disarm("Fist detected. Arm again to resume." if observation["hand"] == "fist" else
                            "Open palm not tracked. Return to center and arm again.")
                return
            intent = direction(self.center, point, self.intent)
            self.same_direction = self.same_direction + 1 if intent == self.intent else 1
            self.intent = intent
            if intent is None:
                self.message = "Centered — no new motion."
            elif self.same_direction >= 2:
                try:
                    self.jog(intent)
                except (RobotError, KeyError, TypeError, ValueError) as exc:
                    self.disarm(f"Jog stopped: {exc}")

    def jog(self, intent):
        status = self.checked_status()
        self.require_ready(status, referenced=True)
        if self.expected != self.controller_id(status):
            raise RobotError("Controller command or boot changed; arm again.")
        if status["state"] == "MOVING":
            return  # No queue, no retries, and no heartbeat for a jog chunk.
        if self.stopping.is_set() or self.clock() - self.sample_at > MAX_SAMPLE_AGE:
            self.disarm("Tracking stopped or the frame arrived too late.")
            return
        axis = "j1" if intent in ("left", "right") else "z"
        bottom = status["limits"]["z"] == 0
        if axis == "j1" and (not status["homed"] or bottom):
            self.message = "Lift Z clear and home J1 + J2 before jogging J1."
            return
        if axis == "z" and bottom and intent == "down":
            self.message = "Z bottom switch is active."
            return
        step = (self.j1_right * (1 if intent == "right" else -1) * J1_CHUNK if axis == "j1"
                else (1 if intent == "up" else -1) * Z_CHUNK)
        low, high = status["bounds"][axis]
        pos = status["pos"][axis]
        delta = max(low, min(high, pos + step)) - pos
        if not delta:
            self.message = f"{axis.upper()} travel limit reached."
            return
        self.expected = self.command("mimic-jog", status, axis=axis, delta=delta)
        self.message = f"Jogging {axis.upper()} {intent}."

    def action(self, owner, action):
        if action == "estop":
            self.emergency_stop()
            return
        with self.lock:
            self.expire()
            if action == "disarm":
                if self.owner == owner:
                    self.disarm("Disarmed.")
                return
            if self.stopping.is_set() or self.home_busy():
                raise RobotError("Stop or homing is in progress.")
            if self.armed:
                raise RobotError("Disarm before changing settings.")
            if self.pending:
                raise RobotError("Palm setup is already waiting. Select Disarm to cancel.")
            if action in ("center", "arm"):
                if owner != self.owner:
                    if self.clock() - self.sample_at <= MAX_SAMPLE_AGE:
                        raise RobotError("Another dashboard tab is active.")
                    self.owner, self.center, self.sample_sequence = owner, None, -1
                if action == "center":
                    self.pending = ("center", self.clock(), None)
                    self.message = "Hold one open palm steady where you want the joystick center."
                    return
                if self.center is None:
                    raise RobotError("Set the joystick center first.")
            status = self.checked_status()
            if action == "reset":
                self.client.request("reset", {})
                self.message = "Reset. Set Z Zero again before arming."
                return
            self.require_ready(status, referenced=action != "zero")
            if status["state"] != "IDLE":
                raise RobotError("Wait until the controller is idle.")
            if action == "zero":
                self.command("z-zero", status)
                self.message = "Z zero recorded. Use an upward gesture to clear the switch."
            elif action == "arm":
                if self.stopping.is_set():
                    raise RobotError("Stop is in progress.")
                self.pending = ("arm", self.clock(), self.controller_id(status))
                self.message = "Hold your open palm steady inside the center box to arm."
            elif action == "home":
                if status["limits"]["z"] != 1:
                    raise RobotError("Lift Z clear of the bottom switch first.")
                self.home_cancel.clear()
                self.message = "Homing J1 and J2."
                self.home_thread = Thread(target=self.home, args=(status,), daemon=True)
                self.home_thread.start()

    def home(self, status):
        try:
            boot, command = self.controller_id(status)
            self.client.motion("home", {}, self.home_cancel,
                               expected_boot=boot, expected_command=command)
            message = "Rotary joints homed. Select Arm to use J1 and Z."
        except RobotError as exc:
            message = f"Homing stopped: {exc}"
        with self.lock:
            if not self.home_cancel.is_set():
                self.message = message

    def emergency_stop(self):
        self.stopping.set()
        self.home_cancel.set()
        try:
            self.client.request("estop", {})
        except RobotError:
            with self.lock:
                self.disarm("Network stop not confirmed. Use the hardwired E-stop.")
            raise
        else:
            with self.lock:
                self.disarm("Network stop latched. Reset and set Z zero again.")
        finally:
            self.stopping.clear()

    def describe(self, owner):
        try:
            status, error = self.checked_status(), None
        except RobotError as exc:
            status, error = None, str(exc)
        with self.lock:
            if error and (self.armed or (self.pending and self.pending[0] == "arm")):
                self.disarm("Controller link lost. Arm again after reconnecting.")
            self.expire()
            return {"robot": status, "error": error, "armed": self.armed,
                    "owner": owner == self.owner, "homing": self.home_busy(),
                    "pending": self.pending[0] if self.pending else None,
                    "center": self.center if owner == self.owner else None,
                    "intent": self.intent, "message": self.message}
