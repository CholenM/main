"""Start → home/park → scan → approve → spindle-off motion → rescan."""

import json
import math
import threading

from . import poses
from .client import MotionStopped, RobotError, check_cancelled
from .home_trace import HomeTrace

JOINTS = ("j1", "j2")
JOG_STEPS = (10, 25, 50)
Z_JOG_STEPS = (*JOG_STEPS, 100, 500, 1000)
MANUAL_FEED_US = 5000
READY_STAGES = ("IDLE", "DONE", "REVIEW")


class Workflow:
    def __init__(self, client, session, planner=None, park=None, simulated=False,
                 enable_dry_run=False, budget=0.5, enable_manual=False, pose_log=None,
                 manual_only=False):
        self.client, self.session, self.planner = client, session, planner
        self.park, self.simulated = park, simulated
        self.manual_only = manual_only
        self.enable_dry_run, self.budget = (enable_dry_run or simulated) and not manual_only, budget
        self.enable_manual = enable_manual or simulated or manual_only
        self.pose_log = pose_log
        self.trace_root = pose_log.parent if pose_log else None
        self.home_trace, self.home_trace_error = None, None
        self.condition = threading.RLock()
        self.cancelled = threading.Event()
        self.worker = None
        self.stage, self.message = "IDLE", ""
        self.status, self.link_error = None, None
        self.before, self.plan, self.boot = None, [], None
        self.directory = None
        self.virtual_z, self.virtual_drill = 0, False
        self.drill_on, self.drill_stop, self.drill_keepalive = False, threading.Event(), None

    def describe(self):
        with self.condition:
            tool_xy = None
            status = self.status
            if self.planner and status and status["homed"]:
                pair = [status["pos"][axis] for axis in JOINTS]
                try:
                    self.planner.contains(pair)
                except RobotError:
                    pass
                else:
                    tool_xy = self.planner.forward(pair).tolist()
            return {"stage": self.stage, "message": self.message, "status": self.status,
                    "link_error": self.link_error, "simulated": self.simulated,
                    "park_configured": self.park is not None,
                    "park_steps": self.park,
                    "manual_enabled": self.enable_manual,
                    "manual_only": self.manual_only,
                    "pose_logging": self.pose_log is not None,
                    "home_trace": self.home_trace[1] if self.home_trace else None,
                    "home_trace_error": self.home_trace_error,
                    "busy": bool(self.worker and self.worker.is_alive()),
                    "review_id": self.before["id"] if self.before else None,
                    "plan": [dict(p) for p in self.plan],
                    "plan_url": (f"/run/{self.directory.name}/plan.json" if self.directory
                                 and self.plan else None),
                    "dry_run_url": (f"/run/{self.directory.name}/dry_run.json"
                                    if self.directory and self.stage == "DONE" else None),
                    "virtual_tools": {"z_steps": self.virtual_z, "drill": self.virtual_drill}
                    if self.simulated else None,
                    "drill": self.drill_on,
                    "tool_xy_mm": tool_xy,
                    "residual_mm": self.planner.residual_mm if self.planner else None,
                    "can_execute": self.stage == "REVIEW" and self.enable_dry_run and bool(self.plan)}

    def poll(self):
        try:
            status = self.client.status()
            self.status, self.link_error = status, None
            if self.drill_on and not status.get("spindle"):
                self.stop_drill_keepalive()
        except RobotError as exc:
            self.status, self.link_error = None, str(exc)
            self.stop_drill_keepalive()

    def transition(self, stage, message=""):
        with self.condition:
            check_cancelled(self.cancelled)
            self.stage, self.message = stage, message

    def clear_plan(self):
        self.before, self.plan, self.boot, self.directory = None, [], None, None

    def manual_status(self, data, *, prehome=False, axis=None):
        status = self.client.status()
        self.check_controller(status)
        if status["state"] != "IDLE":
            raise RobotError("The controller must be idle.")
        if not prehome and axis != "z" and not status["homed"]:
            raise RobotError("Home J1/J2 before joint moves.")
        if (data.get("boot") != status["boot"] or type(data.get("command_id")) is not int or
                data["command_id"] != status["command_id"]):
            raise RobotError("The displayed pose is stale. Wait for the live readout and try again.")
        limits = status.get("limits")
        if (not isinstance(limits, dict) or
                any(type(limits.get(name)) is not int or limits[name] not in (0, 1)
                    for name in ("z", *JOINTS))):
            raise RobotError("The controller did not report all three limit inputs.")
        if axis != "z":
            if limits["z"] != 1:
                raise RobotError("Raise Z off the bottom switch before moving J1/J2.")
            for name in JOINTS:
                if limits[name] != 1:
                    raise RobotError(f"{name.upper()} limit is pressed.")
        return status

    @staticmethod
    def check_bounds(status, targets):
        joint_bounds = status.get("bounds")
        for axis, target in targets.items():
            bounds = joint_bounds.get(axis) if isinstance(joint_bounds, dict) else None
            if (not isinstance(bounds, list) or len(bounds) != 2 or
                    any(type(value) is not int for value in bounds)):
                raise RobotError("Firmware with joint bounds is required for manual operation.")
            if not bounds[0] <= target <= bounds[1]:
                raise RobotError(f"{axis.upper()} target is outside the commissioned step bounds.")

    def launch(self, action, review_id=None, data=None):
        with self.condition:
            if self.manual_only and action in ("start", "execute"):
                raise RobotError("Scan and execute are unavailable in manual-only mode.")
            if self.worker and self.worker.is_alive():
                raise RobotError("A workflow action is still finishing.")
            if action == "drill":
                if not self.enable_manual and not self.simulated:
                    raise RobotError("Hardware manual controls are not enabled.")
                if not (data and data.get("on") is False):
                    if self.stage not in READY_STAGES:
                        raise RobotError("Drill control requires an idle workflow.")
                self.configure_drill(data or {})
                return
            if action in ("virtual-z", "virtual-drill", "source"):
                if not self.simulated:
                    raise RobotError("Virtual controls never command physical hardware.")
                if self.stage not in READY_STAGES:
                    raise RobotError("Virtual controls require an idle simulator.")
                self.configure_simulation(action, data or {})
                return
            if action in ("start", "home") and self.stage in READY_STAGES:
                if action == "start" and self.park is None:
                    raise RobotError("Configure and verify the camera-clear park pose first.")
                if action == "start" and not self.session.state()["ready"]:
                    raise RobotError("Camera is not ready.")
                self.clear_plan()
                self.virtual_z, self.virtual_drill = 0, False
                self.stop_drill_keepalive()
                self.cancelled.clear()
                self.stage, self.message = "HOMING", ""
                work = self.prepare if action == "start" else self.home_only
            elif (action == "execute" and self.stage == "REVIEW" and self.before
                  and review_id == self.before["id"] and self.plan and self.enable_dry_run):
                self.stage, self.message = "CHECKING", ""
                self.virtual_z, self.virtual_drill = 0, False
                self.stop_drill_keepalive()
                work = self.execute
            elif action == "reset" and (self.stage in ("FAULT", "ESTOP") or
                    self.stage in ("IDLE", "DONE", "REVIEW") and self.status and
                    self.status["state"] in ("FAULT", "ESTOP", "HOLD")):
                self.cancelled.clear()
                self.stop_drill_keepalive()
                self.stage, self.message = "RESETTING", ""
                work = self.reset
            elif action in ("jog", "z-jog", "nudge", "save-park", "return-park",
                            "record-pose") and self.stage in READY_STAGES:
                if action != "record-pose" and not self.enable_manual:
                    raise RobotError("Hardware manual controls are not enabled.")
                data = data if data is not None else {}
                axis, delta = data.get("axis"), data.get("delta")
                if action in ("jog", "z-jog", "nudge"):
                    if action == "z-jog":
                        if axis != "z" or type(delta) is not int or abs(delta) not in Z_JOG_STEPS:
                            raise RobotError("Select Z and an allowed jog increment.")
                    else:
                        allowed = (10,) if action == "nudge" else JOG_STEPS
                        if axis not in JOINTS or type(delta) is not int or abs(delta) not in allowed:
                            raise RobotError("Select J1 or J2 and an allowed jog increment.")
                status = self.manual_status(data, prehome=action == "nudge", axis=axis)
                if action == "record-pose":
                    if self.pose_log is None:
                        raise RobotError("Pose logging is not configured.")
                    row = poses.observation(status, data)
                    poses.append(self.pose_log, row)
                    self.message = f"Pose recorded: J1 {row['j1_steps']}, J2 {row['j2_steps']} steps."
                    return
                if action == "nudge":
                    if status["homed"]:
                        raise RobotError("Use normal jog controls once homed.")
                    nudge = status.get("prehome_nudge")
                    if not isinstance(nudge, dict) or nudge.get("enabled") is not True:
                        raise RobotError("Pre-home nudges are not commissioned in the firmware.")
                    if nudge.get("step") != 10 or type(nudge.get("remaining")) is not int or nudge["remaining"] < 10:
                        raise RobotError("No pre-home nudge budget remaining.")
                    if data.get("clearance_confirmed") is not True:
                        raise RobotError("Confirm physical clearance before each pre-home nudge.")
                    targets = None
                elif action == "jog":
                    targets = {axis: status["pos"][axis] + delta}
                elif action == "z-jog":
                    targets = {"z": status["pos"]["z"] + delta}
                elif action == "return-park":
                    if self.park is None:
                        raise RobotError("No camera-clear park pose is configured.")
                    targets = self.park
                else:
                    targets = {axis: status["pos"][axis] for axis in JOINTS}
                if targets is not None:
                    self.check_bounds(status, targets)
                self.clear_plan()
                if action == "save-park":
                    self.park = targets
                    self.stage, self.message = "IDLE", "Park saved for this session; pending plan cleared."
                    return
                self.boot = status["boot"]
                self.cancelled.clear()
                self.stage, self.message = "MANUAL", "Manual move in progress."
                if action == "nudge":
                    work = lambda: self.prehome_nudge(status, axis, delta)
                else:
                    work = lambda: self.manual_move(status, targets)
            else:
                raise RobotError("Action unavailable in this state or review no longer current.")
            self.worker = threading.Thread(target=self.run, args=(work,), daemon=True)
            self.worker.start()

    def stop_drill_keepalive(self):
        self.drill_stop.set()
        self.drill_on = False

    def configure_drill(self, data):
        on = data.get("on")
        if type(on) is not bool:
            raise RobotError("Select a drill state.")
        status = self.client.status()
        if status["simulator"] != self.simulated:
            raise RobotError("Controller mode mismatch.")
        if on:
            if (data.get("boot") != status["boot"] or type(data.get("command_id")) is not int or
                    data["command_id"] != status["command_id"]):
                raise RobotError("The displayed pose is stale. Wait for the live readout and try again.")
            if status["state"] != "IDLE":
                raise RobotError("The controller must be idle to turn the drill on.")
        self.client.request("drill", {"on": on, "boot": status["boot"]})
        try:
            self.status = self.client.status()
        except RobotError:
            pass
        if on:
            self.drill_on = True
            if not (self.drill_keepalive and self.drill_keepalive.is_alive()):
                self.drill_stop.clear()
                self.drill_keepalive = threading.Thread(target=self.keep_drill_alive, daemon=True)
                self.drill_keepalive.start()
            self.message = ("Drill on. It stops on Stop, fault, link loss, or heartbeat expiry."
                            if not self.simulated else "Virtual drill on.")
        else:
            self.stop_drill_keepalive()
            self.message = "Drill off."

    def keep_drill_alive(self):
        while not self.drill_stop.wait(0.5):
            try:
                status = self.client.status()
                if not status.get("spindle"):
                    break
                self.client.request("heartbeat", {"boot": status["boot"],
                                                   "command_id": status["command_id"]})
            except RobotError:
                break
        with self.condition:
            self.drill_on = False

    def configure_simulation(self, action, data):
        if action == "source":
            source = data.get("source")
            if self.session.demo_frame is None or source not in ("demo", "live"):
                raise RobotError("Choose Built-in fixture or Live camera.")
            with self.session.guard:
                self.session.replay = self.session.demo_frame if source == "demo" else None
                self.session.last = None
            self.clear_plan()
            self.stage, self.message = "IDLE", "Input changed; start a fresh simulation."
        elif action == "virtual-z":
            delta = data.get("delta")
            if type(delta) is not int or delta not in (-10, 10) or not 0 <= self.virtual_z + delta <= 100:
                raise RobotError("Virtual Z moves in steps of 10, within 0–100.")
            self.virtual_z += delta
        elif action == "virtual-drill":
            on = data.get("on")
            if type(on) is not bool:
                raise RobotError("Select a drill state.")
            self.virtual_drill = on

    def pose_csv(self):
        with self.condition:
            if self.pose_log is None:
                raise RobotError("Pose logging is not configured.")
            return poses.export_csv(self.pose_log, self.simulated)

    def run(self, action):
        try:
            action()
        except MotionStopped as exc:
            with self.condition:
                if self.cancelled.is_set():
                    return
                self.stage, self.message = "IDLE", str(exc)
        except Exception as exc:
            # Unexpected errors must fail closed too; the MCU owns the lease.
            with self.condition:
                if self.cancelled.is_set():
                    return
                self.stage, self.message = "FAULT", str(exc)
            try:
                self.client.request("stop", {})
            except RobotError:
                pass

    def stop(self):
        self.cancelled.set()
        with self.condition:
            self.virtual_z, self.virtual_drill = 0, False
            self.stop_drill_keepalive()
            try:
                self.client.request("estop", {})
                self.stage, self.message = "ESTOP", "Network stop latched. Reset and re-home before resuming."
            except RobotError:
                self.stage, self.message = "FAULT", "Stop not confirmed. Use the physical emergency stop."
                raise RobotError(self.message)

    def reset(self):
        if self.client.status()["state"] != "IDLE":
            self.client.request("reset", {})
        self.clear_plan()
        self.virtual_z, self.virtual_drill = 0, False
        self.stop_drill_keepalive()
        self.transition("IDLE", "Reset complete. Next Start will re-home.")

    def move(self, status, targets, feed=2000):
        for axis, target in targets.items():
            if status["pos"][axis] != target:
                status = self.client.motion("move", {"axis": axis, "to": target, "feed": feed},
                                            self.cancelled, expected_boot=self.boot,
                                            expected_command=status["command_id"])
        return status

    def manual_move(self, status, targets):
        self.move(status, targets, MANUAL_FEED_US)
        self.transition("IDLE", "Manual move complete; start a fresh scan to plan again.")

    def prehome_nudge(self, status, axis, delta):
        self.client.motion("nudge", {"axis": axis, "delta": delta, "clearance_confirmed": True},
                           self.cancelled, expected_boot=self.boot,
                           expected_command=status["command_id"])
        self.transition("IDLE", "Pre-home nudge complete. Counts are not referenced to home.")

    def check_controller(self, status):
        if status["simulator"] != self.simulated:
            raise RobotError("Controller mode mismatch.")
        if not status["commissioned"]:
            raise RobotError("Controller is status-only: axis limits and home directions are not commissioned.")

    def home(self, observe=None):
        status = self.client.status()
        if observe is not None:
            observe(status, "before")
        self.check_controller(status)
        self.boot = status["boot"]
        return self.client.motion("home", {}, self.cancelled, expected_boot=self.boot,
                                  expected_command=status["command_id"], observe=observe)

    def traced_home(self):
        if self.trace_root is None:
            return self.home()
        trace = HomeTrace()
        frame = self.session.diagnostic_frame()
        outcome, error = "homed", None
        try:
            return self.home(trace.sample)
        except Exception as exc:
            outcome = "cancelled" if self.cancelled.is_set() else "failed"
            error = str(exc)
            if outcome != "cancelled":
                # A fault can latch between a status poll and its heartbeat.
                try:
                    trace.sample(self.client.status(), "after_error")
                except RobotError:
                    pass
            raise
        finally:
            try:
                bundle = trace.save(self.trace_root,
                                    {"before.jpg": frame, "after.jpg": self.session.diagnostic_frame()},
                                    outcome=outcome, error=error, simulated=self.simulated)
                with self.condition:
                    self.home_trace, self.home_trace_error = bundle, None
            except OSError as exc:
                with self.condition:
                    self.home_trace_error = f"Could not save the latest homing trace: {exc}"

    def home_only(self):
        self.traced_home()
        self.transition("IDLE", "Homing complete. Ready for setup.")

    def prepare(self):
        status = self.traced_home()
        self.transition("PARKING", "Moving to the camera-clear park pose.")
        self.move(status, self.park)
        self.transition("SCANNING")
        result = self.session.measure()
        check_cancelled(self.cancelled)
        if not result["passed"]:
            raise RobotError("Scan did not pass. Review the scan report; no target motion was issued.")
        self.before = result
        if self.session.last and self.session.last[1]["id"] == result["id"]:
            self.directory = self.session.last[0]
        if self.planner is None:
            self.transition("REVIEW", "Coordinates only. Robot calibration is not configured.")
            return
        if result["expected"] is None:
            self.transition("REVIEW", "Coordinates only. A nominal reference is required for motion.")
            return
        self.transition("PLANNING")
        plan = self.planner.plan(result["points"], self.budget)
        if self.directory:
            (self.directory / "plan.json").write_text(json.dumps({
                "schema_version": 1, "source_run": result["id"], "mode": "spindle_off",
                "simulated": self.simulated, "fixture_frame": "vision_bed",
                "park_steps": self.park, "placement_budget_mm": self.budget,
                "held_out_residual_mm": self.planner.residual_mm, "points": plan,
            }, indent=2, allow_nan=False) + "\n")
        self.plan = plan
        self.transition("REVIEW", "Approve the plan to run with the spindle off." if self.enable_dry_run
                        else "Plan is ready. Hardware dry run is not enabled.")

    def compare_scans(self, observed):
        check_cancelled(self.cancelled)
        old = {p["id"]: (float(p["x_mm"]), float(p["y_mm"])) for p in self.before["points"]}
        new = {p["id"]: (float(p["x_mm"]), float(p["y_mm"])) for p in observed["points"]}
        if not observed["passed"] or new.keys() != old.keys() or any(
                math.dist(old[key], new[key]) > self.budget for key in old):
            raise RobotError("The visible targets changed or registration failed.")

    def execute(self):
        status = self.client.status()
        if status.get("spindle"):
            self.client.request("drill", {"on": False, "boot": status["boot"]})
        self.compare_scans(self.session.measure())
        status = self.client.status()
        if (status["boot"] != self.boot or status["state"] != "IDLE" or not status["homed"]
                or any(status["pos"][axis] != self.park[axis] for axis in JOINTS)):
            raise RobotError("Controller is not homed at the approved park pose.")
        self.transition("EXECUTING")
        for point in self.plan:
            check_cancelled(self.cancelled)
            point["state"] = "moving"
            try:
                status = self.move(status, point["steps"])
                confirmed = {axis: status["pos"][axis] for axis in JOINTS}
                point["confirmed_steps"] = confirmed
                if self.simulated:
                    point["confirmed_xy_mm"] = self.planner.forward(
                        [confirmed[axis] for axis in JOINTS]).tolist()
                    point["command_id"] = status["command_id"]
                point["state"] = "visited"
            except RobotError:
                point["state"] = "failed"
                raise
        self.move(status, self.park)
        self.transition("VERIFYING")
        verification = self.session.measure()
        self.compare_scans(verification)
        if self.directory:
            (self.directory / "dry_run.json").write_text(json.dumps({
                "schema_version": 1, "simulated": self.simulated,
                "verification_run": verification["id"], "points": self.plan,
                "status": "visited_and_rescanned", "spindle_enabled": False,
            }, indent=2, allow_nan=False) + "\n")
        self.transition("DONE", "Dry run complete. Visible targets rechecked; no holes were drilled.")
