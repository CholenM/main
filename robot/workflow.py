"""Start → home/park → scan → approve → spindle-off motion → rescan."""

import json
import math
import threading

from .client import RobotError, check_cancelled


class Workflow:
    def __init__(self, client, session, planner=None, park=None, simulated=False,
                 enable_dry_run=False, budget=0.5):
        self.client, self.session, self.planner = client, session, planner
        self.park, self.simulated = park, simulated
        self.enable_dry_run, self.budget = enable_dry_run or simulated, budget
        self.condition = threading.RLock()
        self.cancelled = threading.Event()
        self.worker = None
        self.stage, self.message = "IDLE", ""
        self.status, self.link_error = None, None
        self.before, self.plan, self.boot = None, [], None
        self.directory = None

    def describe(self):
        with self.condition:
            return {"stage": self.stage, "message": self.message, "status": self.status,
                    "link_error": self.link_error, "simulated": self.simulated,
                    "park_configured": self.park is not None,
                    "review_id": self.before["id"] if self.before else None,
                    "plan": [dict(p) for p in self.plan],
                    "plan_url": (f"/run/{self.directory.name}/plan.json" if self.directory
                                 and self.plan else None),
                    "residual_mm": self.planner.residual_mm if self.planner else None,
                    "can_execute": self.stage == "REVIEW" and self.enable_dry_run and bool(self.plan)}

    def poll(self):
        try:
            status = self.client.status()
            self.status, self.link_error = status, None
        except RobotError as exc:
            self.status, self.link_error = None, str(exc)

    def transition(self, stage, message=""):
        with self.condition:
            check_cancelled(self.cancelled)
            self.stage, self.message = stage, message

    def launch(self, action, review_id=None):
        with self.condition:
            if self.worker and self.worker.is_alive():
                raise RobotError("A workflow action is still finishing.")
            if action == "start" and self.stage in ("IDLE", "DONE", "REVIEW"):
                if self.park is None:
                    raise RobotError("Configure and verify the camera-clear park pose first.")
                if not self.session.state()["ready"]:
                    raise RobotError("Camera is not ready.")
                self.before, self.plan, self.boot, self.directory = None, [], None, None
                self.cancelled.clear()
                self.stage, self.message = "HOMING", ""
                work = self.prepare
            elif (action == "execute" and self.stage == "REVIEW" and self.before
                  and review_id == self.before["id"] and self.plan and self.enable_dry_run):
                self.stage, self.message = "CHECKING", ""
                work = self.execute
            elif action == "reset" and (self.stage in ("FAULT", "ESTOP") or
                    self.stage in ("IDLE", "DONE", "REVIEW") and self.status and
                    self.status["state"] in ("FAULT", "ESTOP", "HOLD")):
                self.cancelled.clear()
                self.stage, self.message = "RESETTING", ""
                work = self.reset
            else:
                raise RobotError("Action unavailable in this state or review no longer current.")
            self.worker = threading.Thread(target=self.run, args=(work,), daemon=True)
            self.worker.start()

    def run(self, action):
        try:
            action()
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
            try:
                self.client.request("estop", {})
                self.stage, self.message = "ESTOP", "Network stop latched. Reset and re-home before resuming."
            except RobotError:
                self.stage, self.message = "FAULT", "Stop not confirmed. Use the physical emergency stop."
                raise RobotError(self.message)

    def reset(self):
        if self.client.status()["state"] != "IDLE":
            self.client.request("reset", {})
        self.before, self.plan, self.boot, self.directory = None, [], None, None
        self.transition("IDLE", "Reset complete. Next Start will re-home.")

    def move(self, status, targets):
        if status["pos"]["z"] != 0:
            raise RobotError("Horizontal motion requires Z at the verified safe datum.")
        for axis in ("j1", "j2"):
            target = targets[axis]
            if status["pos"][axis] != target:
                status = self.client.motion("move", {"axis": axis, "to": target, "feed": 2000},
                                            self.cancelled, expected_boot=self.boot)
        return status

    def prepare(self):
        status = self.client.status()
        if status["simulator"] != self.simulated:
            raise RobotError("Controller mode mismatch.")
        if not status["commissioned"]:
            raise RobotError("Controller is status-only: axis limits and home directions are not commissioned.")
        self.boot = status["boot"]
        status = self.client.motion("home", {}, self.cancelled, expected_boot=self.boot)
        self.transition("PARKING")
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
        self.compare_scans(self.session.measure())
        status = self.client.status()
        if (status["boot"] != self.boot or status["state"] != "IDLE" or not status["homed"]
                or status["pos"] != {"z": 0, **self.park}):
            raise RobotError("Controller is not homed at the approved park pose.")
        self.transition("EXECUTING")
        for point in self.plan:
            check_cancelled(self.cancelled)
            point["state"] = "moving"
            try:
                status = self.move(status, point["steps"])
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
