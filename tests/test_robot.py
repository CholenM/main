"""Protocol and integrated workflow tests without physical motion."""

import csv
import json
import tempfile
import time
import unittest
from argparse import Namespace
from functools import partial
from http.server import ThreadingHTTPServer
from pathlib import Path
from threading import Event, Thread
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

from robot.client import MotionStopped, RobotClient, RobotError
from robot.planner import Planner
from robot.sim import Controller, server
from robot.workflow import Workflow
from vision import fixture
from vision.dashboard import Dashboard, Session


class RobotTest(unittest.TestCase):
    def test_home_move_stop_reset(self):
        with server(controller=Controller(move_seconds=.02)) as (url, token):
            client = RobotClient(url, token)
            with self.assertRaisesRegex(RobotError, "NOT_HOMED"):
                client.motion("move", {"axis": "j1", "to": 100, "feed": 2000}, Event())
            home = client.motion("home", {}, Event())
            finish = client.motion("move", {"axis": "j1", "to": 100, "feed": 2000},
                                   Event(), expected_boot=home["boot"])
            self.assertEqual(finish["pos"]["j1"], 100)
            with self.assertRaisesRegex(RobotError, "SPINDLE_DISABLED"):
                client.request("spindle", {"on": True})
            status = client.status()
            client.request("drill", {"on": True, "boot": status["boot"]})
            self.assertTrue(client.status()["spindle"])
            client.request("heartbeat", {"boot": status["boot"],
                                         "command_id": client.status()["command_id"]})
            client.request("drill", {"on": False, "boot": status["boot"]})
            self.assertFalse(client.status()["spindle"])
            for to in (100, -100):
                finish = client.motion("move", {"axis": "z", "to": to, "feed": 2000}, Event())
                self.assertEqual(finish["pos"]["z"], to)
            client.request("estop", {})
            self.assertEqual(client.status()["state"], "ESTOP")
            with self.assertRaises(RobotError):
                client.request("home", {})
            client.request("reset", {})
            self.assertFalse(client.status()["homed"])

    def test_z_and_drill_work_without_homing(self):
        with server(controller=Controller(move_seconds=.02)) as (url, token):
            client = RobotClient(url, token)
            status = client.status()
            client.request("drill", {"on": True, "boot": status["boot"]})
            self.assertTrue(client.status()["spindle"])
            finish = client.motion("move", {"axis": "z", "to": 50, "feed": 2000}, Event())
            self.assertEqual(finish["pos"]["z"], 50)
            self.assertFalse(finish["homed"])
            self.assertFalse(finish["spindle"])
            with self.assertRaisesRegex(RobotError, "NOT_HOMED"):
                client.motion("move", {"axis": "j1", "to": 10, "feed": 2000}, Event())
            status = client.status()
            client.request("drill", {"on": True, "boot": status["boot"]})
            client.request("stop", {})
            self.assertFalse(client.status()["spindle"])

    def test_drill_lease_expires_without_heartbeat(self):
        with server(controller=Controller(move_seconds=.02, drill_seconds=.15)) as (url, token):
            client = RobotClient(url, token)
            status = client.status()
            client.request("drill", {"on": True, "boot": status["boot"]})
            self.assertTrue(client.status()["spindle"])
            time.sleep(.3)
            self.assertFalse(client.status()["spindle"])

    def test_z_bottom_switch_stops_without_losing_home(self):
        controller = Controller(move_seconds=.02)
        with server(controller=controller) as (url, token):
            client = RobotClient(url, token)
            client.motion("home", {}, Event())
            client.motion("move", {"axis": "z", "to": 50, "feed": 2000}, Event())
            controller.z_bottom = True
            with self.assertRaisesRegex(MotionStopped, "bottom switch"):
                client.motion("move", {"axis": "z", "to": 0, "feed": 2000}, Event())
            status = client.status()
            self.assertEqual((status["state"], status["homed"], status["fault"]),
                             ("IDLE", True, "Z_BOTTOM_LIMIT"))
            controller.z_bottom = False
            status = client.motion("move", {"axis": "z", "to": 60, "feed": 2000}, Event())
            self.assertEqual((status["pos"]["z"], status["fault"]), (60, None))

    def test_status_cannot_keep_motion_lease_alive(self):
        with server(controller=Controller(move_seconds=1, lease_seconds=.06)) as (url, token):
            client = RobotClient(url, token)
            before = client.status()
            command = client.request("home", {key: before[key] for key in ("boot", "command_id")})
            for _ in range(6):
                time.sleep(.02)
                status = client.status()
            self.assertEqual((status["state"], status["fault"]), ("FAULT", "HEARTBEAT"))
            with self.assertRaisesRegex(RobotError, "COMMAND_CHANGED"):
                client.request("heartbeat", command)

    def test_bad_token_and_different_boot_fail_closed(self):
        with server() as (url, token):
            with self.assertRaisesRegex(RobotError, "UNAUTHORIZED"):
                RobotClient(url, "bad").status()
            # A stale boot ID is rejected by the controller before it acts.
            with self.assertRaisesRegex(RobotError, "COMMAND_CHANGED"):
                RobotClient(url, token).request("heartbeat", {"boot": "old", "command_id": 0})
            with self.assertRaisesRegex(RobotError, "COMMAND_CHANGED"):
                RobotClient(url, token).request("home", {"boot": "old", "command_id": 0})
            self.assertEqual(RobotClient(url, token).status()["state"], "IDLE")

    def test_planner_uses_fixture_mm_and_no_extrapolation(self):
        planner = Planner.load(None, True)
        target = planner.plan([{"id": "h1", "x_mm": "25.04", "y_mm": "34.96"}], .5)[0]
        self.assertEqual(target["steps"], {"j1": 250, "j2": 350})
        self.assertAlmostEqual(target["achievable_mm"][0], 25)
        self.assertLess(target["rounding_mm"], .06)
        with self.assertRaisesRegex(RobotError, "outside"):
            planner.plan([{"id": "h1", "x_mm": "300", "y_mm": "0"}], .5)

    def test_workflow_review_and_verify(self):
        with server(controller=Controller(move_seconds=.02)) as (url, token):
            session = Session(Namespace(verify=False), object())
            client = RobotClient(url, token)
            workflow = Workflow(client, session, Planner.load(None, True),
                                {"j1": 0, "j2": 0}, simulated=True)
            session.workflow = workflow
            result = {"id": "scan1", "passed": True, "expected": 1,
                      "points": [{"id": "H1", "x_mm": "42", "y_mm": "21"}]}
            with patch.object(session, "measure", return_value=result) as measurement:
                workflow.launch("start")
                workflow.worker.join(5)
                self.assertEqual(workflow.describe()["stage"], "REVIEW", workflow.describe())
                self.assertEqual(client.status()["pos"], {"z": 0, "j1": 0, "j2": 0})
                with self.assertRaisesRegex(RobotError, "review no longer current"):
                    workflow.launch("execute", "another-scan")
                workflow.launch("execute", "scan1")
                workflow.worker.join(5)
                state = workflow.describe()
                self.assertEqual(state["stage"], "DONE", state)
                self.assertEqual(state["plan"][0]["state"], "visited")
                self.assertEqual(measurement.call_count, 3)
                self.assertEqual(client.status()["pos"], {"z": 0, "j1": 0, "j2": 0})

    def test_stop_during_home_requires_reset(self):
        with server(controller=Controller(move_seconds=1)) as (url, token):
            session = Session(Namespace(verify=False), object())
            client = RobotClient(url, token)
            workflow = Workflow(client, session, park={"j1": 0, "j2": 0}, simulated=True)
            workflow.launch("start")
            for _ in range(50):
                if client.status()["state"] == "HOMING":
                    break
                time.sleep(.01)
            else:
                self.fail("simulator did not begin homing")
            workflow.stop()
            workflow.worker.join(5)
            self.assertEqual(workflow.describe()["stage"], "ESTOP")
            self.assertEqual(client.status()["state"], "ESTOP")
            self.assertFalse(client.status()["homed"])

    def test_manual_setup_guards_and_mode_b(self):
        with server(controller=Controller(move_seconds=.02)) as (url, token):
            session = Session(Namespace(verify=False), object())
            client = RobotClient(url, token)
            workflow = Workflow(client, session, simulated=True)

            def jog(axis, delta, status=None):
                workflow.launch("jog", data=dict(status or client.status(), axis=axis, delta=delta))

            with self.assertRaisesRegex(RobotError, "Home J1/J2"):
                jog("j1", 10)
            workflow.launch("home")
            workflow.worker.join(5)
            initial = client.status()
            for axis, delta in (("z", 10), ("j1", 1000), ("j2", True)):
                with self.assertRaisesRegex(RobotError, "allowed jog"):
                    jog(axis, delta)
            jog("j1", 25)
            workflow.worker.join(5)
            self.assertEqual(client.status()["pos"]["j1"], 25)
            with self.assertRaisesRegex(RobotError, "stale"):
                jog("j1", 10, initial)
            workflow.launch("save-park", data=client.status())
            jog("j2", -10)
            workflow.worker.join(5)
            workflow.launch("return-park", data=client.status())
            workflow.worker.join(5)
            self.assertEqual(client.status()["pos"], {"z": 0, "j1": 25, "j2": 0})
            client.motion("move", {"axis": "j1", "to": 10000, "feed": 2000}, Event())
            with self.assertRaisesRegex(RobotError, "outside"):
                jog("j1", 10)
            self.assertEqual(client.status()["pos"]["j1"], 10000)

            result = {"id": "scan1", "passed": True, "expected": 1, "points": []}
            with patch.object(session, "measure", return_value=result):
                workflow.launch("start")
                workflow.worker.join(5)
            self.assertEqual(workflow.stage, "REVIEW")
            self.assertFalse(workflow.describe()["can_execute"])
            self.assertIn("Coordinates only", workflow.message)
            jog("j2", 10)
            workflow.worker.join(5)
            self.assertIsNone(workflow.describe()["review_id"])

    def test_record_uses_live_steps_and_exports_csv(self):
        with tempfile.TemporaryDirectory() as temp, \
                server(controller=Controller(move_seconds=.02)) as (url, token):
            session = Session(Namespace(verify=False), None)
            client = RobotClient(url, token)
            path = Path(temp) / "poses.jsonl"
            workflow = Workflow(client, session, simulated=True, pose_log=path)
            workflow.launch("home")
            workflow.worker.join(5)
            status = client.status()
            with self.assertRaisesRegex(RobotError, "both measured"):
                workflow.launch("record-pose", data=dict(status, role="fit", x_mm=5))
            self.assertFalse(path.exists())
            workflow.launch("record-pose", data=dict(status, label="=Sample", role="check",
                                                     x_mm=0, y_mm=15, j1_steps=999))
            recorded = json.loads(path.read_text())
            exported = next(csv.DictReader(workflow.pose_csv().splitlines()))
            self.assertEqual((recorded["boot"], recorded["j1_steps"], recorded["y_mm"]),
                             (status["boot"], 0, 15))
            self.assertEqual((exported["label"], exported["role"]), ("'=Sample", "check"))

    def test_manual_jog_refused_while_workflow_active(self):
        with server(controller=Controller(move_seconds=.5)) as (url, token):
            session = Session(Namespace(verify=False), None)
            client = RobotClient(url, token)
            workflow = Workflow(client, session, simulated=True)
            workflow.launch("home")
            with self.assertRaisesRegex(RobotError, "still finishing"):
                workflow.launch("jog", data={"axis": "j1", "delta": 10})
            workflow.stop()
            workflow.worker.join(5)

    def test_manual_only_jogs_without_camera_or_park(self):
        with server(controller=Controller(move_seconds=.02)) as (url, token):
            session = Session(Namespace(verify=False), None)
            client = RobotClient(url, token)
            workflow = Workflow(client, session, simulated=True, manual_only=True)
            self.assertFalse(session.state()["ready"])
            self.assertFalse(workflow.describe()["park_configured"])
            for action in ("start", "execute"):
                with self.assertRaisesRegex(RobotError, "manual-only"):
                    workflow.launch(action)
            with self.assertRaisesRegex(RobotError, "Home J1/J2"):
                workflow.launch("jog", data={"axis": "j2", "delta": 10})
            workflow.launch("home")
            workflow.worker.join(5)
            workflow.launch("jog", data=dict(client.status(), axis="j2", delta=10))
            workflow.worker.join(5)
            self.assertEqual(client.status()["pos"]["j2"], 10)
            self.assertFalse(workflow.describe()["can_execute"])

    def test_manual_z_and_drill_work_before_homing(self):
        with server(controller=Controller(move_seconds=.02)) as (url, token):
            session = Session(Namespace(verify=False), None)
            client = RobotClient(url, token)
            workflow = Workflow(client, session, simulated=True, manual_only=True)
            workflow.launch("z-jog", data=dict(client.status(), axis="z", delta=10))
            workflow.worker.join(5)
            self.assertEqual(client.status()["pos"]["z"], 10)
            self.assertFalse(client.status()["homed"])
            workflow.launch("drill", data=dict(client.status(), on=True))
            self.assertTrue(client.status()["spindle"])
            workflow.launch("drill", data=dict(client.status(), on=False))
            self.assertFalse(client.status()["spindle"])
            workflow.drill_keepalive.join(5)

    def test_manual_z_bottom_stop_keeps_workflow_idle(self):
        controller = Controller(move_seconds=.02)
        with server(controller=controller) as (url, token):
            session = Session(Namespace(verify=False), None)
            client = RobotClient(url, token)
            workflow = Workflow(client, session, simulated=True, manual_only=True)
            client.motion("home", {}, Event())
            client.motion("move", {"axis": "z", "to": 50, "feed": 2000}, Event())
            controller.z_bottom = True
            workflow.launch("z-jog", data=dict(client.status(), axis="z", delta=-10))
            workflow.worker.join(5)
            self.assertEqual(workflow.stage, "IDLE")
            self.assertIn("bottom", workflow.message)
            self.assertTrue(client.status()["homed"])
            self.assertEqual(client.status()["pos"]["z"], 50)

    def test_prehome_nudges_are_limited_and_do_not_establish_home(self):
        with server(controller=Controller(move_seconds=.02)) as (url, token):
            session = Session(Namespace(verify=False), None)
            client = RobotClient(url, token)
            workflow = Workflow(client, session, simulated=True, manual_only=True)

            def nudge(axis, delta, confirmed=True):
                workflow.launch("nudge", data=dict(client.status(), axis=axis, delta=delta,
                                                   clearance_confirmed=confirmed))
                workflow.worker.join(5)

            with self.assertRaisesRegex(RobotError, "Confirm physical clearance"):
                nudge("j2", 10, False)
            for axis, delta in (("z", 10), ("j2", 25)):
                with self.assertRaisesRegex(RobotError, "allowed jog"):
                    nudge(axis, delta)
            nudge("j2", -10)
            self.assertEqual(client.status()["pos"]["j2"], -10)
            self.assertFalse(client.status()["homed"])
            client.request("estop", {})
            client.request("reset", {})
            self.assertEqual(client.status()["prehome_nudge"]["remaining"], 20)
            nudge("j1", 10)
            nudge("j1", -10)
            with self.assertRaisesRegex(RobotError, "budget"):
                nudge("j2", 10)
            with self.assertRaisesRegex(RobotError, "NUDGE_BUDGET_EXHAUSTED"):
                client.motion("nudge", {"axis": "j1", "delta": 10, "clearance_confirmed": True}, Event())

    def test_dashboard_manual_actions_and_pose_download(self):
        with tempfile.TemporaryDirectory() as temp, \
                server(controller=Controller(move_seconds=.02)) as (url, token):
            session = Session(Namespace(verify=False), None)
            session.workflow = Workflow(RobotClient(url, token), session, simulated=True,
                                        pose_log=Path(temp) / "poses.jsonl")
            with ThreadingHTTPServer(("127.0.0.1", 0), partial(Dashboard, session=session)) as app:
                thread = Thread(target=app.serve_forever, daemon=True)
                thread.start()
                base = f"http://127.0.0.1:{app.server_port}/api/"

                def post(action, data=None):
                    body = json.dumps(data).encode() if data is not None else None
                    request = Request(base + "robot/" + action, body,
                                      {"Content-Type": "application/json"}, method="POST")
                    with urlopen(request, timeout=3) as response:
                        return json.load(response)

                def status():
                    with urlopen(base + "state", timeout=3) as response:
                        return json.load(response)["workflow"]["status"]

                try:
                    with urlopen(base.removesuffix("api/") + "manual", timeout=3) as response:
                        page = response.read()
                        self.assertIn(b'id="manual-panel"', page)
                        self.assertIn(b'id="camera-panel"', page)
                    with self.assertRaises(HTTPError) as caught:
                        post("jog", [])
                    self.assertEqual(caught.exception.code, 400)
                    with patch.object(session, "diagnostic_frame",
                                      return_value=np.zeros((24, 32, 3), dtype=np.uint8)):
                        post("home")
                        session.workflow.worker.join(5)
                    trace = session.workflow.describe()["home_trace"]
                    self.assertEqual(trace["outcome"], "homed")
                    trace_url = base.removesuffix("api/") + f"homing/{trace['id']}/"
                    with urlopen(trace_url + "trace.csv", timeout=3) as response:
                        samples = list(csv.DictReader(response.read().decode().splitlines()))
                    self.assertEqual(samples[0]["sample"], "before")
                    self.assertEqual(samples[-1]["homed"], "True")
                    with urlopen(trace_url + "after.jpg", timeout=3) as response:
                        self.assertEqual(response.headers.get_content_type(), "image/jpeg")
                    post("jog", dict(status(), axis="j2", delta=10))
                    session.workflow.worker.join(5)
                    post("record-pose", dict(status(), label="Sample"))
                    with urlopen(base + "robot/poses.csv", timeout=3) as response:
                        row = next(csv.DictReader(response.read().decode().splitlines()))
                    self.assertEqual((row["label"], row["j2_steps"]), ("Sample", "10"))
                    post("z-jog", dict(status(), axis="z", delta=10))
                    session.workflow.worker.join(5)
                    self.assertEqual(status()["pos"]["z"], 10)
                    post("drill", dict(status(), on=True))
                    self.assertTrue(status()["spindle"])
                    post("drill", dict(status(), on=False))
                    self.assertFalse(status()["spindle"])
                    session.workflow.drill_keepalive.join(5)
                finally:
                    app.shutdown()
                    thread.join()

    def test_homing_fault_trace_survives_reset(self):
        with tempfile.TemporaryDirectory() as temp, \
                server(controller=Controller(move_seconds=1, lease_seconds=0)) as (url, token):
            session = Session(Namespace(verify=False), None)
            path = Path(temp) / "poses.jsonl"
            workflow = Workflow(RobotClient(url, token), session, simulated=True, pose_log=path)
            workflow.launch("home")
            workflow.worker.join(5)
            self.assertEqual(workflow.stage, "FAULT")
            summary = workflow.describe()["home_trace"]
            self.assertEqual(summary["outcome"], "failed")
            self.assertIn("HEARTBEAT", summary["error"])
            with (path.parent / "homing" / summary["id"] / "trace.csv").open() as file:
                self.assertEqual(list(csv.DictReader(file))[-1]["fault"], "HEARTBEAT")
            self.assertNotIn("after.jpg", summary["files"])
            workflow.launch("reset")
            workflow.worker.join(5)
            self.assertEqual(workflow.describe()["home_trace"], summary)

    def test_console_simulation_scans_image_and_reports_confirmed_visits(self):
        with tempfile.TemporaryDirectory() as temp, \
                server(controller=Controller(move_seconds=.02)) as (url, token):
            args = Namespace(verify=False, replay=None, runs_dir=Path(temp), preset="paper",
                             reference=None, polarity="dark", min_diameter=2.2, max_diameter=4)
            live = Session(args, None)
            simulated = Session(args, fixture.demo_frame(), demo=True)
            simulated.workflow = Workflow(RobotClient(url, token), simulated, Planner.load(None, True),
                                           park={"j1": 0, "j2": 0}, simulated=True)
            handler = partial(Dashboard, session=live, simulation=simulated)
            with ThreadingHTTPServer(("127.0.0.1", 0), handler) as app:
                thread = Thread(target=app.serve_forever, daemon=True)
                thread.start()
                base = f"http://127.0.0.1:{app.server_port}"

                def load(path):
                    with urlopen(base + path, timeout=5) as response:
                        return json.load(response)

                def act(action, data=None):
                    body = json.dumps(data).encode() if data is not None else None
                    request = Request(base + "/sim/api/robot/" + action, body,
                                      {"Content-Type": "application/json"}, method="POST")
                    with urlopen(request, timeout=5) as response:
                        return json.load(response)

                try:
                    self.assertTrue(load("/api/state")["has_simulation"])
                    self.assertIsNone(load("/api/state")["workflow"])
                    self.assertEqual(load("/sim/api/state")["demo_source"], "demo")
                    self.assertEqual(act("virtual-z", {"delta": 10})["virtual_tools"]["z_steps"], 10)
                    self.assertTrue(act("virtual-drill", {"on": True})["virtual_tools"]["drill"])
                    act("start")
                    simulated.workflow.worker.join(10)
                    review = load("/sim/api/state")["workflow"]
                    self.assertEqual(review["stage"], "REVIEW", review)
                    self.assertFalse(review["virtual_tools"]["drill"])
                    act("execute", {"review_id": review["review_id"]})
                    simulated.workflow.worker.join(10)
                    complete = load("/sim/api/state")["workflow"]
                    self.assertEqual(complete["stage"], "DONE", complete)
                    proof = load("/sim" + complete["dry_run_url"])
                    self.assertTrue(proof["simulated"])
                    self.assertEqual(len(proof["points"]), 6)
                    for point in proof["points"]:
                        self.assertEqual(point["state"], "visited")
                        self.assertEqual(point["steps"], point["confirmed_steps"])
                        self.assertEqual(point["achievable_mm"], point["confirmed_xy_mm"])
                        self.assertGreater(point["command_id"], 0)
                    self.assertIsNone(load("/api/state")["workflow"])
                finally:
                    app.shutdown()
                    thread.join()


if __name__ == "__main__":
    unittest.main()
