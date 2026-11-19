"""Protocol and integrated workflow tests without physical motion."""

import tempfile
import time
import unittest
from argparse import Namespace
from pathlib import Path
from threading import Event
from unittest.mock import patch

from robot.client import RobotClient, RobotError
from robot.planner import Planner
from robot.sim import Controller, server
from robot.workflow import Workflow
from vision.dashboard import Session


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
            with self.assertRaisesRegex(RobotError, "INVALID_TARGET"):
                client.motion("move", {"axis": "z", "to": -1, "feed": 2000}, Event())
            client.request("estop", {})
            self.assertEqual(client.status()["state"], "ESTOP")
            with self.assertRaises(RobotError):
                client.request("home", {})
            client.request("reset", {})
            self.assertFalse(client.status()["homed"])

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


if __name__ == "__main__":
    unittest.main()
