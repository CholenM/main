import unittest
from types import SimpleNamespace
from unittest.mock import patch

from mimicry.control import Control, MAX_SAMPLE_AGE, SETUP_WINDOW
from mimicry.tracking import classify
from robot import sim
from robot.client import RobotClient, RobotError


class MimicryTest(unittest.TestCase):
    def setUp(self):
        self.controller = sim.Controller(move_seconds=0, mimic_z_max=45)
        server = sim.server(controller=self.controller)
        address, token = server.__enter__()
        self.addCleanup(server.__exit__, None, None, None)
        self.client = RobotClient(address, token)
        self.now = 1.0
        self.control = Control(self.client, True, clock=lambda: self.now)

    def sample(self, point=None, hand="open", owner="first", start_frame=None):
        frame = start_frame or self.control.frame(virtual=True)[0]
        self.control.observe(owner, {"frame": frame, "hand": hand, "point": point})
        return frame

    def action(self, name):
        self.control.action("first", name)

    def arm(self):
        self.action("center")
        for _ in range(2):
            self.sample([.5, .5])
        self.action("arm")
        for _ in range(2):
            self.sample([.5, .5])
        self.assertTrue(self.control.armed)

    def send(self, command, **extra):
        status = self.client.status()
        return self.client.request(command, dict(extra, boot=status["boot"], command_id=status["command_id"]))

    def test_zero_is_manual_and_requires_bottom_contact(self):
        self.controller.pos["z"] = 5
        with self.assertRaisesRegex(RobotError, "Z_BOTTOM_SWITCH_NOT_ACTIVE"):
            self.action("zero")
        with self.assertRaisesRegex(RobotError, "Z_NOT_REFERENCED"):
            self.send("mimic-jog", axis="z", delta=1)
        self.controller.z_bottom = True
        self.action("zero")
        status = self.client.status()
        self.assertEqual(status["pos"]["z"], 0)
        self.assertEqual(status["bounds"]["z"], [0, 45])
        self.assertTrue(status["mimicry"]["z_referenced"])
        self.action("estop")
        self.action("reset")
        self.assertFalse(self.client.status()["mimicry"]["z_referenced"])
        with self.assertRaisesRegex(RobotError, "Z_NOT_REFERENCED"):
            self.send("move", axis="z", to=1, feed=2000)

    def test_gestures_clip_z_and_j1_never_changes_j2(self):
        self.action("zero")
        self.arm()
        for _ in range(3):
            self.sample([.55, .25])  # Diagonal, but Z has the greater displacement.
        self.assertEqual(self.client.status()["pos"], {"z": 45, "j1": 0, "j2": 0})
        self.sample([.5, .5])
        self.action("disarm")
        self.action("home")
        self.control.home_thread.join(2)
        self.arm()
        for _ in range(2):
            self.sample([.8, .4])
        self.assertEqual(self.client.status()["pos"], {"z": 45, "j1": 8, "j2": 0})
        for _ in range(2):
            self.sample([.5, .8])
        self.assertEqual(self.client.status()["pos"], {"z": 15, "j1": 8, "j2": 0})

    def test_stale_duplicate_center_fist_and_tracking_loss_do_not_send_motion(self):
        self.action("zero")
        self.arm()
        frame = self.sample([.5, .25])
        self.sample([.5, .25], start_frame=frame)
        self.assertEqual(self.client.status()["pos"]["z"], 0)
        self.sample([.5, .25])
        self.sample([.52, .47])  # Back inside the dead zone.
        self.assertEqual(self.client.status()["pos"]["z"], 30)
        self.sample(hand="fist")
        self.assertFalse(self.control.armed)
        self.arm()
        self.sample(hand="lost")
        self.assertFalse(self.control.armed)
        self.arm()
        stale_frame = self.control.frame(virtual=True)[0]
        self.now += MAX_SAMPLE_AGE + .01
        self.sample([.5, .25], start_frame=stale_frame)
        self.sample([.5, .25])
        self.assertFalse(self.control.armed)
        self.assertEqual(self.client.status()["pos"]["z"], 30)

    def test_external_commands_and_other_tabs_cannot_continue_armed_motion(self):
        self.action("zero")
        self.arm()
        with self.assertRaisesRegex(RobotError, "Another dashboard tab"):
            self.sample([.5, .25], owner="second")
        self.send("move", axis="z", to=1, feed=2000)
        self.sample([.5, .25])
        self.sample([.5, .25])
        self.assertFalse(self.control.armed)
        self.assertEqual(self.client.status()["pos"]["z"], 1)

    def test_firmware_contract_rejects_large_or_wrong_axis_jogs(self):
        self.action("zero")
        for axis, delta in (("j2", 1), ("z", 31), ("j1", -9), ("z", 0)):
            with self.subTest(axis=axis, delta=delta), self.assertRaisesRegex(RobotError, "INVALID_MIMIC_JOG"):
                self.send("mimic-jog", axis=axis, delta=delta)
        with self.assertRaisesRegex(RobotError, "NOT_HOMED_OR_SAFE_Z"):
            self.send("mimic-jog", axis="j1", delta=1)
        with self.assertRaisesRegex(RobotError, "MIMIC_TRAVEL_LIMIT"):
            self.send("mimic-jog", axis="z", delta=-1)

    def test_fixed_jog_lease_cannot_be_renewed(self):
        with patch("robot.sim.time.monotonic", side_effect=lambda: self.now):
            self.controller.move_seconds = 2
            self.action("zero")
            self.send("mimic-jog", axis="z", delta=30)
            self.now += .20
            status = self.client.status()
            self.client.request("heartbeat", {"boot": status["boot"], "command_id": status["command_id"]})
            self.now += .10
            status = self.client.status()
            self.assertEqual((status["state"], status["fault"]), ("FAULT", "HEARTBEAT"))
            self.assertFalse(status["mimicry"]["z_referenced"])

    def test_live_observation_cannot_be_overridden_by_browser(self):
        self.action("zero")
        self.arm()
        frame = self.control.frame((99, b"jpeg", self.now, {"hand": "fist"}))[0]
        self.sample([.5, .25], start_frame=frame)
        self.assertFalse(self.control.armed)
        self.assertIn("Fist detected", self.control.message)
        self.assertEqual(self.client.status()["pos"]["z"], 0)

    def test_setup_waits_for_fresh_steady_neutral_palm_after_click(self):
        self.action("zero")
        before_click = self.control.frame(virtual=True)[0]
        self.now += .05
        self.action("center")
        self.sample([.5, .5], start_frame=before_click)
        self.sample([.5, .5])
        self.assertIsNone(self.control.center)
        self.sample(hand="lost")
        self.sample([.5, .5])
        self.assertIsNone(self.control.center)
        self.sample([.5, .5])
        self.assertEqual(self.control.center, (.5, .5))
        self.action("arm")
        self.sample([.5, .2])
        self.sample([.5, .2])
        self.assertFalse(self.control.armed)
        self.sample([.5, .5])
        self.sample([.5, .5])
        self.assertTrue(self.control.armed)
        self.assertEqual(self.client.status()["pos"]["z"], 0)

    def test_setup_expires_and_disarm_cancels_it(self):
        self.action("zero")
        self.action("center")
        self.now += SETUP_WINDOW
        self.sample([.5, .5])
        self.sample([.5, .5])
        self.assertIsNone(self.control.center)
        self.arm()
        self.action("disarm")
        self.action("arm")
        self.action("disarm")
        self.sample([.5, .5])
        self.sample([.5, .5])
        self.assertFalse(self.control.armed)


class TrackingTest(unittest.TestCase):
    def test_only_one_confident_open_palm_provides_a_point(self):
        hand = [SimpleNamespace(x=i / 20, y=.25) for i in range(21)]
        category = SimpleNamespace(score=.9, category_name="Open_Palm")
        result = SimpleNamespace(hand_landmarks=[hand], gestures=[[category]])
        sample = classify(result)
        self.assertEqual(sample["hand"], "open")
        self.assertAlmostEqual(sample["point"][0], (0 + 5 + 9 + 13 + 17) / 100)
        self.assertEqual(sample["point"][1], .25)
        result.hand_landmarks.append(hand)
        self.assertEqual(classify(result), {"hand": "lost"})
        result.hand_landmarks.pop()
        category.score = .5
        self.assertEqual(classify(result), {"hand": "lost"})
        category.score = .9
        category.category_name = "Closed_Fist"
        self.assertEqual(classify(result), {"hand": "fist"})

    def test_open_hand_fallback_does_not_override_a_fist_or_accept_a_curled_finger(self):
        screen = [SimpleNamespace(x=.5, y=.5) for _ in range(21)]
        world = [SimpleNamespace(x=0., y=0., z=0.) for _ in range(21)]
        for start in (5, 9, 13, 17):
            for index in range(4):
                world[start+index] = SimpleNamespace(x=start / 1000, y=.04 + index * .02, z=0.)
        gesture = SimpleNamespace(category_name="None", score=.65)
        result = SimpleNamespace(hand_landmarks=[screen], hand_world_landmarks=[world],
                                 gestures=[[gesture]])
        self.assertEqual(classify(result)["hand"], "open")
        gesture.category_name = "Closed_Fist"
        self.assertEqual(classify(result), {"hand": "fist"})
        gesture.category_name = "None"
        world[8].y = .055
        self.assertEqual(classify(result), {"hand": "lost"})
