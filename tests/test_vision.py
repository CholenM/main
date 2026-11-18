"""Test the print PDFs through the replay CLI, without a camera or printer."""

import csv
import json
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request
from argparse import Namespace
from functools import partial
from http.server import ThreadingHTTPServer
from pathlib import Path
from threading import Thread
from unittest.mock import patch

import cv2
import numpy as np

from vision import fixture
from vision.dashboard import Dashboard, LiveCamera, Session


@unittest.skipUnless(shutil.which("pdftoppm"), "PDF replay tests need poppler-utils")
class VisionReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        output = tempfile.TemporaryDirectory()
        cls.addClassCleanup(output.cleanup)
        cls.path = Path(output.name)
        fixture.generate(cls.path)
        cls.runs = cls.path / "runs"
        cls.full = cls.render("aruco_bed_v1")

    @classmethod
    def render(cls, name):
        output = cls.path / name
        subprocess.run(["pdftoppm", "-r", "152.4", "-png", "-singlefile",
                        str(output.with_suffix(".pdf")), str(output)], check=True)
        return cv2.imread(str(output.with_suffix(".png")))

    def replay(self, frame, *args):
        image = self.path / "input.png"
        self.assertTrue(cv2.imwrite(str(image), frame))
        result = subprocess.run([sys.executable, "-m", "vision.cli", "--replay", str(image),
                                 "--runs-dir", str(self.runs), *args],
                                capture_output=True, text=True, check=False)
        directory = max(self.runs.iterdir())
        return result, directory

    @staticmethod
    def camera_view(frame):
        height, width = frame.shape[:2]
        h = cv2.getPerspectiveTransform(
            np.float32(((0, 0), (width-1, 0), (width-1, height-1), (0, height-1))),
            np.float32(((345, 140), (1540, 107), (1680, 995), (214, 930))))
        return cv2.warpPerspective(frame, h, (1920, 1080), borderValue=(145, 145, 145))

    def test_nominal_pdf_replay(self):
        # PDF page-box rounding can add one output pixel in either direction.
        self.assertLessEqual(max(abs(a-b) for a, b in zip(self.full.shape[:2], (1260, 1782))), 1)
        result, directory = self.replay(self.camera_view(self.full))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        job = json.loads((directory / "job.json").read_text())
        self.assertEqual(job["frame"], "vision_bed")
        self.assertFalse(job["executable"])
        self.assertEqual({p["id"] for p in job["points"]}, {p[0] for p in fixture.POINTS})
        with (directory / "errors.csv").open() as source:
            self.assertLess(max(float(row["radial_mm"]) for row in csv.DictReader(source)), .2)
        self.assertTrue((directory / "overlay.png").is_file())

    def test_shifted_missing_target(self):
        shifted = self.render("aruco_bed_inspection")
        result, directory = self.replay(self.camera_view(shifted), "--verify")
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        with (directory / "errors.csv").open() as source:
            rows = {r["id"]: r["status"] for r in csv.DictReader(source)}
        self.assertEqual(rows, {"H1": "OFFSET", "H2": "PASS", "H3": "MISSING",
                                "H4": "PASS", "H5": "OFFSET", "H6": "MISSING"})
        self.assertFalse((directory / "job.json").exists())

    def test_no_tags_no_job(self):
        result, directory = self.replay(np.full((480, 640, 3), 255, dtype=np.uint8))
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("Required corner tags", (directory / "report.md").read_text())
        self.assertFalse((directory / "job.json").exists())

    def test_missing_corner_tag_no_job(self):
        frame = self.full.copy()
        frame[875:1180, 30:300] = 255
        result, directory = self.replay(self.camera_view(frame))
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("Required corner tags", (directory / "report.md").read_text())
        self.assertFalse((directory / "job.json").exists())

    def test_dashboard_stream_and_full_resolution_scan(self):
        frame = self.camera_view(self.full)

        class Camera:
            def read(self):
                time.sleep(1/30)
                return True, frame

            def release(self):
                pass

        args = Namespace(replay=None, runs_dir=self.runs, verify=False,
                         preset="paper", reference=None, polarity="dark",
                         min_diameter=2.2, max_diameter=4.0)
        session = Session(args, None)
        with patch("vision.dashboard.scan.open_camera", return_value=Camera()), \
                LiveCamera("/dev/fake") as camera, \
                ThreadingHTTPServer(("127.0.0.1", 0), partial(Dashboard, session=session)) as server:
            session.camera = camera
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                url = f"http://127.0.0.1:{server.server_port}/"
                with urllib.request.urlopen(url + "stream.mjpg", timeout=6) as stream:
                    for _ in range(2):
                        self.assertEqual(stream.readline(), b"--frame\r\n")
                        self.assertEqual(stream.readline(), b"Content-Type: image/jpeg\r\n")
                        length = int(stream.readline().split(b": ")[-1])
                        self.assertEqual(stream.readline(), b"\r\n")
                        preview = cv2.imdecode(np.frombuffer(stream.read(length), np.uint8), 1)
                        self.assertEqual(preview.shape[1], 640)
                        self.assertEqual(stream.read(2), b"\r\n")
                request = urllib.request.Request(url + "api/scan", method="POST")
                with urllib.request.urlopen(request, timeout=6) as response:
                    result = json.load(response)
                self.assertEqual((result["status"], result["detected"]), ("PASS", 6))
                saved = cv2.imread(str(self.runs / result["id"] / "raw.png"))
                self.assertEqual(saved.shape, frame.shape)
            finally:
                server.shutdown()
                thread.join()


if __name__ == "__main__":
    unittest.main()
