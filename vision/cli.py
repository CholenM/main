"""Command-line interface, vision dashboard, and optional SCARA workflow."""

import argparse
import fcntl
import json
import math
import os
import signal
import sys
from contextlib import contextmanager, nullcontext
from functools import partial
from http.server import ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from . import fixture as f
from . import scan
from .dashboard import Dashboard, LiveCamera, Session
from robot import sim
from robot.client import RobotClient, RobotError
from robot.planner import Planner
from robot.workflow import Workflow

RUNS = f.ROOT / "runs"
PIDFILE = RUNS / ".viewer.pid"


def proc_token(pid):
    # Field 22 of /proc/.../stat identifies a process beyond its reusable PID.
    return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]


def stop():
    try:
        data = json.loads(PIDFILE.read_text())
        pid = data["pid"]
        if not isinstance(pid, int) or pid < 2 or proc_token(pid) != data["start"]:
            raise ProcessLookupError
        os.kill(pid, signal.SIGTERM)
    except (FileNotFoundError, ProcessLookupError, KeyError, ValueError, IndexError):
        print("No vision viewer is running.")
    else:
        print("Vision viewer stop requested.")
    return 0


@contextmanager
def exclusive(name, message):
    RUNS.mkdir(exist_ok=True)
    with (RUNS / name).open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise scan.ScanError(message) from exc
        yield


def camera_lock():
    return exclusive(".camera.lock", "A vision process owns the camera. Stop the live dashboard first.")


def serve(args, replay_frame):
    with exclusive(".viewer.lock", "A viewer is already running. Use ./stop.sh first."), \
            (sim.server() if args.simulate_robot else nullcontext(None)) as sim_address:
        session = Session(args, replay_frame)
        if sim_address or args.robot_url:
            address, token = sim_address or (args.robot_url, os.environ.get("SCARA_ROBOT_TOKEN", ""))
            if not token:
                raise RobotError("Set SCARA_ROBOT_TOKEN before connecting to a physical controller.")
            planner = Planner.load(args.calibration, args.simulate_robot)
            park = args.park_steps if args.park_steps is not None else (0, 0) if sim_address else None
            session.workflow = Workflow(RobotClient(address, token), session, planner,
                                        dict(zip(("j1", "j2"), park)) if park is not None else None,
                                        args.simulate_robot, args.enable_hardware_dry_run,
                                        args.placement_budget)
        handler = partial(Dashboard, session=session)
        with ThreadingHTTPServer((args.bind, args.port), handler) as server:
            if replay_frame is not None and session.workflow is None:
                session.measure()

            def terminate(_signal, _frame):
                raise KeyboardInterrupt

            signal.signal(signal.SIGTERM, terminate)
            try:
                with (camera_lock() if replay_frame is None else nullcontext()):
                    with (LiveCamera(args.camera) if replay_frame is None else nullcontext()) as camera:
                        session.camera = camera
                        pid = os.getpid()
                        pending = PIDFILE.with_suffix(".tmp")
                        pending.write_text(json.dumps({"pid": pid, "start": proc_token(pid)}))
                        pending.replace(PIDFILE)
                        host = "127.0.0.1" if args.bind == "0.0.0.0" else args.bind
                        print(f"Dashboard: http://{host}:{server.server_port}/ "
                              "(Ctrl-C or ./stop.sh)", flush=True)
                        try:
                            server.serve_forever()
                        finally:
                            workflow = session.workflow
                            if workflow and workflow.worker and workflow.worker.is_alive():
                                try:
                                    workflow.stop()
                                except RobotError as exc:
                                    print(f"Vision: {exc}", file=sys.stderr)
            except KeyboardInterrupt:
                pass
            finally:
                PIDFILE.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description="Measure targets in a 235 × 127 mm printed ArUco fixture.")
    parser.add_argument("--camera", default=os.environ.get("SCARA_CAMERA", "/dev/video0"))
    parser.add_argument("--preset", choices=("paper", "pcb"), default="paper")
    parser.add_argument("--polarity", choices=("dark", "bright"), default="dark")
    parser.add_argument("--min-diameter", type=float, default=2.2, help="candidate lower limit, mm")
    parser.add_argument("--max-diameter", type=float, default=4.0, help="candidate upper limit, mm")
    parser.add_argument("--verify", action="store_true", help="compare to nominal; never writes a job")
    parser.add_argument("--reference", type=Path, help="reference CSV in the fixture frame")
    parser.add_argument("--replay", type=Path, help="image file or previous run directory")
    parser.add_argument("--runs-dir", type=Path, default=RUNS)
    parser.add_argument("--serve", action="store_true", help="start the live dashboard (or offline replay viewer)")
    parser.add_argument("--bind", default="127.0.0.1", help="viewer interface; defaults to local only")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--self-test", action="store_true", help="capture one frame and check camera availability")
    robot_group = parser.add_mutually_exclusive_group()
    robot_group.add_argument("--simulate-robot", action="store_true", help="run workflow against a local fake controller")
    robot_group.add_argument("--robot-url", help="physical controller URL (requires SCARA_ROBOT_TOKEN)")
    parser.add_argument("--park-steps", type=int, nargs=2, metavar=("J1", "J2"),
                        help="verified camera-clear pose at Z=0; (0,0) explicitly confirms home is park")
    parser.add_argument("--calibration", type=Path, help="measured calibration JSON for motion planning")
    parser.add_argument("--enable-hardware-dry-run", action="store_true",
                        help="permit raised-tool moves after calibration and operator approval")
    parser.add_argument("--placement-budget", type=float, default=.5, metavar="MM")
    parser.add_argument("--stop-viewer", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.stop_viewer:
        return stop()
    if not (0 < args.min_diameter <= args.max_diameter <= f.HEIGHT):
        parser.error(f"expected 0 < min-diameter <= max-diameter <= {f.HEIGHT} mm")
    if args.verify and args.preset == "pcb" and args.reference is None:
        parser.error("PCB verification requires --reference CSV")
    if not math.isfinite(args.placement_budget) or args.placement_budget <= 0:
        parser.error("--placement-budget must be positive and finite")
    if (args.simulate_robot or args.robot_url) and (not args.serve or args.verify or args.self_test):
        parser.error("Robot workflows require --serve in target detection mode")
    if args.robot_url and (args.replay or args.bind not in ("127.0.0.1", "localhost")):
        parser.error("Physical controller mode requires live captures and a loopback dashboard. Use an SSH tunnel.")
    if args.enable_hardware_dry_run and not (args.robot_url and args.calibration and args.park_steps is not None):
        parser.error("Hardware dry run requires --robot-url, --calibration and --park-steps")
    try:
        frame = None
        if args.replay:
            path = args.replay / "raw.png" if args.replay.is_dir() else args.replay
            frame = cv2.imread(str(path))
            if frame is None:
                raise scan.ScanError(f"Cannot read replay image: {path}")
        if args.serve and not args.self_test:
            serve(args, frame)
            return 0
        if frame is None:
            with camera_lock():
                frame = scan.capture(args.camera)
        if args.self_test:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            print(f"Camera input: {frame.shape[1]}×{frame.shape[0]}, mean brightness "
                  f"{np.mean(gray):.1f}/255; sharpness proxy {cv2.Laplacian(gray, cv2.CV_64F).var():.1f}")
            try:
                _, data = scan.register(frame)
                print(f'Fixture tags {data["seen_ids"]}; board resolution '
                      f'{data["min_native_px_per_mm"]:.2f} native px/mm.')
            except scan.ScanError as exc:
                print(f"Camera OK. Fixture not yet validated: {exc}")
            return 0
        _, success = scan.run(args, frame)
        return 0 if success else 2
    except (scan.ScanError, RobotError, OSError, ValueError, cv2.error) as error:
        print(f"Vision: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
