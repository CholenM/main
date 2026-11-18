"""Command-line interface and local, read-only presentation viewer."""

import argparse
import fcntl
import json
import os
import signal
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from . import fixture as f
from . import scan

RUNS = f.ROOT / "runs"
PIDFILE = RUNS / ".viewer.pid"
FILES = ("raw.png", "rectified.png", "overlay.png", "report.md", "detections.csv",
         "errors.csv", "ground_truth.csv", "registration.json", "job.json")


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


class Viewer(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            run = Path(self.directory)
            links = " · ".join(f'<a href="{n}">{n}</a>' for n in FILES if (run / n).is_file())
            picture = ('<img src="overlay.png" alt="Measured points and millimetre coordinates">'
                       if (run / "overlay.png").is_file() else
                       '<img src="raw.png" alt="Input snapshot">')
            html = (f'<!doctype html><meta charset="utf-8"><title>SCARA Vision</title>'
                    f'<style>body{{font:17px system-ui;max-width:1100px;margin:2rem auto;'
                    f'padding:0 1rem}}img,iframe{{width:100%;border:0}}'
                    f'iframe{{height:450px}}</style><h1>SCARA Vision</h1>'
                    f'<p>Read-only review · No robot motion</p><p>{links}</p>{picture}'
                    f'<iframe src="report.md" title="Scan report"></iframe>').encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif self.path[1:] in FILES:
            super().do_GET()
        else:
            self.send_error(404)

    def do_HEAD(self):
        self.send_error(405)

    def guess_type(self, path):
        return "text/plain; charset=utf-8" if path.endswith(".md") else super().guess_type(path)


def serve(directory, bind, port):
    RUNS.mkdir(exist_ok=True)
    with (RUNS / ".viewer.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise scan.ScanError("A viewer is already running. Use ./stop.sh first.") from exc
        with ThreadingHTTPServer((bind, port), partial(Viewer, directory=str(directory))) as server:
            pid = os.getpid()

            def terminate(_signal, _frame):
                raise KeyboardInterrupt

            signal.signal(signal.SIGTERM, terminate)
            pending = PIDFILE.with_suffix(".tmp")
            pending.write_text(json.dumps({"pid": pid, "start": proc_token(pid)}))
            pending.replace(PIDFILE)
            try:
                print(f"Viewer: http://{bind if bind != '0.0.0.0' else 'localhost'}:"
                      f"{server.server_port}/ (Ctrl-C or ./stop.sh)", flush=True)
                server.serve_forever()
            except KeyboardInterrupt:
                pass
            finally:
                PIDFILE.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description="Measure a fixed 50 × 50 mm PCB target in a printed ArUco cell.")
    parser.add_argument("--camera", default=os.environ.get("SCARA_CAMERA", "/dev/video0"))
    parser.add_argument("--preset", choices=("paper", "pcb"), default="paper")
    parser.add_argument("--polarity", choices=("dark", "bright"), default="dark")
    parser.add_argument("--min-diameter", type=float, default=1.8, help="candidate lower limit, mm")
    parser.add_argument("--max-diameter", type=float, default=4.0, help="candidate upper limit, mm")
    parser.add_argument("--verify", action="store_true", help="compare to nominal; never writes a job")
    parser.add_argument("--reference", type=Path, help="reference CSV in the 50 mm board frame")
    parser.add_argument("--replay", type=Path, help="image file or previous run directory")
    parser.add_argument("--runs-dir", type=Path, default=RUNS)
    parser.add_argument("--serve", action="store_true", help="serve this one run in a browser")
    parser.add_argument("--bind", default="127.0.0.1", help="viewer interface; defaults to local only")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--self-test", action="store_true", help="capture one frame and check camera availability")
    parser.add_argument("--stop-viewer", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.stop_viewer:
        return stop()
    if not (0 < args.min_diameter <= args.max_diameter <= f.BOARD_SIZE):
        parser.error("expected 0 < min-diameter <= max-diameter <= 50 mm")
    if args.verify and args.preset == "pcb" and args.reference is None:
        parser.error("PCB verification requires --reference CSV")
    try:
        if args.replay:
            path = args.replay / "raw.png" if args.replay.is_dir() else args.replay
            frame = cv2.imread(str(path))
            if frame is None:
                raise scan.ScanError(f"Cannot read replay image: {path}")
        else:
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
        directory, success = scan.run(args, frame)
        if args.serve:
            serve(directory, args.bind, args.port)
        return 0 if success else 2
    except (scan.ScanError, OSError, cv2.error) as error:
        print(f"Vision: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
