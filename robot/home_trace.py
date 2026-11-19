"""Best-effort host telemetry for one homing attempt, not encoder measurements."""

import csv
import json
import time
from datetime import datetime, timezone
from uuid import uuid4

import cv2

FIELDS = ("elapsed_s", "sample", "boot", "command_id", "state", "homed",
          "home_axis", "home_phase", "fault", "j1_steps", "j2_steps",
          "j1_target", "j2_target", "j1_limit", "j2_limit", "z_bottom_limit")
MIME = {"trace.csv": "text/csv; charset=utf-8", "summary.json": "application/json",
        "before.jpg": "image/jpeg", "after.jpg": "image/jpeg"}


class HomeTrace:
    def __init__(self):
        self.id = f"home-{uuid4().hex}"
        self.started = datetime.now(timezone.utc).isoformat()
        self.clock = time.monotonic()
        self.rows = []

    def sample(self, status, kind="poll"):
        row = {key: status.get(key) for key in
               ("boot", "command_id", "state", "homed", "home_axis", "home_phase", "fault")}
        limits, pos, target = (value if isinstance(value, dict) else {} for value in
                               (status.get(key) for key in ("limits", "pos", "target")))
        self.rows.append(dict(row, elapsed_s=round(time.monotonic() - self.clock, 3), sample=kind,
                              j1_steps=pos.get("j1"), j2_steps=pos.get("j2"),
                              j1_target=target.get("j1"), j2_target=target.get("j2"),
                              j1_limit=limits.get("j1"), j2_limit=limits.get("j2"),
                              z_bottom_limit=limits.get("z")))

    def save(self, root, frames, *, outcome, error, simulated):
        directory = root / "homing" / self.id
        directory.mkdir(parents=True)
        with (directory / "trace.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        files = ["trace.csv", "summary.json"]
        for name, frame in frames.items():
            if frame is not None:
                try:
                    if cv2.imwrite(str(directory / name), frame):
                        files.append(name)
                except cv2.error:
                    pass
        summary = {"id": self.id, "started_utc": self.started, "outcome": outcome,
                   "error": error, "simulated": simulated, "sample_count": len(self.rows),
                   "position_units": "commanded_steps_not_encoder_feedback",
                   "files": files}
        (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                                 encoding="utf-8")
        return directory, summary
