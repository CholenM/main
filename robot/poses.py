"""Manual pose observations; joint steps are open-loop, XY is operator-measured."""

import csv
import io
import json
import math
from datetime import datetime, timezone

from .client import RobotError

FIELDS = ("timestamp", "boot", "command_id", "simulated", "homed",
          "j1_steps", "j2_steps", "label", "role", "x_mm", "y_mm")


def observation(status, data):
    label, role = data.get("label", ""), data.get("role", "observation")
    if (not isinstance(label, str) or len(label) > 80 or
            any(not char.isprintable() for char in label)):
        raise RobotError("Enter a printable label of at most 80 characters.")
    if role not in ("observation", "fit", "check"):
        raise RobotError("Invalid pose role.")
    xy = data.get("x_mm"), data.get("y_mm")
    if xy == (None, None) and role == "observation":
        pass
    elif any(type(value) not in (int, float) or not -1e6 <= value <= 1e6
             or not math.isfinite(value) for value in xy):
        raise RobotError("Provide both measured fixture coordinates (X and Y in mm).")
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "boot": status["boot"], "command_id": status["command_id"],
        "simulated": status["simulator"], "homed": status["homed"],
        "j1_steps": status["pos"]["j1"], "j2_steps": status["pos"]["j2"],
        "label": label.strip(), "role": role, "x_mm": xy[0], "y_mm": xy[1],
    }


def append(path, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")


def export_csv(path, simulated):
    output = io.StringIO(newline="")
    writer = csv.DictWriter(output, fieldnames=FIELDS)
    writer.writeheader()
    if path.is_file():
        with path.open(encoding="utf-8") as file:
            for line in file:
                row = json.loads(line)
                if row["simulated"] != simulated:
                    continue
                if row["label"].startswith(("=", "+", "-", "@")):
                    row["label"] = "'" + row["label"]
                writer.writerow(row)
    return output.getvalue()
