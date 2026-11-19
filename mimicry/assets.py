"""Install the hand model and a separate MediaPipe environment."""

import hashlib
import subprocess
import venv
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent
VENV = ROOT / ".venv"
MODEL = ROOT / "model" / "gesture_recognizer.task"
GESTURE = ("https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
           "gesture_recognizer/float16/1/gesture_recognizer.task",
           "97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482")


def install():
    if not (VENV / "bin/python").is_file():
        venv.EnvBuilder(with_pip=True).create(VENV)
    subprocess.run([str(VENV / "bin/python"), "-m", "pip", "--disable-pip-version-check",
                    "install", "mediapipe==1.2.3"], check=True)
    with urlopen(GESTURE[0], timeout=30) as response:
        gesture = response.read(16_000_001)
    if hashlib.sha256(gesture).hexdigest() != GESTURE[1]:
        raise ValueError("Gesture model checksum mismatch.")
    MODEL.parent.mkdir(parents=True, exist_ok=True)
    pending = MODEL.with_suffix(".tmp")
    pending.write_bytes(gesture)
    pending.replace(MODEL)
