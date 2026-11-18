#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

skip_camera=false
case "${1:-}" in
  --no-camera) skip_camera=true ;;
  "") ;;
  *) echo "Usage: $0 [--no-camera]" >&2; exit 2 ;;
esac

install_packages() {
  if [[ "$(id -u)" == 0 ]]; then
    apt-get install -y "$@"
  elif command -v sudo >/dev/null && sudo -n true 2>/dev/null; then
    sudo -n apt-get install -y "$@"
  elif [[ -t 0 ]] && command -v sudo >/dev/null; then
    sudo apt-get install -y "$@"
  else
    echo "Install packages with: sudo apt-get install -y $*" >&2
    return 1
  fi
}

python="${PYTHON:-python3}"
probe='import cv2, numpy, matplotlib; assert hasattr(cv2, "aruco"), "OpenCV needs ArUco support"; print("OpenCV", cv2.__version__, "| NumPy", numpy.__version__, "| matplotlib", matplotlib.__version__)'
if ! "$python" -c "$probe"; then
  install_packages python3 python3-opencv python3-numpy python3-matplotlib
  "$python" -c "$probe"
fi
"$python" -m vision.fixture
if ! command -v v4l2-ctl >/dev/null && ! install_packages v4l2-utils; then
  echo "Continuing without optional v4l2-ctl; inspect exposure in the captured image." >&2
fi
if ! "$skip_camera"; then
  "$python" -m vision.cli --self-test
fi
echo "Ready. Print Docs/print/aruco_bed_v1.pdf at 100% (Actual size)."
