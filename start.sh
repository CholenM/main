#!/usr/bin/env bash
set -euo pipefail
# Resolve relative option paths from the caller's working directory.
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$root${PYTHONPATH:+:$PYTHONPATH}"
if [[ $# -eq 0 ]]; then
  set -- --serve --console
fi
exec "${PYTHON:-python3}" -m vision.cli "$@"
