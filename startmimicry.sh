#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$root${PYTHONPATH:+:$PYTHONPATH}"
python="${PYTHON:-python3}"
if [[ -z "${PYTHON:-}" && -x "$root/mimicry/.venv/bin/python" ]]; then
  python="$root/mimicry/.venv/bin/python"
fi
exec "$python" -m mimicry.app "$@"
