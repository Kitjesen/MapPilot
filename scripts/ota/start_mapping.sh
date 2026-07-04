#!/bin/bash
set -euo pipefail

NAV_DIR="${NAV_DIR:-/opt/robot/navigation/current}"

if [ ! -f "$NAV_DIR/lingtu.py" ]; then
    echo "ERROR: $NAV_DIR/lingtu.py not found" >&2
    exit 1
fi

cd "$NAV_DIR"
exec python3 lingtu.py map
