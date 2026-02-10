#!/bin/bash
# reload_dog_model.sh — Notify Dog Board to hot-reload a model after OTA copy
#
# Called by ota_daemon after RELOAD_MODEL apply action for dog-targeted artifacts.
# The Dog Board (han_dog CMS) does not have a gRPC reload API, so we use a
# pragmatic signal approach:
#
#   1. Write a trigger file that the Dog Board process watches (inotify)
#   2. If a TCP health check confirms the Dog Board is reachable
#
# Architecture:
#   App → (gRPC) → Nav Board ota_daemon → copy model → THIS SCRIPT → Dog Board
#
# Usage: reload_dog_model.sh <model_path> <dog_host> <dog_port>
#
# Arguments:
#   model_path  — Absolute path to the newly installed model file
#   dog_host    — Dog Board IP address (e.g., 192.168.123.161)
#   dog_port    — Dog Board CMS gRPC port (e.g., 13145)
#
# Exit codes:
#   0  — Notification sent successfully
#   1  — Dog Board unreachable
#   2  — Invalid arguments

set -euo pipefail

MODEL_PATH="${1:-}"
DOG_HOST="${2:-}"
DOG_PORT="${3:-13145}"
TRIGGER_DIR="/home/sunrise/models/.reload_triggers"
TIMEOUT=5

if [ -z "$MODEL_PATH" ] || [ -z "$DOG_HOST" ]; then
  echo "Usage: $0 <model_path> <dog_host> [dog_port]"
  exit 2
fi

MODEL_NAME="$(basename "$MODEL_PATH")"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"

echo "[reload_dog_model] Model: $MODEL_PATH"
echo "[reload_dog_model] Dog Board: $DOG_HOST:$DOG_PORT"

# ── Step 1: Check Dog Board reachability ──
echo "[reload_dog_model] Checking Dog Board connectivity..."
if ! timeout "$TIMEOUT" bash -c "echo > /dev/tcp/$DOG_HOST/$DOG_PORT" 2>/dev/null; then
  echo "[reload_dog_model] ERROR: Dog Board unreachable at $DOG_HOST:$DOG_PORT"
  exit 1
fi
echo "[reload_dog_model] Dog Board reachable"

# ── Step 2: Write reload trigger file ──
# The Dog Board's RL controller watches this directory for .trigger files.
# Each trigger file contains metadata about which model was updated.
mkdir -p "$TRIGGER_DIR"
TRIGGER_FILE="${TRIGGER_DIR}/${MODEL_NAME}.trigger"
cat > "$TRIGGER_FILE" <<EOF
{
  "model_path": "$MODEL_PATH",
  "model_name": "$MODEL_NAME",
  "timestamp": "$TIMESTAMP",
  "source": "ota_daemon"
}
EOF

echo "[reload_dog_model] Trigger written: $TRIGGER_FILE"

# ── Step 3: Optional — send UDP signal for immediate reload ──
# Some Dog Board firmware listens on a UDP reload port (configurable).
# This provides faster notification than filesystem polling.
RELOAD_UDP_PORT=13200
echo -n "RELOAD:${MODEL_NAME}:${TIMESTAMP}" | \
  timeout 2 bash -c "cat > /dev/udp/$DOG_HOST/$RELOAD_UDP_PORT" 2>/dev/null || true

echo "[reload_dog_model] Done — model reload notification sent for $MODEL_NAME"
exit 0
