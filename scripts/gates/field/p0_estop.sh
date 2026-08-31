#!/usr/bin/env bash
# P0-05: emergency stop reflex.
#
# Verifies the Gateway stop command is accepted and that the robot-reported
# navigation speed settles to zero after the stop. This script uses current
# Gateway contracts only: POST /api/v1/stop and GET /api/v1/state.

set -e

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_estop.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-05 E-stop reflex - $(date) ==="

STOP_ON_EXIT=0
STOP_SENT=0

stop_robot() {
  curl -sf -X POST http://localhost:5050/api/v1/stop \
    -H 'Content-Type: application/json' \
    -d '{"request_id":"p0-estop-cleanup","client_id":"p0_estop"}' \
    >/dev/null 2>&1 || true
  STOP_SENT=1
}

cleanup_stop() {
  if [[ "$STOP_ON_EXIT" == "1" && "$STOP_SENT" != "1" ]]; then
    echo "Cleanup: sending stop command before exiting P0-05"
    stop_robot
  fi
}

trap cleanup_stop EXIT

read_speed() {
  curl -sf http://localhost:5050/api/v1/state 2>/dev/null | python3 -c '
import json, sys
d = json.load(sys.stdin)
motion = d.get("navigation", {}).get("motion", {})
value = motion.get("current_speed_mps")
if value is None:
    raise SystemExit(2)
print(value)
'
}

echo "[1/4] Baseline health"
curl -sf http://localhost:5050/api/v1/health > /dev/null || {
  echo "FAIL: Gateway down"
  exit 2
}

echo "[2/4] Please manually drive the robot so reported speed is non-zero."
echo "      Press [Enter] when ready to issue Gateway stop."
read -r
STOP_ON_EXIT=1
PRE_STOP_SPEED="$(read_speed 2>/dev/null || echo "")"
if [[ -z "$PRE_STOP_SPEED" ]]; then
  echo "FAIL: cannot read pre-stop navigation.motion.current_speed_mps"
  exit 3
fi
if [[ "$(python3 -c "print(abs(float('$PRE_STOP_SPEED')) >= 0.03)")" != "True" ]]; then
  echo "FAIL: pre-stop speed is not non-zero enough for an estop test (speed=$PRE_STOP_SPEED)"
  exit 4
fi
echo "  pre-stop speed: $PRE_STOP_SPEED m/s"

echo "[3/4] Sending stop command and measuring RPC latency ..."
T0=$(date +%s%3N)
curl -sf -X POST http://localhost:5050/api/v1/stop \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"p0-estop","client_id":"p0_estop"}' \
  | python3 -m json.tool
STOP_SENT=1
STOP_ON_EXIT=0
T1=$(date +%s%3N)
RTT=$((T1 - T0))
echo "  stop RPC round-trip: ${RTT} ms"

echo "[4/4] Waiting for navigation speed -> zero (deadline 2s) ..."
ZERO_TICKS=0
DEADLINE=$((SECONDS + 2))
LAST_SPEED="?"
while [[ $SECONDS -lt $DEADLINE ]]; do
  SPEED="$(read_speed 2>/dev/null || echo "1.0")"
  LAST_SPEED="$SPEED"
  if [[ "$(python3 -c "print(abs(float('$SPEED')) < 0.01)")" == "True" ]]; then
    ZERO_TICKS=$((ZERO_TICKS + 1))
    [[ "$ZERO_TICKS" -ge 3 ]] && break
  else
    ZERO_TICKS=0
  fi
  sleep 0.1
done

if [[ "$ZERO_TICKS" -lt 3 ]]; then
  echo "FAIL: reported navigation speed did not reach zero within 2s (speed=$LAST_SPEED)"
  exit 5
fi

echo ""
echo "=== PASS - stop accepted (rpc=${RTT}ms), speed ${PRE_STOP_SPEED} -> zero ==="
echo "Log: $LOG"
