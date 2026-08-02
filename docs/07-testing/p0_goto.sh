#!/usr/bin/env bash
# P0-04: point goal -> arrival.
#
# Pre-condition: nav Product is committed and the active map is already loaded. The
# script first runs the no-motion route preview, then requires operator
# confirmation before sending /api/v1/goal and waiting for SUCCESS.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GOAL_X="${LINGTU_P0_GOAL_X:-${1:-}}"
GOAL_Y="${LINGTU_P0_GOAL_Y:-${2:-}}"
TIMEOUT="${LINGTU_P0_GOTO_TIMEOUT:-${3:-60}}"

if [[ -z "$GOAL_X" || -z "$GOAL_Y" ]]; then
  echo "FAIL: set LINGTU_P0_GOAL_X and LINGTU_P0_GOAL_Y, or pass GOAL_X GOAL_Y [TIMEOUT]."
  echo "      P0-04 will not choose a motion target automatically."
  exit 2
fi

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_goto.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-04 Goto - $(date) - goal=($GOAL_X, $GOAL_Y) timeout=${TIMEOUT}s ==="

echo "[1/5] Sanity"
HEALTH="$(curl -sf http://localhost:5050/api/v1/health)"
HAS_ODOM="$(echo "$HEALTH" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("has_odom", False))')"
if [[ "$HAS_ODOM" != "True" ]]; then
  echo "FAIL: no odometry - is the nav Product healthy?"
  exit 2
fi

echo "[2/5] Previewing route without motion"
bash "$SCRIPT_DIR/p0_route_safety.sh" "$GOAL_X" "$GOAL_Y"

echo "[3/5] Confirming motion goal"
echo "Route preview passed for goal=($GOAL_X, $GOAL_Y)."
echo "Confirm the displayed path is safe in the real test area before motion."
echo "Type RUN to send the goal through /api/v1/goal:"
read -r answer
if [[ "$answer" != "RUN" ]]; then
  echo "FAIL: operator did not confirm motion goal"
  exit 3
fi

echo "[4/5] Sending goal"
curl -sf -X POST http://localhost:5050/api/v1/goal \
  -H 'Content-Type: application/json' \
  -d "{\"x\":$GOAL_X,\"y\":$GOAL_Y,\"z\":0.0,\"frame_id\":\"map\",\"client_id\":\"p0_goto\",\"request_id\":\"p0-goto-$(date +%s)\"}" \
  | python3 -m json.tool

echo "[5/5] Polling mission state (timeout ${TIMEOUT}s) ..."
DEADLINE=$((SECONDS + TIMEOUT))
LAST_STATE="?"
while [[ $SECONDS -lt $DEADLINE ]]; do
  STATUS_JSON="$(curl -sf http://localhost:5050/api/v1/navigation/status 2>/dev/null || echo '{}')"
  STATUS="$(echo "$STATUS_JSON" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("state","?"))')"
  if [[ "$STATUS" != "$LAST_STATE" ]]; then
    echo "  nav state: $LAST_STATE -> $STATUS"
    LAST_STATE="$STATUS"
  fi
  case "$STATUS" in
    SUCCESS)    echo ""; echo "=== PASS - reached goal in $SECONDS s ==="; exit 0 ;;
    FAILED)     echo ""; echo "FAIL - navigation FAILED state"; exit 3 ;;
    CANCELLED)  echo ""; echo "FAIL - cancelled"; exit 4 ;;
  esac
  sleep 2
done

echo ""
echo "FAIL - timeout after ${TIMEOUT}s, last state = $LAST_STATE"
exit 5
