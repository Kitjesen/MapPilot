#!/usr/bin/env bash
# P0-06: autonomous exploration start -> observe -> stop.
#
# Pre-condition: LingTu is in a safe open test area and an operator is ready to
# stop the robot manually if needed. Omit --map for live mapping; pass --map MAP
# for saved-map localization.

set -e

DURATION=30
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then
  DURATION="$1"
  shift
fi

EXPLORE_MAP=""
case "$#" in
  0) ;;
  2)
    if [[ "$1" != "--map" || -z "$2" ]]; then
      echo "Usage: $0 [DURATION_SECONDS] [--map MAP]" >&2
      exit 2
    fi
    EXPLORE_MAP="$2"
    ;;
  *)
    echo "Usage: $0 [DURATION_SECONDS] [--map MAP]" >&2
    exit 2
    ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_explore.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-06 Explore - $(date) - duration=${DURATION}s ==="

json_field() {
  local field="$1"
  python3 -c "import json,sys; d=json.load(sys.stdin); print(d.get('$field'))"
}

stop_explore() {
  bash "$REPO_ROOT/scripts/lingtu" explore stop >/dev/null 2>&1 || true
}

echo "[1/6] Baseline health"
curl -sf http://localhost:5050/api/v1/health >/dev/null || {
  echo "FAIL: Gateway down"
  exit 2
}

echo "[2/6] Activating explore Product"
if [[ -n "$EXPLORE_MAP" ]]; then
  bash "$REPO_ROOT/scripts/lingtu" explore start --map "$EXPLORE_MAP"
else
  bash "$REPO_ROOT/scripts/lingtu" explore start
fi
trap stop_explore EXIT

echo "[3/6] Exploration readiness"
STATUS_JSON="$(bash "$REPO_ROOT/scripts/lingtu" explore status)"
echo "$STATUS_JSON" | python3 -m json.tool
AVAILABLE="$(echo "$STATUS_JSON" | json_field available)"
CAN_START="$(echo "$STATUS_JSON" | json_field can_start)"
BACKEND="$(echo "$STATUS_JSON" | json_field backend)"
if [[ "$AVAILABLE" != "True" || "$CAN_START" != "True" ]]; then
  echo "FAIL: exploration is not ready (backend=$BACKEND available=$AVAILABLE can_start=$CAN_START)"
  exit 3
fi

echo ""
echo "[4/6] Starting exploration through the operator CLI"
bash "$REPO_ROOT/scripts/lingtu" explore task start

echo "[5/6] Observing exploration status for ${DURATION}s"
DEADLINE=$((SECONDS + DURATION))
SAW_EXPLORING=0
while [[ $SECONDS -lt $DEADLINE ]]; do
  STATUS_JSON="$(bash "$REPO_ROOT/scripts/lingtu" explore status)"
SUMMARY="$(echo "$STATUS_JSON" | python3 -c '
import json, sys
d = json.load(sys.stdin)
print(
    "backend={} exploring={} frontier_count={} blockers={}".format(
        d.get("backend"),
        d.get("exploring"),
        d.get("frontier_count"),
        d.get("blockers", []),
    )
)
')"
  echo "  $SUMMARY"
  EXPLORING="$(echo "$STATUS_JSON" | json_field exploring)"
  if [[ "$EXPLORING" == "True" ]]; then
    SAW_EXPLORING=1
  fi
  sleep 2
done

if [[ "$SAW_EXPLORING" != "1" ]]; then
  echo "FAIL: exploration never reported exploring=true"
  exit 4
fi

echo "[6/6] Stopping exploration"
bash "$REPO_ROOT/scripts/lingtu" explore stop
trap - EXIT

echo ""
echo "=== PASS - exploration started, reported active, and ProductControl confirmed stop ==="
echo "Log: $LOG"
