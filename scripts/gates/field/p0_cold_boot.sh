#!/usr/bin/env bash
# P0-01: S100P cold boot - ProductControl map start, Gateway stable 3 minutes.
#
# Run on sunrise (not local dev):
#   ssh sunrise@192.168.66.190
#   cd ~/data/SLAM/navigation
#   bash scripts/gates/field/p0_cold_boot.sh

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_cold_boot.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-01 Cold Boot - $(date) ==="

# 1. Start the mapping Product. ProductControl owns staging, process launch,
# readiness, rollback, and current RunPlan commit.
echo "[1/3] Starting map Product ..."
bash "$REPO_ROOT/scripts/lingtu" switch map

# 2. Poll Gateway health (max 60s)
echo "[2/3] Polling Gateway /api/v1/health ..."
DEADLINE=$((SECONDS + 60))
while ! curl -sf http://localhost:5050/api/v1/health > /dev/null; do
  if [[ $SECONDS -gt $DEADLINE ]]; then
    echo "FAIL: Gateway not responsive after 60s"
    exit 2
  fi
  sleep 2
done

# 3. 3-minute stability watch - no module should report failed.
echo "[3/3] 3-minute stability watch (polling every 15s) ..."
LAST_MODULES_OK="?"
for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
  sleep 15
  HEALTH_JSON="$(curl -s http://localhost:5050/api/v1/health)"
  MODULES_OK="$(echo "$HEALTH_JSON" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("modules_ok", "?"))' 2>/dev/null || echo "?")"
  MODULES_FAIL="$(echo "$HEALTH_JSON" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("modules_fail", "?"))' 2>/dev/null || echo "?")"
  LAST_MODULES_OK="$MODULES_OK"
  echo "  tick $i/12  modules_ok=$MODULES_OK modules_fail=$MODULES_FAIL"
  if [[ "$MODULES_FAIL" != "0" ]]; then
    echo "FAIL: module failure detected at tick $i"
    echo "$HEALTH_JSON" | python3 -m json.tool
    exit 3
  fi
done

echo ""
echo "=== PASS - map Product cold boot stable 3 minutes, modules_ok=$LAST_MODULES_OK ==="
echo "Log: $LOG"
