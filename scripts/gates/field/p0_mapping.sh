#!/usr/bin/env bash
# P0-02: map -> save -> transition to nav Product.
#
# Interactive: requires a human walking the robot around for ~3 minutes.
# Pre-condition: LingTu mapping Product is already running.
#
# Post-condition: a new native durable navigation map package is saved under:
#   $NAV_MAP_DIR, defaulting to /var/lib/lingtu/maps on the field target.
#   Required current artifacts: map.pcd + metadata.json + occupancy.npz + octomap.ot.
#   map.pgm/map.yaml may also be present when produced by the compatibility path.
#   ProductControl has committed the nav RunPlan for this map.

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

resolve_map_root() {
  echo "${NAV_MAP_DIR:-/var/lib/lingtu/maps}"
}

MAP_NAME="${1:-p0_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_mapping.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-02 Mapping - $(date) - map=$MAP_NAME ==="

# 1. Sanity: is SLAM running?
echo "[1/6] Sanity check - Gateway + SLAM responsive ..."
SLAM_HZ="$(curl -sf http://localhost:5050/api/v1/health | python3 -c 'import json,sys; print(json.load(sys.stdin).get("slam_hz", 0))')"
if [[ "$(python3 -c "print($SLAM_HZ > 0.5)")" != "True" ]]; then
  echo "FAIL: SLAM rate $SLAM_HZ Hz - is the mapping Product healthy?"
  exit 2
fi
echo "  SLAM Hz = $SLAM_HZ"

# 2. Prompt human to walk the robot
echo ""
echo "[2/6] Please walk the robot around the target area for ~3 minutes."
echo "      Press [Enter] when mapping is complete ..."
read -r

# 3. Save through the authenticated Gateway map API.
echo "[3/6] Saving durable map package: $MAP_NAME"
GATEWAY_URL="${LINGTU_GATEWAY_URL:-http://127.0.0.1:5050}"
MAP_API_KEY="${LINGTU_MAP_API_KEY:?set LINGTU_MAP_API_KEY}"
SAVE_JSON="$(curl -fsS -X POST "$GATEWAY_URL/api/v1/map/save" \
  -H "Authorization: Bearer $MAP_API_KEY" \
  -H "Content-Type: application/json" \
  --data-binary "{\"name\":\"$MAP_NAME\"}")"
OPERATION_ID="$(printf '%s' "$SAVE_JSON" | python3 -c \
  'import json,sys; print(json.load(sys.stdin)["operation_id"])')"
while :; do
  OPERATION_JSON="$(curl -fsS \
    -H "Authorization: Bearer $MAP_API_KEY" \
    "$GATEWAY_URL/api/v1/maps/operations/$OPERATION_ID")"
  OPERATION_STATE="$(printf '%s' "$OPERATION_JSON" | python3 -c \
    'import json,sys; print((json.load(sys.stdin).get("operation") or {}).get("state", ""))')"
  case "$OPERATION_STATE" in
    SUCCEEDED) break ;;
    FAILED|CANCELLED) echo "$OPERATION_JSON"; exit 3 ;;
  esac
  sleep 1
done

# 4. Verify artifacts
MAP_ROOT="$(resolve_map_root)"
SAVED_MAP_DIR="$MAP_ROOT/$MAP_NAME"
echo "[4/6] Verifying artifacts at $SAVED_MAP_DIR"
for f in map.pcd metadata.json occupancy.npz octomap.ot; do
  if [[ -f "$SAVED_MAP_DIR/$f" ]]; then
    SZ=$(stat -c%s "$SAVED_MAP_DIR/$f")
    echo "  OK $f  ($SZ bytes)"
  else
    echo "  MISSING $f"
    exit 4
  fi
done
for f in map.pgm map.yaml; do
  if [[ -f "$SAVED_MAP_DIR/$f" ]]; then
    SZ=$(stat -c%s "$SAVED_MAP_DIR/$f")
    echo "  OK optional $f  ($SZ bytes)"
  fi
done

# 5. Transition through the thin operator CLI. ProductControl owns map staging,
# native process switching, readiness, rollback, and current RunPlan commit.
echo "[5/6] Starting nav Product for map: $MAP_NAME"
bash "$REPO_ROOT/scripts/lingtu" switch nav --map "$MAP_NAME"

# 6. Print committed state for operator review. A successful nav start is the
# ProductControl transaction boundary; this script does not parse RunPlan internals.
echo "[6/6] ProductControl status"
bash "$REPO_ROOT/scripts/lingtu" status --json | python3 -m json.tool

echo ""
echo "=== PASS - map '$MAP_NAME' saved + nav Product committed ==="
echo "Next:"
echo "  Continue with P0 route preview/goto checks against the committed nav Product."
echo ""
echo "Log: $LOG"
