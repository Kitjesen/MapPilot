#!/usr/bin/env bash
# P0-02: map -> save -> transition to nav Product.
#
# Interactive: requires a human walking the robot around for ~3 minutes.
# Pre-condition: LingTu mapping Product is already running.
#
# Post-condition: a new native durable navigation map package is saved under:
#   $NAV_MAP_DIR, then ~/data/nova/maps.
#   Required current artifacts: map.pcd + metadata.json + occupancy.npz + octomap.ot.
#   map.pgm/map.yaml may also be present when produced by the compatibility path.
#   ProductControl has committed the nav RunPlan for this map.

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

resolve_map_root() {
  if [[ -n "${NAV_MAP_DIR:-}" ]]; then
    echo "$NAV_MAP_DIR"
    return
  fi
  echo "${MAP_DIR:-$HOME/data/nova/maps}"
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

# 3. Save through the canonical operator command. It submits the dedicated
# /api/v1/map/save request and waits on /api/v1/maps/operations/{operation_id}.
echo "[3/6] Saving durable map package: $MAP_NAME"
bash "$REPO_ROOT/scripts/lingtu" map save "$MAP_NAME"

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
bash "$REPO_ROOT/scripts/lingtu" nav start "$MAP_NAME"

# 6. Print committed state for operator review. A successful nav start is the
# ProductControl transaction boundary; this script does not parse RunPlan internals.
echo "[6/6] ProductControl status"
bash "$REPO_ROOT/scripts/lingtu" status --explain | python3 -m json.tool

echo ""
echo "=== PASS - map '$MAP_NAME' saved + nav Product committed ==="
echo "Next:"
echo "  Continue with P0 route preview/goto checks against the committed nav Product."
echo ""
echo "Log: $LOG"
