#!/usr/bin/env bash
# P0-03: no-motion route safety preview.
#
# Pre-condition: nav Product is committed and healthy, the active map is loaded,
# odometry is healthy, and the robot is stationary. This script previews a
# route through Gateway without publishing goal_pose or cmd_vel.

set -e

GOAL_X="${1:-2.0}"
GOAL_Y="${2:-0.0}"

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_route_safety.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-03 Route Safety - $(date) - goal=($GOAL_X, $GOAL_Y) ==="

json_expr() {
  local expr="$1"
  python3 -c "import json,sys; d=json.load(sys.stdin); print($expr)"
}

cmd_source() {
  python3 -c '
import json, sys
d = json.load(sys.stdin)
control = d.get("control") or {}
value = control.get("active_cmd_source")
if value is None:
    value = control.get("command_owner")
value = str(value or "none").strip().lower()
print(value if value not in {"", "unknown", "null", "-"} else "none")
'
}

echo "[1/4] Navigation readiness"
STATUS_JSON="$(curl -sf http://localhost:5050/api/v1/navigation/status)"
echo "$STATUS_JSON" | python3 -m json.tool
HAS_ODOM="$(echo "$STATUS_JSON" | json_expr 'd.get("has_odometry", False)')"
CAN_ACCEPT="$(echo "$STATUS_JSON" | json_expr 'd.get("can_accept_goal", False)')"
SOURCE_BEFORE="$(echo "$STATUS_JSON" | cmd_source)"
if [[ "$HAS_ODOM" != "True" || "$CAN_ACCEPT" != "True" ]]; then
  echo "FAIL: navigation is not ready (has_odometry=$HAS_ODOM can_accept_goal=$CAN_ACCEPT)"
  exit 2
fi
if [[ "$SOURCE_BEFORE" != "none" ]]; then
  echo "FAIL: robot is not idle before preview (active_cmd_source=$SOURCE_BEFORE)"
  exit 3
fi

echo "[2/4] Previewing route without motion"
PLAN_JSON="$(curl -sf -X POST http://localhost:5050/api/v1/navigation/plan \
  -H 'Content-Type: application/json' \
  -d "{\"x\":$GOAL_X,\"y\":$GOAL_Y,\"z\":0.0,\"frame_id\":\"map\",\"client_id\":\"p0_route_safety\"}")"
echo "$PLAN_JSON" | python3 -m json.tool

echo "[3/4] Checking planner and path safety"
FEASIBLE="$(echo "$PLAN_JSON" | json_expr 'd.get("feasible", False)')"
COUNT="$(echo "$PLAN_JSON" | json_expr 'int(d.get("count") or len(d.get("path") or []))')"
SELECTED="$(echo "$PLAN_JSON" | json_expr 'd.get("selected_planner") or d.get("planner") or ""')"
POLICY="$(echo "$PLAN_JSON" | json_expr 'd.get("plan_safety_policy") or ""')"
PATH_OK="$(echo "$PLAN_JSON" | json_expr '(d.get("path_safety") or {}).get("ok")')"
BLOCKED="$(echo "$PLAN_JSON" | json_expr '(d.get("path_safety") or {}).get("blocked_sample_count", 0)')"
if [[ "$FEASIBLE" != "True" || "$COUNT" -lt 2 || -z "$SELECTED" ]]; then
  echo "FAIL: route preview is not feasible (feasible=$FEASIBLE count=$COUNT planner=$SELECTED)"
  exit 4
fi
if [[ -z "$POLICY" || "$POLICY" == "off" || "$PATH_OK" != "True" ]]; then
  echo "FAIL: path safety did not pass (policy=$POLICY path_ok=$PATH_OK blocked=$BLOCKED)"
  exit 5
fi

echo "[4/4] Verifying preview did not start motion"
AFTER_JSON="$(curl -sf http://localhost:5050/api/v1/navigation/status)"
SOURCE_AFTER="$(echo "$AFTER_JSON" | cmd_source)"
if [[ "$SOURCE_AFTER" != "$SOURCE_BEFORE" ]]; then
  echo "FAIL: command source changed during preview ($SOURCE_BEFORE -> $SOURCE_AFTER)"
  exit 6
fi

echo ""
echo "=== PASS - no-motion route preview feasible with path_safety.ok=true ==="
echo "planner=$SELECTED policy=$POLICY blocked_sample_count=$BLOCKED"
echo "Log: $LOG"
