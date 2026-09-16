#!/usr/bin/env bash
# P0-03: no-motion route safety preview.
#
# Pre-condition: nav Product is committed and healthy, the active map is loaded,
# odometry is healthy, and the robot is stationary. This script previews a
# route through Gateway without publishing goal_pose or cmd_vel.

set -e

GOAL_X="${1:-2.0}"
GOAL_Y="${2:-0.0}"
AUTH_ARGS=()
if [[ -n "${LINGTU_API_KEY:-}" ]]; then
  AUTH_ARGS=(-H "X-API-Key: ${LINGTU_API_KEY}")
fi

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_route_safety.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-03 Route Safety - $(date) - goal=($GOAL_X, $GOAL_Y) ==="

json_expr() {
  local expr="$1"
  python3 -c "import json,sys; d=json.load(sys.stdin); print($expr)"
}

status_axes() {
  python3 -c '
import json, sys
d = json.load(sys.stdin)
task = d.get("task") or {}
admission = d.get("goal_admission") or {}
control = d.get("control") or {}
motion = d.get("motion") or {}
values = (
    task.get("state") or "UNKNOWN",
    admission.get("state") or "UNKNOWN",
    control.get("authority") or "UNKNOWN",
    str(control.get("resume_required", "UNKNOWN")).upper(),
    motion.get("permission") or "UNKNOWN",
    motion.get("observation") or "UNKNOWN",
    motion.get("stop_confirmation") or "UNKNOWN",
)
print("|".join(str(value).strip().upper() for value in values))
'
}

echo "[1/4] Navigation readiness"
STATUS_JSON="$(curl -sf "${AUTH_ARGS[@]}" http://localhost:5050/api/v1/navigation/status)"
echo "$STATUS_JSON" | python3 -m json.tool
AXES_BEFORE="$(echo "$STATUS_JSON" | status_axes)"
IFS='|' read -r TASK ADMISSION AUTHORITY RESUME PERMISSION OBSERVATION STOP_CONFIRMATION <<< "$AXES_BEFORE"
if [[ "$TASK" != "IDLE" && "$TASK" != "SUCCESS" && "$TASK" != "FAILED" && "$TASK" != "CANCELLED" ]]; then
  echo "FAIL: navigation task is not quiescent (task=$TASK)"
  exit 2
fi
if [[ "$ADMISSION" != "ACCEPTING" || "$AUTHORITY" != "NONE" || "$RESUME" != "FALSE" || \
      "$PERMISSION" != "CLEAR" || "$OBSERVATION" != "QUIET" || \
      ( "$STOP_CONFIRMATION" != "NOT_REQUESTED" && "$STOP_CONFIRMATION" != "CONFIRMED" ) ]]; then
  echo "FAIL: navigation is not safe for preview (axes=$AXES_BEFORE)"
  exit 3
fi

echo "[2/4] Previewing route without motion"
PLAN_JSON="$(curl -sf "${AUTH_ARGS[@]}" -X POST http://localhost:5050/api/v1/navigation/plan \
  -H 'Content-Type: application/json' \
  -d "{\"x\":$GOAL_X,\"y\":$GOAL_Y,\"z\":0.0,\"frame_id\":\"map\"}")"
echo "$PLAN_JSON" | python3 -m json.tool

echo "[3/4] Checking native planner result"
FEASIBLE="$(echo "$PLAN_JSON" | json_expr 'd.get("feasible", False)')"
COUNT="$(echo "$PLAN_JSON" | json_expr 'int(d.get("count") or len(d.get("path") or []))')"
PLANNER="$(echo "$PLAN_JSON" | json_expr 'd.get("planner") or ""')"
if [[ "$FEASIBLE" != "True" || "$COUNT" -lt 2 || -z "$PLANNER" ]]; then
  echo "FAIL: route preview is not feasible (feasible=$FEASIBLE count=$COUNT planner=$PLANNER)"
  exit 4
fi

echo "[4/4] Verifying preview did not start motion"
AFTER_JSON="$(curl -sf "${AUTH_ARGS[@]}" http://localhost:5050/api/v1/navigation/status)"
AXES_AFTER="$(echo "$AFTER_JSON" | status_axes)"
if [[ "$AXES_AFTER" != "$AXES_BEFORE" ]]; then
  echo "FAIL: navigation state changed during preview ($AXES_BEFORE -> $AXES_AFTER)"
  exit 6
fi

echo ""
echo "=== PASS - native no-motion route preview is feasible ==="
echo "planner=$PLANNER count=$COUNT"
echo "Log: $LOG"
