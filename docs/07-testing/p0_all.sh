#!/usr/bin/env bash
# p0_all.sh - run core P0 scripts in sequence. Each one logs to its own file;
# this script collates a summary. Exploration is opt-in because it requires an
# explicit ProductControl switch and a safe open area.

set -e
cd "$(dirname "$0")"
REPO_ROOT="$(cd "../.." && pwd)"

STAMP=$(date +%Y%m%d_%H%M%S)
MAP_NAME="${LINGTU_P0_MAP_NAME:-p0_${STAMP}}"
GOAL_X="${LINGTU_P0_GOAL_X:-${1:-}}"
GOAL_Y="${LINGTU_P0_GOAL_Y:-${2:-}}"
GOTO_TIMEOUT="${LINGTU_P0_GOTO_TIMEOUT:-${3:-60}}"
EXPLORE_MAP="${LINGTU_P0_EXPLORE_MAP:-}"
if [[ -z "$GOAL_X" || -z "$GOAL_Y" ]]; then
  echo "FAIL: set LINGTU_P0_GOAL_X and LINGTU_P0_GOAL_Y, or pass GOAL_X GOAL_Y [TIMEOUT]."
  echo "      The aggregate P0 flow will not choose a motion target automatically."
  exit 2
fi
LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
SUMMARY="$LOG_DIR/${STAMP}_p0_all_summary.log"
exec > >(tee -a "$SUMMARY") 2>&1

echo "=== P0 ALL - $(date) ==="
echo "map=$MAP_NAME goal=($GOAL_X, $GOAL_Y) goto_timeout=${GOTO_TIMEOUT}s"

ran=0
passed=0

run_one() {
  local name="$1"
  local script="$2"
  shift 2
  ran=$((ran + 1))
  echo ""
  echo "------------ $name ------------"
  if bash "$script" "$@"; then
    echo "[SUMMARY] $name  PASS"
    passed=$((passed + 1))
  else
    local code=$?
    echo "[SUMMARY] $name  FAIL (exit $code)"
    return "$code"
  fi
}


run_one "P0-01 cold boot"   p0_cold_boot.sh
run_one "P0-02 mapping"     p0_mapping.sh "$MAP_NAME"
run_one "P0-03/P0-04 route preview + goto" p0_goto.sh "$GOAL_X" "$GOAL_Y" "$GOTO_TIMEOUT"
run_one "P0-05 estop"       p0_estop.sh
if [[ "${LINGTU_P0_RUN_EXPLORE:-0}" == "1" ]]; then
  echo ""
  if [[ -n "$EXPLORE_MAP" ]]; then
    run_one "P0-06 explore (saved map: $EXPLORE_MAP)" p0_explore.sh \
      "${LINGTU_P0_EXPLORE_DURATION:-30}" --map "$EXPLORE_MAP"
  else
    run_one "P0-06 explore (live mapping)" p0_explore.sh \
      "${LINGTU_P0_EXPLORE_DURATION:-30}"
  fi
else
  echo ""
  echo "[SUMMARY] P0-06 explore SKIP (set LINGTU_P0_RUN_EXPLORE=1 in a safe area; optionally set LINGTU_P0_EXPLORE_MAP=MAP)"
fi

echo ""
echo "=== P0 ALL COMPLETE: ${passed}/${ran} passed ==="
echo "Summary log: $SUMMARY"
[[ "$passed" -eq "$ran" ]] || exit 1
