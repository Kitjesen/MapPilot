#!/usr/bin/env bash
# ==============================================================================
# LingTu CI test runner éˆ?fast tests first, slow/ROS2/sim tests last.
#
# Usage:
#   bash scripts/dev/run_tests.sh                     # all tests (fast-first ordering)
#   bash scripts/dev/run_tests.sh --quick             # only fast tests (skips ros2 + sim)
#   bash scripts/dev/run_tests.sh --coverage          # all tests with coverage report
#
# Exit code: number of failed test groups (0 = all passed).
# ==============================================================================

set -o pipefail

find_repo_root() {
    local dir="$1"
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/pyproject.toml" && -f "$dir/AGENTS.md" ]]; then
            printf '%s\n' "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    return 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(find_repo_root "$SCRIPT_DIR")"
cd "$REPO_ROOT"

PYTEST="python -m pytest"
FLAGS="-q --tb=short"
EXIT_CODE=0
RUN_ALL=true

if [[ "$1" == "--quick" ]]; then
    RUN_ALL=false
elif [[ "$1" == "--coverage" ]]; then
    FLAGS="$FLAGS --cov=src --cov-report=term-missing"
fi

echo "============================================"
echo " LingTu CI Test Suite"
echo " $(date)"
echo "============================================"
echo ""

# ======================================================================
# Tier 1 éˆ?Core framework (fastest, highest value)
# ======================================================================
echo "--- Tier 1: Core framework ---"
if ! $PYTEST src/runtime/tests/ -m "not ros2 and not sim" $FLAGS; then
    echo "FAIL: Tier 1"
    EXIT_CODE=1
fi
echo ""

# ======================================================================
# Tier 2 éˆ?Memory, perception/decision, webrtc (pure Python, fast)
# ======================================================================
echo "--- Tier 2: Memory ---"
if ! $PYTEST src/memory/tests/ $FLAGS; then
    echo "FAIL: Tier 2 (memory)"
    EXIT_CODE=1
fi

echo "--- Tier 2: Perception / Decision ---"
if ! $PYTEST src/perception/tests/ src/decision/tests/ $FLAGS; then
    echo "FAIL: Tier 2 (perception/decision)"
    EXIT_CODE=1
fi

echo "--- Tier 2: WebRTC ---"
if ! $PYTEST src/gateway/tests/ $FLAGS; then
    echo "FAIL: Tier 2 (webrtc)"
    EXIT_CODE=1
fi
echo ""

# ======================================================================
# Tier 3 éˆ?Navigation, slam, exploration (pure Python, moderate)
# ======================================================================
echo "--- Tier 3: Navigation (no ros2) ---"
if ! $PYTEST src/nav/tests/ -m "not ros2" $FLAGS; then
    echo "FAIL: Tier 3 (nav)"
    EXIT_CODE=1
fi

echo "--- Tier 3: SLAM (no ros2) ---"
if ! $PYTEST src/localization/tests/ -m "not ros2" $FLAGS; then
    echo "FAIL: Tier 3 (slam)"
    EXIT_CODE=1
fi

echo "--- Tier 3: Exploration (no ros2) ---"
if ! $PYTEST src/nav/tests/exploration/ -m "not ros2" $FLAGS; then
    echo "FAIL: Tier 3 (exploration)"
    EXIT_CODE=1
fi
echo ""

# ======================================================================
# Tier 4 éˆ?Drivers, base_autonomy
# ======================================================================
echo "--- Tier 4: Drivers (no sim) ---"
if ! $PYTEST tests/drivers/ -m "not sim" $FLAGS; then
    echo "FAIL: Tier 4 (drivers)"
    EXIT_CODE=1
fi

echo "--- Tier 4: Nav local autonomy ---"
if ! $PYTEST src/nav/tests/local/ $FLAGS; then
    echo "FAIL: Tier 4 (nav local autonomy)"
    EXIT_CODE=1
fi
echo ""

# ======================================================================
# Tier 5 éˆ?Global planning (isolated C++ tests, if .so available)
# ======================================================================
if [[ -f src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/libpy_planner.so ]]; then
    echo "--- Tier 5: Nav planning backends ---"
    if ! $PYTEST src/nav/tests/planning_backends/ $FLAGS; then
        echo "FAIL: Tier 5 (nav planning backends)"
        EXIT_CODE=1
    fi
    echo ""
fi

# ======================================================================
# Tier 6 éˆ?Gateway (fastapi-based, may need deps)
# ======================================================================
echo "--- Tier 6: Gateway ---"
if ! $PYTEST src/gateway/tests/test_gateway_helpers.py src/gateway/tests/test_gateway_app_bootstrap.py $FLAGS; then
    echo "FAIL: Tier 6 (gateway core)"
    EXIT_CODE=1
fi
echo ""

# ======================================================================
# Tier 7 éˆ?Slow / ROS2 / Simulation (if --all)
# ======================================================================
if $RUN_ALL; then
    echo "--- Tier 7: ROS2 tests ---"
    if ! $PYTEST src/ -m "ros2" $FLAGS; then
        echo "FAIL: Tier 7 (ros2)"
        EXIT_CODE=1
    fi

    echo "--- Tier 7: Simulation tests ---"
    if ! $PYTEST src/ -m "sim" $FLAGS; then
        echo "FAIL: Tier 7 (sim)"
        EXIT_CODE=1
    fi
    echo ""
fi

# ======================================================================
# Summary
# ======================================================================
echo "============================================"
if [[ $EXIT_CODE -eq 0 ]]; then
    echo " ALL TIERS PASSED"
else
    echo " $EXIT_CODE tier(s) FAILED"
fi
echo " $(date)"
echo "============================================"
exit $EXIT_CODE
