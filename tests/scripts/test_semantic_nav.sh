#!/bin/bash
# ============================================================
# test_semantic_nav.sh 鈥?semantic navigation repository smoke checks
#
# Usage:
#   bash tests/scripts/test_semantic_nav.sh [--unit-only]
#
# This is a repo-layout smoke script for the current Module-First Python
# semantic navigation stack under src/perception/ and src/decision/. ROS2/Flutter legacy checks were removed
# from this relocated script because those surfaces no longer own the active
# decision/perception runtime contract.
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

UNIT_ONLY=false
if [[ "${1:-}" == "--unit-only" ]]; then
  UNIT_ONLY=true
fi

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass() { echo -e "${GREEN}[PASS]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; exit 1; }
info() { echo -e "${YELLOW}[INFO]${NC} $1"; }

check_file() {
  if [ -f "$REPO_ROOT/$1" ]; then
    pass "$1 exists"
  else
    fail "$1 missing"
  fi
}

check_dir() {
  if [ -d "$REPO_ROOT/$1" ]; then
    pass "$1/ exists"
  else
    fail "$1/ missing"
  fi
}

info "Phase 0: Checking current semantic stack layout..."
check_file "AGENTS.md"
check_file "pyproject.toml"
check_file "config/perception.yaml"
check_file "config/decision.yaml"
check_file "config/topic_contract.yaml"
check_dir  "src/perception"
check_dir  "src/decision"
check_dir  "src/perception/reconstruction"
check_file "src/perception/__init__.py"
check_file "src/decision/modules/semantic_planner_module.py"
check_file "src/decision/goal_resolution/goal_resolver.py"
check_file "src/decision/goal_resolution/slow_path.py"

info "Phase 1: Checking semantic runtime contracts..."
if grep -q "semantic:" "$REPO_ROOT/config/topic_contract.yaml"; then
  pass "semantic section in topic_contract.yaml"
else
  fail "semantic section missing from topic_contract.yaml"
fi

if grep -q "resolved_goal:" "$REPO_ROOT/config/topic_contract.yaml"; then
  pass "resolved_goal semantic contract defined"
else
  fail "resolved_goal semantic contract missing"
fi

if grep -q "topo_summary" "$REPO_ROOT/src/decision/modules/semantic_planner_module.py"; then
  pass "SemanticPlanner topo_summary port present"
else
  fail "SemanticPlanner topo_summary port missing"
fi

if grep -q "room_graph" "$REPO_ROOT/src/decision/modules/semantic_planner_module.py"; then
  pass "SemanticPlanner room_graph port present"
else
  fail "SemanticPlanner room_graph port missing"
fi

info "Phase 2: Running semantic unit smoke tests..."
cd "$REPO_ROOT"
export PYTHONPATH="$REPO_ROOT/src:${PYTHONPATH:-}"
PYTHON_BIN="${PYTHON_BIN:-python}"
env -u PYTHONHOME -u UV_INTERNAL__PYTHONHOME -u PYTHONPATH "$PYTHON_BIN" -m pytest \
  src/decision/tests/test_domain_imports.py \
  src/decision/tests/test_planner_node_init.py \
  -q
pass "semantic unit smoke tests"

if $UNIT_ONLY; then
  info "Unit-only mode 鈥?skipping optional profile smoke."
  echo ""
  echo "============================================================"
  echo -e "${GREEN} Semantic repo-layout and unit smoke checks PASSED${NC}"
  echo "============================================================"
  exit 0
fi

info "Phase 3: Running CLI profile list smoke..."
env -u PYTHONHOME -u UV_INTERNAL__PYTHONHOME -u PYTHONPATH "$PYTHON_BIN" lingtu.py --list >/tmp/lingtu_semantic_nav_list.out
pass "lingtu.py --list"

echo ""
echo "============================================================"
echo -e "${GREEN} Semantic navigation smoke checks PASSED${NC}"
echo "============================================================"
