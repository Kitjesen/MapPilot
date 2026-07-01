#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT_DIR}"

run_shell() {
  local name="$1"
  local script="$2"
  if [[ -f "${script}" ]]; then
    echo "== ${name} =="
    bash "${script}"
  else
    echo "skip ${name}: ${script} not found"
  fi
}

run_python() {
  local name="$1"
  local script="$2"
  if [[ -f "${script}" ]]; then
    echo "== ${name} =="
    python "${script}"
  else
    echo "skip ${name}: ${script} not found"
  fi
}

run_shell "full stack smoke" "tests/integration/test_full_stack.sh"
run_python "gateway endpoints" "tests/integration/test_grpc_endpoints.py"
run_python "topic rate audit" "tests/integration/test_topic_hz.py"
run_python "network failure audit" "tests/integration/test_network_failure.py"
