#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

SESSION_ID="staged-session"
RUN_PLAN="${TEST_ROOT}/plan-${SESSION_ID}.json"
cat >"${RUN_PLAN}" <<'JSON'
{
  "identity": {"product": "teleop_avoid", "env": "real"},
  "launch": {
    "process_catalog": {
      "selected": [{"name": "host"}]
    }
  }
}
JSON

export LINGTU_RUNTIME_ENV_FILE="${TEST_ROOT}/missing-runtime-env.sh"
export LINGTU_PYTHON="$(command -v python3)"
export LINGTU_PRODUCT="teleop_avoid"
export LINGTU_ENV="real"
export LINGTU_PRODUCT_SESSION_ID="${SESSION_ID}"
export LINGTU_RUN_PLAN="${RUN_PLAN}"
export LINGTU_SESSION_ROOT="${TEST_ROOT}"

source "${ROOT}/scripts/deploy/thunder/require_product_session.sh" host
test ! -e "${TEST_ROOT}/current.json"

echo "product session guard tests passed"
