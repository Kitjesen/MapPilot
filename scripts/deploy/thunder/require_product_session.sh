#!/usr/bin/env bash

runtime_env="${LINGTU_RUNTIME_ENV_FILE:-/opt/lingtu/config/thunder-runtime-env.sh}"
if [ -r "${runtime_env}" ]; then
  source "${runtime_env}"
fi
: "${LINGTU_PYTHON:=python3}"

fail_product_session() {
  echo "ERROR: $*" >&2
  return 2 2>/dev/null || exit 2
}

: "${LINGTU_PRODUCT:?ProductControl must set LINGTU_PRODUCT}"
: "${LINGTU_ENV:?ProductControl must set LINGTU_ENV}"
: "${LINGTU_PRODUCT_SESSION_ID:?ProductControl must set LINGTU_PRODUCT_SESSION_ID}"
: "${LINGTU_RUN_PLAN:?ProductControl must set LINGTU_RUN_PLAN}"

case "${LINGTU_PRODUCT}" in
  teleop|teleop_avoid|map|explore|nav|tracking|inspection) ;;
  *) fail_product_session "invalid Product session: ${LINGTU_PRODUCT}" ;;
esac
case "${LINGTU_ENV}" in
  real|sim) ;;
  *) fail_product_session "invalid Product env: ${LINGTU_ENV}" ;;
esac
if [[ ! "${LINGTU_PRODUCT_SESSION_ID}" =~ ^[A-Za-z0-9_.-]{1,96}$ ]]; then
  fail_product_session "invalid Product session id"
fi
session_root="${LINGTU_SESSION_ROOT:-/run/lingtu}"
expected_plan="${session_root}/plan-${LINGTU_PRODUCT_SESSION_ID}.json"
if [[ "${LINGTU_RUN_PLAN}" != "${expected_plan}" || ! -s "${LINGTU_RUN_PLAN}" ]]; then
  fail_product_session "Product session does not reference its readable RunPlan"
fi

expected_role="${1:-${LINGTU_EXPECTED_ROLE:-}}"
if [ -n "${expected_role}" ]; then
  if [[ ! "${expected_role}" =~ ^[A-Za-z0-9_.-]{1,64}$ ]]; then
    fail_product_session "invalid expected RunPlan role: ${expected_role}"
  fi
fi
"${LINGTU_PYTHON}" - "${expected_role}" <<'PY' || fail_product_session "Product session does not match its staged RunPlan"
import json
import os
import sys

expected_role = sys.argv[1]
with open(os.environ["LINGTU_RUN_PLAN"], encoding="utf-8") as handle:
    plan = json.load(handle)

identity = plan.get("identity") or {}
if identity.get("product") != os.environ["LINGTU_PRODUCT"]:
    raise SystemExit(2)
if identity.get("env") != os.environ["LINGTU_ENV"]:
    raise SystemExit(2)

selected = (
    (plan.get("launch") or {})
    .get("process_catalog", {})
    .get("selected", [])
)
if not isinstance(selected, list):
    raise SystemExit(2)
if expected_role and not any(
    isinstance(process, dict) and process.get("name") == expected_role
    for process in selected
):
    raise SystemExit(2)
PY
