#!/usr/bin/env bash

fail_product_session() {
  echo "ERROR: $*" >&2
  return 2 2>/dev/null || exit 2
}

: "${LINGTU_PRODUCT:?ProductControl must set LINGTU_PRODUCT}"
: "${LINGTU_ENV:?ProductControl must set LINGTU_ENV}"
: "${LINGTU_PRODUCT_SESSION_ID:?ProductControl must set LINGTU_PRODUCT_SESSION_ID}"
: "${LINGTU_RUN_PLAN:?ProductControl must set LINGTU_RUN_PLAN}"
: "${LINGTU_RUN_PLAN_FINGERPRINT:?ProductControl must set LINGTU_RUN_PLAN_FINGERPRINT}"

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
if [[ ! "${LINGTU_RUN_PLAN_FINGERPRINT}" =~ ^[0-9a-f]{64}$ ]]; then
  fail_product_session "invalid RunPlan fingerprint"
fi
expected_plan="/run/lingtu/plan-${LINGTU_RUN_PLAN_FINGERPRINT}.json"
if [[ "${LINGTU_RUN_PLAN}" != "${expected_plan}" || ! -s "${LINGTU_RUN_PLAN}" ]]; then
  fail_product_session "Product session does not reference its readable RunPlan"
fi

expected_role="${1:-${LINGTU_EXPECTED_ROLE:-}}"
if [ -n "${expected_role}" ]; then
  if [[ ! "${expected_role}" =~ ^[A-Za-z0-9_.-]{1,64}$ ]]; then
    fail_product_session "invalid expected RunPlan role: ${expected_role}"
  fi
  python3 - "${expected_role}" <<'PY' || fail_product_session "RunPlan does not declare the expected Product role: ${expected_role}"
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
if identity.get("fingerprint") != os.environ["LINGTU_RUN_PLAN_FINGERPRINT"]:
    raise SystemExit(2)

selected = (
    (plan.get("launch") or {})
    .get("process_catalog", {})
    .get("selected", [])
)
if not any(process.get("name") == expected_role for process in selected):
    raise SystemExit(2)
PY
fi
