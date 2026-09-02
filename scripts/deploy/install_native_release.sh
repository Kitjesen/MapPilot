#!/usr/bin/env bash
# Install one verified native release and switch back to the current Product.

set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASES_DIR="${LINGTU_RELEASES_DIR:-/opt/lingtu/releases}"
CURRENT_LINK="${LINGTU_CURRENT_LINK:-/opt/lingtu/current}"
STATE_DIR="${LINGTU_SESSION_ROOT:-/run/lingtu}"
TEST_ROOT="${LINGTU_NATIVE_RELEASE_TEST_ROOT:-}"
TEST_CONTROL="${LINGTU_NATIVE_RELEASE_TEST_CONTROL:-}"
TEST_MODE=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package-dir)
      PACKAGE_DIR="$(cd "$2" && pwd)"
      shift 2
      ;;
    --releases-dir)
      RELEASES_DIR="$2"
      shift 2
      ;;
    --current-link)
      CURRENT_LINK="$2"
      shift 2
      ;;
    --state-dir)
      STATE_DIR="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      cat <<'USAGE'
Usage: install_native_release.sh [--dry-run] [--package-dir DIR]
                                 [--releases-dir DIR] [--current-link PATH]
                                 [--state-dir DIR]

The installer verifies native artifact checksums, stages a versioned release,
updates the current link, then switches back to the same Product and map.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown installer argument: $1" >&2
      exit 2
      ;;
  esac
done

METADATA="${PACKAGE_DIR}/metadata.json"
CHECKSUMS="${PACKAGE_DIR}/config/native-release-sha256.txt"
if [[ ! -s "${METADATA}" || ! -s "${CHECKSUMS}" ]]; then
  echo "Package metadata or native checksum manifest is missing" >&2
  exit 1
fi

VERSION="$(python3 - "${METADATA}" <<'PY'
import json
from pathlib import Path
import sys

value = str(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))["version"])
print(value)
PY
)"
if [[ ! "${VERSION}" =~ ^v[0-9]+[.][0-9]+[.][0-9]+([+-][0-9A-Za-z.-]+)?$ ]]; then
  echo "Package metadata contains an invalid version: ${VERSION}" >&2
  exit 1
fi

(
  cd "${PACKAGE_DIR}"
  sha256sum -c config/native-release-sha256.txt
)

for absolute_path in "${RELEASES_DIR}" "${CURRENT_LINK}" "${STATE_DIR}"; do
  if [[ "${absolute_path}" != /* ]]; then
    echo "Installer paths must be absolute: ${absolute_path}" >&2
    exit 1
  fi
done

TARGET_DIR="${RELEASES_DIR}/${VERSION}"
CURRENT_RECORD="${STATE_DIR}/current.json"
RUN_PLAN_PATH=""
if [[ -s "${CURRENT_RECORD}" ]]; then
  RUN_PLAN_PATH="$(python3 - "${CURRENT_RECORD}" <<'PY'
import json
from pathlib import Path
import sys

record = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
run_plan = Path(str(record.get("run_plan_path") or ""))
if not run_plan.is_file():
    raise SystemExit("current RunPlan is missing")
print(run_plan)
PY
)"
fi

ACTIVE_REQUIRES_MAPD=0
ACTIVE_PRODUCT=""
ACTIVE_ENV=""
ACTIVE_ROBOT=""
ACTIVE_MAP=""
if [[ -n "${RUN_PLAN_PATH}" ]]; then
  mapfile -d '' -t ACTIVE_PRODUCT_FACTS < <(
    PYTHONPATH="${PACKAGE_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}" \
      python3 - "${RUN_PLAN_PATH}" "${CURRENT_RECORD}" <<'PY'
import json
import sys

from lingtu.run_plan import RunPlan

plan = RunPlan.load(sys.argv[1])
current = json.loads(open(sys.argv[2], encoding="utf-8").read())
if current.get("product") != plan.product or current.get("env") != plan.env:
    raise SystemExit("current Product identity does not match its RunPlan")
robot = plan.robot
map_name = current.get("map_name")
if map_name is None:
    map_name = ""
elif not isinstance(map_name, str):
    raise SystemExit("current Product map name is invalid")
requires_mapd = any(
    process.name.lower() in {"maps", "mapd"} or "mapd" in process.target.lower()
    for process in plan.processes
)
values = (plan.product, plan.env, robot, map_name, "1" if requires_mapd else "0")
sys.stdout.write("\0".join(values) + "\0")
PY
  )
  if [[ "${#ACTIVE_PRODUCT_FACTS[@]}" -ne 5 ]]; then
    echo "Current Product identity could not be loaded" >&2
    exit 1
  fi
  ACTIVE_PRODUCT="${ACTIVE_PRODUCT_FACTS[0]}"
  ACTIVE_ENV="${ACTIVE_PRODUCT_FACTS[1]}"
  ACTIVE_ROBOT="${ACTIVE_PRODUCT_FACTS[2]}"
  ACTIVE_MAP="${ACTIVE_PRODUCT_FACTS[3]}"
  ACTIVE_REQUIRES_MAPD="${ACTIVE_PRODUCT_FACTS[4]}"
fi

if [[ "${ACTIVE_REQUIRES_MAPD}" == "1" ]] \
    && { [[ ! -x "${PACKAGE_DIR}/bin/mapd" ]] \
      || [[ ! -x "${PACKAGE_DIR}/bin/lingtu-mapctl" ]]; }; then
  echo \
    "Active Product requires maps/mapd, but this release lacks mapd artifacts" \
    >&2
  exit 1
fi

if [[ -e "${CURRENT_LINK}" && ! -L "${CURRENT_LINK}" ]]; then
  echo "Current release path exists but is not a symlink: ${CURRENT_LINK}" >&2
  exit 1
fi
if [[ -L "${CURRENT_LINK}" && -z "${RUN_PLAN_PATH}" ]]; then
  echo "Refusing to replace an active release without its current RunPlan" >&2
  exit 1
fi
if [[ -e "${TARGET_DIR}" || -L "${TARGET_DIR}" ]]; then
  echo "Release target already exists: ${TARGET_DIR}" >&2
  exit 1
fi

if [[ -n "${TEST_ROOT}" || -n "${TEST_CONTROL}" ]]; then
  if [[ -z "${TEST_ROOT}" || -z "${TEST_CONTROL}" || ! -x "${TEST_CONTROL}" ]]; then
    echo "Installer test mode requires a root and executable fake control" >&2
    exit 1
  fi
  python3 - \
    "${TEST_ROOT}" \
    "${PACKAGE_DIR}" \
    "${RELEASES_DIR}" \
    "${CURRENT_LINK}" \
    "${STATE_DIR}" \
    "${TEST_CONTROL}" \
    "${RUN_PLAN_PATH}" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()
for raw in sys.argv[2:]:
    if not raw:
        continue
    path = Path(raw).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise SystemExit(f"installer test path escapes test root: {path}") from exc
PY
  TEST_MODE=1
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  echo "Native release verified: ${VERSION}"
  echo "Target: ${TARGET_DIR}"
  echo "Current RunPlan: ${RUN_PLAN_PATH:-none}"
  exit 0
fi
if [[ "${EUID}" -ne 0 && "${TEST_MODE}" != "1" ]]; then
  echo "Native release installation requires root privileges" >&2
  exit 1
fi

if [[ "${TEST_MODE}" == "1" ]]; then
  mkdir -p "${RELEASES_DIR}" "$(dirname "${CURRENT_LINK}")"
else
  install -d -o root -g root -m 0755 "${RELEASES_DIR}"
  install -d -o root -g root -m 0755 "$(dirname "${CURRENT_LINK}")"
fi
STAGE_DIR="$(mktemp -d "${RELEASES_DIR}/.${VERSION}.staging.XXXXXX")"
NEW_LINK="${CURRENT_LINK}.new.$$"
cleanup() {
  rm -f -- "${NEW_LINK}"
  if [[ -n "${STAGE_DIR}" && -d "${STAGE_DIR}" \
      && "$(dirname "${STAGE_DIR}")" == "${RELEASES_DIR}" \
      && "$(basename "${STAGE_DIR}")" == ".${VERSION}.staging."* ]]; then
    rm -rf -- "${STAGE_DIR}"
  fi
}
trap cleanup EXIT

cp -a "${PACKAGE_DIR}/." "${STAGE_DIR}/"
(
  cd "${STAGE_DIR}"
  sha256sum -c config/native-release-sha256.txt
)

OLD_TARGET=""
if [[ -L "${CURRENT_LINK}" ]]; then
  OLD_TARGET="$(readlink -f "${CURRENT_LINK}")"
fi

product_control() {
  local repo="$1"
  local -a control_command
  if [[ "${TEST_MODE}" == "1" ]]; then
    "${TEST_CONTROL}" \
      "${repo}" switch "${CURRENT_LINK}" \
      "${ACTIVE_ROBOT}" "${ACTIVE_PRODUCT}" "${ACTIVE_MAP}"
    return
  fi
  (
    cd "${repo}"
    control_command=(
      python3 -m lingtu.control
      switch "${ACTIVE_PRODUCT}"
      --robot "${ACTIVE_ROBOT}"
      --env "${ACTIVE_ENV}"
      --state-dir "${STATE_DIR}"
      --json
    )
    if [[ -n "${ACTIVE_MAP}" ]]; then
      control_command+=(--map "${ACTIVE_MAP}")
    fi
    PYTHONPATH="${repo}/src${PYTHONPATH:+:${PYTHONPATH}}" \
      "${control_command[@]}"
  )
}

restore_previous_release() {
  local rollback_failed=0
  local current_target=""
  local target_real=""

  if [[ -L "${CURRENT_LINK}" ]]; then
    current_target="$(readlink -f "${CURRENT_LINK}" || true)"
  fi
  if [[ -e "${TARGET_DIR}" ]]; then
    target_real="$(readlink -f "${TARGET_DIR}" || true)"
  fi
  if [[ -n "${OLD_TARGET}" ]]; then
    if [[ "${current_target}" != "${OLD_TARGET}" ]]; then
      if ! rm -f -- "${NEW_LINK}"; then
        echo "Rollback could not clear temporary release link" >&2
        rollback_failed=1
      elif [[ -e "${NEW_LINK}" || -L "${NEW_LINK}" ]]; then
        echo "Rollback temporary release link still exists after removal" >&2
        rollback_failed=1
      elif ! ln -s -- "${OLD_TARGET}" "${NEW_LINK}"; then
        echo "Rollback could not create the previous release link" >&2
        rollback_failed=1
      elif [[ "$(readlink -f "${NEW_LINK}" || true)" != "${OLD_TARGET}" ]]; then
        echo "Rollback temporary link does not target the previous release" >&2
        rollback_failed=1
      elif ! mv -Tf -- "${NEW_LINK}" "${CURRENT_LINK}"; then
        echo "Rollback could not restore the previous current link" >&2
        rollback_failed=1
      fi
    fi
    current_target=""
    if [[ -L "${CURRENT_LINK}" ]]; then
      current_target="$(readlink -f "${CURRENT_LINK}" || true)"
    fi
    if [[ "${current_target}" == "${OLD_TARGET}" ]]; then
      if [[ -n "${RUN_PLAN_PATH}" ]]; then
        product_control "${OLD_TARGET}" || rollback_failed=1
      fi
    else
      echo \
        "Rollback did not restore current to the previous release; old Product was not started" \
        >&2
      rollback_failed=1
    fi
  elif [[ -L "${CURRENT_LINK}" ]]; then
    if ! rm -f -- "${CURRENT_LINK}"; then
      echo "Rollback could not remove the failed initial release link" >&2
      rollback_failed=1
    elif [[ -e "${CURRENT_LINK}" || -L "${CURRENT_LINK}" ]]; then
      echo "Failed initial release link still exists after rollback" >&2
      rollback_failed=1
    fi
  elif [[ -e "${CURRENT_LINK}" ]]; then
    echo "Rollback found an unexpected non-link current release path" >&2
    rollback_failed=1
  fi
  return "${rollback_failed}"
}

if ! mv -- "${STAGE_DIR}" "${TARGET_DIR}"; then
  if ! restore_previous_release; then
    echo "Rollback failed; operator intervention is required" >&2
  fi
  exit 1
fi
STAGE_DIR=""

if ! ln -s -- "${TARGET_DIR}" "${NEW_LINK}"; then
  if ! restore_previous_release; then
    echo "Rollback failed; operator intervention is required" >&2
  fi
  exit 1
fi
if [[ "$(readlink -f "${NEW_LINK}" || true)" != \
    "$(readlink -f "${TARGET_DIR}" || true)" ]]; then
  echo "New release link does not target the staged release" >&2
  if ! restore_previous_release; then
    echo "Rollback failed; operator intervention is required" >&2
  fi
  exit 1
fi
if ! mv -Tf -- "${NEW_LINK}" "${CURRENT_LINK}"; then
  if ! restore_previous_release; then
    echo "Rollback failed; operator intervention is required" >&2
  fi
  exit 1
fi
if [[ "$(readlink -f "${CURRENT_LINK}" || true)" != \
    "$(readlink -f "${TARGET_DIR}" || true)" ]]; then
  echo "Current release link verification failed after activation" >&2
  if ! restore_previous_release; then
    echo "Rollback failed; operator intervention is required" >&2
  fi
  exit 1
fi

if [[ -n "${RUN_PLAN_PATH}" ]]; then
  if ! product_control "${TARGET_DIR}"; then
    echo "New release failed to switch the active Product; restoring the previous release" >&2
    if ! restore_previous_release; then
      echo "Rollback failed; operator intervention is required" >&2
    fi
    exit 1
  fi
fi

echo "Installed LingTu native release ${VERSION} at ${TARGET_DIR}"
