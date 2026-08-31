#!/usr/bin/env bash
# Assemble a downloadable LingTu native release from already-built artifacts.
# This script only reads the checkout and writes to a temporary staging tree
# and the selected output directory. It never installs or activates services.

set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROOT="${LINGTU_NATIVE_RELEASE_SOURCE_ROOT:-${SCRIPT_ROOT}}"
INSTALLER_SOURCE="${SCRIPT_ROOT}/scripts/deploy/install_native_release.sh"

NATIVE_EXECUTABLES=(
  build/livox_sdk2_stream/livox_sdk2_stream
  build/slam_core/slamd
  build/slam_core/slamctl
  build/dds_probe/lingtu_dds_probe
  build/driver/lingtu_driver
  build/camera_dds/lingtu_camera_dds
  build/orbbec_native/orbbec_capture
)

NAV_EXECUTABLES=(
  navd
  lingtu_traversability_dds
  lingtu_explore_dds
  lingtu_nav_control
)
NAV_LIBRARIES=(
  liblingtu_nav_client.so
  liblingtu_inspection_evidence_bridge.so
  inspection/liblingtu_inspection.so
)
CMU_PATH_PROFILES=(go2 thunder)
CMU_PATH_FILES=(
  startPaths.ply
  pathList.ply
  paths.ply
  correspondences.txt
  search_radius.txt
)
RECORDING_EXECUTABLES=(
  lingtu_recorder
  lingtu_dds_recorder
  lingtu_dds_player
  lingtu_camera_recorder
  lingtu_camera_player
)

# mapd is being integrated independently. A checkout that contains its build
# entrypoint declares the component available, so the release must then carry
# the complete maps runtime bundle. Clean checkouts without that entrypoint
# remain valid.
MAPD_EXECUTABLES=(
  build/maps/mapd
  build/maps/lingtu-mapctl
  build/prune/prune
  build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap
)
MAP_OPT_EXECUTABLES=(
  build/map_opt/lt_pgo
)

write_run_fixture() {
  local kind="$1"
  local current_path="$2"

  python3 - "${kind}" "${current_path}" <<'PY'
import json
from pathlib import Path
import sys

from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from runtime.graph import ProcessSpec

kind, current_value = sys.argv[1:]
if kind == "map":
    product = "map"
    contracts = ("lingtu.product.map.v1",)
    processes = (ProcessSpec("maps", "systemd", "lt-maps.service", 10, 5, "mode"),)
elif kind == "transaction":
    product = "nav"
    contracts = ("lingtu.product.nav.v1",)
    processes = (
        ProcessSpec("driver", "systemd", "lt-driver.service", 10, 5, "mode"),
        ProcessSpec("nav", "systemd", "navd.service", 20, 5, "mode"),
    )
else:
    raise SystemExit(f"unknown RunPlan fixture: {kind}")

plan = RunPlan.create(
    product=product,
    env="real",
    robot="unitree/go2",
    process_control="systemd",
    modules=(),
    processes=processes,
    available_processes=processes,
    stop_before_start=tuple(process.target for process in processes),
    contracts=contracts,
    critical_modules=(),
    route_contract=None,
    host_config={},
    lifecycle={},
    native_nav={
        "control_mode": "test",
        "global_planner": "octoplanner3d",
        "publish_cmd_vel": False,
        "check_obstacle": False,
        "use_traversability_cost": False,
        "allow_teleop_takeover": False,
        "teleop_local_planner": False,
    },
)
product_session_id = "product-package-test"
plan_path = plan.write(Path(current_value).parent / f"plan-{product_session_id}.json")
Path(current_value).write_text(
    json.dumps(
        {
            "schema_version": CURRENT_RUN_SCHEMA,
            "product": plan.product,
            "product_variant": plan.product_variant,
            "env": plan.env,
            "run_plan_path": str(plan_path),
            "committed_at": 0.0,
            "product_session_id": product_session_id,
            "map_name": "plant-a" if kind == "transaction" else None,
        }
    )
    + "\n",
    encoding="utf-8",
)
PY
}

list_checkout_files() {
  local source_root="$1"
  local relative

  {
    git -C "${source_root}" ls-files --cached -z
    git -C "${source_root}" ls-files \
      --others --exclude-standard -z -- \
      src config scripts tools web \
      pyproject.toml uv.lock VERSION Makefile README.md
  } \
    | LC_ALL=C sort -zu \
    | while IFS= read -r -d '' relative; do
        if [[ -e "${source_root}/${relative}" || -L "${source_root}/${relative}" ]]; then
          printf '%s\0' "${relative}"
        fi
      done
}

run_self_test() {
  local PYTHONPATH="${SCRIPT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
  local test_root
  local output_dir
  local nav_install
  local recording_install
  local relative
  local tarball
  local manifest
  local key_file
  local extracted_package
  local mapd_tarball
  local tar_listing
  local mapd_tar_listing
  local mapd_missing_output
  local mapd_missing_converter_output
  local map_opt_missing_output
  local preflight_output
  local fake_control
  local success_root
  local failure_root
  local link_failure_root
  local fake_ln_dir
  local link_failure_output
  local header_output

  export PYTHONPATH

  test_root="$(mktemp -d "${TMPDIR:-/tmp}/lingtu-native-release-test.XXXXXX")"
  output_dir="${test_root}/output"
  nav_install="${test_root}/nav-install"
  recording_install="${test_root}/recording-install"
  cleanup_self_test() {
    if [[ -d "${test_root}" \
        && "$(basename "${test_root}")" == lingtu-native-release-test.* ]]; then
      rm -rf -- "${test_root}"
    fi
  }
  trap cleanup_self_test RETURN

  mkdir -p \
    "${test_root}/source/config" \
    "${test_root}/source/src/localization/fastlio2/config" \
    "${test_root}/source/src/nav/cpp" \
    "${test_root}/source/src/nav/inspection" \
    "${nav_install}" \
    "${recording_install}"
  printf '0.0.0\n' > "${test_root}/source/VERSION"
  printf 'native release self-test\n' > "${test_root}/source/config/self-test.txt"
  printf 'native: true\n' \
    > "${test_root}/source/src/localization/fastlio2/config/self-test.yaml"
  printf 'internal only\n' > "${test_root}/source/src/nav/cpp/internal.hpp"
  : > "${test_root}/source/src/nav/inspection/__init__.py"
  printf 'runtime = True\n' > "${test_root}/source/src/nav/inspection/service.py"
  printf 'internal only\n' > "${test_root}/source/src/nav/inspection/internal.hpp"
  printf 'removed before packaging\n' > "${test_root}/source/config/deleted.txt"
  git -C "${test_root}/source" init -q
  git -C "${test_root}/source" add VERSION config src
  rm "${test_root}/source/config/deleted.txt"
  mkdir -p "${test_root}/source/src/message"
  printf 'CURRENT_CONTRACT = True\n' \
    > "${test_root}/source/src/message/current_contract.py"

  for relative in "${NATIVE_EXECUTABLES[@]}"; do
    mkdir -p "$(dirname "${test_root}/source/${relative}")"
    install -m 0755 /dev/null "${test_root}/source/${relative}"
  done
  mkdir -p "${test_root}/source/build/orbbec_native/lib/extensions/depthengine"
  install -m 0644 /dev/null \
    "${test_root}/source/build/orbbec_native/lib/libOrbbecSDK.so.2"
  ln -s libOrbbecSDK.so.2 \
    "${test_root}/source/build/orbbec_native/lib/libOrbbecSDK.so"
  install -m 0644 /dev/null \
    "${test_root}/source/build/orbbec_native/lib/extensions/depthengine/libdepthengine.so"
  for relative in "${NAV_EXECUTABLES[@]}"; do
    mkdir -p "$(dirname "${nav_install}/${relative}")"
    install -m 0755 /dev/null "${nav_install}/${relative}"
  done
  for relative in "${NAV_LIBRARIES[@]}"; do
    mkdir -p "$(dirname "${nav_install}/${relative}")"
    install -m 0644 /dev/null "${nav_install}/${relative}"
  done
  for profile in "${CMU_PATH_PROFILES[@]}"; do
    mkdir -p "${nav_install}/cmu_paths/${profile}"
    for relative in "${CMU_PATH_FILES[@]}"; do
      printf 'fixture\n' > "${nav_install}/cmu_paths/${profile}/${relative}"
    done
  done
  for relative in "${RECORDING_EXECUTABLES[@]}"; do
    install -m 0755 /dev/null "${recording_install}/${relative}"
  done

  # A map-optimizer build entrypoint makes lt_pgo a required release artifact.
  # Prove that omission fails before supplying the fixture binary.
  mkdir -p "${test_root}/source/scripts/build"
  install -m 0755 /dev/null "${test_root}/source/scripts/build/build_map_opt.sh"
  if map_opt_missing_output="$(
    LINGTU_NATIVE_RELEASE_SOURCE_ROOT="${test_root}/source" \
      LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE="${nav_install}" \
      LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE="${recording_install}" \
      LINGTU_NATIVE_RELEASE_ARCH=aarch64 \
      SOURCE_DATE_EPOCH=1704067200 \
      bash "${BASH_SOURCE[0]}" v0.0.0 "${test_root}/output-map-opt-missing" 2>&1
  )"; then
    echo "packager accepted a map optimizer without lt_pgo" >&2
    return 1
  fi
  grep -Fq \
    "Required native executable is missing: ${test_root}/source/build/map_opt/lt_pgo" \
    <<<"${map_opt_missing_output}"
  mkdir -p "${test_root}/source/build/map_opt"
  install -m 0755 /dev/null "${test_root}/source/build/map_opt/lt_pgo"

  key_file=""
  if python3 -c 'import cryptography' >/dev/null 2>&1; then
    key_file="${test_root}/test-signing-key.pem"
    python3 - "${key_file}" <<'PY'
from pathlib import Path
import sys

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import Encoding, NoEncryption, PrivateFormat

key = Ed25519PrivateKey.generate()
Path(sys.argv[1]).write_bytes(
    key.private_bytes(Encoding.PEM, PrivateFormat.PKCS8, NoEncryption())
)
PY
  fi

  mkdir -p "${nav_install}/include"
  : > "${nav_install}/include/internal.hpp"
  if header_output="$(
    LINGTU_NATIVE_RELEASE_SOURCE_ROOT="${test_root}/source" \
      LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE="${nav_install}" \
      LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE="${recording_install}" \
      LINGTU_NATIVE_RELEASE_ARCH=aarch64 \
      SOURCE_DATE_EPOCH=1704067200 \
      bash "${BASH_SOURCE[0]}" v0.0.0 "${test_root}/output-header" 2>&1
  )"; then
    echo "packager accepted a navigation install containing a header" >&2
    return 1
  fi
  grep -Fq 'Native navigation runtime install must not contain C/C++ headers' \
    <<<"${header_output}"
  rm -f "${nav_install}/include/internal.hpp"

  LINGTU_NATIVE_RELEASE_SOURCE_ROOT="${test_root}/source" \
    LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE="${nav_install}" \
    LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE="${recording_install}" \
    LINGTU_NATIVE_RELEASE_ARCH=aarch64 \
    LINGTU_NATIVE_RELEASE_SIGNING_KEY="${key_file}" \
    SOURCE_DATE_EPOCH=1704067200 \
    bash "${BASH_SOURCE[0]}" v0.0.0 "${output_dir}"

  tarball="${output_dir}/lingtu-0.0.0-aarch64-native-release.tar.gz"
  manifest="${output_dir}/lingtu-0.0.0-aarch64-native-release-manifest.json"
  test -s "${tarball}"
  test -s "${manifest}"
  (
    cd "${output_dir}"
    sha256sum -c lingtu-0.0.0-aarch64-native-release.sha256
  )
  python3 -m json.tool "${manifest}" >/dev/null
  python3 - "${manifest}" "${key_file}" <<'PY'
import json
from pathlib import Path
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
assert payload["release_format"] == "lingtu.native-release.v1"
assert payload["artifacts"][0]["filename"].endswith("-native-release.tar.gz")
assert payload["artifacts"][0]["apply_action"] == "install_script"
assert payload["artifacts"][0]["install_script"] == "install_nav.sh"
assert bool(payload["signature"]) == bool(sys.argv[2])
PY
  tar_listing="$(tar -tzf "${tarball}")"
  grep -Fq 'lingtu-0.0.0-aarch64-native-release/metadata.json' \
    <<<"${tar_listing}"
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/src/localization/fastlio2/config/self-test.yaml' \
    <<<"${tar_listing}"
  grep -Fq 'lingtu-0.0.0-aarch64-native-release/install_nav.sh' \
    <<<"${tar_listing}"
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/src/nav/inspection/service.py' \
    <<<"${tar_listing}"
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/src/message/current_contract.py' \
    <<<"${tar_listing}"
  if grep -Fq 'lingtu-0.0.0-aarch64-native-release/config/deleted.txt' \
      <<<"${tar_listing}"; then
    echo "native release contains a deleted tracked source file" >&2
    return 1
  fi
  if grep -Eq 'lingtu-0.0.0-aarch64-native-release/src/nav/cpp/' \
      <<<"${tar_listing}"; then
    echo "native release contains the internal navigation C++ tree" >&2
    return 1
  fi
  if grep -Eq 'lingtu-0.0.0-aarch64-native-release/src/nav/inspection/.*\.(c|cc|cpp|cxx|h|hpp)$' \
      <<<"${tar_listing}"; then
    echo "native release contains inspection C/C++ sources" >&2
    return 1
  fi
  for relative in "${RECORDING_EXECUTABLES[@]}"; do
    grep -Fq \
      "lingtu-0.0.0-aarch64-native-release/build/native-recording/${relative}" \
      <<<"${tar_listing}"
  done
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/build/camera_dds/lingtu_camera_dds' \
    <<<"${tar_listing}"
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/build/orbbec_native/orbbec_capture' \
    <<<"${tar_listing}"
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/build/orbbec_native/lib/libOrbbecSDK.so' \
    <<<"${tar_listing}"
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/build/orbbec_native/lib/extensions/depthengine/libdepthengine.so' \
    <<<"${tar_listing}"
  grep -Fq \
    'lingtu-0.0.0-aarch64-native-release/build/map_opt/lt_pgo' \
    <<<"${tar_listing}"
  if grep -Fq 'lingtu-0.0.0-aarch64-native-release/build/maps/mapd' \
      <<<"${tar_listing}"; then
    echo "mapd must not be inferred when its build entrypoint is absent" >&2
    return 1
  fi
  if grep -Fq 'lingtu-0.0.0-aarch64-native-release/build/maps/lingtu-mapctl' \
      <<<"${tar_listing}"; then
    echo "lingtu-mapctl must not be inferred when its build entrypoint is absent" >&2
    return 1
  fi
  mkdir -p "${test_root}/extracted"
  tar -xzf "${tarball}" -C "${test_root}/extracted"
  extracted_package="${test_root}/extracted/lingtu-0.0.0-aarch64-native-release"
  for relative in "${RECORDING_EXECUTABLES[@]}"; do
    test -x "${extracted_package}/build/native-recording/${relative}"
    grep -Fq \
      "build/native-recording/${relative}" \
      "${extracted_package}/config/native-release-sha256.txt"
  done
  test -x "${extracted_package}/build/camera_dds/lingtu_camera_dds"
  test -x "${extracted_package}/build/orbbec_native/orbbec_capture"
  test -f "${extracted_package}/build/orbbec_native/lib/libOrbbecSDK.so"
  test -f \
    "${extracted_package}/build/orbbec_native/lib/extensions/depthengine/libdepthengine.so"
  test -x "${extracted_package}/build/map_opt/lt_pgo"
  bash "${extracted_package}/install_nav.sh" \
    --dry-run \
    --package-dir "${extracted_package}" \
    --releases-dir "${test_root}/releases" \
    --current-link "${test_root}/current" \
    --state-dir "${test_root}/state"

  # An active Product that owns maps must be rejected before its old release
  # is stopped or the current symlink changes when mapd is absent.
  mkdir -p \
    "${test_root}/old-release/src/lingtu" \
    "${test_root}/state"
  : > "${test_root}/old-release/src/lingtu/__init__.py"
  cat > "${test_root}/old-release/src/lingtu/control.py" <<'PY'
import os
from pathlib import Path

Path(os.environ["LINGTU_NATIVE_RELEASE_STOP_SENTINEL"]).write_text("called\n")
PY
  write_run_fixture \
    map \
    "${test_root}/state/current.json"
  ln -s "${test_root}/old-release" "${test_root}/current"
  if preflight_output="$(
    LINGTU_NATIVE_RELEASE_STOP_SENTINEL="${test_root}/stop-called" \
      bash "${extracted_package}/install_nav.sh" \
      --package-dir "${extracted_package}" \
      --releases-dir "${test_root}/releases" \
      --current-link "${test_root}/current" \
      --state-dir "${test_root}/state" 2>&1
  )"; then
    echo "installer accepted a maps Product without mapd artifacts" >&2
    return 1
  fi
  grep -Fq \
    'Active Product requires maps/mapd, but this release lacks mapd artifacts' \
    <<<"${preflight_output}"
  test ! -e "${test_root}/stop-called"
  test "$(readlink -f "${test_root}/current")" = \
    "$(readlink -f "${test_root}/old-release")"
  test ! -e "${test_root}/releases/v0.0.0"

  # A checkout that exposes the mapd build entrypoint must produce a complete
  # maps runtime bundle; missing artifacts are fatal in the normal copy path.
  mkdir -p "${test_root}/source/scripts/build" "${test_root}/source/build/maps"
  install -m 0755 /dev/null "${test_root}/source/scripts/build/build_mapd.sh"
  git -C "${test_root}/source" add scripts/build/build_mapd.sh
  install -m 0755 /dev/null "${test_root}/source/build/maps/mapd"
  install -m 0755 /dev/null "${test_root}/source/build/maps/lingtu-mapctl"
  if mapd_missing_output="$(
    LINGTU_NATIVE_RELEASE_SOURCE_ROOT="${test_root}/source" \
      LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE="${nav_install}" \
      LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE="${recording_install}" \
      LINGTU_NATIVE_RELEASE_ARCH=aarch64 \
      SOURCE_DATE_EPOCH=1704067200 \
      bash "${BASH_SOURCE[0]}" v0.0.1 "${test_root}/output-mapd-missing" 2>&1
  )"; then
    echo "packager accepted a mapd bundle without prune" >&2
    return 1
  fi
  grep -Fq \
    "Required native executable is missing: ${test_root}/source/build/prune/prune" \
    <<<"${mapd_missing_output}"
  mkdir -p "${test_root}/source/build/prune"
  install -m 0755 /dev/null "${test_root}/source/build/prune/prune"
  if mapd_missing_converter_output="$(
    LINGTU_NATIVE_RELEASE_SOURCE_ROOT="${test_root}/source" \
      LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE="${nav_install}" \
      LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE="${recording_install}" \
      LINGTU_NATIVE_RELEASE_ARCH=aarch64 \
      SOURCE_DATE_EPOCH=1704067200 \
      bash "${BASH_SOURCE[0]}" v0.0.1 "${test_root}/output-mapd-missing-converter" 2>&1
  )"; then
    echo "packager accepted a mapd bundle without the OctoMap converter" >&2
    return 1
  fi
  grep -Fq \
    "Required native executable is missing: ${test_root}/source/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap" \
    <<<"${mapd_missing_converter_output}"
  mkdir -p "${test_root}/source/build/octoplanner3d_headless"
  install -m 0755 /dev/null \
    "${test_root}/source/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap"
  LINGTU_NATIVE_RELEASE_SOURCE_ROOT="${test_root}/source" \
    LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE="${nav_install}" \
    LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE="${recording_install}" \
    LINGTU_NATIVE_RELEASE_ARCH=aarch64 \
    SOURCE_DATE_EPOCH=1704067200 \
    bash "${BASH_SOURCE[0]}" v0.0.1 "${test_root}/output-mapd"
  mapd_tarball="${test_root}/output-mapd/lingtu-0.0.1-aarch64-native-release.tar.gz"
  mapd_tar_listing="$(tar -tzf "${mapd_tarball}")"
  grep -Fq 'lingtu-0.0.1-aarch64-native-release/build/maps/mapd' \
    <<<"${mapd_tar_listing}"
  grep -Fq 'lingtu-0.0.1-aarch64-native-release/build/maps/lingtu-mapctl' \
    <<<"${mapd_tar_listing}"
  grep -Fq 'lingtu-0.0.1-aarch64-native-release/build/prune/prune' \
    <<<"${mapd_tar_listing}"
  grep -Fq \
    'lingtu-0.0.1-aarch64-native-release/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap' \
    <<<"${mapd_tar_listing}"

  # Exercise the real installer transaction with a fake ProductControl. The
  # active Product and map must switch from the new tree, or from the restored
  # old tree after a failed activation.
  fake_control="${test_root}/fake-product-control"
  cat > "${fake_control}" <<'FAKE_CONTROL'
#!/usr/bin/env bash
set -euo pipefail

repo="$1"
action="$2"
current_link="$3"
robot="$4"
product="$5"
map_name="$6"
printf '%s|%s|%s|%s|%s|%s\n' \
  "$(basename "${repo}")" \
  "$(basename "$(readlink -f "${current_link}")")" \
  "${action}" "${robot}" "${product}" "${map_name}" \
  >> "${LINGTU_FAKE_CONTROL_LOG}"
if [[ "$(basename "${repo}")" != "old-release" \
    && "${LINGTU_FAKE_CONTROL_FAIL_NEW:-0}" == "1" ]]; then
  exit 1
fi
FAKE_CONTROL
  chmod 0755 "${fake_control}"
  success_root="${test_root}/transaction-success"
  mkdir -p "${success_root}/old-release" "${success_root}/state"
  ln -s "${success_root}/old-release" "${success_root}/current"
  write_run_fixture \
    transaction "${success_root}/state/current.json"
  LINGTU_NATIVE_RELEASE_TEST_ROOT="${test_root}" \
    LINGTU_NATIVE_RELEASE_TEST_CONTROL="${fake_control}" \
    LINGTU_FAKE_CONTROL_LOG="${success_root}/control.log" \
    bash "${extracted_package}/install_nav.sh" \
    --package-dir "${extracted_package}" \
    --releases-dir "${success_root}/releases" \
    --current-link "${success_root}/current" \
    --state-dir "${success_root}/state"
  python3 - "${success_root}/control.log" <<'PY'
from pathlib import Path
import sys

assert Path(sys.argv[1]).read_text(encoding="utf-8").splitlines() == [
    "v0.0.0|v0.0.0|switch|unitree/go2|nav|plant-a",
]
PY
  test "$(readlink -f "${success_root}/current")" = \
    "$(readlink -f "${success_root}/releases/v0.0.0")"

  failure_root="${test_root}/transaction-failure"
  mkdir -p "${failure_root}/old-release" "${failure_root}/state"
  ln -s "${failure_root}/old-release" "${failure_root}/current"
  write_run_fixture \
    transaction "${failure_root}/state/current.json"
  if LINGTU_NATIVE_RELEASE_TEST_ROOT="${test_root}" \
      LINGTU_NATIVE_RELEASE_TEST_CONTROL="${fake_control}" \
      LINGTU_FAKE_CONTROL_LOG="${failure_root}/control.log" \
      LINGTU_FAKE_CONTROL_FAIL_NEW=1 \
      bash "${extracted_package}/install_nav.sh" \
      --package-dir "${extracted_package}" \
      --releases-dir "${failure_root}/releases" \
      --current-link "${failure_root}/current" \
      --state-dir "${failure_root}/state"; then
    echo "installer accepted a failed Product switch" >&2
    return 1
  fi
  python3 - "${failure_root}/control.log" <<'PY'
from pathlib import Path
import sys

assert Path(sys.argv[1]).read_text(encoding="utf-8").splitlines() == [
    "v0.0.0|v0.0.0|switch|unitree/go2|nav|plant-a",
    "old-release|old-release|switch|unitree/go2|nav|plant-a",
]
PY
  test "$(readlink -f "${failure_root}/current")" = \
    "$(readlink -f "${failure_root}/old-release")"

  link_failure_root="${test_root}/transaction-link-failure"
  fake_ln_dir="${test_root}/fake-bin"
  mkdir -p \
    "${link_failure_root}/old-release" \
    "${link_failure_root}/state" \
    "${fake_ln_dir}"
  ln -s "${link_failure_root}/old-release" "${link_failure_root}/current"
  write_run_fixture \
    transaction \
    "${link_failure_root}/state/current.json"
  cat > "${fake_ln_dir}/ln" <<'FAKE_LN'
#!/usr/bin/env bash
set -euo pipefail

count=0
if [[ -s "${LINGTU_FAKE_LN_COUNT}" ]]; then
  count="$(<"${LINGTU_FAKE_LN_COUNT}")"
fi
count=$((count + 1))
printf '%s\n' "${count}" > "${LINGTU_FAKE_LN_COUNT}"
if [[ "${count}" -ge 2 ]]; then
  exit 1
fi
exec "${LINGTU_REAL_LN}" "$@"
FAKE_LN
  chmod 0755 "${fake_ln_dir}/ln"
  if link_failure_output="$(
    PATH="${fake_ln_dir}:${PATH}" \
      LINGTU_REAL_LN="$(command -v ln)" \
      LINGTU_FAKE_LN_COUNT="${link_failure_root}/ln-count" \
      LINGTU_NATIVE_RELEASE_TEST_ROOT="${test_root}" \
      LINGTU_NATIVE_RELEASE_TEST_CONTROL="${fake_control}" \
      LINGTU_FAKE_CONTROL_LOG="${link_failure_root}/control.log" \
      LINGTU_FAKE_CONTROL_FAIL_NEW=1 \
      bash "${extracted_package}/install_nav.sh" \
      --package-dir "${extracted_package}" \
      --releases-dir "${link_failure_root}/releases" \
      --current-link "${link_failure_root}/current" \
      --state-dir "${link_failure_root}/state" 2>&1
  )"; then
    echo "installer accepted a failed rollback link replacement" >&2
    return 1
  fi
  grep -Fq 'Rollback failed; operator intervention is required' \
    <<<"${link_failure_output}"
  python3 - "${link_failure_root}/control.log" <<'PY'
from pathlib import Path
import sys

assert Path(sys.argv[1]).read_text(encoding="utf-8").splitlines() == [
    "v0.0.0|v0.0.0|switch|unitree/go2|nav|plant-a",
]
PY
  test "$(readlink -f "${link_failure_root}/current")" = \
    "$(readlink -f "${link_failure_root}/releases/v0.0.0")"
  echo "package_native_release self-test passed"
}

if [[ "${1:-}" == "--self-test" ]]; then
  run_self_test
  exit 0
fi

VERSION="${1:-}"
OUTPUT_DIR="${2:-${ROOT}/dist}"

if [[ ! "${VERSION}" =~ ^v[0-9]+[.][0-9]+[.][0-9]+([+-][0-9A-Za-z.-]+)?$ ]]; then
  echo "Usage: $0 <vMAJOR.MINOR.PATCH> [output-directory]" >&2
  exit 2
fi

ARCH_RAW="${LINGTU_NATIVE_RELEASE_ARCH:-$(dpkg --print-architecture 2>/dev/null || uname -m)}"
case "${ARCH_RAW}" in
  arm64|aarch64)
    ARCH="aarch64"
    ;;
  amd64|x86_64)
    ARCH="x86_64"
    ;;
  *)
    echo "Unsupported native release architecture: ${ARCH_RAW}" >&2
    exit 2
    ;;
esac

for command_name in cmake git install python3 rsync sha256sum tar gzip; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Required packaging command is missing: ${command_name}" >&2
    exit 2
  fi
done

VERSION_CLEAN="${VERSION#v}"
PACKAGE_NAME="lingtu-${VERSION_CLEAN}-${ARCH}-native-release"
mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"

STAGING_DIR="$(mktemp -d "${TMPDIR:-/tmp}/lingtu-native-release.XXXXXX")"
cleanup() {
  if [[ -d "${STAGING_DIR}" \
      && "$(basename "${STAGING_DIR}")" == lingtu-native-release.* ]]; then
    rm -rf -- "${STAGING_DIR}"
  fi
}
trap cleanup EXIT

PACKAGE_ROOT="${STAGING_DIR}/${PACKAGE_NAME}"
mkdir -p "${PACKAGE_ROOT}"

# Mirror the deployable Host tree while omitting development-only and explicit
# compatibility surfaces. Native Fast-LIO2 only needs its runtime config here;
# the algorithm implementation is already linked into the packaged binary.
list_checkout_files "${ROOT}" \
  | rsync -a \
  --from0 \
  --files-from=- \
  --prune-empty-dirs \
  --exclude='/.git/' \
  --exclude='/.github/' \
  --exclude='/build/' \
  --exclude='/dist/' \
  --exclude='/log/' \
  --exclude='/logs/' \
  --exclude='/.lingtu/' \
  --exclude='/.pytest_cache/' \
  --exclude='/.mypy_cache/' \
  --exclude='/.ruff_cache/' \
  --exclude='/**/__pycache__/' \
  --exclude='*.pyc' \
  --exclude='/tools/calibration/' \
  --exclude='/docs/' \
  --exclude='/integrations/' \
  --exclude='/launch/' \
  --exclude='/research/' \
  --exclude='/sim/' \
  --exclude='/tests/' \
  --exclude='/third_party/' \
  --exclude='/docker/' \
  --exclude='/docker-compose.yml' \
  --exclude='/docker-compose.dev.yml' \
  --include='/src/' \
  --include='/src/localization/' \
  --include='/src/localization/fastlio2/' \
  --include='/src/localization/fastlio2/config/***' \
  --exclude='/src/localization/fastlio2/***' \
  --exclude='/src/localization/localizer/***' \
  --exclude='/src/nav/cpp/***' \
  --exclude='/src/nav/inspection/*.c' \
  --exclude='/src/nav/inspection/*.cc' \
  --exclude='/src/nav/inspection/*.cpp' \
  --exclude='/src/nav/inspection/*.cxx' \
  --exclude='/src/nav/inspection/*.h' \
  --exclude='/src/nav/inspection/*.hpp' \
  --exclude='/src/nav/inspection/CMakeLists.txt' \
  --exclude='/src/nav/inspection/README.md' \
  --exclude='/scripts/deploy/cut_release.sh' \
  --exclude='/scripts/deploy/s100p/***' \
  "${ROOT}/" "${PACKAGE_ROOT}/"

if [[ -e "${PACKAGE_ROOT}/src/nav/cpp" ]]; then
  echo "Native release must not contain internal source: src/nav/cpp" >&2
  exit 1
fi
if find "${PACKAGE_ROOT}/src/nav/inspection" -type f \
    \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' \
       -o -name '*.h' -o -name '*.hpp' -o -name CMakeLists.txt \) \
    -print -quit | grep -q .; then
  echo "Native release must not contain inspection C/C++ sources" >&2
  exit 1
fi

copy_executable() {
  local relative="$1"
  local source="${ROOT}/${relative}"
  local destination="${PACKAGE_ROOT}/${relative}"
  if [[ ! -x "${source}" ]]; then
    echo "Required native executable is missing: ${source}" >&2
    exit 1
  fi
  mkdir -p "$(dirname "${destination}")"
  cp -L "${source}" "${destination}"
  chmod 0755 "${destination}"
}

copy_library() {
  local relative="$1"
  local source="${ROOT}/${relative}"
  local destination="${PACKAGE_ROOT}/${relative}"
  if [[ ! -f "${source}" ]]; then
    echo "Required native library is missing: ${source}" >&2
    exit 1
  fi
  mkdir -p "$(dirname "${destination}")"
  cp -L "${source}" "${destination}"
  chmod 0644 "${destination}"
}

for relative in "${NATIVE_EXECUTABLES[@]}"; do
  copy_executable "${relative}"
done

ORBBEC_RUNTIME_SOURCE="${ROOT}/build/orbbec_native/lib"
ORBBEC_RUNTIME_DIR="${PACKAGE_ROOT}/build/orbbec_native/lib"
orbbec_sdk_libraries=("${ORBBEC_RUNTIME_SOURCE}"/libOrbbecSDK.so*)
if [[ ! -e "${orbbec_sdk_libraries[0]}" ]]; then
  echo "Orbbec runtime is missing libOrbbecSDK.so: ${ORBBEC_RUNTIME_SOURCE}" >&2
  exit 1
fi
if [[ ! -d "${ORBBEC_RUNTIME_SOURCE}/extensions/depthengine" ]] \
    || [[ -z "$(find "${ORBBEC_RUNTIME_SOURCE}/extensions/depthengine" -type f -print -quit)" ]]; then
  echo "Orbbec runtime is missing depthengine: ${ORBBEC_RUNTIME_SOURCE}/extensions/depthengine" >&2
  exit 1
fi
mkdir -p "${ORBBEC_RUNTIME_DIR}"
rsync -aL "${ORBBEC_RUNTIME_SOURCE}/" "${ORBBEC_RUNTIME_DIR}/"

if git -C "${ROOT}" ls-files --error-unmatch -- \
    scripts/build/build_mapd.sh >/dev/null 2>&1; then
  echo "mapd build entrypoint detected; requiring mapd release artifacts"
  for relative in "${MAPD_EXECUTABLES[@]}"; do
    copy_executable "${relative}"
  done
fi

if [[ -f "${ROOT}/scripts/build/build_map_opt.sh" ]]; then
  echo "map optimizer build entrypoint detected; requiring lt_pgo release artifact"
  for relative in "${MAP_OPT_EXECUTABLES[@]}"; do
    copy_executable "${relative}"
  done
fi

NAV_INSTALL_DIR="${PACKAGE_ROOT}/build/nav_endpoint"
if [[ -n "${LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE:-}" ]]; then
  rsync -aL "${LINGTU_NATIVE_RELEASE_NAV_INSTALL_SOURCE}/" "${NAV_INSTALL_DIR}/"
else
  cmake --install "${ROOT}/build/nav_endpoint" --prefix "${NAV_INSTALL_DIR}"
fi

if find "${NAV_INSTALL_DIR}" -type f \( -name '*.h' -o -name '*.hpp' \) \
    -print -quit | grep -q .; then
  echo "Native navigation runtime install must not contain C/C++ headers" >&2
  exit 1
fi

for relative in "${NAV_EXECUTABLES[@]}"; do
  if [[ ! -x "${NAV_INSTALL_DIR}/${relative}" ]]; then
    echo "Native navigation install is missing executable: ${relative}" >&2
    exit 1
  fi
done
for relative in "${NAV_LIBRARIES[@]}"; do
  if [[ ! -f "${NAV_INSTALL_DIR}/${relative}" ]]; then
    echo "Native navigation install is missing library: ${relative}" >&2
    exit 1
  fi
done
for profile in "${CMU_PATH_PROFILES[@]}"; do
  for relative in "${CMU_PATH_FILES[@]}"; do
    if [[ ! -s "${NAV_INSTALL_DIR}/cmu_paths/${profile}/${relative}" ]]; then
      echo "Native navigation install is missing CMU ${profile} asset: ${relative}" >&2
      exit 1
    fi
  done
done

RECORDING_INSTALL_DIR="${PACKAGE_ROOT}/build/native-recording"
if [[ -n "${LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE:-}" ]]; then
  rsync -aL "${LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE}/" "${RECORDING_INSTALL_DIR}/"
else
  cmake --install "${ROOT}/build/native-recording" --prefix "${RECORDING_INSTALL_DIR}"
fi
for relative in "${RECORDING_EXECUTABLES[@]}"; do
  if [[ ! -x "${RECORDING_INSTALL_DIR}/${relative}" ]]; then
    echo "Native recording install is missing executable: ${relative}" >&2
    exit 1
  fi
done

if [[ ! -f "${INSTALLER_SOURCE}" ]]; then
  echo "Native release installer source is missing: ${INSTALLER_SOURCE}" >&2
  exit 1
fi
install -m 0755 "${INSTALLER_SOURCE}" "${PACKAGE_ROOT}/install_nav.sh"

mkdir -p "${PACKAGE_ROOT}/config"
(
  cd "${PACKAGE_ROOT}"
  find build -type f -print0 \
    | LC_ALL=C sort -z \
    | xargs -0 sha256sum
) > "${PACKAGE_ROOT}/config/native-release-sha256.txt"

GIT_COMMIT="$(git -C "${ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
BUILD_EPOCH="${SOURCE_DATE_EPOCH:-$(git -C "${ROOT}" show -s --format=%ct HEAD 2>/dev/null || date +%s)}"
BUILD_TIME="$(python3 - "${BUILD_EPOCH}" <<'PY'
from datetime import datetime, timezone
import sys

print(datetime.fromtimestamp(int(sys.argv[1]), timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"))
PY
)"

python3 - \
  "${PACKAGE_ROOT}/metadata.json" \
  "${VERSION}" \
  "${ARCH}" \
  "${GIT_COMMIT}" \
  "${BUILD_TIME}" \
  "${PACKAGE_ROOT}/config/native-release-sha256.txt" <<'PY'
import json
from pathlib import Path
import sys

output, version, arch, commit, build_time, checksum_path = sys.argv[1:]
artifacts = []
for line in Path(checksum_path).read_text(encoding="utf-8").splitlines():
    digest, relative = line.split(maxsplit=1)
    artifacts.append({"path": relative, "sha256": digest})

payload = {
    "schema_version": "lingtu.native-release.v1",
    "version": version,
    "architecture": arch,
    "git_commit": commit,
    "build_time": build_time,
    "data_plane": "native_typed_dds",
    "artifacts": artifacts,
}
Path(output).write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY

TARBALL="${OUTPUT_DIR}/${PACKAGE_NAME}.tar.gz"
SHA_FILE="${OUTPUT_DIR}/${PACKAGE_NAME}.sha256"
MANIFEST="${OUTPUT_DIR}/${PACKAGE_NAME}-manifest.json"
SYSTEM_MANIFEST="${OUTPUT_DIR}/${PACKAGE_NAME}-system-manifest.json"

tar \
  --sort=name \
  --mtime="@${BUILD_EPOCH}" \
  --owner=0 \
  --group=0 \
  --numeric-owner \
  -C "${STAGING_DIR}" \
  -cf - "${PACKAGE_NAME}" \
  | gzip -n -9 > "${TARBALL}"

TARBALL_SHA="$(sha256sum "${TARBALL}" | awk '{print $1}')"
printf '%s  %s\n' "${TARBALL_SHA}" "$(basename "${TARBALL}")" > "${SHA_FILE}"

python3 - \
  "${MANIFEST}" \
  "${SYSTEM_MANIFEST}" \
  "${TARBALL}" \
  "${TARBALL_SHA}" \
  "${VERSION}" \
  "${ARCH}" \
  "${GIT_COMMIT}" \
  "${BUILD_TIME}" \
  "${LINGTU_NATIVE_RELEASE_SIGNING_KEY:-}" \
  "${LINGTU_NATIVE_RELEASE_KEY_ID:-ota-signing-key-01}" <<'PY'
import json
from pathlib import Path
import sys

(
    manifest_path,
    system_manifest_path,
    tarball_path,
    tarball_sha,
    version,
    arch,
    commit,
    build_time,
    signing_key,
    key_id,
) = sys.argv[1:]

tarball = Path(tarball_path)
artifact = {
    "name": "lingtu-native-release",
    "category": "package",
    "version": version.removeprefix("v"),
    "filename": tarball.name,
    "sha256": tarball_sha,
    "size_bytes": tarball.stat().st_size,
    "target_path": f"/opt/lingtu/releases/{version}",
    "target_board": "nav",
    "hw_compat": ["s100p", arch],
    "apply_action": "install_script",
    "install_script": "install_nav.sh",
    "requires_reboot": False,
    "min_battery_percent": 30,
    "changelog": "LingTu native typed-DDS runtime release",
    "rollback_safe": True,
    "safety_level": "cold",
    "owner_module": "system",
    "dependencies": [],
}
manifest = {
    "schema_version": "2",
    "release_version": version,
    "release_date": build_time,
    "channel": "stable",
    "min_system_version": "1.0.0",
    "signature": "",
    "public_key_id": key_id,
    "release_format": "lingtu.native-release.v1",
    "git_commit": commit,
    "artifacts": [artifact],
}

if signing_key:
    try:
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
        from cryptography.hazmat.primitives.serialization import load_pem_private_key
    except ImportError as exc:
        raise SystemExit("cryptography is required when a signing key is supplied") from exc

    key = load_pem_private_key(Path(signing_key).read_bytes(), password=None)
    if not isinstance(key, Ed25519PrivateKey):
        raise SystemExit("native release signing key must be Ed25519")
    unsigned = dict(manifest)
    unsigned.pop("signature", None)
    canonical = json.dumps(
        unsigned,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    )
    manifest["signature"] = key.sign(canonical.encode("utf-8")).hex()

Path(manifest_path).write_text(
    json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
    encoding="utf-8",
)

system_manifest = {
    "system_version": version.removeprefix("v"),
    "build_time": build_time,
    "release_format": "lingtu.native-release.v1",
    "components": {
        "lingtu-native-release": {
            "version": version.removeprefix("v"),
            "git_commit": commit,
            "sha256": tarball_sha,
        }
    },
}
Path(system_manifest_path).write_text(
    json.dumps(system_manifest, indent=2, ensure_ascii=False) + "\n",
    encoding="utf-8",
)
PY

echo "Native release package: ${TARBALL}"
echo "Native release manifest: ${MANIFEST}"
echo "Native release system manifest: ${SYSTEM_MANIFEST}"
