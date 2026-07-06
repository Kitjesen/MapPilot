#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${LINGTU_ROOT:-}" ]]; then
  ROOT="${LINGTU_ROOT}"
else
  ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
  if [[ -z "${ROOT}" ]]; then
    candidate="${SCRIPT_DIR}"
    while [[ "${candidate}" != "/" ]]; do
      if [[ -f "${candidate}/sim/scripts/mujoco/live_gate.py" ]] && [[ -d "${candidate}/src" ]]; then
        ROOT="${candidate}"
        break
      fi
      candidate="$(dirname "${candidate}")"
    done
  fi
  if [[ -z "${ROOT}" ]]; then
    ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
  fi
fi
cd "${ROOT}"

source_setup_if_present() {
  local setup_path="$1"
  if [[ -f "${setup_path}" ]]; then
    set +u
    source "${setup_path}"
    set -u
  fi
}

source_setup_if_present /opt/ros/humble/setup.bash
source_setup_if_present "${ROOT}/install/setup.bash"

LINGTU_SPEED_SCAN_PYTHONPATH="src:."
if [[ -n "${PYTHONPATH:-}" ]]; then
  LINGTU_SPEED_SCAN_PYTHONPATH="${LINGTU_SPEED_SCAN_PYTHONPATH}:${PYTHONPATH}"
fi

WORLD="${LINGTU_SPEED_SCAN_WORLD:-artifacts/server_sim_closure/large_terrain_odom/large_terrain_scene.xml}"
DURATION="${LINGTU_SPEED_SCAN_DURATION:-20}"
COMMON_ARGS=(
  --world "${WORLD}"
  --duration "${DURATION}"
  --duration-clock sim
  --drive-source fixed
  --drive-vy 0
  --nav-data-source fastlio2
  --runtime-fault-confirm-samples 2
  --max-fastlio-z-drift-m 1.0
  --max-fastlio-yaw-drift-rad 0.5
)

mkdir -p artifacts/server_sim_closure/diagnosis_matrix

read -r -a SPEED_SCAN_VX_VALUES <<< "${LINGTU_SPEED_SCAN_VX_VALUES:-0.060 0.075 0.090 0.100}"
read -r -a SPEED_SCAN_WZ_VALUES <<< "${LINGTU_SPEED_SCAN_WZ_VALUES:-0.000 0.250}"
read -r -a COMMAND_SHAPE_VX_VALUES <<< "${LINGTU_COMMAND_SHAPE_VX_VALUES:-0.000 0.100}"
read -r -a COMMAND_SHAPE_VY_VALUES <<< "${LINGTU_COMMAND_SHAPE_VY_VALUES:-0.000 0.080}"
read -r -a COMMAND_SHAPE_WZ_VALUES <<< "${LINGTU_COMMAND_SHAPE_WZ_VALUES:-0.000 0.250 0.450}"

for vx in "${SPEED_SCAN_VX_VALUES[@]}"; do
  for wz in "${SPEED_SCAN_WZ_VALUES[@]}"; do
    out="${ROOT}/artifacts/server_sim_closure/diagnosis_matrix/speed_boundary_refined/fixed_vx_${vx}_wz_${wz}"
    mkdir -p "${out}"
    PYTHONPATH="${LINGTU_SPEED_SCAN_PYTHONPATH}" python3 sim/scripts/mujoco/live_gate.py \
      "${COMMON_ARGS[@]}" \
      --drive-vx "${vx}" \
      --drive-wz "${wz}" \
      --json-out "${out}/report.json" --strict || true
  done
done

for vx in "${COMMAND_SHAPE_VX_VALUES[@]}"; do
  for vy in "${COMMAND_SHAPE_VY_VALUES[@]}"; do
    for wz in "${COMMAND_SHAPE_WZ_VALUES[@]}"; do
      out="${ROOT}/artifacts/server_sim_closure/diagnosis_matrix/command_shape_boundary/fixed_vx_${vx}_vy_${vy}_wz_${wz}"
      mkdir -p "${out}"
      PYTHONPATH="${LINGTU_SPEED_SCAN_PYTHONPATH}" python3 sim/scripts/mujoco/live_gate.py \
        "${COMMON_ARGS[@]}" \
        --drive-vx "${vx}" \
        --drive-vy "${vy}" \
        --drive-wz "${wz}" \
        --json-out "${out}/report.json" --strict || true
    done
  done
done

for profile in physical_rolling synthetic_rolling instantaneous; do
  out="${ROOT}/artifacts/server_sim_closure/diagnosis_matrix/scan_timing/vx010_${profile}"
  mkdir -p "${out}"
  PYTHONPATH="${LINGTU_SPEED_SCAN_PYTHONPATH}" python3 sim/scripts/mujoco/live_gate.py \
    "${COMMON_ARGS[@]}" \
    --drive-vx 0.10 \
    --drive-wz 0.0 \
    --scan-time-profile "${profile}" \
    --imu-acc-mode finite_difference \
    --json-out "${out}/report.json" --strict || true
done

out="${ROOT}/artifacts/server_sim_closure/diagnosis_matrix/tuned_fastlio/vx010_best_known"
mkdir -p "${out}"
PYTHONPATH="${LINGTU_SPEED_SCAN_PYTHONPATH}" python3 sim/scripts/mujoco/live_gate.py \
  "${COMMON_ARGS[@]}" \
  --drive-vx 0.10 \
  --drive-wz 0.0 \
  --fastlio-lidar-filter-num 2 \
  --fastlio-scan-resolution 0.10 \
  --fastlio-map-resolution 0.20 \
  --fastlio-near-search-num 8 \
  --fastlio-ieskf-max-iter 8 \
  --fastlio-lidar-cov-inv 500 \
  --json-out "${out}/report.json" --strict || true

python3 sim/scripts/fastlio_speed_boundary_gate.py \
  --report-glob "artifacts/server_sim_closure/diagnosis_matrix/fastlio_controls/*/*/report.json" \
  --report-glob "artifacts/server_sim_closure/diagnosis_matrix/speed_boundary_refined/*/report.json" \
  --report-glob "artifacts/server_sim_closure/diagnosis_matrix/command_shape_boundary/*/report.json" \
  --report-glob "artifacts/server_sim_closure/diagnosis_matrix/scan_timing/*/report.json" \
  --report-glob "artifacts/server_sim_closure/diagnosis_matrix/tuned_fastlio/*/report.json" \
  --json-out artifacts/server_sim_closure/diagnosis_matrix/fastlio_speed_boundary/refined_report.json \
  --strict || true
