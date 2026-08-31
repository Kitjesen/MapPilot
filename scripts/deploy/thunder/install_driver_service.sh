#!/usr/bin/env bash
# Install the single native field-driver service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
source "${SCRIPT_DIR}/../python-runtime.sh"
PYTHON_BIN="$(resolve_lingtu_python)"
export LINGTU_PYTHON="${PYTHON_BIN}"

configured_backend="$(
  PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
    "${PYTHON_BIN}" - <<'PY'
from runtime.config import load_config, validate_config

config = load_config()
errors = validate_config(config)
if errors:
    raise SystemExit("; ".join(errors))
print(config.driver.backend)
PY
)"

if [[ -n "${LINGTU_DRIVER_BACKEND:-}" ]] &&
    [[ "${LINGTU_DRIVER_BACKEND}" != "${configured_backend}" ]]; then
  echo "ERROR: requested driver backend disagrees with RobotConfig." >&2
  exit 64
fi

echo "Installing lt-driver for RobotConfig backend: ${configured_backend}"
bash "${SCRIPT_DIR}/install_catalog_service.sh" driver

LEGACY_MOTION_SINKS=(
  "lingtu-thunder-dds-endpoint.service"
  "thunder-dds-endpoint.service"
  "dds-endpoint.service"
  "lingtu-thunder-lite.service"
  "driver.service"
)

for unit in "${LEGACY_MOTION_SINKS[@]}"; do
  if systemctl list-unit-files "${unit}" --no-legend 2>/dev/null \
      | grep -Fq "${unit}"; then
    sudo systemctl disable --now "${unit}"
    echo "Disabled legacy hardware motion sink: ${unit}"
  fi
done
