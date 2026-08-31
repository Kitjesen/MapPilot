#!/usr/bin/env bash
# Start only MID-360 and Fast-LIO, prove sensor freshness, then cleanly stop.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
export LINGTU_REPO="${LINGTU_REPO:-$repo_root}"

runtime_env="${LINGTU_RUNTIME_ENV_FILE:-/opt/lingtu/config/thunder-runtime-env.sh}"
if [[ -r "$runtime_env" ]]; then
  source "$runtime_env"
elif [[ -r "$repo_root/scripts/deploy/thunder/runtime-env.sh" ]]; then
  source "$repo_root/scripts/deploy/thunder/runtime-env.sh"
fi
if declare -F prepend_cyclonedds_libs >/dev/null 2>&1; then
  prepend_cyclonedds_libs
fi

: "${LINGTU_PYTHON:=python3}"
export PYTHONPATH="$repo_root/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$LINGTU_PYTHON" -m diagnostics.field.go2_mid360_no_motion "$@"
