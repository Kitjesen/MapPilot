#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${LINGTU_ERASOR2_BUILD_DIR:-${REPO_ROOT}/build/research-erasor2}"
if [[ "$#" -eq 1 && "$1" == "--help" ]]; then
  printf 'Usage: bash research/maps/erasor2/check.sh MAP_DIRECTORY NEW_STAGE_DIRECTORY\n'
  printf 'Legacy synthetic labels. This checks execution only, not cleaning quality.\n'
  exit 0
fi
if [[ "$#" -ne 2 ]]; then
  printf 'Expected an existing map directory and a new stage directory.\n' >&2
  exit 2
fi
bash "${SCRIPT_DIR}/build.sh"
"${BUILD_DIR}/erasor2_clean" --map-dir "$1" --stage-dir "$2" --skip-run
printf 'Dataset staging smoke check passed. Quality comparison requires real labels.\n'
