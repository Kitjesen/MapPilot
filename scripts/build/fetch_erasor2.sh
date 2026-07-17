#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ERASOR2_REPO="${LINGTU_ERASOR2_REPO:-https://github.com/url-kaist/ERASOR2.git}"
ERASOR2_REV="${LINGTU_ERASOR2_REV:-d43d94f7e06a900456042979e29c3c933a39fd48}"
ERASOR2_DIR="${LINGTU_ERASOR2_DIR:-${REPO_ROOT}/third_party/research_nav/ERASOR2}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/build/fetch_erasor2.sh

Environment:
  LINGTU_ERASOR2_REPO   Default: https://github.com/url-kaist/ERASOR2.git
  LINGTU_ERASOR2_REV    Default: d43d94f7e06a900456042979e29c3c933a39fd48
  LINGTU_ERASOR2_DIR    Default: third_party/research_nav/ERASOR2
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

mkdir -p "$(dirname "${ERASOR2_DIR}")"

if [[ -e "${ERASOR2_DIR}" && ! -d "${ERASOR2_DIR}/.git" ]]; then
  printf 'ERROR: %s exists but is not a git checkout.\n' "${ERASOR2_DIR}" >&2
  exit 1
fi

if [[ ! -d "${ERASOR2_DIR}/.git" ]]; then
  git clone "${ERASOR2_REPO}" "${ERASOR2_DIR}"
fi

git -C "${ERASOR2_DIR}" fetch --tags origin "${ERASOR2_REV}"
git -C "${ERASOR2_DIR}" checkout --detach "${ERASOR2_REV}"

printf 'ERASOR2_DIR=%s\n' "${ERASOR2_DIR}"
git -C "${ERASOR2_DIR}" rev-parse HEAD
