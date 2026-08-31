#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${ROOT}/scripts/deploy/python-runtime.sh"

TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

ln -s "$(command -v python3)" "${TEST_ROOT}/python"
LINGTU_PYTHON="${TEST_ROOT}/python"
RESOLVED="$(resolve_lingtu_python)"

if [ "${RESOLVED}" != "${LINGTU_PYTHON}" ]; then
    echo "virtualenv-style Python symlink was dereferenced: ${RESOLVED}" >&2
    exit 1
fi

FAKE_PYTHON="${TEST_ROOT}/field-python"
SELECTED_PYTHON="${TEST_ROOT}/selected-python"
cat >"${FAKE_PYTHON}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
if [ "${1:-}" = "-c" ]; then
    exit 0
fi
printf '%s\n' "$0" >"${LINGTU_TEST_SELECTED_PYTHON:?}"
EOF
chmod +x "${FAKE_PYTHON}"
cat >"${TEST_ROOT}/python.env" <<EOF
LINGTU_PYTHON=${FAKE_PYTHON}
export LINGTU_PYTHON
EOF

env -u LINGTU_PYTHON \
    LINGTU_PYTHON_ENV="${TEST_ROOT}/python.env" \
    LINGTU_TEST_SELECTED_PYTHON="${SELECTED_PYTHON}" \
    bash "${ROOT}/scripts/lingtu" status

if [ "$(cat "${SELECTED_PYTHON}")" != "${FAKE_PYTHON}" ]; then
    echo "scripts/lingtu ignored the installed Python environment" >&2
    exit 1
fi

echo "python runtime tests passed"
