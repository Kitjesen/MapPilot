#!/usr/bin/env bash
# Shared Python selection for LingTu field deployment and runtime entrypoints.

resolve_lingtu_python() {
    local requested="${LINGTU_PYTHON:-python3}"
    local resolved=""

    if [[ "${requested}" == */* ]]; then
        resolved="${requested}"
    else
        resolved="$(command -v -- "${requested}" 2>/dev/null || true)"
    fi
    if [ -z "${resolved}" ] || [ ! -x "${resolved}" ]; then
        echo "ERROR: LingTu Python is missing or not executable: ${requested}" >&2
        return 2
    fi
    if [[ "${resolved}" != /* ]]; then
        resolved="$(cd -- "$(dirname -- "${resolved}")" && pwd -P)/$(basename -- "${resolved}")"
    fi
    if ! "${resolved}" -c \
        'import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)'; then
        echo "ERROR: Python 3.10 or newer is required: ${resolved}" >&2
        return 2
    fi
    printf '%s\n' "${resolved}"
}

install_lingtu_python_env() {
    local destination="${1:?python environment destination is required}"
    local python_bin="${2:-$(resolve_lingtu_python)}"
    local staged

    staged="$(mktemp)"
    printf 'LINGTU_PYTHON=%q\nexport LINGTU_PYTHON\n' "${python_bin}" >"${staged}"
    if ! sudo install -o root -g root -m 0644 "${staged}" "${destination}"; then
        rm -f -- "${staged}"
        return 1
    fi
    rm -f -- "${staged}"
}
