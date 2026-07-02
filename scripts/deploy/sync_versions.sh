#!/usr/bin/env bash
# Sync release metadata from the repository VERSION file.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
VERSION_FILE="${ROOT_DIR}/VERSION"
CHECK_ONLY=0

case "${1:-}" in
  --check)
    CHECK_ONLY=1
    ;;
  "")
    ;;
  *)
    echo "Usage: $0 [--check]" >&2
    exit 1
    ;;
esac

if [[ ! -f "$VERSION_FILE" ]]; then
  echo "ERROR: VERSION file not found at $VERSION_FILE" >&2
  exit 1
fi

VERSION="$(head -n 1 "$VERSION_FILE" | tr -d '[:space:]')"
if [[ ! "$VERSION" =~ ^[0-9]+[.][0-9]+[.][0-9]+(-[0-9A-Za-z.-]+)?([+][0-9A-Za-z.-]+)?$ ]]; then
  echo "ERROR: VERSION must be semver without leading v, got: $VERSION" >&2
  exit 1
fi

export LINGTU_ROOT="$ROOT_DIR"
export LINGTU_VERSION="$VERSION"
export LINGTU_SYNC_CHECK="$CHECK_ONLY"

PYTHON_BIN="${PYTHON:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN=python3
  elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN=python
  else
    echo "ERROR: python3 or python is required" >&2
    exit 1
  fi
fi

"$PYTHON_BIN" <<'PY'
from __future__ import annotations

import json
import os
import re
from pathlib import Path

root = Path(os.environ["LINGTU_ROOT"])
version = os.environ["LINGTU_VERSION"]
check_only = os.environ.get("LINGTU_SYNC_CHECK") == "1"
stale: list[Path] = []


def write_if_changed(path: Path, text: str) -> None:
    old = path.read_text(encoding="utf-8")
    if old == text:
        return
    if check_only:
        stale.append(path)
        print(f"  stale: {path.relative_to(root)}")
        return
    path.write_text(text, encoding="utf-8")
    print(f"  {path.relative_to(root)}")


def replace_one(path: Path, pattern: str, repl: str) -> None:
    if not path.exists():
        return
    text = path.read_text(encoding="utf-8")
    new, count = re.subn(pattern, repl, text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise SystemExit(f"ERROR: expected one version field in {path}")
    write_if_changed(path, new)


def replace_toml_section_string(path: Path, section: str, key: str, value: str) -> None:
    if not path.exists():
        return
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    in_section = False
    replaced = False
    header = f"[{section}]"
    key_re = re.compile(rf'^(\s*{re.escape(key)}\s*=\s*)"[^"]+"(.*)$')

    for idx, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            in_section = stripped == header
            continue
        if not in_section:
            continue

        newline = ""
        body = line
        if body.endswith("\r\n"):
            body = body[:-2]
            newline = "\r\n"
        elif body.endswith("\n"):
            body = body[:-1]
            newline = "\n"

        match = key_re.match(body)
        if match:
            lines[idx] = f'{match.group(1)}"{value}"{match.group(2)}{newline}'
            replaced = True
            break

    if not replaced:
        raise SystemExit(f"ERROR: expected {key} in [{section}] of {path}")
    write_if_changed(path, "".join(lines))


print(f"==> Syncing LingTu version: {version}")

replace_toml_section_string(root / "pyproject.toml", "project", "version", version)

replace_one(
    root / "config" / "robot_config.yaml",
    r'(^\s*firmware_version:\s*)".*"',
    rf'\g<1>"{version}"',
)

uv_lock = root / "uv.lock"
if uv_lock.exists():
    text = uv_lock.read_text(encoding="utf-8")
    new, count = re.subn(
        r'(\[\[package\]\]\nname = "lingtu"\nversion = )"[^"]+"',
        rf'\g<1>"{version}"',
        text,
        count=1,
    )
    if count != 1:
        raise SystemExit("ERROR: expected one lingtu package entry in uv.lock")
    write_if_changed(uv_lock, new)

package_json = root / "web" / "package.json"
if package_json.exists():
    data = json.loads(package_json.read_text(encoding="utf-8"))
    data["version"] = version
    write_if_changed(package_json, json.dumps(data, indent=2, ensure_ascii=False) + "\n")

package_lock = root / "web" / "package-lock.json"
if package_lock.exists():
    data = json.loads(package_lock.read_text(encoding="utf-8"))
    data["version"] = version
    if "" in data.get("packages", {}):
        data["packages"][""]["version"] = version
    write_if_changed(package_lock, json.dumps(data, indent=2, ensure_ascii=False) + "\n")

if stale:
    raise SystemExit("ERROR: version metadata is stale")

print("==> Version metadata is in sync" if check_only else "==> Done")
PY
