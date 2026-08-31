#!/usr/bin/env bash
# Fetch the field-tested pure Orbbec SDK v2 package into the ignored build tree.

set -euo pipefail

REPO="${LINGTU_ORBBEC_SDK_REPO:-orbbec/OrbbecSDK_v2}"
REF="${LINGTU_ORBBEC_SDK_REF:-v2.8.7}"
DEST="${LINGTU_ORBBEC_SDK_DEST:-build/deps/orbbec-sdk}"
ARCH="${LINGTU_ORBBEC_SDK_ARCH:-$(uname -m)}"
TMP="${LINGTU_ORBBEC_SDK_TMP:-${TMPDIR:-/tmp}/lingtu-orbbec-sdk}"

case "$ARCH" in
  aarch64|arm64) ASSET_RE='OrbbecSDK_.*(arm64|aarch64).*(deb|tar.gz|tgz|zip)$' ;;
  x86_64|amd64) ASSET_RE='OrbbecSDK_.*(amd64|x64|x86_64).*(deb|tar.gz|tgz|zip)$' ;;
  *) echo "unsupported architecture: $ARCH" >&2; exit 1 ;;
esac

need() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 1
  }
}

need curl
need python3
need find

rm -rf "$TMP"
mkdir -p "$TMP"

API_URL="https://api.github.com/repos/${REPO}/releases/tags/${REF}"

ASSET_URL="$(
  RELEASE_JSON="${TMP}/release.json"
  curl -fsSL "$API_URL" -o "$RELEASE_JSON"
  python3 - "$ASSET_RE" "$RELEASE_JSON" <<'PY'
import json
import re
import sys

pattern = re.compile(sys.argv[1], re.I)
with open(sys.argv[2], encoding="utf-8") as handle:
    payload = json.load(handle)
for asset in payload.get("assets", []):
    name = asset.get("name", "")
    if pattern.search(name):
        print(asset["browser_download_url"])
        break
else:
    raise SystemExit(f"no OrbbecSDK asset matched {pattern.pattern!r}")
PY
)"

ASSET_NAME="${ASSET_URL##*/}"
ARCHIVE="${TMP}/${ASSET_NAME}"
curl -fL "$ASSET_URL" -o "$ARCHIVE"

rm -rf "${TMP}/extract"
mkdir -p "${TMP}/extract"

case "$ASSET_NAME" in
  *.deb)
    need dpkg-deb
    dpkg-deb -x "$ARCHIVE" "${TMP}/extract"
    SRC="${TMP}/extract/usr/local"
    ;;
  *.tar.gz|*.tgz)
    need tar
    tar -xzf "$ARCHIVE" -C "${TMP}/extract"
    SRC="$(find "${TMP}/extract" -type d -path '*/include/libobsensor' -print -quit)"
    SRC="${SRC%/include/libobsensor}"
    ;;
  *.zip)
    need unzip
    unzip -q "$ARCHIVE" -d "${TMP}/extract"
    SRC="$(find "${TMP}/extract" -type d -path '*/include/libobsensor' -print -quit)"
    SRC="${SRC%/include/libobsensor}"
    ;;
  *)
    echo "unsupported SDK asset type: $ASSET_NAME" >&2
    exit 1
    ;;
esac

if [[ -z "${SRC:-}" || ! -d "$SRC/include" ]]; then
  echo "downloaded SDK does not contain include/: $ASSET_NAME" >&2
  exit 1
fi
if ! find "$SRC" -type f -name 'libOrbbecSDK.so*' | grep -q .; then
  echo "downloaded SDK does not contain libOrbbecSDK.so*: $ASSET_NAME" >&2
  exit 1
fi

if [[ -e "$DEST" && ! -d "$DEST/.git" && ! -f "$DEST/.lingtu-orbbec-sdk" ]]; then
  echo "refusing to overwrite non-managed path: $DEST" >&2
  exit 1
fi

rm -rf "$DEST"
mkdir -p "$DEST"
cp -a "$SRC"/. "$DEST"/
{
  echo "repo=$REPO"
  echo "ref=$REF"
  echo "asset=$ASSET_NAME"
  echo "url=$ASSET_URL"
} > "$DEST/.lingtu-orbbec-sdk"

echo "Orbbec SDK ready at $DEST from $ASSET_NAME"
