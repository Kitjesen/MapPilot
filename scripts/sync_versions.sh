#!/bin/bash
# sync_versions.sh — Reads VERSION file and propagates to all package manifests.
# Run: ./scripts/sync_versions.sh
# This ensures package.xml, pubspec.yaml, and MSIX versions stay in sync.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
VERSION_FILE="${ROOT_DIR}/VERSION"

if [[ ! -f "$VERSION_FILE" ]]; then
  echo "ERROR: VERSION file not found at $VERSION_FILE" >&2
  exit 1
fi

VERSION=$(head -1 "$VERSION_FILE" | tr -d '[:space:]')
echo "==> Syncing version: $VERSION"

# ── 1. ROS2 package.xml files ──
find "$ROOT_DIR/src" -name package.xml -not -path '*/3rdparty/*' | while read -r f; do
  if grep -q '<version>' "$f"; then
    sed -i "s|<version>[^<]*</version>|<version>${VERSION}</version>|" "$f"
    echo "  [pkg.xml] $f"
  fi
done

# ── 2. Flutter pubspec.yaml ──
PUBSPEC="${ROOT_DIR}/client/flutter_monitor/pubspec.yaml"
if [[ -f "$PUBSPEC" ]]; then
  sed -i "s|^version:.*|version: ${VERSION}|" "$PUBSPEC"
  echo "  [pubspec] $PUBSPEC"
fi

# ── 3. MSIX version (needs 4-part: X.Y.Z.0) ──
MSIX_VERSION="${VERSION}.0"
if [[ -f "$PUBSPEC" ]]; then
  sed -i "s|msix_version:.*|msix_version: ${MSIX_VERSION}|" "$PUBSPEC"
  echo "  [msix]    msix_version -> $MSIX_VERSION"
fi

# ── 4. Dart kAppVersion constant ──
SPLASH="${ROOT_DIR}/client/flutter_monitor/lib/features/connection/splash_screen.dart"
if [[ -f "$SPLASH" ]]; then
  sed -i "s|const String kAppVersion = '[^']*'|const String kAppVersion = '${VERSION}'|" "$SPLASH"
  echo "  [dart]    kAppVersion -> $VERSION"
fi

# ── 5. robot_config.yaml firmware_version ──
ROBOT_CFG="${ROOT_DIR}/config/robot_config.yaml"
if [[ -f "$ROBOT_CFG" ]]; then
  sed -i "s|firmware_version:.*|firmware_version: \"${VERSION}\"|" "$ROBOT_CFG"
  echo "  [yaml]    robot_config firmware_version -> $VERSION"
fi

# ── 6. grpc_gateway.yaml firmware_version ──
GRPC_CFG="${ROOT_DIR}/src/remote_monitoring/config/grpc_gateway.yaml"
if [[ -f "$GRPC_CFG" ]]; then
  sed -i "s|firmware_version:.*|firmware_version: \"${VERSION}\"|" "$GRPC_CFG"
  echo "  [yaml]    grpc_gateway firmware_version -> $VERSION"
fi

echo "==> Done. All versions set to $VERSION"
