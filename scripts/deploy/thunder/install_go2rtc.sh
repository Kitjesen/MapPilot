#!/usr/bin/env bash
# scripts/deploy/thunder/install_go2rtc.sh — install go2rtc sidecar on S100P (aarch64).
#
# Why a sidecar?
#   go2rtc is a single Go binary that handles the media hot path natively
#   and exposes the WHEP endpoint used by the dashboard.
#   It is an optional machine-level external media sidecar. It stays outside
#   ProductControl, is not part of any Product or RunPlan, and does not
#   participate in Product readiness. Unavailable WHEP falls back to Gateway JPEG.
#
# Usage:
#   sudo bash scripts/deploy/thunder/install_go2rtc.sh           # systemd mode
#   bash scripts/deploy/thunder/install_go2rtc.sh --no-systemd   # just install the binary
#
# Security:
#   Set GO2RTC_SHA256 to the expected SHA-256 for the selected GO2RTC_VERSION.
#   The installer refuses to install downloaded binaries without verification.
#   For local lab testing only, set GO2RTC_SKIP_SHA256=1 to bypass this check.
#
set -euo pipefail

GO2RTC_VERSION="${GO2RTC_VERSION:-v1.9.10}"
BIN_URL="https://github.com/AlexxIT/go2rtc/releases/download/${GO2RTC_VERSION}/go2rtc_linux_arm64"
BIN_PATH="${BIN_PATH:-/usr/local/bin/go2rtc}"
GO2RTC_SHA256="${GO2RTC_SHA256:-}"
GO2RTC_SKIP_SHA256="${GO2RTC_SKIP_SHA256:-0}"
CONFIG_DIR="${CONFIG_DIR:-/etc/go2rtc}"
CONFIG_PATH="${CONFIG_DIR}/go2rtc.yaml"
SYSTEMD_UNIT="/etc/systemd/system/go2rtc.service"

find_repo_root() {
    local dir="$1"
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/pyproject.toml" && -f "$dir/AGENTS.md" ]]; then
            printf '%s\n' "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    return 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(find_repo_root "$SCRIPT_DIR")"

INSTALL_SYSTEMD=1
for arg in "$@"; do
    case "$arg" in
        --no-systemd) INSTALL_SYSTEMD=0 ;;
        --help|-h)
            sed -n '2,/^set -e/p' "$0" | sed 's/^# \?//'
            exit 0
            ;;
    esac
done

arch="$(uname -m)"
if [[ "$arch" != "aarch64" && "$arch" != "arm64" ]]; then
    echo "WARNING: this installer targets aarch64 (S100P); detected $arch" >&2
fi

if [[ -z "$GO2RTC_SHA256" && "$GO2RTC_SKIP_SHA256" != "1" ]]; then
    cat >&2 <<EOF
FAIL: GO2RTC_SHA256 is required before installing a downloaded go2rtc binary.

Look up the official SHA-256 for ${GO2RTC_VERSION} go2rtc_linux_arm64, then run:
  GO2RTC_SHA256=<expected-sha256> bash scripts/deploy/thunder/install_go2rtc.sh

For local lab testing only, bypass verification explicitly with:
  GO2RTC_SKIP_SHA256=1 bash scripts/deploy/thunder/install_go2rtc.sh
EOF
    exit 1
fi

tmp_bin="$(mktemp)"
trap 'rm -f "$tmp_bin"' EXIT

echo "==> Downloading go2rtc ${GO2RTC_VERSION}"
curl -fsSL "$BIN_URL" -o "$tmp_bin"
if [[ -n "$GO2RTC_SHA256" ]]; then
    echo "==> Verifying SHA-256"
    printf '%s  %s\n' "$GO2RTC_SHA256" "$tmp_bin" | sha256sum -c -
else
    echo "WARNING: GO2RTC_SKIP_SHA256=1 set; installing unverified binary" >&2
fi

echo "==> Installing go2rtc 鈫?${BIN_PATH}"
sudo mkdir -p "$(dirname "$BIN_PATH")"
sudo install -m 0755 "$tmp_bin" "$BIN_PATH"
"$BIN_PATH" --version || { echo "FAIL: go2rtc binary not executable"; exit 1; }

echo "==> Installing config 鈫?${CONFIG_PATH}"
sudo mkdir -p "$CONFIG_DIR"
if [[ -f "$CONFIG_PATH" ]]; then
    echo "    (existing config kept; template at ${REPO_ROOT}/config/go2rtc.yaml)"
else
    sudo cp "${REPO_ROOT}/config/go2rtc.yaml" "$CONFIG_PATH"
fi

if [[ "$INSTALL_SYSTEMD" == "1" ]]; then
    echo "==> Writing systemd unit 鈫?${SYSTEMD_UNIT}"
    sudo tee "$SYSTEMD_UNIT" > /dev/null <<EOF
[Unit]
Description=go2rtc 鈥?low-latency camera gateway (LingTu sidecar)
After=network.target

[Service]
Type=simple
ExecStart=${BIN_PATH} -config ${CONFIG_PATH}
Restart=always
RestartSec=2
# Give it realtime-ish priority so ffmpeg ingestion doesn't starve.
Nice=-5
# v4l2 device + dma-heap (HW codec) access
SupplementaryGroups=video render

[Install]
WantedBy=multi-user.target
EOF
    # Machine-level provisioning is intentionally outside ProductControl, so
    # the installer owns daemon-reload/enable/restart for this optional sidecar.
    sudo systemctl daemon-reload
    sudo systemctl enable --now go2rtc
    sleep 1
    echo "==> Service status"
    sudo systemctl --no-pager status go2rtc | head -12 || true
    echo
    echo "鉁?go2rtc running.  WHEP endpoint:  http://<robot-ip>:1984/api/webrtc?src=cam"
    echo "   Web UI (debug):                   http://<robot-ip>:1984/"
    echo "   Logs:                             journalctl -u go2rtc -f"
else
    echo
    echo "鉁?Installed.  Run manually: ${BIN_PATH} -config ${CONFIG_PATH}"
fi

cat <<'POST'

Next steps:
  This optional external media sidecar stays outside ProductControl.
  Installing, restarting, or losing it does not change the active Product,
  RunPlan, or Product readiness; unavailable WHEP falls back to Gateway JPEG.
  1. Edit ${CONFIG_PATH} and point it at your camera device
     (default: /dev/video0, the Orbbec Gemini RGB UVC node).
     Run `v4l2-ctl --list-devices` to discover the right path.
  2. Restart the service:  sudo systemctl restart go2rtc
  3. Open the LingTu dashboard; CameraFeed will auto-detect go2rtc
     and use WHEP. If unavailable, it falls back to the Gateway JPEG stream.
  4. For troubleshooting, see src/gateway/README.md and
     chrome://webrtc-internals.
POST
