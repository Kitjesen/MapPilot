#!/usr/bin/env bash
# =============================================================================
# apply_firmware.sh - Robot firmware apply script template
#
# Called by the gRPC gateway after a firmware file has been uploaded.
# Usage: apply_firmware.sh <firmware_path>
#
# Install to: /usr/local/bin/apply_firmware.sh
# Make executable: chmod +x /usr/local/bin/apply_firmware.sh
#
# This is a TEMPLATE. Customize the APPLY step for your robot hardware.
# =============================================================================

set -euo pipefail

FIRMWARE_PATH="${1:?Usage: apply_firmware.sh <firmware_path>}"
LOG_FILE="/tmp/apply_firmware.log"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

log "========================================="
log "Firmware apply started"
log "File: $FIRMWARE_PATH"
log "========================================="

# 1. Validate file exists
if [ ! -f "$FIRMWARE_PATH" ]; then
    log "ERROR: Firmware file not found: $FIRMWARE_PATH"
    exit 1
fi

FILE_SIZE=$(stat -c%s "$FIRMWARE_PATH" 2>/dev/null || stat -f%z "$FIRMWARE_PATH" 2>/dev/null || echo "unknown")
log "File size: $FILE_SIZE bytes"

# 2. Optional: Verify checksum (uncomment and customize)
# EXPECTED_MD5="your_expected_md5_here"
# ACTUAL_MD5=$(md5sum "$FIRMWARE_PATH" | awk '{print $1}')
# if [ "$EXPECTED_MD5" != "$ACTUAL_MD5" ]; then
#     log "ERROR: Checksum mismatch! Expected=$EXPECTED_MD5 Actual=$ACTUAL_MD5"
#     exit 2
# fi
# log "Checksum verified: $ACTUAL_MD5"

# 3. Backup current firmware (optional)
# BACKUP_DIR="/firmware/backup"
# mkdir -p "$BACKUP_DIR"
# cp /path/to/current/firmware "$BACKUP_DIR/firmware_$(date +%Y%m%d_%H%M%S).bak"
# log "Current firmware backed up"

# 4. Detect firmware type and apply
# ===========================================================================
FIRMWARE_EXT="${FIRMWARE_PATH##*.}"
FIRMWARE_DIR="/home/sunrise/firmware"
FIRMWARE_INSTALL_DIR="/opt/robot/firmware"
mkdir -p "$FIRMWARE_DIR" "$FIRMWARE_INSTALL_DIR"

case "$FIRMWARE_EXT" in
    deb)
        log "Detected DEB package, installing..."
        dpkg -i "$FIRMWARE_PATH" 2>&1 | tee -a "$LOG_FILE"
        apt-get -f install -y 2>&1 | tee -a "$LOG_FILE"
        log "DEB package installed"
        ;;
    bin|hex)
        log "Detected binary firmware ($FIRMWARE_EXT)"
        # STM32 串口刷写（根据实际硬件调整端口和波特率）
        if command -v stm32flash &>/dev/null; then
            SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
            BAUD_RATE="${BAUD_RATE:-115200}"
            log "Flashing via stm32flash: port=$SERIAL_PORT baud=$BAUD_RATE"
            stm32flash -w "$FIRMWARE_PATH" -v -b "$BAUD_RATE" "$SERIAL_PORT" 2>&1 | tee -a "$LOG_FILE"
            log "STM32 flash completed"
        else
            # Fallback: 复制到安装目录
            cp "$FIRMWARE_PATH" "$FIRMWARE_INSTALL_DIR/"
            log "Firmware copied to $FIRMWARE_INSTALL_DIR/ (stm32flash not available)"
        fi
        ;;
    img)
        log "Detected raw image, writing to block device..."
        # 危险操作，默认禁用，需显式设置 TARGET_DEVICE 环境变量
        TARGET_DEVICE="${TARGET_DEVICE:-}"
        if [ -z "$TARGET_DEVICE" ]; then
            log "WARNING: TARGET_DEVICE not set, skipping raw flash."
            log "Set TARGET_DEVICE env var (e.g. /dev/mmcblk0p2) to enable."
            cp "$FIRMWARE_PATH" "$FIRMWARE_INSTALL_DIR/"
            log "Image copied to $FIRMWARE_INSTALL_DIR/ instead."
        else
            log "Writing to $TARGET_DEVICE ..."
            dd if="$FIRMWARE_PATH" of="$TARGET_DEVICE" bs=4M status=progress 2>&1 | tee -a "$LOG_FILE"
            sync
            log "Raw image write completed"
        fi
        ;;
    tar|tar.gz|tgz)
        log "Detected archive, extracting..."
        tar -xzf "$FIRMWARE_PATH" -C "$FIRMWARE_INSTALL_DIR/" 2>&1 | tee -a "$LOG_FILE"
        # 检查是否有安装脚本
        if [ -f "$FIRMWARE_INSTALL_DIR/install.sh" ]; then
            log "Running install.sh..."
            chmod +x "$FIRMWARE_INSTALL_DIR/install.sh"
            bash "$FIRMWARE_INSTALL_DIR/install.sh" 2>&1 | tee -a "$LOG_FILE"
        fi
        log "Archive extracted and installed"
        ;;
    *)
        log "Unknown firmware type: .$FIRMWARE_EXT"
        cp "$FIRMWARE_PATH" "$FIRMWARE_INSTALL_DIR/"
        log "File copied to $FIRMWARE_INSTALL_DIR/"
        ;;
esac

# 5. Post-install: restart relevant services
log "Restarting robot services..."
if systemctl is-active --quiet robot_navigation; then
    systemctl restart robot_navigation 2>&1 | tee -a "$LOG_FILE" || true
    log "robot_navigation service restarted"
fi

# 6. Optional: Full reboot (uncomment if your firmware requires it)
# log "Rebooting in 10 seconds..."
# sleep 10
# reboot

log "========================================="
log "Firmware apply script completed successfully."
log "========================================="
exit 0
