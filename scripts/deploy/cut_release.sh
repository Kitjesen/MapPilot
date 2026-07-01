#!/bin/bash
# Cut a new LingTu Thunder release on the field robot.
#
# Snapshot the current development checkout into
# /opt/lingtu/releases/<version>/, atomically swap /opt/lingtu/current, and
# restart release-mode services.
#
# Usage on the robot:
#   bash scripts/deploy/cut_release.sh v2.1.1
#
# Rollback:
#   sudo ln -sfn /opt/lingtu/releases/v2.1.0 /opt/lingtu/current
#   sudo systemctl restart lingtu robot-fastlio2 robot-localizer

set -euo pipefail

VERSION="${1:-}"
if [ -z "$VERSION" ]; then
    echo "Usage: $0 <version>   (example: v2.1.1)"
    exit 1
fi
if [[ ! "$VERSION" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "ERROR: version must look like v2.1.1 (got: $VERSION)"
    exit 1
fi

DEV_DIR="${LINGTU_DEV_DIR:-$HOME/data/SLAM/navigation}"
RELEASES_DIR="${LINGTU_RELEASES_DIR:-/opt/lingtu/releases}"
TARGET_DIR="$RELEASES_DIR/$VERSION"
CURRENT_LINK="${LINGTU_CURRENT_LINK:-/opt/lingtu/current}"

if [ -e "$TARGET_DIR" ]; then
    echo "ERROR: $TARGET_DIR already exists. Pick a different version."
    exit 1
fi

if [ ! -d "$DEV_DIR/install" ]; then
    echo "ERROR: $DEV_DIR/install not found. Build first: cd $DEV_DIR && colcon build"
    exit 1
fi

RUNTIME_PKGS="fastlio2 genz_icp hba interface livox_ros_driver2 localizer local_planner nav_core nav_services pct_adapters pct_planner pgo pointlio semantic_perception semantic_planner sensor_scan_generation tare_planner terrain_analysis terrain_analysis_ext wtrtk980_ros2_reader"
for pkg in $RUNTIME_PKGS; do
    if [ ! -d "$DEV_DIR/install/$pkg" ]; then
        echo "ERROR: install/$pkg missing in dev checkout; refusing to cut release."
        exit 1
    fi
done

echo "========================================"
echo "  Cutting LingTu Thunder release: $VERSION"
echo "  From: $DEV_DIR"
echo "  To:   $TARGET_DIR"
echo "========================================"

echo ">>> [1/5] Building $TARGET_DIR via rsync..."
sudo mkdir -p "$TARGET_DIR"
sudo chown "$USER:$USER" "$TARGET_DIR"
rsync -aL \
    --exclude=.git \
    --exclude=build \
    --exclude=log \
    --exclude=logs \
    --exclude=.lingtu \
    --exclude=.omc \
    --exclude=.pytest_cache \
    --exclude=__pycache__ \
    --exclude='*.pyc' \
    --exclude=research \
    "$DEV_DIR/" "$TARGET_DIR/"

REL_SIZE="$(du -sh "$TARGET_DIR" | awk '{print $1}')"
echo ">>> [1/5] Done; release size: $REL_SIZE"

PREV_TARGET="$(readlink "$CURRENT_LINK" 2>/dev/null || echo none)"
echo ">>> [2/5] Previous current: $PREV_TARGET"

echo ">>> [3/5] Swapping $CURRENT_LINK -> $TARGET_DIR ..."
sudo ln -sfn "$TARGET_DIR" "$CURRENT_LINK"
echo ">>> [3/5] Done."

echo ">>> [4/5] Restarting services..."
sudo systemctl restart robot-fastlio2.service
sleep 3
sudo systemctl restart robot-localizer.service
sleep 3
sudo systemctl restart lingtu.service
echo ">>> [4/5] Done."

echo ">>> [5/5] Waiting for /api/v1/health to report a clean module set..."
DEADLINE=$((SECONDS + 60))
while [ $SECONDS -lt $DEADLINE ]; do
    HEALTH="$(curl -sf --max-time 2 http://localhost:5050/api/v1/health 2>/dev/null || true)"
    if echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); sys.exit(0 if d["modules_fail"]==0 and d["modules_ok"]>=30 else 1)' 2>/dev/null; then
        OK="$(echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d["modules_ok"])')"
        FSM="$(echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d["brainstem"]["fsm"])')"
        echo ">>> [5/5] Healthy: modules=$OK/0, FSM=$FSM"
        echo ""
        echo "Release $VERSION is live."
        echo "Rollback if needed:"
        echo "  sudo ln -sfn $PREV_TARGET $CURRENT_LINK"
        echo "  sudo systemctl restart lingtu robot-fastlio2 robot-localizer"
        exit 0
    fi
    sleep 3
done

echo ">>> [5/5] FAIL: health endpoint did not report a clean module set within 60s."
echo "    Inspect: journalctl -u lingtu.service -n 100"
echo "    Or roll back: sudo ln -sfn $PREV_TARGET $CURRENT_LINK"
exit 1
