#!/usr/bin/env python3
"""
SLAM 数据集鲁棒性测试 — 通过 paramiko SSH 在机器人上执行
Robot: 192.168.66.190, user: sunrise

v2: 使用 nohup 后台执行，轮询结果文件
"""
import os
import sys
import time

import paramiko

ROBOT_IP = "192.168.66.190"
ROBOT_USER = "sunrise"
ROBOT_PASS = os.environ.get("S100P_PASSWORD", "")

SCRIPT_CONTENT = r'''#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash

NAV_DIR=/home/sunrise/data/SLAM/navigation
BAG_DIR=/home/sunrise/data/slam_datasets/legkilo_corridor
RESULT_FILE="/tmp/slam_test_result.txt"

# Redirect all output to result file
exec > "$RESULT_FILE" 2>&1

echo "============================================================"
echo "  SLAM 数据集鲁棒性测试 — Point-LIO + Leg-KILO 走廊"
echo "  数据集: Unitree Go1 四足 + VLP-16 LiDAR (走廊, 四足振动)"
echo "  SLAM:   Point-LIO (velody16 配置, scan_line=16)"
echo "  时间:   $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"
echo ""

# 清理残留
echo "[准备] 清理残留进程..."
pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
pkill -f "ros2 bag play" 2>/dev/null || true
pkill -f nova_nav_bridge 2>/dev/null || true
sleep 2

# 生成 VLP-16 配置
BASE_CONFIG="$NAV_DIR/install/pointlio/share/pointlio/config/velody16.yaml"
CONFIG_FILE="/tmp/vlp16_legkilo.yaml"
sed -e 's/scan_line: 32/scan_line: 16/' \
    -e 's|lid_topic:.*|lid_topic: "/lidar/raw_frame"|' \
    -e 's|imu_topic:.*|imu_topic: "/imu/raw"|' \
    "$BASE_CONFIG" > "$CONFIG_FILE"
echo "[INFO] VLP-16 配置: $CONFIG_FILE"

# 启动 Point-LIO
echo ""
echo "[1/3] 启动 Point-LIO..."
ros2 run pointlio pointlio_node --ros-args \
    --params-file "$CONFIG_FILE" \
    -r cloud_registered_body:=/test/registered_cloud \
    -r cloud_registered:=/test/map_cloud \
    -r aft_mapped_to_init:=/test/odometry \
    > /tmp/slam_dataset_test.log 2>&1 &
SLAM_PID=$!
sleep 5

if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "[ERROR] Point-LIO 启动失败!"
    cat /tmp/slam_dataset_test.log
    echo "SLAM_TEST_DONE"
    exit 1
fi
echo "  Point-LIO PID: $SLAM_PID — 运行中"

# 播放数据集
echo ""
echo "[2/3] 播放 Leg-KILO 走廊数据集 (445s, rate=1.0)..."
ros2 bag play "$BAG_DIR" --rate 1.0 &
PLAY_PID=$!
sleep 3

# 等待初始化
echo ""
echo "等待 SLAM 初始化..."
INIT_OK=false
for i in $(seq 1 30); do
    ODOM=$(timeout 2 ros2 topic echo /test/odometry --once --no-arr 2>/dev/null | head -3)
    if [ -n "$ODOM" ]; then
        INIT_OK=true
        echo "  SLAM 初始化完成 (${i}s)"
        break
    fi
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        echo "[CRASH] SLAM 在初始化阶段崩溃!"
        tail -20 /tmp/slam_dataset_test.log
        kill $PLAY_PID 2>/dev/null
        echo "SLAM_TEST_DONE"
        exit 1
    fi
    sleep 1
done

if [ "$INIT_OK" = false ]; then
    echo "[WARN] SLAM 初始化超时 (30s)"
fi

# 监控
echo ""
echo "[3/3] 监控 SLAM 输出..."
echo ""
echo " SEC | POS_X    | POS_Y    | POS_Z    | DRIFT    | STATUS"
echo "-----|----------|----------|----------|----------|--------"

TRAJECTORY_FILE="/tmp/slam_dataset_trajectory.csv"
echo "t,x,y,z" > $TRAJECTORY_FILE

CRASH=false
SAMPLE_INTERVAL=5
MAX_SAMPLES=100

INIT_X=""
INIT_Y=""
INIT_Z=""
MAX_DRIFT=0
TOTAL_SAMPLES=0
OK_SAMPLES=0

for i in $(seq 1 $MAX_SAMPLES); do
    sleep $SAMPLE_INTERVAL

    if ! kill -0 $SLAM_PID 2>/dev/null; then
        CRASH=true
        SEC=$((i * SAMPLE_INTERVAL))
        echo "[CRASH] SLAM 在 ${SEC}s 处崩溃!"
        break
    fi

    if ! kill -0 $PLAY_PID 2>/dev/null; then
        SEC=$((i * SAMPLE_INTERVAL))
        echo ""
        echo "[DONE] 数据集播放完毕 (${SEC}s)"
        break
    fi

    TOTAL_SAMPLES=$((TOTAL_SAMPLES + 1))

    ODOM_MSG=$(timeout 2 ros2 topic echo /test/odometry --once 2>/dev/null || true)
    if [ -z "$ODOM_MSG" ]; then
        SEC=$((i * SAMPLE_INTERVAL))
        printf "%4ds | ---      | ---      | ---      | ---      | NO_DATA\n" "$SEC"
        continue
    fi

    OK_SAMPLES=$((OK_SAMPLES + 1))

    POS_X=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    POS_Y=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    POS_Z=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')

    SEC=$((i * SAMPLE_INTERVAL))

    if [ -z "$INIT_X" ]; then
        INIT_X="${POS_X:-0}"
        INIT_Y="${POS_Y:-0}"
        INIT_Z="${POS_Z:-0}"
    fi

    DRIFT=$(echo "$POS_X $POS_Y $POS_Z $INIT_X $INIT_Y $INIT_Z" | awk '{
        dx=$1-$4; dy=$2-$5; dz=$3-$6;
        printf "%.3f", sqrt(dx*dx+dy*dy+dz*dz)
    }')
    MAX_DRIFT=$(echo "$DRIFT $MAX_DRIFT" | awk '{if($1>$2) print $1; else print $2}')

    printf "%4ds | %8.3f | %8.3f | %8.3f | %8.3fm | OK\n" \
        "$SEC" "${POS_X:-0}" "${POS_Y:-0}" "${POS_Z:-0}" "$DRIFT"

    echo "$SEC,${POS_X:-0},${POS_Y:-0},${POS_Z:-0}" >> $TRAJECTORY_FILE
done

# 频率测量
echo ""
echo "测量话题频率 (5s 采样)..."
ODOM_HZ=$(timeout 6 ros2 topic hz /test/odometry 2>&1 | grep "average rate:" | tail -1 | awk '{print $3}' || echo "?")
echo "  里程计频率: ${ODOM_HZ} Hz"

# 结果汇总
echo ""
echo "============================================================"
echo "  SLAM 数据集鲁棒性测试 — 最终结果"
echo "============================================================"

if [ "$CRASH" = true ]; then
    echo "  结果:       SLAM CRASHED"
    echo ""
    echo "  最后 20 行日志:"
    tail -20 /tmp/slam_dataset_test.log
    RESULT="FAIL"
else
    echo "  结果:       SLAM SURVIVED"
    FINAL_MSG=$(timeout 2 ros2 topic echo /test/odometry --once 2>/dev/null || true)
    FX=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    FY=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    FZ=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')
    echo "  最终位置:   (${FX:-?}, ${FY:-?}, ${FZ:-?})"
    RESULT="PASS"
fi

echo ""
echo "  数据集:     Leg-KILO 走廊 (Unitree Go1 四足, VLP-16)"
echo "  SLAM 算法:  Point-LIO (lidar_type=2, scan_line=16)"
echo "  数据集时长: ~445s (7.4 分钟)"
echo "  最大位移:   ${MAX_DRIFT}m"
echo "  里程计频率: ${ODOM_HZ} Hz"
echo "  采样成功率: ${OK_SAMPLES}/${TOTAL_SAMPLES}"
echo "  轨迹文件:   $TRAJECTORY_FILE"
echo ""
echo "  结论: Point-LIO + 四足振动数据集 → ${RESULT}"
echo "============================================================"
echo ""

# 输出完整轨迹 CSV
echo "=== 完整轨迹数据 ==="
cat $TRAJECTORY_FILE

# SLAM 日志
echo ""
echo "=== SLAM 日志 ==="
cat /tmp/slam_dataset_test.log

# 清理
kill $SLAM_PID $PLAY_PID 2>/dev/null || true
wait $SLAM_PID $PLAY_PID 2>/dev/null || true
echo ""
echo "测试完成."
echo "SLAM_TEST_DONE"
'''


def ssh_exec(ssh, cmd, timeout=30):
    """Execute command and return stdout."""
    _, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace")


def main():
    if not ROBOT_PASS:
        raise SystemExit("S100P_PASSWORD is required")
    print(f"[LOCAL] 连接机器人 {ROBOT_IP}...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ROBOT_IP, username=ROBOT_USER, password=ROBOT_PASS, timeout=10)
    print("[LOCAL] SSH 连接成功")

    # Step 1: Upload script
    print("[LOCAL] 上传测试脚本...")
    sftp = ssh.open_sftp()
    with sftp.file("/tmp/slam_full_test.sh", "w") as f:
        f.write(SCRIPT_CONTENT)
    sftp.close()

    # Step 2: chmod + clean old result
    ssh_exec(ssh, "chmod +x /tmp/slam_full_test.sh && rm -f /tmp/slam_test_result.txt")
    time.sleep(1)
    print("[LOCAL] 脚本准备完成")

    # Step 3: Kill any previous test
    ssh_exec(ssh, "pkill -f slam_full_test 2>/dev/null; true")
    time.sleep(1)

    # Step 4: Launch in background with nohup
    print("[LOCAL] 后台启动测试脚本...")
    ssh.exec_command("nohup bash /tmp/slam_full_test.sh &")
    time.sleep(3)

    # Step 5: Poll for completion
    print("[LOCAL] 等待测试完成 (数据集 445s + 初始化/清理)...")
    print("[LOCAL] 每 15s 检查一次进度...")
    print("")

    last_lines = 0
    max_wait = 600  # 10 minutes max
    elapsed = 0

    while elapsed < max_wait:
        time.sleep(15)
        elapsed += 15

        # Check if result file exists and has content
        result = ssh_exec(ssh, "cat /tmp/slam_test_result.txt 2>/dev/null || echo '__WAITING__'", timeout=10)

        if "__WAITING__" in result:
            print(f"[LOCAL] {elapsed}s — 脚本尚未产生输出...")
            continue

        lines = result.strip().split("\n")
        current_lines = len(lines)

        # Print new lines
        if current_lines > last_lines:
            for line in lines[last_lines:]:
                print(line)
            last_lines = current_lines

        # Check if done
        if "SLAM_TEST_DONE" in result:
            print(f"\n[LOCAL] 测试完成! 总耗时约 {elapsed}s")
            break
    else:
        print(f"\n[LOCAL] 等待超时 ({max_wait}s)，获取当前结果...")
        result = ssh_exec(ssh, "cat /tmp/slam_test_result.txt 2>/dev/null || echo 'NO RESULT'", timeout=10)
        print(result)

    ssh.close()
    print("[LOCAL] SSH 连接关闭")


if __name__ == "__main__":
    main()
