#!/bin/bash
# test_factory_nova.sh 鈥?NOVA Dog (ONNX policy) + 涓夊眰宸ュ巶鍏ㄦ爤瀵艰埅娴嬭瘯
#
# 鏋舵瀯:
#   nova_nav_bridge.py (ONNX policy + MuJoCo physics) 鈫?
#   terrain_analysis 鈫?localPlanner 鈫?pathFollower 鈫?
#   global_planner (ele_planner.so C++ A*) 鈫?pct_path_adapter
#
# 鍓嶇疆: 鍏堣繍琛?gen_factory_nova_map.py 鐢熸垚鍦板浘
#   python3 gen_factory_nova_map.py
#
# 鐢ㄦ硶:
#   bash test_factory_nova.sh [goal_x] [goal_y] [goal_z] [monitor_sec] [viz]
#   bash test_factory_nova.sh 14 3 0.35 240        # 鏃犲ご妯″紡
#   bash test_factory_nova.sh 14 3 0.35 240 viz    # MuJoCo viewer 鍙鍖?
#   bash test_factory_nova.sh 22 10 0.35 240 viz   # 1F 绌垮唴澧欓棬娲?+ 鍙鍖?
#
# 娉ㄦ剰: goal_z=0.0 浼氳Е鍙?auto-terrain 璇垽鍒版渶楂樻ゼ灞?鈫?A* 澶辫触
#       1F 璇风敤 z=0.35, 2F 鐢?z=3.35, 3F 鐢?z=6.35
set -e

GOAL_X=${1:-14.0}
GOAL_Y=${2:-3.0}
GOAL_Z=${3:-0.35}
MONITOR_SEC=${4:-240}
VIZ=${5:-}       # "viz" 寮€鍚?MuJoCo viewer 鍙鍖?(DISPLAY=:0)

# 鈹€鈹€ 璺緞 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
SIM_DIR=${LINGTU_SIM_DIR:-${REPO_ROOT}/sim}
BRIDGE_SCRIPT=${LINGTU_NOVA_BRIDGE_SCRIPT:-${REPO_ROOT}/src/drivers/sim/nova_nav_bridge.py}
SCENE_XML=${LINGTU_NOVA_SCENE_XML:-${SIM_DIR}/robots/nova_dog/robot_with_camera.xml}
MAP_DIR=${LINGTU_SIM_MAP_DIR:-/tmp/sim_maps}
MAP_FILE=${LINGTU_FACTORY_MAP_FILE:-${MAP_DIR}/factory_nova}

# 鈹€鈹€ 妫€鏌ユ枃浠?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
if [ ! -f "${BRIDGE_SCRIPT}" ]; then
    echo "ERROR: nova_nav_bridge.py not found at ${BRIDGE_SCRIPT}"
    echo "Set LINGTU_NOVA_BRIDGE_SCRIPT or run from the LingTu repository root"
    exit 1
fi

if [ ! -f "${SCENE_XML}" ]; then
    echo "ERROR: Nova scene XML not found: ${SCENE_XML}"
    echo "Run: python3 ${SIM_DIR}/scripts/gen_factory_nova_map.py"
    exit 1
fi

if [ ! -f "${MAP_FILE}.pickle" ] && [ ! -f "${MAP_FILE}.pcd" ]; then
    echo "ERROR: factory map not found at ${MAP_FILE}.pickle or .pcd"
    echo "Run: python3 ${SIM_DIR}/scripts/gen_factory_nova_map.py"
    exit 1
fi

# 鈹€鈹€ 娓呯悊宸叉湁杩涚▼ 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "=== Cleaning up existing processes ==="
pkill -9 -f nova_nav_bridge 2>/dev/null || true
pkill -9 -f terrainAnalysis 2>/dev/null || true
pkill -9 -f localPlanner 2>/dev/null || true
pkill -9 -f pathFollower 2>/dev/null || true
pkill -9 -f pct_path_adapter 2>/dev/null || true
pkill -9 -f pct_planner_astar 2>/dev/null || true
pkill -9 -f global_planner 2>/dev/null || true
pkill -9 -f gp_compat 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash 2>/dev/null || true

echo "=== NOVA Dog Factory Navigation Test ==="
echo "  Scene:   ${SCENE_XML}"
echo "  Map:     ${MAP_FILE}"
echo "  Start:   (2.0, 2.0, 0.35)"
echo "  Goal:    ($GOAL_X, $GOAL_Y, $GOAL_Z)"
echo "  Monitor: ${MONITOR_SEC}s"
echo "  Mode:    ${VIZ:-headless}"
echo ""

# 鈹€鈹€ [1] nova_nav_bridge.py 鈥?ONNX policy + MuJoCo physics 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[1/6] Starting nova_nav_bridge (ONNX + factory scene) ..."
if [ "${VIZ}" = "viz" ]; then
    # 鍙鍖栨ā寮? DISPLAY=:0, MuJoCo viewer 绐楀彛
    DISPLAY=:0 python3 ${BRIDGE_SCRIPT} \
        --scene ${SCENE_XML} \
        --start 2.0 2.0 0.35 \
        > /tmp/nova_bridge.log 2>&1 &
else
    # 鏃犲ご妯″紡: EGL 杞欢娓叉煋
    MUJOCO_GL=egl python3 ${BRIDGE_SCRIPT} \
        --headless \
        --scene ${SCENE_XML} \
        --start 2.0 2.0 0.35 \
        > /tmp/nova_bridge.log 2>&1 &
fi
BRIDGE_PID=$!
echo "  PID=$BRIDGE_PID"
sleep 5

if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "ERROR: nova_nav_bridge died!"
    tail -30 /tmp/nova_bridge.log
    exit 1
fi
echo "  Bridge alive, height check:"
tail -3 /tmp/nova_bridge.log

# 鈹€鈹€ [1.5] 闈欐€?TF: map 鈫?odom (identity) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
# bridge 鍙戝竷 odom鈫抌ody, global_planner 闇€瑕?map鈫抌ody 閾捐矾
echo "  Publishing static TF: map 鈫?odom (identity) ..."
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 \
    --yaw 0.0 --pitch 0.0 --roll 0.0 \
    --frame-id map --child-frame-id odom \
    > /tmp/static_tf.log 2>&1 &
TF_PID=$!
sleep 1

# 鈹€鈹€ [2] terrain_analysis 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo ""
echo "[2/6] Starting terrain_analysis ..."
ros2 run terrain_analysis terrainAnalysis \
    --ros-args \
    -r /Odometry:=/slam/odometry \
    -r /cloud_map:=/slam/map_cloud \
    -r /terrain_map:=/nav/terrain_map \
    -p scanVoxelSize:=0.1 \
    -p decayTime:=5.0 \
    -p noDecayDis:=3.0 \
    -p clearingDis:=15.0 \
    > /tmp/terrain_analysis.log 2>&1 &
TA_PID=$!
sleep 2

# 鈹€鈹€ [3] global_planner 鈥?鐢熶骇鑺傜偣 (ele_planner.so C++ A*) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
# 涓や釜淇:
# 1. VENV_SITE 浼樺厛浜?ROS2 璺緞 鈫?纭繚 numpy 1.26.4 (涓嶈 ~/.local/2.2.6 瑕嗙洊)
# 2. gp_adapters.py 鎻愪緵 numpy._core 鍒悕 鈫?鍙姞杞?numpy 2.x 鏍煎紡鐨?pickle
echo "[3/6] Starting global_planner (ele_planner.so C++ A*) ..."
PCT_SCRIPTS=/home/sunrise/data/SLAM/navigation/install/pct_planner/share/pct_planner/planner/scripts
VENV_PY=/tmp/venv_np1/bin/python3
VENV_SITE=$(${VENV_PY} -c "import sysconfig; print(sysconfig.get_path('purelib'))")
ROS_PYPATH=$(python3 -c "import sys; print(':'.join(p for p in sys.path if p))")
COMBINED="${VENV_SITE}:${ROS_PYPATH}"

(cd ${PCT_SCRIPTS} && PYTHONPATH="${COMBINED}" ${VENV_PY} /tmp/gp_adapters.py \
    --ros-args \
    -r /goal_pose:=/nav/goal_pose \
    -r /pct_path:=/nav/global_path \
    -r /pct_planner/status:=/nav/planner_status \
    -p map_file:="${MAP_FILE}.pickle" \
    -p map_frame:=map \
    -p robot_frame:=body \
    -p obstacle_thr:=50 \
    -p tomogram_ground_h:=0.0 \
    > /tmp/global_planner.log 2>&1) &
GP_PID=$!
sleep 6

if ! kill -0 $GP_PID 2>/dev/null; then
    echo "ERROR: global_planner died!"
    tail -20 /tmp/global_planner.log
    kill $BRIDGE_PID $TA_PID $TF_PID 2>/dev/null
    exit 1
fi
echo "  global_planner started (PID=$GP_PID)"

# 鈹€鈹€ [4] pct_path_adapter 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[4/6] Starting pct_path_adapter ..."
ros2 run pct_adapters pct_path_adapter \
    --ros-args \
    -r /pct_path:=/nav/global_path \
    -r /Odometry:=/slam/odometry \
    -r /planner_waypoint:=/nav/way_point \
    -p waypoint_distance:=1.5 \
    -p arrival_threshold:=0.8 \
    -p stuck_timeout_sec:=30.0 \
    -p max_replan_count:=2 \
    > /tmp/pct_adapter.log 2>&1 &
PA_PID=$!
sleep 1

# 鈹€鈹€ [5] localPlanner 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[5/6] Starting localPlanner ..."
PATHS_DIR=/home/sunrise/data/SLAM/navigation/install/local_planner/share/local_planner/paths
ros2 run local_planner localPlanner \
    --ros-args \
    -r /Odometry:=/slam/odometry \
    -r /cloud_map:=/slam/map_cloud \
    -r /terrain_map:=/nav/terrain_map \
    -r /way_point:=/nav/way_point \
    -p pathFolder:="$PATHS_DIR" \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p useTerrainAnalysis:=true \
    -p checkObstacle:=false \
    -p useCost:=true \
    -p slopeWeight:=3.0 \
    -p twoWayDrive:=false \
    > /tmp/local_planner.log 2>&1 &
LP_PID=$!
sleep 1

# 鈹€鈹€ [6] pathFollower 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[6/6] Starting pathFollower ..."
ros2 run local_planner pathFollower \
    --ros-args \
    -r /Odometry:=/slam/odometry \
    -r /cmd_vel:=/nav/cmd_vel \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p stuck_timeout:=15.0 \
    -p stuck_dist_thre:=0.15 \
    > /tmp/path_follower.log 2>&1 &
PF_PID=$!
sleep 3

echo ""
echo "All 6 nodes started. Waiting 5s for terrain stabilization ..."
sleep 5

# 鈹€鈹€ 鍙戝竷鐩爣鐐?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "=== Sending goal_pose: ($GOAL_X, $GOAL_Y, $GOAL_Z) ==="
ros2 topic pub /nav/goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {w: 1.0}}}" \
    --once > /dev/null 2>&1

sleep 2

echo ""
echo "=== Global Planner Initial Status ==="
tail -5 /tmp/global_planner.log
echo ""

# 鈹€鈹€ 鐩戞帶 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "=== Monitoring for ${MONITOR_SEC}s (checking every 2s) ==="
for i in $(seq 2 2 $MONITOR_SEC); do
    sleep 2

    # nova bridge 鏃ュ織: 瀵绘壘 h= 琛?
    POS_LINE=$(tail -5 /tmp/nova_bridge.log 2>/dev/null | grep -oP 't=[\d.]+s\s+h=[\d.]+m\s+cmd_vx=[\d.+-]+' | tail -1)
    ADAPTER_EVENT=$(tail -3 /tmp/pct_adapter.log 2>/dev/null | grep -oP '(Reached Waypoint \d+|Goal Reached|Replan L\d|FAILED|planning failed)' | tail -1)

    if [ -n "$POS_LINE" ]; then
        printf "  t=%3ds | %s | adapter: %s\n" "$i" "$POS_LINE" "$ADAPTER_EVENT"
    fi

    # 妫€鏌?goal_reached (鍖归厤 "Goal Reached!" 鏍煎紡)
    if grep -qi "goal reached" /tmp/pct_adapter.log 2>/dev/null; then
        echo ""
        echo "*** GOAL REACHED! ***"
        FINAL_POS=$(tail -5 /tmp/nova_bridge.log 2>/dev/null | grep -oP 't=[\d.]+s\s+h=[\d.]+m' | tail -1)
        echo "  Final: $FINAL_POS"
        break
    fi

    # 妫€鏌?bridge 鏄惁瀛樻椿
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        echo "ERROR: nova_nav_bridge died at t=${i}s!"
        tail -10 /tmp/nova_bridge.log
        break
    fi
done

# 鈹€鈹€ 鏈€缁堢姸鎬?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo ""
echo "=== Final Status ==="
echo "-- nova_bridge --"
tail -5 /tmp/nova_bridge.log
echo "-- global_planner --"
tail -3 /tmp/global_planner.log
echo "-- pct_adapter --"
tail -5 /tmp/pct_adapter.log
echo "-- local_planner --"
tail -3 /tmp/local_planner.log
echo "-- path_follower --"
tail -3 /tmp/path_follower.log
echo ""
echo "PIDs: BRIDGE=$BRIDGE_PID TF=$TF_PID TA=$TA_PID GP=$GP_PID PA=$PA_PID LP=$LP_PID PF=$PF_PID"
echo ""
echo "Logs: /tmp/nova_bridge.log /tmp/terrain_analysis.log /tmp/global_planner.log"
echo "      /tmp/pct_adapter.log /tmp/local_planner.log /tmp/path_follower.log"
