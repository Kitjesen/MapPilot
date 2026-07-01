#!/bin/bash
# test_semantic_nav.sh 鈥?NOVA Dog + 璇箟瀵艰埅鍏ㄩ摼璺祴璇?
#
# 鏋舵瀯:
#   factory_stub_test.py (stub鍦烘櫙鍥?+ 鑷劧璇█鎸囦护) 鈫?
#   semantic_planner_node (Fast-Slow鍙岃繘绋嬬洰鏍囪В鏋? 鈫?
#   global_planner (PCT A*) 鈫?pct_path_adapter 鈫?
#   localPlanner 鈫?pathFollower 鈫?nova_nav_bridge (MuJoCo)
#
# 鐢ㄦ硶:
#   bash test_semantic_nav.sh [instruction] [timeout_sec] [viz]
#   bash test_semantic_nav.sh "瀵艰埅鍒扮洰鏍囧尯鍩? 120        # 鏃犲ご妯″紡
#   bash test_semantic_nav.sh "瀵艰埅鍒版帶鍒跺"   120 viz    # MuJoCo viewer
#
# 娉ㄦ剰: 鎵€鏈?Z 鍧愭爣鍦ㄥ満鏅浘涓凡璁句负 0.35 (涓€妤?
set -e

INSTRUCTION=${1:-"瀵艰埅鍒扮洰鏍囧尯鍩?}
MONITOR_SEC=${2:-240}
VIZ=${3:-}

# 鈹€鈹€ 璺緞 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
SIM_DIR=${LINGTU_SIM_DIR:-${REPO_ROOT}/sim}
BRIDGE_SCRIPT=${LINGTU_NOVA_BRIDGE_SCRIPT:-${REPO_ROOT}/src/drivers/sim/nova_nav_bridge.py}
SCENE_XML=${LINGTU_NOVA_SCENE_XML:-${SIM_DIR}/robots/nova_dog/robot_with_camera.xml}
MAP_DIR=${LINGTU_SIM_MAP_DIR:-/tmp/sim_maps}
MAP_FILE=${LINGTU_FACTORY_MAP_FILE:-${MAP_DIR}/factory_nova}
SEMANTIC_STUB=${LINGTU_FACTORY_SEMANTIC_STUB:-${SIM_DIR}/semantic/factory_stub_test.py}

# 鈹€鈹€ 妫€鏌ユ枃浠?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
for f in "${BRIDGE_SCRIPT}" "${SCENE_XML}"; do
    if [ ! -f "$f" ]; then
        echo "ERROR: not found: $f"
        exit 1
    fi
done

if [ ! -f "${MAP_FILE}.pickle" ] && [ ! -f "${MAP_FILE}.pcd" ]; then
    echo "ERROR: factory map not found at ${MAP_FILE}.pickle"
    exit 1
fi

if [ ! -f "${SEMANTIC_STUB}" ]; then
    echo "ERROR: factory_stub_test.py not found at ${SEMANTIC_STUB}"
    exit 1
fi

# 鈹€鈹€ 娓呯悊 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "=== Cleaning up existing processes ==="
pkill -9 -f nova_nav_bridge 2>/dev/null || true
pkill -9 -f factory_stub_test 2>/dev/null || true
pkill -9 -f semantic_planner_node 2>/dev/null || true
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

echo "=== Semantic Navigation Test ==="
echo "  Scene:       ${SCENE_XML}"
echo "  Map:         ${MAP_FILE}"
echo "  Instruction: ${INSTRUCTION}"
echo "  Monitor:     ${MONITOR_SEC}s"
echo "  Mode:        ${VIZ:-headless}"
echo ""

# 鈹€鈹€ [1] nova_nav_bridge 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[1/7] Starting nova_nav_bridge ..."
if [ "${VIZ}" = "viz" ]; then
    DISPLAY=:0 python3 ${BRIDGE_SCRIPT} \
        --scene ${SCENE_XML} \
        --start 2.0 2.0 0.35 \
        > /tmp/nova_bridge.log 2>&1 &
else
    MUJOCO_GL=egl python3 ${BRIDGE_SCRIPT} \
        --headless \
        --scene ${SCENE_XML} \
        --start 2.0 2.0 0.35 \
        > /tmp/nova_bridge.log 2>&1 &
fi
BRIDGE_PID=$!
sleep 5

if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "ERROR: nova_nav_bridge died!"; tail -20 /tmp/nova_bridge.log; exit 1
fi

# 鈹€鈹€ [1.5] Static TF 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 --yaw 0.0 --pitch 0.0 --roll 0.0 \
    --frame-id map --child-frame-id odom \
    > /tmp/static_tf.log 2>&1 &
TF_PID=$!
sleep 1

# 鈹€鈹€ [2] terrain_analysis 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[2/7] Starting terrain_analysis ..."
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

# 鈹€鈹€ [3] global_planner (Python A*) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[3/7] Starting global_planner (PCT A*) ..."
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
    echo "ERROR: global_planner died!"; tail -20 /tmp/global_planner.log
    kill $BRIDGE_PID $TA_PID $TF_PID 2>/dev/null; exit 1
fi

# 鈹€鈹€ [4] pct_path_adapter 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[4/7] Starting pct_path_adapter ..."
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
echo "[5/7] Starting localPlanner ..."
PATHS_DIR=/home/sunrise/data/SLAM/navigation/install/local_planner/share/local_planner/paths
ros2 run local_planner localPlanner \
    --ros-args \
    -r /Odometry:=/slam/odometry \
    -r /cloud_map:=/slam/map_cloud \
    -r /terrain_map:=/nav/terrain_map \
    -r /way_point:=/nav/way_point \
    -p pathFolder:="${PATHS_DIR}" \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p useTerrainAnalysis:=true \
    -p checkObstacle:=false \
    -p useCost:=true \
    -p slopeWeight:=3.0 \
    -p twoWayDrive:=false \
    > /tmp/local_planner.log 2>&1 &
LP_PID=$!

# 鈹€鈹€ [6] pathFollower 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[6/7] Starting pathFollower ..."
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

# 鈹€鈹€ [7] semantic_planner_node 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "[7/7] Starting semantic_planner_node ..."
ros2 run semantic_planner semantic_planner_node \
    --ros-args \
    -r instruction:=/nav/semantic/instruction \
    -r scene_graph:=/nav/semantic/scene_graph \
    -r odometry:=/slam/odometry \
    -r resolved_goal:=/nav/goal_pose \
    -r status:=/nav/semantic/status \
    -r cmd_vel:=/nav/semantic/cmd_vel \
    -p llm.backend:=kimi \
    -p llm_fallback.backend:=openai \
    -p exploration.enable:=false \
    -p goal_resolution.replan_on_failure:=false \
    > /tmp/decision.log 2>&1 &
SP_PID=$!
sleep 3

if ! kill -0 $SP_PID 2>/dev/null; then
    echo "ERROR: semantic_planner_node died!"
    tail -20 /tmp/decision.log
    kill $BRIDGE_PID $TA_PID $TF_PID $GP_PID $PA_PID $LP_PID $PF_PID 2>/dev/null
    exit 1
fi
echo "  semantic_planner_node started (PID=$SP_PID)"

echo ""
echo "All 7 nodes started. Waiting 5s for terrain stabilization ..."
sleep 5

# 鈹€鈹€ 鍚姩 stub 娴嬭瘯鑺傜偣 (鍦烘櫙鍥?+ 鎸囦护) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo "=== Starting factory_stub_test (instruction: '${INSTRUCTION}') ==="
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash 2>/dev/null || true

python3 ${SEMANTIC_STUB} \
    --instruction "${INSTRUCTION}" \
    --timeout ${MONITOR_SEC} \
    > /tmp/semantic_stub.log 2>&1
STUB_RC=$?
cat /tmp/semantic_stub.log

# 鈹€鈹€ 鏈€缁堢姸鎬?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
echo ""
echo "=== Final Status ==="
echo "-- nova_bridge (last 3) --"
tail -3 /tmp/nova_bridge.log
echo "-- global_planner (last 3) --"
tail -3 /tmp/global_planner.log
echo "-- pct_adapter (last 5) --"
tail -5 /tmp/pct_adapter.log
echo "-- semantic_planner (last 5) --"
tail -5 /tmp/decision.log
echo ""
echo "PIDs: BRIDGE=$BRIDGE_PID TF=$TF_PID TA=$TA_PID GP=$GP_PID"
echo "      PA=$PA_PID LP=$LP_PID PF=$PF_PID SP=$SP_PID"
echo ""

if [ $STUB_RC -eq 0 ]; then
    echo "*** SEMANTIC NAVIGATION: PASS ***"
else
    echo "*** SEMANTIC NAVIGATION: FAIL (rc=$STUB_RC) ***"
fi

exit $STUB_RC
