#!/bin/bash
# ═══════════════════════════════════════════════════════════
# 感知管线 E2E 测试脚本
#
# 用法:
#   # 在 S100P 上运行 (需要先上传到 /opt/nav/tools/)
#   bash /opt/nav/tools/perception/test_perception_e2e.sh
#
#   # 或从 Windows 远程执行
#   ssh sunrise@192.168.66.190 'bash /opt/nav/tools/perception/test_perception_e2e.sh'
#
# 测试流程:
#   1. 下载 COCO 样本图片 (bus.jpg, zidane.jpg)
#   2. 启动 image_replay → /camera/ 话题
#   3. 启动 perception_node (BPU + TF fallback)
#   4. 监控 scene_graph 输出
#   5. 报告检测结果和性能数据
# ═══════════════════════════════════════════════════════════
set -e

NAV_DIR="${NAV_DIR:-/opt/nova/lingtu/v1.8.0}"
IMAGES_DIR="/tmp/test_images"
TIMEOUT=45

echo "═══════════════════════════════════════════════════════"
echo "  感知管线 E2E 测试"
echo "═══════════════════════════════════════════════════════"

# ── Step 0: Environment
source /opt/ros/humble/setup.bash
cd "$NAV_DIR"
source install/setup.bash 2>/dev/null || true

# ── Step 1: Test images
echo ""
echo "[1/4] 准备测试图片..."
mkdir -p "$IMAGES_DIR"
cd "$IMAGES_DIR"
for url_name in "https://ultralytics.com/images/bus.jpg:bus.jpg" \
                "https://ultralytics.com/images/zidane.jpg:zidane.jpg"; do
    url="${url_name%%:*}"
    name="${url_name##*:}"
    if [ ! -f "$name" ] || [ ! -s "$name" ]; then
        wget -q "$url" -O "$name" 2>/dev/null || echo "  WARN: $name 下载失败"
    fi
done
count=$(ls -1 *.jpg 2>/dev/null | wc -l)
echo "  图片: ${count} 张"

# ── Step 2: Stop conflicts
echo ""
echo "[2/4] 清理旧进程..."
pkill -f demo_scene_graph_publisher 2>/dev/null || true
pkill -f image_replay 2>/dev/null || true
pkill -f semantic_perception_node 2>/dev/null || true
sleep 2
echo "  清理完成"

# ── Step 3: Start pipeline
echo ""
echo "[3/4] 启动管线..."

# Image replay (5 Hz)
nohup python3 /opt/nav/tools/perception/image_replay.py --dir "$IMAGES_DIR" --fps 5 > /tmp/replay_test.log 2>&1 &
REPLAY_PID=$!
sleep 2
if ! kill -0 $REPLAY_PID 2>/dev/null; then
    echo "  FAIL: image_replay 启动失败"
    cat /tmp/replay_test.log
    exit 1
fi
echo "  image_replay: OK (PID=$REPLAY_PID)"

# Perception node (BPU + TF fallback)
cd "$NAV_DIR"
rm -f install/semantic_perception/lib/python3.10/site-packages/semantic_perception/__pycache__/perception_node.cpython-310.pyc
nohup ros2 run semantic_perception semantic_perception_node --ros-args \
    --params-file "$NAV_DIR/config/semantic_perception.yaml" \
    -p detector_type:=bpu \
    -p clip.enable:=false \
    -p tf_fallback_enable:=true \
    -r color_image:=/camera/color/image_raw \
    -r depth_image:=/camera/depth/image_raw \
    -r camera_info:=/camera/color/camera_info \
    -r odometry:=/nav/odometry \
    -r detections_3d:=/nav/semantic/detections_3d \
    -r scene_graph:=/nav/semantic/scene_graph \
    > /tmp/perception_test.log 2>&1 &
PERC_PID=$!
echo "  perception_node: 启动中 (PID=$PERC_PID)..."
sleep 12
if ! kill -0 $PERC_PID 2>/dev/null; then
    echo "  FAIL: perception_node 启动失败"
    tail -20 /tmp/perception_test.log
    kill $REPLAY_PID 2>/dev/null
    exit 1
fi

# Check BPU loaded
if grep -q "Detector.*loaded" /tmp/perception_test.log; then
    echo "  perception_node: BPU 加载成功"
else
    echo "  WARN: BPU 未确认加载"
fi

# Check TF fallback
if grep -q "static fallback" /tmp/perception_test.log; then
    echo "  TF: 降级模式 (无 SLAM/TF, 使用静态变换)"
else
    echo "  TF: 正常模式"
fi

# ── Step 4: Monitor scene graph
echo ""
echo "[4/4] 监控场景图 (${TIMEOUT}s)..."
timeout $TIMEOUT python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import json, time, statistics

class E2EMonitor(Node):
    def __init__(self):
        super().__init__("e2e_monitor")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(String, "/nav/semantic/scene_graph", self.on_sg, qos)
        self.count = 0
        self.all_labels = set()
        self.obj_counts = []
        self.intervals = []
        self.last_time = None
        self.real_objects = {}
        self.start = time.time()

    def on_sg(self, msg):
        now = time.time()
        self.count += 1
        if self.last_time:
            self.intervals.append(now - self.last_time)
        self.last_time = now

        try:
            sg = json.loads(msg.data)
            objects = sg.get("objects", [])
            self.obj_counts.append(len(objects))

            for obj in objects:
                label = obj.get("label", "?")
                det = obj.get("detection_count", 0)
                pos = obj.get("position", {})
                score = obj.get("score", 0)
                self.all_labels.add(label)

                if label not in self.real_objects or det > self.real_objects[label]["det"]:
                    px = pos.get("x", 0) if isinstance(pos, dict) else 0
                    py = pos.get("y", 0) if isinstance(pos, dict) else 0
                    pz = pos.get("z", 0) if isinstance(pos, dict) else 0
                    self.real_objects[label] = {
                        "det": det, "score": score,
                        "pos": f"({px:.1f}, {py:.1f}, {pz:.1f})"
                    }
        except:
            pass

rclpy.init()
node = E2EMonitor()
duration = 30
start = time.time()
while rclpy.ok() and (time.time() - start) < duration:
    rclpy.spin_once(node, timeout_sec=0.2)

elapsed = time.time() - start

print()
print("═" * 60)
print(f"  E2E 测试报告 ({elapsed:.1f}s)")
print("═" * 60)
print()

if node.count == 0:
    print("FAIL: 未收到场景图消息")
    print("  检查 perception_node 日志: tail -20 /tmp/perception_test.log")
else:
    rate = 1.0 / statistics.mean(node.intervals) if node.intervals else 0
    print(f"场景图统计:")
    print(f"  消息数: {node.count}")
    print(f"  发布率: {rate:.1f} Hz")
    print(f"  平均物体数: {statistics.mean(node.obj_counts):.1f}")
    print(f"  最大物体数: {max(node.obj_counts)}")
    print()

    print(f"检测到的物体类型:")
    passed = False
    for label, info in sorted(node.real_objects.items(), key=lambda x: -x[1]["det"]):
        status = "REAL" if info["det"] > 2 else "weak"
        print(f"  [{status:>4}] {label:20s}  detections={info['det']:>3}  "
              f"score={info['score']:.3f}  pos={info['pos']}")
        if info["det"] > 2:
            passed = True

    print()
    if passed:
        real_count = sum(1 for v in node.real_objects.values() if v["det"] > 2)
        print(f"RESULT: PASS — {real_count} 种真实检测物体, 管线运行正常")
    else:
        print(f"RESULT: FAIL — 无稳定检测 (所有物体 detections ≤ 2)")

print("═" * 60)

node.destroy_node()
rclpy.shutdown()
PYEOF

# Cleanup
echo ""
echo "清理测试进程..."
kill $REPLAY_PID 2>/dev/null || true
kill $PERC_PID 2>/dev/null || true
echo "完成"
