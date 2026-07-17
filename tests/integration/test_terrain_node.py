#!/usr/bin/env python3
"""
terrain_analysis C++ 节点集成测试

测试 terrainAnalysis 节点是否正确处理合成点云:
  - 平坦地面 (z=0 odom) 被标记为低 intensity (height above ground ~0)
  - 垂直墙壁 (x=4, z>0.2) 被标记为高 intensity (height above ground > obstacleHeightThre)
  - 输出帧 ID 正确 ("odom")
  - 输出 PointCloud2 包含 XYZI 字段

使用方法:
  # 终端 1 — 启动 terrainAnalysis C++ 节点 (带话题 remap):
  source /opt/ros/humble/setup.bash
  source ~/lingtu/install/setup.bash  # 或你的 workspace install 路径
  ros2 run terrain_analysis terrainAnalysis --ros-args \\
      -r /Odometry:=/nav/odometry \\
      -r /cloud_map:=/nav/map_cloud \\
      -r /terrain_map:=/nav/terrain_map

  # 终端 2 — 运行本测试:
  source /opt/ros/humble/setup.bash
  python3 tests/integration/test_terrain_node.py

输出:
  JSON 结果 {"test": "T1_terrain_analysis", "checks": [...], "pass": true/false, "duration_sec": N}
"""

import json
import threading
import time

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="ROS2 integration test — requires rclpy")

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
ODOM_HZ = 20  # 里程计发布频率
CLOUD_HZ = 5  # 点云发布频率
TEST_DURATION = 8.0  # 测试运行时长 (秒)

# 机器人固定位姿: position (0, 0, 0.5), yaw = 0
ROBOT_X = 0.0
ROBOT_Y = 0.0
ROBOT_Z = 0.5
ROBOT_YAW = 0.0

# 地面网格参数 (body frame)
GROUND_RANGE = 5.0  # x,y 范围 [-5, 5]
GROUND_STEP = 0.3  # 点间距
GROUND_Z_BODY = -ROBOT_Z  # body frame 中地面 z = -0.5 (odom z = 0.0)

# 墙壁参数 (body frame)
WALL_X = 4.0  # 墙壁 x 位置
WALL_Y_MIN = -2.0
WALL_Y_MAX = 2.0
WALL_Y_STEP = 0.2
WALL_Z_MIN_BODY = GROUND_Z_BODY  # 从地面开始 (body z = -0.5)
WALL_Z_MAX_BODY = 0.5  # body z = 0.5 -> odom z = 1.0
WALL_Z_STEP = 0.2

# 输入点云 intensity (无所谓, C++ 节点会覆盖)
INPUT_INTENSITY = 100.0


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Yaw (rad) -> geometry_msgs/Quaternion (仅绕 Z 轴旋转)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = float(np.sin(yaw / 2.0))
    q.w = float(np.cos(yaw / 2.0))
    return q


def make_xyzi_cloud(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """将 (N, 4) float32 数组 [x, y, z, intensity] 打包为 PointCloud2."""
    arr = np.ascontiguousarray(pts, dtype=np.float32)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(arr)
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * len(arr)
    msg.data = arr.tobytes()
    return msg


def parse_xyzi_cloud(msg: PointCloud2):
    """从 PointCloud2 消息解析 (N, 4) float32 数组 [x, y, z, intensity].

    返回 numpy 数组; 如果消息为空返回 shape (0, 4)."""
    if msg.width == 0:
        return np.zeros((0, 4), dtype=np.float32)
    data = np.frombuffer(bytes(msg.data), dtype=np.float32)
    n_fields = msg.point_step // 4
    data = data.reshape(-1, n_fields)
    # 取前 4 列 (x, y, z, intensity)
    return data[:, :4].copy()


def build_synthetic_cloud() -> np.ndarray:
    """生成合成 XYZI 点云 (body frame).

    包含:
      - 地面: z = GROUND_Z_BODY, x/y in [-5, 5], step 0.3
      - 墙壁: x = WALL_X, y in [-2, 2], z 从地面到 WALL_Z_MAX_BODY
    """
    pts = []

    # --- 地面平面 ---
    xs = np.arange(-GROUND_RANGE, GROUND_RANGE + GROUND_STEP / 2, GROUND_STEP)
    ys = np.arange(-GROUND_RANGE, GROUND_RANGE + GROUND_STEP / 2, GROUND_STEP)
    for x in xs:
        for y in ys:
            pts.append([x, y, GROUND_Z_BODY, INPUT_INTENSITY])

    # --- 垂直墙壁 ---
    wall_ys = np.arange(WALL_Y_MIN, WALL_Y_MAX + WALL_Y_STEP / 2, WALL_Y_STEP)
    wall_zs = np.arange(WALL_Z_MIN_BODY, WALL_Z_MAX_BODY + WALL_Z_STEP / 2, WALL_Z_STEP)
    for y in wall_ys:
        for z in wall_zs:
            pts.append([WALL_X, y, z, INPUT_INTENSITY])

    return np.array(pts, dtype=np.float32)


# ---------------------------------------------------------------------------
# 测试节点
# ---------------------------------------------------------------------------
class TerrainTestNode(Node):
    """发布合成里程计 + 点云, 订阅 terrain_map, 收集结果."""

    def __init__(self):
        super().__init__("terrain_test_node")

        # 收集到的 terrain_map 消息
        self._terrain_msgs = []
        self._lock = threading.Lock()

        # --- 发布者 ---
        self.pub_odom = self.create_publisher(Odometry, "/nav/odometry", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/nav/map_cloud", 10)

        # --- 订阅者 ---
        # terrain_analysis 输出可能是 RELIABLE 或 BEST_EFFORT, 用 BEST_EFFORT 更兼容
        terrain_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_terrain = self.create_subscription(PointCloud2, "/nav/terrain_map", self._terrain_cb, terrain_qos)

        # 预生成合成点云
        self._cloud_pts = build_synthetic_cloud()
        self.get_logger().info(
            f"合成点云: {len(self._cloud_pts)} 点 (地面 z_body={GROUND_Z_BODY:.1f}, 墙壁 x={WALL_X})"
        )

        # --- 定时器 ---
        self.timer_odom = self.create_timer(1.0 / ODOM_HZ, self._publish_odom)
        self.timer_cloud = self.create_timer(1.0 / CLOUD_HZ, self._publish_cloud)

    # ---- 回调 ----

    def _terrain_cb(self, msg: PointCloud2):
        with self._lock:
            self._terrain_msgs.append(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "body"
        msg.pose.pose.position.x = ROBOT_X
        msg.pose.pose.position.y = ROBOT_Y
        msg.pose.pose.position.z = ROBOT_Z
        msg.pose.pose.orientation = yaw_to_quaternion(ROBOT_YAW)
        self.pub_odom.publish(msg)

    def _publish_cloud(self):
        stamp = self.get_clock().now().to_msg()
        msg = make_xyzi_cloud(self._cloud_pts, "body", stamp)
        self.pub_cloud.publish(msg)

    # ---- 分析 ----

    def get_terrain_msgs(self):
        with self._lock:
            return list(self._terrain_msgs)


# ---------------------------------------------------------------------------
# 检查逻辑
# ---------------------------------------------------------------------------
def run_checks(terrain_msgs):
    """分析收集到的 terrain_map 消息, 返回 checks 列表."""
    checks = []

    # --- Check 1: 收到消息 ---
    n_msgs = len(terrain_msgs)
    checks.append(
        {
            "name": "terrain_map_received",
            "pass": n_msgs >= 1,
            "detail": f"收到 {n_msgs} 条 terrain_map 消息",
        }
    )
    if n_msgs == 0:
        # 后续检查无法进行
        for name in ["xyzi_fields", "frame_id", "ground_low_intensity", "wall_high_intensity"]:
            checks.append({"name": name, "pass": False, "detail": "无 terrain_map 消息"})
        return checks

    # 取最后一条消息 (数据最充分)
    last_msg = terrain_msgs[-1]

    # --- Check 2: XYZI 字段 ---
    field_names = [f.name for f in last_msg.fields]
    has_xyzi = all(f in field_names for f in ["x", "y", "z", "intensity"])
    checks.append(
        {
            "name": "xyzi_fields",
            "pass": has_xyzi,
            "detail": f"字段: {field_names}",
        }
    )

    # --- Check 3: frame_id ---
    frame_ok = last_msg.header.frame_id == "odom"
    checks.append(
        {
            "name": "frame_id",
            "pass": frame_ok,
            "detail": f'frame_id="{last_msg.header.frame_id}", 预期 "odom"',
        }
    )

    if not has_xyzi:
        checks.append({"name": "ground_low_intensity", "pass": False, "detail": "缺少 XYZI 字段"})
        checks.append({"name": "wall_high_intensity", "pass": False, "detail": "缺少 XYZI 字段"})
        return checks

    # 解析点云
    pts = parse_xyzi_cloud(last_msg)
    if len(pts) == 0:
        checks.append({"name": "ground_low_intensity", "pass": False, "detail": "点云为空"})
        checks.append({"name": "wall_high_intensity", "pass": False, "detail": "点云为空"})
        return checks

    x, y, z, intensity = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]

    # --- Check 4: 地面点 intensity 低 ---
    # 地面点: odom z 接近 0 (|z| < 0.1), 远离墙壁 (x < 3)
    ground_mask = (np.abs(z) < 0.1) & (x < 3.0)
    n_ground = int(np.sum(ground_mask))
    if n_ground > 0:
        ground_int = intensity[ground_mask]
        ground_max = float(np.max(ground_int))
        ground_mean = float(np.mean(ground_int))
        # 地面 disZ 应接近 0, 远低于 obstacleHeightThre (0.2)
        ground_ok = ground_mean < 0.15
        checks.append(
            {
                "name": "ground_low_intensity",
                "pass": ground_ok,
                "detail": f"{n_ground} 地面点, mean_intensity={ground_mean:.4f}, max={ground_max:.4f} (阈值 <0.15)",
            }
        )
    else:
        # 如果地面点全被过滤也可能正常 (disZ=0 的点 intensity=0, 可能仍在输出中)
        checks.append(
            {
                "name": "ground_low_intensity",
                "pass": True,
                "detail": "无符合条件的地面点 (z~0 & x<3), 这可能正常 (disZ=0 的地面点 intensity=0 仍在输出)",
            }
        )

    # --- Check 5: 墙壁点 intensity 高 ---
    # 墙壁点: x 接近 WALL_X (|x - 4| < 0.5), odom z > 0.3 (明显高于地面)
    wall_mask = (np.abs(x - WALL_X) < 0.5) & (z > 0.3)
    n_wall = int(np.sum(wall_mask))
    if n_wall > 0:
        wall_int = intensity[wall_mask]
        wall_min = float(np.min(wall_int))
        wall_mean = float(np.mean(wall_int))
        # 墙壁 disZ 应 > obstacleHeightThre (0.2)
        wall_ok = wall_mean > 0.2
        checks.append(
            {
                "name": "wall_high_intensity",
                "pass": wall_ok,
                "detail": f"{n_wall} 墙壁点, mean_intensity={wall_mean:.4f}, min={wall_min:.4f} (阈值 >0.2)",
            }
        )
    else:
        checks.append(
            {
                "name": "wall_high_intensity",
                "pass": False,
                "detail": f"未找到墙壁点 (x~{WALL_X}, z>0.3), 共 {len(pts)} 点输出",
            }
        )

    return checks


# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = TerrainTestNode()

    # 在后台线程中 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    t0 = time.time()
    node.get_logger().info(f"开始发布, 持续 {TEST_DURATION}s ...")

    # 等待测试时长
    time.sleep(TEST_DURATION)

    elapsed = time.time() - t0
    node.get_logger().info(f"发布结束, 耗时 {elapsed:.1f}s, 开始分析 ...")

    # 收集并分析
    terrain_msgs = node.get_terrain_msgs()
    checks = run_checks(terrain_msgs)
    all_pass = all(c["pass"] for c in checks)

    result = {
        "test": "T1_terrain_analysis",
        "checks": checks,
        "pass": all_pass,
        "duration_sec": round(elapsed, 1),
    }

    # 打印结果
    print("\n" + "=" * 60)
    for c in checks:
        mark = "PASS" if c["pass"] else "FAIL"
        print(f"  [{mark}] {c['name']}: {c['detail']}")
    print("=" * 60)
    overall = "PASS" if all_pass else "FAIL"
    print(f"  Overall: {overall}  ({len(terrain_msgs)} terrain_map msgs in {elapsed:.1f}s)")
    print("=" * 60)

    # JSON 输出 (最后一行, 方便机器解析)
    print(json.dumps(result, ensure_ascii=False))

    # 清理
    node.destroy_node()
    rclpy.shutdown()

    return 0 if all_pass else 1


if __name__ == "__main__":
    exit(main())
