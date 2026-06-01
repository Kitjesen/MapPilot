#!/usr/bin/env python3
"""
DEPRECATED — use sim/engine/bridge/ros2_bridge.py (SimROS2Bridge) instead.
This file is kept for backward compatibility with legacy scripts.
The unified bridge is used by sim/engine/cli.py.

MuJoCo + ROS2 最小闭环 Bridge

验证通过的完整闭环:
  MuJoCo physics → mj_multiRay LiDAR → terrain_analysis → localPlanner
  → pathFollower → cmd_vel → MuJoCo

发布:
  /nav/odometry          → 50 Hz  (nav_msgs/Odometry, odom frame)
  /nav/registered_cloud  → 10 Hz  (PointCloud2 XYZI, body frame)
  /livox/lidar           → 10 Hz  (同上, terrain_analysis 订阅)
  TF: map → odom → body

订阅:
  /nav/cmd_vel  (TwistStamped) → qvel 注入

平台: S100P (aarch64), MuJoCo 3.5.0, ROS2 Humble

用法:
    python3 src/drivers/sim/mujoco_ros2_bridge.py          # 默认内联场景
    python3 src/drivers/sim/mujoco_ros2_bridge.py scene.xml # 自定义场景
"""
import numpy as np
import time
import sys

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

from core.runtime_interface import FRAMES, TOPICS

# ── 内联 XML: 测试场景 ─────────────────────────────────────────────
DEFAULT_XML = """
<mujoco model="sim_bridge_test">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <!-- 环境: group=1 (LiDAR 只检测此组) -->
    <geom name="floor" type="plane" size="50 50 0.1" group="1" conaffinity="1" condim="3"
          rgba=".85 .85 .82 1"/>
    <geom name="box1"  type="box" size="1 0.5 0.8" pos="5 0 0.8" group="1" conaffinity="1"
          rgba=".75 .3 .25 1"/>
    <geom name="box2"  type="box" size="0.5 1.8 1.2" pos="9 4 1.2" group="1" conaffinity="1"
          rgba=".3 .75 .3 1"/>
    <geom name="cyl1"  type="cylinder" size="0.4 1.0" pos="4 -5 1.0" group="1" conaffinity="1"
          rgba=".8 .7 .2 1"/>
    <geom name="wall1" type="box" size="0.2 8.0 2.0" pos="-8 0 2.0" group="1" conaffinity="1"
          rgba=".6 .6 .6 1"/>
    <geom name="wall2" type="box" size="8.0 0.2 2.0" pos="0 -8 2.0" group="1" conaffinity="1"
          rgba=".6 .6 .6 1"/>
    <!-- 坡道 -->
    <geom name="ramp" type="box" size="3 1.5 0.05" pos="0 5 0.4" euler="0 0.15 0" group="1"
          conaffinity="1" rgba=".6 .5 .3 1"/>

    <!-- 机器人: group=0 (LiDAR 不检测) -->
    <body name="robot" pos="0 0 0.45">
      <freejoint name="root"/>
      <geom name="chassis" type="box" size="0.4 0.25 0.1" group="0" mass="20" conaffinity="1"
            rgba=".18 .38 .76 1"/>
      <geom name="leg_fl" type="capsule" size="0.035" fromto="0.28 0.18 -0.10 0.28 0.18 -0.42" group="0" mass="0.8"/>
      <geom name="leg_fr" type="capsule" size="0.035" fromto="0.28 -0.18 -0.10 0.28 -0.18 -0.42" group="0" mass="0.8"/>
      <geom name="leg_rl" type="capsule" size="0.035" fromto="-.28  0.18 -0.10 -.28  0.18 -0.42" group="0" mass="0.8"/>
      <geom name="leg_rr" type="capsule" size="0.035" fromto="-.28 -0.18 -0.10 -.28 -0.18 -0.42" group="0" mass="0.8"/>
      <body name="lidar" pos="0 0 0.25">
        <geom name="lidar_vis" type="sphere" size="0.05" group="0" mass="0.1" conaffinity="0"
              rgba="0 .78 .85 1"/>
      </body>
      <inertial pos="0 0 0" mass="22" diaginertia="0.75 1.15 1.15"/>
    </body>
  </worldbody>
</mujoco>
"""

# ── LiDAR 配置 (Livox MID-360 模式) ──────────────────────────────
N_RAYS = 6400
VFOV_MIN, VFOV_MAX = np.deg2rad(-7.0), np.deg2rad(52.0)
RANGE_MAX = 70.0
NOISE_STD = 0.02
GOLDEN_ANG = np.pi * (3 - np.sqrt(5))


def build_ray_dirs(n):
    i = np.arange(n, dtype=np.float64)
    ha = (i * GOLDEN_ANG) % (2 * np.pi)
    va = VFOV_MIN + i / n * (VFOV_MAX - VFOV_MIN)
    cv = np.cos(va)
    return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])


RAY_DIRS_LOCAL = build_ray_dirs(N_RAYS)

# geomgroup: 只检测 group 1 (环境), 不检测 group 0 (机器人)
GEOMGROUP = np.zeros(6, dtype=np.uint8)
GEOMGROUP[1] = 1


def scan_lidar(model, data, lidar_id, robot_id, frame_offset=0):
    """mj_multiRay LiDAR 扫描, 返回 body frame 点云 (N,3) float32"""
    pos = data.xpos[lidar_id].copy()
    rmat = data.xmat[lidar_id].reshape(3, 3).copy()

    # 旋转偏移模拟非重复扫描
    ang = frame_offset * 0.628
    c, s = np.cos(ang), np.sin(ang)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    dirs_world = RAY_DIRS_LOCAL @ Rz.T @ rmat.T

    geomid_out = np.full(N_RAYS, -1, dtype=np.int32)
    dist_out = np.full(N_RAYS, -1.0, dtype=np.float64)

    mujoco.mj_multiRay(
        model, data, pos, dirs_world.flatten(),
        GEOMGROUP, 1, -1,
        geomid_out, dist_out, None, N_RAYS, RANGE_MAX
    )

    mask = dist_out > 0.1
    if not mask.any():
        return np.zeros((0, 3), dtype=np.float32)

    pts_world = pos + dirs_world[mask] * dist_out[mask, None]

    # world → body frame
    robot_pos = data.xpos[robot_id].copy()
    robot_rmat = data.xmat[robot_id].reshape(3, 3).copy()
    pts_body = (pts_world - robot_pos) @ robot_rmat

    pts_body += np.random.normal(0, NOISE_STD, pts_body.shape)
    return pts_body.astype(np.float32)


def pack_pointcloud2(pts, frame_id, stamp):
    """(N,3) float32 → PointCloud2 with XYZI (terrain_analysis/localPlanner need intensity)"""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(pts)
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * len(pts)
    xyzi = np.column_stack([pts, np.full(len(pts), 100.0, dtype=np.float32)])
    msg.data = xyzi.astype(np.float32).tobytes()
    return msg


class MuJoCoROS2Bridge(Node):
    def __init__(self, model, data, robot_body='robot', lidar=None, lidar_freq=10.0):
        super().__init__('mujoco_sim')
        self.model = model
        self.data = data
        self._ext_lidar = lidar  # 外部 LiDAR 对象（LivoxMid360SimVectorized 等）

        self.robot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
        if self.robot_id < 0:
            self.get_logger().warn(f"Body '{robot_body}' not found, falling back to index 1")
            self.robot_id = 1
        lidar_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'lidar')
        self.lidar_id = lidar_body_id if lidar_body_id >= 0 else self.robot_id
        self.jnt_adr = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
        ]

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_odom = self.create_publisher(Odometry, TOPICS.odometry, qos)
        self.pub_cloud = self.create_publisher(PointCloud2, TOPICS.registered_cloud, qos)
        self.pub_cloud2 = self.create_publisher(PointCloud2, '/livox/lidar', qos)
        self.pub_map_cloud = self.create_publisher(PointCloud2, TOPICS.map_cloud, qos)
        self.tf_br = TransformBroadcaster(self)

        self.create_subscription(TwistStamped, TOPICS.cmd_vel, self._cmd_cb, qos)

        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._cmd_ts = 0.0
        self._frame = 0

        self.create_timer(0.02, self._tick_50hz)
        self.create_timer(1.0 / max(lidar_freq, 1.0), self._tick_10hz)

        self.get_logger().info(f'MuJoCo sim node started. Robot at {data.xpos[self.robot_id]}')

    def spin_once(self):
        """主循环外调用，处理一次 ROS2 回调（非阻塞）。"""
        rclpy.spin_once(self, timeout_sec=0)

    def destroy(self):
        """释放节点资源。"""
        self.destroy_node()

    def _cmd_cb(self, msg):
        self._cmd_vx = msg.twist.linear.x
        self._cmd_vy = msg.twist.linear.y
        self._cmd_wz = msg.twist.angular.z
        self._cmd_ts = time.time()

    def _get_yaw(self):
        q = self.data.xquat[self.robot_id]
        return np.arctan2(2*(q[0]*q[3] + q[1]*q[2]),
                          1 - 2*(q[2]**2 + q[3]**2))

    def _apply_cmd(self):
        if time.time() - self._cmd_ts > 1.0:
            self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0

        yaw = self._get_yaw()
        c, s = np.cos(yaw), np.sin(yaw)
        vx_w = self._cmd_vx * c - self._cmd_vy * s
        vy_w = self._cmd_vx * s + self._cmd_vy * c

        adr = self.jnt_adr
        self.data.qvel[adr + 0] = vx_w
        self.data.qvel[adr + 1] = vy_w
        self.data.qvel[adr + 2] = 0.0
        self.data.qvel[adr + 3] = 0.0
        self.data.qvel[adr + 4] = 0.0
        self.data.qvel[adr + 5] = self._cmd_wz

        # Stabilize: keep robot upright (zero roll/pitch in qpos)
        jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
        qadr = self.model.jnt_qposadr[jnt_id]
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
        self.data.qpos[qadr + 3] = cy
        self.data.qpos[qadr + 4] = 0.0
        self.data.qpos[qadr + 5] = 0.0
        self.data.qpos[qadr + 6] = sy

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _tick_50hz(self):
        self._apply_cmd()

        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        stamp = self._stamp()
        pos = self.data.xpos[self.robot_id].copy()
        quat = self.data.xquat[self.robot_id].copy()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = FRAMES.odom  # noqa: FRAMES
        odom.child_frame_id = FRAMES.body  # noqa: FRAMES
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.w = float(quat[0])
        odom.pose.pose.orientation.x = float(quat[1])
        odom.pose.pose.orientation.y = float(quat[2])
        odom.pose.pose.orientation.z = float(quat[3])
        odom.twist.twist.linear.x = float(self._cmd_vx)
        odom.twist.twist.linear.y = float(self._cmd_vy)
        odom.twist.twist.angular.z = float(self._cmd_wz)
        self.pub_odom.publish(odom)

        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = FRAMES.map  # noqa: FRAMES
        t1.child_frame_id = FRAMES.odom  # noqa: FRAMES
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = FRAMES.odom  # noqa: FRAMES
        t2.child_frame_id = FRAMES.body  # noqa: FRAMES
        t2.transform.translation.x = float(pos[0])
        t2.transform.translation.y = float(pos[1])
        t2.transform.translation.z = float(pos[2])
        t2.transform.rotation.w = float(quat[0])
        t2.transform.rotation.x = float(quat[1])
        t2.transform.rotation.y = float(quat[2])
        t2.transform.rotation.z = float(quat[3])

        self.tf_br.sendTransform([t1, t2])

    def _tick_10hz(self):
        if self._ext_lidar is not None:
            pts = self._ext_lidar.scan(self.data)
        else:
            pts = scan_lidar(self.model, self.data, self.lidar_id, self.robot_id, self._frame)
        self._frame += 1

        if len(pts) == 0:
            return

        stamp = self._stamp()
        registered_cloud = pack_pointcloud2(pts, FRAMES.body, stamp)  # noqa: FRAMES
        self.pub_cloud.publish(registered_cloud)
        self.pub_cloud2.publish(registered_cloud)

        pos = self.data.xpos[self.robot_id].copy()
        rmat = self.data.xmat[self.robot_id].reshape(3, 3).copy()
        pts_world = pts @ rmat.T + pos
        map_cloud = pack_pointcloud2(pts_world, FRAMES.odom, stamp)  # noqa: FRAMES
        self.pub_map_cloud.publish(map_cloud)

        if self._frame % 50 == 0:
            pos = self.data.xpos[self.robot_id]
            self.get_logger().info(
                f't={self.data.time:.1f}s pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) '
                f'pts={len(pts)} cmd=({self._cmd_vx:.2f},{self._cmd_wz:.2f})'
            )


# 向后兼容别名
MuJoCoSimNode = MuJoCoROS2Bridge


def main():
    xml = DEFAULT_XML
    if len(sys.argv) > 1:
        with open(sys.argv[1]) as f:
            xml = f.read()
        print(f"Loading scene from {sys.argv[1]}")
    else:
        print("Loading default inline scene...")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    for _ in range(500):
        mujoco.mj_step(model, data)

    print(f"Model loaded. Robot at {data.xpos[1]}")

    rclpy.init()
    node = MuJoCoROS2Bridge(model, data)

    print("\n" + "=" * 60)
    print("MuJoCo Sim running!")
    print("  /nav/odometry          -> 50 Hz")  # noqa: TOPIC
    print("  /nav/registered_cloud  -> 10 Hz (body frame, XYZI)")  # noqa: TOPIC
    print("  /livox/lidar           -> 10 Hz (body frame, XYZI)")
    print("  /nav/cmd_vel           <- subscribe TwistStamped")  # noqa: TOPIC
    print("  TF: map -> odom -> body")
    print("=" * 60 + "\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
