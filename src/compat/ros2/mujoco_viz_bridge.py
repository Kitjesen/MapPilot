#!/usr/bin/env python3
"""
MuJoCo 全功能可视化导航 — 点云 + 轨迹 + 3D建筑场景

可视化内容:
  - LiDAR 点云 (青色小球, 每帧刷新)
  - 规划轨迹 (绿色线段, 来自 /path)
  - 实际轨迹 (蓝色尾迹, 历史位置)
  - 目标点 (红色球体)

场景:
  - 两层楼 + 楼梯 + 坡道
  - 多种障碍物 (箱子/圆柱/墙壁)
  - 走廊 + 门洞

用法:
    DISPLAY=:0 python3 /tmp/sim_viz_full.py
"""
import numpy as np
import time
import sys
import collections
from pathlib import Path as FsPath

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path as ROSPath
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

SRC_DIR = FsPath(__file__).resolve().parents[2]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from core.runtime_interface import FRAMES, TOPICS  # noqa: E402

# ── 3D 建筑场景 ──────────────────────────────────────────────────
SCENE_XML = """
<mujoco model="building_nav">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <headlight ambient="0.35 0.35 0.35"/>
    <quality shadowsize="4096"/>
    <map znear="0.05" zfar="200"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".82 .82 .78" rgb2=".72 .72 .68"/>
    <material name="grid" texture="grid" texrepeat="15 15" texuniform="true"/>
    <texture name="wall_tex" type="2d" builtin="flat" width="1" height="1" rgb1=".88 .85 .80"/>
    <material name="wall_mat" texture="wall_tex"/>
    <texture name="stair_tex" type="2d" builtin="flat" width="1" height="1" rgb1=".65 .60 .55"/>
    <material name="stair_mat" texture="stair_tex"/>
  </asset>

  <worldbody>
    <light pos="10 8 12" dir="-0.3 -0.2 -1" diffuse="0.9 0.88 0.82" castshadow="true"/>
    <light pos="-5 -5 10" dir="0.2 0.2 -1" diffuse="0.4 0.42 0.5" castshadow="false"/>

    <!-- ============= 地面 ============= -->
    <geom name="floor" type="plane" size="30 30 0.1" group="1" conaffinity="1" condim="3"
          material="grid"/>

    <!-- ============= 外墙 (25m x 18m) ============= -->
    <!-- 南墙 -->
    <geom name="wall_s" type="box" size="12.5 0.15 2.0" pos="12.5 0 2.0" group="1"
          conaffinity="1" material="wall_mat"/>
    <!-- 北墙 -->
    <geom name="wall_n" type="box" size="12.5 0.15 2.0" pos="12.5 18 2.0" group="1"
          conaffinity="1" material="wall_mat"/>
    <!-- 西墙 -->
    <geom name="wall_w" type="box" size="0.15 9.0 2.0" pos="0 9 2.0" group="1"
          conaffinity="1" material="wall_mat"/>
    <!-- 东墙 (带门洞: y=7~11 留空) -->
    <geom name="wall_e1" type="box" size="0.15 3.5 2.0" pos="25 3.5 2.0" group="1"
          conaffinity="1" material="wall_mat"/>
    <geom name="wall_e2" type="box" size="0.15 3.5 2.0" pos="25 14.5 2.0" group="1"
          conaffinity="1" material="wall_mat"/>

    <!-- ============= 内墙 (x=15 处, 带门洞 y=7~10) ============= -->
    <geom name="iwall1" type="box" size="0.12 3.5 2.0" pos="15 3.5 2.0" group="1"
          conaffinity="1" rgba=".78 .75 .70 1"/>
    <geom name="iwall2" type="box" size="0.12 4.0 2.0" pos="15 14.0 2.0" group="1"
          conaffinity="1" rgba=".78 .75 .70 1"/>

    <!-- ============= 楼梯 (x=20~23, y=1~4, 上到 z=3.5m) ============= -->
    <geom name="stair01" type="box" size="1.5 1.5 0.125" pos="21.5 2.5 0.125" group="1"
          conaffinity="1" material="stair_mat"/>
    <geom name="stair02" type="box" size="1.5 1.5 0.250" pos="21.5 2.5 0.250" group="1"
          conaffinity="1" material="stair_mat"/>
    <geom name="stair03" type="box" size="1.5 1.2 0.375" pos="21.5 2.5 0.375" group="1"
          conaffinity="1" material="stair_mat"/>
    <geom name="stair04" type="box" size="1.5 0.9 0.500" pos="21.5 2.8 0.500" group="1"
          conaffinity="1" material="stair_mat"/>
    <geom name="stair05" type="box" size="1.5 0.6 0.625" pos="21.5 3.1 0.625" group="1"
          conaffinity="1" material="stair_mat"/>
    <geom name="stair06" type="box" size="1.5 0.3 0.750" pos="21.5 3.4 0.750" group="1"
          conaffinity="1" material="stair_mat"/>

    <!-- 楼梯平台 -->
    <geom name="stair_landing" type="box" size="2.0 2.0 0.05" pos="21.5 5.5 1.5" group="1"
          conaffinity="1" rgba=".7 .65 .6 1"/>

    <!-- ============= 坡道 (x=3~8, y=12~15) ============= -->
    <geom name="ramp1" type="box" size="2.5 1.5 0.05" pos="5.5 13.5 0.3"
          euler="0 -0.12 0" group="1" conaffinity="1" rgba=".6 .55 .45 1"/>

    <!-- ============= 1F 障碍物 ============= -->
    <!-- 货架 -->
    <geom name="shelf1" type="box" size="0.8 0.3 1.0" pos="4 4 1.0" group="1"
          conaffinity="1" rgba=".55 .40 .25 1"/>
    <geom name="shelf2" type="box" size="0.8 0.3 1.0" pos="4 6 1.0" group="1"
          conaffinity="1" rgba=".55 .40 .25 1"/>
    <!-- 桌子 -->
    <geom name="table1" type="box" size="1.0 0.6 0.4" pos="8 9 0.4" group="1"
          conaffinity="1" rgba=".70 .55 .35 1"/>
    <!-- 柱子 -->
    <geom name="pillar1" type="cylinder" size="0.25 2.0" pos="10 5 2.0" group="1"
          conaffinity="1" rgba=".75 .73 .70 1"/>
    <geom name="pillar2" type="cylinder" size="0.25 2.0" pos="10 13 2.0" group="1"
          conaffinity="1" rgba=".75 .73 .70 1"/>
    <!-- 箱子堆 -->
    <geom name="crate1" type="box" size="0.4 0.4 0.4" pos="12 3 0.4" group="1"
          conaffinity="1" rgba=".80 .65 .30 1"/>
    <geom name="crate2" type="box" size="0.35 0.35 0.35" pos="12 3 1.15" group="1"
          conaffinity="1" rgba=".85 .70 .35 1"/>
    <!-- 圆桶 -->
    <geom name="barrel1" type="cylinder" size="0.3 0.5" pos="7 15 0.5" group="1"
          conaffinity="1" rgba=".3 .5 .7 1"/>
    <geom name="barrel2" type="cylinder" size="0.3 0.5" pos="7.8 15.3 0.5" group="1"
          conaffinity="1" rgba=".3 .5 .7 1"/>

    <!-- ============= 东区 (门洞外) 障碍物 ============= -->
    <geom name="container1" type="box" size="1.5 0.8 1.5" pos="22 10 1.5" group="1"
          conaffinity="1" rgba=".2 .45 .65 1"/>
    <geom name="pipe1" type="cylinder" size="0.15 1.5" pos="18 15 1.5" group="1"
          conaffinity="1" rgba=".65 .65 .65 1"/>

    <!-- ============= 目标标记 ============= -->
    <geom name="goal_marker" type="sphere" size="0.35" pos="22 12 0.5"
          conaffinity="0" contype="0" group="1" rgba="1.0 0.2 0.1 0.5"/>
    <geom name="goal_ring" type="cylinder" size="0.5 0.02" pos="22 12 0.02"
          conaffinity="0" contype="0" group="1" rgba="1.0 0.3 0.1 0.3"/>
    <!-- 起点标记 -->
    <geom name="start_marker" type="cylinder" size="0.4 0.02" pos="2 2 0.02"
          conaffinity="0" contype="0" group="1" rgba="0.1 0.5 1.0 0.3"/>

    <!-- ============= 机器人 (group=0) ============= -->
    <body name="robot" pos="2 2 0.45">
      <freejoint name="root"/>
      <geom name="chassis" type="box" size="0.4 0.25 0.1" group="0" mass="20"
            conaffinity="1" rgba=".18 .38 .76 1"/>
      <geom name="leg_fl" type="capsule" size="0.035"
            fromto=" 0.28  0.18 -0.10  0.28  0.18 -0.42" group="0" mass="0.8" rgba=".25 .25 .25 1"/>
      <geom name="leg_fr" type="capsule" size="0.035"
            fromto=" 0.28 -0.18 -0.10  0.28 -0.18 -0.42" group="0" mass="0.8" rgba=".25 .25 .25 1"/>
      <geom name="leg_rl" type="capsule" size="0.035"
            fromto="-.28  0.18 -0.10 -.28  0.18 -0.42" group="0" mass="0.8" rgba=".25 .25 .25 1"/>
      <geom name="leg_rr" type="capsule" size="0.035"
            fromto="-.28 -0.18 -0.10 -.28 -0.18 -0.42" group="0" mass="0.8" rgba=".25 .25 .25 1"/>
      <!-- LiDAR 头 -->
      <body name="lidar" pos="0 0 0.25">
        <geom name="lidar_vis" type="sphere" size="0.05" group="0" mass="0.1"
              conaffinity="0" rgba="0 .78 .85 1"/>
      </body>
      <inertial pos="0 0 0" mass="22" diaginertia="0.75 1.15 1.15"/>
    </body>
  </worldbody>
</mujoco>
"""

# ── LiDAR ──────────────────────────────────────────────────────
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
GEOMGROUP = np.zeros(6, dtype=np.uint8)
GEOMGROUP[1] = 1


def scan_lidar(model, data, lidar_id, robot_id, frame_offset=0):
    pos = data.xpos[lidar_id].copy()
    rmat = data.xmat[lidar_id].reshape(3, 3).copy()
    ang = frame_offset * 0.628
    c, s = np.cos(ang), np.sin(ang)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    dirs_world = RAY_DIRS_LOCAL @ Rz.T @ rmat.T
    geomid_out = np.full(N_RAYS, -1, dtype=np.int32)
    dist_out = np.full(N_RAYS, -1.0, dtype=np.float64)
    mujoco.mj_multiRay(model, data, pos, dirs_world.flatten(),
                        GEOMGROUP, 1, -1, geomid_out, dist_out, None, N_RAYS, RANGE_MAX)
    mask = dist_out > 0.1
    if not mask.any():
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)
    pts_world = pos + dirs_world[mask] * dist_out[mask, None]
    robot_pos = data.xpos[robot_id].copy()
    robot_rmat = data.xmat[robot_id].reshape(3, 3).copy()
    pts_body = (pts_world - robot_pos) @ robot_rmat
    pts_body += np.random.normal(0, NOISE_STD, pts_body.shape)
    return pts_body.astype(np.float32), pts_world.astype(np.float32)


def pack_pointcloud2(pts, frame_id, stamp):
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


# ── 可视化辅助 ──────────────────────────────────────────────────
MAX_VIZ_GEOMS = 800  # user_scn 最大 geom 数
CLOUD_SUBSAMPLE = 300  # 点云显示点数
TRAIL_MAXLEN = 500  # 轨迹最大长度

def height_to_rgba(z, z_min=0.0, z_max=3.0):
    """Z 高度 → 颜色 (蓝→青→绿→黄→红)"""
    t = np.clip((z - z_min) / (z_max - z_min + 1e-6), 0, 1)
    if t < 0.25:
        r, g, b = 0, t*4, 1
    elif t < 0.5:
        r, g, b = 0, 1, 1-(t-0.25)*4
    elif t < 0.75:
        r, g, b = (t-0.5)*4, 1, 0
    else:
        r, g, b = 1, 1-(t-0.75)*4, 0
    return np.array([r, g, b, 0.7], dtype=np.float32)


def draw_viz(viewer, cloud_world, planned_path, trail, goal_pos):
    """在 MuJoCo viewer 中绘制点云、轨迹"""
    scn = viewer.user_scn
    scn.ngeom = 0
    idx = 0

    # 1) 点云 (青色小球, 按高度着色)
    if len(cloud_world) > 0:
        step = max(1, len(cloud_world) // CLOUD_SUBSAMPLE)
        pts = cloud_world[::step]
        for p in pts:
            if idx >= MAX_VIZ_GEOMS - 100:
                break
            rgba = height_to_rgba(p[2])
            mujoco.mjv_initGeom(
                scn.geoms[idx], mujoco.mjtGeom.mjGEOM_SPHERE,
                [0.03, 0, 0], p.astype(np.float64),
                np.eye(3).flatten(), rgba)
            scn.ngeom = idx + 1
            idx += 1

    # 2) 规划轨迹 (绿色线段)
    if len(planned_path) > 1:
        for i in range(len(planned_path) - 1):
            if idx >= MAX_VIZ_GEOMS - 50:
                break
            a = planned_path[i]
            b = planned_path[i + 1]
            mujoco.mjv_initGeom(scn.geoms[idx], mujoco.mjtGeom.mjGEOM_CAPSULE,
                                np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                                np.array([0.1, 0.95, 0.2, 0.85], dtype=np.float32))
            mujoco.mjv_connector(
                scn.geoms[idx], mujoco.mjtGeom.mjGEOM_CAPSULE, 0.025,
                np.array([a[0], a[1], a[2] + 0.05], dtype=np.float64),
                np.array([b[0], b[1], b[2] + 0.05], dtype=np.float64))
            scn.ngeom = idx + 1
            idx += 1

    # 3) 实际轨迹 (蓝色尾迹, 渐变透明)
    trail_list = list(trail)
    if len(trail_list) > 1:
        step = max(1, len(trail_list) // 150)
        sampled = trail_list[::step]
        for i in range(len(sampled) - 1):
            if idx >= MAX_VIZ_GEOMS - 10:
                break
            a = sampled[i]
            b = sampled[i + 1]
            alpha = 0.3 + 0.6 * (i / len(sampled))
            mujoco.mjv_initGeom(scn.geoms[idx], mujoco.mjtGeom.mjGEOM_CAPSULE,
                                np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                                np.array([0.15, 0.4, 1.0, alpha], dtype=np.float32))
            mujoco.mjv_connector(
                scn.geoms[idx], mujoco.mjtGeom.mjGEOM_CAPSULE, 0.018,
                np.array([a[0], a[1], a[2] + 0.03], dtype=np.float64),
                np.array([b[0], b[1], b[2] + 0.03], dtype=np.float64))
            scn.ngeom = idx + 1
            idx += 1

    # 4) 目标点 (脉冲红球)
    if goal_pos is not None:
        pulse = 0.3 + 0.15 * np.sin(time.time() * 3)
        mujoco.mjv_initGeom(
            scn.geoms[idx], mujoco.mjtGeom.mjGEOM_SPHERE,
            [pulse, 0, 0], np.array(goal_pos, dtype=np.float64),
            np.eye(3).flatten(), np.array([1, 0.2, 0.1, 0.6], dtype=np.float32))
        scn.ngeom = idx + 1
        idx += 1


class MuJoCoSimNode(Node):
    def __init__(self, model, data):
        super().__init__('mujoco_sim')
        self.model = model
        self.data = data
        self.robot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
        self.lidar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'lidar')
        self.jnt_adr = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root')]

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_odom = self.create_publisher(Odometry, TOPICS.odometry, qos)
        self.pub_cloud = self.create_publisher(PointCloud2, TOPICS.registered_cloud, qos)
        self.pub_cloud2 = self.create_publisher(PointCloud2, '/livox/lidar', qos)
        self.tf_br = TransformBroadcaster(self)
        self.create_subscription(TwistStamped, TOPICS.cmd_vel, self._cmd_cb, qos)
        self.create_subscription(ROSPath, '/path', self._path_cb, qos)

        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._cmd_ts = 0.0
        self._frame = 0

        # 可视化数据
        self.cloud_world = np.zeros((0, 3), dtype=np.float32)
        self.planned_path = []
        self.trail = collections.deque(maxlen=TRAIL_MAXLEN)
        self.goal_pos = [22.0, 12.0, 0.5]

        self.create_timer(0.02, self._tick_50hz)
        self.create_timer(0.1, self._tick_10hz)
        self.get_logger().info(f'Sim started. Robot at {data.xpos[self.robot_id]}')

    def _cmd_cb(self, msg):
        self._cmd_vx = msg.twist.linear.x
        self._cmd_vy = msg.twist.linear.y
        self._cmd_wz = msg.twist.angular.z
        self._cmd_ts = time.time()

    def _path_cb(self, msg):
        """接收 localPlanner 的规划路径"""
        self.planned_path = []
        for pose in msg.poses:
            p = pose.pose.position
            self.planned_path.append([p.x, p.y, p.z])

    def _get_yaw(self):
        q = self.data.xquat[self.robot_id]
        return np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))

    def _apply_cmd(self):
        if time.time() - self._cmd_ts > 1.0:
            self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0
        yaw = self._get_yaw()
        c, s = np.cos(yaw), np.sin(yaw)
        vx_w = self._cmd_vx * c - self._cmd_vy * s
        vy_w = self._cmd_vx * s + self._cmd_vy * c
        adr = self.jnt_adr
        self.data.qvel[adr:adr+6] = [vx_w, vy_w, 0, 0, 0, self._cmd_wz]
        jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
        qadr = self.model.jnt_qposadr[jnt_id]
        cy, sy = np.cos(yaw/2), np.sin(yaw/2)
        self.data.qpos[qadr+3:qadr+7] = [cy, 0, 0, sy]

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _tick_50hz(self):
        self._apply_cmd()
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
        stamp = self._stamp()
        pos = self.data.xpos[self.robot_id].copy()
        quat = self.data.xquat[self.robot_id].copy()

        # 记录轨迹
        self.trail.append(pos[:3].copy())

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
        pts_body, pts_world = scan_lidar(
            self.model, self.data, self.lidar_id, self.robot_id, self._frame)
        self._frame += 1
        self.cloud_world = pts_world
        if len(pts_body) == 0:
            return
        stamp = self._stamp()
        msg = pack_pointcloud2(pts_body, FRAMES.body, stamp)  # noqa: FRAMES
        self.pub_cloud.publish(msg)
        self.pub_cloud2.publish(msg)
        if self._frame % 50 == 0:
            pos = self.data.xpos[self.robot_id]
            self.get_logger().info(
                f't={self.data.time:.1f}s pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) '
                f'pts={len(pts_body)} cmd=({self._cmd_vx:.2f},{self._cmd_wz:.2f})')


def main():
    print("Loading building scene...")
    model = mujoco.MjModel.from_xml_string(SCENE_XML)
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)

    robot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    print(f"Robot at {data.xpos[robot_id]}")

    rclpy.init()
    node = MuJoCoSimNode(model, data)

    print("Opening MuJoCo viewer...")
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.azimuth = -60
    viewer.cam.elevation = -35
    viewer.cam.distance = 30
    viewer.cam.lookat[:] = [12, 9, 1]

    print("\n" + "=" * 55)
    print("  MuJoCo 3D Navigation Viewer")
    print("  Cyan dots  = LiDAR point cloud")
    print("  Green line = planned path (localPlanner)")
    print("  Blue trail = actual trajectory")
    print("  Red sphere = goal (pulsing)")
    print("=" * 55 + "\n")

    try:
        while viewer.is_running():
            rclpy.spin_once(node, timeout_sec=0.001)
            draw_viz(viewer, node.cloud_world, node.planned_path,
                     node.trail, node.goal_pos)
            viewer.sync()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Done.")


if __name__ == '__main__':
    main()
