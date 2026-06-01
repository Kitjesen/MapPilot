#!/usr/bin/env python3
"""
NOVA Dog 导航仿真 Bridge — 真实 ONNX 步态 + ROS2 全链路

架构:
  cmd_vel (ROS2) → direction → ONNX policy → PD joint ctrl → MuJoCo physics
  MuJoCo state → odometry + TF + PointCloud2 (ROS2)

用法:
  # 在 S100P 上 (需要 ROS2 Humble):
  source /opt/ros/humble/setup.bash
  DISPLAY=:0 python3 nova_nav_bridge.py              # GUI 模式
  MUJOCO_GL=egl python3 nova_nav_bridge.py --headless # 无头模式
"""
import argparse
import sys
import time
import threading
from pathlib import Path
from collections import deque

import numpy as np
import mujoco

# ── 路径 ─────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
SIM_DIR = SCRIPT_DIR.parents[2] / "sim"
SRC_DIR = SCRIPT_DIR.parents[2] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from core.runtime_interface import FRAMES, TOPICS  # noqa: E402

ROBOT_XML = SIM_DIR / "robots" / "nova_dog" / "robot_with_camera.xml"
POLICY_ONNX = SIM_DIR / "robots" / "nova_dog" / "policy.onnx"

# ── 场景 XML ─────────────────────────────────────────────────────
SCENE_TEMPLATE = """\
<mujoco model="nova_nav">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.4 0.4 0.4"/>
    <quality shadowsize="2048"/>
    <map znear="0.01" zfar="200"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".85 .85 .82" rgb2=".7 .7 .68"/>
    <material name="grid" texture="grid" texrepeat="20 20" texuniform="true"/>
    <texture name="wall_tex" type="2d" builtin="flat" width="1" height="1" rgb1=".88 .85 .80"/>
    <material name="wall_mat" texture="wall_tex"/>
  </asset>

  <include file="robot_with_camera.xml"/>

  <worldbody>
    <light pos="5 -5 8" dir="-0.3 0.3 -1" diffuse="0.9 0.88 0.82" castshadow="true"/>
    <light pos="-5 5 8" dir="0.3 -0.3 -1" diffuse="0.4 0.42 0.5" castshadow="false"/>
    <geom name="floor" type="plane" size="30 30 0.1" material="grid"
          conaffinity="1" condim="3" friction="1 0.5 0.5" group="1"/>

    <!-- 简单障碍物 (用于测试避障) -->
    <geom name="wall1" type="box" size="3.0 0.15 1.0" pos="5 3 1.0"
          conaffinity="1" material="wall_mat" group="1"/>
    <geom name="wall2" type="box" size="0.15 2.0 1.0" pos="8 5 1.0"
          conaffinity="1" material="wall_mat" group="1"/>
    <geom name="box1" type="box" size="0.5 0.5 0.5" pos="3 6 0.5"
          conaffinity="1" rgba=".8 .6 .3 1" group="1"/>

    <!-- 目标标记 -->
    <geom name="goal" type="sphere" size="0.3" pos="10 8 0.3"
          contype="0" conaffinity="0" rgba="1 0.2 0.1 0.5" group="1"/>
  </worldbody>
</mujoco>
"""

# ── Dart standingPose (直接用, 不取反) ────────────────────────────
STANDING_POSE = np.array([
    -0.1, -0.8,  1.8,     # FR: hip, thigh, calf
     0.1,  0.8, -1.8,     # FL
     0.1,  0.8, -1.8,     # RR
    -0.1, -0.8,  1.8,     # RL
     0.0,  0.0,  0.0, 0.0 # foot
], dtype=np.float64)

ACTION_SCALE = np.array([
    0.125, 0.25, 0.25,  # FR
    0.125, 0.25, 0.25,  # FL
    0.125, 0.25, 0.25,  # RR
    0.125, 0.25, 0.25,  # RL
    5.0, 5.0, 5.0, 5.0  # foot
], dtype=np.float64)

JOINT_VEL_SCALE = np.array([0.05, 0.05, 0.05, 0.05], dtype=np.float64)
IMU_GYRO_SCALE = 0.25
OBS_DIM = 57
HISTORY_LEN = 5

# ── 关节名 → MuJoCo ID 映射 (leg only, 16 joints) ────────────────
LEG_JOINT_NAMES = [
    'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint', 'fr_foot_joint',
    'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint', 'fl_foot_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint', 'rr_foot_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint', 'rl_foot_joint',
]

# actuator 索引 (arm=0..7, leg=8..23)
LEG_ACT_OFFSET = 8

# LiDAR
N_RAYS = 6400          # fallback: golden angle spiral 射线数
VFOV_MIN, VFOV_MAX = np.deg2rad(-7.0), np.deg2rad(52.0)
RANGE_MAX = 70.0
GOLDEN_ANG = np.pi * (3 - np.sqrt(5))

MID360_NPY = Path('/tmp/nova_sim/sensors/mid360.npy')  # OmniPerception scan pattern
MID360_SAMPLES = 20000   # 每帧采样数 (真实 MID-360 每帧 20000 射线)

# ── MuJoCo ↔ Dart/ONNX 关节顺序转换 ─────────────────────────────
# MuJoCo (4+4+4+4): FR(hip,thigh,calf,foot), FL(...), RR(...), RL(...)
# Dart   (3+3+3+3+4): FR(hip,thigh,calf), FL(h,t,c), RR, RL, FR_f, FL_f, RR_f, RL_f
MJ_TO_DART = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15])
DART_TO_MJ = np.array([0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15])


def build_ray_dirs(n):
    i = np.arange(n, dtype=np.float64)
    ha = (i * GOLDEN_ANG) % (2 * np.pi)
    va = VFOV_MIN + i / n * (VFOV_MAX - VFOV_MIN)
    cv = np.cos(va)
    return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])


RAY_DIRS_LOCAL = build_ray_dirs(N_RAYS)
GEOMGROUP = np.zeros(6, dtype=np.uint8)
GEOMGROUP[1] = 1  # 只检测环境 geom (group=1), 不检测机器人 (group=0)


class LivoxMid360:
    """OmniPerception MID-360 真实扫描模式 — 滑动窗口采样.

    mid360.npy 格式: (N, 2), col0=theta(方位角 rad), col1=phi(仰角 rad)
    每帧取 MID360_SAMPLES 条射线，窗口循环滑动，模拟非重复扫描。
    """

    def __init__(self, npy_path, samples_per_frame=20000):
        pattern = np.load(str(npy_path))  # (N, 2)
        theta = pattern[:, 0]
        phi   = pattern[:, 1]
        cp = np.cos(phi)
        self.dirs = np.column_stack([
            cp * np.cos(theta),
            cp * np.sin(theta),
            np.sin(phi),
        ]).astype(np.float64)  # (N, 3) unit vectors in sensor frame
        self.total   = len(self.dirs)
        self.samples = min(samples_per_frame, self.total)
        self.offset  = 0
        print(f'[LivoxMid360] Loaded {npy_path}: {self.total} rays, '
              f'{self.samples}/frame')

    def get_dirs(self):
        """滑动窗口取 self.samples 条射线，每帧前进 self.samples。"""
        end = self.offset + self.samples
        if end <= self.total:
            dirs = self.dirs[self.offset:end]
        else:
            wrap = end - self.total
            dirs = np.vstack([self.dirs[self.offset:], self.dirs[:wrap]])
        self.offset = end % self.total
        return dirs  # (samples, 3)


# ══════════════════════════════════════════════════════════════════
#  ONNX Policy Wrapper
# ══════════════════════════════════════════════════════════════════

class PolicyRunner:
    """ONNX 步态策略推理, 匹配 brainstem StandardObservationBuilder."""

    def __init__(self, onnx_path: str):
        import onnxruntime as ort
        self.session = ort.InferenceSession(
            onnx_path, providers=['CPUExecutionProvider'])
        inp = self.session.get_inputs()[0]
        out = self.session.get_outputs()[0]
        print(f'[Policy] Loaded {onnx_path}')
        print(f'  input:  {inp.name} {inp.shape}')
        print(f'  output: {out.name} {out.shape}')
        self.input_name = inp.name
        self.output_name = out.name

        # 5-frame history buffer — 用站立姿态初始化 (NOT zeros!)
        # brainstem 初始化时用真实传感器数据填充, 这里用站立时的理论值
        self.history = deque(maxlen=HISTORY_LEN)
        self.last_action = STANDING_POSE.copy()
        # 将在 warm_up() 中用真实传感器数据填充

    def warm_up(self, gyroscope, projected_gravity, joint_pos_16, joint_vel_16):
        """用真实传感器数据填充 history buffer (匹配 brainstem Memory 初始化)."""
        init_obs = self.build_obs(
            gyroscope, projected_gravity,
            np.zeros(3),  # direction = 0 (idle)
            joint_pos_16, joint_vel_16)
        self.history.clear()
        for _ in range(HISTORY_LEN):
            self.history.append(init_obs.copy())
        print(f'[Policy] History warmed up: pg={projected_gravity[:3]}')

    @staticmethod
    def clamp_action(action):
        """clampPerJoint — 匹配 brainstem Walk.clampAction()."""
        clamped = action.copy()
        for leg in range(4):
            base = leg * 3
            clamped[base + 0] = np.clip(clamped[base + 0], -0.5, 0.5)    # hip
            clamped[base + 1] = np.clip(clamped[base + 1], -1.5, 1.5)    # thigh
            clamped[base + 2] = np.clip(clamped[base + 2], -2.5, 2.5)    # calf
        for i in range(12, 16):
            clamped[i] = np.clip(clamped[i], -0.5, 0.5)  # foot
        return clamped

    def build_obs(self, gyroscope, projected_gravity, direction,
                  joint_pos_16, joint_vel_16):
        """构建 57 维观测, 完全匹配 brainstem StandardObservationBuilder.

        注意: joint_pos_16/joint_vel_16 是 MuJoCo 顺序 (4+4+4+4),
        需要先转换到 Dart 顺序 (3+3+3+3+4) 再计算.
        """
        gyro = gyroscope * IMU_GYRO_SCALE
        pg = projected_gravity

        # MuJoCo → Dart 关节顺序转换
        jp_dart = joint_pos_16[MJ_TO_DART]
        jv_dart = joint_vel_16[MJ_TO_DART]

        # joint position: (current - standingPose), foot 置零
        jp = jp_dart - STANDING_POSE
        jp[12:] = 0.0  # discardFoot()

        # joint velocity: scale(hip, thigh, calf, foot)
        # brainstem: jointVelocity * (0.05, 0.05, 0.05, 0.05)
        jv = jv_dart.copy()
        for leg in range(4):
            base = leg * 3
            jv[base + 0] *= JOINT_VEL_SCALE[0]  # hip
            jv[base + 1] *= JOINT_VEL_SCALE[1]  # thigh
            jv[base + 2] *= JOINT_VEL_SCALE[2]  # calf
        for i in range(12, 16):
            jv[i] *= JOINT_VEL_SCALE[3]  # foot

        # last action: (action - standingPose) / actionScale
        # last_action 已经是 Dart 顺序
        act = (self.last_action - STANDING_POSE) / ACTION_SCALE

        obs = np.concatenate([gyro, pg, direction, jp, jv, act]).astype(np.float32)
        return obs

    def infer(self, obs):
        """推理: obs(57) → 加入 history → concat(285) → ONNX → action(16)."""
        self.history.append(obs)
        obs_history = np.concatenate(list(self.history)).reshape(1, -1).astype(np.float32)
        result = self.session.run([self.output_name],
                                  {self.input_name: obs_history})[0]
        raw_action = result[0]  # (16,)

        # realAction = clamp(onnxOutput * actionScale + standingPose)
        real_action = raw_action * ACTION_SCALE + STANDING_POSE
        real_action = self.clamp_action(real_action)
        self.last_action = real_action.copy()
        return real_action


# ══════════════════════════════════════════════════════════════════
#  IMU 仿真 (从 MuJoCo 状态提取)
# ══════════════════════════════════════════════════════════════════

def get_imu(model, data, body_id):
    """
    从 MuJoCo 提取 IMU 数据 (匹配 brainstem 的 ImuService).

    Returns:
        gyroscope: (3,) 机体坐标系角速度 (rad/s)
        projected_gravity: (3,) 机体坐标系重力投影 (归一化)
    """
    # 机体旋转矩阵 (world → body 的转置)
    R = data.xmat[body_id].reshape(3, 3)  # body-to-world

    # 角速度: 从 qvel 的 freejoint 部分取 (index 3:6 是 world frame angular vel)
    omega_world = data.qvel[3:6]
    gyroscope = R.T @ omega_world  # world → body frame

    # 重力投影: 世界重力 (0,0,-1) 投影到机体坐标系
    gravity_world = np.array([0.0, 0.0, -1.0])
    projected_gravity = R.T @ gravity_world

    return gyroscope.astype(np.float64), projected_gravity.astype(np.float64)


def get_joint_state(model, data, joint_ids):
    """获取 16 个腿关节的位置和速度."""
    pos = np.array([data.qpos[model.jnt_qposadr[j]] for j in joint_ids])
    vel = np.array([data.qvel[model.jnt_dofadr[j]] for j in joint_ids])
    return pos, vel


# ══════════════════════════════════════════════════════════════════
#  LiDAR 扫描
# ══════════════════════════════════════════════════════════════════

def scan_lidar(model, data, lidar_body_id, frame_idx, livox=None):
    pos  = data.xpos[lidar_body_id].copy()
    rmat = data.xmat[lidar_body_id].reshape(3, 3).copy()

    if livox is not None:
        # OmniPerception MID-360: 真实扫描模式，滑动窗口采样
        dirs_local = livox.get_dirs()          # (N, 3) sensor frame
        n_rays = len(dirs_local)
    else:
        # Fallback: golden angle 螺旋 + 每帧旋转
        ang = frame_idx * 0.628
        c, s = np.cos(ang), np.sin(ang)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        dirs_local = RAY_DIRS_LOCAL @ Rz.T    # (N, 3) sensor frame
        n_rays = N_RAYS

    dirs_world = dirs_local @ rmat.T           # sensor → world frame

    dist_out   = np.full(n_rays, -1.0, dtype=np.float64)
    geomid_out = np.full(n_rays, -1,   dtype=np.int32)

    mujoco.mj_multiRay(model, data, pos, dirs_world.flatten(),
                        GEOMGROUP, 1, -1, geomid_out, dist_out, None,
                        n_rays, RANGE_MAX)

    mask = dist_out > 0.1
    if not mask.any():
        return np.zeros((0, 4), dtype=np.float32)

    pts = (pos + dirs_world[mask] * dist_out[mask, None]).astype(np.float32)
    intensity = np.full((pts.shape[0], 1), 100.0, dtype=np.float32)
    return np.hstack([pts, intensity])  # (N, 4) XYZI


# ══════════════════════════════════════════════════════════════════
#  ROS2 Bridge
# ══════════════════════════════════════════════════════════════════

class NavBridge:
    """MuJoCo ↔ ROS2 导航 Bridge."""

    def __init__(self, model, data, policy, headless=False):
        self.m = model
        self.d = data
        self.policy = policy
        self.headless = headless
        self.frame_idx = 0

        # Body / joint IDs
        self.base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.lidar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'lidar_link')
        self.leg_joint_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in LEG_JOINT_NAMES
        ]

        # LiDAR: 优先加载 OmniPerception MID-360 真实扫描模式
        if MID360_NPY.exists():
            try:
                self.livox = LivoxMid360(MID360_NPY, MID360_SAMPLES)
            except Exception as e:
                print(f'[Bridge] MID-360 load failed ({e}), fallback to golden angle')
                self.livox = None
        else:
            print(f'[Bridge] {MID360_NPY} not found, using golden angle LiDAR')
            self.livox = None

        # cmd_vel 接收
        self.cmd_vel = np.zeros(3)  # vx, vy, wz
        self.cmd_vel_time = 0.0
        self.lock = threading.Lock()

        # 控制频率
        self.policy_dt = 0.02  # 50 Hz (matching brainstem)
        self.lidar_dt = 0.1    # 10 Hz
        self.odom_dt = 0.02    # 50 Hz

        self._init_ros2()

    def _init_ros2(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TwistStamped, TransformStamped
        from sensor_msgs.msg import PointCloud2, PointField
        from tf2_ros import TransformBroadcaster

        rclpy.init()
        self.node = rclpy.create_node('nova_sim_bridge')

        # Publishers
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.node.create_publisher(Odometry, TOPICS.odometry, qos_reliable)
        self.cloud_pub = self.node.create_publisher(PointCloud2, TOPICS.map_cloud, qos_reliable)
        self.cloud_body_pub = self.node.create_publisher(PointCloud2, TOPICS.registered_cloud, qos_reliable)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Subscriber
        self.node.create_subscription(
            TwistStamped, TOPICS.cmd_vel, self._cmd_vel_cb, qos_reliable)

        self._PointCloud2 = PointCloud2
        self._PointField = PointField
        self._Odometry = Odometry
        self._TransformStamped = TransformStamped

        self.node.get_logger().info('Nova sim bridge started')

    def _cmd_vel_cb(self, msg):
        with self.lock:
            self.cmd_vel[0] = msg.twist.linear.x
            self.cmd_vel[1] = msg.twist.linear.y
            self.cmd_vel[2] = msg.twist.angular.z
            self.cmd_vel_time = time.time()

    def _watchdog_cmd_vel(self):
        """200ms 无 cmd_vel → 归零."""
        if time.time() - self.cmd_vel_time > 0.2:
            self.cmd_vel[:] = 0.0

    def step_policy(self):
        """50Hz: cmd_vel → ONNX → joint ctrl."""
        self._watchdog_cmd_vel()

        with self.lock:
            direction = self.cmd_vel.copy()

        gyro, pg = get_imu(self.m, self.d, self.base_id)
        joint_pos, joint_vel = get_joint_state(self.m, self.d, self.leg_joint_ids)

        obs = self.policy.build_obs(gyro, pg, direction, joint_pos, joint_vel)
        action_dart = self.policy.infer(obs)  # (16,) Dart 顺序

        # Dart → MuJoCo 关节顺序转换, 写入 ctrl
        action_mj = action_dart[DART_TO_MJ]
        self.d.ctrl[LEG_ACT_OFFSET:LEG_ACT_OFFSET + 16] = action_mj

    def publish_odom(self):
        """发布 odometry + TF."""
        pos = self.d.qpos[:3]
        quat_wxyz = self.d.qpos[3:7]  # MuJoCo: w,x,y,z
        vel = self.d.qvel[:3]
        omega = self.d.qvel[3:6]

        now = self.node.get_clock().now().to_msg()

        # Odometry
        odom = self._Odometry()
        odom.header.stamp = now
        odom.header.frame_id = FRAMES.odom  # noqa: FRAMES
        odom.child_frame_id = FRAMES.body  # noqa: FRAMES
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        # ROS uses x,y,z,w
        odom.pose.pose.orientation.x = float(quat_wxyz[1])
        odom.pose.pose.orientation.y = float(quat_wxyz[2])
        odom.pose.pose.orientation.z = float(quat_wxyz[3])
        odom.pose.pose.orientation.w = float(quat_wxyz[0])
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = float(omega[2])
        self.odom_pub.publish(odom)

        # TF: odom → body
        tf = self._TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = FRAMES.odom  # noqa: FRAMES
        tf.child_frame_id = FRAMES.body  # noqa: FRAMES
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        tf.transform.rotation.x = float(quat_wxyz[1])
        tf.transform.rotation.y = float(quat_wxyz[2])
        tf.transform.rotation.z = float(quat_wxyz[3])
        tf.transform.rotation.w = float(quat_wxyz[0])
        self.tf_broadcaster.sendTransform(tf)

    def publish_cloud(self):
        """发布 LiDAR 点云."""
        pts_xyzi = scan_lidar(self.m, self.d, self.lidar_id, self.frame_idx, self.livox)
        if len(pts_xyzi) == 0:
            return

        now = self.node.get_clock().now().to_msg()

        # World frame cloud → /nav/map_cloud
        msg = self._PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = FRAMES.odom  # noqa: FRAMES
        msg.height = 1
        msg.width = len(pts_xyzi)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = [
            self._PointField(name='x', offset=0, datatype=7, count=1),
            self._PointField(name='y', offset=4, datatype=7, count=1),
            self._PointField(name='z', offset=8, datatype=7, count=1),
            self._PointField(name='intensity', offset=12, datatype=7, count=1),
        ]
        msg.point_step = 16
        msg.row_step = 16 * len(pts_xyzi)
        msg.data = pts_xyzi.astype(np.float32).tobytes()
        self.cloud_pub.publish(msg)

        # Body frame cloud → /nav/registered_cloud
        base_pos = self.d.xpos[self.base_id]
        base_R = self.d.xmat[self.base_id].reshape(3, 3)
        pts_body = (pts_xyzi[:, :3] - base_pos) @ base_R
        pts_body_xyzi = np.hstack([pts_body.astype(np.float32),
                                    pts_xyzi[:, 3:4]])
        msg2 = self._PointCloud2()
        msg2.header.stamp = now
        msg2.header.frame_id = FRAMES.body  # noqa: FRAMES
        msg2.height = 1
        msg2.width = len(pts_body_xyzi)
        msg2.is_dense = False
        msg2.is_bigendian = False
        msg2.fields = msg.fields
        msg2.point_step = 16
        msg2.row_step = 16 * len(pts_body_xyzi)
        msg2.data = pts_body_xyzi.astype(np.float32).tobytes()
        self.cloud_body_pub.publish(msg2)

    def spin(self):
        """主循环."""
        import rclpy

        sim_time = 0.0
        last_policy = 0.0
        last_odom = 0.0
        last_cloud = 0.0
        time.time()

        print('\n[Bridge] Running — policy 50Hz, odom 50Hz, LiDAR 10Hz')
        print(f'  base_id={self.base_id}, lidar_id={self.lidar_id}')
        print('  Send cmd_vel to /nav/cmd_vel to move the robot\n')  # noqa: TOPIC

        if not self.headless:
            viewer_mod = __import__('mujoco.viewer', fromlist=['viewer'])
            with viewer_mod.launch_passive(self.m, self.d) as viewer:
                viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
                viewer.cam.distance = 3.0
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -25

                while viewer.is_running():
                    mujoco.mj_step(self.m, self.d)
                    sim_time += self.m.opt.timestep

                    if sim_time - last_policy >= self.policy_dt:
                        self.step_policy()
                        last_policy = sim_time
                    if sim_time - last_odom >= self.odom_dt:
                        self.publish_odom()
                        last_odom = sim_time
                    if sim_time - last_cloud >= self.lidar_dt:
                        self.publish_cloud()
                        self.frame_idx += 1
                        last_cloud = sim_time

                    try:
                        rclpy.spin_once(self.node, timeout_sec=0)
                    except Exception:
                        logger.debug("nova_nav_bridge: rclpy spin_once error (viewer may have shutdown)")
                    viewer.sync()
        else:
            try:
                while rclpy.ok():
                    mujoco.mj_step(self.m, self.d)
                    sim_time += self.m.opt.timestep

                    if sim_time - last_policy >= self.policy_dt:
                        self.step_policy()
                        last_policy = sim_time
                    if sim_time - last_odom >= self.odom_dt:
                        self.publish_odom()
                        last_odom = sim_time
                    if sim_time - last_cloud >= self.lidar_dt:
                        self.publish_cloud()
                        self.frame_idx += 1
                        last_cloud = sim_time

                    rclpy.spin_once(self.node, timeout_sec=0)

                    # 打印状态
                    if int(sim_time * 10) % 50 == 0 and sim_time > 0.1:
                        h = self.d.qpos[2]
                        vx = self.cmd_vel[0]
                        print(f'  t={sim_time:.1f}s  h={h:.3f}m  cmd_vx={vx:.2f}',
                              end='\r')
            except (KeyboardInterrupt, Exception):
                print('\n[Bridge] Shutting down...')

    def _tick(self, *args):
        pass


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='NOVA Dog Navigation Sim Bridge')
    parser.add_argument('--headless', action='store_true', help='无头模式')
    parser.add_argument('--scene', type=str, default=None,
                        help='自定义场景 XML 路径')
    parser.add_argument('--policy', type=str, default=None,
                        help='ONNX policy 路径 (默认: sim/robots/nova_dog/policy.onnx)')
    parser.add_argument('--start', type=float, nargs=3, default=[0.0, 0.0, 0.35],
                        metavar=('X', 'Y', 'Z'), help='机器人起始位置 (m), 默认 0 0 0.35')
    args = parser.parse_args()

    # 写场景到 robot/ 目录
    scene_path = ROBOT_XML.parent / "_nav_scene.xml"
    if args.scene:
        scene_path = Path(args.scene)
    else:
        scene_path.write_text(SCENE_TEMPLATE)

    print(f'Loading scene: {scene_path}')
    m = mujoco.MjModel.from_xml_path(str(scene_path))
    d = mujoco.MjData(m)
    print(f'  nq={m.nq} nv={m.nv} nu={m.nu}')

    # 设置初始站立姿态
    standing_qpos = {
        'fr_hip_joint': -0.1, 'fr_thigh_joint': -0.8, 'fr_calf_joint':  1.8,
        'fl_hip_joint':  0.1, 'fl_thigh_joint':  0.8, 'fl_calf_joint': -1.8,
        'rr_hip_joint':  0.1, 'rr_thigh_joint':  0.8, 'rr_calf_joint': -1.8,
        'rl_hip_joint': -0.1, 'rl_thigh_joint': -0.8, 'rl_calf_joint':  1.8,
    }
    for jname, val in standing_qpos.items():
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        d.qpos[m.jnt_qposadr[jid]] = val

    # 初始 ctrl = standing
    standing_ctrl = np.zeros(m.nu)
    standing_ctrl[8:12]  = [-0.1, -0.8,  1.8, 0.0]
    standing_ctrl[12:16] = [ 0.1,  0.8, -1.8, 0.0]
    standing_ctrl[16:20] = [ 0.1,  0.8, -1.8, 0.0]
    standing_ctrl[20:24] = [-0.1, -0.8,  1.8, 0.0]
    d.ctrl[:] = standing_ctrl

    # 设置起始位置 (freejoint qpos[0:7] = x,y,z, qw,qx,qy,qz)
    d.qpos[0:3] = args.start
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # identity quaternion
    print(f'  Start position: {args.start}')

    # 稳定 1s
    for _ in range(500):
        mujoco.mj_step(m, d)
    print(f'  Standing height: {d.qpos[2]:.3f}m')

    # 加载 ONNX policy
    onnx_path = args.policy if args.policy else str(POLICY_ONNX)
    policy = PolicyRunner(onnx_path)

    # 用稳定后的真实传感器数据初始化 history (关键!)
    base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
    leg_joint_ids = [
        mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        for n in LEG_JOINT_NAMES
    ]
    gyro, pg = get_imu(m, d, base_id)
    joint_pos, joint_vel = get_joint_state(m, d, leg_joint_ids)
    policy.warm_up(gyro, pg, joint_pos, joint_vel)

    # 启动 bridge
    bridge = NavBridge(m, d, policy, headless=args.headless)
    bridge.spin()


if __name__ == '__main__':
    main()
