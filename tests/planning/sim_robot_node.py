#!/usr/bin/env python3
"""
Building2_9 闭环仿真节点 (ROS2 SITL)

全栈测试: PCT A* 全局规划 → pct_path_adapter → localPlanner → pathFollower → cmd_vel 积分
地图:     building2_9.pickle (真实建筑番茄图, 17×18m, 97×94 grid @ 0.2m/voxel)

该节点承担 "虚拟机器人" 角色:
  - 发布平坦合成点云 → /nav/map_cloud + /nav/terrain_map + /nav/terrain_map_ext
    (替代真实 LiDAR + terrain_analysis, 避免建筑墙体被误判为近场障碍物触发 E-stop)
  - 接收 /nav/cmd_vel → 积分二维运动学 → 更新位姿
  - 持续发布 /nav/stop = 0 (清除 pathFollower safetyStop_ 旗标)
  - 预热 WARMUP_S 秒后发送 /nav/goal_pose 触发 PCT 规划
  - 到达目标或超时后保存 /tmp/sim_result.json + /tmp/sim_traj.png

配置 (通过环境变量, 由 sim_navigation.launch.py 注入):
  SIM_GOAL_X  目标 X (m), 默认  5.0
  SIM_GOAL_Y  目标 Y (m), 默认  7.3  (Corridor_E 场景)
  SIM_START_X 起点 X (m), 默认 -5.5
  SIM_START_Y 起点 Y (m), 默认  7.3

building2_9 世界坐标范围: X∈[-7.95, 10.85] m, Y∈[-10.09, 9.31] m
"""
import math
import os
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Int8, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# ── 默认参数 (被环境变量覆盖) ──────────────────────────────────────────────────
_DEF_START_X   = float(os.environ.get('SIM_START_X',   '-5.5'))
_DEF_START_Y   = float(os.environ.get('SIM_START_Y',    '7.3'))
_DEF_START_YAW = 0.0   # rad
_DEF_GOAL_X    = float(os.environ.get('SIM_GOAL_X',    '5.0'))
_DEF_GOAL_Y    = float(os.environ.get('SIM_GOAL_Y',    '7.3'))

# ── 小车 URDF 路径 ─────────────────────────────────────────────────────────────
_URDF_PATH = os.path.join(os.path.dirname(__file__), 'simple_car.urdf')

# ── 差速驱动参数 (与 simple_car.urdf 一致) ────────────────────────────────────
WHEEL_RADIUS = 0.07   # m
WHEEL_BASE   = 0.34   # m (两轮间距, left +0.17 right -0.17)

# ── 仿真参数 ───────────────────────────────────────────────────────────────────
GOAL_THRESH = 0.5    # m, 到达判定距离
MAX_SECS    = 180.0  # s, 超时时间
DT          = 0.05   # s, 主循环周期 (20 Hz)
TERRAIN_R   = 10.0   # m, 合成平坦地形半径
TERRAIN_S   =  0.4   # m, 格栅间距
WARMUP_S    =  4.0   # s, 预热时间 (≥ joyToSpeedDelay=2.0s)
GOAL_RESEND_S = 15.0 # s, 重新发送目标的间隔


# ── 建筑点云 PCD 默认路径 (按优先级查找) ─────────────────────────────────────
_PCD_CANDIDATES = [
    os.environ.get('SIM_PCD_PATH', ''),
    '/home/sunrise/data/SLAM/navigation/install/pct_planner/share/pct_planner/rsc/pcd/building2_9.pcd',
    '/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/pcd/building2_9.pcd',
]


def _find_pcd() -> Optional[str]:
    for p in _PCD_CANDIDATES:
        if p and os.path.exists(p):
            return p
    # 尝试 ament 安装路径
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('pct_planner')
        p = os.path.join(share, 'rsc', 'pcd', 'building2_9.pcd')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return None


def _load_pcd_binary(pcd_path: str, max_pts: int = 80000) -> Optional[np.ndarray]:
    """
    加载 binary PCD 文件 → (N, 4) float32 [x, y, z, intensity=z].

    - 支持 plain binary 和 binary 格式 (含 x y z ... 字段)
    - 超过 max_pts 时随机降采样
    - intensity 列设为 z 高度值, 供 Foxglove 按高度着色
    """
    try:
        with open(pcd_path, 'rb') as f:
            raw = f.read()

        # 解析 ASCII 头部
        pos, meta = 0, {}
        while pos < len(raw):
            end = raw.find(b'\n', pos)
            if end == -1:
                break
            line = raw[pos:end].decode('ascii', errors='ignore').strip()
            parts = line.split()
            if parts:
                meta[parts[0].upper()] = parts[1:]
            pos = end + 1
            if parts and parts[0].upper() == 'DATA':
                break  # data_offset = pos

        data_offset = pos
        data_type = meta.get('DATA', ['ascii'])[0].lower()
        n_pts  = int(meta.get('POINTS', [0])[0])
        fields = meta.get('FIELDS', [])
        sizes  = [int(s) for s in meta.get('SIZE', [])]
        types  = meta.get('TYPE', [])

        if data_type != 'binary' or 'x' not in fields:
            return None

        # 构建 numpy dtype (每个字段 float32/int32/uint32)
        np_map = {'F': 'f4', 'I': 'i4', 'U': 'u4'}
        dt = np.dtype([(f, np_map.get(t, 'f4'))
                       for f, t in zip(fields, types)])

        arr = np.frombuffer(raw[data_offset: data_offset + n_pts * dt.itemsize],
                            dtype=dt)
        x = arr['x'].astype(np.float32)
        y = arr['y'].astype(np.float32)
        z = arr['z'].astype(np.float32)

        pts = np.column_stack([x, y, z, z.copy()])   # intensity = z for height colormap

        if len(pts) > max_pts:
            idx = np.random.choice(len(pts), max_pts, replace=False)
            pts = pts[idx]

        return pts
    except Exception as e:
        return None


def _make_xyzi_cloud(pts: np.ndarray, frame_id: str, stamp=None) -> PointCloud2:
    """将 (N,4) float32 [x,y,z,i] 数组打包为 PointCloud2 消息。"""
    arr = pts.astype(np.float32)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width  = len(arr)
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step   = 16 * len(arr)
    msg.data       = arr.tobytes()
    return msg


def _flat_cloud(cx: float, cy: float, stamp=None) -> PointCloud2:
    """以 (cx, cy) 为中心生成合成平坦 XYZI=0 点云 (odom 坐标系)。"""
    r = int(TERRAIN_R / TERRAIN_S)
    pts = []
    for ix in range(-r, r + 1):
        for iy in range(-r, r + 1):
            pts.append([cx + ix * TERRAIN_S, cy + iy * TERRAIN_S, 0.0, 0.0])
    arr = np.array(pts, dtype=np.float32)

    msg = PointCloud2()
    msg.header.frame_id = 'odom'
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(arr)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step   = 16 * len(arr)
    msg.data       = arr.tobytes()
    return msg


class SimRobotNode(Node):
    def __init__(self):
        super().__init__('sim_robot_node')

        # ── 机器人状态 ──
        self.x   = _DEF_START_X
        self.y   = _DEF_START_Y
        self.yaw = _DEF_START_YAW
        self.gx  = _DEF_GOAL_X
        self.gy  = _DEF_GOAL_Y

        self.vx = self.vy = self.wz = 0.0
        self.cmd_count = 0
        self.traj = []
        self.goal_reached = False
        self.t_start = None
        self.phase = 'warmup'   # warmup → running → done → saved

        # ── 车轮角度 (for JointState + RViz 车轮滚动) ──
        self._left_wheel_angle  = 0.0
        self._right_wheel_angle = 0.0

        # ── TF ──
        self.static_br = StaticTransformBroadcaster(self)
        self.tf_br     = TransformBroadcaster(self)
        self._pub_static_tf()

        # ── Publishers ──
        self.pub_odom    = self.create_publisher(Odometry,    '/nav/odometry',       10)
        self.pub_cloud   = self.create_publisher(PointCloud2, '/nav/map_cloud',       10)
        self.pub_terrain = self.create_publisher(PointCloud2, '/nav/terrain_map',     10)
        self.pub_te      = self.create_publisher(PointCloud2, '/nav/terrain_map_ext', 10)
        self.pub_goal    = self.create_publisher(PoseStamped, '/nav/goal_pose',       10)
        self.pub_stop    = self.create_publisher(Int8,        '/nav/stop',            10)

        # robot_state_publisher 需要 /joint_states (轮子关节)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)

        # /robot_description latched (TRANSIENT_LOCAL): RViz/Foxglove 随时可读取 URDF
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_desc  = self.create_publisher(String,       '/robot_description',  latched)
        # /nav/building_cloud: 真实建筑 3D 点云 (latched, 仅用于可视化)
        self.pub_bldg  = self.create_publisher(PointCloud2,  '/nav/building_cloud', latched)
        self._publish_robot_description()
        self._publish_building_cloud()

        # ── Subscribers ──
        # pathFollower 以 BEST_EFFORT 50 Hz 发布 cmd_vel
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(TwistStamped, '/nav/cmd_vel',        self._on_cmd,        be)
        # 航点跟踪事件 (JSON)
        self.create_subscription(String,       '/nav/adapter_status', self._on_adapter,    10)
        # 全局规划器状态 (pathFollower 发布 GOAL_REACHED/STUCK 等)
        self.create_subscription(String,       '/nav/planner_status', self._on_planner,    10)

        # ── 20 Hz 主循环 ──
        self.create_timer(DT, self._tick)

        self.get_logger().info(
            f'[sim] 仿真节点启动. '
            f'起点=({self.x:.2f},{self.y:.2f}) '
            f'目标=({self.gx:.2f},{self.gy:.2f}) '
            f'预热={WARMUP_S:.1f}s 超时={MAX_SECS:.0f}s')

    # ── TF 发布 ──────────────────────────────────────────────────────────────

    def _publish_robot_description(self):
        """将 simple_car.urdf 发布到 /robot_description (latched)。"""
        if os.path.exists(_URDF_PATH):
            with open(_URDF_PATH) as f:
                content = f.read()
        else:
            self.get_logger().warn(f'URDF not found: {_URDF_PATH}')
            content = ''
        msg = String()
        msg.data = content
        self.pub_desc.publish(msg)
        self.get_logger().info(f'[sim] /robot_description published ({len(content)} chars)')

    def _publish_building_cloud(self):
        """加载 building2_9.pcd → 发布真实建筑 3D 点云到 /nav/building_cloud (latched)。"""
        pcd_path = _find_pcd()
        if pcd_path is None:
            self.get_logger().warn('[sim] building2_9.pcd not found — 3D cloud skipped')
            return

        pts = _load_pcd_binary(pcd_path, max_pts=80000)
        if pts is None or len(pts) == 0:
            self.get_logger().warn(f'[sim] PCD load failed: {pcd_path}')
            return

        msg = _make_xyzi_cloud(pts, frame_id='map',
                                stamp=self.get_clock().now().to_msg())
        self.pub_bldg.publish(msg)
        self.get_logger().info(
            f'[sim] Building cloud: {len(pts)} pts published from {pcd_path}')

    def _pub_static_tf(self):
        """发布静态 TF:  map→odom  +  body→base_link (URDF 根坐标系)。"""
        tfs = []

        # map → odom (仿真中无 SLAM 漂移, 恒等变换)
        t1 = TransformStamped()
        t1.header.stamp    = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id  = 'odom'
        t1.transform.rotation.w = 1.0
        tfs.append(t1)

        # body → base_link (恒等变换, 让 robot_state_publisher 的 URDF TF 链接到导航坐标系)
        t2 = TransformStamped()
        t2.header.stamp    = t1.header.stamp
        t2.header.frame_id = 'body'
        t2.child_frame_id  = 'base_link'
        t2.transform.rotation.w = 1.0
        tfs.append(t2)

        self.static_br.sendTransform(tfs)

    def _pub_odom_tf(self, now):
        """发布 odom → body TF + /nav/odometry。"""
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'body'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z    = math.sin(self.yaw / 2)
        tf.transform.rotation.w    = math.cos(self.yaw / 2)
        self.tf_br.sendTransform(tf)

        od = Odometry()
        od.header.stamp    = now
        od.header.frame_id = 'odom'
        od.child_frame_id  = 'body'
        od.pose.pose.position.x    = self.x
        od.pose.pose.position.y    = self.y
        od.pose.pose.orientation.z = math.sin(self.yaw / 2)
        od.pose.pose.orientation.w = math.cos(self.yaw / 2)
        od.twist.twist.linear.x    = self.vx
        od.twist.twist.linear.y    = self.vy
        od.twist.twist.angular.z   = self.wz
        self.pub_odom.publish(od)

    # ── 回调 ──────────────────────────────────────────────────────────────────

    def _on_cmd(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.wz = msg.twist.angular.z
        self.cmd_count += 1

    def _on_adapter(self, msg: String):
        self.get_logger().info(f'[adapter] {msg.data}')

    def _on_planner(self, msg: String):
        status = msg.data.strip()
        self.get_logger().info(f'[planner] {status}')
        if status == 'GOAL_REACHED' and self.phase == 'running':
            self.goal_reached = True
            self.get_logger().info('*** GOAL_REACHED (from pathFollower) ***')
            self.phase = 'done'

    # ── 运动学积分 ────────────────────────────────────────────────────────────

    def _integrate(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x   += (c * self.vx - s * self.vy) * DT
        self.y   += (s * self.vx + c * self.vy) * DT
        self.yaw += self.wz * DT
        self.yaw  = (self.yaw + math.pi) % (2 * math.pi) - math.pi

        # 差速驱动车轮角度积分
        v_left  = self.vx - self.wz * WHEEL_BASE / 2.0
        v_right = self.vx + self.wz * WHEEL_BASE / 2.0
        self._left_wheel_angle  += v_left  / WHEEL_RADIUS * DT
        self._right_wheel_angle += v_right / WHEEL_RADIUS * DT

        # 发布 JointState (robot_state_publisher 需要)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self._left_wheel_angle, self._right_wheel_angle]
        js.velocity = [v_left / WHEEL_RADIUS, v_right / WHEEL_RADIUS]
        self.pub_joints.publish(js)

    # ── 主循环 ────────────────────────────────────────────────────────────────

    def _tick(self):
        if self.phase == 'saved':
            return
        if self.phase == 'done':
            self._finish()
            return

        if self.t_start is None:
            self.t_start = time.time()
        elapsed = time.time() - self.t_start
        now     = self.get_clock().now().to_msg()

        # ── 始终发布: 里程计 + TF + 合成平坦地形 + stop=0 ──
        self._pub_odom_tf(now)

        terrain = _flat_cloud(self.x, self.y, stamp=now)
        self.pub_cloud.publish(terrain)
        self.pub_terrain.publish(terrain)
        self.pub_te.publish(terrain)

        stop_msg = Int8()
        stop_msg.data = 0
        self.pub_stop.publish(stop_msg)

        # ── 预热阶段 ──
        if self.phase == 'warmup':
            if elapsed >= WARMUP_S:
                self.phase = 'running'
                self._pub_goal(now)
                self.get_logger().info(
                    f'== RUNNING: 目标已发送, PCT 规划器开始规划 ==')

        # ── 运行阶段 ──
        elif self.phase == 'running':
            # 每 GOAL_RESEND_S 秒重新发送目标 (防规划器重启后丢失)
            if elapsed > WARMUP_S:
                slot_now  = int((elapsed - WARMUP_S) / GOAL_RESEND_S)
                slot_prev = int((elapsed - WARMUP_S - DT) / GOAL_RESEND_S)
                if slot_now > slot_prev:
                    self._pub_goal(now)

            self._integrate()

            # 以约 2 Hz 记录轨迹和日志
            if int(elapsed * 2) > int((elapsed - DT) * 2):
                dist = math.hypot(self.x - self.gx, self.y - self.gy)
                self.get_logger().info(
                    f't={elapsed:6.1f}s  '
                    f'pos=({self.x:.3f},{self.y:.3f})  '
                    f'yaw={math.degrees(self.yaw):6.1f}°  '
                    f'v=({self.vx:.2f},{self.vy:.2f})  ω={self.wz:.3f}  '
                    f'dist={dist:.2f}m  cmds={self.cmd_count}')
                self.traj.append({
                    't':    round(elapsed, 2),
                    'x':    round(self.x, 3),
                    'y':    round(self.y, 3),
                    'yaw':  round(math.degrees(self.yaw), 1),
                    'vx':   round(self.vx, 3),
                    'vy':   round(self.vy, 3),
                    'wz':   round(self.wz, 3),
                    'dist': round(dist, 3),
                })

                if dist < GOAL_THRESH:
                    self.goal_reached = True
                    self.get_logger().info(
                        f'*** GOAL REACHED (距离)  t={elapsed:.1f}s  dist={dist:.3f}m ***')
                    self.phase = 'done'
                    return

            if elapsed > MAX_SECS:
                dist = math.hypot(self.x - self.gx, self.y - self.gy)
                self.get_logger().warn(
                    f'TIMEOUT {elapsed:.0f}s  final_dist={dist:.2f}m')
                self.phase = 'done'
                return

        if self.phase == 'done':
            self._finish()

    def _pub_goal(self, now):
        msg = PoseStamped()
        msg.header.stamp    = now
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.gx
        msg.pose.position.y = self.gy
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        self.get_logger().info(f'[goal] 目标已发布: ({self.gx:.2f}, {self.gy:.2f})')

    # ── 保存结果 ──────────────────────────────────────────────────────────────

    def _finish(self):
        self.phase = 'saved'
        result = {
            'goal_reached':  self.goal_reached,
            'start':         [_DEF_START_X, _DEF_START_Y],
            'goal':          [self.gx, self.gy],
            'final_pos':     [round(self.x, 3), round(self.y, 3)],
            'final_yaw_deg': round(math.degrees(self.yaw), 1),
            'cmd_count':     self.cmd_count,
            'trajectory':    self.traj,
        }
        json_path = '/tmp/sim_result.json'
        with open(json_path, 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f'结果已保存: {json_path}')
        self._plot(result)

    def _plot(self, result):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            traj = result['trajectory']
            if not traj:
                self.get_logger().warn('轨迹为空, 跳过绘图')
                return

            ts  = [p['t']    for p in traj]
            xs  = [p['x']    for p in traj]
            ys  = [p['y']    for p in traj]
            ds  = [p['dist'] for p in traj]
            vxs = [p['vx']   for p in traj]
            wzs = [p['wz']   for p in traj]

            fig, axes = plt.subplots(1, 2, figsize=(14, 6))

            # 面板 1: XY 轨迹
            ax = axes[0]
            ax.plot(xs, ys, 'b-', lw=2, label='机器人轨迹')
            ax.plot(xs[0],  ys[0],  'go',  ms=12, label=f'起点 ({xs[0]:.1f},{ys[0]:.1f})')
            ax.plot(self.gx, self.gy, 'r*', ms=18, label=f'目标 ({self.gx:.1f},{self.gy:.1f})')
            ax.plot(xs[-1], ys[-1], 'bs', ms=10,
                    label=f'终点 ({xs[-1]:.2f},{ys[-1]:.2f})')
            status = '✓ 到达目标' if result['goal_reached'] else f'✗ 超时 (dist={ds[-1]:.2f}m)'
            ax.set_title(f'Building2_9 SITL — {status}', fontsize=11)
            ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
            ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

            # 面板 2: 时间序列指标
            ax2 = axes[1]
            ax2.plot(ts, ds,  'r-',  lw=2,   label='到目标距离 (m)')
            ax2.plot(ts, vxs, 'b-',  lw=1.5, label='vx (m/s)')
            ax2.plot(ts, wzs, 'g--', lw=1.2, label='ωz (rad/s)')
            ax2.axhline(GOAL_THRESH, color='r', ls=':', alpha=0.5,
                        label=f'到达阈值 {GOAL_THRESH}m')
            ax2.set_xlabel('时间 (s)'); ax2.set_title('导航指标')
            ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

            plt.tight_layout()
            png_path = '/tmp/sim_traj.png'
            plt.savefig(png_path, dpi=130, bbox_inches='tight')
            self.get_logger().info(f'轨迹图已保存: {png_path}')
        except Exception as e:
            self.get_logger().error(f'绘图失败: {e}')


def main():
    rclpy.init()
    node = SimRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[sim] 用户中断')
    finally:
        if node.phase not in ('done', 'saved') and node.traj:
            node._finish()
        node.destroy_node()
        rclpy.shutdown()
    print('[sim] 完成.')


if __name__ == '__main__':
    main()
