#!/usr/bin/env python3
"""
Han Dog gRPC Bridge — 将 3d_NAV 导航栈接入四足机器人
=====================================================

功能:
  1. 订阅 /cmd_vel (TwistStamped) → 转换为 han_dog CMS gRPC Walk() 调用
  2. 流式订阅 CMS.ListenImu()  → 发布 /Odometry (IMU 姿态) + /robot_state (IMU 字段)
  3. 流式订阅 CMS.ListenJoint() → 发布 /robot_state (关节角度/速度/力矩)
  4. 管理机器人生命周期: Enable/Disable, StandUp/SitDown
  5. 独立看门狗: cmd_vel 超时 → 自动发送零速

架构:
  Flutter App
      ↓ gRPC :50051
  GrpcGateway (3d_NAV)
      ↓ ROS2 /cmd_vel
  ★ han_dog_bridge (本节点) ★
      ↓ gRPC :13145
  四足机器人 CMS 服务

用法:
  ros2 run robot_driver han_dog_bridge.py --ros-args \\
      -p dog_host:=192.168.4.100 -p dog_port:=13145
"""

import asyncio
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Int8
from tf_transformations import quaternion_from_euler

# interface 包定义的 RobotState 消息
from interface.msg import RobotState, BatteryState

# han_dog gRPC 客户端
from grpc import aio as grpc_aio
import han_dog_message as dog_msg


class HanDogBridge(Node):
    """ROS2 ↔ han_dog gRPC 桥接节点."""

    def __init__(self):
        super().__init__('han_dog_bridge')

        # ─── 参数声明 ───
        self.declare_parameter('dog_host', '127.0.0.1')
        self.declare_parameter('dog_port', 13145)
        self.declare_parameter('max_linear_speed', 1.0)   # m/s → Walk 归一化基准
        self.declare_parameter('max_angular_speed', 1.0)   # rad/s → Walk 归一化基准
        self.declare_parameter('cmd_vel_timeout_ms', 200.0) # 看门狗超时
        self.declare_parameter('control_rate', 50.0)        # Hz
        self.declare_parameter('auto_enable', True)         # 启动时自动 Enable
        self.declare_parameter('auto_standup', True)        # 启动时自动 StandUp
        self.declare_parameter('reconnect_interval', 3.0)   # 重连间隔 (秒)

        self._dog_host = self.get_parameter('dog_host').value
        self._dog_port = self.get_parameter('dog_port').value
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._watchdog_timeout = self.get_parameter('cmd_vel_timeout_ms').value / 1000.0
        self._control_rate = self.get_parameter('control_rate').value
        self._auto_enable = self.get_parameter('auto_enable').value
        self._auto_standup = self.get_parameter('auto_standup').value
        self._reconnect_interval = self.get_parameter('reconnect_interval').value

        # ─── ROS2 发布者 ───
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.robot_state_pub = self.create_publisher(RobotState, '/robot_state', 10)
        self.watchdog_pub = self.create_publisher(Bool, '/driver/watchdog_active', 10)

        # ─── ROS2 订阅者 ───
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self._cmd_vel_callback, 10)
        # 控制信号: 0=normal, 1=soft_stop, 2=hard_stop
        self.stop_sub = self.create_subscription(
            Int8, '/stop', self._stop_callback, 10)

        # ─── 内部状态 ───
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_triggered = False
        self._connected = False
        self._standing = False
        self._enabled = False
        self._shutdown = False

        # gRPC 相关 (在 asyncio 线程中初始化)
        self._channel: Optional[grpc_aio.Channel] = None
        self._stub: Optional[dog_msg.CmsStub] = None

        # 最新 IMU 数据缓存 (用于合成 Odometry)
        self._latest_quaternion = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w
        self._latest_gyro = (0.0, 0.0, 0.0)
        self._imu_stamp = None

        # 最新关节数据缓存 (用于发布 RobotState)
        self._latest_joint_pos = [0.0] * 12
        self._latest_joint_vel = [0.0] * 12
        self._latest_joint_eff = [0.0] * 12

        # ─── 看门狗定时器 ───
        self._watchdog_timer = self.create_timer(
            1.0 / self._control_rate, self._watchdog_loop)

        # ─── 启动 asyncio 事件循环 (独立线程) ───
        self._loop = asyncio.new_event_loop()
        self._grpc_thread = threading.Thread(
            target=self._run_async_loop, daemon=True)
        self._grpc_thread.start()

        self.get_logger().info(
            f'Han Dog Bridge starting → {self._dog_host}:{self._dog_port} '
            f'(max_linear={self._max_linear} m/s, max_angular={self._max_angular} rad/s)')

    # ================================================================
    #  ROS2 回调
    # ================================================================

    def _cmd_vel_callback(self, msg: TwistStamped):
        """收到 /cmd_vel → 缓存指令, 喂狗."""
        self._last_cmd_time = self.get_clock().now()
        self._cmd_vx = msg.twist.linear.x
        self._cmd_vy = msg.twist.linear.y
        self._cmd_wz = msg.twist.angular.z

        if self._watchdog_triggered:
            self._watchdog_triggered = False
            self.get_logger().info('Watchdog cleared: cmd_vel resumed')

        # 异步发送 Walk 指令
        if self._connected and self._standing:
            walk_vec = self._twist_to_walk(
                self._cmd_vx, self._cmd_vy, self._cmd_wz)
            asyncio.run_coroutine_threadsafe(
                self._send_walk(walk_vec), self._loop)

    def _stop_callback(self, msg: Int8):
        """收到 /stop 信号."""
        if msg.data == 2:
            # 硬停: 坐下
            self.get_logger().warn('Hard stop received → SitDown')
            asyncio.run_coroutine_threadsafe(self._sit_down(), self._loop)
        elif msg.data == 1:
            # 软停: 发零速
            self.get_logger().info('Soft stop received → zero velocity')
            asyncio.run_coroutine_threadsafe(
                self._send_walk(dog_msg.Vector3(x=0.0, y=0.0, z=0.0)),
                self._loop)

    # ================================================================
    #  看门狗
    # ================================================================

    def _watchdog_loop(self):
        """定时检查 cmd_vel 超时."""
        elapsed = (
            self.get_clock().now() - self._last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > self._watchdog_timeout:
            if not self._watchdog_triggered:
                self._watchdog_triggered = True
                self.get_logger().warn(
                    f'WATCHDOG: No cmd_vel for {elapsed * 1000:.0f}ms → zero velocity')
            # 发送零速
            if self._connected and self._standing:
                asyncio.run_coroutine_threadsafe(
                    self._send_walk(dog_msg.Vector3(x=0.0, y=0.0, z=0.0)),
                    self._loop)

        wd_msg = Bool()
        wd_msg.data = self._watchdog_triggered
        self.watchdog_pub.publish(wd_msg)

        # 发布合成的里程计 (基于 IMU)
        self._publish_odometry()

        # 发布机器人状态
        self._publish_robot_state()

    # ================================================================
    #  Twist → Walk 转换
    # ================================================================

    def _twist_to_walk(self, vx: float, vy: float, wz: float) -> dog_msg.Vector3:
        """将 m/s, rad/s 速度归一化到 [-1, 1] 的 Walk 指令.

        han_dog Walk(Vector3):
          x = 前后 [-1, 1]  (正=前进)
          y = 左右 [-1, 1]  (正=左移)
          z = 旋转 [-1, 1]  (正=逆时针)
        """
        nx = max(-1.0, min(1.0, vx / self._max_linear)) if self._max_linear > 0 else 0.0
        ny = max(-1.0, min(1.0, vy / self._max_linear)) if self._max_linear > 0 else 0.0
        nz = max(-1.0, min(1.0, wz / self._max_angular)) if self._max_angular > 0 else 0.0
        return dog_msg.Vector3(x=nx, y=ny, z=nz)

    # ================================================================
    #  ROS2 发布
    # ================================================================

    def _publish_odometry(self):
        """从 IMU 数据合成并发布 /Odometry."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'body'

        # 姿态 (来自 dog IMU)
        qx, qy, qz, qw = self._latest_quaternion
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # 角速度 (来自 dog IMU)
        gx, gy, gz = self._latest_gyro
        odom.twist.twist.angular.x = gx
        odom.twist.twist.angular.y = gy
        odom.twist.twist.angular.z = gz

        # 线速度 (使用当前指令作为估计)
        odom.twist.twist.linear.x = self._cmd_vx
        odom.twist.twist.linear.y = self._cmd_vy

        self.odom_pub.publish(odom)

    def _publish_robot_state(self):
        """发布 /robot_state (interface::msg::RobotState)."""
        state = RobotState()
        state.header.stamp = self.get_clock().now().to_msg()

        # 关节数据 (12 DOF: 4 legs × 3 joints)
        state.joint_positions = self._latest_joint_pos[:12]
        state.joint_velocities = self._latest_joint_vel[:12]
        state.joint_efforts = self._latest_joint_eff[:12]

        # 足端力 (暂无数据, 填零)
        state.foot_forces = [0.0, 0.0, 0.0, 0.0]

        # IMU
        qx, qy, qz, qw = self._latest_quaternion
        state.imu_quaternion = [qw, qx, qy, qz]  # RobotState 格式: w, x, y, z
        gx, gy, gz = self._latest_gyro
        state.imu_gyroscope = [gx, gy, gz]
        state.imu_accelerometer = [0.0, 0.0, 0.0]
        state.imu_temperature = 0

        # 电池 (暂用模拟值, 后续可从 dog 获取)
        battery = BatteryState()
        battery.percentage = 80
        battery.voltage = 25.0
        battery.current = 0.0
        battery.temperature = [25, 25]
        battery.status = 0
        battery.cycle_count = 0
        state.battery = battery

        self.robot_state_pub.publish(state)

    # ================================================================
    #  gRPC 异步逻辑
    # ================================================================

    def _run_async_loop(self):
        """在独立线程运行 asyncio 事件循环."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self):
        """主 async 入口: 连接 + 启动流式监听."""
        while not self._shutdown:
            try:
                await self._connect_and_run()
            except Exception as e:
                self._connected = False
                self.get_logger().error(
                    f'gRPC connection lost: {e}. '
                    f'Reconnecting in {self._reconnect_interval}s...')
                await asyncio.sleep(self._reconnect_interval)

    async def _connect_and_run(self):
        """连接到 dog gRPC 服务并启动所有流."""
        addr = f'{self._dog_host}:{self._dog_port}'
        self.get_logger().info(f'Connecting to Han Dog CMS at {addr}...')

        async with grpc_aio.insecure_channel(addr) as channel:
            self._channel = channel
            self._stub = dog_msg.CmsStub(channel)
            self._connected = True
            self.get_logger().info(f'Connected to Han Dog CMS at {addr}')

            # 自动使能
            if self._auto_enable:
                await self._enable()

            # 自动站立
            if self._auto_standup:
                await self._stand_up()

            # 并发启动所有流式监听
            await asyncio.gather(
                self._listen_imu(),
                self._listen_joint(),
                self._listen_history(),
                return_exceptions=True,
            )

    # ─── 生命周期控制 ───

    async def _enable(self):
        """使能电机."""
        try:
            await self._stub.Enable(dog_msg.Empty())
            self._enabled = True
            self.get_logger().info('Dog motors ENABLED')
        except Exception as e:
            self.get_logger().error(f'Enable failed: {e}')

    async def _disable(self):
        """禁用电机."""
        try:
            await self._stub.Disable(dog_msg.Empty())
            self._enabled = False
            self.get_logger().info('Dog motors DISABLED')
        except Exception as e:
            self.get_logger().error(f'Disable failed: {e}')

    async def _stand_up(self):
        """站立."""
        try:
            await self._stub.StandUp(dog_msg.Empty())
            self._standing = True
            self.get_logger().info('Dog STANDING UP')
        except Exception as e:
            self.get_logger().error(f'StandUp failed: {e}')

    async def _sit_down(self):
        """坐下."""
        try:
            # 先发零速
            await self._stub.Walk(dog_msg.Vector3(x=0.0, y=0.0, z=0.0))
            await self._stub.SitDown(dog_msg.Empty())
            self._standing = False
            self.get_logger().info('Dog SITTING DOWN')
        except Exception as e:
            self.get_logger().error(f'SitDown failed: {e}')

    async def _send_walk(self, vec: dog_msg.Vector3):
        """发送 Walk 指令."""
        try:
            await self._stub.Walk(vec)
        except Exception as e:
            self.get_logger().warn(f'Walk command failed: {e}')

    # ─── 流式监听 ───

    async def _listen_imu(self):
        """订阅 dog IMU 数据流 → 更新内部状态."""
        self.get_logger().info('Starting IMU stream listener...')
        async for imu in self._stub.ListenImu(dog_msg.Empty()):
            # han_dog Imu: gyroscope (Vector3), quaternion (Quaternion), timestamp
            self._latest_gyro = (imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z)
            # han_dog Quaternion: Hamilton convention (w, x, y, z)
            # ROS convention: (x, y, z, w)
            self._latest_quaternion = (
                imu.quaternion.x,
                imu.quaternion.y,
                imu.quaternion.z,
                imu.quaternion.w,
            )

    async def _listen_joint(self):
        """订阅 dog 关节数据流 → 更新内部状态."""
        self.get_logger().info('Starting Joint stream listener...')
        async for joint in self._stub.ListenJoint(dog_msg.Empty()):
            if joint.HasField('all_joints'):
                aj = joint.all_joints
                # han_dog Matrix4: 16 个值 (4 legs × 4 joints: hip,thigh,calf,foot)
                # RobotState: 12 个值 (4 legs × 3 joints: hip,thigh,calf)
                # 需要跳过 foot 关节 (index 12,13,14,15 → 每条腿第4个)
                pos = list(aj.position.values) if aj.position.values else [0.0] * 16
                vel = list(aj.velocity.values) if aj.velocity.values else [0.0] * 16
                tor = list(aj.torque.values) if aj.torque.values else [0.0] * 16

                # 提取 12 DOF (跳过 foot)
                # dog 顺序: FR(hip,thigh,calf), FL(hip,thigh,calf), RR(hip,thigh,calf), RL(hip,thigh,calf), FR_foot, FL_foot, RR_foot, RL_foot
                self._latest_joint_pos = pos[:12]
                self._latest_joint_vel = vel[:12]
                self._latest_joint_eff = tor[:12]

            elif joint.HasField('single_joint'):
                sj = joint.single_joint
                # 单关节更新 (id 0-11 → 12 DOF, 12-15 → foot, 跳过)
                if sj.id < 12:
                    self._latest_joint_pos[sj.id] = sj.position
                    self._latest_joint_vel[sj.id] = sj.velocity
                    self._latest_joint_eff[sj.id] = sj.torque

    async def _listen_history(self):
        """订阅 dog 推理历史流 (可选, 用于高级调试/记录)."""
        self.get_logger().info('Starting History stream listener...')
        try:
            async for history in self._stub.ListenHistory(dog_msg.Empty()):
                # History 包含完整的 RL policy 输入输出
                # 目前仅用于日志, 后续可扩展用于状态估计
                pass
        except Exception as e:
            self.get_logger().debug(f'History stream ended: {e}')

    # ================================================================
    #  生命周期
    # ================================================================

    def destroy_node(self):
        """关闭节点时清理."""
        self._shutdown = True
        # 尝试安全停车
        if self._connected and self._standing:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self._safe_shutdown(), self._loop)
                future.result(timeout=5.0)
            except Exception as e:
                self.get_logger().warn(f'Shutdown cleanup failed: {e}')
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._grpc_thread.join(timeout=3.0)
        super().destroy_node()

    async def _safe_shutdown(self):
        """安全关机: 零速 → 坐下 → 禁用."""
        try:
            await self._stub.Walk(dog_msg.Vector3(x=0.0, y=0.0, z=0.0))
            await self._stub.SitDown(dog_msg.Empty())
            await self._stub.Disable(dog_msg.Empty())
            self.get_logger().info('Safe shutdown complete')
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HanDogBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
