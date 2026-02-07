#!/usr/bin/env python3
"""
Generic Robot Driver Node with Independent Watchdog
----------------------------------------------------
最底层安全保障: 即使上层所有节点 (pathFollower, SafetyGate, remote_monitoring) 全部崩溃,
driver 自身的 watchdog 仍能在 cmd_vel_timeout_ms 内将机器人停车。

设计原则:
  - 解耦: 不依赖任何上层模块, 只订阅 /cmd_vel
  - 自保护: 超时未收到指令 → 零速
  - 透明: 发布 /driver/watchdog_active 供外部监控
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler
from math import sin, cos


class GenericRobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')

        # --- Configuration Parameters ---
        self.declare_parameter('enable_odom_tf', False)
        self.declare_parameter('control_rate', 50.0)       # Hz
        self.declare_parameter('robot_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_timeout_ms', 200.0) # 看门狗超时 (ms)

        self._control_rate = self.get_parameter('control_rate').value
        self._watchdog_timeout_sec = (
            self.get_parameter('cmd_vel_timeout_ms').value / 1000.0
        )

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.watchdog_pub = self.create_publisher(Bool, '/driver/watchdog_active', 10)

        # --- Subscribers ---
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # --- Internal State ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()

        # --- Watchdog State (Layer 1 - 独立于所有上层) ---
        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._watchdog_triggered = False

        # --- Hardware Connection ---
        self.connect_hardware()

        # Main control loop (reads hardware + enforces watchdog)
        self.timer = self.create_timer(
            1.0 / self._control_rate, self.hardware_loop
        )

        self.get_logger().info(
            f'Robot Driver initialized (watchdog={self._watchdog_timeout_sec*1000:.0f}ms)'
        )

    # ================================================================
    #  Layer 1: Watchdog - 独立 cmd_vel 超时保护
    # ================================================================

    def cmd_vel_callback(self, msg):
        """收到速度指令 → 更新看门狗时间戳 + 缓存指令."""
        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vx = msg.twist.linear.x
        self._cmd_vy = msg.twist.linear.y
        self._cmd_wz = msg.twist.angular.z

        if self._watchdog_triggered:
            self._watchdog_triggered = False
            self.get_logger().info('Watchdog cleared: cmd_vel resumed')

    def _check_watchdog(self):
        """检查 cmd_vel 是否超时. 超时则强制零速."""
        elapsed = (
            self.get_clock().now() - self._last_cmd_vel_time
        ).nanoseconds / 1e9

        if elapsed > self._watchdog_timeout_sec:
            if not self._watchdog_triggered:
                self._watchdog_triggered = True
                self.get_logger().warn(
                    f'WATCHDOG: No cmd_vel for {elapsed*1000:.0f}ms, '
                    f'forcing zero velocity'
                )
            # 强制零速 — 不依赖任何上层模块
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
            self._cmd_wz = 0.0

        # 发布看门狗状态供外部监控
        wd_msg = Bool()
        wd_msg.data = self._watchdog_triggered
        self.watchdog_pub.publish(wd_msg)

    # ================================================================
    #  Hardware Interface (模板方法，按实际硬件实现)
    # ================================================================

    def connect_hardware(self):
        """[IMPLEMENT] 初始化串口/TCP/CAN 连接."""
        port = self.get_parameter('robot_port').value
        baud = self.get_parameter('baudrate').value
        self.get_logger().info(f'Connecting to hardware on {port} at {baud}...')

    def send_to_motors(self, vx, vy, wz):
        """[IMPLEMENT] 将 m/s, rad/s 转换为电机指令."""
        pass

    def hardware_loop(self):
        """主控制循环: 看门狗检查 → 发送指令 → 读取反馈."""
        # 1. 看门狗检查 (最高优先级)
        self._check_watchdog()

        # 2. 发送当前指令到电机 (可能已被看门狗清零)
        self.send_to_motors(self._cmd_vx, self._cmd_vy, self._cmd_wz)

        # 3. 读取硬件反馈
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # [IMPLEMENT] 用编码器实际值替换
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        self.publish_odometry(current_time)
        self.last_time = current_time

    def publish_odometry(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GenericRobotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
