#!/usr/bin/env python3
"""
ROS 2 话题频率验证测试
确保关键话题以预期频率发布
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import sys
from collections import defaultdict


class Colors:
    """终端颜色"""
    GREEN = '\033[0;32m'
    RED = '\033[0;31m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'


class TopicHzChecker(Node):
    """话题频率检查器"""

    def __init__(self):
        super().__init__('topic_hz_checker')

        # 定义预期频率（Hz）：(最小值, 最大值)
        self.expected_hz = {
            '/nav/odometry': (8.0, 12.0),      # 期望 10 Hz ± 20%
            '/nav/terrain_map': (0.5, 2.0),    # 期望 1 Hz
            '/nav/path': (0.5, 2.0),           # 期望 1 Hz
            '/cmd_vel': (8.0, 12.0),           # 期望 10 Hz
        }

        self.message_counts = defaultdict(int)
        self.subscribers = {}

        # 创建订阅者
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        for topic in self.expected_hz.keys():
            msg_type = self._get_msg_type(topic)
            if msg_type:
                self.subscribers[topic] = self.create_subscription(
                    msg_type=msg_type,
                    topic=topic,
                    callback=lambda msg, t=topic: self._callback(t),
                    qos_profile=qos
                )

    def _get_msg_type(self, topic):
        """根据话题名获取消息类型"""
        try:
            if 'odometry' in topic.lower():
                from nav_msgs.msg import Odometry
                return Odometry
            elif 'terrain_map' in topic.lower():
                from sensor_msgs.msg import PointCloud2
                return PointCloud2
            elif 'path' in topic.lower():
                from nav_msgs.msg import Path
                return Path
            elif 'cmd_vel' in topic.lower():
                from geometry_msgs.msg import Twist
                return Twist
        except ImportError as e:
            self.get_logger().warn(f"无法导入消息类型: {e}")
            return None

    def _callback(self, topic):
        """消息回调"""
        self.message_counts[topic] += 1

    def check_frequencies(self, duration=10.0):
        """检查频率"""
        print(f"监听话题 {duration:.0f} 秒...")
        print()

        # 重置计数
        self.message_counts.clear()

        # 监听指定时间
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 计算频率
        results = {}
        for topic, (min_hz, max_hz) in self.expected_hz.items():
            count = self.message_counts.get(topic, 0)
            actual_hz = count / duration

            passed = min_hz <= actual_hz <= max_hz
            results[topic] = {
                'count': count,
                'hz': actual_hz,
                'expected': (min_hz, max_hz),
                'passed': passed
            }

        return results


def main():
    """主函数"""
    print("=" * 70)
    print("  ROS 2 话题频率验证测试")
    print("=" * 70)
    print()

    # 初始化 ROS 2
    try:
        rclpy.init()
    except Exception as e:
        print(f"{Colors.RED}❌ ROS 2 初始化失败: {e}{Colors.NC}")
        print()
        print("提示:")
        print("  1. 确保 ROS 2 环境已配置")
        print("  2. 运行: source /opt/ros/humble/setup.bash")
        print("  3. 运行: source install/setup.bash")
        sys.exit(1)

    # 创建检查器
    checker = TopicHzChecker()

    # 检查是否有订阅者
    if not checker.subscribers:
        print(f"{Colors.RED}❌ 无法创建话题订阅者{Colors.NC}")
        print()
        print("提示:")
        print("  1. 确保系统正在运行: make navigation")
        print("  2. 检查话题是否存在: ros2 topic list")
        checker.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"监控 {len(checker.subscribers)} 个话题:")
    for topic in checker.subscribers.keys():
        print(f"  - {topic}")
    print()

    # 检查频率
    results = checker.check_frequencies(duration=10.0)

    # 输出结果
    print()
    print("=" * 70)
    print("  测试结果")
    print("=" * 70)
    print()

    all_passed = True
    for topic, result in results.items():
        status = f"{Colors.GREEN}✓{Colors.NC}" if result['passed'] else f"{Colors.RED}✗{Colors.NC}"
        print(f"{status} {topic}")
        print(f"    实际频率: {result['hz']:.2f} Hz")
        print(f"    预期范围: {result['expected'][0]:.1f} - {result['expected'][1]:.1f} Hz")
        print(f"    消息数量: {result['count']}")

        if not result['passed']:
            all_passed = False
            if result['count'] == 0:
                print(f"    {Colors.YELLOW}⚠ 未收到任何消息{Colors.NC}")
            elif result['hz'] < result['expected'][0]:
                print(f"    {Colors.YELLOW}⚠ 频率过低{Colors.NC}")
            else:
                print(f"    {Colors.YELLOW}⚠ 频率过高{Colors.NC}")

        print()

    print("=" * 70)

    # 清理
    checker.destroy_node()
    rclpy.shutdown()

    # 返回结果
    if all_passed:
        print(f"{Colors.GREEN}✅ 所有话题频率正常{Colors.NC}")
        print()
        sys.exit(0)
    else:
        print(f"{Colors.RED}❌ 部分话题频率异常{Colors.NC}")
        print()
        print("提示:")
        print("  1. 检查系统是否完全启动")
        print("  2. 检查传感器是否正常工作")
        print("  3. 查看日志: ros2 topic hz <topic_name>")
        print()
        sys.exit(1)


if __name__ == '__main__':
    main()
