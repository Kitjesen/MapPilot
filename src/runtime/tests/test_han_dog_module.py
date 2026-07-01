"""Test ThunderDriver — 验证端口声明、数据流、看门狗逻辑。

不需要 gRPC/brainstem — 纯端口测试。
"""

import time
import unittest

from runtime import Module, Out, autoconnect
from runtime.msgs.geometry import Pose, Twist, Vector3
from runtime.msgs.nav import Odometry
from drivers.real.thunder.han_dog_module import ThunderDriver


class TestPortDeclaration(unittest.TestCase):
    """端口扫描：ThunderDriver 应该有正确的 In/Out。"""

    def test_has_in_ports(self):
        mod = ThunderDriver()
        self.assertIn("cmd_vel", mod.ports_in)
        self.assertIn("stop_signal", mod.ports_in)
        self.assertIn("slam_odom", mod.ports_in)

    def test_has_out_ports(self):
        mod = ThunderDriver()
        self.assertIn("odometry", mod.ports_out)
        self.assertIn("alive", mod.ports_out)

    def test_port_types(self):
        mod = ThunderDriver()
        self.assertEqual(mod.ports_in["cmd_vel"]._msg_type, Twist)
        self.assertEqual(mod.ports_in["stop_signal"]._msg_type, int)
        self.assertEqual(mod.ports_in["slam_odom"]._msg_type, Odometry)
        self.assertEqual(mod.ports_out["odometry"]._msg_type, Odometry)
        self.assertEqual(mod.ports_out["alive"]._msg_type, bool)

    def test_layer(self):
        self.assertEqual(ThunderDriver._layer, 1)


class TestTwistToWalk(unittest.TestCase):
    """归一化转换测试。"""

    def test_zero(self):
        mod = ThunderDriver()
        nx, ny, nz = mod._twist_to_walk(0.0, 0.0, 0.0)
        self.assertEqual((nx, ny, nz), (0.0, 0.0, 0.0))

    def test_max_speed(self):
        mod = ThunderDriver(max_linear_speed=1.0, max_angular_speed=1.0)
        nx, _ny, nz = mod._twist_to_walk(1.0, 0.0, 1.0)
        self.assertAlmostEqual(nx, 1.0)
        self.assertAlmostEqual(nz, 1.0)

    def test_clamp(self):
        mod = ThunderDriver(max_linear_speed=0.5)
        nx, _ny, _nz = mod._twist_to_walk(999.0, 0.0, 0.0)
        self.assertAlmostEqual(nx, 1.0)  # clamped

    def test_negative(self):
        mod = ThunderDriver(max_linear_speed=1.0)
        nx, _ny, _nz = mod._twist_to_walk(-0.5, 0.0, 0.0)
        self.assertAlmostEqual(nx, -0.5)


class TestCmdVelCallback(unittest.TestCase):
    """cmd_vel In 端口 → 内部状态更新。"""

    def test_cmd_vel_updates_state(self):
        mod = ThunderDriver()
        mod.setup()
        twist = Twist(linear=Vector3(0.5, 0.1, 0.0), angular=Vector3(0.0, 0.0, 0.3))
        mod.cmd_vel._deliver(twist)
        self.assertAlmostEqual(mod._cmd_vx, 0.5)
        self.assertAlmostEqual(mod._cmd_vy, 0.1)
        self.assertAlmostEqual(mod._cmd_wz, 0.3)

    def test_cmd_vel_clears_watchdog(self):
        mod = ThunderDriver()
        mod.setup()
        mod._watchdog_triggered = True
        twist = Twist(linear=Vector3(0.1, 0.0, 0.0), angular=Vector3())
        mod.cmd_vel._deliver(twist)
        self.assertFalse(mod._watchdog_triggered)


class TestSlamOdomReset(unittest.TestCase):
    """SLAM 位置重置回调。"""

    def test_resets_position(self):
        mod = ThunderDriver(slam_reset_interval=0.0)  # 立即重置
        mod.setup()
        mod._pos_x = 999.0
        mod._pos_y = 999.0
        odom = Odometry(
            pose=Pose(position=Vector3(5.0, 3.0, 0.0)),
            ts=time.time(), frame_id="map",
        )
        mod.slam_odom._deliver(odom)
        self.assertAlmostEqual(mod._pos_x, 5.0)
        self.assertAlmostEqual(mod._pos_y, 3.0)


class TestOdometryPublish(unittest.TestCase):
    """Odometry Out 端口发布。"""

    def test_publishes_odometry(self):
        mod = ThunderDriver()
        mod.setup()
        received = []
        mod.odometry._add_callback(received.append)

        mod._last_odom_time = time.time() - 0.02  # 20ms ago
        mod._cmd_vx = 0.5
        mod._latest_quat = (0.0, 0.0, 0.0, 1.0)  # facing forward
        mod._publish_odometry()

        self.assertEqual(len(received), 1)
        odom = received[0]
        self.assertIsInstance(odom, Odometry)
        self.assertAlmostEqual(odom.twist.linear.x, 0.5)
        # Position should have moved forward slightly
        self.assertGreater(odom.pose.position.x, 0.0)

    def test_position_integration(self):
        mod = ThunderDriver()
        mod.setup()
        received = []
        mod.odometry._add_callback(received.append)

        mod._pos_x = 0.0
        mod._pos_y = 0.0
        mod._cmd_vx = 1.0  # 1 m/s forward
        mod._latest_quat = (0.0, 0.0, 0.0, 1.0)  # yaw=0
        mod._last_odom_time = time.time() - 0.1  # 100ms ago

        mod._publish_odometry()

        odom = received[0]
        # ~0.1m forward in 100ms at 1 m/s
        self.assertAlmostEqual(odom.pose.position.x, 0.1, places=1)
        self.assertAlmostEqual(odom.pose.position.y, 0.0, places=1)


class TestAutoconnect(unittest.TestCase):
    """ThunderDriver 能通过 autoconnect 和其他模块连线。"""

    def test_wire_with_planner(self):
        """Planner 的 cmd_vel Out → ThunderDriver 的 cmd_vel In。"""
        class FakePlanner(Module, layer=4):
            cmd_vel: Out[Twist]

        handle = autoconnect(
            FakePlanner.blueprint(),
            ThunderDriver.blueprint(dog_host="stub"),
        ).build()
        # setup() registers subscribers — must call before data flows
        handle.start()

        received_odom = []
        dog = handle.modules["ThunderDriver"]
        dog.odometry._add_callback(received_odom.append)
        dog._last_odom_time = time.time() - 0.02

        # Planner 发 cmd_vel → Dog 收到
        planner = handle.modules["FakePlanner"]
        twist = Twist(linear=Vector3(0.3, 0.0, 0.0), angular=Vector3())
        planner.cmd_vel.publish(twist)

        self.assertAlmostEqual(dog._cmd_vx, 0.3)

        # Dog 发布 odometry
        dog._publish_odometry()
        self.assertEqual(len(received_odom), 1)

    def test_health_report(self):
        mod = ThunderDriver()
        h = mod.health()
        self.assertIn("grpc", h)
        self.assertFalse(h["grpc"]["connected"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
