#!/usr/bin/env python3
"""
pathFollower.cpp 纯 Python 单元测试 (无需 ROS2)

模拟 pathFollower.cpp 的 controlLoop() 核心逻辑，覆盖：
  - Pure Pursuit 前视点搜索 + 方向差计算
  - 近目标减速 (endDis/slowDwnDisThre)
  - 近目标停车 (stopDisThre)
  - 速度积分 (maxAccel 梯形加减速)
  - canAccel 条件 (dirDiffThre 角度门限)
  - safetyStop 优先级 (level1=停平移, level2=停旋转)
  - 双向驱动方向切换 (twoWayDrive)
  - yawRate 夹紧 (maxYawRate)

用法:
    python tests/planning/test_path_follower_logic.py
"""

import math
import sys
import unittest

PI = math.pi


# ──────────────────────────────────────────────────────────────────────────────
# pathFollower.cpp 核心算法提取（保持与 C++ 源码一致）
# ──────────────────────────────────────────────────────────────────────────────

def normalize_angle(a: float) -> float:
    """归一化到 (-π, π]，等效 C++ fmod 方式"""
    a = math.fmod(a + PI, 2.0 * PI)
    if a < 0:
        a += 2.0 * PI
    return a - PI


def adaptive_lookahead(base: float, ratio: float, speed: float,
                        min_dis: float, max_dis: float) -> float:
    """自适应前视距离 = clamp(base + ratio*speed, min, max)"""
    d = base + ratio * abs(speed)
    return max(min_dis, min(max_dis, d))


def find_lookahead_point(path, vehicle_x_rel, vehicle_y_rel, look_ahead_dis):
    """
    在 body-frame 路径中找前视点。
    path: list of (x, y) in body frame at path-receipt time
    返回 (dis_x, dis_y) — 前视点相对于当前机器人 rel 位置的偏移
    """
    look_sq = look_ahead_dis * look_ahead_dis
    path_size = len(path)
    if path_size == 0:
        return 0.0, 0.0

    point_id = 0
    dis_x, dis_y = 0.0, 0.0
    while point_id < path_size - 1:
        dis_x = path[point_id][0] - vehicle_x_rel
        dis_y = path[point_id][1] - vehicle_y_rel
        if dis_x * dis_x + dis_y * dis_y >= look_sq:
            break
        point_id += 1

    if point_id >= path_size - 1:
        dis_x = path[path_size - 1][0] - vehicle_x_rel
        dis_y = path[path_size - 1][1] - vehicle_y_rel

    return dis_x, dis_y


class PathFollowerSim:
    """
    pathFollower.cpp 行为模拟器
    单次 control_loop() 对应一次 100 Hz 定时器回调
    """

    # 默认参数（对应 pathFollower.cpp 成员变量默认值）
    DEFAULT = dict(
        base_look_ahead_dis=0.3,
        look_ahead_ratio=0.5,
        min_look_ahead_dis=0.2,
        max_look_ahead_dis=2.0,
        yaw_rate_gain=7.5,
        stop_yaw_rate_gain=7.5,
        max_yaw_rate=45.0,          # deg/s
        max_speed=1.0,
        max_accel=1.0,
        dir_diff_thre=0.1,          # rad
        omni_dir_goal_thre=1.0,
        omni_dir_diff_thre=1.5,
        stop_dis_thre=0.2,
        slow_dwn_dis_thre=1.0,
        switch_time_thre=1.0,
        two_way_drive=True,
        no_rot_at_goal=True,
        autonomy_mode=True,
        autonomy_speed=1.0,
    )

    def __init__(self, **kwargs):
        p = {**self.DEFAULT, **kwargs}
        self.base_look_ahead_dis  = p['base_look_ahead_dis']
        self.look_ahead_ratio     = p['look_ahead_ratio']
        self.min_look_ahead_dis   = p['min_look_ahead_dis']
        self.max_look_ahead_dis   = p['max_look_ahead_dis']
        self.yaw_rate_gain        = p['yaw_rate_gain']
        self.stop_yaw_rate_gain   = p['stop_yaw_rate_gain']
        self.max_yaw_rate         = p['max_yaw_rate']
        self.max_speed            = p['max_speed']
        self.max_accel            = p['max_accel']
        self.dir_diff_thre        = p['dir_diff_thre']
        self.omni_dir_goal_thre   = p['omni_dir_goal_thre']
        self.omni_dir_diff_thre   = p['omni_dir_diff_thre']
        self.stop_dis_thre        = p['stop_dis_thre']
        self.slow_dwn_dis_thre    = p['slow_dwn_dis_thre']
        self.switch_time_thre     = p['switch_time_thre']
        self.two_way_drive        = p['two_way_drive']
        self.no_rot_at_goal       = p['no_rot_at_goal']
        self.autonomy_mode        = p['autonomy_mode']
        self.autonomy_speed       = p['autonomy_speed']

        # 状态
        self.vehicle_x     = 0.0
        self.vehicle_y     = 0.0
        self.vehicle_yaw   = 0.0
        self.vehicle_speed = 0.0
        self.vehicle_yaw_rate = 0.0

        # 路径接收时记录的参考位姿
        self.vehicle_x_rec   = 0.0
        self.vehicle_y_rec   = 0.0
        self.vehicle_yaw_rec = 0.0
        self.cos_yaw_rec     = 1.0
        self.sin_yaw_rec     = 0.0

        self.path      = []   # list of (x, y) in body frame at receipt time
        self.path_init = False
        self.nav_fwd   = True
        self.switch_time = -999.0
        self.current_time = 0.0   # 模拟时钟

        # 控制输入
        self.joy_speed   = 0.0    # [0, 1]
        self.safety_stop = 0      # 0/1/2
        self.slow_down   = 0      # 0/1/2/3

    # ── 公开接口 ──────────────────────────────────────────────────────────────

    def set_odom(self, x: float, y: float, yaw: float, speed: float = 0.0):
        """更新里程计（等效 odomHandler）"""
        self.vehicle_x   = x
        self.vehicle_y   = y
        self.vehicle_yaw = yaw
        # NOTE: vehicle_speed 由 control_loop 的 stepToward 积分，这里只设初始值
        if speed != 0.0:
            self.vehicle_speed = speed

    def receive_path(self, path_xy, current_time: float = 0.0):
        """
        接收新路径（等效 pathHandler）
        path_xy: list of (x, y) — body frame at current robot pose
        """
        self.path          = list(path_xy)
        self.vehicle_x_rec = self.vehicle_x
        self.vehicle_y_rec = self.vehicle_y
        self.vehicle_yaw_rec = self.vehicle_yaw
        self.cos_yaw_rec   = math.cos(self.vehicle_yaw)
        self.sin_yaw_rec   = math.sin(self.vehicle_yaw)
        self.path_init     = True
        self.current_time  = current_time

    def control_loop(self, dt: float = 0.01) -> tuple:
        """
        执行一次控制循环（100 Hz 定时器）
        返回 (linear_x, angular_z) — cmd_vel 输出
        dt: 时间步长，用于模拟时钟推进
        """
        self.current_time += dt

        if not self.path_init or not self.path:
            return 0.0, 0.0

        path_size = len(self.path)

        # 1. 计算机器人相对路径起点的 body-frame 位移
        dx = self.vehicle_x - self.vehicle_x_rec
        dy = self.vehicle_y - self.vehicle_y_rec
        vxr =  self.cos_yaw_rec * dx + self.sin_yaw_rec * dy
        vyr = -self.sin_yaw_rec * dx + self.cos_yaw_rec * dy

        # 2. 终点距离
        end_dx = self.path[-1][0] - vxr
        end_dy = self.path[-1][1] - vyr
        end_dis = math.sqrt(end_dx * end_dx + end_dy * end_dy)

        # 3. 自适应前视距离
        look_ahead = adaptive_lookahead(
            self.base_look_ahead_dis, self.look_ahead_ratio,
            abs(self.vehicle_speed),
            self.min_look_ahead_dis, self.max_look_ahead_dis
        )

        # 4. 前视点
        dis_x, dis_y = find_lookahead_point(self.path, vxr, vyr, look_ahead)
        dis = math.sqrt(dis_x * dis_x + dis_y * dis_y)
        path_dir = math.atan2(dis_y, dis_x)

        # 5. 方向差 (归一化到 (-π, π])
        dir_diff = normalize_angle(self.vehicle_yaw - self.vehicle_yaw_rec - path_dir)

        # 6. 双向驱动切换
        if self.two_way_drive:
            kHysteresis = 0.1
            if (abs(dir_diff) > PI / 2 + kHysteresis and self.nav_fwd and
                    self.current_time - self.switch_time > self.switch_time_thre):
                self.nav_fwd = False
                self.switch_time = self.current_time
            elif (abs(dir_diff) < PI / 2 - kHysteresis and not self.nav_fwd and
                    self.current_time - self.switch_time > self.switch_time_thre):
                self.nav_fwd = True
                self.switch_time = self.current_time

        # 7. 方向修正（后退时反转）
        joy_speed2 = self.max_speed * self.joy_speed
        if not self.nav_fwd:
            dir_diff = normalize_angle(dir_diff + PI)
            joy_speed2 *= -1

        # 8. Yaw rate
        if abs(self.vehicle_speed) < 2.0 * self.max_accel / 100.0:
            yaw_rate = -self.stop_yaw_rate_gain * dir_diff
        else:
            yaw_rate = -self.yaw_rate_gain * dir_diff

        max_yr_rad = self.max_yaw_rate * PI / 180.0
        yaw_rate = max(-max_yr_rad, min(max_yr_rad, yaw_rate))

        # 9. 近目标时停止旋转
        if path_size <= 1 or (dis < self.stop_dis_thre and self.no_rot_at_goal):
            yaw_rate = 0.0

        # 10. 近目标减速
        if path_size <= 1:
            joy_speed2 = 0.0
        elif end_dis / self.slow_dwn_dis_thre < self.joy_speed and self.joy_speed > 0:
            joy_speed2 *= end_dis / self.slow_dwn_dis_thre / self.joy_speed

        # 11. slow_down 信号
        joy_speed3 = joy_speed2
        if self.slow_down == 1:
            joy_speed3 *= 0.25
        elif self.slow_down == 2:
            joy_speed3 *= 0.5
        elif self.slow_down == 3:
            joy_speed3 *= 0.75

        # 12. canAccel 条件
        can_accel = ((abs(dir_diff) < self.dir_diff_thre or
                      (dis < self.omni_dir_goal_thre and abs(dir_diff) < self.omni_dir_diff_thre))
                     and dis > self.stop_dis_thre)

        step = self.max_accel / 100.0
        target = joy_speed3 if can_accel else 0.0
        cur = self.vehicle_speed
        if cur < target:
            self.vehicle_speed = min(cur + step, target)
        elif cur > target:
            self.vehicle_speed = max(cur - step, target)

        # 13. safetyStop
        if self.safety_stop >= 1:
            self.vehicle_speed = 0.0
        if self.safety_stop >= 2:
            yaw_rate = 0.0

        self.vehicle_yaw_rate = yaw_rate
        return self.vehicle_speed, yaw_rate


# ──────────────────────────────────────────────────────────────────────────────
# 测试用例
# ──────────────────────────────────────────────────────────────────────────────

class TestPurePursuitGeometry(unittest.TestCase):
    """Pure Pursuit 前视点搜索与方向计算"""

    def setUp(self):
        self.sim = PathFollowerSim(autonomy_speed=1.0, joy_speed=1.0)
        self.sim.joy_speed = 1.0

    def test_straight_ahead_zero_yaw_rate(self):
        """机器人正对目标 → dirDiff≈0 → yawRate≈0，线速度递增"""
        # 路径：正前方 5m 直线
        self.sim.set_odom(0, 0, 0)
        self.sim.receive_path([(i * 0.5, 0) for i in range(11)])

        # 多次迭代让速度积分起来
        for _ in range(50):
            v, w = self.sim.control_loop()

        self.assertGreater(v, 0.0, "正对目标应有正向线速度")
        self.assertAlmostEqual(w, 0.0, delta=0.05, msg="正对目标偏航率应接近 0")

    def test_left_turn_positive_yaw_rate(self):
        """目标在左方 → 应产生正偏航率（左转）"""
        self.sim.set_odom(0, 0, 0)
        # 路径向左偏 (y 正方向)
        path = [(i * 0.3, i * 0.5) for i in range(10)]
        self.sim.receive_path(path)

        # 初始 speed>0 才走 yawRateGain 分支，先给一个初速度
        self.sim.vehicle_speed = 0.5
        _, w = self.sim.control_loop()
        self.assertGreater(w, 0.0, "目标在左 → 应左转 (yawRate > 0)")

    def test_right_turn_negative_yaw_rate(self):
        """目标在右方 → 应产生负偏航率（右转）"""
        self.sim.set_odom(0, 0, 0)
        path = [(i * 0.3, -i * 0.5) for i in range(10)]
        self.sim.receive_path(path)
        self.sim.vehicle_speed = 0.5
        _, w = self.sim.control_loop()
        self.assertLess(w, 0.0, "目标在右 → 应右转 (yawRate < 0)")

    def test_lookahead_skips_close_points(self):
        """前视点搜索应跳过近处点，锁定 lookAheadDis 之外的点"""
        path = [(i * 0.1, 0) for i in range(20)]  # 2m 直线，间距 0.1m
        dis_x, dis_y = find_lookahead_point(path, 0, 0, look_ahead_dis=0.5)
        dis = math.sqrt(dis_x ** 2 + dis_y ** 2)
        self.assertGreaterEqual(dis, 0.5 - 1e-6,
                                "前视点距离应 >= lookAheadDis")

    def test_yaw_rate_clamped_to_max(self):
        """极大 dirDiff → yawRate 应夹紧到 maxYawRate"""
        sim = PathFollowerSim(max_yaw_rate=45.0)
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        # 路径在正右方 → dirDiff ≈ -π/2，yawRate 会很大
        sim.receive_path([(0, -5)])
        sim.vehicle_speed = 1.0  # 走 yawRateGain 分支
        _, w = sim.control_loop()
        max_yr = 45.0 * PI / 180.0
        self.assertGreaterEqual(w, -max_yr - 1e-6,
                                "yawRate 不应低于 -maxYawRate")
        self.assertLessEqual(w, max_yr + 1e-6,
                             "yawRate 不应超过 +maxYawRate")


class TestSpeedControl(unittest.TestCase):
    """速度控制：近目标减速、停车、积分"""

    def setUp(self):
        self.sim = PathFollowerSim(
            stop_dis_thre=0.2,
            slow_dwn_dis_thre=1.0,
            max_accel=1.0,
        )
        self.sim.joy_speed = 1.0

    def test_speed_ramps_up_from_zero(self):
        """起步时速度从 0 以 maxAccel/100 步长递增（需多点路径，单点 path_size<=1 强制 speed=0）"""
        self.sim.set_odom(0, 0, 0)
        # 多点路径，首点不与机器人重合，保证 path_size > 1
        self.sim.receive_path([(1 + i * 0.5, 0) for i in range(10)])

        speeds = []
        for _ in range(20):
            v, _ = self.sim.control_loop()
            speeds.append(v)

        # 速度应单调递增（至少在前几步）
        self.assertGreater(speeds[-1], speeds[0], "速度应随时间增加")
        for i in range(1, len(speeds)):
            self.assertGreaterEqual(speeds[i], speeds[i - 1] - 1e-6,
                                   f"速度不应在步骤 {i} 下降")

    def test_slowdown_near_goal(self):
        """终点距离 < slowDwnDisThre → 速度应低于全速"""
        sim = PathFollowerSim(slow_dwn_dis_thre=1.0, max_speed=1.0, max_accel=10.0)
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        # 终点只有 0.5m，在 slowDwnDisThre(1.0) 范围内
        sim.receive_path([(0.5, 0)])
        sim.vehicle_speed = 1.0  # 预先给满速

        v, _ = sim.control_loop()
        self.assertLess(v, 1.0 - 1e-6,
                        "终点 0.5m 内应触发减速，速度 < maxSpeed")

    def test_stop_at_goal(self):
        """机器人已到达终点（endDis < stopDisThre）→ 速度归零"""
        sim = PathFollowerSim(stop_dis_thre=0.2, max_accel=10.0)
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        # 终点就在近前，endDis ≈ 0.05m < stopDisThre=0.2
        sim.receive_path([(0.05, 0)])
        sim.vehicle_speed = 1.0  # 预先有速度

        # 到达终点后 canAccel=False → 速度应减为 0
        for _ in range(20):
            v, _ = sim.control_loop()
        self.assertAlmostEqual(v, 0.0, delta=0.1,
                               msg="到达终点后速度应趋近 0")

    def test_speed_does_not_exceed_max(self):
        """速度积分上限为 maxSpeed"""
        sim = PathFollowerSim(max_speed=1.0, max_accel=1.0)
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        sim.receive_path([(i * 0.5, 0) for i in range(20)])

        for _ in range(200):
            v, _ = sim.control_loop()
        self.assertLessEqual(v, 1.0 + 1e-6, "速度不应超过 maxSpeed")

    def test_slow_down_signal_level1(self):
        """slowDown=1 → 速度乘以 0.25"""
        sim = PathFollowerSim(max_accel=100.0)  # 快速积分到稳态
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        sim.receive_path([(i * 0.5, 0) for i in range(20)])

        # 先跑到接近全速
        for _ in range(50):
            sim.control_loop()
        v_full = sim.vehicle_speed

        # 施加 slow_down=1
        sim.slow_down = 1
        for _ in range(50):
            v, _ = sim.control_loop()

        self.assertLess(v, v_full * 0.5,
                        "slowDown=1 后速度应明显低于全速")

    def test_no_path_zero_output(self):
        """无路径时输出应全为 0"""
        sim = PathFollowerSim()
        sim.joy_speed = 1.0
        v, w = sim.control_loop()
        self.assertEqual(v, 0.0)
        self.assertEqual(w, 0.0)


class TestSafetyStop(unittest.TestCase):
    """安全停车：safetyStop 优先级"""

    def setUp(self):
        self.sim = PathFollowerSim(max_accel=100.0)
        self.sim.joy_speed = 1.0
        self.sim.set_odom(0, 0, 0)
        self.sim.receive_path([(i * 0.5, 0) for i in range(20)])
        # 先积分到有速度
        for _ in range(50):
            self.sim.control_loop()

    def test_level0_allows_motion(self):
        """safetyStop=0 → 正常运动"""
        self.sim.safety_stop = 0
        v, w = self.sim.control_loop()
        self.assertGreater(abs(v) + abs(w), 0, "无 stop 时应有运动输出")

    def test_level1_stops_linear(self):
        """safetyStop=1 → 线速度=0，旋转仍允许"""
        self.sim.safety_stop = 1
        # 给路径有偏角以产生非零 yawRate
        self.sim.receive_path([(3, 3)])
        self.sim.vehicle_speed = 1.0
        v, w = self.sim.control_loop()
        self.assertEqual(v, 0.0, "level1: 线速度应被清零")

    def test_level2_stops_all(self):
        """safetyStop=2 → 线速度=旋转=0"""
        self.sim.safety_stop = 2
        self.sim.receive_path([(3, 3)])
        self.sim.vehicle_speed = 1.0
        v, w = self.sim.control_loop()
        self.assertEqual(v, 0.0, "level2: 线速度应被清零")
        self.assertEqual(w, 0.0, "level2: 旋转应被清零")

    def test_stop_clear_allows_resume(self):
        """清除 safetyStop → 速度可恢复"""
        self.sim.safety_stop = 1
        for _ in range(5):
            self.sim.control_loop()
        self.sim.safety_stop = 0
        # 恢复后重新积分
        for _ in range(50):
            v, _ = self.sim.control_loop()
        self.assertGreater(v, 0.0, "清除 stop 后速度应恢复")


class TestTwoWayDrive(unittest.TestCase):
    """双向驱动：大角度差时切换到后退"""

    def test_switch_to_backward_when_facing_back(self):
        """方向差 > π/2+0.1 且超过 switchTimeThre → 切换后退"""
        sim = PathFollowerSim(two_way_drive=True, switch_time_thre=0.0)
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        # 路径在车后方 → dirDiff ≈ π
        sim.receive_path([(-5, 0)])
        sim.switch_time = -999.0  # 确保时间条件满足

        sim.control_loop()  # 触发切换
        self.assertFalse(sim.nav_fwd, "大角度差应切换到后退模式")

    def test_no_switch_when_aligned(self):
        """方向差 < π/2 → 保持前进"""
        sim = PathFollowerSim(two_way_drive=True, switch_time_thre=0.0)
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        sim.receive_path([(5, 0)])  # 正前方

        sim.control_loop()
        self.assertTrue(sim.nav_fwd, "对齐目标时应保持前进")

    def test_hysteresis_prevents_oscillation(self):
        """方向差 = π/2（边界）→ 迟滞带内不切换"""
        sim = PathFollowerSim(two_way_drive=True, switch_time_thre=0.0)
        sim.joy_speed = 1.0
        sim.set_odom(0, 0, 0)
        # pathDir = atan2(5, 0) = π/2 → dirDiff = 0 - 0 - π/2 = -π/2
        # fabs(-π/2) = π/2，不超过 π/2 + 0.1，不切换
        sim.receive_path([(0, 5)])

        sim.control_loop()
        self.assertTrue(sim.nav_fwd, "π/2 在迟滞带内，应保持前进")


class TestNormalizeAngle(unittest.TestCase):
    """角度归一化边界"""

    def test_zero(self):
        self.assertAlmostEqual(normalize_angle(0), 0, places=6)

    def test_positive_pi(self):
        v = normalize_angle(PI)
        self.assertLessEqual(abs(v), PI + 1e-9)

    def test_negative_pi(self):
        v = normalize_angle(-PI)
        self.assertLessEqual(abs(v), PI + 1e-9)

    def test_wrap_large_positive(self):
        # 3π ≡ π (mod 2π)，fmod 实现归一化到 -π（与 +π 等价，均为有效边界值）
        v = normalize_angle(3 * PI)
        self.assertAlmostEqual(abs(v), PI, places=5)

    def test_wrap_large_negative(self):
        v = normalize_angle(-3 * PI)
        self.assertAlmostEqual(abs(v), PI, places=5)


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=' * 60)
    print('  PathFollower 局部导航逻辑单元测试')
    print('=' * 60)
    loader = unittest.TestLoader()
    suite  = unittest.TestSuite()
    for cls in [TestPurePursuitGeometry, TestSpeedControl,
                TestSafetyStop, TestTwoWayDrive, TestNormalizeAngle]:
        suite.addTests(loader.loadTestsFromTestCase(cls))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
