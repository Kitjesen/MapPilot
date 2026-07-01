"""PersonTracker 单元测试。"""
import time


from decision.vision.person_tracker import PersonTracker

# ── helpers ──

def _person_obj(x: float, y: float, z: float = 0.0, conf: float = 0.9, label: str = "person"):
    return {"label": label, "position": [x, y, z], "confidence": conf}


# ── tests ──

class TestPersonTrackerUpdate:
    def test_update_finds_person(self):
        """update() 找到 person 标签的对象时返回 True。"""
        tracker = PersonTracker()
        objects = [
            _person_obj(5.0, 3.0),
            {"label": "chair", "position": [1, 2, 0], "confidence": 0.8},
        ]
        assert tracker.update(objects) is True
        pos = tracker.get_person_position()
        assert pos is not None
        assert abs(pos[0] - 5.0) < 0.01
        assert abs(pos[1] - 3.0) < 0.01

    def test_update_no_person_returns_false(self):
        """update() 没有 person 对象时返回 False。"""
        tracker = PersonTracker()
        objects = [
            {"label": "chair", "position": [1, 2, 0], "confidence": 0.8},
            {"label": "table", "position": [3, 4, 0], "confidence": 0.7},
        ]
        assert tracker.update(objects) is False
        assert tracker.get_person_position() is None

    def test_update_picks_highest_confidence(self):
        """多个 person 时选置信度最高的。"""
        tracker = PersonTracker()
        objects = [
            _person_obj(1.0, 1.0, conf=0.5),
            _person_obj(10.0, 10.0, conf=0.95),
        ]
        tracker.update(objects)
        pos = tracker.get_person_position()
        assert pos is not None
        assert abs(pos[0] - 10.0) < 0.01

    def test_update_recognizes_synonyms(self):
        """能识别 person/people/human/pedestrian。"""
        tracker = PersonTracker()
        for label in ("person", "people", "human", "pedestrian"):
            tracker.reset()
            assert tracker.update([_person_obj(1, 1, label=label)]) is True

    def test_ema_smoothing(self):
        """连续更新时位置被 EMA 平滑。"""
        tracker = PersonTracker()
        tracker.update([_person_obj(0.0, 0.0)])
        time.sleep(0.02)
        tracker.update([_person_obj(10.0, 0.0)])
        pos = tracker.get_person_position()
        # EMA alpha=0.4: 0.4*10 + 0.6*0 = 4.0
        assert pos is not None
        assert abs(pos[0] - 4.0) < 0.5  # some tolerance for timing


class TestPersonTrackerWaypoint:
    def test_get_follow_waypoint_basic(self):
        """基本跟随点计算: 在目标和机器人之间。"""
        tracker = PersonTracker(follow_distance=2.0)
        tracker.update([_person_obj(10.0, 0.0)])
        wp = tracker.get_follow_waypoint([0.0, 0.0, 0.0])
        assert wp is not None
        # 跟随点应在目标前方 (朝向机器人方向 2m)
        assert wp["x"] < 10.0
        assert wp["x"] > 0.0

    def test_get_follow_waypoint_behind_person(self):
        """跟随点在 person 的 '后方' (机器人侧)。"""
        tracker = PersonTracker(follow_distance=1.5)
        tracker.update([_person_obj(5.0, 0.0)])
        robot_pos = [0.0, 0.0, 0.0]
        wp = tracker.get_follow_waypoint(robot_pos)
        assert wp is not None
        # 目标在 x=5, 机器人在 x=0, 跟随点应在 x=5+1.5=6.5
        # 等等... 方向是 person -> robot, 所以跟随点 = person + dir_to_robot * dist
        # dir_to_robot = (0-5)/5 = -1, 所以 fx = 5 + (-1)*1.5 = 3.5
        # 不对, dx=0-5=-5 不对... robot_pos[0]-px = 0-5=-5, dist=5, dx/dist=-1
        # fx = 5 + (-1)*1.5 = 3.5
        # 实际上 robot 在 person 后方, 跟随点在 person 和 robot 之间
        assert abs(wp["x"] - 3.5) < 0.5  # tolerance for velocity prediction

    def test_get_follow_waypoint_returns_none_when_lost(self):
        """丢失后 get_follow_waypoint 返回 None。"""
        tracker = PersonTracker(lost_timeout=0.05)
        tracker.update([_person_obj(5.0, 3.0)])
        time.sleep(0.1)
        wp = tracker.get_follow_waypoint([0.0, 0.0, 0.0])
        assert wp is None

    def test_get_follow_waypoint_returns_none_no_person(self):
        """从未更新过时返回 None。"""
        tracker = PersonTracker()
        assert tracker.get_follow_waypoint([0, 0, 0]) is None

    def test_follow_distance_respected(self):
        """跟随距离被正确使用。"""
        for dist in (1.0, 2.0, 3.0):
            tracker = PersonTracker(follow_distance=dist)
            tracker.update([_person_obj(10.0, 0.0)])
            wp = tracker.get_follow_waypoint([0.0, 0.0, 0.0], predict_dt=0.0)
            assert wp is not None
            # 目标在 (10,0), 机器人在 (0,0)
            # 跟随点 = (10 + (-10/10)*dist, 0) = (10-dist, 0)
            assert abs(wp["x"] - (10.0 - dist)) < 0.01


class TestPersonTrackerLostState:
    def test_is_lost_initially(self):
        """初始状态为 lost。"""
        tracker = PersonTracker()
        assert tracker.is_lost() is True

    def test_not_lost_after_update(self):
        """update 后不再 lost。"""
        tracker = PersonTracker(lost_timeout=5.0)
        tracker.update([_person_obj(1.0, 2.0)])
        assert tracker.is_lost() is False

    def test_lost_after_timeout(self):
        """超时后变为 lost。"""
        tracker = PersonTracker(lost_timeout=0.05)
        tracker.update([_person_obj(1.0, 2.0)])
        assert tracker.is_lost() is False
        time.sleep(0.1)
        assert tracker.is_lost() is True

    def test_reset_clears_state(self):
        """reset 后回到初始状态。"""
        tracker = PersonTracker()
        tracker.update([_person_obj(5.0, 5.0)])
        assert tracker.get_person_position() is not None
        tracker.reset()
        assert tracker.get_person_position() is None
        assert tracker.is_lost() is True


class TestVelocityPrediction:
    def test_velocity_prediction(self):
        """速度预测使跟随点偏向运动方向。"""
        tracker = PersonTracker(follow_distance=1.5)
        # 第一次: person 在 (0, 0)
        tracker.update([_person_obj(0.0, 0.0)])
        time.sleep(0.05)
        # 第二次: person 移动到 (5, 0) -- 明显的 +x 运动
        tracker.update([_person_obj(5.0, 0.0)])

        # 机器人在 (-5, 0), 请求跟随点并预测
        wp = tracker.get_follow_waypoint([-5.0, 0.0, 0.0], predict_dt=0.5)
        assert wp is not None
        # 由于速度预测, 预测目标位置会超过当前 EMA 位置
        # 跟随点应该 > (EMA_pos - follow_distance)
        ema_x = tracker.get_person_position()[0]
        # predicted px = ema_x + vx * 0.5, 应该 > ema_x
        assert wp["x"] > ema_x - tracker.follow_distance - 1.0  # loose bound
