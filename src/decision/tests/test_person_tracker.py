"""Decision module."""

import time

from decision.vision.person import PersonTracker


def _person_obj(x: float, y: float, z: float = 0.0, conf: float = 0.9, label: str = "person"):
    return {"label": label, "position": [x, y, z], "confidence": conf}


class TestPersonTrackerUpdate:
    def test_update_finds_person(self):
        """Test update finds person."""
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
        """Test update no person returns false."""
        tracker = PersonTracker()
        objects = [
            {"label": "chair", "position": [1, 2, 0], "confidence": 0.8},
            {"label": "table", "position": [3, 4, 0], "confidence": 0.7},
        ]
        assert tracker.update(objects) is False
        assert tracker.get_person_position() is None

    def test_update_picks_highest_confidence(self):
        """Test update picks highest confidence."""
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
        """Test update recognizes synonyms."""
        tracker = PersonTracker()
        for label in ("person", "people", "human", "pedestrian"):
            tracker.reset()
            assert tracker.update([_person_obj(1, 1, label=label)]) is True

    def test_ema_smoothing(self):
        """Test ema smoothing."""
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
        """Test get follow waypoint basic."""
        tracker = PersonTracker(follow_distance=2.0)
        tracker.update([_person_obj(10.0, 0.0)])
        wp = tracker.get_follow_waypoint([0.0, 0.0, 0.0])
        assert wp is not None

        assert wp["x"] < 10.0
        assert wp["x"] > 0.0

    def test_get_follow_waypoint_behind_person(self):
        """Test get follow waypoint behind person."""
        tracker = PersonTracker(follow_distance=1.5)
        tracker.update([_person_obj(5.0, 0.0)])
        robot_pos = [0.0, 0.0, 0.0]
        wp = tracker.get_follow_waypoint(robot_pos)
        assert wp is not None

        # fx = 5 + (-1)*1.5 = 3.5

        assert abs(wp["x"] - 3.5) < 0.5  # tolerance for velocity prediction

    def test_get_follow_waypoint_returns_none_when_lost(self):
        """Test get follow waypoint returns none when lost."""
        tracker = PersonTracker(lost_timeout=0.05)
        tracker.update([_person_obj(5.0, 3.0)])
        time.sleep(0.1)
        wp = tracker.get_follow_waypoint([0.0, 0.0, 0.0])
        assert wp is None

    def test_get_follow_waypoint_returns_none_no_person(self):
        """Test get follow waypoint returns none no person."""
        tracker = PersonTracker()
        assert tracker.get_follow_waypoint([0, 0, 0]) is None

    def test_follow_distance_respected(self):
        """Test follow distance respected."""
        for dist in (1.0, 2.0, 3.0):
            tracker = PersonTracker(follow_distance=dist)
            tracker.update([_person_obj(10.0, 0.0)])
            wp = tracker.get_follow_waypoint([0.0, 0.0, 0.0], predict_dt=0.0)
            assert wp is not None

            assert abs(wp["x"] - (10.0 - dist)) < 0.01


class TestPersonTrackerLostState:
    def test_is_lost_initially(self):
        """Test is lost initially."""
        tracker = PersonTracker()
        assert tracker.is_lost() is True

    def test_not_lost_after_update(self):
        """Test not lost after update."""
        tracker = PersonTracker(lost_timeout=5.0)
        tracker.update([_person_obj(1.0, 2.0)])
        assert tracker.is_lost() is False

    def test_lost_after_timeout(self):
        """Test lost after timeout."""
        tracker = PersonTracker(lost_timeout=0.05)
        tracker.update([_person_obj(1.0, 2.0)])
        assert tracker.is_lost() is False
        time.sleep(0.1)
        assert tracker.is_lost() is True

    def test_reset_clears_state(self):
        """Test reset clears state."""
        tracker = PersonTracker()
        tracker.update([_person_obj(5.0, 5.0)])
        assert tracker.get_person_position() is not None
        tracker.reset()
        assert tracker.get_person_position() is None
        assert tracker.is_lost() is True


class TestVelocityPrediction:
    def test_velocity_prediction(self):
        """Test velocity prediction."""
        tracker = PersonTracker(follow_distance=1.5)

        tracker.update([_person_obj(0.0, 0.0)])
        time.sleep(0.05)

        tracker.update([_person_obj(5.0, 0.0)])

        wp = tracker.get_follow_waypoint([-5.0, 0.0, 0.0], predict_dt=0.5)
        assert wp is not None

        ema_x = tracker.get_person_position()[0]

        assert wp["x"] > ema_x - tracker.follow_distance - 1.0  # loose bound
