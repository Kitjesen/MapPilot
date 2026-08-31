"""Decision module."""

import math
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

import decision.vision.person as person_module
from decision.vision.person import PersonTracker, TrackedPerson
from decision.vision.reid import OSNetReIDEncoder


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

    def test_different_non_empty_track_id_cannot_replace_the_locked_person_by_distance(self):
        tracker = PersonTracker()
        tracker.lock_target(
            {
                "id": "person-a",
                "label": "person",
                "position": [1.0, 0.0, 0.0],
                "confidence": 0.9,
                "ts": 20.0,
            }
        )

        assert (
            tracker.update(
                [
                    {
                        "id": "person-b",
                        "label": "person",
                        "position": [1.2, 0.0, 0.0],
                        "confidence": 0.9,
                        "ts": 21.0,
                    }
                ]
            )
            is False
        )
        assert tracker._person.obj_id == "person-a"
        assert tracker._person.position == [1.0, 0.0, 0.0]

    def test_changed_track_id_requires_three_consistent_frames_to_reacquire(self):
        tracker = PersonTracker()
        tracker.lock_target(
            {
                "id": "person-a",
                "label": "person",
                "position": [1.0, 0.0, 0.0],
                "confidence": 0.9,
                "ts": 20.0,
            }
        )

        for timestamp in (21.0, 22.0):
            assert tracker.update(
                [
                    {
                        "id": "person-b",
                        "label": "person",
                        "position": [1.1, 0.0, 0.0],
                        "confidence": 0.9,
                        "ts": timestamp,
                    }
                ]
            ) is False
            assert tracker._person.obj_id == "person-a"

        assert tracker.update(
            [
                {
                    "id": "person-b",
                    "label": "person",
                    "position": [1.2, 0.0, 0.0],
                    "confidence": 0.9,
                    "ts": 23.0,
                }
            ]
        ) is True
        assert tracker._person.obj_id == "person-b"

    def test_two_nearby_new_tracks_do_not_replace_the_locked_person(self):
        tracker = PersonTracker()
        tracker.lock_target(
            {
                "id": "person-a",
                "label": "person",
                "position": [1.0, 0.0, 0.0],
                "confidence": 0.9,
                "ts": 20.0,
            }
        )

        for timestamp in (21.0, 22.0, 23.0):
            assert tracker.update(
                [
                    {
                        "id": "person-b",
                        "label": "person",
                        "position": [1.1, 0.0, 0.0],
                        "confidence": 0.9,
                        "ts": timestamp,
                    },
                    {
                        "id": "person-c",
                        "label": "person",
                        "position": [1.2, 0.0, 0.0],
                        "confidence": 0.9,
                        "ts": timestamp,
                    },
                ]
            ) is False

        assert tracker._person.obj_id == "person-a"

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

    def test_person_inside_follow_distance_keeps_the_robot_in_place(self):
        tracker = PersonTracker(follow_distance=1.5)
        tracker.update([_person_obj(1.0, 0.0, 1.2)])

        wp = tracker.get_follow_waypoint([0.0, 0.0, 0.2], predict_dt=0.0)

        assert wp == {"x": 0.0, "y": 0.0, "z": 0.2}


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
    def test_velocity_uses_source_observation_interval_not_callback_time(self, monkeypatch):
        now = [100.0]
        monkeypatch.setattr(person_module.time, "time", lambda: now[0])
        tracker = PersonTracker()
        first = _person_obj(0.0, 0.0)
        first["id"] = "person-1"
        first["ts"] = 20.0
        tracker.update([first])

        now[0] = 100.01
        second = _person_obj(1.0, 0.0)
        second["id"] = "person-1"
        second["ts"] = 21.0
        tracker.update([second])

        assert tracker._person.velocity[0] == 0.3

    def test_non_increasing_source_time_is_not_a_reliable_current_observation(
        self,
        monkeypatch,
    ):
        now = [100.0]
        monkeypatch.setattr(person_module.time, "time", lambda: now[0])
        tracker = PersonTracker()
        tracker.lock_target(
            {
                "id": "person-1",
                "label": "person",
                "position": [1.0, 0.0, 0.0],
                "confidence": 0.9,
                "ts": 20.0,
            }
        )
        original_last_seen = tracker._person.last_seen

        now[0] = 101.0
        assert (
            tracker.update(
                [
                    {
                        "id": "person-1",
                        "label": "person",
                        "position": [9.0, 0.0, 0.0],
                        "confidence": 0.9,
                        "ts": 20.0,
                    }
                ]
            )
            is False
        )

        assert tracker._person.position == [1.0, 0.0, 0.0]
        assert tracker._person.velocity == [0.0, 0.0]
        assert tracker._person.last_seen == original_last_seen

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


class TestOSNetReIDEncoder:
    def test_raises_when_no_backend_is_available(self):
        with (
            patch("decision.vision.reid._try_load_bpu", return_value=None),
            patch("decision.vision.reid._try_load_torchreid", return_value=None),
        ):
            try:
                OSNetReIDEncoder()
            except RuntimeError as exc:
                assert "no backend available" in str(exc)
            else:
                raise AssertionError("OSNetReIDEncoder must reject a missing backend")

    def test_bpu_encode_returns_normalized_512d_feature(self):
        model = MagicMock()
        model.forward.return_value = np.random.default_rng(7).standard_normal(512).astype(np.float32)
        with patch("decision.vision.reid._try_load_bpu", return_value=model):
            encoder = OSNetReIDEncoder()

        feature = encoder.encode(np.zeros((256, 128, 3), dtype=np.uint8))
        assert encoder.backend == "bpu"
        assert encoder.feature_dim == 512
        assert feature.shape == (512,)
        assert feature.dtype == np.float32
        assert float(np.linalg.norm(feature)) == pytest.approx(1.0)


class TestPersonTrackerReID:
    def test_adaptive_threshold_tracks_crowd_density(self):
        tracker = PersonTracker()
        assert tracker._adaptive_reid_threshold(2) == pytest.approx(0.55)
        assert tracker._adaptive_reid_threshold(3) == pytest.approx(0.60)
        assert tracker._adaptive_reid_threshold(5) == pytest.approx(0.70)

    def test_osnet_match_stat_increments_on_reidentification(self):
        tracker = PersonTracker()
        feature = np.ones(512, dtype=np.float32) / math.sqrt(512)
        tracker._osnet_encoder = MagicMock(encode=MagicMock(return_value=feature))
        tracker._person = TrackedPerson(
            position=[1.0, 1.0, 0.0],
            velocity=[0.0, 0.0],
            osnet_feat=feature.copy(),
        )
        candidates = [{
            "id": "unknown",
            "label": "person",
            "position": [6.0, 6.0, 0.0],
            "bbox": [100, 100, 200, 300],
            "confidence": 0.9,
        }]

        tracker._match_person(candidates, np.zeros((480, 640, 3), dtype=np.uint8))
        assert tracker._reid_stats["osnet_match"] == 1

    def test_motion_prediction_advances_by_velocity(self):
        tracker = PersonTracker()
        tracker._person = TrackedPerson(position=[2.0, 3.0, 0.0], velocity=[1.0, -0.5])
        assert tracker._predict_position(dt=0.3) == pytest.approx([2.3, 2.85, 0.0])

    def test_reid_stats_start_at_zero(self):
        assert PersonTracker()._reid_stats == {
            "osnet_match": 0,
            "clip_fallback": 0,
            "motion_dominant": 0,
            "lost": 0,
        }
