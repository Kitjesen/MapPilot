"""Room CLIP feature aggregation tests."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

SRC_ROOT = Path(__file__).resolve().parents[2]
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from perception.tracking.instance_tracker import InstanceTracker, Region, TrackedObject


def _make_feature(seed: int, dim: int = 512) -> np.ndarray:
    rng = np.random.RandomState(seed)
    feature = rng.randn(dim).astype(np.float64)
    return feature / np.linalg.norm(feature)


def _make_tracker_with_objects(objects_per_room: dict) -> InstanceTracker:
    tracker = InstanceTracker()
    object_id = 0
    for room_id, items in objects_per_room.items():
        for label, feature in items:
            obj = TrackedObject(
                object_id=object_id,
                label=label,
                position=np.array([float(room_id), float(object_id), 0.0]),
                best_score=0.9,
            )
            if feature is not None:
                obj.features = feature.copy()
            obj.region_id = room_id
            tracker._objects[object_id] = obj
            object_id += 1
    tracker._next_id = object_id
    return tracker


def _make_regions(objects_per_room: dict) -> list:
    regions = []
    object_id = 0
    for room_id, items in objects_per_room.items():
        object_ids = list(range(object_id, object_id + len(items)))
        regions.append(
            Region(
                region_id=room_id,
                name=f"room_{room_id}",
                center=np.array([float(room_id), 0.0]),
                object_ids=object_ids,
            )
        )
        object_id += len(items)
    return regions


class TestRoomClipAggregation:
    def test_room_clip_feature_aggregated(self):
        f1 = _make_feature(1)
        f2 = _make_feature(2)
        objects_per_room = {
            0: [("chair", f1), ("table", f2)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)

        rooms = tracker.compute_rooms(regions, [])
        assert len(rooms) == 1
        room = rooms[0]

        assert room.clip_feature is not None
        assert room.feature_count == 2
        norm = np.linalg.norm(room.clip_feature)
        assert abs(norm - 1.0) < 1e-6

    def test_room_without_features(self):
        objects_per_room = {
            0: [("chair", None), ("table", None)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)

        rooms = tracker.compute_rooms(regions, [])
        assert rooms[0].clip_feature is None
        assert rooms[0].feature_count == 0

    def test_mixed_features(self):
        f1 = _make_feature(10)
        objects_per_room = {
            0: [("chair", f1), ("table", None), ("lamp", None)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)

        rooms = tracker.compute_rooms(regions, [])
        room = rooms[0]
        assert room.clip_feature is not None
        assert room.feature_count == 1
        cos_sim = float(np.dot(room.clip_feature, f1 / np.linalg.norm(f1)))
        assert cos_sim > 0.999


class TestQueryRoomsByEmbedding:
    def test_query_returns_most_similar(self):
        f_kitchen_1 = _make_feature(100)
        f_kitchen_2 = _make_feature(101)
        f_office_1 = _make_feature(200)
        f_office_2 = _make_feature(201)
        f_bedroom_1 = _make_feature(300)

        objects_per_room = {
            0: [("refrigerator", f_kitchen_1), ("microwave", f_kitchen_2)],
            1: [("computer", f_office_1), ("desk", f_office_2)],
            2: [("bed", f_bedroom_1)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)
        tracker.compute_rooms(regions, [])

        results = tracker.query_rooms_by_embedding(f_kitchen_1, top_k=3)
        assert len(results) == 3
        best_room, best_score = results[0]
        assert best_room.room_id == 0
        assert best_score > 0.5

    def test_query_top_k(self):
        objects_per_room = {
            i: [("obj", _make_feature(i * 10))] for i in range(5)
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)
        tracker.compute_rooms(regions, [])

        results = tracker.query_rooms_by_embedding(_make_feature(0), top_k=2)
        assert len(results) == 2

    def test_query_empty_rooms(self):
        tracker = InstanceTracker()
        results = tracker.query_rooms_by_embedding(_make_feature(0))
        assert results == []

    def test_query_rooms_no_features(self):
        objects_per_room = {
            0: [("chair", None)],
            1: [("table", None)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)
        tracker.compute_rooms(regions, [])

        results = tracker.query_rooms_by_embedding(_make_feature(0))
        assert results == []


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
