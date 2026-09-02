# ruff: noqa: S101  (assert statements are standard in pytest test files)
"""
test_golden_characterization.py

Golden snapshot regression tests for the perception tracking pipeline.

Purpose:
  Freeze the current behavior of Detection3D -> InstanceTracker -> SceneGraph
  so the upcoming perception module refactoring can be verified as
  behavior-preserving ("characterization tests" / "golden master").

Determinism guarantees:
  - One-hot feature vectors (cosine similarity is exactly 0.0 or 1.0)
  - time.time() patched to a constant (all time-dependent fields frozen)
  - No point clouds in tracking tests (avoids random voxel downsampling)
  - No KG injected (BP phases 1/2 are no-ops; phase 3 lateral sharing is
    deterministic given frozen time)
  - DBSCAN/sklearn clustering is deterministic given fixed positions

These tests do NOT use real ML models, GPU, ROS2, or hardware.
"""

import json
import math
import os
import time
from unittest.mock import patch

import numpy as np
import pytest

from perception.tracking.instance_tracker import InstanceTracker
from perception.tracking.projection import (
    CameraIntrinsics,
    Detection3D,
    bbox_center_depth,
    mask_to_pointcloud,
    pointcloud_centroid,
    project_to_3d,
    transform_point,
)
from perception.tracking.tracked_objects import TrackedObject

# ─────────────────────────────────────────────────────────────────────────────
#  Determinism helpers
# ─────────────────────────────────────────────────────────────────────────────

FIXED_TIME = 1_000_000.0

# Fixed label -> feature index mapping (avoids Python hash randomization).
# Different labels map to orthogonal one-hot vectors so cosine similarity
# between different labels is exactly 0.0 (never matches SEM_THRESHOLD=0.75).
_LABEL_FIDX = {
    "chair": 0,
    "desk": 1,
    "monitor": 2,
    "keyboard": 3,
    "door": 4,
    "sofa": 5,
    "table": 6,
    "refrigerator": 7,
    "toilet": 8,
    "stairs": 9,
    "mouse": 10,
    "shelf": 11,
    "cabinet": 12,
    "bed": 13,
    "sink": 14,
}


def _onehot(idx: int, dim: int = 512) -> np.ndarray:
    """Unit-norm one-hot feature vector (cosine sim is 0.0 or 1.0)."""
    v = np.zeros(dim, dtype=np.float32)
    v[idx] = 1.0
    return v


def _make_det(
    label: str,
    pos: tuple[float, float, float],
    score: float = 0.9,
    depth: float | None = None,
    bbox: tuple[int, int, int, int] = (10, 10, 100, 100),
) -> Detection3D:
    """Construct a deterministic Detection3D for golden tests."""
    fidx = _LABEL_FIDX.get(label, 0)
    features = _onehot(fidx)
    if depth is None:
        depth = float(np.linalg.norm(np.array(pos[:2])))
    return Detection3D(
        position=np.array(pos, dtype=np.float32),
        label=label,
        score=score,
        bbox_2d=np.array(bbox, dtype=np.float32),
        depth=depth,
        features=features,
    )


def _frozen_time():
    """Context manager that freezes time.time() to FIXED_TIME."""
    return patch("time.time", return_value=FIXED_TIME)


def _normalize_scene_graph(sg: dict) -> dict:
    """Zero out timestamps so the snapshot is purely behavioral."""
    sg = json.loads(json.dumps(sg))  # deep copy via JSON round-trip
    sg["timestamp"] = 0.0
    for obj in sg.get("objects", []):
        obj["last_observed_time"] = 0.0
    for v in sg.get("views", []):
        v["timestamp"] = 0.0
    return sg


# ─────────────────────────────────────────────────────────────────────────────
#  Test: Projection (2D -> 3D)
# ─────────────────────────────────────────────────────────────────────────────


class TestProjectionGolden:
    """Golden snapshot of 2D->3D projection primitives."""

    def test_project_to_3d_center(self):
        """Principal point at 2 m depth -> [0, 0, 2]."""
        K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)
        p = project_to_3d(320.0, 240.0, 2.0, K)
        np.testing.assert_allclose(p, [0.0, 0.0, 2.0], rtol=1e-10)

    def test_project_to_3d_off_center(self):
        """Off-center pixel at 3 m depth."""
        K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)
        p = project_to_3d(100.0, 200.0, 3.0, K)
        # x = (100 - 320) * 3 / 600 = -1.1
        # y = (200 - 240) * 3 / 600 = -0.2
        np.testing.assert_allclose(p, [-1.1, -0.2, 3.0], rtol=1e-10)

    def test_project_to_3d_zero_focal(self):
        """Zero focal length falls back to [0, 0, depth]."""
        K = CameraIntrinsics(fx=0.0, fy=0.0, cx=320.0, cy=240.0, width=640, height=480)
        p = project_to_3d(100.0, 200.0, 5.0, K)
        np.testing.assert_allclose(p, [0.0, 0.0, 5.0], rtol=1e-10)

    def test_transform_point(self):
        """Camera-frame point -> world frame via 4x4 transform."""
        K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)
        p_cam = project_to_3d(320.0, 240.0, 2.0, K)
        tf = np.eye(4)
        tf[0, 3] = 10.0
        tf[1, 3] = 5.0
        p_world = transform_point(p_cam, tf)
        np.testing.assert_allclose(p_world, [10.0, 5.0, 2.0], rtol=1e-10)

    def test_mask_to_pointcloud(self):
        """Back-project a 40x40 mask at uniform 2 m depth.

        Uses 1600 pixels (< 2048 threshold) to avoid the random pre-downsample
        path in mask_to_pointcloud, ensuring full determinism.  Voxel
        downsampling (0.02 m) of the planar patch yields a deterministic
        point count and centroid.
        """
        K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)
        depth = np.full((480, 640), 2000, dtype=np.uint16)  # 2 m
        mask = np.zeros((480, 640), dtype=bool)
        mask[200:240, 300:340] = True  # 40x40 = 1600 pixels (< 2048)

        tf = np.eye(4)
        tf[0, 3] = 10.0
        tf[1, 3] = 5.0

        pc = mask_to_pointcloud(mask, depth, K, tf)
        assert pc is not None
        assert len(pc) == 56
        centroid = pointcloud_centroid(pc)
        np.testing.assert_allclose(centroid, [9.9929, 4.9210, 2.0], atol=1e-3)

    def test_mask_to_pointcloud_too_few_pixels(self):
        """Mask with fewer than POINTCLOUD_MIN_POINTS pixels -> None."""
        K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)
        depth = np.full((480, 640), 2000, dtype=np.uint16)
        mask = np.zeros((480, 640), dtype=bool)
        mask[200:202, 300:302] = True  # 4 pixels < 10 minimum

        tf = np.eye(4)
        pc = mask_to_pointcloud(mask, depth, K, tf)
        assert pc is None

    def test_bbox_center_depth(self):
        """Median depth of the bbox center region."""
        depth = np.full((480, 640), 0, dtype=np.uint16)
        depth[200:250, 300:350] = 1500  # 1.5 m
        d = bbox_center_depth(depth, np.array([290, 190, 360, 260]))
        assert d == 1.5

    def test_bbox_center_depth_no_valid(self):
        """All-zero depth in bbox center -> None."""
        depth = np.zeros((480, 640), dtype=np.uint16)
        d = bbox_center_depth(depth, np.array([290, 190, 360, 260]))
        assert d is None


# ─────────────────────────────────────────────────────────────────────────────
#  Test: Tracking (InstanceTracker create / match / dedup)
# ─────────────────────────────────────────────────────────────────────────────


class TestTrackingGolden:
    """Golden snapshot of InstanceTracker create / match / dedup behavior."""

    def test_single_object_creation(self):
        """One Detection3D -> one TrackedObject with expected initial state."""
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            result = tracker.update([_make_det("chair", (1.0, 0.0, 0.5))])

        assert len(tracker.objects) == 1
        assert len(result) == 1
        obj = result[0]
        assert obj.object_id == 0
        assert obj.label == "chair"
        assert obj.detection_count == 1
        assert obj.best_score == 0.9
        assert obj.belief_alpha == 1.5
        assert obj.belief_beta == 1.0
        assert obj.existence_prob == pytest.approx(0.6)  # 1.5 / 2.5
        assert obj.credibility == pytest.approx(0.495)
        assert obj.position_variance == pytest.approx(1.0)
        np.testing.assert_allclose(obj.position, [1.0, 0.0, 0.5], rtol=1e-6)

    def test_multi_object_creation(self):
        """Three different labels -> three objects with incrementing IDs."""
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            dets = [
                _make_det("chair", (1.0, 0.0, 0.5)),
                _make_det("desk", (5.0, 0.0, 0.5)),
                _make_det("monitor", (10.0, 0.0, 0.5)),
            ]
            tracker.update(dets)

        assert len(tracker.objects) == 3
        labels = {o.label for o in tracker.objects.values()}
        assert labels == {"chair", "desk", "monitor"}

    def test_multi_frame_merge(self):
        """Same objects in frame 2 -> merge (same IDs, detection_count=2)."""
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            frame1 = [
                _make_det("chair", (1.0, 0.0, 0.5)),
                _make_det("desk", (5.0, 0.0, 0.5)),
            ]
            frame2 = [
                _make_det("chair", (1.1, 0.0, 0.5)),
                _make_det("desk", (5.1, 0.0, 0.5)),
            ]
            tracker.update(frame1)
            ids_frame1 = set(tracker.objects.keys())
            tracker.update(frame2)

        assert len(tracker.objects) == 2
        assert set(tracker.objects.keys()) == ids_frame1
        for obj in tracker.objects.values():
            assert obj.detection_count == 2

    def test_same_label_same_feature_merges(self):
        """Same label + identical one-hot features + close position -> merge.

        Cosine similarity of identical features = 1.0 > SEM_THRESHOLD (0.75),
        and geometric similarity (1 - dist/merge_distance) > GEO_WEAK (0.1),
        so the semantic-match priority path fires.
        """
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            tracker.update([_make_det("chair", (1.0, 0.0, 0.5))])
            tracker.update([_make_det("chair", (1.1, 0.0, 0.5))])

        assert len(tracker.objects) == 1

    def test_different_labels_do_not_merge(self):
        """Different labels at close positions -> separate objects.

        Orthogonal features -> cosine sim = 0.0 < 0.75 (no semantic match).
        Fallback requires same label, so no match either.
        """
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            tracker.update(
                [
                    _make_det("chair", (1.0, 0.0, 0.5)),
                    _make_det("desk", (1.2, 0.0, 0.5)),
                ]
            )

        assert len(tracker.objects) == 2

    def test_far_same_label_does_not_merge(self):
        """Same label far apart -> separate objects.

        Beyond CANDIDATE_RADIUS (2.0 m) so no candidates are considered.
        """
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            tracker.update([_make_det("chair", (0.0, 0.0, 0.5))])
            tracker.update([_make_det("chair", (20.0, 0.0, 0.5))])

        assert len(tracker.objects) == 2

    def test_empty_detections(self):
        """Empty detection list -> no objects, no crash."""
        with _frozen_time():
            tracker = InstanceTracker()
            tracker.update([])

        assert len(tracker.objects) == 0

    def test_merged_position_updates(self):
        """After merge, position is a variance-weighted average (BA-HSG).

        The observation variance is much smaller than the prior (1.0),
        so the fused position is pulled strongly toward the new observation.
        """
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            tracker.update([_make_det("chair", (1.0, 0.0, 0.5))])
            result = tracker.update([_make_det("chair", (1.1, 0.0, 0.5))])

        obj = result[0]
        assert obj.detection_count == 2
        assert obj.belief_alpha == pytest.approx(2.5)  # 1.5 + 1.0
        np.testing.assert_allclose(obj.position, [1.0995, 0.0, 0.5], atol=1e-3)

    def test_tracked_object_belief_dict(self):
        """TrackedObject.to_belief_dict() golden snapshot."""
        with _frozen_time():
            tracker = InstanceTracker()
            result = tracker.update([_make_det("chair", (1.0, 0.0, 0.5))])
            obj = result[0]

        bd = obj.to_belief_dict()
        assert bd["P_exist"] == pytest.approx(0.6)
        assert bd["detections"] == 1
        assert bd["miss_streak"] == 0
        assert bd["confirmed_nav"] is True  # credibility 0.495 >= 0.25
        assert bd["confirmed_interact"] is True  # 0.495 >= 0.40

    def test_tracked_object_record_miss(self):
        """record_miss increments miss_streak and belief_beta."""
        with _frozen_time():
            tracker = InstanceTracker()
            result = tracker.update([_make_det("chair", (1.0, 0.0, 0.5))])
            obj = result[0]
            beta_before = obj.belief_beta
            obj.record_miss()

        assert obj.miss_streak == 1
        assert obj.belief_beta > beta_before

    def test_incrementing_ids(self):
        """Object IDs increment from 0."""
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            tracker.update(
                [
                    _make_det("chair", (0.0, 0.0, 0.5)),
                    _make_det("desk", (5.0, 0.0, 0.5)),
                ]
            )

        ids = sorted(tracker.objects.keys())
        assert ids == [0, 1]


# ─────────────────────────────────────────────────────────────────────────────
#  Test: Scene Graph Golden Snapshot
# ─────────────────────────────────────────────────────────────────────────────


class TestSceneGraphGolden:
    """Golden snapshot of the full scene graph JSON output.

    The canonical scene: 5 objects (chair, desk, monitor, keyboard, door)
    fed across 2 frames.  Four office objects cluster in region 0; the
    isolated door forms region 1.  All assertions below are exact golden
    values captured from the current pipeline behavior.
    """

    @pytest.fixture
    def scene_graph(self) -> dict:
        """Build the canonical scene and return the normalized scene graph."""
        with _frozen_time():
            tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)
            frame1 = [
                _make_det("chair", (1.0, 0.5, 0.4)),
                _make_det("desk", (1.2, 0.3, 0.5)),
                _make_det("monitor", (1.3, 0.4, 1.2)),
                _make_det("keyboard", (1.1, 0.2, 0.8)),
                _make_det("door", (5.0, 0.0, 1.0)),
            ]
            frame2 = [
                _make_det("chair", (1.05, 0.52, 0.4)),
                _make_det("desk", (1.22, 0.31, 0.5)),
                _make_det("monitor", (1.31, 0.41, 1.2)),
                _make_det("keyboard", (1.12, 0.22, 0.8)),
                _make_det("door", (5.02, 0.01, 1.0)),
            ]
            tracker.update(frame1)
            tracker.update(frame2)
            sg = json.loads(tracker.get_scene_graph_json())

        return _normalize_scene_graph(sg)

    # ── Object-level golden ──

    def test_object_count(self, scene_graph):
        assert scene_graph["object_count"] == 5
        assert len(scene_graph["objects"]) == 5

    def test_object_ids_and_labels(self, scene_graph):
        expected = [
            (0, "chair"),
            (1, "desk"),
            (2, "monitor"),
            (3, "keyboard"),
            (4, "door"),
        ]
        actual = [(o["id"], o["label"]) for o in scene_graph["objects"]]
        assert actual == expected

    def test_object_positions(self, scene_graph):
        """Positions are variance-weighted averages of 2 frames."""
        expected_positions = {
            0: (1.05, 0.52, 0.4),
            1: (1.22, 0.31, 0.5),
            2: (1.31, 0.41, 1.2),
            3: (1.12, 0.22, 0.8),
            4: (5.02, 0.01, 1.0),
        }
        for obj in scene_graph["objects"]:
            exp = expected_positions[obj["id"]]
            assert obj["position"]["x"] == pytest.approx(exp[0], abs=0.001)
            assert obj["position"]["y"] == pytest.approx(exp[1], abs=0.001)
            assert obj["position"]["z"] == pytest.approx(exp[2], abs=0.001)

    def test_detection_counts(self, scene_graph):
        for obj in scene_graph["objects"]:
            assert obj["detection_count"] == 2

    def test_region_assignment(self, scene_graph):
        """4 office objects -> region 0; door -> region 1."""
        region_map = {o["id"]: o["region_id"] for o in scene_graph["objects"]}
        assert region_map == {0: 0, 1: 0, 2: 0, 3: 0, 4: 1}

    def test_belief_state(self, scene_graph):
        """2 detections -> P_exist~0.714, credibility~0.61, confirmed."""
        for obj in scene_graph["objects"]:
            b = obj["belief"]
            assert b["P_exist"] == pytest.approx(0.714, abs=0.001)
            assert b["detections"] == 2
            assert b["miss_streak"] == 0
            assert b["confirmed_nav"] is True
            assert b["confirmed_interact"] is True

    # ── Relations golden ──

    def test_relation_count(self, scene_graph):
        """12 spatial relations among the 4 office objects."""
        assert len(scene_graph["relations"]) == 12

    def test_relation_types(self, scene_graph):
        rel_types = {r["relation"] for r in scene_graph["relations"]}
        assert rel_types == {"near", "on", "below"}

    def test_relation_golden(self, scene_graph):
        """Exact golden snapshot of all 12 relations."""
        expected = [
            {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 0.29},
            {"subject_id": 1, "relation": "on", "object_id": 0, "distance": 0.23},
            {"subject_id": 0, "relation": "near", "object_id": 2, "distance": 0.85},
            {"subject_id": 0, "relation": "below", "object_id": 2, "distance": 0.46},
            {"subject_id": 0, "relation": "near", "object_id": 3, "distance": 0.5},
            {"subject_id": 3, "relation": "on", "object_id": 0, "distance": 0.07},
            {"subject_id": 1, "relation": "near", "object_id": 2, "distance": 0.71},
            {"subject_id": 1, "relation": "below", "object_id": 2, "distance": 0.36},
            {"subject_id": 1, "relation": "near", "object_id": 3, "distance": 0.33},
            {"subject_id": 3, "relation": "on", "object_id": 1, "distance": 0.03},
            {"subject_id": 2, "relation": "near", "object_id": 3, "distance": 0.48},
            {"subject_id": 2, "relation": "on", "object_id": 3, "distance": 0.06},
        ]
        assert scene_graph["relations"] == expected

    # ── Regions / Rooms / Groups golden ──

    def test_regions(self, scene_graph):
        regions = scene_graph["regions"]
        assert len(regions) == 2
        assert regions[0]["object_ids"] == [0, 1, 2, 3]
        assert regions[0]["name"] == "office"
        assert regions[1]["object_ids"] == [4]
        assert regions[1]["name"] == "corridor"

    def test_rooms(self, scene_graph):
        rooms = scene_graph["rooms"]
        assert len(rooms) == 2
        assert rooms[0]["name"] == "office"
        assert rooms[0]["object_ids"] == [0, 1, 2, 3]
        assert rooms[1]["name"] == "corridor"
        assert rooms[1]["object_ids"] == [4]

    def test_groups(self, scene_graph):
        groups = scene_graph["groups"]
        assert len(groups) == 4
        gmap = {g["name"]: g["object_ids"] for g in groups}
        assert gmap["furniture"] == [0, 1]
        assert gmap["electronics"] == [2]
        assert gmap["others"] == [3]
        assert gmap["structure"] == [4]

    # ── Topology / Floors golden ──

    def test_topology_edges(self, scene_graph):
        edges = scene_graph["topology_edges"]
        assert len(edges) == 1
        e = edges[0]
        assert e["from_room"] == 0
        assert e["to_room"] == 1
        assert e["type"] == "door"
        assert e["distance"] == pytest.approx(3.86, abs=0.01)

    def test_floors(self, scene_graph):
        floors = scene_graph["floors"]
        assert len(floors) == 1
        f = floors[0]
        assert f["floor_level"] == 0
        assert f["z_range"] == [0.4, 1.2]
        assert f["room_ids"] == [0, 1]
        assert f["object_count"] == 5

    # ── Hierarchy / Subgraphs golden ──

    def test_hierarchy_edges(self, scene_graph):
        """9 edges: 4 room->group + 5 group->object."""
        edges = scene_graph["hierarchy_edges"]
        assert len(edges) == 9
        assert len([e for e in edges if e["parent_type"] == "room"]) == 4
        assert len([e for e in edges if e["parent_type"] == "group"]) == 5

    def test_subgraphs(self, scene_graph):
        """6 subgraphs: 2 room-level + 4 group-level."""
        subs = scene_graph["subgraphs"]
        assert len(subs) == 6
        assert len([s for s in subs if s["level"] == "room"]) == 2
        assert len([s for s in subs if s["level"] == "group"]) == 4

    # ── KG / Belief propagation golden ──

    def test_kg_stats(self, scene_graph):
        assert scene_graph["kg_stats"] == {"enriched": 0, "dangerous": 0, "graspable": 0}

    def test_phantom_and_posteriors(self, scene_graph):
        """No KG -> no phantom nodes, no room type posteriors."""
        assert scene_graph["phantom_nodes"] == []
        assert scene_graph["room_type_posteriors"] == {}

    def test_belief_propagation(self, scene_graph):
        """BP ran 1 iteration and converged immediately (delta=0)."""
        bp = scene_graph["belief_propagation"]
        assert bp["iterations"] == 1
        assert bp["convergence"] == [0.0]

    # ── Full JSON golden ──

    def test_full_json_matches_fixture(self, scene_graph):
        """Complete normalized scene graph matches the golden JSON fixture."""
        fixture_path = os.path.join(
            os.path.dirname(__file__),
            "golden_fixtures",
            "scene_graph_office.json",
        )
        with open(fixture_path, encoding="utf-8") as f:
            golden = json.load(f)
        assert scene_graph == golden, (
            "Scene graph does not match golden fixture. "
            "If the pipeline behavior changed intentionally, regenerate "
            "the fixture with the _gen_golden helper."
        )
