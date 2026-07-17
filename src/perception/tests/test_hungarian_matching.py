# ruff: noqa: S101  (assert statements are standard in pytest test files)
"""
test_hungarian_matching.py

Stage 1a tracking-improvement tests for InstanceTracker:

  1. Hungarian global-optimal matching (_match_detections_hungarian)
  2. Post-matching duplicate merge (_dedup_merge)

Both features are gated behind config flags that default OFF, so the golden
characterization snapshot and the existing greedy tests are unaffected.  These
tests explicitly enable the new paths and prove their correctness.

Determinism:
  - one-hot feature vectors -> cosine similarity is exactly 0.0 or 1.0 for
    orthogonal / identical labels
  - time.time() frozen where object timestamps would otherwise leak in
"""

import itertools
from unittest.mock import patch

import numpy as np

from perception.tracking.instance_tracker import InstanceTracker
from perception.tracking.projection import Detection3D
from perception.tracking.tracked_objects import TrackedObject

FIXED_TIME = 1_000_000.0

_LABEL_FIDX = {
    "chair": 0,
    "desk": 1,
    "monitor": 2,
    "keyboard": 3,
    "door": 4,
    "sofa": 5,
    "table": 6,
}


def _onehot(idx: int, dim: int = 512) -> np.ndarray:
    v = np.zeros(dim, dtype=np.float32)
    v[idx] = 1.0
    return v


def _blend(idx_a: int, idx_b: int, w: float, dim: int = 512) -> np.ndarray:
    """Unit-norm blend of two one-hot axes; lets us dial cosine similarity."""
    v = np.zeros(dim, dtype=np.float32)
    v[idx_a] = 1.0 - w
    v[idx_b] = w
    n = np.linalg.norm(v)
    return (v / n).astype(np.float32)


def _make_det(label, pos, score=0.9, features=None):
    fidx = _LABEL_FIDX.get(label, 0)
    feat = _onehot(fidx) if features is None else features
    return Detection3D(
        position=np.array(pos, dtype=np.float32),
        label=label,
        score=score,
        bbox_2d=np.array([10, 10, 100, 100], dtype=np.float32),
        depth=float(np.linalg.norm(np.array(pos[:2]))),
        features=feat,
    )


def _make_obj(oid, label, pos, features=None, credibility=0.5, last_seen=FIXED_TIME):
    fidx = _LABEL_FIDX.get(label, 0)
    feat = _onehot(fidx) if features is None else features
    return TrackedObject(
        object_id=oid,
        label=label,
        position=np.array(pos, dtype=np.float64),
        best_score=0.9,
        last_seen=last_seen,
        features=feat,
        credibility=credibility,
    )


def _frozen_time():
    return patch("time.time", return_value=FIXED_TIME)


def _seed(tracker, objs):
    """Insert pre-built TrackedObjects into a tracker's internal table."""
    for o in objs:
        tracker._objects[o.object_id] = o
    tracker._next_id = max((o.object_id for o in objs), default=-1) + 1


def _total_cost(cost, mapping, obj_index):
    """Sum of cost matrix cells for a {det_idx: TrackedObject} mapping."""
    total = 0.0
    for det_idx, obj in mapping.items():
        total += cost[det_idx, obj_index[obj.object_id]]
    return total


# ─────────────────────────────────────────────────────────────────────────────
#  Hungarian matching
# ─────────────────────────────────────────────────────────────────────────────


class TestHungarianMatching:
    def test_no_candidates_returns_empty(self):
        tracker = InstanceTracker(use_hungarian_matching=True)
        dets = [_make_det("chair", (0.0, 0.0, 0.0))]
        assert tracker._match_detections_hungarian(dets, []) == {}

    def test_no_detections_returns_empty(self):
        tracker = InstanceTracker(use_hungarian_matching=True)
        objs = [_make_obj(0, "chair", (0.0, 0.0, 0.0))]
        assert tracker._match_detections_hungarian([], objs) == {}

    def test_basic_one_to_one(self):
        """Two detections, two matching objects -> distinct 1:1 assignment."""
        tracker = InstanceTracker(use_hungarian_matching=True, merge_distance=0.5)
        objs = [
            _make_obj(0, "chair", (0.0, 0.0, 0.0)),
            _make_obj(1, "desk", (5.0, 0.0, 0.0)),
        ]
        dets = [
            _make_det("chair", (0.05, 0.0, 0.0)),
            _make_det("desk", (5.05, 0.0, 0.0)),
        ]
        m = tracker._match_detections_hungarian(dets, objs)
        assert m[0].object_id == 0
        assert m[1].object_id == 1
        assert len({o.object_id for o in m.values()}) == len(m)

    def test_global_optimality_beats_greedy(self):
        """A crafted case where per-detection greedy is sub-optimal.

        Object O0 sits between the two detections; a greedy pass that handles
        D0 first would grab O0 (its nearest), forcing D1 onto its worse option.
        The Hungarian solver minimises the *total* cost instead.
        """
        merge = 1.0
        tracker = InstanceTracker(use_hungarian_matching=True, merge_distance=merge)
        # Same label so omega_sem == 1.0 for every pair; geometry decides.
        objs = [
            _make_obj(0, "chair", (0.40, 0.0, 0.0)),  # near both dets
            _make_obj(1, "chair", (0.90, 0.0, 0.0)),  # only near D1
        ]
        dets = [
            _make_det("chair", (0.00, 0.0, 0.0)),  # D0 closest to O0
            _make_det("chair", (0.85, 0.0, 0.0)),  # D1 closest to O1
        ]
        m = tracker._match_detections_hungarian(dets, objs)

        # Optimal assignment: D0->O0, D1->O1 (distinct, minimal total distance).
        assert m[0].object_id == 0
        assert m[1].object_id == 1

        # Prove minimality against brute force over all injective mappings.
        n_det, n_obj = len(dets), len(objs)
        obj_index = {o.object_id: j for j, o in enumerate(objs)}
        INVALID = 999.0
        cost = np.full((n_det, n_obj), INVALID)
        for i, det in enumerate(dets):
            for j, obj in enumerate(objs):
                dist = float(np.linalg.norm(obj.position - det.position))
                omega_sem = tracker._cosine_similarity(det.features, obj.features)
                omega_geo = max(0.0, 1.0 - dist / merge) if dist < merge else 0.0
                combined = 0.6 * omega_sem + 0.4 * omega_geo
                if combined > 0.3:
                    cost[i, j] = -combined
        best = min(sum(cost[i, perm[i]] for i in range(n_det)) for perm in itertools.permutations(range(n_obj), n_det))
        got = _total_cost(cost, m, obj_index)
        assert got == best

    def test_rectangular_5_dets_3_objs_no_duplicates(self):
        """5 detections vs 3 candidates: <=3 matches, all distinct objects."""
        merge = 0.5
        tracker = InstanceTracker(use_hungarian_matching=True, merge_distance=merge)
        objs = [
            _make_obj(0, "chair", (0.0, 0.0, 0.0)),
            _make_obj(1, "desk", (2.0, 0.0, 0.0)),
            _make_obj(2, "monitor", (4.0, 0.0, 0.0)),
        ]
        dets = [
            _make_det("chair", (0.02, 0.0, 0.0)),
            _make_det("desk", (2.02, 0.0, 0.0)),
            _make_det("monitor", (4.02, 0.0, 0.0)),
            _make_det("sofa", (10.0, 0.0, 0.0)),  # no candidate nearby
            _make_det("table", (20.0, 0.0, 0.0)),  # no candidate nearby
        ]
        m = tracker._match_detections_hungarian(dets, objs)

        assert len(m) == 3
        assigned_ids = [o.object_id for o in m.values()]
        assert sorted(assigned_ids) == [0, 1, 2]  # each object used once
        assert len(assigned_ids) == len(set(assigned_ids))
        assert m[0].object_id == 0
        assert m[1].object_id == 1
        assert m[2].object_id == 2
        # The two far detections stay unmatched (-> would create new objects).
        assert 3 not in m
        assert 4 not in m

    def test_low_combined_score_not_matched(self):
        """A candidate below the 0.3 combined-score gate is never assigned."""
        merge = 0.5
        tracker = InstanceTracker(use_hungarian_matching=True, merge_distance=merge)
        objs = [_make_obj(0, "chair", (0.0, 0.0, 0.0))]
        # Different label (omega_sem=0) AND out of merge_distance (omega_geo=0).
        dets = [_make_det("desk", (5.0, 0.0, 0.0))]
        m = tracker._match_detections_hungarian(dets, objs)
        assert m == {}

    def test_update_uses_hungarian_when_enabled(self):
        """End-to-end: enabling the gate routes update() through Hungarian."""
        with _frozen_time():
            tracker = InstanceTracker(use_hungarian_matching=True, merge_distance=0.5)
            tracker.update([_make_det("chair", (1.0, 0.0, 0.5))])
            tracker.update([_make_det("chair", (1.1, 0.0, 0.5))])
        assert len(tracker.objects) == 1


# ─────────────────────────────────────────────────────────────────────────────
#  Dedup merge
# ─────────────────────────────────────────────────────────────────────────────


class TestDedupMerge:
    def _tracker(self):
        return InstanceTracker(
            enable_dedup_merge=True,
            dedup_distance=0.3,
            dedup_clip_threshold=0.85,
            dedup_time_window=5.0,
        )

    def test_merges_true_duplicates(self):
        """Same label, close, high sim, recent -> merged into one."""
        tracker = self._tracker()
        _seed(
            tracker,
            [
                _make_obj(0, "chair", (1.00, 0.0, 0.0), credibility=0.8, last_seen=FIXED_TIME),
                _make_obj(1, "chair", (1.10, 0.0, 0.0), credibility=0.4, last_seen=FIXED_TIME + 1.0),
            ],
        )
        remap = tracker._dedup_merge()

        assert len(tracker._objects) == 1
        assert 0 in tracker._objects  # higher credibility survives
        assert 1 not in tracker._objects
        assert remap == {1: tracker._objects[0]}
        # EMA position between the two originals.
        np.testing.assert_allclose(tracker._objects[0].position, [1.05, 0.0, 0.0], atol=1e-6)

    def test_keeps_higher_credibility_object(self):
        """Survivor is always the higher-credibility object, regardless order."""
        tracker = self._tracker()
        _seed(
            tracker,
            [
                _make_obj(7, "desk", (2.00, 0.0, 0.0), credibility=0.2),
                _make_obj(9, "desk", (2.05, 0.0, 0.0), credibility=0.9),
            ],
        )
        tracker._dedup_merge()
        assert list(tracker._objects.keys()) == [9]

    def test_no_merge_different_label(self):
        tracker = self._tracker()
        _seed(
            tracker,
            [
                _make_obj(0, "chair", (1.00, 0.0, 0.0)),
                _make_obj(1, "desk", (1.05, 0.0, 0.0)),
            ],
        )
        assert tracker._dedup_merge() == {}
        assert len(tracker._objects) == 2

    def test_no_merge_too_far(self):
        tracker = self._tracker()
        _seed(
            tracker,
            [
                _make_obj(0, "chair", (1.00, 0.0, 0.0)),
                _make_obj(1, "chair", (1.50, 0.0, 0.0)),  # 0.5 m > 0.3 m
            ],
        )
        assert tracker._dedup_merge() == {}
        assert len(tracker._objects) == 2

    def test_no_merge_low_clip_similarity(self):
        tracker = self._tracker()
        # sim ~0.5 (blend at w=0.5 gives cosine 0.707 vs pure axis) -> keep < 0.85
        low_sim_feat = _blend(0, 6, 0.5)  # chair-axis blended with table-axis
        _seed(
            tracker,
            [
                _make_obj(0, "chair", (1.00, 0.0, 0.0)),  # pure chair one-hot
                _make_obj(1, "chair", (1.05, 0.0, 0.0), features=low_sim_feat),
            ],
        )
        # sanity: similarity is genuinely below threshold
        sim = tracker._cosine_similarity(tracker._objects[0].features, tracker._objects[1].features)
        assert sim < 0.85
        assert tracker._dedup_merge() == {}
        assert len(tracker._objects) == 2

    def test_no_merge_outside_time_window(self):
        tracker = self._tracker()
        _seed(
            tracker,
            [
                _make_obj(0, "chair", (1.00, 0.0, 0.0), last_seen=FIXED_TIME),
                _make_obj(1, "chair", (1.05, 0.0, 0.0), last_seen=FIXED_TIME + 10.0),  # 10 s > 5 s window
            ],
        )
        assert tracker._dedup_merge() == {}
        assert len(tracker._objects) == 2

    def test_update_runs_dedup_when_enabled(self):
        """update() drops merged ids from its return value (API contract)."""
        with _frozen_time():
            tracker = self._tracker()
            _seed(
                tracker,
                [
                    _make_obj(0, "chair", (1.00, 0.0, 0.0), credibility=0.8),
                    _make_obj(1, "chair", (1.08, 0.0, 0.0), credibility=0.4),
                ],
            )
            # Empty detection batch: matching is a no-op, dedup still runs.
            result = tracker.update([])
        assert len(tracker.objects) == 1
        returned_ids = {o.object_id for o in result}
        assert 1 not in returned_ids


# ─────────────────────────────────────────────────────────────────────────────
#  Gates default OFF
# ─────────────────────────────────────────────────────────────────────────────


class TestGatesDefaultOff:
    def test_defaults_disable_new_paths(self):
        tracker = InstanceTracker()
        assert tracker.use_hungarian_matching is False
        assert tracker.enable_dedup_merge is False
        assert tracker.dedup_distance == 0.3
        assert tracker.dedup_clip_threshold == 0.85
        assert tracker.dedup_time_window == 5.0

    def test_default_tracker_uses_greedy_no_dedup(self):
        """With defaults, two near-duplicate seeded objects are NOT merged."""
        with _frozen_time():
            tracker = InstanceTracker()
            _seed(
                tracker,
                [
                    _make_obj(0, "chair", (1.00, 0.0, 0.0), credibility=0.8),
                    _make_obj(1, "chair", (1.05, 0.0, 0.0), credibility=0.4),
                ],
            )
            tracker.update([])
        assert len(tracker.objects) == 2
