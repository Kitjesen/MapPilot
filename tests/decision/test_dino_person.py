"""Independent appearance matching must preserve language selection and target identity."""

import numpy as np

from decision.vision.person import PersonTracker


class Dino:
    def encode(self, crop):
        value = int(crop[0, 0, 0])
        return np.array([1.0, 0.0]) if value == 1 else np.array([0.0, 1.0])


class BatchDino(Dino):
    def __init__(self):
        self.single_calls = []
        self.batch_calls = []

    @staticmethod
    def _feature(crop):
        value = int(crop[0, 0, 0])
        if value == 1:
            return np.array([1.0, 0.0])
        if value == 2:
            return np.array([0.8, 0.6])
        return np.array([0.0, 1.0])

    def encode(self, crop):
        self.single_calls.append(int(crop[0, 0, 0]))
        return self._feature(crop)

    def encode_batch(self, crops):
        self.batch_calls.append([int(crop[0, 0, 0]) for crop in crops])
        return [self._feature(crop) for crop in crops]


class Clip:
    def encode_text(self, texts):
        return np.array([[0.0, 1.0]])

    def encode_image(self, crop):
        return np.array([1.0, 0.0]) if crop[0, 0, 0] == 1 else np.array([0.0, 1.0])


def person(identifier, ts=1.0, bbox=(0, 0, 16, 16), x=0.0):
    return {"id": identifier, "ts": ts, "label": "person", "position": [x, 0, 0], "bbox": list(bbox)}


def external_dino(x=1.0, y=0.0):
    feature = np.zeros(384, dtype=np.float32)
    feature[:2] = [x, y]
    return feature


def configured():
    tracker = PersonTracker()
    tracker.set_dino_encoder(Dino())
    tracker.lock_target(person("old"), np.ones((16, 16, 3), dtype=np.uint8))
    return tracker


def approve(tracker):
    pending = tracker.identity_candidate()
    assert pending is not None
    assert tracker.resolve_identity(pending["object"]["id"], pending["object"]["ts"], "MATCH")


def test_dino_cannot_replace_clip_text_selection():
    tracker = PersonTracker()
    tracker.set_clip_encoder(Clip())
    tracker.set_dino_encoder(Dino())
    crops = [np.full((16, 16, 3), value, np.uint8) for value in (1, 2)]
    assert tracker.select_by_clip("target", crops, [person("a"), person("b")]) == 1
    np.testing.assert_array_equal(tracker._person.appearance, [0, 1])
    np.testing.assert_array_equal(tracker._person.dino_feat, [0, 1])


def test_changed_id_needs_three_consistent_dino_matches():
    tracker = configured()
    identity_id = tracker.status()["identity_id"]
    frame = np.ones((16, 16, 3), dtype=np.uint8)
    assert not tracker.update([person("new", ts=2)], frame)
    assert not tracker.update([person("new", ts=3)], frame)
    assert not tracker.update([person("new", ts=4)], frame)
    assert tracker.status()["id"] == "old"
    approve(tracker)
    assert tracker.update([person("new", ts=4.03)], frame)
    assert tracker.status()["id"] == "new"
    assert tracker.status()["native_track_id"] == "new"
    assert tracker.status()["identity_id"] == identity_id
    assert tracker.status()["observation_accepted"]


def test_new_lock_creates_a_new_persistent_identity():
    tracker = configured()
    first = tracker.status()["identity_id"]

    tracker.lock_target(person("new", ts=2), np.ones((16, 16, 3), dtype=np.uint8))

    assert tracker.status()["identity_id"] != first
    assert tracker.status()["native_track_id"] == "new"


def test_lock_uses_provided_dino_feature_without_encoding_crop():
    encoder = BatchDino()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    target = person("old")
    target["dino_feature"] = external_dino(3.0, 4.0)

    tracker.lock_target(target, np.ones((16, 16, 3), dtype=np.uint8))

    assert encoder.single_calls == []
    np.testing.assert_allclose(tracker._person.dino_feat[:2], [0.6, 0.8])
    np.testing.assert_array_equal(tracker._person.dino_feat[2:], np.zeros(382))


def test_provided_candidate_features_skip_single_and_batch_encoding():
    encoder = BatchDino()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    initial = person("old")
    initial["dino_feature"] = external_dino()
    tracker.lock_target(initial, np.ones((16, 16, 3), dtype=np.uint8))
    frame = np.ones((16, 32, 3), dtype=np.uint8)

    for timestamp in (8.0, 8.03, 8.06):
        candidates = [person("returned", ts=timestamp), person("other", ts=timestamp, bbox=(16, 0, 32, 16))]
        candidates[0]["dino_feature"] = external_dino()
        candidates[1]["dino_feature"] = external_dino(0.0, 1.0)
        assert not tracker.update(candidates, frame)

    assert encoder.single_calls == []
    assert encoder.batch_calls == []
    assert tracker.identity_candidate()["object"]["id"] == "returned"


def test_provided_feature_matches_local_encoder_result_without_encoder_calls():
    class Encoder384:
        def __init__(self):
            self.calls = 0

        def encode(self, _crop):
            self.calls += 1
            return external_dino(0.6, 0.8)

    frame = np.ones((16, 16, 3), dtype=np.uint8)
    local_encoder = Encoder384()
    local = PersonTracker()
    local.set_dino_encoder(local_encoder)
    local.lock_target(person("old"), frame)
    assert local.update([person("old", ts=1.03)], frame)

    supplied_encoder = Encoder384()
    supplied = PersonTracker()
    supplied.set_dino_encoder(supplied_encoder)
    initial = person("old")
    initial["dino_feature"] = external_dino(3.0, 4.0)
    supplied.lock_target(initial, frame)
    observed = person("old", ts=1.03)
    observed["dino_feature"] = external_dino(3.0, 4.0)
    assert supplied.update([observed], frame)

    np.testing.assert_allclose(supplied._person.dino_feat, local._person.dino_feat)
    assert supplied.status()["dino_similarity"] == local.status()["dino_similarity"]
    assert local_encoder.calls == 2
    assert supplied_encoder.calls == 0


def test_provided_dino_feature_requires_exact_finite_384_vector():
    tracker = PersonTracker()
    tracker.set_dino_encoder(Dino())
    target = person("old")
    target["dino_feature"] = np.ones(383, dtype=np.float32)

    with np.testing.assert_raises_regex(ValueError, "384 values"):
        tracker.lock_target(target, np.ones((16, 16, 3), dtype=np.uint8))


def test_empty_provided_feature_uses_encoder_fallback():
    encoder = BatchDino()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    target = person("old")
    target["dino_feature"] = []

    tracker.lock_target(target, np.ones((16, 16, 3), dtype=np.uint8))

    assert encoder.single_calls == [1]


def test_wrong_person_with_same_tracker_id_cannot_poison_reference_or_position():
    tracker = configured()
    reference = tracker._person.dino_feat.copy()
    frame = np.full((16, 16, 3), 2, dtype=np.uint8)
    assert not tracker.update([person("old", ts=2, x=1)], frame)
    np.testing.assert_array_equal(tracker._person.dino_feat, reference)
    assert tracker.status()["raw_position"] == [0, 0, 0]
    assert tracker.status()["dino_event"] == "confirming"


def test_visible_current_id_is_not_stolen_by_a_higher_scoring_distractor():
    tracker = configured()

    class ViewChanged:
        calls = 0

        def encode(self, crop):
            self.calls += 1
            return np.array([0.8, 0.6]) if crop[0, 0, 0] == 2 else np.array([1.0, 0.0])

    encoder = ViewChanged()
    tracker.set_dino_encoder(encoder)
    frame = np.ones((16, 32, 3), np.uint8)
    frame[:, :16] = 2
    for timestamp in (2, 3, 4):
        assert tracker.update([
            person("distractor", ts=timestamp, bbox=(16, 0, 32, 16)),
            person("old", ts=timestamp),
        ], frame)
        assert tracker.status()["id"] == "old"
    assert encoder.calls == 3


def test_full_reacquire_scan_batches_eligible_new_candidates():
    encoder = BatchDino()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    tracker.lock_target(person("old"), np.ones((16, 16, 3), dtype=np.uint8))
    encoder.single_calls.clear()
    frame = np.ones((16, 48, 3), np.uint8)
    frame[:, 16:32] = 2
    frame[:, 32:48] = 3

    assert not tracker.update(
        [
            person("a", ts=8, bbox=(0, 0, 16, 16)),
            person("b", ts=8, bbox=(16, 0, 32, 16)),
            person("c", ts=8, bbox=(32, 0, 48, 16)),
        ],
        frame,
    )

    assert encoder.batch_calls == [[1, 2, 3]]
    assert encoder.single_calls == []


def test_batch_filters_stale_and_geometrically_unreachable_candidates_first():
    encoder = BatchDino()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    tracker.lock_target(person("old"), np.ones((16, 16, 3), dtype=np.uint8))
    encoder.single_calls.clear()
    frame = np.ones((16, 64, 3), np.uint8)
    frame[:, 16:32] = 2
    frame[:, 32:48] = 3
    frame[:, 48:64] = 4

    assert not tracker.update(
        [
            person("a", ts=8, bbox=(0, 0, 16, 16)),
            person("b", ts=8, bbox=(16, 0, 32, 16)),
            person("stale", ts=1, bbox=(32, 0, 48, 16)),
            person("far", ts=8, bbox=(48, 0, 64, 16), x=100),
        ],
        frame,
    )

    assert encoder.batch_calls == [[1, 2]]
    assert encoder.single_calls == []


def test_current_id_keeps_sequential_early_return():
    encoder = BatchDino()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    tracker.lock_target(person("old"), np.ones((16, 16, 3), dtype=np.uint8))
    encoder.single_calls.clear()
    frame = np.ones((16, 32, 3), np.uint8)
    frame[:, 16:] = 3

    assert tracker.update(
        [
            person("other", ts=1.03, bbox=(16, 0, 32, 16)),
            person("old", ts=1.03, bbox=(0, 0, 16, 16)),
        ],
        frame,
    )

    assert encoder.batch_calls == []
    assert encoder.single_calls == [1]


def test_prepared_observation_preserves_current_id_and_batch_encoding_cost():
    for current_visible in (True, False):
        encoder = BatchDino()
        tracker = PersonTracker()
        tracker.set_dino_encoder(encoder)
        tracker.lock_target(person("old"), np.ones((16, 16, 3), np.uint8))
        encoder.single_calls.clear()
        frame = np.ones((16, 48, 3), np.uint8)
        frame[:, 16:32] = 2
        frame[:, 32:] = 3
        timestamp = 1.03 if current_visible else 8.0
        objects = [
            person("old" if current_visible else "a", ts=timestamp),
            person("b", ts=timestamp, bbox=(16, 0, 32, 16)),
            person("far", ts=timestamp, bbox=(32, 0, 48, 16), x=100),
        ]

        prepared = tracker.prepare_observation(objects, frame)
        assert tracker.update(prepared, frame) is current_visible

        assert encoder.single_calls == ([1] if current_visible else [])
        assert encoder.batch_calls == ([] if current_visible else [[1, 2]])


def test_failed_prepared_feature_is_not_reencoded_during_tracking_commit():
    class Encoder(Dino):
        fail = False
        calls = 0

        def encode(self, crop):
            self.calls += 1
            if self.fail:
                raise RuntimeError("inference unavailable")
            return super().encode(crop)

    tracker = PersonTracker()
    encoder = Encoder()
    tracker.set_dino_encoder(encoder)
    frame = np.ones((16, 16, 3), np.uint8)
    tracker.lock_target(person("old"), frame)
    encoder.fail = True

    prepared = tracker.prepare_observation([person("old", ts=1.03)], frame)
    assert encoder.calls == 2
    assert not tracker.update(prepared, frame)
    assert encoder.calls == 2


def test_verified_identity_keeps_sequential_early_return():
    encoder = BatchDino()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    tracker.lock_target(person("old"), np.ones((16, 16, 3), dtype=np.uint8))
    tracker._verified_identity = {
        "object": person("verified", ts=7),
        "feature": np.array([1.0, 0.0]),
    }
    encoder.single_calls.clear()
    frame = np.ones((16, 32, 3), np.uint8)
    frame[:, 16:] = 2

    assert tracker.update(
        [
            person("verified", ts=8, bbox=(0, 0, 16, 16)),
            person("other", ts=8, bbox=(16, 0, 32, 16)),
        ],
        frame,
    )

    assert encoder.batch_calls == []
    assert encoder.single_calls == [1]


def test_batch_failure_falls_back_to_per_candidate_errors():
    class BrokenBatch(BatchDino):
        def encode_batch(self, crops):
            self.batch_calls.append([int(crop[0, 0, 0]) for crop in crops])
            raise RuntimeError("batch failed")

    encoder = BrokenBatch()
    tracker = PersonTracker()
    tracker.set_dino_encoder(encoder)
    tracker.lock_target(person("old"), np.ones((16, 16, 3), dtype=np.uint8))
    encoder.single_calls.clear()
    frame = np.ones((16, 32, 3), np.uint8)
    frame[:, 16:] = 2

    assert not tracker.update(
        [
            person("a", ts=8, bbox=(0, 0, 16, 16)),
            person("b", ts=8, bbox=(16, 0, 32, 16)),
        ],
        frame,
    )

    assert encoder.batch_calls == [[1, 2]]
    assert encoder.single_calls == [1, 2]


def test_dino_path_does_not_reencode_unused_clip_or_osnet_appearance():
    class CountingEncoder:
        def __init__(self):
            self.calls = 0

        def encode(self, _crop):
            self.calls += 1
            return np.array([1.0, 0.0])

    class CountingClip:
        def __init__(self):
            self.image_calls = 0

        def encode_image(self, _crop):
            self.image_calls += 1
            return np.array([1.0, 0.0])

    tracker = PersonTracker()
    dino = CountingEncoder()
    osnet = CountingEncoder()
    clip = CountingClip()
    tracker.set_dino_encoder(dino)
    tracker.set_osnet_encoder(osnet)
    tracker.set_clip_encoder(clip)
    frame = np.ones((16, 16, 3), np.uint8)
    tracker.lock_target(person("old"), frame)

    assert tracker.update([person("old", ts=1.03)], frame)
    assert dino.calls == 2
    assert osnet.calls == 1
    assert clip.image_calls == 1


def test_single_long_absence_candidate_does_not_run_clip_ranking():
    class CountingClip(Clip):
        def __init__(self):
            self.text_calls = 0
            self.image_calls = 0

        def encode_text(self, texts):
            self.text_calls += 1
            return super().encode_text(texts)

        def encode_image(self, crop):
            self.image_calls += 1
            return super().encode_image(crop)

    tracker = configured()
    clip = CountingClip()
    tracker.set_clip_encoder(clip)
    tracker._description = "target"
    frame = np.ones((16, 16, 3), np.uint8)

    for timestamp in (8.00, 8.03, 8.06):
        assert not tracker.update([person("returned", ts=timestamp)], frame)

    assert tracker.identity_candidate() is not None
    assert clip.text_calls == 0
    assert clip.image_calls == 0


def test_multicandidate_clip_ranking_keeps_selecting_the_text_target():
    class CountingClip(Clip):
        def __init__(self):
            self.text_calls = 0
            self.image_calls = 0

        def encode_text(self, texts):
            self.text_calls += 1
            return super().encode_text(texts)

        def encode_image(self, crop):
            self.image_calls += 1
            return super().encode_image(crop)

    tracker = configured()
    clip = CountingClip()
    tracker.set_clip_encoder(clip)
    tracker._description = "target"
    frame = np.ones((16, 32, 3), np.uint8)
    frame[:, 16:] = 2

    for timestamp in (8.00, 8.03, 8.06):
        assert not tracker.update([
            person("dino_similar_other", ts=timestamp),
            person("text_target", ts=timestamp, bbox=(16, 0, 32, 16)),
        ], frame)

    assert tracker.identity_candidate()["object"]["id"] == "text_target"
    assert clip.text_calls == 3
    assert clip.image_calls == 6


def test_ambiguous_people_do_not_change_identity():
    tracker = configured()
    assert not tracker.update([person("a", ts=2), person("b", ts=2)], np.ones((16, 16, 3), np.uint8))
    assert tracker.status()["dino_event"] == "ambiguous"
    assert tracker.status()["id"] == "old"


def test_dino_does_not_override_map_frame_motion_gate():
    tracker = configured()
    assert not tracker.update([person("new", ts=2, x=20)], np.ones((16, 16, 3), np.uint8))
    assert tracker.status()["id"] == "old"


def test_dino_failure_does_not_fall_back_to_an_unverified_new_identity():
    tracker = configured()

    class Broken:
        def encode(self, crop):
            raise RuntimeError("model error")

    tracker.set_dino_encoder(Broken())
    assert not tracker.update([person("new", ts=2)], np.ones((16, 16, 3), np.uint8))
    assert tracker.status()["dino_event"] == "encode_failed"
    assert tracker.status()["id"] == "old"


def test_return_after_long_absence_keeps_reference_and_reseeds_position():
    tracker = configured()
    reference = tracker._person.dino_reference.copy()
    frame = np.ones((16, 16, 3), np.uint8)
    assert tracker.has_identity_reference
    assert not tracker.update([], frame)
    assert tracker.status()["dino_event"] == "target_absent"
    assert not tracker.status()["observation_accepted"]
    assert tracker.status()["dino_similarity"] is None
    for timestamp in (11.0, 11.03):
        assert not tracker.update([person("returned", ts=timestamp, x=5)], frame)
    assert not tracker.update([person("returned", ts=11.06, x=5)], frame)
    approve(tracker)
    assert tracker.update([person("returned", ts=11.09, x=5)], frame)
    assert tracker.status()["id"] == "returned"
    assert tracker.status()["position"] == [5.0, 0.0, 0.0]
    assert tracker.status()["velocity"] == [0.0, 0.0]
    assert tracker.status()["observation_accepted"]
    np.testing.assert_array_equal(tracker._person.dino_reference, reference)
    tracker.reset()
    assert not tracker.has_identity_reference


def test_context_recovery_reseeds_position_and_keeps_identity_and_source_time():
    tracker = configured()
    reference = tracker._person.dino_reference.copy()
    identity_id = tracker.status()["identity_id"]
    frame = np.ones((16, 16, 3), np.uint8)
    assert tracker.reset_observation_context()
    assert tracker.get_person_position() is None
    assert tracker.get_follow_waypoint([8.0, 0.0, 0.0]) is None

    for timestamp in (0.1, 0.13, 0.16):
        assert not tracker.update([person("returned", ts=timestamp, x=10.0)], frame)
    approve(tracker)
    assert tracker.update([person("returned", ts=0.19, x=10.0)], frame)

    assert tracker.get_person_position() == [10.0, 0.0, 0.0]
    assert tracker.status()["velocity"] == [0.0, 0.0]
    assert tracker.status()["identity_id"] == identity_id
    np.testing.assert_array_equal(tracker._person.dino_reference, reference)

    assert tracker.update([person("returned", ts=0.29, x=10.1)], frame)
    np.testing.assert_allclose(tracker.get_person_position(), [10.04, 0.0, 0.0])
    np.testing.assert_allclose(tracker.status()["velocity"], [0.3, 0.0])
    assert not tracker.update([person("returned", ts=0.29, x=10.2)], frame)
    np.testing.assert_allclose(tracker.status()["velocity"], [0.3, 0.0])
    assert not tracker.update([person("returned", ts=0.3, x=11.0)], frame)
    assert tracker.status()["observation_reason"] == "position_rejected"
    np.testing.assert_allclose(tracker.get_person_position(), [10.04, 0.0, 0.0])


def test_context_recovery_waypoint_stays_between_robot_and_confirmed_target():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    assert tracker.reset_observation_context()

    for timestamp in (2.0, 2.03, 2.06):
        assert not tracker.update([person("returned", ts=timestamp, x=10.0)], frame)
    approve(tracker)
    assert tracker.update([person("returned", ts=2.09, x=10.0)], frame)

    assert tracker.get_follow_waypoint([8.0, 0.0, 0.0]) == {"x": 8.5, "y": 0.0, "z": 0.0}


def test_reused_same_id_after_absence_also_needs_confirmation():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    assert not tracker.update([person("old", ts=8)], frame)
    assert not tracker.update([person("old", ts=8.03)], frame)
    assert not tracker.update([person("old", ts=8.06)], frame)
    approve(tracker)
    assert tracker.update([person("old", ts=8.09)], frame)


def test_duplicate_frames_do_not_complete_reacquisition():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    for _ in range(5):
        assert not tracker.update([person("returned", ts=8)], frame)
    assert not tracker.update([person("returned", ts=8.03)], frame)
    assert not tracker.update([person("returned", ts=8.06)], frame)
    approve(tracker)
    assert tracker.update([person("returned", ts=8.09)], frame)


def test_return_confirmation_requires_continuous_positions():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    assert not tracker.update([person("returned", ts=11, x=5)], frame)
    assert not tracker.update([person("returned", ts=11.03, x=25)], frame)
    assert not tracker.update([person("returned", ts=11.06, x=25)], frame)
    assert tracker.status()["id"] == "old"


def test_original_reference_blocks_gradual_appearance_drift():
    tracker = configured()

    class Gradual:
        def encode(self, crop):
            angle = float(crop[0, 0, 0]) / 100
            return np.array([np.cos(angle), np.sin(angle)])

    tracker.set_dino_encoder(Gradual())
    for index, angle in enumerate(np.linspace(0.0, 1.4, 60), start=1):
        frame = np.full((16, 16, 3), round(angle * 100), np.uint8)
        tracker.update([person("old", ts=1 + index / 30)], frame)
    assert not tracker.status()["observation_accepted"]
    assert tracker.status()["dino_event"] in {"low_similarity", "confirming", "awaiting_identity_verification"}
    np.testing.assert_array_equal(tracker._person.dino_reference, [1, 0])
    assert tracker._person.observation_ts < 3


def test_depth_outlier_cannot_poison_velocity_position_or_identity():
    tracker = configured()
    identity_id = tracker.status()["identity_id"]
    frame = np.ones((16, 16, 3), np.uint8)
    assert not tracker.update([person("old", ts=1.03, x=1.8)], frame)
    assert tracker.status()["dino_event"] == "position_rejected"
    assert tracker.status()["identity_id"] == identity_id
    assert tracker.status()["native_track_id"] == "old"
    assert tracker.status()["raw_position"] == [0, 0, 0]
    assert tracker.status()["velocity"] == [0, 0]
    assert tracker.update([person("old", ts=1.06, x=0.05)], frame)
    assert tracker.status()["raw_position"] == [0.05, 0, 0]


def test_other_verdict_cannot_change_identity_position_or_reference():
    tracker = configured()
    identity_id = tracker.status()["identity_id"]
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8, 8.03, 8.06):
        assert not tracker.update([person("other", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("other", pending["object"]["ts"], "OTHER")
    assert not tracker.update([person("other", ts=8.09)], frame)
    assert tracker.identity_candidate() is None
    assert tracker.status()["id"] == "old"
    assert tracker.status()["identity_id"] == identity_id
    assert tracker.status()["native_track_id"] == "old"
    assert tracker.status()["raw_position"] == [0, 0, 0]
    np.testing.assert_array_equal(tracker._person.dino_reference, [1, 0])


def test_verified_view_recovers_new_pose_without_replacing_original():
    tracker = configured()
    frame = np.full((16,16,3),2,np.uint8)
    for timestamp in (8,8.03,8.06):
        assert not tracker.update([person("returned",ts=timestamp)],frame)
    approve(tracker)
    assert tracker.update([person("returned",ts=8.09)],frame)
    assert tracker.update([person("returned",ts=8.12)],frame)
    np.testing.assert_array_equal(tracker._person.dino_reference,[1,0])
    np.testing.assert_array_equal(tracker._person.dino_verified_views[0],[0,1])


def test_late_match_after_stop_and_teleport_after_verification_are_rejected():
    tracker=configured()
    frame=np.ones((16,16,3),np.uint8)
    for timestamp in (8,8.03,8.06):
        tracker.update([person("returned",ts=timestamp)],frame)
    approve(tracker)
    assert not tracker.update([person("returned",ts=8.09,x=3)],frame)
    assert tracker.status()["id"] == "old"
    tracker.reset()
    assert not tracker.resolve_identity("returned",8.06,"MATCH")


def test_verification_does_not_survive_an_intervening_absence():
    tracker=configured()
    frame=np.ones((16,16,3),np.uint8)
    for timestamp in (8,8.03,8.06):
        tracker.update([person("returned",ts=timestamp)],frame)
    approve(tracker)
    assert not tracker.update([],frame)
    assert not tracker.update([person("returned",ts=20)],frame)
    assert tracker.status()["id"] == "old"
    assert tracker._person.dino_verified_views == []


def test_clip_text_ranks_vlm_proposals_without_authorizing_motion():
    tracker = configured()
    tracker.set_clip_encoder(Clip())
    tracker._description = "target"
    frame=np.ones((16,32,3),np.uint8)
    frame[:,16:]=2
    for timestamp in (8,8.03,8.06):
        assert not tracker.update([
            person("dino_similar_other",ts=timestamp),
            person("text_target",ts=timestamp,bbox=(16,0,32,16)),
        ],frame)
    assert tracker.identity_candidate()["object"]["id"] == "text_target"
    assert tracker.status()["id"] == "old"
    assert tracker.status()["raw_position"] == [0,0,0]


def test_same_id_view_change_is_verified_without_waiting_for_a_long_absence():
    tracker = configured()
    changed_view = np.full((16, 16, 3), 2, np.uint8)
    for timestamp in (1.03, 1.06, 1.09):
        assert not tracker.update([person("old", ts=timestamp)], changed_view)
    pending = tracker.identity_candidate()
    assert pending is not None
    assert tracker.status()["raw_position"] == [0, 0, 0]
    np.testing.assert_array_equal(tracker._person.dino_reference, [1, 0])
    approve(tracker)
    assert tracker.update([person("old", ts=1.12)], changed_view)
    np.testing.assert_array_equal(tracker._person.dino_reference, [1, 0])


def test_visible_candidate_can_retry_a_false_other_after_the_cooldown():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "OTHER")
    assert not tracker.update([person("returned", ts=8.3)], frame)
    assert tracker.identity_candidate() is None
    for timestamp in (8.37, 8.40, 8.43):
        assert not tracker.update([person("returned", ts=timestamp)], frame)
    assert tracker.identity_candidate() is not None
    assert tracker.status()["id"] == "old"
    approve(tracker)
    assert tracker.update([person("returned", ts=8.46)], frame)
    assert tracker.status()["id"] == "returned"


def test_unknown_candidate_does_not_delay_verifying_another_person():
    tracker = configured()
    identity_id = tracker.status()["identity_id"]
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8, 8.03, 8.06):
        tracker.update([person("uncertain", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("uncertain", pending["object"]["ts"], "UNKNOWN")
    for timestamp in (8.09, 8.12, 8.15):
        assert not tracker.update([
            person("uncertain", ts=timestamp), person("returned", ts=timestamp),
        ], frame)
    assert tracker.identity_candidate()["object"]["id"] == "returned"
    assert tracker.status()["id"] == "old"
    assert tracker.status()["identity_id"] == identity_id
    assert tracker.status()["native_track_id"] == "old"


def test_rejected_current_id_still_needs_match_when_dino_recovers():
    tracker = configured()
    changed_view = np.full((16, 16, 3), 2, np.uint8)
    for timestamp in (1.03, 1.06, 1.09):
        tracker.update([person("old", ts=timestamp)], changed_view)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("old", pending["object"]["ts"], "OTHER")
    old_appearance = np.ones((16, 16, 3), np.uint8)
    for timestamp in (1.4, 1.43, 1.46):
        assert not tracker.update([person("old", ts=timestamp)], old_appearance)
    assert tracker.identity_candidate() is not None
    assert tracker._person.observation_ts == 1.0
    approve(tracker)
    assert tracker.update([person("old", ts=1.49)], old_appearance)


def test_a_detection_dropout_does_not_clear_current_identity_rejection():
    for dropout in ([], [person("bystander", ts=1.12)]):
        tracker = configured()
        changed_view = np.full((16, 16, 3), 2, np.uint8)
        for timestamp in (1.03, 1.06, 1.09):
            tracker.update([person("old", ts=timestamp)], changed_view)
        pending = tracker.identity_candidate()
        assert not tracker.resolve_identity("old", pending["object"]["ts"], "OTHER")
        frame = np.ones((16, 16, 3), np.uint8)
        assert not tracker.update(dropout, frame)
        for timestamp in (1.4, 1.43, 1.46):
            assert not tracker.update([person("old", ts=timestamp)], frame)
        assert tracker.identity_candidate()["object"]["id"] == "old"
        assert tracker._person.observation_ts == 1.0
        approve(tracker)
        assert tracker.update([person("old", ts=1.49)], frame)


def test_identity_candidate_contains_only_consecutive_same_id_crops():
    tracker = configured()
    changed = np.full((16, 16, 3), 2, np.uint8)
    assert not tracker.update([person("first", ts=8.00)], changed)
    assert not tracker.update([person("other", ts=8.03)], changed)
    assert not tracker.update([person("first", ts=8.06)], changed)
    assert not tracker.update([person("first", ts=8.09)], changed)
    assert not tracker.update([person("first", ts=8.12)], changed)
    pending = tracker.identity_candidate()
    assert pending is not None
    assert pending["object"]["id"] == "first"
    assert pending["crop_timestamps"] == [8.06, 8.09, 8.12]
    assert len(pending["crops"]) == 3
    np.testing.assert_array_equal(pending["crop"], pending["crops"][-1])
    pending["crops"][0][:] = 0
    assert tracker.identity_candidate()["crops"][0][0, 0, 0] == 2


def test_identity_evidence_is_cleared_by_disappearance():
    tracker = configured()
    changed = np.full((16, 16, 3), 2, np.uint8)
    assert not tracker.update([person("returned", ts=8.00)], changed)
    assert not tracker.update([person("returned", ts=8.03)], changed)
    assert not tracker.update([], changed)
    assert not tracker.update([person("returned", ts=8.06)], changed)
    assert not tracker.update([person("returned", ts=8.09)], changed)
    assert tracker.identity_candidate() is None
    assert not tracker.update([person("returned", ts=8.12)], changed)
    assert tracker.identity_candidate()["crop_timestamps"] == [8.06, 8.09, 8.12]


def test_trusted_rgb_reference_is_added_only_after_fresh_verified_acceptance():
    tracker = configured()
    changed = np.full((16, 16, 3), 2, np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        assert not tracker.update([person("returned", ts=timestamp)], changed)
    pending = tracker.identity_candidate()
    assert pending["reference_crops"] == []
    approve(tracker)
    assert tracker._verified_reference_crops == []
    assert tracker.update([person("returned", ts=8.09)], changed)
    assert len(tracker._verified_reference_crops) == 1
    np.testing.assert_array_equal(tracker._reference_crop, np.ones((16, 16, 3), np.uint8))

    third_view = np.full((16, 16, 3), 3, np.uint8)
    for timestamp in (10.0, 10.03, 10.06):
        assert not tracker.update([person("again", ts=timestamp)], third_view)
    next_pending = tracker.identity_candidate()
    assert len(next_pending["reference_crops"]) == 1
    np.testing.assert_array_equal(next_pending["reference_crops"][0], changed)


def test_other_uses_bounded_progressive_retry_for_unchanged_evidence():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "OTHER")
    assert np.isclose(tracker.status()["identity_retry_until"]["returned"], 8.36)

    for timestamp in (8.37, 8.40, 8.43):
        assert not tracker.update([person("returned", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "OTHER")
    assert np.isclose(tracker.status()["identity_retry_until"]["returned"], 9.03)
    for timestamp in (8.60, 8.70, 8.90):
        assert not tracker.update([person("returned", ts=timestamp)], frame)
    assert tracker.identity_candidate() is None


def test_unknown_can_retry_early_when_target_evidence_improves():
    tracker = configured()
    frame = np.full((16, 16, 3), 2, np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "UNKNOWN")

    changed = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8.09, 8.12, 8.15):
        assert not tracker.update([person("returned", ts=timestamp)], changed)
    pending = tracker.identity_candidate()
    assert pending is not None
    assert pending["crop_timestamps"] == [8.09, 8.12, 8.15]


def test_unknown_uses_bounded_progressive_retry_for_unchanged_evidence():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "UNKNOWN")
    assert np.isclose(tracker.status()["identity_retry_until"]["returned"], 8.36)

    for timestamp in (8.37, 8.40, 8.43):
        assert not tracker.update([person("returned", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "UNKNOWN")
    assert np.isclose(tracker.status()["identity_retry_until"]["returned"], 9.03)
    for timestamp in (8.60, 8.70, 8.90):
        assert not tracker.update([person("returned", ts=timestamp)], frame)
    assert tracker.identity_candidate() is None


def test_other_can_retry_improved_evidence_without_authorizing_it():
    tracker = configured()
    poor = np.full((16, 16, 3), 2, np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], poor)
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "OTHER")

    improved = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8.09, 8.12, 8.15):
        assert not tracker.update([person("returned", ts=timestamp)], improved)
    assert tracker.identity_candidate() is not None
    assert tracker.status()["id"] == "old"
    assert tracker._person.observation_ts == 1.0


def test_other_retry_level_is_bounded_before_interval_calculation():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], frame)
    tracker._identity_retry_state["returned"] = {
        "attempts": 1024,
        "score": 1.0,
        "verdict": "OTHER",
    }
    pending = tracker.identity_candidate()
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "OTHER")
    assert tracker._identity_retry_state["returned"]["attempts"] == 3
    assert np.isclose(tracker.status()["identity_retry_until"]["returned"], 9.26)


def test_missing_rgb_discards_partial_candidate_evidence():
    tracker = configured()
    changed = np.full((16, 16, 3), 2, np.uint8)
    assert not tracker.update([person("returned", ts=8.00)], changed)
    assert not tracker.update([person("returned", ts=8.03)], changed)
    assert not tracker.update([person("returned", ts=8.06)], None)
    assert tracker.status()["dino_event"] == "missing_image"
    assert not tracker.update([person("returned", ts=8.09)], changed)
    assert not tracker.update([person("returned", ts=8.12)], changed)
    assert tracker.identity_candidate() is None
    assert not tracker.update([person("returned", ts=8.15)], changed)
    assert tracker.identity_candidate()["crop_timestamps"] == [8.09, 8.12, 8.15]


def test_missing_rgb_rejects_an_inflight_identity_result():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], frame)
    pending = tracker.identity_candidate()
    assert not tracker.update([person("returned", ts=8.09)], None)
    assert not tracker.resolve_identity("returned", pending["object"]["ts"], "MATCH")
    assert tracker.status()["id"] == "old"


def test_missing_rgb_discards_a_queued_match_before_commit():
    tracker = configured()
    frame = np.ones((16, 16, 3), np.uint8)
    for timestamp in (8.00, 8.03, 8.06):
        tracker.update([person("returned", ts=timestamp)], frame)
    approve(tracker)
    assert not tracker.update([person("returned", ts=8.09)], None)
    for timestamp in (8.12, 8.15):
        assert not tracker.update([person("returned", ts=timestamp)], frame)
    assert tracker.identity_candidate() is None
    assert tracker.status()["id"] == "old"
    assert not tracker.update([person("returned", ts=8.18)], frame)
    assert tracker.identity_candidate() is not None


def test_duplicate_verified_view_does_not_evict_distinct_reference_crops():
    class Views:
        def encode(self, crop):
            value = int(crop[0, 0, 0])
            return {
                10: np.array([1.0, 0.0, 0.0]),
                20: np.array([0.8, 0.6, 0.0]),
                30: np.array([0.0, 0.8, 0.6]),
                40: np.array([0.6, 0.0, 0.8]),
            }[value]

    tracker = PersonTracker()
    tracker.set_dino_encoder(Views())
    tracker.lock_target(person("old"), np.full((16, 16, 3), 10, np.uint8))

    def verify_view(identifier, start, value):
        frame = np.full((16, 16, 3), value, np.uint8)
        for timestamp in (start, start + 0.03, start + 0.06):
            assert not tracker.update([person(identifier, ts=timestamp)], frame)
        approve(tracker)
        assert tracker.update([person(identifier, ts=start + 0.09)], frame)

    verify_view("side", 8.0, 20)
    verify_view("front", 10.0, 30)
    verify_view("back", 12.0, 40)
    verify_view("side-again", 14.0, 20)

    assert [int(crop[0, 0, 0]) for crop in tracker._verified_reference_crops] == [30, 40]
    assert tracker._verified_reference_timestamps == [10.09, 12.09]
