"""Separate identity changes, uncertain boxes, and missing video targets."""

import numpy as np
from tools.datasets.person_eval import evaluate


def _record(frame, track_id, box):
    return {"frame": frame, "selected_track_id": track_id,
            "target": {"track_id": track_id, "bbox_xyxy": box} if box is not None else None}


def test_new_track_id_can_still_correspond_to_the_same_real_person():
    records = [_record(0, 6, [10, 10, 30, 60]), _record(1, 12, [12, 10, 32, 60])]
    annotations = np.array([[1, 22, 10, 10, 20, 50, 1, 1, 1], [2, 22, 12, 10, 20, 50, 1, 1, 1]])
    result = evaluate(records, annotations)
    assert result["correct_reference_frames"] == 2
    assert result["selected_track_id_changes"] == 1
    assert result["wrong_identity_frames"] == 0


def test_same_track_id_can_switch_to_a_different_real_person():
    records = [_record(0, 6, [10, 10, 30, 60]), _record(1, 6, [100, 10, 120, 60])]
    annotations = np.array([[1, 22, 10, 10, 20, 50, 1, 1, 1], [2, 19, 100, 10, 20, 50, 1, 1, 1]])
    result = evaluate(records, annotations)
    assert result["correct_reference_frames"] == 1
    assert result["wrong_identity_frames"] == 1
    assert result["selected_track_id_changes"] == 0


def test_uncertain_and_missing_targets_do_not_count_as_correct():
    records = [_record(0, 6, [10, 10, 30, 60]), _record(1, 6, [200, 200, 220, 250]), _record(2, 6, None)]
    annotations = np.array([[1, 22, 10, 10, 20, 50, 1, 1, 1], [2, 22, 12, 10, 20, 50, 1, 1, 1]])
    result = evaluate(records, annotations)
    assert result["correct_reference_frames"] == 1
    assert result["unattributed_target_frames"] == 1
    assert result["missing_target_frames"] == 1
