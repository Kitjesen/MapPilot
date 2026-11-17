"""Contracts for the TPT-Bench scorer and the LingTu target-person replay."""

import json

import pytest
import tools.datasets.tpt_bench as tpt_bench
from tools.datasets.person_scenarios import MarkerEncoder, scenarios, write_scenario
from tools.datasets.tpt_bench import (
    load_detections,
    oracle_verifier,
    replay_sequence,
    score_sequence,
    source_clock,
)

import decision.vision.person as person_module


def _truth(*visible: bool) -> dict[str, dict]:
    return {
        str(index + 1): {"is_exist": exists, "bbox": [10, 10, 20, 40] if exists else [0, 0, 0, 0]}
        for index, exists in enumerate(visible)
    }


def _report(box=(10, 10, 20, 40), confidence=0.9) -> dict:
    return {"target_info": [*box, confidence]}


def _absent(tracks=None) -> dict:
    return {"target_info": [0, 0, 0, 0, -1], "tracks_target_conf_bbox": tracks or []}


def test_perfect_tracking_scores_one():
    truth = _truth(True, True, False, True)
    prediction = {"1": _report(), "2": _report(), "3": _absent(), "4": _report()}

    result = score_sequence(truth, prediction)

    assert result["ao"] == pytest.approx(1.0)
    assert result["f_score"] == pytest.approx(1.0, abs=1e-6)
    assert result["amr"] == pytest.approx(1.0)
    assert result["visible_frames"] == 3


def test_reporting_an_absent_target_costs_precision_but_not_ao():
    truth = _truth(True, True, False, False)
    honest = {"1": _report(), "2": _report(), "3": _absent(), "4": _absent()}
    false_present = {"1": _report(), "2": _report(), "3": _report(), "4": _report()}

    assert score_sequence(truth, false_present)["ao"] == score_sequence(truth, honest)["ao"]
    assert score_sequence(truth, false_present)["f_score"] < score_sequence(truth, honest)["f_score"]


def test_unreported_frame_uses_the_official_track_fallback():
    truth = _truth(True, True)
    tracks = [[7, 10, 10, 20, 40, 0.4]]  # [id, x, y, w, h, confidence]

    with_track = score_sequence(truth, {"1": _report(), "2": _absent(tracks)})
    without = score_sequence(truth, {"1": _report(), "2": _absent()})

    assert with_track["ao"] == pytest.approx(1.0)
    assert without["ao"] == pytest.approx(0.5)


def test_prediction_must_cover_exactly_the_ground_truth_frames():
    with pytest.raises(ValueError, match="1 missing"):
        score_sequence(_truth(True, True), {"1": _report()})


def test_source_clock_drives_person_tracker_time_and_restores_it():
    original = person_module.time
    with source_clock() as set_time:
        set_time(12.5)
        assert person_module.time.time() == 12.5
    assert person_module.time is original


@pytest.fixture(scope="module")
def walk_away(tmp_path_factory):
    dataset = tmp_path_factory.mktemp("synthetic")
    scenario = next(item for item in scenarios() if item.name == "walk_away")
    write_scenario(scenario, dataset)
    truth = json.loads((dataset / "GTs" / "walk_away.json").read_text(encoding="utf-8"))
    return dataset, truth, load_detections(dataset / "detections" / "walk_away.jsonl")


@pytest.mark.parametrize("encoder", [None, MarkerEncoder()], ids=["motion", "appearance"])
def test_replay_tracks_an_unambiguous_target(walk_away, encoder):
    dataset, truth, detections = walk_away

    predictions, diagnostics = replay_sequence(
        dataset / "panoramic_images" / "walk_away", truth, detections, encoder=encoder,
    )

    assert diagnostics["initialized"]
    assert diagnostics["heuristic_position_objects"] == 0
    assert score_sequence(truth, predictions)["ao"] > 0.8


def test_oracle_verifier_answers_from_the_labelled_target(walk_away):
    _dataset, truth, _detections = walk_away
    key = next(key for key, value in truth.items() if value["is_exist"])
    x, y, w, h = truth[key]["bbox"]
    verify = oracle_verifier(truth)

    assert verify({"bbox": [x, y, x + w, y + h]}, key) == "MATCH"
    assert verify({"bbox": [x + 3 * w, y, x + 4 * w, y + h]}, key) == "OTHER"


def test_panoramic_position_faces_forward_at_the_image_centre():
    x, y, _ = tpt_bench.panoramic_position([950, 400, 970, 600], 1920, 960)

    assert x > 0
    assert abs(y) < 0.05 * x
