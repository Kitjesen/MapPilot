"""Pure measurement decisions do not assert task success or field qualification."""

import json
from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from perception.inspection.measurement_checks import AcousticRule, ThermalRule, evaluate_acoustic, evaluate_thermal


def thermal_rule(**changes):
    return ThermalRule(**(dict(
        max_kelvin=320.0, roi=(1, 0, 3, 2), min_valid_coverage=1.0,
        temperature_kind="apparent", calibration_id="cal-test",
        model_id="greybody-test", model_version="v1",
    ) | changes))


def metadata(**changes):
    return dict(
        unit="K", temperature_kind="apparent",
        calibration={"calibration_id": "cal-test", "assumptions": ["test-only"]},
        provenance={"model_id": "greybody-test", "model_version": "v1",
                    "qualification": "simulation-only", "simulation": True,
                    "limitations": ["greybody, not a calibrated LWIR camera"]},
    ) | changes


def test_known_hot_pattern_uses_only_roi_and_inclusive_threshold():
    frame = np.array([[900., 300., 320.], [900., 310., 310.]])
    result = evaluate_thermal(frame, np.ones_like(frame, dtype=bool), thermal_rule(), **metadata())
    assert result.verdict == "anomaly"
    assert result.metrics["max_kelvin"] == 320.0
    assert result.metrics["mean_kelvin"] == 310.0
    assert result.metrics["valid_coverage"] == 1.0
    assert result.rule_details["comparison"] == ">="
    assert result.provenance["simulation"] is True


@pytest.mark.parametrize("changes", [
    {"unit": None}, {"unit": "C"}, {"temperature_kind": None}, {"temperature_kind": "corrected"},
    {"calibration": None}, {"calibration": {}}, {"calibration": {"calibration_id": "other"}},
    {"provenance": None}, {"provenance": {}},
    {"provenance": {"model_id": "other", "model_version": "v1", "qualification": "test"}},
    {"provenance": {"model_id": "greybody-test", "model_version": "v2", "qualification": "test"}},
    {"provenance": {"model_id": "greybody-test", "model_version": "v1"}},
])
def test_unknown_or_mismatched_metadata_never_becomes_normal(changes):
    result = evaluate_thermal(np.full((2, 3), 300.), np.ones((2, 3), bool),
                              thermal_rule(), **metadata(**changes))
    assert result.verdict == "inconclusive"
    assert result.metrics == {}
    assert result.reason != "threshold_evaluated"


@pytest.mark.parametrize("frame,mask,reason", [
    ([[300., 300., 300.]], [[False, False, False]], "insufficient_valid_coverage"),
    ([[300., 300.]], [[True, True]], "roi_out_of_bounds"),
    (None, None, "invalid_array_or_mask"),
    ([[300., 300., 300.]], None, "invalid_array_or_mask"),
    ([[300., 300., 300.]], [[1, 1, 1]], "invalid_array_or_mask"),
    ([[300., 300., 300.]], [[True]], "invalid_array_or_mask"),
    ([["300", "300", "300"]], [[True, True, True]], "invalid_array_or_mask"),
    ([[True, True, True]], [[True, True, True]], "invalid_array_or_mask"),
    ([[300j, 300j, 300j]], [[True, True, True]], "invalid_array_or_mask"),
    ([], [], "invalid_array_or_mask"),
])
def test_thermal_unknown_input_and_roi_are_inconclusive(frame, mask, reason):
    result = evaluate_thermal(frame, mask, thermal_rule(roi=(0, 0, 3, 1)), **metadata())
    assert result.verdict == "inconclusive"
    assert result.reason == reason
    assert result.metrics.get("max_kelvin") is None
    assert result.metrics.get("mean_kelvin") is None


@pytest.mark.parametrize("minimum,verdict", [(0.5, "normal"), (0.5001, "inconclusive")])
def test_thermal_partial_coverage_excludes_invalid_values_and_has_no_fake_zero(minimum, verdict):
    frame = [[900., 300., 310.], [900., np.inf, -1.]]
    mask = [[True, True, True], [False, True, True]]
    result = evaluate_thermal(frame, mask, thermal_rule(min_valid_coverage=minimum), **metadata())
    assert result.verdict == verdict
    assert result.metrics["valid_coverage"] == 0.5
    assert result.metrics["valid_pixels"] == 2
    assert result.metrics["max_kelvin"] == 310.
    assert result.metrics["mean_kelvin"] == 305.


@pytest.mark.parametrize("changes", [
    {"max_kelvin": -1}, {"max_kelvin": np.nan}, {"max_kelvin": np.inf}, {"max_kelvin": True},
    {"min_valid_coverage": 0}, {"min_valid_coverage": 1.1}, {"min_valid_coverage": np.nan},
    {"roi": None}, {"roi": (0, 0, 0, 2)}, {"roi": (-1, 0, 1, 2)},
    {"roi": (0, 0, 1.5, 2)}, {"roi": (False, 0, 1, 2)}, {"roi": (0, 1)},
    {"temperature_kind": "unknown"}, {"calibration_id": ""}, {"model_id": " "}, {"model_version": None},
])
def test_invalid_thermal_rule_is_configuration_error(changes):
    with pytest.raises(ValueError):
        thermal_rule(**changes)


def acoustic_rule(**changes):
    return AcousticRule(**(dict(
        rms_pa=2., channels=(0,), min_duration_s=1., min_valid_coverage=1.,
        calibration_id="cal-test", model_id="direct-path-test", model_version="v1",
    ) | changes))


def acoustic_metadata(**changes):
    result = metadata(unit="Pa")
    del result["temperature_kind"]
    result["provenance"].update(
        model_id="direct-path-test", limitations=["direct-path, block-frozen geometry, no reflections"],
    )
    return result | changes


def test_acoustic_known_sine_rms_preserves_pressure_units_and_model_limits():
    samples = 2. * np.sin(2. * np.pi * 10. * np.arange(1000) / 1000.)
    result = evaluate_acoustic(samples[None, :], np.ones((1, 1000), bool),
                               acoustic_rule(), fs=1000., **acoustic_metadata())
    assert result.verdict == "normal"
    assert result.metrics["rms_pa"] == pytest.approx((np.sqrt(2.),))
    assert result.metrics["duration_s"] == 1.
    assert result.rule_details["unit"] == "Pa"
    assert result.provenance["qualification"] == "simulation-only"
    assert "dba" not in result.metrics


@pytest.mark.parametrize("changes", [
    {"unit": None}, {"unit": "dBA"}, {"calibration": None},
    {"calibration": {"calibration_id": "other"}}, {"provenance": None},
    {"provenance": {"model_id": "different", "model_version": "v1", "qualification": "test"}},
    {"provenance": {"model_id": "direct-path-test", "model_version": "v2", "qualification": "test"}},
    {"provenance": {"model_id": "direct-path-test", "model_version": "v1", "qualification": ""}},
])
def test_acoustic_requires_matching_pa_calibration_and_provenance(changes):
    result = evaluate_acoustic(np.zeros((1, 1000)), np.ones((1, 1000), bool),
                               acoustic_rule(), fs=1000., **acoustic_metadata(**changes))
    assert result.verdict == "inconclusive"
    assert result.metrics == {}


@pytest.mark.parametrize("fs", [None, 0, -1, np.nan, np.inf, True, "1000"])
def test_acoustic_unknown_sample_clock_is_inconclusive(fs):
    result = evaluate_acoustic([[0.]], [[True]], acoustic_rule(), fs=fs, **acoustic_metadata())
    assert result.verdict == "inconclusive"
    assert result.reason == "invalid_sample_rate"


@pytest.mark.parametrize("wave,mask,changes,reason", [
    ([[0., 0.]], [[True, True]], {}, "insufficient_duration"),
    ([[0., 0., 0., 0.]], [[False, False, False, False]], {}, "insufficient_valid_coverage"),
    ([[0., np.nan, np.inf, 0.]], [[True, True, True, True]], {}, "insufficient_valid_coverage"),
    ([[0., 0., 0., 0.]], [[True, True, True, True]], {"channels": (1,)}, "channels_unavailable"),
    (None, None, {}, "invalid_array_or_mask"),
    ([0., 0., 0., 0.], [True, True, True, True], {}, "invalid_array_or_mask"),
    ([[0., 0., 0., 0.]], None, {}, "invalid_array_or_mask"),
    ([[0., 0., 0., 0.]], [[1, 1, 1, 1]], {}, "invalid_array_or_mask"),
    ([[]], [[]], {}, "invalid_array_or_mask"),
    ([["0", "0", "0", "0"]], [[True, True, True, True]], {}, "invalid_array_or_mask"),
    ([[0j, 0j, 0j, 0j]], [[True, True, True, True]], {}, "invalid_array_or_mask"),
])
def test_acoustic_invalid_path_window_or_channel_is_inconclusive(wave, mask, changes, reason):
    result = evaluate_acoustic(wave, mask, acoustic_rule(**changes), fs=4., **acoustic_metadata())
    assert result.verdict == "inconclusive"
    assert result.reason == reason


@pytest.mark.parametrize("minimum,verdict", [(0.5, "anomaly"), (0.5001, "inconclusive")])
def test_acoustic_partial_coverage_is_per_channel_and_threshold_is_inclusive(minimum, verdict):
    wave = [[900., 900., 900., 900.], [1., 1., 1., 1.], [2., -2., 999., np.nan]]
    mask = [[True] * 4, [True] * 4, [True, True, False, True]]
    result = evaluate_acoustic(wave, mask, acoustic_rule(channels=(2, 1), min_valid_coverage=minimum),
                               fs=4., **acoustic_metadata())
    assert result.verdict == verdict
    assert result.metrics["rms_pa"] == (2., 1.)
    assert result.metrics["valid_coverage"] == (0.5, 1.)
    assert result.metrics["valid_samples"] == (2, 4)
    assert result.rule_details["channels"] == (2, 1)


def test_acoustic_one_invalid_channel_cannot_be_hidden_by_another_channel():
    result = evaluate_acoustic([[10.] * 4, [0.] * 4], [[True] * 4, [False] * 4],
                               acoustic_rule(channels=(0, 1)), fs=4., **acoustic_metadata())
    assert result.verdict == "inconclusive"
    assert result.metrics["rms_pa"] == (10., None)


@pytest.mark.parametrize("changes", [
    {"rms_pa": -1}, {"rms_pa": np.nan}, {"rms_pa": np.inf}, {"rms_pa": True},
    {"channels": ()}, {"channels": None}, {"channels": (-1,)}, {"channels": (0, 0)},
    {"channels": (0.5,)}, {"channels": (True,)}, {"min_duration_s": 0},
    {"min_duration_s": -1}, {"min_duration_s": np.inf}, {"min_duration_s": "1"},
    {"min_valid_coverage": 0}, {"min_valid_coverage": 1.1}, {"min_valid_coverage": np.nan},
    {"calibration_id": ""}, {"model_id": " "}, {"model_version": None},
])
def test_invalid_acoustic_rule_is_configuration_error(changes):
    with pytest.raises(ValueError):
        acoustic_rule(**changes)


def test_assessment_is_deeply_immutable_and_exports_independent_finding_details():
    inputs = metadata()
    result = evaluate_thermal([[300.] * 3] * 2, [[True] * 3] * 2, thermal_rule(), **inputs)
    inputs["provenance"]["limitations"].append("changed")
    inputs["calibration"]["assumptions"].append("changed")
    with pytest.raises(FrozenInstanceError):
        result.verdict = "anomaly"
    with pytest.raises(TypeError):
        result.metrics["max_kelvin"] = 0
    with pytest.raises(TypeError):
        result.provenance["simulation"] = False
    assert result.provenance["limitations"] == ("greybody, not a calibrated LWIR camera",)
    assert result.calibration["assumptions"] == ("test-only",)
    exported = result.to_dict()
    assert json.loads(json.dumps(exported, allow_nan=False)) == exported
    exported["provenance"]["limitations"].append("external change")
    assert len(result.provenance["limitations"]) == 1


@pytest.mark.parametrize("bad", [np.array([1.]), {"not-json"}, np.nan, np.inf])
@pytest.mark.parametrize("field", ["calibration", "provenance"])
def test_unknown_non_json_metadata_is_inconclusive_not_mutable_or_nan_details(bad, field):
    inputs = metadata()
    inputs[field]["unknown_quality"] = bad
    result = evaluate_thermal([[300.] * 3] * 2, [[True] * 3] * 2, thermal_rule(), **inputs)
    assert result.verdict == "inconclusive"
    json.dumps(result.to_dict(), allow_nan=False)


def test_corrected_temperature_requires_explicit_matching_rule_and_does_not_recorrect():
    result = evaluate_thermal([[301.] * 3] * 2, [[True] * 3] * 2,
                              thermal_rule(temperature_kind="corrected"), **metadata(temperature_kind="corrected"))
    assert result.verdict == "normal"
    assert result.metrics["max_kelvin"] == 301.
    assert result.rule_details["temperature_kind"] == "corrected"


def test_all_nonfinite_thermal_pixels_produce_none_summaries_not_zero():
    result = evaluate_thermal([[np.nan, np.inf]], [[True, True]],
                              thermal_rule(roi=(0, 0, 2, 1)), **metadata())
    assert result.verdict == "inconclusive"
    assert result.metrics == {"max_kelvin": None, "mean_kelvin": None, "valid_coverage": 0., "valid_pixels": 0}


@pytest.mark.parametrize("level,threshold,verdict", [(0., 1., "normal"), (0., 0., "anomaly"),
                                                    (1e200, 1e201, "normal")])
def test_real_silence_is_valid_but_not_a_substitute_for_unknown_samples(level, threshold, verdict):
    result = evaluate_acoustic([[level] * 4], [[True] * 4], acoustic_rule(rms_pa=threshold),
                               fs=4., **acoustic_metadata())
    assert result.verdict == verdict
    assert result.metrics["rms_pa"] == (level,)


def test_unselected_invalid_acoustic_channel_does_not_reject_selected_valid_channel():
    result = evaluate_acoustic([[0.] * 4, [np.nan] * 4], [[True] * 4, [False] * 4],
                               acoustic_rule(), fs=4., **acoustic_metadata())
    assert result.verdict == "normal"
    assert result.metrics["rms_pa"] == (0.,)


def test_mutating_rule_configuration_lists_cannot_change_rules():
    roi, channels = [0, 0, 2, 1], [0, 1]
    thermal, acoustic = thermal_rule(roi=roi), acoustic_rule(channels=channels)
    roi[2] = 99
    channels.append(99)
    assert thermal.roi == (0, 0, 2, 1)
    assert acoustic.channels == (0, 1)
