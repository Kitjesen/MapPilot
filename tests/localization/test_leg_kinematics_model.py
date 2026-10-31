import copy
import os
from pathlib import Path

import numpy as np
import pytest
from tools.bench.leg_kinematics import (
    DEFAULT_PROBE,
    GO2_URDF,
    SDK_NAMES,
    load_leg,
    map_joint_sample,
    probe_cases,
    validate,
)


@pytest.fixture
def leg():
    return load_leg(GO2_URDF, "FR_foot", robot_model="go2", frame_id="body")


@pytest.fixture
def sample():
    # Deliberately different from both SDK and source-XML traversal order.
    names = list(reversed(SDK_NAMES))
    values = {name: index * .01 for index, name in enumerate(SDK_NAMES)}
    return {"robot_model": "go2", "frame_id": "body", "names": names,
            "position": [values[name] for name in names], "velocity": [2 * values[name] for name in names],
            "stamp_s": 10., "time_basis": "aligned_measurement"}


@pytest.fixture
def probe():
    path = Path(os.environ.get("LINGTU_LEG_KINEMATICS_PROBE", DEFAULT_PROBE))
    if not path.is_file():
        pytest.skip(f"native probe is not built: {path}")
    return path


def test_named_mapping_does_not_use_xml_or_motor_array_order(leg, sample):
    result = map_joint_sample(leg, sample)
    np.testing.assert_allclose(result["q"], [0, .01, .02])
    np.testing.assert_allclose(result["dq"], [0, .02, .04])
    assert result["timestamps_aligned"]
    assert leg.joint_names == ("FR_hip_joint", "FR_thigh_joint", "FR_calf_joint")


@pytest.mark.parametrize(("field", "value", "reason"), [
    ("robot_model", "another_robot", "model mismatch"),
    ("frame_id", "unknown", "reference frame"),
    ("time_basis", "unknown", "time basis"),
    ("stamp_s", float("nan"), "joint time"),
    ("position", [0.], "arrays"),
    ("velocity", [0.], "arrays"),
])
def test_input_contract_rejects_ambiguous_or_incomplete_samples(leg, sample, field, value, reason):
    sample[field] = value
    with pytest.raises(ValueError, match=reason):
        map_joint_sample(leg, sample)


def test_input_contract_rejects_duplicate_and_missing_joint(leg, sample):
    duplicate = copy.deepcopy(sample)
    duplicate["names"][0] = duplicate["names"][1]
    with pytest.raises(ValueError, match="duplicate"):
        map_joint_sample(leg, duplicate)
    sample["names"][sample["names"].index("FR_hip_joint")] = "unrelated_joint"
    with pytest.raises(ValueError, match="missing required"):
        map_joint_sample(leg, sample)


def test_receive_only_or_unknown_contact_never_fabricates_velocity(leg, sample, probe):
    sample["time_basis"] = "receive_time"
    mapped = map_joint_sample(leg, sample)
    assert not mapped["timestamps_aligned"]
    cases = [{"model": leg, **mapped, "contact": "stance"},
             {"model": leg, **mapped, "timestamps_aligned": True, "contact": "unknown"},
             {"model": leg, **mapped, "timestamps_aligned": True, "contact": "swing"},
             {"model": leg, **mapped, "timestamps_aligned": True, "contact": "slipping"}]
    for result in probe_cases(probe, cases):
        assert not result["usable"]
        assert "velocity" not in result
        assert "residual" not in result


def test_aligned_joints_without_contact_or_gyro_do_not_authorize_velocity(leg, sample, probe):
    mapped = map_joint_sample(leg, sample)
    result = probe_cases(probe, [{"model": leg, **mapped}])[0]
    assert not result["usable"]
    assert "velocity" not in result
    with pytest.raises(ValueError, match="explicit gyro"):
        probe_cases(probe, [{"model": leg, **mapped, "contact": "stance"}])
    missing_joint_time = {"model": leg, **mapped, "contact": "stance", "omega": np.zeros(3), "gyro_stamp_s": 10.}
    del missing_joint_time["joint_stamp_s"]
    with pytest.raises(ValueError, match="joint timestamp"):
        probe_cases(probe, [missing_joint_time])


def test_four_leg_native_model_matches_mujoco_and_finite_differences(probe):
    report = validate(probe, samples_per_leg=8)
    assert report["passed"]
    assert report["cases"] == 64
    assert report["legs"] == ["FR", "FL", "RR", "RL"]
    assert not report["physical_contact_detected"]
    assert not report["hardware_calibration_validated"]
    assert not report["filter_fusion_enabled"]
