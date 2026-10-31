"""Stable semantic labels from physical nearest-hit IDs, never RGB colours."""

from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.sensors.camera_geometry import CameraPose, FisheyeCalibration, PinholeCalibration, calibrated_rays
from sim.runtime.sensors.measurement_control import MeasurementNoise
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.semantic_camera import (
    SemanticCameraConfig,
    SemanticLabel,
    semantic_from_geom_ids,
)

pytestmark = pytest.mark.sim


def _stamp():
    return SensorSampleStamp("semantic-test", "robot", "semantic", "robot/optical", 2, 3, 4, 123)


def _config(labels=None, **kwargs):
    if labels is None:
        labels = (SemanticLabel(0, 7, 101), SemanticLabel(3, 7, 101), SemanticLabel(4, 2, 202))
    return SemanticCameraConfig(PinholeCalibration(3, 2, 2, 2, 1.5, 1), labels, **kwargs)


def test_nearest_physical_geom_ids_map_to_declared_stable_labels_and_zero_for_misses():
    config = _config()
    stamp = _stamp()
    nearest_ids = np.array([[0, 4, 3], [-1, 999, 4]], dtype=np.int32)
    original = nearest_ids.copy()
    result = semantic_from_geom_ids(config, stamp, nearest_ids, source_stamp=stamp)
    np.testing.assert_array_equal(result.class_ids, [[7, 2, 7], [0, 0, 2]])
    np.testing.assert_array_equal(result.instance_ids, [[101, 202, 101], [0, 0, 202]])
    np.testing.assert_array_equal(result.valid_mask, [[True, True, True], [False, False, True]])
    assert result.class_ids.dtype == result.instance_ids.dtype == np.int32
    assert result.stamp == result.source_stamp == stamp
    assert result.simulation is True and result.ground_truth is True
    assert not hasattr(result, "confidence")
    assert result.configuration.noise == MeasurementNoise(unit="label")
    assert result.calibration_metadata["projection"]["width"] == 3
    np.testing.assert_array_equal(nearest_ids, original)
    # Reordering declarations must not reassign IDs, and hidden IDs are not inferred.
    reordered = semantic_from_geom_ids(_config(tuple(reversed(config.labels))), stamp, nearest_ids, source_stamp=stamp)
    np.testing.assert_array_equal(reordered.instance_ids, result.instance_ids)
    assert result.instance_ids[0, 1] == 202  # only the supplied nearest surface, not the occluded instance 101


@pytest.mark.parametrize(
    "changes",
    [
        {"session_id": "other"},
        {"instance_id": "other"},
        {"sensor_id": "other"},
        {"frame_id": "other"},
        {"model_generation": 3},
        {"reset_generation": 4},
        {"sim_time_ns": 124},
        {"sequence": 5},
    ],
)
def test_semantic_rejects_mismatched_raster_identity_generation_or_source_time(changes):
    stamp = _stamp()
    with pytest.raises(ValueError, match="source_stamp"):
        semantic_from_geom_ids(
            _config(), stamp, np.zeros((2, 3), dtype=np.int32), source_stamp=replace(stamp, **changes)
        )


@pytest.mark.parametrize("noise", [MeasurementNoise(unit="m"), MeasurementNoise(unit="label", enabled=True)])
def test_semantic_rejects_wrong_noise_units_and_unsupported_label_noise(noise):
    with pytest.raises(ValueError, match="noise"):
        _config(noise=noise)


def test_disabled_common_noise_retains_its_declared_stddev_and_seed_without_changing_labels():
    noise = MeasurementNoise(unit="label", enabled=False, stddev=2.5, seed=17)
    stamp = _stamp()
    ids = np.array([[0, 4, 3], [-1, 999, 4]], dtype=np.int32)
    result = semantic_from_geom_ids(_config(noise=noise), stamp, ids, source_stamp=stamp)
    clean = semantic_from_geom_ids(_config(), stamp, ids, source_stamp=stamp)
    np.testing.assert_array_equal(result.class_ids, clean.class_ids)
    np.testing.assert_array_equal(result.instance_ids, clean.instance_ids)
    assert result.configuration.noise.to_dict() == {"unit": "label", "enabled": False, "stddev": 2.5, "seed": 17}


@pytest.mark.parametrize(
    "labels",
    [
        (SemanticLabel(-1, 1, 1),),
        (SemanticLabel(1, 0, 1),),
        (SemanticLabel(1, 1, 0),),
        (SemanticLabel(1, 1, 1), SemanticLabel(1, 2, 2)),
        (SemanticLabel(1, 1, 1), SemanticLabel(2, 2, 1)),
        (SemanticLabel(1, 1, 2**31),),
        (SemanticLabel(True, 1, 1),),
    ],
)
def test_semantic_rejects_ambiguous_or_noninteger_label_declarations(labels):
    with pytest.raises(ValueError, match="label"):
        _config(labels)


@pytest.mark.parametrize(
    "ids", [np.zeros((2, 3), dtype=float), np.zeros((2, 3, 3), dtype=np.uint8), np.zeros((3, 2), dtype=np.int32)]
)
def test_semantic_requires_integer_geom_ids_at_the_calibrated_resolution_not_rgb(ids):
    with pytest.raises(ValueError, match="geom_ids"):
        semantic_from_geom_ids(_config(), _stamp(), ids, source_stamp=_stamp())


def test_semantic_preserves_calibrated_fisheye_mask_and_the_actual_mounted_ray_metadata():
    stamp = _stamp()
    calibration = FisheyeCalibration(5, 5, 4 / np.pi, 2.5, 2.5, 180, 2)
    config = SemanticCameraConfig(calibration, (SemanticLabel(0, 7, 101),))
    rays = calibrated_rays(calibration, stamp, base_pose=CameraPose((1, 2, 3)))
    result = semantic_from_geom_ids(config, stamp, np.zeros((5, 5), dtype=np.int32), source_stamp=stamp, rays=rays)
    np.testing.assert_array_equal(result.valid_mask, rays.valid_mask)
    np.testing.assert_array_equal(result.class_ids[~rays.valid_mask], 0)
    assert result.calibration_metadata == rays.calibration_metadata
    with pytest.raises(ValueError, match="rays"):
        semantic_from_geom_ids(
            config,
            stamp,
            np.zeros((5, 5), dtype=np.int32),
            source_stamp=stamp,
            rays=replace(rays, stamp=replace(stamp, reset_generation=4)),
        )
    with pytest.raises(ValueError, match="calibration"):
        semantic_from_geom_ids(_config(), stamp, np.zeros((2, 3), dtype=np.int32), source_stamp=stamp, rays=rays)
