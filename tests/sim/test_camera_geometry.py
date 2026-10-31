"""Calibrated simulation camera geometry, independent of any renderer."""

import json

import numpy as np
import pytest
from sim.runtime.sensors.camera_geometry import (
    CameraPose,
    FisheyeCalibration,
    GimbalPose,
    PanoramaCalibration,
    PinholeCalibration,
    calibrated_rays,
    project_local_directions,
)
from sim.runtime.sensors.samples import SensorSampleStamp

pytestmark = pytest.mark.sim


def _stamp():
    return SensorSampleStamp("camera-test", "robot", "camera", "robot/optical", 2, 3, 4, 123)


def test_pinhole_pixel_centres_preserve_optical_axes_and_intrinsic_calibration():
    calibration = PinholeCalibration(width=3, height=3, fx_px=2, fy_px=4, cx_px=1.5, cy_px=1.5)
    stamp = _stamp()
    rays = calibrated_rays(calibration, stamp)
    assert rays.stamp == stamp
    assert rays.simulation is True
    assert rays.origins_m.shape == rays.directions_world.shape == (3, 3, 3)
    np.testing.assert_allclose(rays.origins_m, 0)
    np.testing.assert_allclose(rays.directions_world[1, 1], [0, 0, 1])
    np.testing.assert_allclose(rays.directions_world[1, 2], np.array([0.5, 0, 1]) / np.sqrt(1.25))
    np.testing.assert_allclose(rays.directions_world[2, 1], np.array([0, 0.25, 1]) / np.sqrt(1.0625))
    np.testing.assert_allclose(np.linalg.norm(rays.directions_world, axis=-1), 1)
    assert rays.valid_mask.all()
    metadata = json.loads(json.dumps(rays.calibration_metadata, allow_nan=False))
    assert metadata["projection"]["model"] == "pinhole"
    assert metadata["projection"]["fx_px"] == 2
    assert metadata["optical_axes"] == {"x": "right", "y": "down", "z": "forward"}
    assert metadata["pixel_coordinates"] == "u=column+0.5,v=row+0.5; image edges at 0,width and 0,height"


def test_equidistant_fisheye_has_declared_fov_and_valid_disk_not_a_pinhole_approximation():
    calibration = FisheyeCalibration(
        width=5,
        height=5,
        focal_px=4 / np.pi,
        cx_px=2.5,
        cy_px=2.5,
        fov_deg=180,
        valid_radius_px=2,
    )
    rays = calibrated_rays(calibration, _stamp())
    np.testing.assert_allclose(rays.directions_world[2, 2], [0, 0, 1])
    np.testing.assert_allclose(rays.directions_world[2, 4], [1, 0, 0], atol=1e-15)
    np.testing.assert_allclose(rays.directions_world[4, 2], [0, 1, 0], atol=1e-15)
    assert rays.valid_mask.sum() == 13
    assert not rays.valid_mask[0, 0]
    np.testing.assert_array_equal(rays.directions_world[~rays.valid_mask], 0)
    assert rays.calibration_metadata["projection"]["model"] == "equidistant_fisheye"
    wider_disk = FisheyeCalibration(5, 5, 4 / np.pi, 2.5, 2.5, fov_deg=90, valid_radius_px=2)
    assert calibrated_rays(wider_disk, _stamp()).valid_mask.sum() == 5


def test_panorama_longitude_wrap_and_latitude_follow_the_declared_optical_convention():
    calibration = PanoramaCalibration(width=4, height=3)
    rays = calibrated_rays(calibration, _stamp())
    s = np.sqrt(0.5)
    np.testing.assert_allclose(rays.directions_world[1], [[-s, 0, -s], [-s, 0, s], [s, 0, s], [s, 0, -s]], atol=1e-15)
    assert np.all(rays.directions_world[0, :, 1] < 0)
    assert np.all(rays.directions_world[-1, :, 1] > 0)
    assert rays.valid_mask.all()
    metadata = rays.calibration_metadata["projection"]
    assert metadata["model"] == "equirectangular"
    assert metadata["longitude_min_deg"] == -180
    assert metadata["longitude_max_deg"] == 180
    assert (
        metadata["angle_convention"]
        == "longitude=atan2(x,z),latitude=atan2(-y,hypot(x,z)); u increases longitude,v decreases latitude"
    )
    partial = calibrated_rays(PanoramaCalibration(1, 1, 0, 90, 0, 90), _stamp())
    np.testing.assert_allclose(partial.directions_world[0, 0], [0.5, -s, 0.5], atol=1e-15)


def test_ptz_composes_base_mount_pan_tilt_without_moving_the_body():
    s = np.sqrt(0.5)
    base = CameraPose((10, 20, 30), (s, 0, s, 0))
    mount = CameraPose((1, 2, 3), (s, 0, 0, s))
    calibration = PinholeCalibration(1, 1, 1, 1, 0.5, 0.5)
    rays = calibrated_rays(calibration, _stamp(), base_pose=base, mount_pose=mount, gimbal=GimbalPose(pan_deg=90))
    np.testing.assert_allclose(rays.origins_m[0, 0], [13, 22, 29], atol=1e-14)
    np.testing.assert_allclose(rays.directions_world[0, 0], [0, 1, 0], atol=1e-14)
    tilted = calibrated_rays(calibration, _stamp(), base_pose=base, mount_pose=mount, gimbal=GimbalPose(90, 90))
    np.testing.assert_allclose(tilted.directions_world[0, 0], [0, 0, -1], atol=1e-14)
    assert base.position_m == (10, 20, 30)
    assert mount.position_m == (1, 2, 3)
    assert rays.calibration_metadata["transform_composition"] == "base_pose * mount_pose * Ry(pan) * Rx(tilt)"
    assert rays.calibration_metadata["gimbal"]["pan_deg"] == 90
    np.testing.assert_allclose(rays.calibration_metadata["camera_pose_world"]["position_m"], [13, 22, 29])


@pytest.mark.parametrize(
    "factory,kwargs",
    [
        (PinholeCalibration, {"width": 0}),
        (PinholeCalibration, {"height": 2.5}),
        (PinholeCalibration, {"fx_px": 0}),
        (PinholeCalibration, {"fy_px": -1}),
        (PinholeCalibration, {"cx_px": float("nan")}),
        (FisheyeCalibration, {"fov_deg": 0}),
        (FisheyeCalibration, {"fov_deg": 360}),
        (FisheyeCalibration, {"focal_px": -1}),
        (FisheyeCalibration, {"valid_radius_px": 0}),
        (PanoramaCalibration, {"longitude_max_deg": 181}),
        (PanoramaCalibration, {"longitude_min_deg": 180}),
        (PanoramaCalibration, {"latitude_max_deg": 91}),
        (PanoramaCalibration, {"latitude_min_deg": 90}),
        (CameraPose, {"orientation_wxyz": (0, 0, 0, 0)}),
        (CameraPose, {"position_m": (0, float("inf"), 0)}),
        (GimbalPose, {"pan_deg": 181}),
        (GimbalPose, {"tilt_deg": -91}),
        (GimbalPose, {"pan_limits_deg": (20, -20)}),
        (GimbalPose, {"pan_deg": 11, "pan_limits_deg": (-10, 10)}),
        (GimbalPose, {"tilt_limits_deg": (0, float("nan"))}),
    ],
)
def test_calibration_pose_and_gimbal_limits_reject_invalid_declarations(factory, kwargs):
    defaults = {
        PinholeCalibration: dict(width=3, height=3, fx_px=1, fy_px=1, cx_px=1.5, cy_px=1.5),
        FisheyeCalibration: dict(width=3, height=3, focal_px=1, cx_px=1.5, cy_px=1.5, fov_deg=180, valid_radius_px=1),
        PanoramaCalibration: dict(width=3, height=3),
        CameraPose: {},
        GimbalPose: {},
    }
    with pytest.raises(ValueError):
        factory(**(defaults[factory] | kwargs))


@pytest.mark.parametrize(
    "calibration",
    [
        PinholeCalibration(5, 3, 2.3, 4.1, 1.7, 1.2),
        FisheyeCalibration(5, 5, 4 / np.pi, 2.5, 2.5, 180, 2),
        PanoramaCalibration(12, 6),
        PanoramaCalibration(5, 3, 20, 140, -30, 60),
        PanoramaCalibration(8, 4, 0, 360),
    ],
)
def test_projection_roundtrip_uses_the_same_calibration_as_pixel_rays(calibration):
    rays = calibrated_rays(calibration, _stamp())
    uv, valid = project_local_directions(calibration, rays.directions_world)
    u, v = np.meshgrid(np.arange(calibration.width) + 0.5, np.arange(calibration.height) + 0.5)
    np.testing.assert_array_equal(valid, rays.valid_mask)
    np.testing.assert_allclose(uv[valid], np.stack((u, v), axis=-1)[valid], atol=1e-12)


def test_projection_handles_panorama_seam_and_rejects_zero_or_backwards_pinhole_rays():
    panorama = PanoramaCalibration(8, 4)
    uv, valid = project_local_directions(panorama, np.array([[0, 0, -1], [-0.0, 0, -1], [0, 0, 0]]))
    np.testing.assert_array_equal(valid, [True, True, False])
    np.testing.assert_allclose(uv[:2], [[0, 2], [0, 2]])
    pinhole = PinholeCalibration(2, 2, 1, 1, 1, 1)
    _, valid = project_local_directions(pinhole, np.array([[0, 0, 1], [0, 0, -1], [100, 0, 1], [np.nan, 0, 1]]))
    np.testing.assert_array_equal(valid, [True, False, False, False])


def test_ray_sample_requires_a_sensor_stamp_and_preserves_bounded_gimbal_endpoints():
    calibration = PinholeCalibration(1, 1, 1, 1, 0.5, 0.5)
    with pytest.raises(ValueError, match="stamp"):
        calibrated_rays(calibration, None)
    for pan in (-20, 20):
        calibrated_rays(calibration, _stamp(), gimbal=GimbalPose(pan, 0, (-20, 20), (-10, 10)))
