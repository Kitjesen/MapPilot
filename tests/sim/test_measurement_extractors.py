"""Same-owner physical measurement endpoints, no renderer or playback process."""

from contextlib import contextmanager
from fractions import Fraction

import numpy as np
import pytest
from sim.runtime.sensors.camera_geometry import CameraPose, GimbalPose, PinholeCalibration, calibrated_rays
from sim.runtime.sensors.contracts import DeadlinePolicy, ScheduledSensorSample, SensorRoute, SensorStreamPlan
from sim.runtime.sensors.measurement_control import MeasurementNoise
from sim.runtime.sensors.measurement_scene import thermal_calibration
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.thermal import ThermalConfig, ThermalSurface


def _scheduled(kind="thermal", time_ns=0):
    stream = SensorStreamPlan(
        kind,
        "robot",
        f"robot.{kind}",
        "robot/camera",
        "measurement",
        Fraction(100 if kind == "acoustic" else 30),
        SensorRoute("physics", "mujoco_measurement", "in_process"),
    )
    return ScheduledSensorSample("fixture", stream, 2, 3, 0, time_ns, DeadlinePolicy.DROP_INTERMEDIATE)


def _scene():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body name="robot" pos="0 0 0"><freejoint/><geom type="sphere" size=".01"/></body>
      <geom name="wall" type="box" pos="0 0 4" size="4 4 .1"/>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _snapshot(time_ns=0):
    return dict(session_id="fixture", model_generation=2, reset_generation=3, sim_time_ns=time_ns)


def test_thermal_endpoint_uses_actual_source_time_mount_and_noise_control():
    from sim.runtime.sensors.measurement_extractors import MountedCamera, ThermalCameraExtractor

    model, data = _scene()
    lease = []

    @contextmanager
    def read_state():
        lease.append("enter")
        try:
            yield model, data
        finally:
            lease.append("exit")

    calibration = PinholeCalibration(3, 3, 3, 3, 1.5, 1.5)
    request = _scheduled()
    config = ThermalConfig(
        thermal_calibration(calibrated_rays(calibration, SensorSampleStamp.from_scheduled(request))), 293.15
    )
    camera = MountedCamera("robot", calibration, CameraPose((0, 0, 0.1)))
    extractor = ThermalCameraExtractor(read_state, camera, config, {"wall": ThermalSurface(400, 1)})
    data.time = 0.034
    sample = extractor(_scheduled(time_ns=33_333_334), _snapshot(34_000_000))
    assert sample.stamp.sim_time_ns == 34_000_000
    np.testing.assert_array_equal(sample.temperature_k, 400)
    assert lease == ["enter", "exit"]
    assert sample.metadata["scheduled_deadline_ns"] == 33_333_334
    assert sample.metadata["camera_calibration"]["ray_calibration"]["mount_pose"]["position_m"] == [0, 0, 0.1]
    extractor.configure_noise(MeasurementNoise("K", True, 0.5, 7))
    noisy = extractor(_scheduled(time_ns=33_333_334), _snapshot(34_000_000))
    assert not np.array_equal(noisy.temperature_k, sample.temperature_k)
    extractor.set_noise_enabled(False)
    np.testing.assert_array_equal(extractor(_scheduled(), _snapshot(34_000_000)).temperature_k, 400)
    extractor.set_gimbal(GimbalPose(pan_deg=180))
    turned = extractor(_scheduled(), _snapshot(34_000_000))
    assert not turned.valid_mask.any()


def test_endpoint_rejects_wrong_session_and_releases_physics_on_missing_geom():
    from sim.runtime.sensors.measurement_extractors import MountedCamera, ThermalCameraExtractor

    model, data = _scene()
    active = []

    @contextmanager
    def read_state():
        active.append(1)
        try:
            yield model, data
        finally:
            active.pop()

    calibration = PinholeCalibration(1, 1, 1, 1, 0.5, 0.5)
    config = ThermalConfig(
        thermal_calibration(calibrated_rays(calibration, SensorSampleStamp.from_scheduled(_scheduled()))), 293
    )
    extractor = ThermalCameraExtractor(
        read_state, MountedCamera("robot", calibration), config, {"missing": ThermalSurface(400, 1)}
    )
    with pytest.raises(ValueError, match="session"):
        extractor(_scheduled(), {**_snapshot(), "session_id": "wrong"})
    with pytest.raises(ValueError, match="geom"):
        extractor(_scheduled(), _snapshot())
    assert not active


def test_semantic_endpoint_uses_named_physical_instance_and_mount():
    from contextlib import nullcontext

    from sim.runtime.sensors.measurement_extractors import MountedCamera, SemanticCameraExtractor

    model, data = _scene()
    calibration = PinholeCalibration(3, 3, 3, 3, 1.5, 1.5)
    extractor = SemanticCameraExtractor(
        lambda: nullcontext((model, data)), MountedCamera("robot", calibration), {"wall": (7, 12)}
    )
    sample = extractor(_scheduled("semantic"), _snapshot())
    assert sample.ground_truth and sample.simulation
    np.testing.assert_array_equal(sample.class_ids, 7)
    np.testing.assert_array_equal(sample.instance_ids, 12)
    with pytest.raises(ValueError, match="unsupported"):
        extractor.configure_noise(MeasurementNoise("label", True, 1, 1))
    extractor.set_gimbal(GimbalPose(pan_deg=180))
    sample = extractor(_scheduled("semantic"), _snapshot())
    assert not sample.valid_mask.any() and not sample.instance_ids.any()


def test_acoustic_endpoint_outputs_pressure_with_current_body_pose():
    from contextlib import nullcontext

    from sim.runtime.sensors.acoustic_scene import AcousticScene, AcousticSceneSource
    from sim.runtime.sensors.acoustics import MicrophoneConfig
    from sim.runtime.sensors.measurement_extractors import AcousticExtractor

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body name="robot"><geom type="sphere" size=".01"/></body>
      <body name="pump" pos="2 0 0"><geom type="sphere" size=".1"/></body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    data.time = 0.1
    scene = AcousticScene(
        MicrophoneConfig(8000, 80, ((0, 0, 0),)),
        receiver_body_name="robot",
        sources=(AcousticSceneSource("pump", "pump", np.ones(8000), 8000, 0, 1),),
    )
    extractor = AcousticExtractor(lambda: nullcontext((model, data)), scene)
    result = extractor(_scheduled("acoustic", 99_000_000), _snapshot(100_000_000))
    assert result.pressure_pa.shape == (1, 80)
    np.testing.assert_allclose(result.pressure_pa, 0.5)
    assert result.stamp.sim_time_ns == 100_000_000
    extractor.set_rate_hz(100)
    with pytest.raises(ValueError, match="block"):
        extractor.set_rate_hz(30)


def test_engine_measurement_lease_releases_lock_on_failure():
    import threading

    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    model, data = _scene()
    engine = MuJoCoEngine.__new__(MuJoCoEngine)
    engine._model, engine._data = model, data
    engine._data_lock = threading.Lock()
    with pytest.raises(RuntimeError, match="sampling failed"):
        with engine.measurement_state() as (actual_model, actual_data):
            assert actual_model is model and actual_data is data
            assert not engine._data_lock.acquire(blocking=False)
            raise RuntimeError("sampling failed")
    assert engine._data_lock.acquire(blocking=False)
    engine._data_lock.release()


def test_initial_acoustic_plan_rate_cannot_overlap_waveform_blocks():
    from contextlib import nullcontext
    from dataclasses import replace

    from sim.runtime.sensors.acoustic_scene import AcousticScene
    from sim.runtime.sensors.acoustics import MicrophoneConfig
    from sim.runtime.sensors.measurement_extractors import AcousticExtractor

    model, data = _scene()
    extractor = AcousticExtractor(
        lambda: nullcontext((model, data)),
        AcousticScene(MicrophoneConfig(8000, 80, ((0, 0, 0),)), receiver_body_name="robot", sources=()),
    )
    request = _scheduled("acoustic")
    request = replace(request, stream=replace(request.stream, rate_hz=Fraction(200)))
    with pytest.raises(ValueError, match="cadence"):
        extractor(request, _snapshot())


@pytest.mark.parametrize("projection", ["fisheye", "panorama"])
def test_wide_angle_projection_labels_actual_front_and_rear_geometry(projection):
    from contextlib import nullcontext

    from sim.runtime.sensors.camera_geometry import FisheyeCalibration, PanoramaCalibration
    from sim.runtime.sensors.measurement_extractors import MountedCamera, SemanticCameraExtractor

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="robot"><geom type="sphere" size=".01"/></body>
      <geom name="front" type="box" pos="0 0 3" size="2 2 .1"/>
      <geom name="rear" type="box" pos="0 0 -3" size="2 2 .1"/>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    calibration = (PanoramaCalibration(32, 16) if projection == "panorama"
                   else FisheyeCalibration(17, 17, 4, 8.5, 8.5, 200, 7))
    extractor = SemanticCameraExtractor(lambda: nullcontext((model, data)), MountedCamera("robot", calibration),
                                        {"front": (1, 101), "rear": (2, 102)})
    result = extractor(_scheduled("semantic"), _snapshot())
    assert 101 in result.instance_ids
    assert result.valid_mask.shape == (calibration.height, calibration.width)
    if projection == "panorama":
        assert 102 in result.instance_ids
    else:
        assert not result.valid_mask[0, 0] and result.instance_ids[0, 0] == 0
        extractor.set_gimbal(GimbalPose(pan_deg=180))
        result = extractor(_scheduled("semantic"), _snapshot())
        assert 102 in result.instance_ids and 101 not in result.instance_ids
