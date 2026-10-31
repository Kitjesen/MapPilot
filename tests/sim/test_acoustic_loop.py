"""Continuous machinery sources retain physical delay and bounded clip storage."""

import mujoco
import numpy as np
import pytest
from sim.catalog.measurement import validate_scene
from sim.runtime.sensors.acoustic_scene import AcousticScene, AcousticSceneSource
from sim.runtime.sensors.acoustics import MicrophoneConfig
from sim.runtime.sensors.samples import SensorSampleStamp


def source_values():
    return dict(
        source_id="motor",
        body_name="motor",
        pressure_pa=[1.0, 0.0, -1.0, 0.0],
        sample_rate_hz=1000,
        start_sim_time_ns=0,
        reference_distance_m=1.0,
        loop=True,
    )


def sample(values, *, time_ns=10_000_000_000, distance=2.0):
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
        <body name="receiver"><geom type="sphere" size=".1"/></body>
        <body name="motor" pos="{distance} 0 0"><geom type="sphere" size=".1"/></body>
        </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.time = time_ns / 1e9
    stamp = SensorSampleStamp(
        session_id="loop-test",
        instance_id="robot",
        sensor_id="robot.mic",
        frame_id="robot/mic",
        model_generation=0,
        reset_generation=0,
        sequence=0,
        sim_time_ns=time_ns,
    )
    scene = AcousticScene(
        MicrophoneConfig(sample_rate_hz=1000, block_samples=6, channel_offsets_m=((0, 0, 0),), speed_of_sound_mps=1000),
        receiver_body_name="receiver",
        sources=(AcousticSceneSource(**values),),
    )
    return scene.sample(model, data, stamp)


def test_loop_is_audible_after_navigation_duration_with_delay_and_attenuation():
    frame = sample(source_values())
    np.testing.assert_allclose(frame.pressure_pa, [[-0.5, 0, 0.5, 0, -0.5, 0]])
    assert frame.valid_mask.all()
    assert frame.metadata["sources"][0]["loop"] is True


def test_loop_wrap_interpolates_last_sample_to_first_without_early_sound():
    np.testing.assert_allclose(sample(source_values(), distance=2.5).pressure_pa, [[-0.2, -0.2, 0.2, 0.2, -0.2, -0.2]])
    np.testing.assert_allclose(sample(source_values(), time_ns=0).pressure_pa, [[0, 0, 0.5, 0, -0.5, 0]])
    values = source_values() | {"start_sim_time_ns": 20_000_000_000}
    assert not sample(values).pressure_pa.any()


def test_finite_source_remains_silent_after_clip():
    values = source_values()
    values.pop("loop")
    frame = sample(values)
    assert not frame.pressure_pa.any()
    assert frame.metadata["sources"][0]["loop"] is False


def test_catalog_preserves_explicit_loop_and_rejects_non_boolean():
    scene = {"acoustic_sources": [source_values()]}
    assert validate_scene(scene, geom_names=set(), body_names={"motor"})["acoustic_sources"][0]["loop"] is True
    for invalid in (1, "true", None):
        scene["acoustic_sources"][0]["loop"] = invalid
        with pytest.raises(ValueError, match="loop"):
            validate_scene(scene, geom_names=set(), body_names={"motor"})
        with pytest.raises(ValueError, match="loop"):
            AcousticSceneSource(**scene["acoustic_sources"][0])
