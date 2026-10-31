from __future__ import annotations

import tracemalloc

import mujoco
import numpy as np
import pytest
from sim.runtime.sensors.acoustic_scene import AcousticScene, AcousticSceneSource
from sim.runtime.sensors.acoustics import MicrophoneConfig
from sim.runtime.sensors.samples import SensorSampleStamp

pytestmark = [pytest.mark.sim]


def _stamp(time_ns=0):
    return SensorSampleStamp(
        session_id="acoustic-scene",
        instance_id="robot",
        sensor_id="robot.microphone",
        frame_id="robot/microphone",
        model_generation=1,
        reset_generation=0,
        sequence=0,
        sim_time_ns=time_ns,
    )


def _fixture(*, obstacle="", receiver_child="", source_child="", source_quat="1 0 0 0", source_free=False):
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco><worldbody>
          <body name="receiver">
            <freejoint/>
            <geom name="receiver_shell" type="sphere" size="0.1" mass="1"/>
            {receiver_child}
          </body>
          <body name="source" pos="2 0 0" quat="{source_quat}">
            {'<freejoint name="source_free"/>' if source_free else ""}
            <geom name="source_shell" type="sphere" size="0.1"/>
            {source_child}
          </body>
          {obstacle}
        </worldbody></mujoco>
    """)
    return model, mujoco.MjData(model)


def _source(**changes):
    fields = dict(
        source_id="speaker",
        body_name="source",
        pressure_pa=[1.0, 0.0, 0.0],
        sample_rate_hz=1000,
        start_sim_time_ns=0,
        reference_distance_m=1.0,
    )
    fields.update(changes)
    return AcousticSceneSource(**fields)


def _scene(*, sources=None, receiver_body_name="receiver", channel_offsets_m=((0, 0, 0),)):
    return AcousticScene(
        MicrophoneConfig(
            sample_rate_hz=1000,
            block_samples=6,
            channel_offsets_m=channel_offsets_m,
            speed_of_sound_mps=1000,
        ),
        receiver_body_name=receiver_body_name,
        sources=(_source(),) if sources is None else sources,
    )


def test_physical_body_pose_drives_direct_path_delay_without_advancing_physics():
    model, data = _fixture()
    qpos, qvel, time = data.qpos.copy(), data.qvel.copy(), data.time
    frame = _scene().sample(model, data, _stamp())
    np.testing.assert_allclose(frame.pressure_pa, [[0, 0, 0.5, 0, 0, 0]])
    assert frame.valid_mask.all()
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)
    assert data.time == time
    assert frame.metadata["simulation"] is True
    assert frame.metadata["sources"][0]["position_world_m"] == [2, 0, 0]
    assert frame.metadata["scene"]["pose_source"] == "mujoco_mj_kinematics"


def test_receiver_translation_rotation_and_source_local_mount_use_current_kinematics():
    model, data = _fixture(source_quat="0.7071067811865476 0 0 0.7071067811865476")
    scene = _scene(
        channel_offsets_m=((0, 0, 0), (1, 0, 0)),
        sources=(_source(pressure_pa=np.ones(20), local_offset_m=(0, -1, 0)),),
    )
    initial = scene.sample(model, data, _stamp())
    np.testing.assert_allclose(initial.metadata["sources"][0]["position_world_m"], [3, 0, 0], atol=1e-14)
    data.qpos[:3] = (1, 0, 0)
    data.qpos[3:7] = (np.sqrt(0.5), 0, 0, np.sqrt(0.5))
    before = data.qpos.copy()
    moved = scene.sample(model, data, _stamp())
    np.testing.assert_allclose(moved.metadata["channel_positions_world_m"], [[1, 0, 0], [1, 1, 0]], atol=1e-14)
    np.testing.assert_allclose(moved.pressure_pa[:, 3], [0.5, 1 / np.sqrt(5)], atol=1e-7)
    assert not np.array_equal(moved.pressure_pa, initial.pressure_pa)
    np.testing.assert_array_equal(data.qpos, before)


def test_obstacle_before_source_blocks_direct_path_instead_of_synthesizing_silence():
    obstacle = '<geom name="wall" type="box" pos="1 0 0" size="0.05 0.3 0.3"/>'
    model, data = _fixture(obstacle=obstacle)
    frame = _scene().sample(model, data, _stamp())
    assert not frame.valid_mask.any()
    assert np.isnan(frame.pressure_pa).all()
    assert frame.metadata["blocked_paths"] == [{"source_id": "speaker", "channel": 0}]
    assert frame.metadata["scene"]["occlusion_hits"][0]["geom_name"] == "wall"


@pytest.mark.parametrize(
    "receiver_body_name,source_body_name",
    [
        ("absent", "source"),
        ("receiver", "absent"),
        ("world", "source"),
        ("receiver", "world"),
    ],
)
def test_missing_or_world_body_is_rejected_not_indexed_as_another_body(receiver_body_name, source_body_name):
    model, data = _fixture()
    scene = _scene(receiver_body_name=receiver_body_name, sources=(_source(body_name=source_body_name),))
    with pytest.raises(ValueError, match="body"):
        scene.sample(model, data, _stamp())


@pytest.mark.parametrize("time,stamp_time", [(0.001, 0), (0, 1_000_000), (0.001000001, 1_000_000)])
def test_stamp_time_must_match_current_physics_time_at_nanosecond_precision(time, stamp_time):
    model, data = _fixture()
    data.time = time
    before = data.qpos.copy()
    with pytest.raises(ValueError, match="time"):
        _scene().sample(model, data, _stamp(stamp_time))
    assert data.time == time
    np.testing.assert_array_equal(data.qpos, before)


def test_matching_rounded_physics_time_preserves_stamp_and_waveform_start():
    model, data = _fixture()
    data.time = 0.0010000004
    stamp = _stamp(1_000_000)
    frame = _scene().sample(model, data, stamp)
    assert frame.stamp is stamp
    np.testing.assert_allclose(frame.pressure_pa, [[0, 0.5, 0, 0, 0, 0]])
    assert frame.metadata["sources"][0]["start_sim_time_ns"] == 0
    assert frame.metadata["sources"][0]["source_sim_time_ns"] == stamp.sim_time_ns


@pytest.mark.parametrize("parent,position", [("receiver", "1 0 0"), ("source", "-1 0 0")])
def test_body_descendants_are_real_occluders_not_silently_excluded(parent, position):
    child = f'<body name="child" pos="{position}"><geom name="child_shell" type="sphere" size="0.1"/></body>'
    model, data = _fixture(**{f"{parent}_child": child})
    frame = _scene().sample(model, data, _stamp())
    assert not frame.valid_mask.any()
    assert frame.metadata["scene"]["occlusion_hits"][0]["geom_name"] == "child_shell"
    assert frame.metadata["scene"]["receiver_exclusion"] == "exact_body_only_not_subtree"


def test_occlusion_is_tested_per_receiver_channel():
    obstacle = '<geom name="small_blocker" type="sphere" pos="1 0 0" size="0.1"/>'
    model, data = _fixture(obstacle=obstacle)
    frame = _scene(channel_offsets_m=((0, 0, 0), (0, 1, 0))).sample(model, data, _stamp())
    assert not frame.valid_mask[0].any()
    assert np.isnan(frame.pressure_pa[0]).all()
    assert frame.valid_mask[1].all()
    assert np.isfinite(frame.pressure_pa[1]).all()
    assert frame.metadata["blocked_paths"] == [{"source_id": "speaker", "channel": 0}]


@pytest.mark.parametrize(
    "changes",
    [
        {"local_offset_m": (0, 1)},
        {"local_offset_m": (0, float("nan"), 0)},
        {"body_name": ""},
        {"source_id": ""},
        {"pressure_pa": []},
        {"pressure_pa": [float("nan")]},
        {"sample_rate_hz": 0},
        {"sample_rate_hz": True},
        {"start_sim_time_ns": -1},
        {"reference_distance_m": 0},
    ],
)
def test_scene_source_declaration_rejects_missing_physical_information(changes):
    with pytest.raises(ValueError):
        _source(**changes)


def test_moving_source_uses_updated_body_pose_and_ray_geometry_without_stepping():
    model, data = _fixture(source_free=True)
    scene = _scene()
    initial = scene.sample(model, data, _stamp())
    address = int(model.jnt_qposadr[model.joint("source_free").id])
    data.qpos[address : address + 3] = (4, 0, 0)
    before = data.qpos.copy()
    moved = scene.sample(model, data, _stamp())
    np.testing.assert_allclose(initial.pressure_pa, [[0, 0, 0.5, 0, 0, 0]])
    np.testing.assert_allclose(moved.pressure_pa, [[0, 0, 0, 0, 0.25, 0]])
    np.testing.assert_array_equal(data.qpos, before)
    assert moved.valid_mask.all()
    assert data.time == 0


def test_dynamic_obstacle_pose_is_refreshed_before_ray_query():
    obstacle = """<body name="blocker" pos="1 0 0">
      <joint name="blocker_y" type="slide" axis="0 1 0"/>
      <geom name="moving_wall" type="sphere" size="0.1"/>
    </body>"""
    model, data = _fixture(obstacle=obstacle)
    scene = _scene()
    assert not scene.sample(model, data, _stamp()).valid_mask.any()
    address = int(model.jnt_qposadr[model.joint("blocker_y").id])
    data.qpos[address] = 2
    before = data.qpos.copy()
    clear = scene.sample(model, data, _stamp())
    assert clear.valid_mask.all()
    np.testing.assert_allclose(clear.pressure_pa, [[0, 0, 0.5, 0, 0, 0]])
    np.testing.assert_array_equal(data.qpos, before)


def test_scene_source_snapshot_is_immutable_and_isolated_from_input_mutation():
    original = np.array([1.0, 0.0, 0.0])
    declaration = _source(pressure_pa=original)
    original[:] = np.nan
    with pytest.raises(ValueError):
        declaration.pressure_pa.setflags(write=True)
    model, data = _fixture()
    scene = _scene(sources=(declaration,))
    first = scene.sample(model, data, _stamp())
    data.time = 0.001
    second = scene.sample(model, data, _stamp(1_000_000))
    np.testing.assert_array_equal(first.pressure_pa, [[0, 0, 0.5, 0, 0, 0]])
    np.testing.assert_array_equal(second.pressure_pa, [[0, 0.5, 0, 0, 0, 0]])
    assert second.metadata["sources"][0]["source_sim_time_ns"] == 1_000_000


def test_scene_block_does_not_allocate_a_full_waveform_for_new_geometry_stamp():
    model, data = _fixture()
    declaration = _source(pressure_pa=np.ones(400_000))
    scene = _scene(sources=(declaration,))
    data.time = 0.001
    tracemalloc.start()
    try:
        frame = scene.sample(model, data, _stamp(1_000_000))
        _, peak_bytes = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    np.testing.assert_array_equal(frame.pressure_pa, [[0, 0.5, 0.5, 0.5, 0.5, 0.5]])
    assert peak_bytes < declaration.pressure_pa.nbytes
