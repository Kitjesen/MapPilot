"""Product feeder measurement controls and exact source-time behavior."""

from fractions import Fraction
from types import SimpleNamespace

import pytest
from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.gnss import GnssConfig
from sim.scripts.mujoco.formal_gnss import FormalGnss


def measurement():
    stream = SensorStreamPlan(
        "gnss",
        "robot",
        "robot.gnss",
        "robot/antenna",
        "lingtu.dds.GnssFix",
        Fraction(10),
        SensorRoute("physics", "mujoco_gnss", "typed_dds"),
        gnss=GnssConfig(
            0, 0, 0, "robot/base", antenna_offset_m=(0, 0, 0.14), horizontal_std_m=2, vertical_std_m=3, seed=13
        ),
    )
    return FormalGnss(stream, "a" * 32, 2, 3)


def test_owner_controls_change_measurement_not_source_pose():
    sampler = measurement()
    pose = SimpleNamespace(xpos=[0.0, 0.0, 0.6], xquat=[1.0, 0.0, 0.0, 0.0])
    data = SimpleNamespace(get_body_pose=lambda name: (pose.xpos, pose.xquat))
    assert sampler.sample(data, 4.0).fix.fix_type == 0
    sampler.set_fix_available(True)
    fix = sampler.sample(data, 4.1)
    assert fix.fix.alt == pytest.approx(0.74)
    assert fix.stamp.sim_time_ns == 4_100_000_000
    sampler.set_noise_enabled(True)
    noisy = sampler.sample(data, 4.2)
    assert noisy.fix.covariance[0] > 0 and noisy.configuration.noise_enabled
    assert pose.xpos == [0.0, 0.0, 0.6]
    assert pose.xquat == [1.0, 0.0, 0.0, 0.0]
    sampler.set_noise_enabled(False)
    assert sampler.sample(data, 4.3).fix.alt == pytest.approx(0.74)
    sampler.set_fix_available(False)
    assert sampler.sample(data, 4.4).fix.fix_type == 0


def test_late_tick_samples_current_pose_once_and_records_missed_deadlines():
    sampler = measurement()
    pose = SimpleNamespace(xpos=[0.0, 0.0, 0.6], xquat=[1.0, 0.0, 0.0, 0.0])
    data = SimpleNamespace(get_body_pose=lambda name: (pose.xpos, pose.xquat))
    assert sampler.sample(data, 4.0).stamp.sequence == 0
    assert sampler.sample(data, 4.005) is None
    late = sampler.sample(data, 4.345)
    assert (late.stamp.sequence, late.stamp.sim_time_ns) == (1, 4_345_000_000)
    assert sampler.evidence()["missed_deadlines"] == 2
    assert sampler.sample(data, 4.35) is None
    assert sampler.sample(data, 4.4).stamp.sequence == 2
    with pytest.raises(ValueError, match="backwards"):
        sampler.sample(data, 0.0)


@pytest.mark.parametrize("value", [True, False])
def test_configured_body_must_exist_even_during_outage(value):
    sampler = measurement()
    sampler.set_fix_available(value)

    def missing(name):
        raise KeyError(name)

    with pytest.raises(KeyError):
        sampler.sample(SimpleNamespace(get_body_pose=missing), 0.0)
    assert sampler.evidence()["sampled_count"] == 0


def test_dynamic_mujoco_pose_matches_post_integration_timestamp_and_antenna():
    from dataclasses import replace

    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    mujoco = pytest.importorskip("mujoco")
    import numpy as np

    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <option timestep="0.005" gravity="0 0 0" integrator="Euler"/>
      <worldbody><body name="base" pos="0 0 0.6"><freejoint/>
        <geom type="sphere" size="0.1" mass="1"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qvel[:] = [2.0, 0.3, 0.0, 0.0, 0.0, 1.5]
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    # Establish that this fixture actually exposes the post-step cache lag.
    assert np.linalg.norm(data.xpos[1] - data.qpos[:3]) > 0.001
    qpos, qvel, time_s = data.qpos.copy(), data.qvel.copy(), data.time
    reference = mujoco.MjData(model)
    mujoco.mj_copyData(reference, model, data)
    mujoco.mj_kinematics(model, reference)
    template = measurement().stream
    stream = replace(template, gnss=replace(template.gnss, fix_available=True, antenna_offset_m=(0.8, 0.2, 0.14)))
    engine = MuJoCoEngine()
    engine._model, engine._data = model, data
    actual = FormalGnss(stream, "a" * 32, 2, 3).sample(engine, data.time)
    expected = FormalGnss(stream, "a" * 32, 2, 3).sample(
        SimpleNamespace(get_body_pose=lambda name: (reference.body(name).xpos, reference.body(name).xquat)),
        reference.time,
    )
    assert actual.stamp.sim_time_ns == 5_000_000
    assert (actual.fix.lat, actual.fix.lon, actual.fix.alt) == pytest.approx(
        (expected.fix.lat, expected.fix.lon, expected.fix.alt), abs=1e-11,
    )
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)
    assert data.time == time_s
