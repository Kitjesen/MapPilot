"""Physical coverage for the 4998 MuJoCo velocity feedback adapter."""

import json

import numpy as np
import pytest
from sim.scripts.mujoco.continuous_walk import VelocityCommand, _build_engine


@pytest.mark.parametrize("sign", [-1, 1])
def test_low_speed_sideways_ramp_stop_and_restart(sign):
    engine = _build_engine()
    try:
        for _ in range(round(1.0 / engine.control_dt)):
            engine.step(VelocityCommand())
        for direction in (sign, -sign):
            samples = []
            for tick in range(round(7.0 / engine.control_dt)):
                speed = min(.15, .5 * (tick + 1) * engine.control_dt)
                state = engine.step(VelocityCommand(linear_y=direction * speed))
                rotation = engine._data.xmat[engine._base_body_id].reshape(3, 3)
                samples.append(rotation.T @ state.linear_velocity)
                assert .30 < state.position[2] < .65
            tail = np.asarray(samples[-round(3.0 / engine.control_dt):])
            assert abs(tail[:, 1].mean() - direction * .15) < .03
            assert abs(tail[:, 0].mean()) < .02
            assert np.max(np.linalg.norm(tail[:, :2], axis=1)) < .25
            for _ in range(round(2.0 / engine.control_dt)):
                state = engine.step(VelocityCommand())
            assert np.linalg.norm(state.linear_velocity[:2]) < .03
            np.testing.assert_array_equal(engine._velocity_feedback.integral, 0)
    finally:
        engine.close()


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("slew", [.25, .5, 2.0])
def test_sideways_reverse_and_turn_without_stopping(sign, slew, record_property):
    engine = _build_engine()
    try:
        for _ in range(round(1.0 / engine.control_dt)):
            engine.step(VelocityCommand())
        reference = np.zeros(3)
        measurements = []
        for target in ((0, sign * .15, 0), (0, -sign * .15, 0),
                       (0, 0, sign * .3), (0, sign * .15, 0)):
            samples = []
            for _ in range(round(7.0 / engine.control_dt)):
                reference += np.clip(np.asarray(target) - reference,
                                     -slew * engine.control_dt, slew * engine.control_dt)
                state = engine.step(VelocityCommand(
                    linear_x=reference[0], linear_y=reference[1], angular_z=reference[2],
                ))
                rotation = engine._data.xmat[engine._base_body_id].reshape(3, 3)
                body_velocity = rotation.T @ state.linear_velocity
                samples.append((*body_velocity[:2], state.angular_velocity[2]))
                assert .30 < state.position[2] < .65
            tail = np.asarray(samples[-round(3.0 / engine.control_dt):])
            mean = tail.mean(axis=0)
            measurements.append({"target": target, "actual_mean": mean.tolist()})
            assert abs(mean[0]) < .02
            assert abs(mean[1] - target[1]) < .03
            assert abs(mean[2] - target[2]) < .08
            assert np.max(np.linalg.norm(tail[:, :2], axis=1)) < .25
        for _ in range(round(2.0 / engine.control_dt)):
            state = engine.step(VelocityCommand())
        assert np.linalg.norm(state.linear_velocity[:2]) < .03
        assert abs(state.angular_velocity[2]) < .03
        np.testing.assert_array_equal(engine._velocity_feedback.integral, 0)
        record_property("phase_velocity_means", json.dumps(measurements))
    finally:
        engine.close()
