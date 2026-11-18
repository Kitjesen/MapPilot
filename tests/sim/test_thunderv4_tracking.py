"""Physical coverage for the CTS v39 supported lateral command envelope."""

import json
from pathlib import Path

import numpy as np
import pytest
from sim.scripts.mujoco.continuous_walk import VelocityCommand, _build_engine

POLICY_4998 = (Path(__file__).resolve().parents[2]
               / "sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_4998.onnx")


SUPPORTED_LATERAL_SPEED_MPS = 0.35


def _body_velocity(engine, state):
    rotation = engine._data.xmat[engine._base_body_id].reshape(3, 3)
    return rotation.T @ state.linear_velocity


def _stop_and_check(engine):
    for _ in range(round(2.0 / engine.control_dt)):
        state = engine.step(VelocityCommand())
    assert np.linalg.norm(state.linear_velocity[:2]) < 0.03


@pytest.mark.parametrize("sign", [-1, 1])
def test_low_speed_sideways_ramp_stop_and_restart(sign):
    engine = _build_engine(policy_path=POLICY_4998)
    try:
        for _ in range(round(1.0 / engine.control_dt)):
            engine.step(VelocityCommand())
        samples = []
        for tick in range(round(7.0 / engine.control_dt)):
            speed = min(SUPPORTED_LATERAL_SPEED_MPS, 1.2 * (tick + 1) * engine.control_dt)
            state = engine.step(VelocityCommand(linear_y=sign * speed))
            samples.append(_body_velocity(engine, state))
            assert .30 < state.position[2] < .65
        tail = np.asarray(samples[-round(3.0 / engine.control_dt):])
        mean = tail.mean(axis=0)
        assert sign * mean[1] > 0.15
        assert abs(mean[0]) < 0.05
        assert np.max(np.linalg.norm(tail[:, :2], axis=1)) < 0.50
        _stop_and_check(engine)
    finally:
        engine.close()


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("slew", [.25, .5, 2.0])
def test_sideways_reverse_and_turn_without_stopping(sign, slew, record_property):
    engine = _build_engine(policy_path=POLICY_4998)
    try:
        for _ in range(round(1.0 / engine.control_dt)):
            engine.step(VelocityCommand())
        reference = 0.0
        measurements = []
        for target in (sign * SUPPORTED_LATERAL_SPEED_MPS, -sign * SUPPORTED_LATERAL_SPEED_MPS):
            samples = []
            for _ in range(round(7.0 / engine.control_dt)):
                reference += float(np.clip(
                    target - reference,
                    -1.2 * engine.control_dt,
                    1.2 * engine.control_dt,
                ))
                state = engine.step(VelocityCommand(linear_y=reference))
                samples.append(_body_velocity(engine, state))
                assert .30 < state.position[2] < .65
            tail = np.asarray(samples[-round(3.0 / engine.control_dt):])
            mean = tail.mean(axis=0)
            measurements.append({"target": target, "actual_mean": mean.tolist()})
            assert np.sign(mean[1]) == np.sign(target)
            assert abs(mean[1]) > 0.15
            assert abs(mean[0]) < 0.05
            assert np.max(np.linalg.norm(tail[:, :2], axis=1)) < 0.50
        _stop_and_check(engine)
        record_property("phase_velocity_means", json.dumps(measurements))
    finally:
        engine.close()
