"""Physical idle and restart regression for the deployed MuJoCo controller."""

import numpy as np

from sim.scripts.mujoco.continuous_walk import (
    MAX_RELEASE_DISPLACEMENT_M,
    MAX_RELEASE_FINAL_SPEED_MPS,
    VelocityCommand,
    _build_engine,
)


def test_long_idle_then_keyboard_motion_can_stop_and_restart() -> None:
    engine = _build_engine()
    try:
        def advance(command, seconds):
            start = engine.get_robot_state()
            maximum = 0.0
            for _ in range(round(seconds / engine.control_dt)):
                state = engine.step(command)
                maximum = max(maximum, float(np.linalg.norm(state.position[:2] - start.position[:2])))
                assert np.isfinite(state.position).all()
                assert 0.30 <= state.position[2] <= 0.65
            return start, state, maximum

        _, stopped, maximum = advance(VelocityCommand(), 60.0)
        assert maximum <= MAX_RELEASE_DISPLACEMENT_M
        assert np.linalg.norm(stopped.linear_velocity[:2]) <= MAX_RELEASE_FINAL_SPEED_MPS

        for _ in range(2):
            start, moving, _ = advance(VelocityCommand(linear_x=0.5), 3.0)
            assert moving.position[0] - start.position[0] >= 0.30
            _, stopped, maximum = advance(VelocityCommand(), 3.0)
            assert maximum <= MAX_RELEASE_DISPLACEMENT_M
            assert np.linalg.norm(stopped.linear_velocity[:2]) <= MAX_RELEASE_FINAL_SPEED_MPS
    finally:
        engine.close()
