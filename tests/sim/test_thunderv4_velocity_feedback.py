"""Exercise feedback state across command changes and output saturation."""

import numpy as np
import pytest
from sim.runtime.control.thunderv4_flat import VelocityFeedback


@pytest.mark.parametrize("next_reference", [(0, 0, .25), (0, -.15, 0)])
def test_command_transition_discards_previous_sideways_integral(next_reference):
    feedback = VelocityFeedback((1, 1, 1))
    for _ in range(100):
        feedback.update(np.array([0, .15, 0]), np.array([0, .14, 0]), .02)
    assert feedback.integral[1] > .01
    reference = np.array(next_reference)
    command = feedback.update(reference, reference.copy(), .02)
    assert command[1] == pytest.approx(reference[1])
    assert feedback.integral[1] == 0


def test_saturated_correction_does_not_accumulate_more_integral():
    feedback = VelocityFeedback((1, 1, 1))
    reference = np.array([0, .15, 0])
    for _ in range(100):
        feedback.update(reference, np.array([0, -.5, 0]), .02)
    assert feedback.integral[1] == 0
    command = feedback.update(reference, reference.copy(), .02)
    assert command[1] == pytest.approx(.15)


def test_robot_command_limit_also_prevents_windup():
    feedback = VelocityFeedback((.3, .3, .5))
    reference = np.array([0, .25, 0])
    for _ in range(100):
        command = feedback.update(reference, np.array([0, .15, 0]), .02)
        assert command[1] == pytest.approx(.3)
    assert feedback.integral[1] == 0
    command = feedback.update(reference, reference.copy(), .02)
    assert command[1] == pytest.approx(.25)


def test_opposing_error_unwinds_bias_even_while_output_is_saturated():
    feedback = VelocityFeedback((.3, .3, .5))
    reference = np.array([0, .25, 0])
    for _ in range(100):
        feedback.update(reference, np.array([0, .24, 0]), .02)
    previous = feedback.integral[1]
    assert previous > .01
    command = feedback.update(np.array([0, .295, 0]), np.array([0, .3, 0]), .02)
    assert command[1] == pytest.approx(.3)
    assert 0 < feedback.integral[1] < previous


def test_axis_transition_preserves_compensation_for_unchanged_drift_axis():
    feedback = VelocityFeedback((1, 1, 1))
    for _ in range(100):
        feedback.update(np.array([0, .15, 0]), np.array([-.01, .14, 0]), .02)
    previous = feedback.integral[0]
    assert previous > .01
    reference = np.array([0, 0, .25])
    command = feedback.update(reference, reference.copy(), .02)
    assert command[0] == pytest.approx(previous)
    assert command[1] == 0


@pytest.mark.parametrize("stop", ["zero_command", "reset"])
def test_stop_clears_bias_before_same_direction_restart(stop):
    feedback = VelocityFeedback((1, 1, 1))
    reference = np.array([0, .15, 0])
    for _ in range(100):
        feedback.update(reference, np.array([0, .14, 0]), .02)
    if stop == "zero_command":
        command = feedback.update(np.zeros(3), reference, .02)
        np.testing.assert_array_equal(command, 0)
    else:
        feedback.reset()
    command = feedback.update(reference, reference.copy(), .02)
    np.testing.assert_array_equal(command, reference)
