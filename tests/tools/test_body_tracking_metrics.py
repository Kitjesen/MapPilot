"""Prevent endpoint success or an infinite line from hiding route deviation."""
import numpy as np
import pytest
from tools.bench.body_tracking_metrics import path_distances


def test_endpoint_overshoot_counts_even_when_robot_is_on_supporting_line():
    route = np.array([[0., 0.], [1.2, 0.], [1.2, 0.]])
    actual = np.array([[.6, 0.], [2.2, 0.], [1.2, .3]])
    assert path_distances(actual, route) == pytest.approx([0., 1., .3])


def test_in_place_turn_still_measures_position_drift():
    route = np.array([[1., 2.], [1., 2.]])
    actual = np.array([[1., 2.], [1.03, 2.04]])
    assert path_distances(actual, route) == pytest.approx([0., .05])
