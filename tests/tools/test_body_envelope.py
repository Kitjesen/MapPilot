"""Check the behavior that endpoint-only shape comparisons would miss."""
import math

import pytest

pytest.importorskip("mujoco")

from tools.bench.body_envelope import URDF, bounds, proxy, sweep, walls
from tools.bench.body_model import BodyModel


def test_free_end_headings_do_not_authorize_a_colliding_turn():
    body = BodyModel(URDF, walls(.60))
    start, end = [0, 0, .35, 0], [0, 0, .35, math.pi]
    assert body.clearance(start)[0] > .02
    assert body.clearance(end)[0] > .02
    assert not sweep(body, start, end, .02, .40)[0]


def test_aligned_full_model_translation_fits_where_all_heading_circle_does_not(tmp_path):
    file = tmp_path / 'circle.urdf'
    proxy(file, .44, 0, .25)
    obstacles = walls(.60)
    full = BodyModel(URDF, obstacles)
    circle = BodyModel(file, obstacles)
    start, end = [-.8, 0, .35, 0], [.8, 0, .35, 0]
    assert sweep(full, start, end, .02, .40)[0]
    assert not sweep(circle, start, end, 0, .44)[0]
    low, high = bounds(full.geometry([0, 0, 0, 0]))
    assert .32 < high[1] - low[1] < .34


def test_all_heading_distances_match_known_parallel_wall_gaps(tmp_path):
    circle, twin = tmp_path/'circle.urdf', tmp_path/'twin.urdf'
    proxy(circle, .44, 0, .25)
    proxy(twin, .25, .19, .24)
    for path in (circle, twin, URDF):
        model = BodyModel(path, walls(.60))
        for degrees in range(0, 181, 5):
            pose = [0, 0, .35, math.radians(degrees)]
            low, high = bounds(model.geometry(pose))
            expected = .30 - max(high[1], -low[1])
            distance = model.clearance(pose)[0]
            if expected > 1e-5:
                assert distance == pytest.approx(expected, abs=1e-5), (path, degrees)
            elif expected < -1e-5:
                assert distance < 0, (path, degrees, expected, distance)
