import math

import numpy as np
from tools.bench.pct_cpu import Config, inflate, point_indices, select_layer, simplify, surfaces, traversability


def test_surfaces_keep_separate_floors_and_overhangs():
    points = np.array([[0, 0, 0], [0, 0, .4], [0, 0, 1.0], [.1, 0, .17]], dtype=np.float32)
    ground, ceiling = surfaces(points, [0, 0], (5, 5), [.2, .6], .1)
    np.testing.assert_allclose(ground[:, 2, 2], [0, .4])
    np.testing.assert_allclose(ceiling[:, 2, 2], [.4, 1.0])
    assert ground[0, 3, 2] == np.float32(.17)
    assert ground[0, 0, 0] == -1e6
    assert ceiling[0, 0, 0] == 1e6


def test_index_rounding_matches_c_round_at_exact_half_cells():
    points = np.array([[.125, 0, 0], [-.125, 0, 0]], dtype=np.float32)
    np.testing.assert_array_equal(point_indices(points, [0, 0], .25, (5, 5)), [[3, 2], [1, 2]])


def test_traversability_matches_scalar_kernel_rules():
    cfg = Config(kernel_size=3)
    g = np.zeros((1, 7, 7), dtype=np.float32)
    g[0, 2:5, 2:5] = .1
    g[0, 4, 4] = .4
    g[0, 5, 5] = -1e6
    ceiling = np.full_like(g, 1.0)
    ceiling[0, 1, 1] = .3
    sq = np.zeros_like(g)
    mx = np.zeros_like(g)
    for x in range(1, 6):
        for y in range(1, 6):
            dx = max(float(g[0,x,y]-g[0,x-1,y])**2, float(g[0,x,y]-g[0,x+1,y])**2)
            dy = max(float(g[0,x,y]-g[0,x,y-1])**2, float(g[0,x,y]-g[0,x,y+1])**2)
            sq[0,x,y], mx[0,x,y] = dx+dy, max(dx,dy)
    expected = np.zeros_like(g)
    stand = (1.2*cfg.resolution*math.tan(cfg.slope_max))**2
    for x in range(7):
        for y in range(7):
            gap = ceiling[0,x,y]-g[0,x,y]
            cost = max(0, 20*(cfg.interval_free-gap))
            count = sum(sq[0,a,b] < stand for a in range(max(0,x-1),min(7,x+2))
                        for b in range(max(0,y-1),min(7,y+2)))
            if gap < cfg.interval_min:
                cost = cfg.cost_barrier
            elif sq[0,x,y] <= stand:
                cost += 15*sq[0,x,y]/stand
            elif mx[0,x,y] <= cfg.step_max**2 and count >= int(cfg.standable_ratio*9)-1:
                cost += 20*mx[0,x,y]/cfg.step_max**2
            else:
                cost = cfg.cost_barrier
            expected[0,x,y] = cost
    np.testing.assert_allclose(traversability(g, ceiling, cfg), expected, rtol=1e-5, atol=1e-5)


def test_inflation_preserves_layers_and_clips_boundaries():
    cfg = Config(safe_margin=.1, inflation=.1)
    costs = np.zeros((2, 5, 5), dtype=np.float32)
    costs[0, 0, 0] = 50
    result = inflate(costs, cfg)
    assert result[0, 0, 0] == 50
    assert result[0, 1, 0] == 50
    assert result[0, 2, 0] == 25
    assert result[0, 4, 4] == 0
    assert not result[1].any()


def test_layer_simplification_retains_distinct_surface():
    ground = np.zeros((5, 3, 3), dtype=np.float32)
    ground[1] = .1
    ground[2:] = .2
    cost = np.zeros_like(ground)
    assert simplify(ground, cost, 50) == [0, 1, 3]


def test_endpoint_layer_contains_body_not_just_nearest_floor():
    ground = np.array([-.5, -.45, 2.0])
    ceiling = np.array([-.4, 1.8, 3.5])
    cost = np.array([0, 30, 0])
    assert select_layer(ground, ceiling, cost, -.15) == 1
    assert select_layer(ground, ceiling, cost, 2.35) == 2
    assert select_layer(ground, ceiling, cost, 1.9) is None
