"""Paper-equation and existing-frontier adapter tests, not simulator evidence."""

import math
from dataclasses import replace

import numpy as np
import pytest

from decision.frontiers.scorer import FrontierScorer
from decision.frontiers.vlnav import Candidate, Cue, observe_frontiers, rank_candidates, unknown_ratio, vl_score


def rank(candidates, robot=(0.0, 0.0, 0.4), **kwargs):
    return rank_candidates(candidates, robot, detection_threshold=0.5, reached_distance=0.25,
                           distance_weight=0.2, vl_weight=0.8, unknown_scale=2.0, **kwargs)


def test_gaussian_and_fov_weighting_match_paper():
    assert vl_score(0.0, [Cue(0.0, 0.8)], math.pi) == pytest.approx(0.8)
    assert vl_score(0.1, [Cue(0.0, 0.8)], math.pi) == pytest.approx(
        0.8 * math.exp(-0.5) * math.cos(0.1) ** 2)
    assert vl_score(0.0, [Cue(0.0, 0.8), Cue(0.0, 0.9)], math.pi) == 1.0


def test_outside_camera_cannot_receive_periodic_cosine_evidence():
    assert vl_score(math.pi, [Cue(math.pi, 0.9)], math.pi / 2) == 0.0
    assert vl_score(math.pi / 4, [Cue(math.pi / 4, 0.9)], math.pi / 2) == 0.0
    assert vl_score(0.0, [], math.pi / 2) == 0.0
    assert vl_score(2 * math.pi, [Cue(0.0, 0.8)], math.pi) == pytest.approx(0.8)


def test_unknown_count_does_not_cross_wall():
    grid = np.array([[0, 100, -1], [0, 100, -1], [0, 100, -1]])
    assert unknown_ratio(grid, (1, 0), 6) == 0
    grid[1, 1] = 0
    assert unknown_ratio(grid, (1, 0), 6) == pytest.approx(3 / 7)
    assert unknown_ratio(grid, (1, 0), 1) == 0
    assert unknown_ratio(grid, (1, 2), 0) == 1


def test_instance_priority_and_score_omit_curiosity():
    frontier = Candidate("f", "frontier", (1, 0, 0.4), 1, unknown=1)
    far = Candidate("bottle", "instance", (8, 0, 1.2), 0.4, confidence=0.9)
    result = rank([frontier, far])
    assert result[0].candidate.id == "bottle"
    assert result[0].score == 0.4
    assert result[0].distance_score == result[0].unknown_score == 0
    assert result[0].candidate.position[2] == 1.2


def test_instances_rank_by_vl_not_distance_or_raw_detection_confidence():
    close = Candidate("close", "instance", (1, 0, 0.4), 0.3, confidence=0.99)
    far = Candidate("far", "instance", (10, 0, 0.4), 0.8, confidence=0.6)
    assert rank([close, far])[0].candidate.id == "far"


def test_threshold_reached_and_rejected_instances_fall_back():
    frontier = Candidate("f", "frontier", (2, 0, 0.4), 0.4, unknown=0.5)
    weak = Candidate("weak", "instance", (1, 0, 0.4), 0.9, confidence=0.5)
    reached = Candidate("near", "instance", (0.25, 0, 0.4), 0.9, confidence=0.99)
    wrong = Candidate("wrong", "instance", (4, 0, 0.4), 0.9, confidence=0.99)
    assert rank([weak, reached, wrong, frontier], excluded_ids=frozenset({"wrong"}))[0].candidate.id == "f"
    assert rank([weak, reached]) == []


def test_frontier_equation_and_selection_time_distance():
    candidate = Candidate("f", "frontier", (3, 4, 0.4), 0.6, unknown=0.5)
    first = rank([candidate])[0]
    assert first.score == pytest.approx(0.2 / 6 + 0.8 * 0.6 * (1 - math.exp(-1)))
    moved = rank([candidate], robot=(3, 3, 0.4))[0]
    assert moved.score > first.score
    assert moved.candidate.vl == first.candidate.vl
    assert rank([replace(candidate, unknown=0)])[0].score == pytest.approx(0.2 / 6)


def test_semantic_cue_can_outweigh_nearest_frontier():
    near = Candidate("near", "frontier", (1, 0, 0.4), 0.0, unknown=0.8)
    relevant = Candidate("relevant", "frontier", (3, 0, 0.4), 0.8, unknown=0.8)
    assert rank([near, relevant])[0].candidate.id == "relevant"
    assert rank([near, replace(relevant, vl=0)])[0].candidate.id == "near"


def test_real_frontier_extraction_adapts_to_vlnav_without_mutating_scorer():
    grid = np.full((20, 20), -1, dtype=np.int8)
    grid[:10, :] = 0
    scorer = FrontierScorer(min_frontier_size=3, tsp_reorder=False)
    scorer.update_costmap(grid, 0.1, 0, 0)
    frontiers = scorer.extract_frontiers(np.array([0.9, 0.3]))
    result = observe_frontiers(frontiers, grid, (0.9, 0.3, 1.4), math.pi / 2, [Cue(0, 0.8)],
                               hfov=math.pi, sensor_range=3, radius_cells=3, reference_z=1.4)
    assert result
    assert rank(result, robot=(0.9, 0.3, 1.4))
    assert all(c.position[2] == 1.4 for c in result)
    assert all(f.score == 0 for f in frontiers)
    assert not observe_frontiers(frontiers, grid, (0.9, 0.3, 1.4), -math.pi / 2, [],
                                  hfov=math.pi / 2, sensor_range=3, radius_cells=3, reference_z=1.4)


def test_invalid_detection_cannot_win_selection():
    nan_goal = Candidate("nan", "instance", (float("nan"), 0, 0), 0.9, confidence=0.99)
    invalid_score = Candidate("bad", "frontier", (2, 0, 0), float("nan"), unknown=1)
    assert rank([nan_goal, invalid_score]) == []
    with pytest.raises(ValueError, match="robot_position"):
        rank([], robot=(float("nan"), 0, 0))
