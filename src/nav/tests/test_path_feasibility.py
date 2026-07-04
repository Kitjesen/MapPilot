"""Tests for ground path feasibility reporting."""

from __future__ import annotations

import math

import pytest

from nav.services.plan.global_planner.path_feasibility import evaluate_ground_path


def test_flat_path_passes_ground_feasibility():
    path = [
        (0.0, 0.0, 0.20),
        (1.0, 0.0, 0.25),
        (2.0, 1.0, 0.30),
    ]

    report = evaluate_ground_path(path)

    assert report.ground_executable is True
    assert report.reasons == []
    assert report.point_count == 3
    assert report.path_length_xy == pytest.approx(1.0 + math.sqrt(2.0))
    assert report.path_length_3d > report.path_length_xy
    assert report.z_min == pytest.approx(0.20)
    assert report.z_max == pytest.approx(0.30)
    assert report.z_range == pytest.approx(0.10)
    assert report.max_segment_dz == pytest.approx(0.05)
    assert report.max_slope < report.thresholds["max_slope"]


def test_huge_z_range_fails_ground_feasibility():
    path = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 1.2),
        (20.0, 0.0, 3.1),
    ]

    report = evaluate_ground_path(path)

    assert report.ground_executable is False
    assert report.z_range == pytest.approx(3.1)
    assert "z_range_too_large" in report.reasons


def test_steep_segment_fails_ground_feasibility():
    path = [
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 1.0),
    ]

    report = evaluate_ground_path(path)

    assert report.ground_executable is False
    assert report.z_range == pytest.approx(1.0)
    assert report.max_segment_dz == pytest.approx(1.0)
    assert report.max_slope == pytest.approx(2.0)
    assert report.reasons == ["segment_slope_too_large"]


def test_empty_and_too_short_paths_fail_ground_feasibility():
    empty_report = evaluate_ground_path([])
    single_point_report = evaluate_ground_path([(0.0, 0.0, 0.0)])

    assert empty_report.ground_executable is False
    assert empty_report.point_count == 0
    assert empty_report.z_min is None
    assert empty_report.z_max is None
    assert empty_report.z_range is None
    assert empty_report.reasons == ["too_few_points"]

    assert single_point_report.ground_executable is False
    assert single_point_report.point_count == 1
    assert single_point_report.z_range == pytest.approx(0.0)
    assert single_point_report.reasons == ["too_few_points"]


def test_large_z_excursion_fails_ground_feasibility():
    # This is a standalone execution gate: a global 3D path can be valid only
    # if the robot has support and acceptable local geometry along the route.
    path = [
        (0.0, 0.0, 0.3),
        (10.0, 0.0, 13.9),
        (20.0, 0.0, 0.3),
    ]

    report = evaluate_ground_path(path)

    assert report.ground_executable is False
    assert report.point_count == 3
    assert report.z_min == pytest.approx(0.3)
    assert report.z_max == pytest.approx(13.9)
    assert report.z_range == pytest.approx(13.6)
    assert "z_range_too_large" in report.reasons
    assert report.to_dict()["ground_executable"] is False
