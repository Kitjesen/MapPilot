from __future__ import annotations

import math

import pytest

from lingtu.assembly.config.native_parameters import resolve_parameters

PRODUCT_PARAMETERS = {
    "segment.max_distance_m": 4.0,
    "risk.stop_threshold": 60.0,
    "risk.resume_threshold": 45.0,
}


def test_scan_uses_shared_path_and_body_axis_limits() -> None:
    resolved = resolve_parameters()
    values = resolved.as_dict()
    environment = resolved.environment()
    for name, key, limit in (
        ("scan_planner.max_velocity_mps", "LINGTU_NAV_SCAN_PLANNER_MAX_VELOCITY_MPS", 2 ** 0.5),
        ("scan_follower.max_vx_mps", "LINGTU_NAV_SCAN_MAX_VX_MPS", 1.0),
        ("scan_follower.max_vy_mps", "LINGTU_NAV_SCAN_MAX_VY_MPS", 1.0),
    ):
        assert values[name] == limit
        assert float(environment[key]) == limit


@pytest.mark.parametrize("axis", ["vx", "vy"])
@pytest.mark.parametrize("limit", [0.005, 0.0, -0.005])
def test_scan_axis_limits_accept_positive_low_speeds(axis: str, limit: float) -> None:
    name = f"scan_follower.max_{axis}_mps"
    if limit <= 0.0:
        with pytest.raises(ValueError):
            resolve_parameters(session_overrides={name: limit})
    else:
        parameters = resolve_parameters(session_overrides={name: limit})
        assert parameters.as_dict()[name] == limit


def test_parameter_precedence_is_session_then_product_then_environment() -> None:
    resolved = resolve_parameters(
        product_parameters=PRODUCT_PARAMETERS,
        env_overrides={
            "segment.max_distance_m": 3.0,
            "map_input.max_age_s": 0.5,
        },
        session_overrides={"segment.max_distance_m": 2.0},
        map_publish_hz=10.0,
    )

    assert resolved.values["segment.max_distance_m"].value == 2.0
    assert resolved.values["risk.stop_threshold"].value == 60.0
    assert resolved.values["map_input.max_age_s"].value == 0.5
    assert resolved.values["map_input.max_cells"].value == 262_144


def test_product_parameters_have_higher_priority_than_env() -> None:
    resolved = resolve_parameters(
        product_parameters=PRODUCT_PARAMETERS,
        env_overrides={"risk.stop_threshold": 55.0},
    )
    assert resolved.values["risk.stop_threshold"].value == 60.0


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"segment.max_waypoints": 1}, "outside"),
        ({"map_input.max_cells": 1}, "outside"),
        ({"map_input.max_cells": 1_048_577}, "outside"),
        ({"risk.unknown": 1}, "unknown runtime parameters"),
        (
            {"risk.stop_threshold": 40.0, "risk.resume_threshold": 50.0},
            "resume_threshold",
        ),
    ],
)
def test_invalid_parameters_fail_before_launch(
    overrides: dict[str, float], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        resolve_parameters(
            session_overrides=overrides,
        )


def test_map_age_must_cover_two_publication_periods() -> None:
    with pytest.raises(ValueError, match="two configured map publication periods"):
        resolve_parameters(
            session_overrides={"map_input.max_age_s": 0.35},
            map_publish_hz=5.0,
        )


def test_environment_uses_native_contract_names() -> None:
    resolved = resolve_parameters(
        session_overrides={
            "segment.max_distance_m": 2.5,
            "segment.max_waypoints": 20,
        },
    )
    environment = resolved.environment()
    assert environment["LINGTU_NAV_SEGMENT_MAX_DISTANCE_M"] == "2.5"
    assert environment["LINGTU_NAV_SEGMENT_MAX_WAYPOINTS"] == "20"
    assert environment["LINGTU_NAV_SEGMENT_MAP_MAX_AGE_S"] == "0.35"


def test_scan_overrides_follow_the_existing_parameter_precedence() -> None:
    resolved = resolve_parameters(
        env_overrides={"scan_planner.max_velocity_mps": 0.5},
        product_parameters={"scan_planner.max_velocity_mps": 0.4},
        session_overrides={
            "scan_planner.max_velocity_mps": 0.3,
            "scan_follower.position_gain": 1.2,
            "local_collision.max_age_s": 0.75,
        },
    )
    assert resolved.as_dict()["scan_planner.max_velocity_mps"] == 0.3
    assert resolved.environment()["LINGTU_NAV_SCAN_PLANNER_MAX_VELOCITY_MPS"] == "0.3"
    assert resolved.environment()["LINGTU_NAV_SCAN_POSITION_GAIN"] == "1.2"
    assert resolved.environment()["LINGTU_NAV_LOCAL_COLLISION_MAX_AGE_S"] == "0.75"


def test_lateral_floor_reaches_native_environment_and_remains_overridable() -> None:
    assert resolve_parameters().as_dict()["scan_follower.min_vy_mps"] == 0.0
    resolved = resolve_parameters(
        product_parameters={"scan_follower.min_vy_mps": 0.35},
        session_overrides={"scan_follower.min_vy_mps": 0.25},
    )
    assert resolved.environment()["LINGTU_NAV_SCAN_MIN_VY_MPS"] == "0.25"


@pytest.mark.parametrize("overrides", [
    {"scan_planner.max_velocity_mps": True},
    {"scan_planner.max_velocity_mps": float("nan")},
    {"scan_planner.max_velocity_mps": 0.02},
    {"scan_planner.max_acceleration_mps2": 0.0},
    {"scan_planner.control_point_spacing_m": 0.01},
    {"scan_planner.smooth_weight": -1.0},
    {"scan_planner.replan_distance_m": 0.05},
    {"scan_follower.max_vx_mps": 0.0},
    {"scan_follower.min_vy_mps": -0.1},
    {"scan_follower.min_vy_mps": float("nan")},
    {"scan_follower.max_yaw_rate_rad_s": 1.1},
    {"local_collision.max_age_s": 0.09},
])
def test_invalid_scan_parameters_fail_before_launch(overrides) -> None:
    with pytest.raises(ValueError):
        resolve_parameters(session_overrides=overrides)


def test_removed_terminal_settings_are_not_accepted():
    with pytest.raises(ValueError):
        resolve_parameters(product_parameters={"goal_follower.min_yaw_rate_rad_s": 0.35})
    environment = resolve_parameters().environment()
    assert "LINGTU_NAV_GOAL_MIN_YAW_RATE_RAD_S" not in environment
    assert "LINGTU_NAV_GOAL_YAW_TOLERANCE_RAD" not in environment
