from __future__ import annotations

import json
import math

import pytest

from runtime.profiles.native_nav_config import native_nav_profile_config
from runtime.profiles.resolver import resolve_profile_config


def test_nav_profile_compiles_native_endpoint_motion_parameters() -> None:
    resolved = resolve_profile_config("nav", runtime_endpoint="thunder_field")

    compiled = native_nav_profile_config("nav", resolved)
    payload = compiled.as_dict()

    assert payload["schema_version"] == "lingtu.native_nav_profile.v2"
    assert payload["profile"] == "nav"
    assert payload["parameters"] == {
        "corridor_lookahead_m": 3.0,
        "goal_reached_m": 0.1,
        "path_follower_goal_tolerance_m": 0.05,
        "path_follower_lookahead_m": 0.35,
        "path_follower_max_accel_mps2": 1.0,
        "path_follower_max_speed_mps": 0.2,
        "path_follower_min_speed_mps": 0.08,
        "teleop_planner_horizon_m": 2.0,
        "teleop_planner_max_deviation_deg": 55.0,
        "waypoint_reached_m": 0.2,
    }
    assert payload["environment"] == {
        "LINGTU_NAV_CONFIG_FINGERPRINT": payload["fingerprint"],
        "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": "3",
        "LINGTU_NAV_GOAL_REACHED_M": "0.1",
        "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": "0.05",
        "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": "0.35",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": "1",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": "0.2",
        "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": "0.08",
        "LINGTU_NAV_PROFILE": "nav",
        "LINGTU_TELEOP_PLANNER_HORIZON_M": "2",
        "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": "55",
        "LINGTU_NAV_WAYPOINT_REACHED_M": "0.2",
    }
    assert len(payload["fingerprint"]) == 64
    int(payload["fingerprint"], 16)


def test_native_nav_profile_fingerprint_is_stable_and_parameter_sensitive() -> None:
    base = {
        "path_follower_max_speed": 0.2,
        "path_follower_min_speed": 0.08,
    }
    first = native_nav_profile_config("nav", base)
    second = native_nav_profile_config("nav", json.loads(json.dumps(base)))
    changed = native_nav_profile_config("nav", {**base, "path_follower_max_speed": 0.3})

    assert first.fingerprint == second.fingerprint
    assert first.fingerprint != changed.fingerprint


@pytest.mark.parametrize(
    "overrides",
    [
        {"path_follower_min_speed": 0.3, "path_follower_max_speed": 0.2},
        {"path_follower_max_speed": math.nan},
        {"path_follower_goal_tolerance": 0.2, "final_waypoint_threshold": 0.1},
        {"teleop_planner_horizon_m": 0.4},
        {"teleop_planner_max_deviation_deg": 91.0},
    ],
)
def test_native_nav_profile_rejects_incoherent_motion_parameters(overrides) -> None:
    with pytest.raises(ValueError):
        native_nav_profile_config("nav", overrides)


def test_teleop_avoid_compiles_assisted_planner_parameters() -> None:
    resolved = resolve_profile_config(
        "teleop_avoid",
        runtime_endpoint="thunder_field",
    )

    payload = native_nav_profile_config("teleop_avoid", resolved).as_dict()

    assert payload["schema_version"] == "lingtu.native_nav_profile.v2"
    assert payload["parameters"]["teleop_planner_horizon_m"] == 2.0
    assert payload["parameters"]["teleop_planner_max_deviation_deg"] == 55.0
    assert payload["environment"]["LINGTU_TELEOP_PLANNER_HORIZON_M"] == "2"
    assert payload["environment"]["LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG"] == "55"
