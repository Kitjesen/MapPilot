from __future__ import annotations

import json
import math

import pytest

from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.profile_builder import compile_run_plan
from runtime.profiles.native_nav_config import compile_native_nav_config


def _compiled_product_config(product_name: str) -> dict:
    resolved = resolve_product_host_runtime(product_name, "real")
    manifest = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
    )
    return {
        **manifest.host_config,
        "native_control_mode": manifest.native_nav["control_mode"],
        "native_nav": dict(manifest.native_nav),
    }


def test_nav_product_compiles_native_endpoint_motion_parameters() -> None:
    compiled = compile_native_nav_config("nav", _compiled_product_config("nav"))
    payload = compiled.as_dict()

    assert payload["schema_version"] == "lingtu.native_nav_config.v1"
    assert payload["product"] == "nav"
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
        "LINGTU_NAV_CONTROL_MODE": "autonomy",
        "LINGTU_NAV_PUBLISH_CMD_VEL": "1",
        "LINGTU_NAV_CHECK_OBSTACLE": "1",
        "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": "0.05",
        "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": "0.35",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": "1",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": "0.2",
        "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": "0.08",
        "LINGTU_PRODUCT": "nav",
        "LINGTU_NAV_USE_TRAVERSABILITY_COST": "1",
        "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": "1",
        "LINGTU_TELEOP_PLANNER_HORIZON_M": "2",
        "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": "55",
        "LINGTU_TELEOP_LOCAL_PLANNER": "1",
        "LINGTU_NAV_WAYPOINT_REACHED_M": "0.2",
    }
    assert len(payload["fingerprint"]) == 64
    int(payload["fingerprint"], 16)


def test_native_nav_config_fingerprint_is_stable_and_parameter_sensitive() -> None:
    base = {
        "native_control_mode": "autonomy",
        "path_follower_max_speed": 0.2,
        "path_follower_min_speed": 0.08,
    }
    first = compile_native_nav_config("nav", base)
    second = compile_native_nav_config("nav", json.loads(json.dumps(base)))
    changed = compile_native_nav_config("nav", {**base, "path_follower_max_speed": 0.3})

    assert first.fingerprint == second.fingerprint
    assert first.fingerprint != changed.fingerprint


def test_native_nav_config_fingerprint_is_control_contract_sensitive() -> None:
    base = {
        "native_nav": {
            "control_mode": "teleop_avoid",
            "publish_cmd_vel": True,
            "check_obstacle": True,
            "use_traversability_cost": True,
            "allow_teleop_takeover": False,
            "teleop_local_planner": True,
        }
    }
    changed = {
        **base,
        "native_nav": {
            **base["native_nav"],
            "teleop_local_planner": False,
        },
    }
    takeover_changed = {
        **base,
        "native_nav": {
            **base["native_nav"],
            "allow_teleop_takeover": True,
        },
    }

    first = compile_native_nav_config("teleop_avoid", base)
    second = compile_native_nav_config("teleop_avoid", takeover_changed)
    with pytest.raises(ValueError):
        compile_native_nav_config("teleop_avoid", changed)
    plain_teleop = compile_native_nav_config(
        "teleop",
        {
            "native_nav": {
                "control_mode": "teleop",
                "publish_cmd_vel": True,
                "check_obstacle": False,
                "use_traversability_cost": False,
                "allow_teleop_takeover": False,
                "teleop_local_planner": False,
            }
        },
    )

    assert first.fingerprint != plain_teleop.fingerprint
    assert first.fingerprint != second.fingerprint


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
def test_native_nav_config_rejects_incoherent_motion_parameters(overrides) -> None:
    with pytest.raises(ValueError):
        compile_native_nav_config(
            "nav",
            {**overrides, "native_control_mode": "autonomy"},
        )


def test_native_nav_config_rejects_missing_product_control_mode() -> None:
    with pytest.raises(ValueError, match="compiled Product"):
        compile_native_nav_config("nav", {})


def test_teleop_avoid_compiles_assisted_planner_parameters() -> None:
    payload = compile_native_nav_config(
        "teleop_avoid",
        _compiled_product_config("teleop_avoid"),
    ).as_dict()

    assert payload["schema_version"] == "lingtu.native_nav_config.v1"
    assert payload["parameters"]["teleop_planner_horizon_m"] == 2.0
    assert payload["parameters"]["teleop_planner_max_deviation_deg"] == 55.0
    assert payload["environment"]["LINGTU_TELEOP_PLANNER_HORIZON_M"] == "2"
    assert payload["environment"]["LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG"] == "55"
    assert payload["environment"]["LINGTU_NAV_CONTROL_MODE"] == "teleop_avoid"
    assert payload["environment"]["LINGTU_NAV_CHECK_OBSTACLE"] == "1"
    assert payload["environment"]["LINGTU_NAV_USE_TRAVERSABILITY_COST"] == "1"
    assert payload["environment"]["LINGTU_TELEOP_LOCAL_PLANNER"] == "1"
