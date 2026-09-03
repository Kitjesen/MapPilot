from __future__ import annotations

import math

import pytest

from lingtu.assembly.native_nav import compile_native_nav_config
from lingtu.assembly.products import resolve_product_host_runtime


def _compiled_product_config(product_name: str) -> dict:
    resolved = resolve_product_host_runtime(product_name, "real", robot="unitree/go2")
    return {
        **resolved.config,
        "native_control_mode": resolved.product_spec["native_control_mode"],
        "native_nav": dict(resolved.product_spec.get("native_nav", {})),
    }


def test_native_nav_omits_legacy_final_stop_distances() -> None:
    compiled = compile_native_nav_config("teleop_avoid", _compiled_product_config("teleop_avoid"))

    assert "teleop_slow_distance_m" not in compiled.parameters
    assert "teleop_stop_distance_m" not in compiled.parameters
    assert "LINGTU_TELEOP_SLOW_DISTANCE_M" not in compiled.environment
    assert "LINGTU_TELEOP_STOP_DISTANCE_M" not in compiled.environment


def test_nav_product_compiles_native_endpoint_motion_parameters() -> None:
    compiled = compile_native_nav_config("nav", _compiled_product_config("nav"))
    payload = compiled.as_dict()

    assert payload["schema_version"] == "lingtu.native_nav_config.v1"
    assert payload["product"] == "nav"
    assert "fingerprint" not in payload
    assert payload["parameters"] == {
        "collision_clearance_above_m": 0.35,
        "collision_clearance_below_m": 0.25,
        "collision_cylinder_offset_m": 0.18,
        "collision_cylinder_radius_m": 0.25,
        "collision_hard_margin_m": 0.10,
        "corridor_lookahead_m": 3.0,
        "dynamic_confirm_frames": 4,
        "dynamic_min_cells": 8,
        "dynamic_min_speed_mps": 0.25,
        "goal_reached_m": 0.35,
        "path_follower_goal_tolerance_m": 0.2,
        "path_follower_lookahead_m": 0.35,
        "path_follower_max_accel_mps2": 0.3,
        "path_follower_max_speed_mps": 0.5,
        "path_follower_max_yaw_accel_rad_s2": 1.0,
        "path_follower_max_yaw_rate_rad_s": 0.5,
        "path_follower_min_speed_mps": 0.08,
        "path_follower_heading_align_enter_rad": math.pi / 4.0,
        "path_follower_heading_align_exit_rad": 0.35,
        "scan_finish_distance_m": 0.2,
        "scan_heading_error_rad": 0.8,
        "scan_max_vx_mps": 0.75,
        "scan_max_vy_mps": 0.35,
        "scan_max_yaw_rate_rad_s": 0.5,
        "scan_position_gain": 0.8,
        "scan_time_forward_s": 0.8,
        "scan_yaw_gain": 1.5,
        "teleop_max_speed_mps": 0.5,
        "teleop_max_yaw_rate_rad_s": 1.0,
        "teleop_planner_horizon_m": 3.5,
        "teleop_planner_max_deviation_deg": 55.0,
        "tick_hz": 100.0,
        "vehicle_length_m": 0.76,
        "vehicle_width_m": 0.31,
        "sensor_offset_x_m": 0.16143,
        "sensor_offset_y_m": 0.0,
        "sensor_offset_z_m": 0.12262,
        "waypoint_reached_m": 0.2,
    }
    expected_environment = {
        "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": "3",
        "LINGTU_NAV_GOAL_REACHED_M": "0.35",
        "LINGTU_NAV_CONTROL_MODE": "autonomy",
        "NAV_GLOBAL_PLANNER": "octoplanner3d",
        "LINGTU_NAV_LOCAL_PLANNER_BACKEND": "scan",
        "LINGTU_NAV_PUBLISH_CMD_VEL": "1",
        "LINGTU_NAV_CHECK_OBSTACLE": "1",
        "LINGTU_NAV_DYNAMIC_MIN_CELLS": "8",
        "LINGTU_NAV_DYNAMIC_MIN_SPEED_MPS": "0.25",
        "LINGTU_NAV_DYNAMIC_CONFIRM_FRAMES": "4",
        "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": "0.2",
        "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": "0.35",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": "0.3",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": "0.5",
        "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": "0.08",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_RATE_RAD_S": "0.5",
        "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_ENTER_RAD": "0.785398163397448",
        "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_EXIT_RAD": "0.35",
        "LINGTU_NAV_RECOVERY_ORDER": "translate,rotate",
        "LINGTU_NAV_RECOVERY_BLOCKED_INTERVAL_S": "2",
        "LINGTU_NAV_RECOVERY_ROTATION_TIMEOUT_S": "2.5",
        "LINGTU_NAV_RECOVERY_TRANSLATION_TIMEOUT_S": "1.5",
        "LINGTU_NAV_RECOVERY_MAX_ATTEMPTS": "0",
        "LINGTU_NAV_RECOVERY_TRANSLATION_SPEED_MPS": "0.15",
        "LINGTU_NAV_RECOVERY_ROTATION_RATE_RAD_S": "0.25",
        "LINGTU_NAV_RECOVERY_MIN_ROTATION_RAD": "0.2",
        "LINGTU_NAV_RECOVERY_MAX_ROTATION_RAD": "1.2",
        "LINGTU_NAV_RECOVERY_ROTATION_CANDIDATE_STEP_RAD": "0.2",
        "LINGTU_NAV_RECOVERY_ROTATION_SAMPLE_STEP_RAD": "0.05",
        "LINGTU_PRODUCT": "nav",
        "LINGTU_NAV_USE_TRAVERSABILITY_COST": "0",
        "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": "1",
        "LINGTU_TELEOP_PLANNER_HORIZON_M": "3.5",
        "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": "55",
        "LINGTU_TELEOP_LOCAL_PLANNER": "1",
        "LINGTU_TELEOP_OBSTACLE_MARGIN_M": "0.1",
        "LINGTU_NAV_VEHICLE_LENGTH_M": "0.76",
        "LINGTU_NAV_VEHICLE_WIDTH_M": "0.31",
        "LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M": "0.25",
        "LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M": "0.18",
        "LINGTU_NAV_SENSOR_OFFSET_X_M": "0.16143",
        "LINGTU_NAV_SENSOR_OFFSET_Y_M": "0",
        "LINGTU_NAV_SENSOR_OFFSET_Z_M": "0.12262",
        "LINGTU_NAV_WAYPOINT_REACHED_M": "0.2",
    }
    assert expected_environment.items() <= payload["environment"].items()
    assert payload["native_nav"]["global_planner"] == "octoplanner3d"
    assert payload["native_nav"]["local_planner"] == "scan"
    assert payload["native_nav"]["recovery"]["behavior_order"] == (
        "translate",
        "rotate",
    )
    assert payload["native_nav"]["recovery"]["max_attempts"] == 0


def test_go2_collision_hard_margin_reaches_native_endpoint() -> None:
    compiled = compile_native_nav_config(
        "teleop_avoid", _compiled_product_config("teleop_avoid")
    )

    assert compiled.parameters["collision_hard_margin_m"] == pytest.approx(0.10)
    assert compiled.environment["LINGTU_TELEOP_OBSTACLE_MARGIN_M"] == "0.1"


def test_dynamic_obstacle_thresholds_reach_native_endpoint() -> None:
    config = _compiled_product_config("teleop_avoid")
    config["native_nav"].update(
        {
            "dynamic_min_cells": 5,
            "dynamic_min_speed_mps": 0.18,
            "dynamic_confirm_frames": 3,
        }
    )

    compiled = compile_native_nav_config("teleop_avoid", config)

    assert compiled.environment["LINGTU_NAV_DYNAMIC_MIN_CELLS"] == "5"
    assert compiled.environment["LINGTU_NAV_DYNAMIC_MIN_SPEED_MPS"] == "0.18"
    assert compiled.environment["LINGTU_NAV_DYNAMIC_CONFIRM_FRAMES"] == "3"


def test_native_nav_config_accepts_scan_as_second_local_backend() -> None:
    compiled = compile_native_nav_config(
        "nav",
        {
            "native_control_mode": "autonomy",
            "native_nav": {"local_planner": "scan", "tick_hz": 100.0},
        },
    )

    assert compiled.native_nav["local_planner"] == "scan"
    assert compiled.environment["LINGTU_NAV_LOCAL_PLANNER_BACKEND"] == "scan"
    assert compiled.environment["LINGTU_NAV_DDS_TICK_HZ"] == "100"
    assert compiled.native_nav["use_traversability_cost"] is False

    with pytest.raises(ValueError, match="local_planner"):
        compile_native_nav_config(
            "nav",
            {
                "native_control_mode": "autonomy",
                "native_nav": {"local_planner": "unknown"},
            },
        )


def test_octoplanner_explicit_override_reaches_environment() -> None:
    compiled = compile_native_nav_config(
        "nav",
        {
            "native_control_mode": "autonomy",
            "octoplanner3d_robot_radius": 0.4,
        },
    )

    assert compiled.environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"] == "0.4"


@pytest.mark.parametrize(
    ("radius", "offset", "expected"),
    [
        (0.40, 0.25, 0.65),
        (0.25, 0.18, 0.43),
    ],
)
def test_octoplanner_default_radius_follows_robot_footprint(
    radius: float, offset: float, expected: float
) -> None:
    compiled = compile_native_nav_config(
        "nav",
        {
            "native_control_mode": "autonomy",
            "collision_cylinder_radius_m": radius,
            "collision_cylinder_offset_m": offset,
        },
    )

    assert float(compiled.environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"]) == pytest.approx(
        expected
    )


def test_velocity_smoother_is_left_to_the_native_endpoint() -> None:
    compiled = compile_native_nav_config("nav", {"native_control_mode": "autonomy"})

    assert "velocity_smoother" not in compiled.native_nav
    assert compiled.native_nav["smoothing"] is False
    assert compiled.environment["LINGTU_NAV_SMOOTHER_ENABLED"] == "0"


@pytest.mark.parametrize(
    "overrides",
    [
        {"vehicle_length_m": 0.0},
        {"vehicle_width_m": -0.1},
        {"vehicle_width_m": math.inf},
        {"sensor_offset_z_m": math.nan},
    ],
)
def test_native_nav_config_rejects_invalid_robot_safety_parameters(overrides) -> None:
    with pytest.raises(ValueError):
        compile_native_nav_config(
            "teleop_avoid",
            {**overrides, "native_control_mode": "teleop_avoid"},
        )


def test_native_nav_config_preserves_control_contract() -> None:
    base = {
        "native_control_mode": "teleop_avoid",
        "native_nav": {
            "publish_cmd_vel": True,
            "check_obstacle": True,
            "use_traversability_cost": True,
            "allow_teleop_takeover": False,
            "teleop_local_planner": True,
        },
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
            "native_control_mode": "teleop",
            "native_nav": {
                "publish_cmd_vel": True,
                "check_obstacle": False,
                "use_traversability_cost": False,
                "allow_teleop_takeover": False,
                "teleop_local_planner": False,
            },
        },
    )

    assert first.native_nav["control_mode"] == "teleop_avoid"
    assert second.native_nav["allow_teleop_takeover"] is True
    assert plain_teleop.native_nav["control_mode"] == "teleop"


@pytest.mark.parametrize(
    "overrides",
    [
        {
            "native_nav": {
                "path_follower_min_speed_mps": 0.3,
                "path_follower_max_speed_mps": 0.2,
            }
        },
        {"native_nav": {"path_follower_max_speed_mps": math.nan}},
        {
            "native_nav": {
                "path_follower_goal_tolerance_m": 0.2,
                "goal_reached_m": 0.1,
            }
        },
        {"native_nav": {"teleop_planner_horizon_m": 0.4}},
        {"native_nav": {"teleop_planner_max_deviation_deg": 91.0}},
        {
            "native_nav": {
                "path_follower_heading_align_enter_rad": 0.3,
                "path_follower_heading_align_exit_rad": 0.4,
            }
        },
        {"native_nav": {"recovery": {"behavior_order": ["rotate", "rotate"]}}},
        {
            "native_nav": {
                "recovery": {"min_rotation_rad": 1.0, "max_rotation_rad": 0.5}
            }
        },
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


def test_native_nav_config_rejects_legacy_host_parameter_names() -> None:
    with pytest.raises(ValueError, match="legacy navigation parameters"):
        compile_native_nav_config(
            "nav",
            {
                "native_control_mode": "autonomy",
                "path_follower_max_speed": 0.5,
            },
        )


def test_teleop_avoid_compiles_assisted_planner_parameters() -> None:
    payload = compile_native_nav_config(
        "teleop_avoid",
        _compiled_product_config("teleop_avoid"),
    ).as_dict()

    assert payload["schema_version"] == "lingtu.native_nav_config.v1"
    assert payload["parameters"]["teleop_planner_horizon_m"] == 3.5
    assert payload["parameters"]["teleop_planner_max_deviation_deg"] == 90.0
    assert payload["environment"]["LINGTU_TELEOP_PLANNER_HORIZON_M"] == "3.5"
    assert payload["environment"]["LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG"] == "90"
    assert payload["environment"]["LINGTU_NAV_CONTROL_MODE"] == "teleop_avoid"
    assert payload["environment"]["LINGTU_NAV_CHECK_OBSTACLE"] == "1"
    assert payload["environment"]["LINGTU_NAV_USE_TRAVERSABILITY_COST"] == "0"
    assert payload["environment"]["LINGTU_TELEOP_LOCAL_PLANNER"] == "1"
    assert payload["environment"]["LINGTU_NAV_RECOVERY_MAX_ATTEMPTS"] == "0"
