from __future__ import annotations

import math
from pathlib import Path
from shutil import copytree

import pytest
import yaml

from lingtu.assembly.compile_plan import compile_run_plan
from lingtu.assembly.config.maps import default_map_settings, mapd_environment
from lingtu.assembly.config.native_navigation import compile_native_nav_config
from lingtu.assembly.config.product import resolve_product_host_runtime
from lingtu.assembly.graph.loader import load_runtime_graph


def _compiled_product_config(product_name: str, robot: str = "unitree/go2") -> dict:
    resolved = resolve_product_host_runtime(product_name, "real", robot=robot)
    return {
        **resolved.config,
        "native_control_mode": resolved.product_spec["native_control_mode"],
        "native_nav": dict(resolved.product_spec.get("native_nav", {})),
    }


def test_component_settings_reach_exploration_launch(tmp_path):
    system = tmp_path / "system"
    copytree(Path(__file__).resolve().parents[3] / "config/system", system)
    settings = system / "settings"
    for name, section, key, value in (
        ("explore", ("explore",), "tick_hz", 3.0),
        ("explore", ("explore",), "directed_target_max_ttl_s", 900.0),
        ("explore", ("standalone", "parameters"), "map_input.future_tolerance_s", 0.7),
        ("control", ("native_nav",), "stop_confirmation_timeout_s", 3.0),
        ("control", ("native_nav",), "smoother_feedback_timeout_s", 0.4),
        ("control", ("native_nav",), "input_future_tolerance_s", 0.07),
        ("local", ("native_nav",), "obstacle_voxel_size_m", 0.06),
        ("local", ("native_nav",), "local_planner_obstacle_height_min_m", 0.10),
        ("local", ("native_nav",), "local_planner_obstacle_height_max_m", 0.8),
        ("local", ("native_nav",), "scan_motion_voxel_size_m", 0.09),
        ("maps", ("maps", "occupancy"), "resolution_m", 0.10),
        ("local", ("native_nav", "cmu"), "path_scale", 0.9),
        ("follower", ("native_nav", "cmu"), "follower_lookahead_m", 0.7),
        ("teleop", ("native_nav", "teleop"), "cmd_max_age_s", 0.5),
    ):
        path = settings / f"{name}.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        group = data
        for part in section:
            group = group[part]
        group[key] = value
        path.write_text(yaml.safe_dump(data), encoding="utf-8")

    graph = load_runtime_graph(system)
    plan = compile_run_plan("explore", "real", robot="unitree/go2", graph=graph)
    environment = plan.native_process_environment
    assert float(environment["LINGTU_EXPLORE_TICK_HZ"]) == 3.0
    assert float(environment["LINGTU_EXPLORE_DIRECTED_TARGET_MAX_TTL_S"]) == 900.0
    assert float(environment["LINGTU_NAV_SEGMENT_FUTURE_TOLERANCE_S"]) == 0.7
    assert float(environment["LINGTU_NAV_STOP_CONFIRMATION_TIMEOUT_S"]) == 3.0
    assert float(environment["LINGTU_NAV_SMOOTHER_FEEDBACK_TIMEOUT_S"]) == 0.4
    assert float(environment["LINGTU_TELEOP_CMD_MAX_AGE_S"]) == 0.5
    assert float(environment["LINGTU_NAV_INPUT_FUTURE_TOLERANCE_S"]) == 0.07
    assert float(environment["LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M"]) == 0.06
    assert environment["LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MIN_M"] == "0.1"
    assert environment["LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MAX_M"] == "0.8"
    assert "LINGTU_TELEOP_OBSTACLE_HEIGHT_MIN_M" not in environment
    assert "LINGTU_TELEOP_OBSTACLE_HEIGHT_MAX_M" not in environment
    assert float(environment["LINGTU_NAV_CMU_PATH_SCALE"]) == 0.9
    assert float(environment["LINGTU_NAV_CMU_FOLLOWER_LOOKAHEAD_M"]) == 0.7
    assert "LINGTU_NAV_LIVE_OBSTACLE_INFLATION_RADIUS_M" not in environment
    scan = compile_run_plan("nav", "real", robot="unitree/go2", graph=graph)
    assert scan.native_process_environment["LINGTU_NAV_SCAN_MOTION_VOXEL_SIZE_M"] == "0.09"
    assert scan.native_process_environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.1"
    assert scan.native_process_environment["LINGTU_NAV_SCAN_VOXEL_RESOLUTION_M"] == "0.1"
    assert "LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M" not in scan.native_process_environment

    for invalid_min, message in ((-0.1, "must be non-negative"), (0.9, "maximum must be at least")):
        with pytest.raises(ValueError, match=message):
            compile_native_nav_config(
                "test",
                {
                    "native_control_mode": "autonomy",
                    "native_nav": {
                        "local_planner_obstacle_height_min_m": invalid_min,
                        "local_planner_obstacle_height_max_m": 0.8,
                    },
                },
                settings_directory=settings,
            )


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
    expected_parameters = {
        "braking_deceleration_mps2": 4.0,
        "collision_clearance_above_m": 0.24,
        "collision_clearance_below_m": 0.10,
        "collision_cylinder_offset_m": 0.19,
        "collision_cylinder_radius_m": 0.25,
        "collision_hard_margin_m": 0.05,
        "corridor_lookahead_m": 3.5,
        "dynamic_confirm_frames": 4,
        "dynamic_min_cells": 8,
        "dynamic_min_speed_mps": 0.25,
        "goal_height_tolerance_m": 0.35,
        "goal_reached_m": 0.15,
        "path_follower_goal_tolerance_m": 0.15,
        "path_follower_lookahead_m": 0.35,
        "path_follower_max_accel_mps2": 0.75,
        "path_follower_max_speed_mps": math.sqrt(2.0),
        "path_follower_max_yaw_accel_rad_s2": 4.0,
        "path_follower_max_yaw_rate_rad_s": 1.0,
        "path_follower_min_speed_mps": 0.08,
        "path_follower_heading_align_enter_rad": math.pi / 4.0,
        "path_follower_heading_align_exit_rad": 0.35,
        "teleop_max_speed_mps": 1.0,
        "teleop_max_yaw_rate_rad_s": 1.0,
        "teleop_planner_horizon_m": 3.5,
        "teleop_planner_max_deviation_deg": 55.0,
        "tick_hz": 100.0,
        "vehicle_length_m": 0.76,
        "vehicle_width_m": 0.31,
        "sensor_offset_x_m": 0.24,
        "sensor_offset_y_m": 0.0,
        "sensor_offset_z_m": 0.20262,
        "waypoint_reached_m": 0.2,
    }
    assert {key: payload["parameters"][key] for key in expected_parameters} == expected_parameters
    expected_environment = {
        "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": "3.5",
        "LINGTU_NAV_GOAL_REACHED_M": "0.15",
        "LINGTU_NAV_GOAL_HEIGHT_TOLERANCE_M": "0.35",
        "LINGTU_NAV_CONTROL_MODE": "autonomy",
        "NAV_GLOBAL_PLANNER": "octoplanner3d",
        "LINGTU_NAV_LOCAL_PLANNER_BACKEND": "scan",
        "LINGTU_NAV_PUBLISH_CMD_VEL": "1",
        "LINGTU_NAV_CHECK_OBSTACLE": "1",
        "LINGTU_NAV_DYNAMIC_MIN_CELLS": "8",
        "LINGTU_NAV_DYNAMIC_MIN_SPEED_MPS": "0.25",
        "LINGTU_NAV_DYNAMIC_CONFIRM_FRAMES": "4",
        "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": "0.15",
        "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": "0.35",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": "0.75",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": "1.4142135623731",
        "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": "0.08",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_RATE_RAD_S": "1",
        "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_ENTER_RAD": "0.785398163397448",
        "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_EXIT_RAD": "0.35",
        "LINGTU_NAV_RECOVERY_ORDER": "translate,rotate",
        "LINGTU_NAV_RECOVERY_BLOCKED_INTERVAL_S": "2",
        "LINGTU_NAV_RECOVERY_ROTATION_TIMEOUT_S": "2.5",
        "LINGTU_NAV_RECOVERY_TRANSLATION_TIMEOUT_S": "1.5",
        "LINGTU_NAV_RECOVERY_MAX_ATTEMPTS": "3",
        "LINGTU_NAV_RECOVERY_TRANSLATION_SPEED_MPS": "0.5",
        "LINGTU_NAV_RECOVERY_ROTATION_RATE_RAD_S": "0.8",
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
        "LINGTU_TELEOP_OBSTACLE_MARGIN_M": "0.05",
        "LINGTU_NAV_VEHICLE_LENGTH_M": "0.76",
        "LINGTU_NAV_VEHICLE_WIDTH_M": "0.31",
        "LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M": "0.25",
        "LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M": "0.19",
        "LINGTU_NAV_SENSOR_OFFSET_X_M": "0.24",
        "LINGTU_NAV_SENSOR_OFFSET_Y_M": "0",
        "LINGTU_NAV_SENSOR_OFFSET_Z_M": "0.20262",
        "LINGTU_NAV_WAYPOINT_REACHED_M": "0.2",
    }
    assert expected_environment.items() <= payload["environment"].items()
    assert payload["native_nav"]["global_planner"] == "octoplanner3d"
    assert payload["native_nav"]["local_planner"] == "scan"
    assert payload["native_nav"]["recovery"]["behavior_order"] == (
        "translate",
        "rotate",
    )
    assert payload["native_nav"]["recovery"]["max_attempts"] == 3


def test_go2_double_cylinders_cover_declared_outline() -> None:
    parameters = compile_native_nav_config("nav", _compiled_product_config("nav")).parameters
    half_length = 0.5 * parameters["vehicle_length_m"]
    half_width = 0.5 * parameters["vehicle_width_m"]
    offset = parameters["collision_cylinder_offset_m"]
    required_radius = math.hypot(half_width, max(offset, half_length - offset))
    assert parameters["collision_cylinder_radius_m"] >= required_radius


def test_go2_collision_hard_margin_reaches_native_endpoint() -> None:
    compiled = compile_native_nav_config(
        "teleop_avoid", _compiled_product_config("teleop_avoid")
    )

    assert compiled.parameters["collision_hard_margin_m"] == pytest.approx(0.05)
    assert compiled.environment["LINGTU_TELEOP_OBSTACLE_MARGIN_M"] == "0.05"


def test_go2_standing_height_reaches_native_planner() -> None:
    compiled = compile_native_nav_config("nav", _compiled_product_config("nav"))

    assert compiled.environment["LINGTU_NAV_OCTO_SEARCH_RESOLUTION_M"] == "0.1"
    assert compiled.environment["LINGTU_NAV_OCTO_SURFACE_CLOSING_RADIUS_M"] == "0.2"
    assert compiled.environment["LINGTU_NAV_OCTO_SUPPORT_HEIGHT_M"] == "0.35"
    assert compiled.environment["LINGTU_NAV_OCTO_SUPPORT_HEIGHT_TOLERANCE_M"] == "0.05"
    assert compiled.environment["LINGTU_NAV_OCTO_DIRECT_SUPPORT_ONLY"] == "0"
    assert float(compiled.environment["LINGTU_NAV_OCTO_SURFACE_MAX_HOLE_AREA_M2"]) == .16
    assert float(compiled.environment["LINGTU_NAV_OCTO_SURFACE_MAX_SLOPE"]) == .7
    assert compiled.environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"] == "0.25"
    assert compiled.environment["LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M"] == "0.25"
    assert compiled.environment["LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M"] == "0.19"
    assert compiled.environment["LINGTU_NAV_OCTO_TERMINAL_GOAL_TOLERANCE_M"] == "0.15"
    assert compiled.environment["LINGTU_NAV_OCTO_TERMINAL_GOAL_XY_TOLERANCE_M"] == "0.15"
    assert compiled.environment["LINGTU_NAV_OCTO_TERMINAL_GOAL_Z_TOLERANCE_M"] == "0.15"


@pytest.mark.parametrize("tolerance", [0.25, 0.45])
def test_snapped_global_goal_uses_explicit_planner_tolerance(tolerance: float) -> None:
    config = _compiled_product_config("nav")
    config["native_nav"]["octoplanner3d_terminal_goal_xy_tolerance_m"] = tolerance
    compiled = compile_native_nav_config("nav", config)

    assert float(compiled.environment["LINGTU_NAV_OCTO_TERMINAL_GOAL_XY_TOLERANCE_M"]) == tolerance


def test_snapped_global_goal_does_not_change_execution_arrival_tolerance() -> None:
    config = _compiled_product_config("nav")
    config["native_nav"]["octoplanner3d_terminal_goal_xy_tolerance_m"] = 0.35
    compiled = compile_native_nav_config("nav", config)

    assert compiled.environment["LINGTU_NAV_GOAL_REACHED_M"] == "0.15"
    assert compiled.environment["LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M"] == "0.15"
    assert compiled.environment["LINGTU_NAV_OCTO_TERMINAL_GOAL_XY_TOLERANCE_M"] == "0.35"


@pytest.mark.parametrize("env", ["real", "sim"])
@pytest.mark.parametrize("local_planner", ["scan", "cmu"])
def test_nav_product_height_arrival_is_independent_of_env_and_local_planner(
    env: str,
    local_planner: str,
) -> None:
    resolved = resolve_product_host_runtime(
        "nav",
        env,
        robot="unitree/go2" if env == "real" else "doso/thunder_v4",
        local_planner=local_planner,
    )
    compiled = compile_native_nav_config(
        "nav",
        {
            **resolved.config,
            "native_control_mode": resolved.product_spec["native_control_mode"],
            "native_nav": dict(resolved.product_spec.get("native_nav", {})),
        },
    )

    assert compiled.environment["LINGTU_NAV_GOAL_HEIGHT_TOLERANCE_M"] == "0.35"
    assert "LINGTU_NAV_SCAN_ROUTE_Z_TOLERANCE_M" not in compiled.environment


def test_scan_tuning_does_not_change_global_planner_or_arrival_configuration() -> None:
    config = _compiled_product_config("nav")
    baseline = compile_native_nav_config("nav", config)
    changed = compile_native_nav_config(
        "nav",
        {
            **config,
            "parameters": {
                "scan_planner.collision_weight": 9.0,
                "scan_planner.planning_horizon_m": 1.5,
            },
        },
    )
    prefix = "LINGTU_NAV_OCTO"

    assert {
        key: value for key, value in baseline.environment.items() if key.startswith(prefix)
    } == {
        key: value for key, value in changed.environment.items() if key.startswith(prefix)
    }
    assert changed.environment["LINGTU_NAV_GOAL_HEIGHT_TOLERANCE_M"] == "0.35"


def test_unspecified_goal_height_tolerance_follows_resolved_goal_distance() -> None:
    compiled = compile_native_nav_config(
        "test",
        {
            "native_control_mode": "autonomy",
            "native_nav": {"goal_reached_m": 0.27},
        },
    )

    assert compiled.environment["LINGTU_NAV_GOAL_HEIGHT_TOLERANCE_M"] == "0.27"


@pytest.mark.parametrize("value", [0.0, -0.1, math.inf, math.nan])
def test_goal_height_tolerance_must_be_finite_and_positive(value: float) -> None:
    with pytest.raises(ValueError, match="goal_height_tolerance_m"):
        compile_native_nav_config(
            "test",
            {
                "native_control_mode": "autonomy",
                "native_nav": {"goal_height_tolerance_m": value},
            },
        )


def test_scan_mapd_inflation_covers_asymmetric_body_clearances() -> None:
    config = _compiled_product_config("nav")
    config["collision_clearance_below_m"] = 0.25
    config["collision_clearance_above_m"] = 0.35
    compiled = compile_native_nav_config("nav", config)
    environment = mapd_environment(default_map_settings(), compiled.environment)

    assert environment["LINGTU_MAPD_INFLATION_Z_UP_M"] == "0.25"
    assert environment["LINGTU_MAPD_INFLATION_Z_DOWN_M"] == "0.35"


@pytest.mark.parametrize("product", ["nav", "teleop_avoid"])
def test_go2_vertical_clearance_reaches_nav_and_mapd(product: str) -> None:
    compiled = compile_native_nav_config(product, _compiled_product_config(product))
    assert compiled.environment["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"] == "0.1"
    assert compiled.environment["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"] == "0.24"
    assert compiled.environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M"] == "0.1"
    assert compiled.environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"] == "0.25"
    environment = mapd_environment(default_map_settings(), compiled.environment)
    assert environment["LINGTU_MAPD_INFLATION_Z_UP_M"] == "0.1"
    assert environment["LINGTU_MAPD_INFLATION_Z_DOWN_M"] == "0.24"


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


    with pytest.raises(ValueError, match="tick_hz must be 100"):
        compile_native_nav_config(
            "nav",
            {
                "native_control_mode": "autonomy",
                "native_nav": {"local_planner": "scan", "tick_hz": 20.0},
            },
        )

    with pytest.raises(ValueError, match="local_planner"):
        compile_native_nav_config(
            "nav",
            {
                "native_control_mode": "autonomy",
                "native_nav": {"local_planner": "unknown"},
            },
        )


@pytest.mark.parametrize("mode", ["autonomy", "teleop_avoid"])
def test_scan_does_not_compile_an_unused_2d_terrain_dependency(mode: str) -> None:
    compiled = compile_native_nav_config(
        "nav",
        {
            "native_control_mode": mode,
            "native_nav": {
                "local_planner": "scan",
                "tick_hz": 100.0,
                "use_traversability_cost": True,
            },
        },
    )
    assert compiled.native_nav["use_traversability_cost"] is False
    assert compiled.environment["LINGTU_NAV_USE_TRAVERSABILITY_COST"] == "0"
    assert compiled.native_nav["check_obstacle"] is True


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
    "settings,expected",
    [
        ({}, ("0", "0")),
        (
            {"octoplanner3d_support_patch_radius_cells": 2,
             "octoplanner3d_support_patch_min_samples": 3},
            ("2", "3"),
        ),
    ],
)
def test_octoplanner_support_patch_settings_reach_environment(settings, expected) -> None:
    compiled = compile_native_nav_config(
        "nav", {"native_control_mode": "autonomy", "native_nav": settings}
    )

    assert compiled.environment["LINGTU_NAV_OCTO_SUPPORT_PATCH_RADIUS_CELLS"] == expected[0]
    assert compiled.environment["LINGTU_NAV_OCTO_SUPPORT_PATCH_MIN_SAMPLES"] == expected[1]


@pytest.mark.parametrize(
    ("radius", "offset"),
    [
        (0.40, 0.25),
        (0.25, 0.18),
    ],
)
def test_cmu_octoplanner_default_radius_keeps_the_width_contract(
    radius: float, offset: float
) -> None:
    compiled = compile_native_nav_config(
        "nav",
        {
            "native_control_mode": "autonomy",
            "vehicle_width_m": 0.31,
            "collision_cylinder_radius_m": radius,
            "collision_cylinder_offset_m": offset,
        },
    )

    assert float(compiled.environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"]) == pytest.approx(
        0.155
    )
    assert compiled.parameters["collision_cylinder_radius_m"] == radius
    assert compiled.parameters["collision_cylinder_offset_m"] == offset


@pytest.mark.parametrize(
    ("robot", "radius", "below", "above"),
    [
        ("unitree/go2", 0.25, 0.10, 0.25),
        ("doso/thunder_v4", 0.32, 0.05, 0.35),
    ],
)
def test_scan_global_corridor_uses_robot_width_and_vertical_clearance(
    robot: str, radius: float, below: float, above: float
) -> None:
    compiled = compile_native_nav_config("nav", _compiled_product_config("nav", robot))

    assert float(compiled.environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"]) == pytest.approx(radius)
    assert float(compiled.environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M"]) == pytest.approx(below)
    assert float(compiled.environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"]) == pytest.approx(above)


@pytest.mark.parametrize("robot", ["unitree/go2", "doso/thunder_v4"])
@pytest.mark.parametrize("product", ["nav", "inspection", "tracking", "explore"])
def test_global_profile_keeps_platform_support_and_slope(product: str, robot: str) -> None:
    config = _compiled_product_config(product, robot)
    compiled = compile_native_nav_config(product, config)
    assert config["octoplanner3d_max_slope"] > 0
    for field in ("support_height_m", "support_height_tolerance_m", "max_slope"):
        env = "LINGTU_NAV_OCTO_" + field.upper()
        assert float(compiled.environment[env]) == pytest.approx(config["octoplanner3d_" + field])


@pytest.mark.parametrize(("requested", "effective"), [(0.1, 0.25), (0.44, 0.44), (0.7, 0.7)])
def test_scan_global_radius_override_cannot_shrink_the_corridor_width(
    requested: float, effective: float
) -> None:
    config = _compiled_product_config("nav")
    config["native_nav"]["octoplanner3d_robot_radius"] = requested
    compiled = compile_native_nav_config("nav", config)

    assert float(compiled.environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"]) == pytest.approx(effective)
    assert compiled.parameters["collision_cylinder_radius_m"] == 0.25
    assert compiled.parameters["collision_cylinder_offset_m"] == 0.19


def test_scan_global_clearance_uses_the_mapd_voxel_resolution() -> None:
    config = _compiled_product_config("nav")
    config["collision_clearance_below_m"] = 0.22
    config["collision_clearance_above_m"] = 0.36
    compiled = compile_native_nav_config("nav", config)
    profile = mapd_environment(default_map_settings(), compiled.environment)

    assert profile["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.05"
    assert compiled.environment["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"] == "0.22"
    assert compiled.environment["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"] == "0.36"
    assert compiled.environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M"] == "0.25"
    assert compiled.environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"] == "0.4"


@pytest.mark.parametrize(
    ("mode", "path_speed"),
    [(mode, math.sqrt(2.0)) for mode in ("autonomy", "teleop", "teleop_avoid")],
)
def test_bare_native_config_uses_shared_motion_defaults(mode, path_speed) -> None:
    compiled = compile_native_nav_config("test", {"native_control_mode": mode})
    assert compiled.parameters["teleop_max_speed_mps"] == 1.0
    assert compiled.parameters["path_follower_max_speed_mps"] == pytest.approx(path_speed)
    assert float(compiled.environment["LINGTU_TELEOP_MAX_SPEED_MPS"]) == 1.0
    assert float(compiled.environment["LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS"]) == pytest.approx(path_speed)
    assert compiled.environment["LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M"] == "0.05"


def test_velocity_smoother_is_left_to_the_native_endpoint() -> None:
    compiled = compile_native_nav_config("nav", {"native_control_mode": "autonomy"})

    assert "velocity_smoother" not in compiled.native_nav
    assert compiled.native_nav["smoothing"] is False
    assert compiled.environment["LINGTU_NAV_SMOOTHER_ENABLED"] == "0"


@pytest.mark.parametrize(
    "product", ["nav", "inspection", "teleop", "teleop_avoid", "map", "explore", "tracking"]
)
def test_product_smoother_uses_the_complete_shared_motion_profile(product: str) -> None:
    compiled = compile_native_nav_config(product, _compiled_product_config(product))
    environment = compiled.environment

    for axis in ("X", "Y"):
        assert environment[f"LINGTU_NAV_SMOOTHER_{axis}_MIN_MPS"] == "-1"
        assert environment[f"LINGTU_NAV_SMOOTHER_{axis}_MAX_MPS"] == "1"
        assert environment[f"LINGTU_NAV_SMOOTHER_{axis}_ACCEL_MPS2"] == ("0.75" if product == "nav" else "0.5")
        assert environment[f"LINGTU_NAV_SMOOTHER_{axis}_DECEL_MPS2"] == "4"
    assert environment["LINGTU_NAV_SMOOTHER_YAW_MIN_RADPS"] == "-1"
    assert environment["LINGTU_NAV_SMOOTHER_YAW_MAX_RADPS"] == "1"
    assert environment["LINGTU_NAV_SMOOTHER_YAW_ACCEL_RADPS2"] == "4"
    assert environment["LINGTU_NAV_SMOOTHER_YAW_DECEL_RADPS2"] == "4"


@pytest.mark.parametrize(
    ("control_mode", "expected_yaw_rate"),
    [("autonomy", 0.7), ("teleop", 0.9), ("teleop_avoid", 0.9)],
)
def test_smoother_derives_braking_and_yaw_from_resolved_product_parameters(
    control_mode: str, expected_yaw_rate: float
) -> None:
    compiled = compile_native_nav_config(
        "test",
        {
            "native_control_mode": control_mode,
            "native_nav": {
                "path_follower_max_accel_mps2": 3.5,
                "braking_deceleration_mps2": 2.5,
                "path_follower_max_yaw_rate_rad_s": 0.7,
                "teleop_max_yaw_rate_rad_s": 0.9,
                "path_follower_max_yaw_accel_rad_s2": 3.0,
            },
        },
    )
    environment = compiled.environment

    for axis in ("X", "Y"):
        assert float(environment[f"LINGTU_NAV_SMOOTHER_{axis}_ACCEL_MPS2"]) == 3.5
        assert float(environment[f"LINGTU_NAV_SMOOTHER_{axis}_DECEL_MPS2"]) == 2.5
    assert float(environment["LINGTU_NAV_SMOOTHER_YAW_MIN_RADPS"]) == -expected_yaw_rate
    assert float(environment["LINGTU_NAV_SMOOTHER_YAW_MAX_RADPS"]) == expected_yaw_rate
    assert float(environment["LINGTU_NAV_SMOOTHER_YAW_ACCEL_RADPS2"]) == 3.0
    assert float(environment["LINGTU_NAV_SMOOTHER_YAW_DECEL_RADPS2"]) == 3.0


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


@pytest.mark.parametrize("section", ["scan_follower", "scan_planner"])
def test_scan_tuning_requires_the_unified_parameter_mapping(section) -> None:
    with pytest.raises(ValueError, match="Product parameters"):
        compile_native_nav_config(
            "nav", {"native_control_mode": "autonomy", "native_nav": {section: {}}}
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
