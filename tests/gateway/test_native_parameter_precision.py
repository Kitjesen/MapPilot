import math
import time
from types import SimpleNamespace

import pytest

from gateway.navigation.status import _native_endpoint_readiness


@pytest.mark.parametrize("parameter", ["path_follower.max_speed_mps", "teleop_planner_horizon_m"])
@pytest.mark.parametrize("actual,accepted", [(1.414214, True), (1.4, False), (1.414215, False)])
def test_native_parameter_comparison_preserves_status_decimal_precision(parameter, actual, accepted):
    environment = {
        "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": str(math.sqrt(2)),
        "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": "0.1",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": "4",
        "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": "1",
        "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": "0.2",
        "LINGTU_NAV_WAYPOINT_REACHED_M": "0.3",
        "LINGTU_NAV_GOAL_REACHED_M": "0.2",
        "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": "2",
        "LINGTU_TELEOP_PLANNER_HORIZON_M": str(math.sqrt(2)),
        "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": "60",
    }
    plan = SimpleNamespace(
        product="nav", has_process=lambda role: role == "nav",
        required_capabilities=("operator_assisted_local_planner_control",),
        native_process_environment=environment,
        native_nav={"control_mode": "autonomy", "use_traversability_cost": False},
        lifecycle={"requires_map": True},
    )
    snapshot = {
        "stamp_s": time.time(), "native_product": {"product": "nav"},
        "control_mode": "autonomy", "navigation_ready": True,
        "control_loop_health": {"ready": True, "healthy": True},
        "input_gate": {"ready": True}, "active_cmd_source": "none",
        "control_authority": {"owner": "native_endpoint"},
        "path_follower": {"max_speed_mps": 1.414214, "min_speed_mps": 0.1,
                          "max_accel_mps2": 4, "lookahead_m": 1, "goal_tolerance_m": 0.2},
        "nav_loop": {"waypoint_reached_m": 0.3, "goal_reached_m": 0.2, "corridor_lookahead_m": 2},
        "teleop_planner_horizon_m": 1.414214, "teleop_planner_max_deviation_deg": 60,
        "teleop_local_planner": True, "check_obstacle": True, "use_traversability_cost": False,
        "global_planner": "octoplanner3d", "planner_map": "/maps/active/octomap.ot",
        "publish_cmd_vel": True,
    }
    if parameter.startswith("path_follower."):
        snapshot["path_follower"]["max_speed_mps"] = actual
    else:
        snapshot[parameter] = actual
    result = _native_endpoint_readiness(
        {"mode": "navigating"}, SimpleNamespace(_compiled_run_plan=plan), native_status=snapshot,
    )
    assert result["ok"] is accepted
    if accepted:
        assert result["parameter_mismatches"] == {}
        assert result["blockers"] == []
    else:
        assert result["parameter_mismatches"][parameter] == {"actual": actual, "expected": math.sqrt(2)}
        assert result["blockers"] == ["native_product_parameters_mismatch"]
