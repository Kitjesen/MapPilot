from __future__ import annotations

from pathlib import Path

from runtime.profiles.binding_policy import (
    LEGACY_NAV_IN_ADAPTER_KEYS,
    LEGACY_NAV_OUT_ADAPTER_KEYS,
    NAV_IN_ADAPTER_KEYS,
    NAV_OUT_ADAPTER_KEYS,
    autonomy_backend_selection,
    endpoint_contract_for_config,
    endpoint_transport_for_config,
    exploration_backend_for_config,
    global_planner_backend_selection,
    legacy_sensor_binding_violations,
    localization_adapter_for_config,
    map_output_uses_dds,
    module_transport_for_config,
    nav_kernel_backend_required,
    resolved_autonomy_backend_selection,
    ros2_autonomy_backend_violations,
    ros2_camera_bridge_violations,
    ros2_driver_runtime_violations,
    ros2_global_planner_backend_violations,
    ros2_lidar_driver_violations,
    ros2_rerun_bridge_violations,
    ros2_runtime_binding_violations,
)

ROOT = Path(__file__).resolve().parents[3]


def test_runtime_binding_policy_prefers_operator_transport_over_endpoint_default() -> None:
    config = {
        "_module_transport": "local",
        "module_transport": "shm",
        "_endpoint_transport": "local",
        "endpoint_transport": "dds",
    }

    assert module_transport_for_config(config) == "shm"
    assert endpoint_transport_for_config(config) == "dds"


def test_runtime_binding_policy_does_not_infer_removed_lcm_adapter() -> None:
    config = {
        "_endpoint_transport": "zmq",
        "_endpoint_contract": "thunder_field_zmq_v1",
    }

    assert endpoint_contract_for_config(config) == "thunder_field_zmq_v1"
    assert localization_adapter_for_config(config) == ""


def test_runtime_binding_policy_derives_dds_localization_from_endpoint_transport() -> None:
    config = {"_endpoint_transport": "dds"}

    assert localization_adapter_for_config(config) == "dds_endpoint"


def test_runtime_binding_policy_keeps_nav_adapter_keys_canonical() -> None:
    assert NAV_IN_ADAPTER_KEYS == ("nav_in_adapter",)
    assert NAV_OUT_ADAPTER_KEYS == ("nav_out_adapter",)
    assert "endpoint_command_bridge" in LEGACY_NAV_IN_ADAPTER_KEYS
    assert "endpoint_path_bridge" in LEGACY_NAV_OUT_ADAPTER_KEYS


def test_autonomy_backend_selection_uses_ros_free_native_defaults() -> None:
    assert autonomy_backend_selection({}, enable_native=True) == {
        "terrain_backend": None,
        "local_planner_backend": "nanobind",
        "path_follower_backend": "nav_kernel",
    }


def test_autonomy_backend_selection_uses_python_defaults_when_native_disabled() -> None:
    assert autonomy_backend_selection({}, enable_native=False) == {
        "terrain_backend": None,
        "local_planner_backend": "nanobind",
        "path_follower_backend": "nav_kernel",
    }


def test_autonomy_backend_selection_accepts_python_backend_aliases() -> None:
    assert autonomy_backend_selection(
        {
            "terrain_backend": "simple",
            "python_autonomy_backend": "simple",
            "python_path_follower_backend": "pid",
        },
        enable_native=False,
    ) == {
        "terrain_backend": "simple",
        "local_planner_backend": "simple",
        "path_follower_backend": "pid",
    }


def test_autonomy_backend_selection_prefers_explicit_backend_over_alias() -> None:
    assert autonomy_backend_selection(
        {
            "local_planner_backend": "cmu_py",
            "path_follower_backend": "pid",
        },
        enable_native=False,
    ) == {
        "terrain_backend": None,
        "local_planner_backend": "cmu_py",
        "path_follower_backend": "pid",
    }


def test_resolved_autonomy_backend_selection_keeps_local_only_alias_off_terrain() -> None:
    assert resolved_autonomy_backend_selection(
        {
            "local_planner_backend": "cmu_py",
            "path_follower_backend": "pid",
        },
        enable_native=False,
    ) == {
        "terrain_backend": "nanobind",
        "local_planner_backend": "cmu_py",
        "path_follower_backend": "pid",
    }


def test_nav_kernel_backend_required_matches_resolved_autonomy_chain() -> None:
    assert nav_kernel_backend_required({}, enable_native=False) is True
    assert (
        nav_kernel_backend_required(
            {
                "terrain_backend": "simple",
                "local_planner_backend": "simple",
                "path_follower_backend": "pid",
            },
            enable_native=False,
        )
        is False
    )


def test_ros2_autonomy_backend_violations_reports_native_module_backends() -> None:
    assert ros2_autonomy_backend_violations(
        {
            "terrain_backend": "native",
            "local_planner_backend": "cmu",
            "path_follower_backend": "pure_pursuit",
        },
        enable_native=True,
    ) == [
        "terrain_backend=native requires ROS2 NativeModule",
        "local_planner_backend=cmu requires ROS2 NativeModule",
        "path_follower_backend=pure_pursuit requires ROS2 NativeModule",
    ]


def test_ros2_autonomy_backend_violations_allows_ros_free_defaults() -> None:
    assert ros2_autonomy_backend_violations({}, enable_native=True) == []
    assert ros2_autonomy_backend_violations({}, enable_native=False) == []


def test_ros2_driver_runtime_violations_report_ros2_driver_boundary() -> None:
    assert ros2_driver_runtime_violations({"robot": "sim_ros2", "driver_module": "ROS2SimDriverModule"}) == [
        "robot=sim_ros2 selects a ROS2 driver runtime",
        "driver_module=ros2simdrivermodule selects a ROS2 driver runtime",
    ]


def test_ros2_driver_runtime_violations_allow_product_and_portable_drivers() -> None:
    assert ros2_driver_runtime_violations({"robot": "thunder"}) == []
    assert ros2_driver_runtime_violations({"robot": "stub"}) == []
    assert ros2_driver_runtime_violations({"robot": "sim_mujoco"}) == []
    assert ros2_driver_runtime_violations({"robot": "sim_gazebo"}) == []


def test_ros2_lidar_driver_violations_report_legacy_driver_start() -> None:
    assert ros2_lidar_driver_violations({"lidar_start_driver": True, "start_lidar_driver": True}) == [
        "lidar_start_driver=true starts the legacy local Livox ROS2 driver",
        "start_lidar_driver=true starts the legacy local Livox ROS2 driver",
    ]


def test_ros2_lidar_driver_violations_allow_endpoint_subscription() -> None:
    assert ros2_lidar_driver_violations({"enable_lidar": True}) == []


def test_legacy_sensor_binding_violations_report_driver_sensor_paths() -> None:
    assert legacy_sensor_binding_violations(
        {
            "use_driver_camera": True,
            "use_driver_lidar": True,
            "use_driver_imu": True,
            "legacy_driver_sensor_fallback": True,
            "enable_legacy_sim_lidar": True,
        }
    ) == [
        "use_driver_camera=true enables a legacy driver sensor path",
        "use_driver_lidar=true enables a legacy driver sensor path",
        "use_driver_imu=true enables a legacy driver sensor path",
        "legacy_driver_sensor_fallback=true enables a legacy driver sensor path",
        "enable_legacy_sim_lidar=true enables a legacy driver sensor path",
    ]


def test_legacy_sensor_binding_violations_allow_short_sensor_roles() -> None:
    assert (
        legacy_sensor_binding_violations(
            {
                "enable_camera": True,
                "camera_backend": "orbbec",
                "enable_lidar": True,
                "lidar_backend": "mid360",
                "enable_imu": True,
                "imu_backend": "livox",
            }
        )
        == []
    )


def test_ros2_camera_bridge_violations_report_explicit_ros2_bridge() -> None:
    assert ros2_camera_bridge_violations({"enable_ros2_camera_bridge": True}) == [
        "enable_ros2_camera_bridge=true enables a ROS2 camera bridge",
    ]


def test_ros2_camera_bridge_violations_allow_disabled_or_native_camera() -> None:
    assert ros2_camera_bridge_violations({"enable_camera": True}) == []
    assert ros2_camera_bridge_violations({"use_driver_camera": True}) == []


def test_ros2_rerun_bridge_violations_report_explicit_ros2_bridge() -> None:
    assert ros2_rerun_bridge_violations({"enable_ros2_rerun_bridge": True}) == [
        "enable_ros2_rerun_bridge=true enables a ROS2 Rerun bridge",
    ]


def test_ros2_rerun_bridge_violations_allow_module_native_rerun() -> None:
    assert ros2_rerun_bridge_violations({"enable_rerun": True}) == []


def test_global_planner_backend_selection_normalizes_octplanner_alias() -> None:
    assert global_planner_backend_selection(
        {
            "planner": "octplanner",
            "fallback_planner_name": "astar",
        }
    ) == {
        "primary": "octoplanner3d",
        "fallback_planners": ["astar"],
    }


def test_ros2_global_planner_backend_violations_allow_current_product_chain() -> None:
    assert (
        ros2_global_planner_backend_violations(
            {
                "planner": "octoplanner3d",
                "fallback_planner_name": "astar",
            }
        )
        == []
    )


def test_ros2_global_planner_backend_violations_report_ros2_wrappers() -> None:
    assert ros2_global_planner_backend_violations(
        {
            "planner": "ros2_pct",
            "fallback_planners": ["astar", "octoplanner3d_ros2"],
        }
    ) == [
        "planner=ros2_pct selects a ROS2 global planner wrapper",
        "fallback_planner=octoplanner3d_ros2 selects a ROS2 global planner wrapper",
    ]


def test_exploration_backend_for_config_defaults_to_none() -> None:
    assert exploration_backend_for_config({}) == "none"
    assert exploration_backend_for_config({"exploration_backend": "TARE"}) == "tare"


def test_ros2_runtime_binding_violations_allows_tare_dds_bridge() -> None:
    assert (
        ros2_runtime_binding_violations(
            {
                "slam_profile": "none",
                "exploration_backend": "tare",
            },
            enable_native=False,
        )
        == []
    )


def test_ros2_runtime_binding_violations_reports_legacy_tare_native_exploration() -> None:
    assert ros2_runtime_binding_violations(
        {
            "slam_profile": "none",
            "exploration_backend": "tare_native",
        },
        enable_native=False,
    ) == [
        "exploration_backend=tare_native requires ROS2 NativeModule",
    ]


def test_ros2_runtime_binding_violations_allows_external_tare_bridge() -> None:
    assert (
        ros2_runtime_binding_violations(
            {
                "slam_profile": "none",
                "exploration_backend": "tare_external",
            },
            enable_native=False,
        )
        == []
    )


def test_ros2_runtime_binding_violations_allows_native_slam_default() -> None:
    assert (
        ros2_runtime_binding_violations(
            {"slam_profile": "bridge"},
            enable_native=False,
        )
        == []
    )


def test_ros2_runtime_binding_violations_reports_explicit_ros2_adapters() -> None:
    assert ros2_runtime_binding_violations(
        {
            "slam_profile": "bridge",
            "localization_adapter": "ros2_slam_bridge",
            "nav_out_adapter": "ros2_nav_output",
            "nav_in_adapter": "ros2_nav_input",
            "enable_ros2_bridge": True,
            "enable_map_out": True,
            "map_out_adapter": "ros2_map_output",
            "terrain_backend": "native",
            "local_planner_backend": "cmu",
            "path_follower_backend": "pure_pursuit",
        },
        enable_native=True,
    ) == [
        "terrain_backend=native requires ROS2 NativeModule",
        "local_planner_backend=cmu requires ROS2 NativeModule",
        "path_follower_backend=pure_pursuit requires ROS2 NativeModule",
        "enable_ros2_bridge=true enables a ROS2 IO adapter",
        "map output adapter selects a ROS2 IO adapter",
        "localization_adapter=ros2_slam_bridge selects a ROS2 localization adapter",
        "nav_out_adapter=ros2_nav_output selects a ROS2 IO adapter",
        "nav_in_adapter=ros2_nav_input selects a ROS2 IO adapter",
    ]


def test_ros2_runtime_binding_violations_reports_ros2_global_planner() -> None:
    assert ros2_runtime_binding_violations(
        {
            "slam_profile": "none",
            "planner": "pct_ros2",
        },
        enable_native=False,
    ) == [
        "planner=pct_ros2 selects a ROS2 global planner wrapper",
    ]


def test_ros2_runtime_binding_violations_reports_ros2_driver_runtime() -> None:
    assert ros2_runtime_binding_violations(
        {"slam_profile": "none", "robot": "sim_ros2"},
        enable_native=False,
    ) == ["robot=sim_ros2 selects a ROS2 driver runtime"]


def test_ros2_runtime_binding_violations_reports_legacy_lidar_driver() -> None:
    assert ros2_runtime_binding_violations(
        {"slam_profile": "none", "lidar_start_driver": True},
        enable_native=False,
    ) == ["lidar_start_driver=true starts the legacy local Livox ROS2 driver"]


def test_ros2_runtime_binding_violations_reports_ros2_camera_bridge() -> None:
    assert ros2_runtime_binding_violations(
        {"slam_profile": "none", "enable_ros2_camera_bridge": True},
        enable_native=False,
    ) == ["enable_ros2_camera_bridge=true enables a ROS2 camera bridge"]


def test_ros2_runtime_binding_violations_reports_ros2_rerun_bridge() -> None:
    assert ros2_runtime_binding_violations(
        {"slam_profile": "none", "enable_ros2_rerun_bridge": True},
        enable_native=False,
    ) == ["enable_ros2_rerun_bridge=true enables a ROS2 Rerun bridge"]


def test_ros2_runtime_binding_violations_reports_endpoint_enablement_without_adapter() -> None:
    assert ros2_runtime_binding_violations(
        {
            "slam_profile": "none",
            "enable_endpoint_command_bridge": True,
            "enable_endpoint_path_bridge": True,
            "enable_endpoint_waypoint_bridge": True,
            "enable_endpoint_grid_bridge": True,
        },
        enable_native=False,
    ) == [
        "enable_endpoint_grid_bridge=true without explicit non-ROS map output adapter has no safe default",
        "enable_endpoint_command_bridge=true without explicit non-ROS navigation input adapter has no safe default",
        "enable_endpoint_path_bridge=true without explicit non-ROS navigation output adapter has no safe default",
        "enable_endpoint_waypoint_bridge=true without explicit non-ROS navigation output adapter has no safe default",
    ]


def test_map_output_can_use_typed_dds_adapter() -> None:
    assert map_output_uses_dds({"map_out_adapter": "dds_map_output"})
    assert map_output_uses_dds({"enable_map_out": True, "_endpoint_transport": "dds"})
    assert (
        ros2_runtime_binding_violations(
            {
                "slam_profile": "none",
                "enable_map_out": True,
                "map_out_adapter": "dds_map_output",
            },
            enable_native=False,
        )
        == []
    )


def test_ros2_runtime_binding_violations_reports_nav_io_enablement_without_adapter() -> None:
    assert ros2_runtime_binding_violations(
        {
            "slam_profile": "none",
            "enable_nav_in": True,
            "enable_nav_out": True,
        },
        enable_native=False,
    ) == [
        "enable_nav_in=true without explicit non-ROS navigation input adapter has no safe default",
        "enable_nav_out=true without explicit non-ROS navigation output adapter has no safe default",
    ]


def test_ros2_runtime_binding_violations_rejects_removed_endpoint_nav_adapters() -> None:
    assert ros2_runtime_binding_violations(
        {
            "slam_profile": "bridge",
            "_endpoint_transport": "zmq",
            "_endpoint_contract": "thunder_field_zmq_v1",
            "endpoint_egress_adapter": "removed_endpoint",
            "endpoint_ingress_adapter": "removed_endpoint",
            "enable_nav_in": True,
            "enable_nav_out": True,
        },
        enable_native=False,
    ) == [
        "enable_nav_in=true without explicit non-ROS navigation input adapter has no safe default",
        "enable_nav_out=true without explicit non-ROS navigation output adapter has no safe default",
    ]


def test_runtime_binding_policy_is_the_blueprint_delivery_seam() -> None:
    checked_files = [
        ROOT / "src/lingtu/assembly/graph.py",
        ROOT / "src/lingtu/assembly/stacks/composition.py",
    ]

    for path in checked_files:
        source = path.read_text(encoding="utf-8-sig")
        assert "from runtime.profiles.binding_policy import" in source
        assert "def _endpoint_egress_uses_lcm" not in source
        assert "def _endpoint_ingress_uses_lcm" not in source
        assert "def _localization_adapter_for_config" not in source


def test_navigation_stack_has_no_python_endpoint_adapter_hooks() -> None:
    source = (ROOT / "src/lingtu/assembly/stacks/navigation.py").read_text(encoding="utf-8-sig")

    assert "enable_ros2_" not in source
    assert "enable_endpoint_" not in source
    assert "navigation_io" not in source


def test_autonomy_chain_delegates_backend_selection_to_runtime_policy() -> None:
    source = (ROOT / "src/lingtu/assembly/stacks/autonomy_chain.py").read_text(encoding="utf-8-sig")

    assert "from runtime.profiles.binding_policy import resolved_autonomy_backend_selection" in source
    assert "resolved_autonomy_backend_selection(" in source
    assert "from runtime.profiles.binding_policy import autonomy_backend_selection" not in source
    assert "\nautonomy_backend_selection(" not in source
    assert 'config.get("local_planner_backend", "cmu")' not in source
    assert 'config.get("python_autonomy_backend", "nanobind")' not in source
    assert 'config.get("python_path_follower_backend", "nav_kernel")' not in source
