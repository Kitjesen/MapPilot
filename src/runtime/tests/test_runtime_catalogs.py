from __future__ import annotations

import ast
from pathlib import Path

from runtime.blueprints.catalog.endpoints import (
    COMPAT_RUNTIME_ENDPOINT_ALIASES as CATALOG_COMPAT_ENDPOINT_ALIASES,
)
from runtime.blueprints.catalog.endpoints import (
    PRODUCT_RUNTIME_ENDPOINT_ALIASES as CATALOG_PRODUCT_ENDPOINT_ALIASES,
)
from runtime.blueprints.catalog.endpoints import (
    RUNTIME_ENDPOINT_ALIASES as CATALOG_ENDPOINT_ALIASES,
)
from runtime.blueprints.catalog.endpoints import RUNTIME_ENDPOINTS as CATALOG_ENDPOINTS
from runtime.blueprints.catalog.products import (
    LIGHTWEIGHT_PRODUCT_PROFILES,
    PRODUCT_INTENT_PROFILES,
    PRODUCT_PROFILES,
    PROFILE_SNAPSHOT_TARGETS,
    SIMULATION_ENTRYPOINT_PROFILES,
    SIMULATION_PROFILES,
    is_product_profile,
    is_simulation_profile,
    product_profile,
)
from runtime.blueprints.catalog.products import (
    PROFILES as CATALOG_PROFILES,
)
from runtime.blueprints.catalog.robots import (
    CANONICAL_ROBOT_DRIVER_PROFILES,
    CANONICAL_ROBOT_PRESETS,
    COMPAT_ROBOT_DRIVER_PROFILES as CATALOG_COMPAT_ROBOT_DRIVER_PROFILES,
    COMPAT_ROBOT_PRESETS as CATALOG_COMPAT_ROBOT_PRESETS,
    ROBOT_DRIVER_PROFILES,
    ROBOT_PRESETS,
    robot_driver_module_name,
    robot_driver_profile_names,
    robot_preset_names,
)
from runtime.blueprints.catalog.runtime_paths import _resolve_tomogram
from runtime.profiles.catalog.endpoints import (
    PRODUCT_PROFILE_ENDPOINTS,
    RUNTIME_ENDPOINTS as RUNTIME_CATALOG_ENDPOINTS,
)
from runtime.profiles.catalog.endpoint_adapter_configs import (
    CMU_UNITY_CONFIG,
    GAZEBO_CONFIG,
    MUJOCO_LIVE_CONFIG,
)
from runtime.profiles.catalog.products import (
    OPTIONAL_NATIVE_PRODUCT_PROFILES,
    PROFILE_NAME_OVERLAP,
    PROFILES as RUNTIME_CATALOG_PROFILES,
)
from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
    DEFAULT_SAMPLE_OCTOPLANNER3D_MAP,
    DEFAULT_SAMPLE_TOMOGRAM,
    RUNTIME_MAP_FRAME_ID,
    RUNTIME_ODOM_FRAME_ID,
    _resolve_octoplanner3d_map,
)
from runtime.profiles.catalog.robot_runtime_defaults import (
    CANONICAL_ROBOT_RUNTIME_DEFAULTS,
    COMPAT_ROBOT_RUNTIME_DEFAULTS,
    ROBOT_RUNTIME_DEFAULTS,
)
from runtime.profiles.binding_policy import (
    LIDAR_LEGACY_DRIVER_START_KEYS,
    ROS2_CAMERA_BRIDGE_ENABLE_KEYS,
    ROS2_RERUN_BRIDGE_ENABLE_KEYS,
    resolved_autonomy_backend_selection,
    ros2_autonomy_backend_violations,
    ros2_runtime_binding_violations,
)
from runtime.profiles.catalog.robot_archives import (
    ROBOT_ARCHIVE_SCHEMA_VERSION,
    robot_archive,
    robot_archive_canonical_driver_modules,
    robot_archive_canonical_driver_profiles,
    robot_archive_canonical_presets,
    robot_archive_canonical_runtime_defaults,
    robot_archive_compat_driver_modules,
    robot_archive_compat_driver_profiles,
    robot_archive_compat_presets,
    robot_archive_compat_runtime_defaults,
    robot_archive_path,
)
from runtime.profiles.catalog.robots import ROBOT_PRESETS as RUNTIME_ROBOT_PRESETS
from runtime.profiles.endpoint_config import endpoint_config_for_profile
from runtime.profiles.resolver import resolve_runtime_config
from runtime.profiles.endpoints import (
    RUNTIME_ENDPOINTS as RUNTIME_ENDPOINTS_RUNTIME,
)
from runtime.profiles.endpoints import (
    resolve_runtime_run_spec as resolve_runtime_run_spec_runtime,
)
from runtime.blueprints.runtime_endpoint import (
    RUNTIME_ENDPOINTS as COMPAT_ENDPOINTS,
)
from runtime.blueprints.runtime_endpoint import (
    resolve_runtime_run_spec as resolve_runtime_run_spec_compat,
)
from runtime.blueprints.runtime_endpoint import runtime_endpoint_names
from runtime.blueprints.stacks.driver import RobotProfile
from runtime.runtime_profiles import PROFILES as COMPAT_PROFILES
from runtime.runtime_profiles import ROBOT_PRESETS as RUNTIME_PROFILES_ROBOT_PRESETS

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"


def test_runtime_profiles_reexports_robot_catalog() -> None:
    assert COMPAT_PROFILES is CATALOG_PROFILES
    assert RUNTIME_PROFILES_ROBOT_PRESETS is ROBOT_PRESETS


def test_blueprint_catalog_imports_are_runtime_catalog_facades() -> None:
    assert CATALOG_ENDPOINTS is RUNTIME_CATALOG_ENDPOINTS
    assert CATALOG_PROFILES is RUNTIME_CATALOG_PROFILES
    assert ROBOT_PRESETS is RUNTIME_ROBOT_PRESETS


def test_runtime_endpoint_resolver_reexports_endpoint_catalog() -> None:
    assert COMPAT_ENDPOINTS is CATALOG_ENDPOINTS
    assert COMPAT_ENDPOINTS is RUNTIME_ENDPOINTS_RUNTIME
    assert resolve_runtime_run_spec_compat is resolve_runtime_run_spec_runtime
    assert PRODUCT_PROFILE_ENDPOINTS["nav"] == ("thunder_field", "replay")
    assert CATALOG_ENDPOINTS["thunder_lite"].robot_preset == "thunder"
    assert CATALOG_ENDPOINTS["thunder_lite"].data_source == "thunder_lite_local"
    assert CATALOG_ENDPOINTS["thunder_lite"].config_overrides == {
        "enable_device_manager": False,
    }
    assert CATALOG_ENDPOINTS["thunder_field"].robot_preset == "thunder"
    assert CATALOG_ENDPOINTS["thunder_field"].data_source == "thunder_field"
    assert CATALOG_ENDPOINTS["thunder_field"].module_transport == "local"
    assert CATALOG_ENDPOINTS["thunder_field"].endpoint_transport == "dds"
    assert CATALOG_ENDPOINTS["thunder_field"].endpoint_contract == "thunder_field_dds_v1"
    assert CATALOG_ENDPOINTS["thunder_field"].config_overrides == {
        "enable_device_manager": False,
        "enable_robot_driver": False,
        "enable_lidar": False,
        "command_output_mode": "endpoint_only",
        "hardware_control_boundary": "dds_endpoint_source",
        "localization_adapter": "cpp_slam_status",
        "nav_in_adapter": "dds_nav_input",
        "nav_out_adapter": "dds_nav_output",
        "enable_nav_in": True,
        "enable_nav_out": True,
        "enable_camera": True,
        "camera_backend": "orbbec_native",
    }
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["field"] == "thunder_field"
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["thunder-field"] == "thunder_field"
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["thunder"] == "thunder_field"
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["thunder-lite"] == "thunder_lite"
    assert CATALOG_ENDPOINT_ALIASES["thunder-field"] == "thunder_field"
    assert "portable_fastlio2" not in CATALOG_ENDPOINTS
    assert "portable-lio" not in CATALOG_PRODUCT_ENDPOINT_ALIASES
    assert "windows-fastlio2" not in CATALOG_PRODUCT_ENDPOINT_ALIASES


def test_legacy_board_endpoint_names_are_compatibility_only() -> None:
    assert CATALOG_COMPAT_ENDPOINT_ALIASES["real_s100p"] == "thunder_field"
    assert CATALOG_COMPAT_ENDPOINT_ALIASES["s100p"] == "thunder_field"
    assert CATALOG_ENDPOINT_ALIASES["real_s100p"] == "thunder_field"


def test_runtime_endpoint_names_hide_compat_aliases_by_default() -> None:
    product_names = runtime_endpoint_names(include_aliases=True)
    compat_names = runtime_endpoint_names(
        include_aliases=True,
        include_compat_aliases=True,
    )

    assert "thunder-field" in product_names
    assert "field" in product_names
    assert "real_s100p" not in product_names
    assert "s100p" not in product_names
    assert "real_s100p" in compat_names
    assert "s100p" in compat_names


def test_robot_driver_profiles_share_robot_catalog_names() -> None:
    assert set(ROBOT_DRIVER_PROFILES) == set(ROBOT_PRESETS)
    assert set(RobotProfile.known_presets()) == set(ROBOT_PRESETS)
    assert set(ROBOT_RUNTIME_DEFAULTS) == set(ROBOT_PRESETS)


def test_robot_catalog_hides_legacy_board_names_from_canonical_presets() -> None:
    assert "thunder" in CANONICAL_ROBOT_PRESETS
    assert "thunder" in CANONICAL_ROBOT_DRIVER_PROFILES
    assert "s100p" not in CANONICAL_ROBOT_PRESETS
    assert "navigate" not in CANONICAL_ROBOT_PRESETS
    assert CATALOG_COMPAT_ROBOT_PRESETS["s100p"] == CANONICAL_ROBOT_PRESETS["thunder"]
    assert (
        CATALOG_COMPAT_ROBOT_DRIVER_PROFILES["s100p"]
        == CANONICAL_ROBOT_DRIVER_PROFILES["thunder"]
    )
    assert "s100p" not in robot_preset_names(include_compat=False)
    assert "s100p" not in robot_driver_profile_names(include_compat=False)
    assert "s100p" in robot_preset_names(include_compat=True)
    assert "s100p" in robot_driver_profile_names(include_compat=True)
    assert "s100p" not in RobotProfile.known_presets(include_compat=False)
    assert robot_driver_module_name("thunder") == "ThunderDriver"
    assert robot_driver_module_name("s100p") == "ThunderDriver"
    assert COMPAT_ROBOT_RUNTIME_DEFAULTS["s100p"] == CANONICAL_ROBOT_RUNTIME_DEFAULTS[
        "thunder"
    ]


def test_thunder_is_canonical_product_robot_name() -> None:
    assert ROBOT_PRESETS["thunder"]["robot"] == "thunder"
    assert ROBOT_PRESETS["thunder"]["dog_host"] == "127.0.0.1"
    assert ROBOT_PRESETS["thunder_remote"]["dog_host"] == "192.168.66.190"
    assert ROBOT_PRESETS["s100p"]["robot"] == "thunder"
    assert ROBOT_PRESETS["navigate"]["robot"] == "thunder"


def test_thunder_robot_catalog_is_sourced_from_robot_archive() -> None:
    archive = robot_archive("thunder")

    assert robot_archive_path("thunder").name == "thunder.yaml"
    assert archive["schema_version"] == ROBOT_ARCHIVE_SCHEMA_VERSION
    assert archive["robot"] == "thunder"
    assert robot_archive_canonical_presets("thunder") == {
        "thunder": CANONICAL_ROBOT_PRESETS["thunder"],
        "thunder_remote": CANONICAL_ROBOT_PRESETS["thunder_remote"],
    }
    assert robot_archive_compat_presets("thunder") == CATALOG_COMPAT_ROBOT_PRESETS
    assert robot_archive_canonical_driver_profiles("thunder") == {
        "thunder": CANONICAL_ROBOT_DRIVER_PROFILES["thunder"],
        "thunder_remote": CANONICAL_ROBOT_DRIVER_PROFILES["thunder_remote"],
    }
    assert robot_archive_compat_driver_profiles("thunder") == (
        CATALOG_COMPAT_ROBOT_DRIVER_PROFILES
    )
    assert robot_archive_canonical_driver_modules("thunder") == {
        "thunder": "ThunderDriver",
        "thunder_remote": "ThunderDriver",
    }
    assert robot_archive_compat_driver_modules("thunder") == {
        "s100p": "ThunderDriver",
        "navigate": "ThunderDriver",
    }
    assert robot_archive_canonical_runtime_defaults("thunder") == {
        "thunder": CANONICAL_ROBOT_RUNTIME_DEFAULTS["thunder"],
        "thunder_remote": CANONICAL_ROBOT_RUNTIME_DEFAULTS["thunder_remote"],
    }
    assert robot_archive_compat_runtime_defaults("thunder") == (
        COMPAT_ROBOT_RUNTIME_DEFAULTS
    )


def test_robot_presets_do_not_own_upper_stack_defaults() -> None:
    upper_stack_fields = {"slam_profile", "detector", "encoder"}

    for preset, config in ROBOT_PRESETS.items():
        assert upper_stack_fields.isdisjoint(config), preset

    assert ROBOT_RUNTIME_DEFAULTS["thunder"]["slam_profile"] == "localizer"
    assert ROBOT_RUNTIME_DEFAULTS["thunder"]["detector"] == "bpu"
    assert ROBOT_RUNTIME_DEFAULTS["stub"]["slam_profile"] == "none"
    assert ROBOT_RUNTIME_DEFAULTS["stub"]["detector"] == "yoloe"


def test_product_catalog_groups_cover_known_profiles() -> None:
    grouped = set(PRODUCT_PROFILES) | set(SIMULATION_PROFILES)

    assert grouped <= set(CATALOG_PROFILES)
    assert set(PROFILE_SNAPSHOT_TARGETS) <= grouped
    assert not set(PRODUCT_PROFILES) & set(SIMULATION_PROFILES)
    assert PROFILE_NAME_OVERLAP == frozenset()
    assert LIGHTWEIGHT_PRODUCT_PROFILES == ("lite",)
    assert "lite" in PRODUCT_PROFILES
    assert "lite" not in PROFILE_SNAPSHOT_TARGETS
    assert product_profile("nav") == CATALOG_PROFILES["nav"]


def test_profile_catalog_exposes_explicit_product_and_simulation_views() -> None:
    assert PRODUCT_INTENT_PROFILES is not SIMULATION_ENTRYPOINT_PROFILES
    assert set(PRODUCT_INTENT_PROFILES) == set(PRODUCT_PROFILES)
    assert set(SIMULATION_ENTRYPOINT_PROFILES) == set(SIMULATION_PROFILES)
    assert set(PRODUCT_INTENT_PROFILES).isdisjoint(SIMULATION_ENTRYPOINT_PROFILES)

    assert is_product_profile("lite") is True
    assert is_product_profile("nav") is True
    assert is_product_profile("sim") is False
    assert is_simulation_profile("sim") is True
    assert is_simulation_profile("dev") is True
    assert is_simulation_profile("lite") is False


def test_product_intents_do_not_own_robot_defaults() -> None:
    for profile, config in PRODUCT_INTENT_PROFILES.items():
        assert "_default_robot" not in config, profile
        assert "dog_host" not in config, profile
        assert "dog_port" not in config, profile
        assert "auto_enable" not in config, profile
        assert "auto_standup" not in config, profile


def test_endpoint_config_helper_owns_endpoint_layer_composition() -> None:
    endpoint = CATALOG_ENDPOINTS["thunder_field"]
    endpoint_config = endpoint_config_for_profile(endpoint, "nav")

    assert endpoint.config_for_profile("nav") == endpoint_config
    assert endpoint_config["_runtime_endpoint"] == "thunder_field"
    assert endpoint_config["_endpoint_data_source"] == "thunder_field"
    assert endpoint_config["_endpoint_transport"] == "dds"
    assert "slam_profile" not in endpoint_config
    assert endpoint_config["enable_lidar"] is False
    assert endpoint_config["command_output_mode"] == "endpoint_only"


def test_product_endpoint_matrix_matches_endpoint_catalog() -> None:
    assert set(PRODUCT_PROFILE_ENDPOINTS) == set(PRODUCT_PROFILES)
    for profile, expected_endpoints in PRODUCT_PROFILE_ENDPOINTS.items():
        actual_endpoints = tuple(
            endpoint_name
            for endpoint_name, endpoint in CATALOG_ENDPOINTS.items()
            if profile in endpoint.supported_profiles
        )
        assert actual_endpoints == expected_endpoints


def test_product_endpoint_layers_do_not_write_legacy_planner_backend() -> None:
    for profile in PRODUCT_PROFILES:
        assert "planner_backend" not in CATALOG_PROFILES[profile], profile

    for endpoint_name, endpoint in CATALOG_ENDPOINTS.items():
        supports_product = bool(set(endpoint.supported_profiles) & set(PRODUCT_PROFILES))
        if supports_product:
            assert "planner_backend" not in endpoint.config_overrides, endpoint_name
        for profile in set(endpoint.profile_overrides) & set(PRODUCT_PROFILES):
            assert "planner_backend" not in endpoint.profile_overrides[profile], (
                endpoint_name,
                profile,
            )


def test_runtime_paths_own_shared_runtime_defaults() -> None:
    assert DEFAULT_GATEWAY_PORT == 5050
    assert DEFAULT_PLANNING_FRAME_ID == RUNTIME_MAP_FRAME_ID
    assert RUNTIME_MAP_FRAME_ID == "map"
    assert RUNTIME_ODOM_FRAME_ID == "odom"
    assert DEFAULT_SAMPLE_TOMOGRAM.endswith("building2_9.pickle")
    assert DEFAULT_SAMPLE_OCTOPLANNER3D_MAP.endswith("result_cleaned.bt")

    resolved = Path(_resolve_tomogram()).as_posix()
    assert resolved.endswith(DEFAULT_SAMPLE_TOMOGRAM)
    octo_map = Path(_resolve_octoplanner3d_map()).as_posix()
    assert octo_map.endswith((".bt", ".ot", ".octomap", ".pcd"))


def test_catalog_profiles_consume_shared_runtime_defaults() -> None:
    assert CATALOG_PROFILES["lite"]["gateway_port"] == DEFAULT_GATEWAY_PORT
    assert CATALOG_PROFILES["lite"]["planning_frame_id"] == DEFAULT_PLANNING_FRAME_ID
    assert CATALOG_PROFILES["nav"]["gateway_port"] == DEFAULT_GATEWAY_PORT
    assert CATALOG_PROFILES["sim"]["tomogram"] == _resolve_octoplanner3d_map()
    assert MUJOCO_LIVE_CONFIG["gateway_port"] == DEFAULT_GATEWAY_PORT
    assert MUJOCO_LIVE_CONFIG["planning_frame_id"] == DEFAULT_PLANNING_FRAME_ID
    assert MUJOCO_LIVE_CONFIG["enable_map_out"] is False
    assert MUJOCO_LIVE_CONFIG["enable_nav_out"] is False
    assert GAZEBO_CONFIG["tomogram"] == _resolve_octoplanner3d_map()
    assert CMU_UNITY_CONFIG["python_autonomy_backend"] == "nanobind"
    assert CMU_UNITY_CONFIG["python_path_follower_backend"] == "nav_kernel"

    assert CATALOG_ENDPOINTS["mujoco_live"].profile_overrides[
        "tare_explore"
    ]["planning_frame_id"] == RUNTIME_ODOM_FRAME_ID
    assert CATALOG_ENDPOINTS["mujoco_live"].profile_overrides[
        "sim_mujoco_octo_live"
    ]["tomogram"] == _resolve_octoplanner3d_map()


def test_nav_product_does_not_own_field_bridge_policy() -> None:
    assert "slam_profile" not in CATALOG_PROFILES["nav"]
    assert "endpoint_transport" not in CATALOG_PROFILES["nav"]
    assert "nav" not in CATALOG_ENDPOINTS["thunder_field"].profile_overrides


def _ros_bridge_keys(config: dict) -> list[str]:
    return [
        key
        for key in config
        if key.startswith("enable_ros2")
    ]


def test_catalog_profiles_do_not_own_ros_bridge_switches() -> None:
    for profile in CATALOG_PROFILES:
        assert _ros_bridge_keys(CATALOG_PROFILES[profile]) == [], profile


def test_catalog_profiles_do_not_start_legacy_lidar_driver() -> None:
    for profile, config in CATALOG_PROFILES.items():
        for key in LIDAR_LEGACY_DRIVER_START_KEYS:
            assert key not in config, profile


def test_catalog_profiles_do_not_enable_ros2_camera_bridge() -> None:
    for profile, config in CATALOG_PROFILES.items():
        for key in ROS2_CAMERA_BRIDGE_ENABLE_KEYS:
            assert key not in config, profile


def test_catalog_profiles_do_not_enable_ros2_rerun_bridge() -> None:
    for profile, config in CATALOG_PROFILES.items():
        for key in ROS2_RERUN_BRIDGE_ENABLE_KEYS:
            assert key not in config, profile


def test_catalog_profiles_do_not_default_to_ros2_autonomy_backends() -> None:
    for profile, config in CATALOG_PROFILES.items():
        enable_native = bool(config.get("enable_native", False))
        assert (
            ros2_autonomy_backend_violations(config, enable_native=enable_native)
            == []
        ), profile


def test_default_product_runtime_configs_do_not_select_ros2_bindings() -> None:
    for profile in PRODUCT_PROFILES:
        resolved = resolve_runtime_config(profile)
        config = resolved.config
        enable_native = bool(config.get("enable_native", False))
        assert (
            ros2_runtime_binding_violations(config, enable_native=enable_native)
            == []
        ), profile


def test_product_runtime_configs_use_ros_free_autonomy_backends() -> None:
    expected = {
        "teleop": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "teleop_avoid": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "lite": {
            "terrain_backend": "simple",
            "local_planner_backend": "simple",
            "path_follower_backend": "pid",
        },
        "map": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "nav": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "tracking": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "inspection": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "explore": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "tare_explore": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "super_lio": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "super_lio_relocation": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
    }

    assert set(expected) == set(PRODUCT_PROFILES)

    for profile in PRODUCT_PROFILES:
        resolved = resolve_runtime_config(profile)
        config = resolved.config
        enable_native = bool(config.get("enable_native", False))

        assert (
            resolved_autonomy_backend_selection(config, enable_native=enable_native)
            == expected[profile]
        )


def test_no_product_profiles_are_optional_native_runtime_configs() -> None:
    assert OPTIONAL_NATIVE_PRODUCT_PROFILES == ()


def test_octoplanner3d_product_profiles_use_supported_map_formats() -> None:
    for profile in ("nav", "explore", "tare_explore", "super_lio", "super_lio_relocation"):
        config = CATALOG_PROFILES[profile]
        assert config["planner"] == "octoplanner3d"
        assert Path(config["tomogram"]).suffix.lower() in {
            ".bt",
            ".ot",
            ".octomap",
            ".pcd",
        }


def test_runtime_endpoints_do_not_own_ros_bridge_switches() -> None:
    for endpoint_name, endpoint in CATALOG_ENDPOINTS.items():
        assert _ros_bridge_keys(dict(endpoint.config_overrides)) == [], endpoint_name
        for profile, overrides in endpoint.profile_overrides.items():
            assert _ros_bridge_keys(dict(overrides)) == [], (endpoint_name, profile)


def test_runtime_endpoints_do_not_start_legacy_lidar_driver() -> None:
    for endpoint_name, endpoint in CATALOG_ENDPOINTS.items():
        endpoint_config = dict(endpoint.config_overrides)
        for key in LIDAR_LEGACY_DRIVER_START_KEYS:
            assert key not in endpoint_config, endpoint_name
        for profile, overrides in endpoint.profile_overrides.items():
            profile_config = dict(overrides)
            for key in LIDAR_LEGACY_DRIVER_START_KEYS:
                assert key not in profile_config, (endpoint_name, profile)


def test_runtime_endpoints_do_not_enable_ros2_camera_bridge() -> None:
    for endpoint_name, endpoint in CATALOG_ENDPOINTS.items():
        endpoint_config = dict(endpoint.config_overrides)
        for key in ROS2_CAMERA_BRIDGE_ENABLE_KEYS:
            assert key not in endpoint_config, endpoint_name
        for profile, overrides in endpoint.profile_overrides.items():
            profile_config = dict(overrides)
            for key in ROS2_CAMERA_BRIDGE_ENABLE_KEYS:
                assert key not in profile_config, (endpoint_name, profile)


def test_runtime_endpoints_do_not_enable_ros2_rerun_bridge() -> None:
    for endpoint_name, endpoint in CATALOG_ENDPOINTS.items():
        endpoint_config = dict(endpoint.config_overrides)
        for key in ROS2_RERUN_BRIDGE_ENABLE_KEYS:
            assert key not in endpoint_config, endpoint_name
        for profile, overrides in endpoint.profile_overrides.items():
            profile_config = dict(overrides)
            for key in ROS2_RERUN_BRIDGE_ENABLE_KEYS:
                assert key not in profile_config, (endpoint_name, profile)


def test_product_blueprints_do_not_import_compat_runtime_profiles() -> None:
    path = SRC / "runtime" / "blueprints" / "products" / "thunder.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))

    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)

    assert "runtime.runtime_profiles" not in modules
    assert "runtime.profiles.catalog.runtime_paths" in modules
    assert "runtime.blueprints.catalog.runtime_paths" not in modules


def test_profile_graph_does_not_hardcode_legacy_robot_driver_aliases() -> None:
    source = (SRC / "runtime" / "blueprints" / "profile_graph.py").read_text(
        encoding="utf-8-sig"
    )

    assert '"s100p": "ThunderDriver"' not in source
    assert '"navigate": "ThunderDriver"' not in source
    assert "robot_driver_module_name" in source


def test_catalog_tomogram_fallback_points_to_repo_sample(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    resolved = Path(_resolve_tomogram())

    assert resolved.as_posix().endswith(DEFAULT_SAMPLE_TOMOGRAM)


def test_catalog_octoplanner3d_map_prefers_active_saved_map(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_MAP", raising=False)
    monkeypatch.delenv("NAV_OCTOMAP", raising=False)
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    active = tmp_path / "active"
    active.mkdir()
    active_map = active / "map.pcd"
    active_map.write_text("VERSION .7\n", encoding="utf-8")

    assert Path(_resolve_octoplanner3d_map()) == active_map


def test_catalog_octoplanner3d_map_env_override_wins(monkeypatch, tmp_path) -> None:
    active = tmp_path / "active"
    active.mkdir()
    (active / "map.pcd").write_text("VERSION .7\n", encoding="utf-8")
    override = tmp_path / "override.bt"
    override.write_bytes(b"# OctoMap OcTree binary file\n")

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    monkeypatch.setenv("LINGTU_OCTOPLANNER3D_MAP", str(override))
    monkeypatch.delenv("NAV_OCTOMAP", raising=False)

    assert Path(_resolve_octoplanner3d_map()) == override
