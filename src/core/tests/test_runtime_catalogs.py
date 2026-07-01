from __future__ import annotations

import ast
from pathlib import Path

from compat.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME
from core.blueprints.catalog.endpoints import (
    COMPAT_RUNTIME_ENDPOINT_ALIASES as CATALOG_COMPAT_ENDPOINT_ALIASES,
)
from core.blueprints.catalog.endpoints import (
    PRODUCT_RUNTIME_ENDPOINT_ALIASES as CATALOG_PRODUCT_ENDPOINT_ALIASES,
)
from core.blueprints.catalog.endpoints import (
    RUNTIME_ENDPOINT_ALIASES as CATALOG_ENDPOINT_ALIASES,
)
from core.blueprints.catalog.endpoints import RUNTIME_ENDPOINTS as CATALOG_ENDPOINTS
from core.blueprints.catalog.products import (
    LIGHTWEIGHT_PRODUCT_PROFILES,
    PRODUCT_PROFILES,
    PROFILE_SNAPSHOT_TARGETS,
    SIMULATION_PROFILES,
    product_profile,
)
from core.blueprints.catalog.products import (
    PROFILES as CATALOG_PROFILES,
)
from core.blueprints.catalog.robots import (
    CANONICAL_ROBOT_DRIVER_PROFILES,
    CANONICAL_ROBOT_PRESETS,
    COMPAT_ROBOT_DRIVER_PROFILES,
    COMPAT_ROBOT_PRESETS,
    ROBOT_DRIVER_PROFILES,
    ROBOT_PRESETS,
    robot_driver_module_name,
    robot_driver_profile_names,
    robot_preset_names,
)
from core.blueprints.catalog.runtime_paths import _resolve_tomogram
from core.blueprints.runtime_endpoint import (
    RUNTIME_ENDPOINTS as COMPAT_ENDPOINTS,
)
from core.blueprints.runtime_endpoint import runtime_endpoint_names
from core.blueprints.stacks.driver import RobotProfile
from core.runtime_profiles import PROFILES as COMPAT_PROFILES
from core.runtime_profiles import ROBOT_PRESETS as COMPAT_ROBOT_PRESETS

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"


def test_runtime_profiles_reexports_robot_catalog() -> None:
    assert COMPAT_PROFILES is CATALOG_PROFILES
    assert COMPAT_ROBOT_PRESETS is ROBOT_PRESETS


def test_runtime_endpoint_resolver_reexports_endpoint_catalog() -> None:
    assert COMPAT_ENDPOINTS is CATALOG_ENDPOINTS
    assert CATALOG_ENDPOINTS["thunder_lite"].robot_preset == "thunder"
    assert CATALOG_ENDPOINTS["thunder_lite"].data_source == "thunder_lite_local"
    assert CATALOG_ENDPOINTS["thunder_field"].robot_preset == "thunder"
    assert CATALOG_ENDPOINTS["thunder_field"].data_source == "thunder_field"
    assert CATALOG_ENDPOINTS["thunder_field"].module_transport == "local"
    assert CATALOG_ENDPOINTS["thunder_field"].endpoint_transport == "lcm"
    assert (
        CATALOG_ENDPOINTS["thunder_field"].endpoint_contract
        == THUNDER_FIELD_LCM_CONTRACT_NAME
    )
    assert CATALOG_ENDPOINTS["thunder_field"].config_overrides == {
        "enable_robot_driver": False,
        "command_output_mode": "endpoint_only",
        "hardware_control_boundary": "lcm_endpoint_source",
        "localization_adapter": "lcm_endpoint",
        "endpoint_ingress_adapter": "lcm_endpoint",
        "endpoint_egress_adapter": "lcm_endpoint",
        "enable_endpoint_command_bridge": True,
        "enable_endpoint_path_bridge": True,
        "enable_camera": False,
    }
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["field"] == "thunder_field"
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["thunder-field"] == "thunder_field"
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["thunder"] == "thunder_field"
    assert CATALOG_PRODUCT_ENDPOINT_ALIASES["thunder-lite"] == "thunder_lite"
    assert CATALOG_ENDPOINT_ALIASES["thunder-field"] == "thunder_field"


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


def test_robot_catalog_hides_legacy_board_names_from_canonical_presets() -> None:
    assert "thunder" in CANONICAL_ROBOT_PRESETS
    assert "thunder" in CANONICAL_ROBOT_DRIVER_PROFILES
    assert "s100p" not in CANONICAL_ROBOT_PRESETS
    assert "navigate" not in CANONICAL_ROBOT_PRESETS
    assert COMPAT_ROBOT_PRESETS["s100p"] == CANONICAL_ROBOT_PRESETS["thunder"]
    assert (
        COMPAT_ROBOT_DRIVER_PROFILES["s100p"]
        == CANONICAL_ROBOT_DRIVER_PROFILES["thunder"]
    )
    assert "s100p" not in robot_preset_names(include_compat=False)
    assert "s100p" not in robot_driver_profile_names(include_compat=False)
    assert "s100p" in robot_preset_names(include_compat=True)
    assert "s100p" in robot_driver_profile_names(include_compat=True)
    assert "s100p" not in RobotProfile.known_presets(include_compat=False)
    assert robot_driver_module_name("thunder") == "ThunderDriver"
    assert robot_driver_module_name("s100p") == "ThunderDriver"


def test_thunder_is_canonical_product_robot_name() -> None:
    assert ROBOT_PRESETS["thunder"]["robot"] == "thunder"
    assert ROBOT_PRESETS["thunder"]["dog_host"] == "127.0.0.1"
    assert ROBOT_PRESETS["thunder_remote"]["dog_host"] == "192.168.66.190"
    assert ROBOT_PRESETS["s100p"]["robot"] == "thunder"
    assert ROBOT_PRESETS["navigate"]["robot"] == "thunder"


def test_product_catalog_groups_cover_known_profiles() -> None:
    grouped = set(PRODUCT_PROFILES) | set(SIMULATION_PROFILES)

    assert grouped <= set(CATALOG_PROFILES)
    assert set(PROFILE_SNAPSHOT_TARGETS) <= grouped
    assert not set(PRODUCT_PROFILES) & set(SIMULATION_PROFILES)
    assert LIGHTWEIGHT_PRODUCT_PROFILES == ("lite",)
    assert "lite" in PRODUCT_PROFILES
    assert "lite" not in PROFILE_SNAPSHOT_TARGETS
    assert product_profile("nav") == CATALOG_PROFILES["nav"]


def test_nav_product_does_not_own_field_bridge_policy() -> None:
    assert "slam_profile" not in CATALOG_PROFILES["nav"]
    assert "endpoint_transport" not in CATALOG_PROFILES["nav"]
    assert CATALOG_ENDPOINTS["thunder_field"].profile_overrides["nav"] == {
        "slam_profile": "bridge",
    }


def _ros_bridge_keys(config: dict) -> list[str]:
    return [
        key
        for key in config
        if key.startswith("enable_ros2")
    ]


def test_catalog_profiles_do_not_own_ros_bridge_switches() -> None:
    for profile in CATALOG_PROFILES:
        assert _ros_bridge_keys(CATALOG_PROFILES[profile]) == [], profile


def test_runtime_endpoints_do_not_own_ros_bridge_switches() -> None:
    for endpoint_name, endpoint in CATALOG_ENDPOINTS.items():
        assert _ros_bridge_keys(dict(endpoint.config_overrides)) == [], endpoint_name
        for profile, overrides in endpoint.profile_overrides.items():
            assert _ros_bridge_keys(dict(overrides)) == [], (endpoint_name, profile)


def test_product_blueprints_do_not_import_compat_runtime_profiles() -> None:
    path = SRC / "core" / "blueprints" / "products" / "thunder.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))

    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)

    assert "core.runtime_profiles" not in modules
    assert "core.blueprints.catalog.runtime_paths" in modules


def test_profile_graph_does_not_hardcode_legacy_robot_driver_aliases() -> None:
    source = (SRC / "core" / "blueprints" / "profile_graph.py").read_text(
        encoding="utf-8-sig"
    )

    assert '"s100p": "ThunderDriver"' not in source
    assert '"navigate": "ThunderDriver"' not in source
    assert "robot_driver_module_name" in source


def test_catalog_tomogram_fallback_points_to_repo_sample(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    resolved = Path(_resolve_tomogram())

    assert resolved.as_posix().endswith(
        "src/global_planning/pct_planner/rsc/tomogram/building2_9.pickle"
    )
