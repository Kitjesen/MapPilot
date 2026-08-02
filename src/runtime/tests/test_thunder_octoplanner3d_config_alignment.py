"""Keep Thunder's native OctoPlanner3D defaults aligned with product intent."""

from __future__ import annotations

import math
import re
from pathlib import Path

from runtime.profiles.catalog.navigation_defaults import THUNDER_OCTOPLANNER_DEFAULTS

REPO_ROOT = Path(__file__).resolve().parents[3]
RUNTIME_ENV = REPO_ROOT / "scripts" / "deploy" / "thunder" / "runtime-env.sh"
NAV_SERVICE = REPO_ROOT / "scripts" / "deploy" / "thunder" / "lingtu-nav-dds.service"
NAV_RUNNER = REPO_ROOT / "scripts" / "deploy" / "thunder" / "run_nav_dds.sh"
NAV_ENDPOINT_CONFIG = (
    REPO_ROOT / "src" / "nav" / "cpp" / "endpoint" / "nav_endpoint_config.cpp"
)

NATIVE_ENV_BY_PRODUCT_KEY = {
    "octoplanner3d_robot_radius": "LINGTU_NAV_OCTO_ROBOT_RADIUS_M",
    "octoplanner3d_max_iterations": "LINGTU_NAV_OCTO_MAX_ITERATIONS",
    "octoplanner3d_snap_search_radius_cells": "LINGTU_NAV_OCTO_SNAP_RADIUS_CELLS",
    "octoplanner3d_require_ground_support": "LINGTU_NAV_OCTO_REQUIRE_GROUND_SUPPORT",
    "octoplanner3d_strict_direct_ground_support": "LINGTU_NAV_OCTO_STRICT_GROUND_SUPPORT",
    "octoplanner3d_ground_support_xy_radius_cells": (
        "LINGTU_NAV_OCTO_GROUND_SUPPORT_XY_RADIUS_CELLS"
    ),
    "octoplanner3d_ground_support_depth_cells": (
        "LINGTU_NAV_OCTO_GROUND_SUPPORT_DEPTH_CELLS"
    ),
    "octoplanner3d_enable_preblocked_costmap": (
        "LINGTU_NAV_OCTO_ENABLE_PREBLOCKED_COSTMAP"
    ),
    "octoplanner3d_preblocked_costmap_radius_cells": (
        "LINGTU_NAV_OCTO_PREBLOCKED_RADIUS_CELLS"
    ),
    "octoplanner3d_preblocked_costmap_weight": "LINGTU_NAV_OCTO_PREBLOCKED_WEIGHT",
    "octoplanner3d_lowest_traversable_only": (
        "LINGTU_NAV_OCTO_LOWEST_TRAVERSABLE_ONLY"
    ),
    "octoplanner3d_floor_change_penalty": "LINGTU_NAV_OCTO_FLOOR_CHANGE_PENALTY",
    "octoplanner3d_max_step_height": "LINGTU_NAV_OCTO_MAX_STEP_HEIGHT_M",
    "octoplanner3d_max_slope": "LINGTU_NAV_OCTO_MAX_SLOPE",
    "octoplanner3d_same_floor_preference": "LINGTU_NAV_OCTO_SAME_FLOOR_PREFERENCE",
    "octoplanner3d_same_floor_z_tolerance": (
        "LINGTU_NAV_OCTO_SAME_FLOOR_Z_TOLERANCE_M"
    ),
    "octoplanner3d_max_same_floor_z_excursion": (
        "LINGTU_NAV_OCTO_MAX_SAME_FLOOR_Z_EXCURSION_M"
    ),
    "octoplanner3d_obstacle_clearance_radius_cells": (
        "LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_RADIUS_CELLS"
    ),
    "octoplanner3d_obstacle_clearance_weight": (
        "LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_WEIGHT"
    ),
}

NON_NATIVE_OCTO_PRODUCT_KEYS = {"octoplanner3d_timeout_s"}


def _runtime_env_defaults() -> dict[str, str]:
    pattern = re.compile(r'^:\s+"\$\{([A-Z0-9_]+):=([^}]*)\}"$')
    defaults: dict[str, str] = {}
    for line in RUNTIME_ENV.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line.strip())
        if match:
            defaults[match.group(1)] = match.group(2)
    return defaults


def _runtime_env_exports() -> set[str]:
    return {
        line.removeprefix("export ").strip()
        for line in RUNTIME_ENV.read_text(encoding="utf-8").splitlines()
        if line.startswith("export ")
    }


def _assert_equivalent(actual: str, expected: object) -> None:
    if isinstance(expected, bool):
        assert actual.lower() in ({"1", "true"} if expected else {"0", "false"})
        return
    if isinstance(expected, int):
        assert int(actual) == expected
        return
    assert isinstance(expected, float)
    assert math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=1e-12)


def test_native_octoplanner_defaults_match_thunder_product_intent() -> None:
    defaults = _runtime_env_defaults()
    exports = _runtime_env_exports()

    for product_key, env_name in NATIVE_ENV_BY_PRODUCT_KEY.items():
        assert product_key in THUNDER_OCTOPLANNER_DEFAULTS
        assert env_name in defaults, f"native navd is missing product constraint {env_name}"
        assert env_name in exports, f"native navd constraint is not exported: {env_name}"
        _assert_equivalent(
            defaults[env_name],
            THUNDER_OCTOPLANNER_DEFAULTS[product_key],
        )


def test_every_native_octoplanner_product_setting_has_an_environment_mapping() -> None:
    product_keys = {
        key
        for key in THUNDER_OCTOPLANNER_DEFAULTS
        if key.startswith("octoplanner3d_")
    }

    assert NON_NATIVE_OCTO_PRODUCT_KEYS <= product_keys
    assert set(NATIVE_ENV_BY_PRODUCT_KEY) == product_keys - NON_NATIVE_OCTO_PRODUCT_KEYS


def test_native_navd_consumes_the_shared_octoplanner_environment() -> None:
    service = NAV_SERVICE.read_text(encoding="utf-8")
    runner = NAV_RUNNER.read_text(encoding="utf-8")
    endpoint_config = NAV_ENDPOINT_CONFIG.read_text(encoding="utf-8")

    assert "run_nav_dds.sh" in service
    assert "source /opt/lingtu/config/thunder-runtime-env.sh" in runner
    assert '--global-planner "${LINGTU_NAV_GLOBAL_PLANNER}"' in runner
    assert '--map "${LINGTU_ACTIVE_PLANNER_MAP}"' in runner
    for env_name in NATIVE_ENV_BY_PRODUCT_KEY.values():
        assert f'"{env_name}"' in endpoint_config


def test_far_is_explicit_and_uses_the_occupancy_artifact() -> None:
    runtime_env = RUNTIME_ENV.read_text(encoding="utf-8")
    runner = NAV_RUNNER.read_text(encoding="utf-8")

    assert 'LINGTU_NAV_GLOBAL_PLANNER:=octoplanner3d' in runtime_env
    assert 'LINGTU_ACTIVE_PLANNER_MAP="${LINGTU_ACTIVE_OCCUPANCY}"' in runtime_env
    assert 'LINGTU_NAV_FAR_ALLOW_UNKNOWN_FALLBACK:=0' in runtime_env
    assert "unsupported LINGTU_NAV_GLOBAL_PLANNER" in runtime_env
    assert "LINGTU_ACTIVE_PLANNER_MAP" in runner
