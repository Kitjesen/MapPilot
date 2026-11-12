from copy import deepcopy
from pathlib import Path
from shutil import copytree

import pytest
import yaml

from lingtu.assembly.config.maps import mapd_environment
from lingtu.assembly.config.native_navigation import compile_native_nav_config
from lingtu.assembly.graph.settings import SYSTEM_SETTINGS_DIR


def _settings():
    path = Path(__file__).resolve().parents[3] / "config/system/settings/maps.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["maps"]


def test_yaml_values_reach_map_service_environment():
    settings = _settings()
    settings["publish"]["cloud_hz"] = 12.0
    settings["input"]["max_points"] = 120000
    settings["occupancy"]["resolution_m"] = 0.10
    settings["occupancy"]["size_x"] = 80
    settings["surface"]["residual_m"] = 0.06
    settings["save"]["dynamic_filter"] = False
    before = deepcopy(settings)

    environment = mapd_environment(settings, {})

    assert environment["LINGTU_MAPD_CLOUD_HZ"] == "12"
    assert environment["LINGTU_MAPD_MAX_POINTS"] == "120000"
    assert environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.1"
    assert environment["LINGTU_MAPD_OCCUPANCY_SIZE_X"] == "80"
    assert environment["LINGTU_MAPD_SURFACE_RESIDUAL_M"] == "0.06"
    assert environment["LINGTU_SAVE_DYNAMIC_FILTER"] == "0"
    assert environment["LINGTU_MAPD_QUERY_MAX_JSON_BYTES"] == str(4 * 1024 * 1024)
    assert settings == before


@pytest.mark.parametrize("radius, below, above", [(0.25, 0.15, 0.35), (0.38, 0.30, 0.50)])
def test_scan_uses_selected_robot_inflation(radius, below, above):
    settings = _settings()
    native = {
        "LINGTU_NAV_LOCAL_PLANNER_BACKEND": "scan",
        "LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M": str(radius),
        "LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M": str(below),
        "LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M": str(above),
    }

    environment = mapd_environment(settings, native)
    generic = mapd_environment(settings, {"LINGTU_NAV_LOCAL_PLANNER_BACKEND": "cmu"})

    assert environment["LINGTU_MAPD_INFLATION_RADIUS_M"] == str(radius)
    assert environment["LINGTU_MAPD_INFLATION_Z_UP_M"] == str(below)
    assert environment["LINGTU_MAPD_INFLATION_Z_DOWN_M"] == str(above)
    assert generic["LINGTU_MAPD_INFLATION_RADIUS_M"] == "0.25"
    assert generic["LINGTU_MAPD_INFLATION_Z_UP_M"] == "0.1"
    assert generic["LINGTU_MAPD_INFLATION_Z_DOWN_M"] == "0.1"


@pytest.mark.parametrize("group, field, value", [
    ("publish", "cloud_hz", 0),
    ("input", "max_points", 1.5),
    ("occupancy", "resolution_m", float("nan")),
    ("occupancy", "p_hit", 1),
    ("surface", "min_columns", 2),
    ("save", "dynamic_filter", "false"),
])
def test_invalid_map_settings_fail_at_compilation(group, field, value):
    settings = _settings()
    settings[group][field] = value
    with pytest.raises(ValueError, match=f"maps.{group}.{field}"):
        mapd_environment(settings, {})


def test_unknown_map_setting_is_not_silently_ignored():
    settings = _settings()
    settings["occupancy"]["resoluton_m"] = 0.1
    with pytest.raises(ValueError, match=r"maps\.occupancy"):
        mapd_environment(settings, {})


def test_map_save_frame_is_not_a_tunable_setting():
    settings = _settings()
    assert "LINGTU_MAP_FRAME" not in mapd_environment(settings, {})
    settings["save"]["frame"] = "odom"
    with pytest.raises(ValueError, match=r"maps\.save"):
        mapd_environment(settings, {})


def test_standalone_scan_uses_selected_settings_directory(tmp_path):
    directory = tmp_path / "settings"
    copytree(SYSTEM_SETTINGS_DIR, directory)
    path = directory / "maps.yaml"
    settings = yaml.safe_load(path.read_text(encoding="utf-8"))
    settings["maps"]["occupancy"]["resolution_m"] = 0.1
    path.write_text(yaml.safe_dump(settings), encoding="utf-8")

    compiled = compile_native_nav_config(
        "nav", {"native_control_mode": "autonomy", "native_nav": {"local_planner": "scan", "tick_hz": 100}},
        settings_directory=directory,
    )

    assert compiled.map_resolution_m == 0.1
