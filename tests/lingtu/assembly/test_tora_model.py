from pathlib import Path

import pytest
import yaml

from lingtu.assembly.compile_plan import compile_run_plan
from lingtu.assembly.config.maps import default_map_settings, mapd_environment
from lingtu.assembly.config.native_navigation import compile_native_nav_config
from lingtu.assembly.config.product import _load_robot_model
from runtime.config import GeometryConfig

ROOT = Path(__file__).resolve().parents[3] / "config/robots/paxini/tora_double_one_x"


def test_tora_model_is_discoverable_without_a_fake_driver() -> None:
    robot, directory, model = _load_robot_model("paxini/tora_double_one_x")

    assert robot == "paxini/tora_double_one_x"
    assert (directory / model["urdf"]).is_file()
    assert (directory / model["navigation_geometry"]).is_file()
    with pytest.raises(FileNotFoundError, match=r"robot\.yaml"):
        compile_run_plan("nav", "real", robot=robot)


def test_tora_envelope_compiles_with_existing_navigation_fields() -> None:
    profile = yaml.safe_load((ROOT / "navigation.yaml").read_text(encoding="utf-8"))
    geometry = GeometryConfig(**profile["geometry"])
    config = {
        "native_control_mode": "autonomy",
        "native_nav": {"local_planner": "scan", "tick_hz": 100.0},
        "vehicle_length_m": geometry.vehicle_length,
        "vehicle_width_m": geometry.vehicle_width,
        "vehicle_height_m": geometry.vehicle_height,
        "collision_cylinder_radius_m": geometry.collision_cylinder_radius,
        "collision_cylinder_offset_m": geometry.collision_cylinder_offset,
        "collision_hard_margin_m": geometry.collision_hard_margin,
        "collision_clearance_below_m": geometry.collision_clearance_below,
        "collision_clearance_above_m": geometry.collision_clearance_above,
        "octoplanner3d_support_height_m": geometry.support_height,
        "octoplanner3d_support_height_tolerance_m": geometry.support_height_tolerance,
    }
    compiled = compile_native_nav_config("nav", config)
    environment = compiled.environment

    assert float(environment["LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M"]) == .52
    assert float(environment["LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M"]) == 0.0
    assert float(environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"]) == .52
    assert float(environment["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"]) == .14
    assert float(environment["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"]) == 1.57
    assert float(environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M"]) == pytest.approx(.15)
    assert float(environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"]) == 1.60
    assert float(environment["LINGTU_NAV_OCTO_SUPPORT_HEIGHT_M"]) == pytest.approx(.178000853265161)
    map_environment = mapd_environment(default_map_settings(), environment)
    assert float(map_environment["LINGTU_MAPD_INFLATION_RADIUS_M"]) == .52
    assert float(map_environment["LINGTU_MAPD_INFLATION_Z_UP_M"]) == .14
    assert float(map_environment["LINGTU_MAPD_INFLATION_Z_DOWN_M"]) == 1.57
