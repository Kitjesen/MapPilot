import json

import pytest
import yaml
from tools.bench.tora_garage_navigation import PROFILE, native_environment, write_scene_map_input
from tools.bench.tora_garage_scene import build_scene


def test_map_fixture_preserves_floor_and_scene_obstacles(tmp_path):
    scene = build_scene(tmp_path / "tora-garage", include_robot=False)
    path = tmp_path / "scene.txt"
    write_scene_map_input(scene, path)
    rows = [list(map(float, line.split())) for line in path.read_text().splitlines()]

    assert rows[0][-1] == len(scene["obstacles"]) + 1
    assert rows[1] == pytest.approx([-9, -6, -.1, 9, 6, 0])
    for row, obstacle in zip(rows[2:], scene["obstacles"], strict=True):
        for axis in range(3):
            assert (row[axis] + row[axis + 3]) / 2 == pytest.approx(obstacle["center"][axis])
            assert (row[axis + 3] - row[axis]) / 2 == pytest.approx(obstacle["half_size"][axis])


def test_component_run_uses_tora_geometry_without_reducing_clearance():
    environment = native_environment(PROFILE)
    geometry = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))["geometry"]

    assert environment["LINGTU_NAV_LOCAL_PLANNER_BACKEND"] == "scan"
    for field in ("CYLINDER_RADIUS", "CYLINDER_OFFSET", "CLEARANCE_BELOW", "CLEARANCE_ABOVE"):
        assert float(environment[f"LINGTU_NAV_COLLISION_{field}_M"]) == geometry[f"collision_{field.lower()}"]
    assert float(environment["LINGTU_NAV_OCTO_SUPPORT_HEIGHT_M"]) == geometry["support_height"]
