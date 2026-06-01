import pytest

pytestmark = [pytest.mark.sim]

from pathlib import Path

from drivers.sim.mujoco_scene_metadata import extract_robot_height_obstacle_boxes


def test_industrial_park_scene_metadata_extracts_robot_height_obstacles():
    scene = Path("sim/worlds/mujoco/industrial_park_scene.xml")

    obstacles = extract_robot_height_obstacle_boxes(scene)
    names = {item["name"] for item in obstacles}

    assert "machine_b1" in names
    assert "rack_c1" in names
    assert "wall_south" in names
    assert "ground_main" not in names
    assert "roof_deck_main" not in names
    assert len(obstacles) >= 20
