import pytest

pytestmark = [pytest.mark.sim]

from pathlib import Path

from drivers.sim.mujoco.scene import extract_obstacle_boxes


def test_industrial_park_scene_metadata_extracts_robot_height_obstacles():
    scene = Path("sim/worlds/mujoco/industrial_park_scene.xml")

    obstacles = extract_obstacle_boxes(scene)
    names = {item["name"] for item in obstacles}

    assert "machine_b1" in names
    assert "rack_c1" in names
    assert "wall_south" in names
    assert "ground_main" not in names
    assert "roof_deck_main" not in names
    assert len(obstacles) >= 20


def test_scene_metadata_applies_parent_body_transform_and_rotation(tmp_path: Path):
    scene = tmp_path / "rotated_scene.xml"
    scene.write_text(
        """
<mujoco>
  <worldbody>
    <body name="obstacle_body" pos="1 2 0.5" euler="0 0 1.57079632679">
      <geom name="rotated_crate" type="box" pos="1 0 0" size="1 0.25 0.4"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )

    obstacles = extract_obstacle_boxes(scene)

    assert len(obstacles) == 1
    obstacle = obstacles[0]
    assert obstacle["name"] == "rotated_crate"
    assert obstacle["position"] == pytest.approx([1.0, 3.0, 0.5], abs=1e-6)
    assert obstacle["half_size"] == pytest.approx([0.25, 1.0, 0.4], abs=1e-6)
