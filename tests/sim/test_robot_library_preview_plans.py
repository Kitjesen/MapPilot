from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
from sim.tools.assets.build_robot_library_preview_plans import (
    PreviewPlanError,
    build_render_plan,
)


def _fixture(tmp_path: Path, *, reference_status: str = "resolved") -> Path:
    source_root = tmp_path / "source-library"
    model_root = source_root / "source/robot_lab/data/Robots/vendor/robot_description"
    (model_root / "urdf").mkdir(parents=True)
    (model_root / "meshes").mkdir()
    (model_root / "meshes/body.stl").write_bytes(b"mesh")
    urdf_relative = "source/robot_lab/data/Robots/vendor/robot_description/urdf/robot.urdf"
    (source_root / urdf_relative).write_text(
        """<robot name="fixture">
          <link name="base">
            <collision><geometry><sphere radius="9"/></geometry></collision>
            <visual name="body"><origin xyz="0 1 0" rpy="0 0 0"/><geometry>
              <box size="2 4 6"/>
            </geometry></visual>
          </link>
          <link name="child"><visual><origin xyz="1 0 0" rpy="0 0 0"/><geometry>
            <mesh filename="package://robot_description/meshes/body.stl" scale="2 3 4"/>
          </geometry></visual></link>
          <joint name="hinge" type="revolute">
            <parent link="base"/><child link="child"/><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>""",
        encoding="utf-8",
    )
    inventory = {
        "schema": "lingtu.sim.robot-source-library.v1",
        "presentation_entries": [
            {
                "id": "vendor_robot",
                "version": "robot_lab_2.3.2",
                "label": "Vendor Robot",
                "source_path": "source/robot_lab/data/Robots/vendor/robot_description",
                "urdf": urdf_relative,
                "status": "source_only",
            }
        ],
        "models": [
            {
                "source_directory": "source/robot_lab/data/Robots/vendor/robot_description",
                "urdf_entries": [
                    {
                        "path": urdf_relative,
                        "references": [
                            {
                                "uri": "package://robot_description/meshes/body.stl",
                                "status": reference_status,
                                "path": "source/robot_lab/data/Robots/vendor/robot_description/meshes/body.stl".removeprefix(
                                    "source/robot_lab/data/Robots/"
                                ),
                            }
                        ],
                    }
                ],
            }
        ],
    }
    inventory_path = source_root / "source-library.json"
    inventory_path.write_text(json.dumps(inventory), encoding="utf-8")
    return inventory_path


def test_builds_visual_only_zero_pose_with_compound_transforms_and_scale(tmp_path: Path) -> None:
    inventory = _fixture(tmp_path)
    plan_path = tmp_path / "build/robot-library-render-plan.json"

    document = build_render_plan(inventory, plan_path, tmp_path / "previews")

    robot = document["robots"][0]
    assert robot["id"] == "vendor_robot"
    assert Path(robot["output"]).name == "robot-vendor_robot.png"
    assert Path(robot["output"]).parent == (tmp_path / "previews").resolve()
    assert len(robot["geometries"]) == 2
    assert robot["geometries"][0]["type"] == "box"
    assert robot["geometries"][0]["size"] == [1.0, 2.0, 3.0]
    mesh = robot["geometries"][1]
    assert mesh["type"] == "mesh"
    assert mesh["scale"] == [2.0, 3.0, 4.0]
    assert mesh["position"] == pytest.approx([1.0, 1.0, 0.0])
    assert mesh["rotation"][0] == pytest.approx([0.0, -1.0, 0.0], abs=1e-12)
    assert mesh["rotation"][1] == pytest.approx([1.0, 0.0, 0.0], abs=1e-12)
    assert math.isclose(mesh["rotation"][2][2], 1.0)
    assert all(geometry["size"] != [9.0, 0.0, 0.0] for geometry in robot["geometries"])
    assert json.loads(plan_path.read_text(encoding="utf-8")) == document


def test_missing_visual_mesh_fails_instead_of_emitting_fake_geometry(tmp_path: Path) -> None:
    inventory = _fixture(tmp_path, reference_status="missing")

    with pytest.raises(PreviewPlanError, match="visual mesh is not resolved"):
        build_render_plan(inventory, tmp_path / "plan.json", tmp_path / "previews")
