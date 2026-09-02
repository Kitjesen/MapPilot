"""Focused tests for the Robot Visual Projection asset tooling."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from sim.catalog import compile_robot_visual_manifest
from sim.tools.assets.build_robot_visual_projection import (
    RobotVisualProjectionToolError,
    build_robot_visual_projection,
    write_robot_visual_projection,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


_THUNDER_PACKAGE_DIR = _repo_root() / "sim" / "packages" / "robots" / "thunderv4"
_THUNDER_ASSET_INDEX_PATH = (
    _repo_root() / "build" / "unreal-assets" / "thunderv4-mjcf-fbx" / "asset-index.json"
)
_THUNDER_DESTINATION_PATH = "/Game/RobotSim/Robots/ThunderV4/Meshes"
_THUNDER_MESH_COMPONENTS = {
    "base_link_visual",
    "camera1_link_visual",
    "camera2_link_visual",
    "FL_calf_visual",
    "FL_foot_visual",
    "FL_hip_visual",
    "FL_thigh_visual",
    "FR_calf_visual",
    "FR_foot_visual",
    "FR_hip_visual",
    "FR_thigh_visual",
    "lidar1_link_visual",
    "lidar2_link_visual",
    "RL_calf_visual",
    "RL_foot_visual",
    "RL_hip_visual",
    "RL_thigh_visual",
    "RR_calf_visual",
    "RR_foot_visual",
    "RR_hip_visual",
    "RR_thigh_visual",
}


def _synthetic_mesh_inputs(tmp_path: Path) -> tuple[Path, Path]:
    package_dir = tmp_path / "synthetic_mesh_robot"
    (package_dir / "mjcf").mkdir(parents=True)
    (package_dir / "meshes").mkdir()
    (package_dir / "meshes" / "base.stl").write_text(
        "solid base\nendsolid base\n",
        encoding="utf-8",
    )
    (package_dir / "robot.package.yaml").write_text(
        "\n".join(
            [
                "schema: lingtu.sim.robot-package.v1",
                "id: synthetic_mesh_fixture",
                "version: 1.0.0",
                "kind: robot",
                "physics:",
                "  mjcf: mjcf/robot.xml",
                "visual:",
                "  binding: RobotVisual:SyntheticMesh",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (package_dir / "mjcf" / "robot.xml").write_text(
        '<mujoco><compiler meshdir="../meshes" /><asset><mesh name="base_visual" file="base.stl" /></asset>'
        '<worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody></mujoco>',
        encoding="utf-8",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    source_sha256 = manifest["visuals"][0]["source_sha256"]
    asset_index_path = tmp_path / "asset-index.json"
    asset_index_path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.fbx-asset-index.v1",
                "assets": [
                    {
                        "asset_name": "base_visual",
                        "fbx": "base_visual.fbx",
                        "source_sha256": source_sha256,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    (asset_index_path.parent / "base_visual.fbx").write_bytes(b"FBX fixture\n")
    return package_dir, asset_index_path


def _require_canonical_thunder_inputs() -> tuple[Path, Path]:
    if not _THUNDER_PACKAGE_DIR.is_dir():
        pytest.skip(f"canonical Thunder robot package unavailable: {_THUNDER_PACKAGE_DIR}")
    if not _THUNDER_ASSET_INDEX_PATH.is_file():
        pytest.skip(f"canonical Thunder FBX asset index unavailable: {_THUNDER_ASSET_INDEX_PATH}")

    asset_index = _read_index(_THUNDER_ASSET_INDEX_PATH)
    missing_fbx = []
    for item in asset_index.get("assets", []):
        fbx_name = item.get("fbx") if isinstance(item, dict) else None
        if (
            isinstance(fbx_name, str)
            and not (_THUNDER_ASSET_INDEX_PATH.parent / fbx_name).is_file()
        ):
            missing_fbx.append(fbx_name)
    if missing_fbx:
        pytest.skip(
            "canonical Thunder FBX files unavailable: "
            + ", ".join(sorted(missing_fbx))
        )
    return _THUNDER_PACKAGE_DIR, _THUNDER_ASSET_INDEX_PATH


def _read_index(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_builds_synthetic_mesh_projection_from_fbx_index(tmp_path: Path) -> None:
    package_dir, asset_index_path = _synthetic_mesh_inputs(tmp_path)

    projection = build_robot_visual_projection(
        robot_package=package_dir,
        asset_index_path=asset_index_path,
        destination_path="/Game/RobotSim/Robots/Synthetic/Meshes",
    )

    assert projection["schema"] == "lingtu.sim.robot-visual-projection.v1"
    assert projection["binding"] == "RobotVisual:SyntheticMesh"
    assert [item["visual_id"] for item in projection["components"]] == ["base_visual"]
    assert projection["components"][0]["geometry"]["kind"] == "mesh"
    assert (
        projection["components"][0]["unreal"]["asset_path"]
        == "/Game/RobotSim/Robots/Synthetic/Meshes/base_visual.base_visual"
    )
    assert projection["components"][0]["material"]["source"] == "compiler_default"
    assert projection["components"][0]["material"]["key"] is None
    assert "instance_id" not in json.dumps(projection)


def test_builds_canonical_thunder_mesh_projection_from_fbx_index() -> None:
    package_dir, asset_index_path = _require_canonical_thunder_inputs()

    projection = build_robot_visual_projection(
        robot_package=package_dir,
        asset_index_path=asset_index_path,
        destination_path=_THUNDER_DESTINATION_PATH,
    )

    components = projection["components"]
    component_names = {item["visual_id"] for item in components}

    assert projection["schema"] == "lingtu.sim.robot-visual-projection.v1"
    assert projection["binding"] == "RobotVisual:ThunderV4"
    assert len(components) == 21
    assert component_names == _THUNDER_MESH_COMPONENTS
    assert {
        item["geometry"]["kind"] for item in components
    } == {"mesh"}
    assert all(item["material"]["source"] == "mjcf_material_rgba" for item in components)
    assert {
        item["unreal"]["asset_path"] for item in components
    } == {
        f"{_THUNDER_DESTINATION_PATH}/{asset_name}.{asset_name}"
        for asset_name in _THUNDER_MESH_COMPONENTS
    }
    assert "instance_id" not in json.dumps(projection)


def test_builds_omni_cart_primitive_projection_without_fbx_inputs() -> None:
    package_dir = _repo_root() / "sim" / "packages" / "robots" / "omni_cart"

    projection = build_robot_visual_projection(robot_package=package_dir)

    assert {item["geometry"]["primitive"] for item in projection["components"]} == {
        "box",
        "cylinder",
    }
    assert all(
        item["unreal"]["asset_path"].startswith("/Engine/BasicShapes/")
        for item in projection["components"]
    )
    assert all("material" in item for item in projection["components"])


def test_rejects_tampered_index_hash(tmp_path: Path) -> None:
    package_dir, asset_index_path = _synthetic_mesh_inputs(tmp_path)
    asset_index = _read_index(asset_index_path)
    asset_index["assets"][0]["source_sha256"] = "0" * 64
    tampered_path = tmp_path / "tampered-index.json"
    tampered_path.write_text(json.dumps(asset_index), encoding="utf-8")

    with pytest.raises(RobotVisualProjectionToolError, match="source SHA-256"):
        build_robot_visual_projection(
            robot_package=package_dir,
            asset_index_path=tampered_path,
            destination_path="/Game/RobotSim/Thunder/Meshes",
        )


def test_rejects_missing_fbx_file(tmp_path: Path) -> None:
    package_dir, asset_index_path = _synthetic_mesh_inputs(tmp_path)
    asset_index = _read_index(asset_index_path)
    missing_dir = tmp_path / "missing-fbx"
    missing_dir.mkdir()
    missing_path = missing_dir / "asset-index.json"
    missing_path.write_text(json.dumps(asset_index), encoding="utf-8")

    with pytest.raises(RobotVisualProjectionToolError, match="existing regular file"):
        build_robot_visual_projection(
            robot_package=package_dir,
            asset_index_path=missing_path,
            destination_path="/Game/RobotSim/Thunder/Meshes",
        )


def test_rejects_duplicate_index_entry(tmp_path: Path) -> None:
    package_dir, asset_index_path = _synthetic_mesh_inputs(tmp_path)
    asset_index = _read_index(asset_index_path)
    asset_index["assets"].append(copy.deepcopy(asset_index["assets"][0]))
    duplicate_path = tmp_path / "duplicate-index.json"
    duplicate_path.write_text(json.dumps(asset_index), encoding="utf-8")

    with pytest.raises(RobotVisualProjectionToolError, match="duplicate asset name"):
        build_robot_visual_projection(
            robot_package=package_dir,
            asset_index_path=duplicate_path,
            destination_path="/Game/RobotSim/Thunder/Meshes",
        )


def test_writer_is_byte_identical_and_creates_output_parent(tmp_path: Path) -> None:
    package_dir, asset_index_path = _synthetic_mesh_inputs(tmp_path)
    projection = build_robot_visual_projection(
        robot_package=package_dir,
        asset_index_path=asset_index_path,
        destination_path="/Game/RobotSim/Thunder/Meshes",
    )

    first = write_robot_visual_projection(projection, tmp_path / "one" / "projection.json")
    second = write_robot_visual_projection(projection, tmp_path / "two" / "projection.json")

    assert first.read_bytes() == second.read_bytes()
    assert first.read_bytes().endswith(b"\n")
    assert b"\r\n" not in first.read_bytes()
    assert json.loads(first.read_text(encoding="utf-8")) == projection


def test_writer_rejects_tampered_projection(tmp_path: Path) -> None:
    package_dir = _repo_root() / "sim" / "packages" / "robots" / "omni_cart"
    projection = build_robot_visual_projection(robot_package=package_dir)
    projection["binding"] = "RobotVisual:Tampered"

    with pytest.raises(RobotVisualProjectionToolError, match="digest does not match"):
        write_robot_visual_projection(projection, tmp_path / "projection.json")
