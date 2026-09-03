"""High-fidelity ThunderV4 visual mesh replacement contracts."""

from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
from sim.catalog import CatalogResolver, compile_robot_visual_manifest

REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / "sim" / "packages" / "robots" / "doso" / "thunder_v4"
PACKAGE_MANIFEST = PACKAGE_ROOT / "robot.package.yaml"

EXPECTED_TRIANGLES = {
    "base_link.STL": 3_996,
    "camera1_Link.STL": 2_000,
    "camera2_Link.STL": 37_438,
    "fl_calf_Link.STL": 69_994,
    "fl_foot_Link.STL": 342_266,
    "fl_hip_Link.STL": 434_348,
    "fl_thigh_Link.STL": 466_450,
    "fr_calf_Link.STL": 69_994,
    "fr_foot_Link.STL": 342_266,
    "fr_hip_link.STL": 434_348,
    "fr_thigh_Link.STL": 466_450,
    "lidar1_Link.STL": 2_000,
    "lidar2_Link.STL": 52_855,
    "rl_calf_Link.STL": 69_994,
    "rl_foot_Link.STL": 342_266,
    "rl_hip_Link.STL": 434_348,
    "rl_thigh_Link.STL": 466_450,
    "rr_calf_Link.STL": 69_994,
    "rr_foot_Link.STL": 342_266,
    "rr_hip_Link.STL": 434_348,
    "rr_thigh_Link.STL": 466_450,
}

def _binary_stl_triangle_count(path: Path) -> int:
    with path.open("rb") as stream:
        stream.seek(80)
        raw_count = stream.read(4)
    assert len(raw_count) == 4
    count = struct.unpack("<I", raw_count)[0]
    assert path.stat().st_size == 84 + count * 50
    return count


def test_high_fidelity_meshes_are_catalog_visible_and_match_manifest() -> None:
    resolver = CatalogResolver.from_repository(REPO_ROOT)
    package = resolver.find_package("thunderv4@1.0.3", kind="robot")
    assert package.manifest_path == PACKAGE_MANIFEST

    compiled = compile_robot_visual_manifest(PACKAGE_ROOT).to_dict()
    compiled_sources = {
        Path(component["geometry"]["source_mesh"]).name
        for component in compiled["visuals"]
        if component["geometry"]["kind"] == "mesh"
    }
    assert set(compiled_sources) == set(EXPECTED_TRIANGLES)
    assert {
        Path(component["geometry"]["source_mesh"]).parent.as_posix()
        for component in compiled["visuals"]
        if component["geometry"]["kind"] == "mesh"
    } == {"visual/meshes"}

    meshes = PACKAGE_ROOT / "visual" / "meshes"
    actual_files = {path.name for path in meshes.iterdir() if path.is_file()}
    assert actual_files == set(EXPECTED_TRIANGLES)
    for name, expected_count in EXPECTED_TRIANGLES.items():
        path = meshes / name
        assert _binary_stl_triangle_count(path) == expected_count


def test_mujoco_keeps_decoder_safe_low_poly_meshes() -> None:
    physics_meshes = PACKAGE_ROOT / "meshes"
    root = ET.parse(PACKAGE_ROOT / "mjcf" / "thunderv4.xml").getroot()
    referenced = {
        mesh.get("file")
        for mesh in root.findall("./asset/mesh")
        if mesh.get("file") is not None
    }
    assert referenced == set(EXPECTED_TRIANGLES)
    assert all(
        _binary_stl_triangle_count(physics_meshes / name) <= 200_000
        for name in referenced
    )

    model = mujoco.MjModel.from_xml_path(
        str(PACKAGE_ROOT / "mjcf" / "thunderv4.xml")
    )
    assert model.nbody > 1
    assert model.nmesh == 21


def test_high_fidelity_visual_meshes_are_not_mujoco_collision_geometry() -> None:
    root = ET.parse(PACKAGE_ROOT / "mjcf" / "thunderv4.xml").getroot()
    mesh_geometries = [geom for geom in root.iter("geom") if geom.get("type") == "mesh"]
    assert len(mesh_geometries) == 21
    for geom in mesh_geometries:
        assert geom.get("contype") == "0"
        assert geom.get("conaffinity") == "0"
        assert geom.get("density") == "0"
