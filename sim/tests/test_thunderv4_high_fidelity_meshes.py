"""High-fidelity ThunderV4 visual mesh replacement contracts."""

from __future__ import annotations

import hashlib
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from sim.catalog import CatalogResolver, compile_robot_visual_manifest

REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / "sim" / "packages" / "robots" / "thunderv4" / "1.0.3"
PACKAGE_MANIFEST = PACKAGE_ROOT / "robot.package.yaml"
SOURCE_PROVENANCE = PACKAGE_ROOT / "provenance" / "mesh-source.provenance.json"
SOURCE_ARCHIVE_SHA256 = "84783c660aa33af4d8705bdac6628a2352661bec31ceab6111415231f80423db"

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

ROLLED_BACK_HEAD_MESHES = {
    "base_link.STL": "f04f6c32157a6314726e32014bb20de9fcf0f70eea439b00c8a99dc4434abdce",
    "camera1_Link.STL": "1d4b7196c7bdededb7586d1a2af0e20a1e0a466911f9d3f55fb8ce60ee9489fb",
    "lidar1_Link.STL": "6dce2c1be2acda38d283b023ef2c8ed3b9a067fba691d595b5bf81b001645388",
}


def _binary_stl_triangle_count(path: Path) -> int:
    with path.open("rb") as stream:
        stream.seek(80)
        raw_count = stream.read(4)
    assert len(raw_count) == 4
    count = struct.unpack("<I", raw_count)[0]
    assert path.stat().st_size == 84 + count * 50
    return count


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def test_high_fidelity_meshes_are_catalog_visible_and_content_bound() -> None:
    resolver = CatalogResolver.from_repository(REPO_ROOT)
    package = resolver.find_package("thunderv4@1.0.3", kind="robot")
    assert package.manifest_path == PACKAGE_MANIFEST

    compiled = compile_robot_visual_manifest(PACKAGE_ROOT).to_dict()
    compiled_sources = {
        Path(component["geometry"]["source_mesh"]).name: component["geometry"][
            "source_sha256"
        ]
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
        assert _sha256(path) == compiled_sources[name]

    provenance = json.loads(SOURCE_PROVENANCE.read_text(encoding="utf-8"))
    assert provenance["schema"] == "lingtu.sim.robot-mesh-source-provenance.v1"
    assert provenance["source_archive"]["sha256"] == SOURCE_ARCHIVE_SHA256
    assert provenance["composition"] == {
        "default_source": "source_archive",
        "overrides": [
            {
                "mesh": "base_link.STL",
                "reason": "user_requested_original_integrated_head_shell",
                "source_package": "thunderv4@1.0.3",
            },
            {
                "mesh": "camera1_Link.STL",
                "reason": "user_requested_original_front_head",
                "source_package": "thunderv4@1.0.3",
            },
            {
                "mesh": "lidar1_Link.STL",
                "reason": "user_requested_original_front_head",
                "source_package": "thunderv4@1.0.3",
            },
        ],
    }
    assert provenance["mesh_count"] == 21
    assert provenance["total_triangles"] == sum(EXPECTED_TRIANGLES.values())
    assert {
        item["path"]: item["triangle_count"] for item in provenance["meshes"]
    } == {f"visual/meshes/{name}": count for name, count in EXPECTED_TRIANGLES.items()}
    assert {
        name: _sha256(meshes / name) for name in ROLLED_BACK_HEAD_MESHES
    } == ROLLED_BACK_HEAD_MESHES


def test_mujoco_keeps_decoder_safe_low_poly_meshes() -> None:
    physics_meshes = PACKAGE_ROOT / "meshes"
    assert {path.name for path in physics_meshes.iterdir() if path.is_file()} == set(
        EXPECTED_TRIANGLES
    )
    assert all(
        _binary_stl_triangle_count(physics_meshes / name) <= 200_000
        for name in EXPECTED_TRIANGLES
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
