"""Focused Robot Visual Projection v1 contract tests."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from sim.catalog import (
    VisualBindingError,
    VisualProjectionError,
    compile_robot_visual_manifest,
    compile_robot_visual_projection,
    validate_robot_visual_projection,
)
from sim.catalog.visual_projection import validate_robot_visual_projection_matches_manifest

from .test_visual_binding_manifest import _write_package


def _pretty_projection(document: dict) -> str:
    return json.dumps(document, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n"


def _schema() -> dict:
    path = Path(__file__).resolve().parents[2] / "sim" / "contracts" / "schemas" / "robot-visual.v1.json"
    return json.loads(path.read_text(encoding="utf-8"))


def test_mesh_projection_is_deterministic_and_schema_valid(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(
        manifest,
        {"base_visual": "/Game/Robots/Tracer/base_visual.base_visual"},
    ).to_dict()
    repeat = compile_robot_visual_projection(
        manifest,
        {"base_visual": "/Game/Robots/Tracer/base_visual.base_visual"},
    ).to_dict()

    assert projection == repeat
    assert projection["schema"] == "lingtu.sim.robot-visual-projection.v1"
    assert "digest" not in projection
    assert "manifest_digest" not in projection
    assert projection["components"][0]["geometry"]["kind"] == "mesh"
    assert projection["components"][0]["unreal"]["asset_path"].startswith("/Game/")
    assert projection["components"][0]["material"]["source"] == "compiler_default"
    assert projection["components"][0]["material"]["key"] is None
    assert "instance_id" not in json.dumps(projection)
    assert validate_robot_visual_projection(projection).to_dict() == projection

    jsonschema = pytest.importorskip("jsonschema")
    jsonschema.validate(projection, _schema())


def test_omni_cart_projection_compiles_primitives_without_asset_bindings() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    projection = compile_robot_visual_projection(manifest).to_dict()

    assert len(projection["components"]) == len(manifest["visuals"])
    assert {item["geometry"]["primitive"] for item in projection["components"]} == {"box", "cylinder"}
    assert all(item["unreal"]["asset_path"].startswith("/Engine/BasicShapes/") for item in projection["components"])
    assert all("material" in item for item in projection["components"])
    assert all("instance_id" not in item for item in projection["components"])


def test_projection_material_resolution_priority_and_pbr_contract(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco>
  <asset>
    <material name="named_red" rgba="0.9 0.1 0.2 1" />
    <material name="named_blue" rgba="0.1 0.2 0.9 1" />
  </asset>
  <worldbody><body name="base">
    <geom name="geom_rgba_wins" type="box" size="1 1 1" rgba="0.2 0.3 0.4 0.5" material="named_red" />
    <geom name="named_rgba" type="sphere" size="0.2" material="named_blue" />
    <geom name="default_rgba" type="cylinder" size="0.1 0.2" />
  </body></worldbody>
</mujoco>
""",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    projection = compile_robot_visual_projection(manifest).to_dict()
    materials = {
        component["visual_id"]: component["material"]
        for component in projection["components"]
    }

    assert materials["geom_rgba_wins"] == {
        "source": "mjcf_geom_rgba",
        "key": None,
        "pbr": {
            "base_color_rgba": [0.2, 0.3, 0.4, 0.5],
            "metallic": 0.0,
            "roughness": 0.65,
        },
    }
    assert materials["named_rgba"] == {
        "source": "mjcf_material_rgba",
        "key": "named_blue",
        "pbr": {
            "base_color_rgba": [0.1, 0.2, 0.9, 1.0],
            "metallic": 0.0,
            "roughness": 0.65,
        },
    }
    assert materials["default_rgba"]["source"] == "compiler_default"
    assert materials["default_rgba"]["key"] is None
    assert materials["default_rgba"]["pbr"]["base_color_rgba"][3] == 1.0
    assert validate_robot_visual_projection(projection).to_dict() == projection


def test_projection_validator_rejects_component_without_resolved_material(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco><worldbody><body name="base">
  <geom name="body" type="box" size="1 1 1" />
</body></worldbody></mujoco>
""",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()
    malformed = copy.deepcopy(projection)
    del malformed["components"][0]["material"]

    with pytest.raises(VisualProjectionError, match="material"):
        validate_robot_visual_projection(malformed)


@pytest.mark.parametrize(
    ("geom", "message"),
    [
        ('type="box" size="1 0 1"', "positive"),
        ('type="box" size="1 2"', "exactly 3"),
        ('type="ellipsoid" size="1 1 1"', "unsupported visual geom type"),
    ],
)
def test_rejects_malformed_or_unsupported_primitive(tmp_path: Path, geom: str, message: str) -> None:
    package_dir = _write_package(
        tmp_path,
        f"""
<mujoco><worldbody><body name="base"><geom name="shape" {geom} /></body></worldbody></mujoco>
""",
    )
    with pytest.raises(VisualBindingError, match=message):
        compile_robot_visual_manifest(package_dir)


def test_mesh_projection_fails_closed_without_cooked_asset_mapping(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    with pytest.raises(VisualProjectionError, match="no explicit cooked Unreal asset mapping"):
        compile_robot_visual_projection(manifest)


def test_thunder_projection_retains_only_canonical_mesh_visuals() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "doso" / "thunder_v4"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    bindings = {
        visual["mesh"]: f"/Game/Robots/ThunderV4/{visual['mesh']}.{visual['mesh']}"
        for visual in manifest["visuals"]
    }

    projection = compile_robot_visual_projection(manifest, bindings).to_dict()

    assert len(projection["components"]) == 21
    assert all(component["geometry"]["kind"] == "mesh" for component in projection["components"])
    assert validate_robot_visual_projection(projection).to_dict() == projection


def test_thunder_projection_uses_layered_black_robot_palette() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "doso" / "thunder_v4"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    bindings = {
        visual["mesh"]: f"/Game/Robots/ThunderV4/{visual['mesh']}.{visual['mesh']}"
        for visual in manifest["visuals"]
    }

    projection = compile_robot_visual_projection(manifest, bindings).to_dict()
    materials = {
        component["local_body_id"]: (
            component["material_key"],
            component["material"]["pbr"]["base_color_rgba"],
        )
        for component in projection["components"]
    }

    assert materials["base_link"] == (
        "body_shell_black",
        [0.006049, 0.006995, 0.008023, 1.0],
    )
    for leg in ("FL", "FR", "RL", "RR"):
        assert materials[f"{leg}_hip"] == (
            "joint_graphite",
            [0.009134, 0.01033, 0.011612, 1.0],
        )
        for link in ("thigh", "calf"):
            assert materials[f"{leg}_{link}"] == (
                "limb_graphite",
                [0.006995, 0.008023, 0.009134, 1.0],
            )
        assert materials[f"{leg}_foot"] == (
            "wheel_black",
            [0.001821, 0.002125, 0.002428, 1.0],
        )
    for sensor in ("camera1_link", "camera2_link", "lidar1_link", "lidar2_link"):
        assert materials[sensor] == (
            "sensor_black",
            [0.003677, 0.004391, 0.005182, 1.0],
        )

    assert all(
        max(component["material"]["pbr"]["base_color_rgba"][:3]) <= 0.012
        for component in projection["components"]
    )


@pytest.mark.parametrize(
    ("package_name", "bindings"),
    [
        ("doso/thunder_v4", "checked_in"),
        ("omni_cart", None),
    ],
)
def test_checked_in_robot_projections_are_valid_and_byte_identical_when_regenerated(
    package_name: str,
    bindings: str | None,
) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / package_name
    projection_path = package_dir / "visual" / "robot.visual-projection.json"
    checked_in = json.loads(projection_path.read_text(encoding="utf-8"))
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    asset_bindings = None
    if bindings == "checked_in":
        asset_bindings = {
            component["asset_key"]: component["unreal"]["asset_path"]
            for component in validate_robot_visual_projection(checked_in).to_dict()["components"]
            if component["geometry"]["kind"] == "mesh"
        }

    regenerated = compile_robot_visual_projection(manifest, asset_bindings).to_dict()

    assert validate_robot_visual_projection_matches_manifest(checked_in, manifest).to_dict() == checked_in
    assert _pretty_projection(regenerated) == projection_path.read_text(encoding="utf-8")


def test_capsule_projection_uses_unambiguous_mujoco_dimensions(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco><worldbody><body name="base">
  <geom name="bumper" type="capsule" size="0.2 0.5" />
</body></worldbody></mujoco>
""",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    projection = compile_robot_visual_projection(manifest).to_dict()
    unreal = projection["components"][0]["unreal"]

    assert unreal == {
        "representation": "component",
        "component_class": "/Script/Engine.CapsuleComponent",
        "radius_m": 0.2,
        "cylinder_half_length_m": 0.5,
        "capsule_half_height_m": 0.7,
        "dimensions_m": [0.4, 0.4, 1.4],
    }
    assert validate_robot_visual_projection(projection).to_dict() == projection


def test_projection_compiler_rejects_raw_mjcf_non_unit_quaternion(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco><worldbody><body name="base">
  <geom name="body" type="box" size="1 1 1" quat="2 0 0 0" />
</body></worldbody></mujoco>
""",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    with pytest.raises(VisualProjectionError, match="unit quaternion"):
        compile_robot_visual_projection(manifest)


@pytest.mark.parametrize(
    "asset_path",
    [
        "/Game/Robots/Bad Path.Mesh",
        "/Game/Robots/./Mesh.Mesh",
        "/Game/Robots//Mesh.Mesh",
        "/Game/Robots/../Mesh.Mesh",
    ],
)
def test_rejects_unsafe_cooked_asset_paths(tmp_path: Path, asset_path: str) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )
    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    with pytest.raises(VisualProjectionError, match="cooked /Game/ Unreal asset path"):
        compile_robot_visual_projection(manifest, {"base_visual": asset_path})


def test_projection_validator_rejects_geometry_unreal_mismatch() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()

    mismatched = copy.deepcopy(projection)
    box = next(
        component for component in mismatched["components"] if component["geometry"]["primitive"] == "box"
    )
    box["unreal"]["asset_path"] = "/Engine/BasicShapes/Sphere.Sphere"
    with pytest.raises(VisualProjectionError, match="does not agree"):
        validate_robot_visual_projection(mismatched)


def test_projection_validator_rejects_tampered_local_quaternion() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()

    tampered = copy.deepcopy(projection)
    tampered["components"][0]["local_transform"]["quaternion_wxyz"] = [2.0, 0.0, 0.0, 0.0]

    with pytest.raises(VisualProjectionError, match="unit quaternion"):
        validate_robot_visual_projection(tampered)


def test_projection_schema_rejects_impossible_quaternion_component() -> None:
    jsonschema = pytest.importorskip("jsonschema")
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()
    projection["components"][0]["local_transform"]["quaternion_wxyz"] = [2.0, 0.0, 0.0, 0.0]

    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(projection, _schema())


def test_projection_schema_rejects_material_additional_properties() -> None:
    jsonschema = pytest.importorskip("jsonschema")
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()
    projection["components"][0]["material"]["extra"] = True

    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(projection, _schema())


def test_projection_schema_rejects_component_without_resolved_material() -> None:
    jsonschema = pytest.importorskip("jsonschema")
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()
    del projection["components"][0]["material"]

    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(projection, _schema())


def test_projection_manifest_match_rejects_deleted_component() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()
    tampered = copy.deepcopy(projection)
    tampered["components"] = tampered["components"][1:]

    with pytest.raises(VisualProjectionError, match="freshly compiled visual manifest"):
        validate_robot_visual_projection_matches_manifest(tampered, manifest)


def test_projection_manifest_match_rejects_altered_manifest_field() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"
    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    projection = compile_robot_visual_projection(manifest).to_dict()
    tampered = copy.deepcopy(projection)
    tampered["components"][0]["body_frame_id"] += "_tampered"

    with pytest.raises(VisualProjectionError, match="freshly compiled visual manifest"):
        validate_robot_visual_projection_matches_manifest(tampered, manifest)
