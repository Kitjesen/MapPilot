# ruff: noqa: S101

"""Contract tests for the fictional RWS-01 simulation payload package."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO_ROOT / "schemas" / "simulation" / "payload.v1.json"
PROJECTION_SCHEMA_PATH = (
    REPO_ROOT / "schemas" / "simulation" / "payload-visual.v1.json"
)
PACKAGE_ROOT = (
    REPO_ROOT / "sim" / "packages" / "payloads" / "fictional_rws_01" / "1.0.0"
)
MANIFEST_PATH = PACKAGE_ROOT / "payload.package.yaml"
PROJECTION_PATH = PACKAGE_ROOT / "visual" / "payload.visual-projection.json"
MJCF_PATH = PACKAGE_ROOT / "mjcf" / "fictional_rws_01.xml"

EXPECTED_MESHES = {
    "SM_RWS01_EOSensor",
    "SM_RWS01_Launcher",
    "SM_RWS01_MountBase",
    "SM_RWS01_RecoilHousing",
    "SM_RWS01_YawFrame",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _manifest() -> dict:
    return yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))


def test_payload_package_is_versioned_fictional_and_simulation_only() -> None:
    from sim.tests.test_sim_plan_schemas import _validate

    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    manifest = _manifest()

    _validate(manifest, schema, schema)
    assert schema["$id"] == "lingtu.sim.payload-package.v1"
    assert manifest["schema"] == schema["$id"]
    assert manifest["id"] == "fictional_rws_01"
    assert manifest["version"] == "1.0.0"
    assert manifest["kind"] == "payload"
    assert manifest["safety"] == {
        "mode": "simulation_only",
        "fictional": True,
        "real_world_calibration": "prohibited",
    }
    serialized = MANIFEST_PATH.read_text(encoding="utf-8").lower()
    assert "api_key" not in serialized
    assert "tsk_" not in serialized


def test_payload_source_asset_and_provenance_are_content_bound() -> None:
    manifest = _manifest()
    source = manifest["provenance"]["source_asset"]
    source_path = PACKAGE_ROOT / source["path"]
    report = manifest["provenance"]["conditioning_report"]
    report_path = PACKAGE_ROOT / report["path"]

    assert source_path.is_file()
    assert source_path.suffix == ".glb"
    assert source["sha256"] == _sha256(source_path)
    assert report_path.is_file()
    assert report["sha256"] == _sha256(report_path)
    assert manifest["provenance"]["generator"] == "tripo3d"
    assert manifest["provenance"]["source_task_id"] == (
        "2dd80649-65d8-4263-a702-f3d29f73e40c"
    )
    assert manifest["provenance"]["segmentation_task_id"] == (
        "74cd1171-28b7-4899-843d-c0f99b1ffada"
    )


def test_payload_mujoco_proxy_compiles_with_generic_mount_and_articulation_frames() -> None:
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))

    for name in ("payload_base", "yaw_frame", "pitch_frame"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0
    for name in ("yaw_joint", "pitch_joint"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
    for name in ("mount_interface", "muzzle_frame", "eo_optical_frame"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) >= 0

    assert model.ngeom >= 5
    assert all(int(kind) != int(mujoco.mjtGeom.mjGEOM_MESH) for kind in model.geom_type)
    assert model.body_mass[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload_base")
    ] > 0


def test_payload_visual_projection_uses_the_clean_pbr_runtime_assets() -> None:
    from sim.tests.test_sim_plan_schemas import _validate

    projection = json.loads(PROJECTION_PATH.read_text(encoding="utf-8"))
    schema = json.loads(PROJECTION_SCHEMA_PATH.read_text(encoding="utf-8"))

    _validate(projection, schema, schema)
    assert projection["schema"] == "lingtu.sim.payload-visual-projection.v1"
    assert projection["binding"] == "PayloadVisual:FictionalRWS01"
    assert projection["authority"] == "mujoco"
    assert projection["visual_policy"] == {
        "collision_enabled": "no_collision",
        "simulate_physics": False,
        "generate_overlap_events": False,
    }
    assert projection["material_contract"] == {
        "source_format": "gltf2",
        "material_count": 5,
        "required_channels": ["base_color", "normal", "metallic_roughness"],
    }
    body = {key: value for key, value in projection.items() if key != "digest"}
    assert projection["digest"] == hashlib.sha256(
        json.dumps(
            body,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    assert {component["mesh"] for component in projection["components"]} == EXPECTED_MESHES

    for component in projection["components"]:
        asset_path = component["unreal_asset"]
        assert asset_path.startswith(
            "/Game/RobotSim/Payloads/FictionalRWS01/Runtime/"
            "rws-01-v002-runtime/StaticMeshes/"
        )
        package_path, object_name = asset_path.rsplit(".", 1)
        assert package_path.rsplit("/", 1)[-1] == object_name


def test_payload_manifest_connects_mount_physics_visual_and_interfaces() -> None:
    manifest = _manifest()

    assert manifest["mount"] == {
        "required_parent_role": "payload_mount",
        "root_frame": "payload_base",
    }
    assert manifest["physics"] == {
        "mjcf": {
            "path": "mjcf/fictional_rws_01.xml",
            "sha256": _sha256(MJCF_PATH),
        },
        "attach_root": "payload_base",
        "global_options": "inherit_session",
        "authority": "mujoco",
        "collision_representation": "primitive_proxy",
    }
    assert manifest["visual"] == {
        "binding": "PayloadVisual:FictionalRWS01",
        "projection": {
            "path": "visual/payload.visual-projection.json",
            "sha256": _sha256(PROJECTION_PATH),
        },
        "authority": "mujoco",
        "ue_collision": "disabled",
    }
    assert {frame["name"] for frame in manifest["frames"]} == {
        "payload_base",
        "yaw_frame",
        "pitch_frame",
        "muzzle_frame",
        "eo_optical_frame",
    }
    assert manifest["interfaces"] == {
        "state": ["lingtu.sim.payload-state.v1"],
        "command": ["lingtu.sim.payload-aim-command.v1"],
    }
