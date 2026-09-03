"""Contract tests for deterministic robot visual binding manifests."""

from __future__ import annotations

from pathlib import Path

import pytest
from sim.catalog import VisualBindingError, compile_robot_visual_manifest


def _write_package(tmp_path: Path, mjcf_text: str) -> Path:
    package_dir = tmp_path / "robot"
    (package_dir / "mjcf").mkdir(parents=True)
    (package_dir / "meshes").mkdir()
    (package_dir / "meshes" / "base.stl").write_text("solid base\nendsolid base\n", encoding="utf-8")
    (package_dir / "meshes" / "arm.stl").write_text("solid arm\nendsolid arm\n", encoding="utf-8")
    (package_dir / "robot.package.yaml").write_text(
        "\n".join(
            [
                "schema: lingtu.sim.robot-package.v1",
                "id: tracer_bot",
                "version: 1.0.0",
                "kind: robot",
                "physics:",
                "  mjcf: mjcf/robot.xml",
                "visual:",
                "  binding: RobotVisual:TracerBot",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (package_dir / "mjcf" / "robot.xml").write_text(mjcf_text, encoding="utf-8")
    return package_dir


def test_compiles_deterministic_robot_visual_manifest_v1(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """\
<mujoco model="tracer">
  <compiler meshdir="../meshes" />
  <asset>
    <material name="body_white" rgba="0.9 0.92 0.94 1" />
    <mesh name="arm_visual" file="arm.stl" scale="0.5 0.6 0.7" />
    <mesh name="base_visual" file="base.stl" />
  </asset>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <geom type="mesh" mesh="base_visual" group="1" />
      <body name="arm">
        <geom name="decorative_collision" type="box" size="1 1 1" />
        <geom type="mesh" mesh="arm_visual" material="body_white" group="1" pos="1 2 3" quat="0.5 0.5 0.5 0.5" />
      </body>
    </body>
  </worldbody>
</mujoco>
""",
    )

    manifest = compile_robot_visual_manifest(package_dir, package_dir / "mjcf" / "robot.xml").to_dict()
    repeat = compile_robot_visual_manifest(package_dir).to_dict()

    assert manifest == repeat
    assert manifest["schema"] == "lingtu.sim.robot-visual-manifest.v1"
    assert manifest["binding"] == "RobotVisual:TracerBot"
    assert "digest" not in manifest
    assert "sources" not in manifest
    assert [entry["body"] for entry in manifest["visuals"]] == ["arm", "base"]
    arm_mesh = next(
        entry
        for entry in manifest["visuals"]
        if entry["geometry"]["kind"] == "mesh" and entry["body"] == "arm"
    )
    assert arm_mesh == {
        "body": "arm",
        "body_frame_id": "arm",
        "visual_id": "arm_visual",
        "visual_frame_id": "arm/visual/arm_visual",
        "asset_key": "RobotVisual:TracerBot/arm/visual/arm_visual",
        "source_mesh": "meshes/arm.stl",
        "mesh": "arm_visual",
        "geom": None,
        "material": "body_white",
        "rgba": None,
        "material_rgba": [0.9, 0.92, 0.94, 1.0],
        "pos": [1.0, 2.0, 3.0],
        "quat": [0.5, 0.5, 0.5, 0.5],
        "scale": [0.5, 0.6, 0.7],
        "geometry": {
            "kind": "mesh",
            "source_mesh": "meshes/arm.stl",
            "mesh": "arm_visual",
            "scale": [0.5, 0.6, 0.7],
        },
    }
    base_mesh = next(entry for entry in manifest["visuals"] if entry["body"] == "base")
    assert base_mesh["source_mesh"] == "meshes/base.stl"
    assert base_mesh["pos"] == [0.0, 0.0, 0.0]
    assert base_mesh["quat"] == [1.0, 0.0, 0.0, 0.0]
    assert base_mesh["scale"] == [1.0, 1.0, 1.0]
    assert base_mesh["visual_id"] == "base_visual"


def test_package_contained_mjcf_uses_package_relative_visual_sources_under_canonical_catalog_path(
    tmp_path: Path,
) -> None:
    package_dir = tmp_path / "repository" / "sim" / "packages" / "robots" / "g005_robot" / "1.0.0"
    (package_dir / "source" / "meshes").mkdir(parents=True)
    (package_dir / "robot.package.yaml").write_text(
        "\n".join(
            [
                "schema: lingtu.sim.robot-package.v1",
                "id: g005_robot",
                "version: 1.0.0",
                "kind: robot",
                "physics:",
                "  mjcf: source/robot.xml",
                "visual:",
                "  binding: RobotVisual:G005",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (package_dir / "source" / "meshes" / "body.stl").write_text("solid body\nendsolid body\n", encoding="utf-8")
    (package_dir / "source" / "robot.xml").write_text(
        """<mujoco>
  <compiler meshdir="meshes" />
  <asset><mesh name="body_visual" file="body.stl" /></asset>
  <worldbody><body name="base_link"><geom type="mesh" mesh="body_visual" /></body></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )

    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    assert manifest["package"]["manifest"] == "robot.package.yaml"
    assert manifest["mjcf"] == "source/robot.xml"
    assert manifest["visuals"][0]["source_mesh"] == "source/meshes/body.stl"


def test_compiles_thunderv4_package_without_hardcoded_body_count() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "doso" / "thunder_v4"

    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    repeat = compile_robot_visual_manifest(package_dir).to_dict()

    assert manifest == repeat
    assert manifest["binding"] == "RobotVisual:ThunderV4"
    assert len(manifest["visuals"]) == 21
    assert {entry["body"] for entry in manifest["visuals"]} >= {"base_link", "FR_hip", "camera2_link"}
    assert all(entry["body_frame_id"] == entry["body"] for entry in manifest["visuals"])
    assert all(entry["asset_key"].startswith("RobotVisual:ThunderV4/") for entry in manifest["visuals"])
    assert all(entry["material_rgba"] is not None for entry in manifest["visuals"])
    assert len({entry["visual_frame_id"] for entry in manifest["visuals"]}) == len(manifest["visuals"])
    mesh_visuals = [entry for entry in manifest["visuals"] if entry["geometry"]["kind"] == "mesh"]
    assert mesh_visuals == manifest["visuals"]
    assert {entry["visual_id"] for entry in mesh_visuals} == {entry["mesh"] for entry in mesh_visuals}
    assert all("source_sha256" not in entry for entry in mesh_visuals)
    assert all(not Path(entry["source_mesh"]).is_absolute() for entry in mesh_visuals)


def test_mesh_content_is_not_fingerprinted_in_visual_manifest(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )

    before = compile_robot_visual_manifest(package_dir).to_dict()
    (package_dir / "meshes" / "base.stl").write_text("solid changed\nendsolid changed\n", encoding="utf-8")
    after = compile_robot_visual_manifest(package_dir).to_dict()

    assert before == after


def test_visual_mesh_root_can_override_mjcf_mesh_sources(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )
    visual_mesh_root = package_dir / "visual" / "meshes"
    visual_mesh_root.mkdir(parents=True)
    (visual_mesh_root / "base.stl").write_bytes(b"high-fidelity visual mesh\n")
    manifest_path = package_dir / "robot.package.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            "  binding: RobotVisual:TracerBot\n",
            "  binding: RobotVisual:TracerBot\n  source_mesh_root: visual/meshes\n",
        ),
        encoding="utf-8",
    )

    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    visual = manifest["visuals"][0]

    assert visual["source_mesh"] == "visual/meshes/base.stl"
    assert visual["geometry"]["source_mesh"] == "visual/meshes/base.stl"
    assert "source_sha256" not in visual
    assert "source_sha256" not in visual["geometry"]


def test_visual_mesh_root_fails_closed_on_escape_or_missing_override(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )
    manifest_path = package_dir / "robot.package.yaml"
    original = manifest_path.read_text(encoding="utf-8")
    manifest_path.write_text(
        original.replace(
            "  binding: RobotVisual:TracerBot\n",
            "  binding: RobotVisual:TracerBot\n  source_mesh_root: ../outside\n",
        ),
        encoding="utf-8",
    )
    with pytest.raises(VisualBindingError, match="escapes robot package"):
        compile_robot_visual_manifest(package_dir)

    (package_dir / "visual" / "meshes").mkdir(parents=True)
    manifest_path.write_text(
        original.replace(
            "  binding: RobotVisual:TracerBot\n",
            "  binding: RobotVisual:TracerBot\n  source_mesh_root: visual/meshes\n",
        ),
        encoding="utf-8",
    )
    with pytest.raises(VisualBindingError, match="does not exist"):
        compile_robot_visual_manifest(package_dir)


def test_rejects_missing_and_escaping_mesh_assets(tmp_path: Path) -> None:
    missing_package = _write_package(
        tmp_path / "missing",
        """\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="missing_visual" file="missing.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="missing_visual" /></body></worldbody>
</mujoco>
""",
    )
    with pytest.raises(VisualBindingError, match="does not exist"):
        compile_robot_visual_manifest(missing_package)

    escaping_package = _write_package(
        tmp_path / "escaping",
        """\
<mujoco>
  <compiler meshdir="../../outside" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )
    with pytest.raises(VisualBindingError, match="escapes robot package"):
        compile_robot_visual_manifest(escaping_package)


def test_rejects_duplicate_named_bodies(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody>
    <body name="base"><geom type="mesh" mesh="base_visual" /></body>
    <body name="base"><geom type="mesh" mesh="base_visual" /></body>
  </worldbody>
</mujoco>
""",
    )

    with pytest.raises(VisualBindingError, match="duplicate named body 'base'"):
        compile_robot_visual_manifest(package_dir)


def test_rejects_unnamed_visual_body(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body><geom type="mesh" mesh="base_visual" /></body></worldbody>
</mujoco>
""",
    )

    with pytest.raises(VisualBindingError, match="supported visual geom must belong to a named body"):
        compile_robot_visual_manifest(package_dir)


@pytest.mark.parametrize(
    "orientation",
    [
        'euler="1 2 3"',
        'axisangle="1 0 0 1"',
        'xyaxes="1 0 0 0 1 0"',
        'zaxis="0 0 1"',
    ],
)
def test_rejects_unsupported_geom_orientation(tmp_path: Path, orientation: str) -> None:
    package_dir = _write_package(
        tmp_path,
        f"""\
<mujoco>
  <compiler meshdir="../meshes" />
  <asset><mesh name="base_visual" file="base.stl" /></asset>
  <worldbody><body name="base"><geom type="mesh" mesh="base_visual" {orientation} /></body></worldbody>
</mujoco>
""",
    )

    with pytest.raises(VisualBindingError, match="unsupported orientation attribute"):
        compile_robot_visual_manifest(package_dir)


def test_compiles_omni_cart_primitive_visual_manifest() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    package_dir = repo_root / "sim" / "packages" / "robots" / "omni_cart"

    manifest = compile_robot_visual_manifest(package_dir).to_dict()
    assert manifest["binding"] == "RobotVisual:OmniCart"
    assert {entry["geometry"]["primitive"] for entry in manifest["visuals"]} == {"box", "cylinder"}
    assert all("rgba" in entry and "material_rgba" in entry for entry in manifest["visuals"])
    assert all("instance_id" not in entry for entry in manifest["visuals"])


def test_rejects_fromto_and_omitted_geom_type(tmp_path: Path) -> None:
    fromto_package = _write_package(
        tmp_path / "fromto",
        """
<mujoco><worldbody><body name="base">
  <geom name="bar" type="capsule" size="0.1" fromto="0 0 0 0 0 1" />
</body></worldbody></mujoco>
""",
    )
    with pytest.raises(VisualBindingError, match="unsupported fromto"):
        compile_robot_visual_manifest(fromto_package)

    omitted_type_package = _write_package(
        tmp_path / "omitted_type",
        """
<mujoco><worldbody><body name="base"><geom name="implicit" size="0.1" /></body></worldbody></mujoco>
""",
    )
    with pytest.raises(VisualBindingError, match=r"geom\.type must be explicit"):
        compile_robot_visual_manifest(omitted_type_package)


def test_filters_hidden_and_collision_helper_primitives_from_mixed_model(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco>
  <compiler meshdir="../meshes" />
  <asset>
    <mesh name="base_visual" file="base.stl" />
    <material name="hidden" rgba="1 1 1 0" />
  </asset>
  <worldbody><body name="base">
    <geom type="mesh" mesh="base_visual" group="1" />
    <geom name="rgba_hidden" type="box" size="1 1 1" rgba="1 1 1 0" contype="0" conaffinity="0" />
    <geom name="material_hidden" type="sphere" size="1" material="hidden" contype="0" conaffinity="0" />
    <geom name="group_hidden" type="box" size="1 1 1" group="3" contype="0" conaffinity="0" />
    <geom name="collision" type="box" size="1 1 1" contype="1" conaffinity="1" />
    <geom name="marker" type="sphere" size="0.2" contype="0" conaffinity="0" />
  </body></worldbody>
</mujoco>
""",
    )

    manifest = compile_robot_visual_manifest(package_dir).to_dict()

    assert [entry["visual_id"] for entry in manifest["visuals"]] == ["base_visual", "marker"]


def test_rejects_default_inherited_visual_fields(tmp_path: Path) -> None:
    package_dir = _write_package(
        tmp_path,
        """
<mujoco>
  <default><default class="visual"><geom pos="1 2 3" /></default></default>
  <worldbody><body name="base"><geom class="visual" type="sphere" size="0.2" /></body></worldbody>
</mujoco>
""",
    )

    with pytest.raises(VisualBindingError, match="default-inherited visual fields"):
        compile_robot_visual_manifest(package_dir)
