# ruff: noqa: S101

from __future__ import annotations

import json
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from sim.catalog.importers import CatalogPromoter, ImportCode, ImportDraft, ImportFailure, RobotImporter, validate_robot_package
from sim.catalog.importers.contracts import file_records

REPO_ROOT = Path(__file__).resolve().parents[2]
TEST_ROOT = REPO_ROOT / "build" / "robot-importer-tests"


def _case_root(name: str) -> Path:
    root = TEST_ROOT / name
    shutil.rmtree(root, ignore_errors=True)
    root.mkdir(parents=True)
    return root


def _source(tmp_path: Path, xml: str, *, license_text: str = "Test license\n") -> Path:
    root = tmp_path / "source"
    root.mkdir()
    (root / "LICENSE.txt").write_text(license_text, encoding="utf-8")
    (root / "robot.xml").write_text(xml, encoding="utf-8")
    return root


def _primitive_xml(*, geom_name: str = "body_visual", angle: str = "radian") -> str:
    return f"""<mujoco model="primitive_bot">
  <compiler angle="{angle}" autolimits="true"/>
  <worldbody>
    <body name="base_link">
      <joint name="floating_base_joint" type="free"/>
      <geom name="{geom_name}" type="box" size="0.2 0.1 0.05" pos="0 0 0.05" rgba="0.2 0.4 0.6 1"/>
      <site name="imu" pos="0 0 0.1" size="0.001"/>
    </body>
  </worldbody>
</mujoco>
"""


def _mesh_xml() -> str:
    return """<mujoco model="mesh_bot">
  <compiler angle="radian" autolimits="true" meshdir="meshes"/>
  <asset>
    <mesh name="body_mesh" file="body.stl"/>
  </asset>
  <worldbody>
    <body name="base_link">
      <joint name="floating_base_joint" type="free"/>
      <geom name="body_visual" mesh="body_mesh" rgba="0.2 0.4 0.6 1"/>
      <site name="imu" pos="0 0 0.1" size="0.001"/>
    </body>
  </worldbody>
</mujoco>
"""


def _valid_binary_stl() -> bytes:
    triangle = "<12fH"
    triangles = [
        (0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0),
        (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0),
        (0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0),
        (0.577, 0.577, 0.577, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0),
    ]
    return (
        b"LingTu test STL".ljust(80, b"\0")
        + struct.pack("<I", len(triangles))
        + b"".join(struct.pack(triangle, *values) for values in triangles)
    )


def _valid_skin() -> bytes:
    return b"".join(
        (
            struct.pack("<4i", 3, 0, 1, 1),
            struct.pack("<9f", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            struct.pack("<3i", 0, 1, 2),
            b"base_link\0".ljust(40, b"\0"),
            struct.pack("<3f4fi", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 3),
            struct.pack("<3i", 0, 1, 2),
            struct.pack("<3f", 1.0, 1.0, 1.0),
        )
    )


def _request(source: Path, **overrides: object) -> dict[str, object]:
    request: dict[str, object] = {
        "schema": "lingtu.sim.robot-import-request.v1",
        "id": "draft_bot",
        "version": "1.0.0",
        "source": str(source),
        "source_format": "mjcf",
        "source_model": "robot.xml",
        "units": {"length": "m", "angle": "radian"},
        "provenance": {
            "owner": "LingTu Test",
            "license": "Test-Only",
            "license_file": "LICENSE.txt",
            "source_uri": "file://test",
        },
        "physics": {"attach_root": "base_link", "root_joint": "floating_base_joint"},
        "visual": {"binding": "RobotVisual:DraftBot"},
        "semantic": {"class": "wheeled_robot"},
        "frames": [{"name": "base_link", "role": "body"}, {"name": "imu", "role": "sensor_mount"}],
        "interfaces": {"state": ["lingtu.sim.base-state.v1"], "command": ["lingtu.sim.base-velocity.v1"]},
        "defaults": {"controller": None, "sensor_rig": None},
        "declared_capabilities": {"locomotion": ["drive"], "sensor_mounts": ["imu"]},
    }
    request.update(overrides)
    return request


def _code(draft: ImportDraft) -> str:
    diagnostic = draft.diagnostics[0]
    code = diagnostic.code
    return code.value if hasattr(code, "value") else str(code)


def test_existing_omnicart_and_thunder_validate_through_robot_importer() -> None:
    assert (
        validate_robot_package(REPO_ROOT / "sim/packages/robots/omni_cart", repo_root=REPO_ROOT)["package"]["id"]
        == "omni_cart"
    )
    assert (
        validate_robot_package(REPO_ROOT / "sim/packages/robots/doso/thunder_v4", repo_root=REPO_ROOT)["package"]["id"]
        == "thunderv4"
    )


def test_robot_import_rejects_session_global_mjcf_options_with_field_diagnostics() -> None:
    tmp_path = _case_root("global-options")
    xml = _primitive_xml().replace(
        '  <compiler angle="radian" autolimits="true"/>',
        '  <compiler angle="radian" autolimits="true"/>\n  <option timestep="0.001" solver="PGS" iterations="40"/>',
    )
    source = _source(tmp_path, xml)

    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "quarantined"
    diagnostic = draft.diagnostics[0]
    assert _code(draft) == "SIMIMPORT_GLOBAL_PHYSICS_OWNERSHIP"
    assert diagnostic.context == "robot.mjcf.option"
    assert diagnostic.details == {
        "fields": ["iterations", "solver", "timestep"],
        "required_policy": "inherit_session",
    }


def test_robot_import_emits_explicit_session_option_inheritance() -> None:
    tmp_path = _case_root("inherit-session")
    source = _source(tmp_path, _primitive_xml())

    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "qualified"
    assert draft.manifest_path is not None
    assert b"\r\n" not in draft.manifest_path.read_bytes()
    manifest = yaml.safe_load(draft.manifest_path.read_text(encoding="utf-8"))
    assert manifest["physics"]["global_options"] == "inherit_session"


def test_static_robot_import_does_not_qualify_declared_capabilities() -> None:
    tmp_path = _case_root("static-capabilities")
    source = _source(tmp_path, _primitive_xml())

    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "qualified"
    assert draft.qualification_path is not None
    qualification = json.loads(draft.qualification_path.read_text(encoding="utf-8"))
    assert qualification["qualified_capabilities"] == {}


def test_primitive_mjcf_import_returns_qualified_draft_and_is_byte_stable() -> None:
    tmp_path = _case_root("primitive")
    source = _source(tmp_path, _primitive_xml())
    importer = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports")
    first = importer.import_robot(_request(source))
    assert first.package_root is not None
    first_files = {
        path.relative_to(first.package_root).as_posix(): path.read_bytes()
        for path in first.package_root.rglob("*")
        if path.is_file()
    }

    second = importer.import_robot(_request(source))
    assert second.package_root is not None
    second_files = {
        path.relative_to(second.package_root).as_posix(): path.read_bytes()
        for path in second.package_root.rglob("*")
        if path.is_file()
    }

    assert first.state == "qualified"
    assert second.state == "qualified"
    assert first_files == second_files
    assert (first.package_root / "source/robot.xml").is_file()
    assert first.qualification_path is not None
    assert first.provenance_path is not None
    qualification = json.loads(first.qualification_path.read_text(encoding="utf-8"))
    assert qualification["checks"][0]["status"] == "passed"
    assert qualification["package"] == {"kind": "robot", "id": "draft_bot", "version": "1.0.0"}
    assert qualification["provenance"] == {"path": "provenance/robot.provenance.json"}


def test_robot_import_promotes_and_rejects_unknown_qualification_field() -> None:
    tmp_path = _case_root("promotion")
    source = _source(tmp_path, _primitive_xml())
    importer = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports")
    draft = importer.import_robot(_request(source))
    assert draft.state == "qualified"
    assert draft.qualification_path is not None

    qualification = json.loads(draft.qualification_path.read_text(encoding="utf-8"))
    assert qualification["package"] == {"kind": "robot", "id": "draft_bot", "version": "1.0.0"}
    assert qualification["provenance"]["path"] == "provenance/robot.provenance.json"

    promoter = CatalogPromoter(tmp_path)
    result = promoter.promote(draft)
    assert result.package_root.is_dir()

    qualification["package"]["unused"] = True
    draft.qualification_path.write_text(json.dumps(qualification), encoding="utf-8")
    with pytest.raises(ImportFailure, match="invalid fields"):
        promoter.promote(draft)
    published = json.loads(result.qualification_path.read_text(encoding="utf-8"))
    assert published["package"] == {"kind": "robot", "id": "draft_bot", "version": "1.0.0"}


def test_promoted_package_contained_visual_projection_resolves_in_fresh_catalog_and_composer() -> None:
    from sim.catalog import CatalogResolver, SimCatalog
    from sim.catalog.composer import SessionComposer

    tmp_path = _case_root("promoted-relocatable-visual")
    shutil.copytree(REPO_ROOT / "sim" / "packages" / "worlds", tmp_path / "sim" / "packages" / "worlds")
    source = _source(tmp_path, _primitive_xml())
    request = _request(source, id="g005_robot", visual={"binding": "RobotVisual:G005"})
    importer = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports")
    promoter = CatalogPromoter(tmp_path)

    first = importer.import_robot(request)
    first_result = promoter.promote(first)
    second = importer.import_robot(request)
    second_result = promoter.promote(second)

    assert first.state == "qualified"
    assert second.state == "qualified"
    assert first_result == second_result
    assert first_result.package_root == tmp_path / "sim" / "packages" / "robots" / "g005_robot"
    assert {
        path.relative_to(first_result.package_root).as_posix(): path.read_bytes()
        for path in first_result.package_root.rglob("*")
        if path.is_file()
    } == {
        path.relative_to(second_result.package_root).as_posix(): path.read_bytes()
        for path in second_result.package_root.rglob("*")
        if path.is_file()
    }

    projection = json.loads(
        (first_result.package_root / "visual" / "robot.visual-projection.json").read_text(encoding="utf-8")
    )
    assert projection["mjcf"]["path"] == "source/robot.xml"

    catalog = SimCatalog.from_repository(tmp_path)
    inspected = catalog.inspect_package("g005_robot@1.0.0", kind="robot")
    validated = catalog.validate_package("g005_robot@1.0.0", kind="robot")
    assert inspected["package"]["ref"] == "g005_robot@1.0.0"
    assert validated["catalog_valid"] is True
    assert validated["diagnostics"] == []

    intent = {
        "schema": "lingtu.sim.session-intent.v1",
        "session": {
            "session_id": "g005_visual_projection",
            "mujoco_version": "3.10.0",
            "seed": 5,
            "world": "open_field@1.0.0",
            "robots": [
                {
                    "instance_id": "g005_01",
                    "package": "g005_robot@1.0.0",
                    "spawn": {
                        "position_m": [0.0, 0.0, 0.0],
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    },
                }
            ],
            "runtime": {
                "backend": "mujoco",
                "mode": "preview",
                "required_bindings": ["physics", "visual"],
            },
        },
    }
    resolver = CatalogResolver.from_repository(tmp_path)
    composed = SessionComposer(resolver, artifact_root=tmp_path / "bundles").compose(
        intent,
        output_dir=tmp_path / "bundles" / "g005",
    )
    resolved = CatalogResolver.from_repository(tmp_path).resolve(composed.session_spec_path)
    visual_projection = resolved.visual_plan["robots"][0]["projection"]

    assert visual_projection["schema"] == "lingtu.sim.robot-visual-projection.v1"
    assert visual_projection["path"] == "sim/packages/robots/g005_robot/visual/robot.visual-projection.json"
    assert resolved.visual_plan["robots"][0]["binding"] == "RobotVisual:G005"


def test_bad_request_units_are_quarantined() -> None:
    tmp_path = _case_root("bad-units")
    source = _source(tmp_path, _primitive_xml())
    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(
        _request(source, units={"length": "cm", "angle": "degree"})
    )

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.UNIT_AMBIGUOUS.value
    assert (draft.root / "quarantine/diagnostics.json").is_file()


def test_duplicate_mjcf_names_are_quarantined() -> None:
    tmp_path = _case_root("duplicate-names")
    source = _source(tmp_path, _primitive_xml(geom_name="base_link"))
    duplicate = (
        (source / "robot.xml")
        .read_text(encoding="utf-8")
        .replace(
            '<site name="imu"',
            '<body name="base_link"/><site name="imu"',
        )
    )
    (source / "robot.xml").write_text(duplicate, encoding="utf-8")

    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.MODEL_INVALID.value


def test_missing_license_is_quarantined() -> None:
    tmp_path = _case_root("missing-license")
    source = _source(tmp_path, _primitive_xml())
    (source / "LICENSE.txt").unlink()

    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.LICENSE_REQUIRED.value


def test_mesh_visual_requires_asset_index_or_converter() -> None:
    tmp_path = _case_root("mesh-no-index")
    source = _source(tmp_path, _mesh_xml())
    mesh_dir = source / "meshes"
    mesh_dir.mkdir()
    (mesh_dir / "body.stl").write_bytes(_valid_binary_stl())

    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.ASSET_MISSING.value


def test_urdf_import_rejects_without_injected_converter() -> None:
    tmp_path = _case_root("urdf-no-converter")
    source = tmp_path / "urdf"
    source.mkdir()
    (source / "LICENSE.txt").write_text("Test license\n", encoding="utf-8")
    (source / "robot.urdf").write_text("<robot name='draft_bot'/>", encoding="utf-8")

    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(
        _request(source, source_format="urdf", source_model="robot.urdf")
    )

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.CONVERTER_UNAVAILABLE.value


def test_reimport_reuses_published_root_without_deleting_user_content() -> None:
    tmp_path = _case_root("atomic-reuse")
    source = _source(tmp_path, _primitive_xml())
    importer = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports")
    first = importer.import_robot(_request(source))
    marker = first.root / "user-marker.txt"
    marker.write_text("preserve me", encoding="utf-8")

    second = importer.import_robot(_request(source))

    assert first.state == second.state == "qualified"
    assert first.root == second.root
    assert marker.read_text(encoding="utf-8") == "preserve me"
    assert not list((tmp_path / "imports" / "robot" / ".staging").glob("*"))


def test_reimport_recomputes_published_package_closure_instead_of_trusting_draft_json() -> None:
    tmp_path = _case_root("atomic-reuse-stale-package")
    source = _source(tmp_path, _primitive_xml())
    importer = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports")
    first = importer.import_robot(_request(source))
    assert first.package_root is not None
    (first.package_root / "source" / "robot.xml").write_text(
        _primitive_xml().replace("primitive_bot", "tampered_bot"),
        encoding="utf-8",
    )

    second = importer.import_robot(_request(source))

    assert second.state == "quarantined"
    assert _code(second) == ImportCode.PROMOTION_CONFLICT.value
    assert "tampered_bot" in (first.package_root / "source" / "robot.xml").read_text(encoding="utf-8")


def test_reimport_revalidates_published_qualification_evidence() -> None:
    tmp_path = _case_root("atomic-reuse-stale-evidence")
    source = _source(tmp_path, _primitive_xml())
    importer = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports")
    first = importer.import_robot(_request(source))
    assert first.qualification_path is not None
    evidence = first.qualification_path.parent / "evidence" / first.version / "mujoco-compile.json"
    evidence.write_text("{}", encoding="utf-8")

    second = importer.import_robot(_request(source))

    assert second.state == "quarantined"
    assert _code(second) == ImportCode.PROMOTION_CONFLICT.value


def test_draft_publication_does_not_use_overwriting_path_replace(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    tmp_path = _case_root("atomic-no-replace")
    source = _source(tmp_path, _primitive_xml())

    def reject_replace(_self: Path, _target: Path) -> None:
        raise AssertionError("draft publication must never use overwriting Path.replace")

    monkeypatch.setattr(Path, "replace", reject_replace)
    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "qualified"


def test_unexpected_compiler_runtime_error_is_reraised_after_staging_cleanup() -> None:
    tmp_path = _case_root("unexpected-runtime-error")
    source = _source(tmp_path, _primitive_xml())

    class UnexpectedCompiler:
        def __call__(self, mjcf: Path) -> object:
            del mjcf
            raise RuntimeError("injected compiler failure")

    importer = RobotImporter(
        REPO_ROOT,
        work_root=tmp_path / "imports",
        mujoco_compiler=UnexpectedCompiler(),
    )

    with pytest.raises(RuntimeError, match="injected compiler failure"):
        importer.import_robot(_request(source))
    assert not list((tmp_path / "imports" / "robot" / ".staging").glob("*"))


def test_mjcf_assetdir_cannot_redirect_mesh_reads_outside_source_root() -> None:
    tmp_path = _case_root("assetdir-escape")
    source = _source(
        tmp_path,
        _mesh_xml().replace('meshdir="meshes"', 'assetdir="../outside"'),
    )
    (tmp_path / "outside").mkdir()
    (tmp_path / "outside" / "body.stl").write_bytes(_valid_binary_stl())

    with pytest.raises(ImportFailure) as captured:
        RobotImporter(REPO_ROOT)._validate_mjcf_references(
            ET.parse(source / "robot.xml").getroot(),
            source / "robot.xml",
            source,
        )

    assert captured.value.code == ImportCode.UNSAFE_SOURCE


def test_explicit_meshdir_overrides_assetdir_and_closure_binds_resolved_asset() -> None:
    tmp_path = _case_root("assetdir-meshdir-override")
    source = _source(
        tmp_path,
        _mesh_xml().replace('meshdir="meshes"', 'assetdir="../unused" meshdir="meshes"'),
    )
    (source / "meshes").mkdir()
    mesh = source / "meshes" / "body.stl"
    mesh.write_bytes(_valid_binary_stl())

    closure = RobotImporter(REPO_ROOT)._validate_mjcf_references(
        ET.parse(source / "robot.xml").getroot(),
        source / "robot.xml",
        source,
    )

    assert closure == [{"path": "meshes/body.stl"}]


def test_mjcf_texturedir_applies_to_cubemap_face_file_attributes() -> None:
    tmp_path = _case_root("texturedir-cubemap-escape")
    xml = (
        _primitive_xml()
        .replace(
            'autolimits="true"',
            'autolimits="true" texturedir="../outside"',
        )
        .replace(
            "  <worldbody>",
            '  <asset><texture name="sky" type="cube" fileup="sky.png"/></asset>\n  <worldbody>',
        )
    )
    source = _source(tmp_path, xml)
    (tmp_path / "outside").mkdir()
    (tmp_path / "outside" / "sky.png").write_bytes(b"not read")

    with pytest.raises(ImportFailure) as captured:
        RobotImporter(REPO_ROOT)._validate_mjcf_references(
            ET.parse(source / "robot.xml").getroot(),
            source / "robot.xml",
            source,
        )

    assert captured.value.code == ImportCode.UNSAFE_SOURCE


def test_mjcf_strippath_is_applied_before_effective_meshdir_resolution() -> None:
    tmp_path = _case_root("meshdir-strippath")
    source = _source(
        tmp_path,
        _mesh_xml()
        .replace('autolimits="true"', 'autolimits="true" strippath="true"')
        .replace('file="body.stl"', 'file="nested/body.stl"'),
    )
    (source / "meshes").mkdir()
    mesh = source / "meshes" / "body.stl"
    mesh.write_bytes(_valid_binary_stl())

    closure = RobotImporter(REPO_ROOT)._validate_mjcf_references(
        ET.parse(source / "robot.xml").getroot(),
        source / "robot.xml",
        source,
    )

    assert closure == [{"path": "meshes/body.stl"}]


@pytest.mark.parametrize("asset_tag", ["hfield", "skin"])
def test_mjcf_hfield_and_skin_use_assetdir_as_default_meshdir(asset_tag: str) -> None:
    tmp_path = _case_root(f"assetdir-default-{asset_tag}")
    source = _source(
        tmp_path,
        _primitive_xml()
        .replace('autolimits="true"', 'autolimits="true" assetdir="assets"')
        .replace("  <worldbody>", f'  <asset><{asset_tag} name="surface" file="surface.bin"/></asset>\n  <worldbody>'),
    )
    (source / "assets").mkdir()
    asset = source / "assets" / "surface.bin"
    asset.write_bytes(b"asset")

    closure = RobotImporter(REPO_ROOT)._validate_mjcf_references(
        ET.parse(source / "robot.xml").getroot(),
        source / "robot.xml",
        source,
    )

    assert closure == [{"path": "assets/surface.bin"}]


def test_mjcf_hfield_uses_explicit_meshdir_and_rejects_actual_outside_compile_read() -> None:
    import mujoco

    tmp_path = _case_root("hfield-explicit-meshdir-outside")
    xml = (
        _primitive_xml()
        .replace(
            'autolimits="true"',
            'autolimits="true" assetdir="assets" meshdir="../outside"',
        )
        .replace(
            "  <worldbody>",
            '  <asset><hfield name="surface" file="surface.bin" '
            'content_type="image/vnd.mujoco.hfield" size="1 1 1 0.1"/></asset>\n  <worldbody>',
        )
    )
    source = _source(tmp_path, xml)
    (source / "assets").mkdir()
    (source / "assets" / "surface.bin").write_bytes(b"package decoy")
    (tmp_path / "outside").mkdir()
    outside = tmp_path / "outside" / "surface.bin"
    outside.write_bytes(struct.pack("<ii4f", 2, 2, 0.0, 0.25, 0.75, 1.0))

    model = mujoco.MjModel.from_xml_path(str(source / "robot.xml"))
    assert model.nhfield == 1
    assert list(model.hfield_data) == pytest.approx([0.0, 0.25, 0.75, 1.0])

    with pytest.raises(ImportFailure) as captured:
        RobotImporter(REPO_ROOT)._validate_mjcf_references(
            ET.parse(source / "robot.xml").getroot(),
            source / "robot.xml",
            source,
        )

    assert captured.value.code == ImportCode.UNSAFE_SOURCE


def test_mjcf_skin_uses_explicit_meshdir_and_rejects_actual_outside_compile_read() -> None:
    import mujoco

    tmp_path = _case_root("skin-explicit-meshdir-outside")
    xml = (
        _primitive_xml()
        .replace(
            'autolimits="true"',
            'autolimits="true" assetdir="assets" meshdir="../outside"',
        )
        .replace(
            "  <worldbody>",
            '  <asset><skin name="surface" file="surface.skn"/></asset>\n  <worldbody>',
        )
    )
    source = _source(tmp_path, xml)
    (source / "assets").mkdir()
    (source / "assets" / "surface.skn").write_bytes(b"package decoy")
    (tmp_path / "outside").mkdir()
    (tmp_path / "outside" / "surface.skn").write_bytes(_valid_skin())

    model = mujoco.MjModel.from_xml_path(str(source / "robot.xml"))
    assert model.nskin == 1

    with pytest.raises(ImportFailure) as captured:
        RobotImporter(REPO_ROOT)._validate_mjcf_references(
            ET.parse(source / "robot.xml").getroot(),
            source / "robot.xml",
            source,
        )

    assert captured.value.code == ImportCode.UNSAFE_SOURCE


def test_mjcf_strippath_does_not_make_absolute_asset_reference_safe() -> None:
    tmp_path = _case_root("strippath-absolute")
    source = _source(
        tmp_path,
        _mesh_xml()
        .replace('autolimits="true"', 'autolimits="true" strippath="true"')
        .replace('file="body.stl"', 'file="C:/outside/body.stl"'),
    )
    (source / "meshes").mkdir()
    (source / "meshes" / "body.stl").write_bytes(_valid_binary_stl())

    with pytest.raises(ImportFailure) as captured:
        RobotImporter(REPO_ROOT)._validate_mjcf_references(
            ET.parse(source / "robot.xml").getroot(),
            source / "robot.xml",
            source,
        )

    assert captured.value.code == ImportCode.UNSAFE_SOURCE


def test_unrecognized_mjcf_file_attribute_is_rejected_before_compile() -> None:
    tmp_path = _case_root("unknown-file-bearing-element")
    source = _source(
        tmp_path,
        _primitive_xml().replace(
            "  <worldbody>",
            '  <asset><model name="nested" file="../outside.xml"/></asset>\n  <worldbody>',
        ),
    )

    with pytest.raises(ImportFailure) as captured:
        RobotImporter(REPO_ROOT)._validate_mjcf_references(
            ET.parse(source / "robot.xml").getroot(),
            source / "robot.xml",
            source,
        )

    assert captured.value.code == ImportCode.UNSAFE_SOURCE


def test_import_identity_covers_request_source_provenance_and_package_digest() -> None:
    tmp_path = _case_root("identity")
    source = _source(tmp_path, _primitive_xml())
    importer = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports")
    draft = importer.import_robot(
        _request(
            source,
            description="identity-bound",
            compatibility={"runtime_abi": "lingtu.sim.robot.v1"},
        )
    )

    identity = json.loads((draft.root / "import-identity.json").read_text(encoding="utf-8"))
    assert identity["request"]["description"] == "identity-bound"
    assert identity["request"]["compatibility"] == {"runtime_abi": "lingtu.sim.robot.v1"}
    assert identity["request"]["provenance"]["license_file"] == "LICENSE.txt"
    assert identity["source_content"]["files"]
    assert draft.provenance_path is not None
    assert draft.package_root is not None
    assert draft.provenance_path.is_relative_to(draft.package_root)
    assert "provenance/robot.provenance.json" in {item.path for item in file_records(draft.package_root)}


def test_failed_compile_cannot_be_qualified() -> None:
    tmp_path = _case_root("compile-failure")
    source = _source(tmp_path, _primitive_xml())
    draft = RobotImporter(
        REPO_ROOT,
        work_root=tmp_path / "imports",
        mujoco_compiler=lambda _path: None,
    ).import_robot(_request(source))

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.QUALIFICATION_FAILED.value
    assert not (draft.root / "package").exists()


def test_commandable_actuators_must_be_named_and_joint_backed() -> None:
    tmp_path = _case_root("actuator-contract")
    source = _source(
        tmp_path,
        _primitive_xml().replace(
            "</mujoco>",
            '<actuator><motor joint="floating_base_joint"/></actuator></mujoco>',
        ),
    )
    draft = RobotImporter(REPO_ROOT, work_root=tmp_path / "imports").import_robot(_request(source))

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.MODEL_INVALID.value


def test_mesh_converter_requires_structured_evidence() -> None:
    tmp_path = _case_root("mesh-evidence")
    source = _source(tmp_path, _mesh_xml())
    mesh_dir = source / "meshes"
    mesh_dir.mkdir()
    (mesh_dir / "body.stl").write_bytes(_valid_binary_stl())

    class StringOnlyConverter:
        def convert(self, visual_manifest: object) -> dict[str, str]:
            visual = visual_manifest["visuals"][0]  # type: ignore[index]
            return {visual["asset_key"]: "/Game/DraftBot/Body"}  # type: ignore[index]

    draft = RobotImporter(
        REPO_ROOT,
        work_root=tmp_path / "imports",
        mesh_converter=StringOnlyConverter(),
    ).import_robot(_request(source))

    assert draft.state == "quarantined"
    assert _code(draft) == ImportCode.PROJECTION_INVALID.value


def test_package_mjcf_path_must_not_escape_before_model_read(tmp_path: Path) -> None:
    package_root = tmp_path / "package"
    package_root.mkdir()
    (tmp_path / "outside.xml").write_text(_primitive_xml(), encoding="utf-8")
    manifest = {
        "schema": "lingtu.sim.robot-package.v1",
        "id": "unsafe",
        "version": "1.0.0",
        "kind": "robot",
        "physics": {
            "mjcf": "../outside.xml",
            "attach_root": "base_link",
            "root_joint": "floating_base_joint",
        },
        "visual": {"binding": "RobotVisual:Unsafe", "projection": "visual/robot.visual-projection.json"},
        "semantic": {"class": "wheeled_robot"},
        "frames": [{"name": "base_link", "role": "body"}],
        "interfaces": {
            "state": ["lingtu.sim.base-state.v1"],
            "command": ["lingtu.sim.base-velocity.v1"],
        },
        "defaults": {"controller": None, "sensor_rig": None},
        "declared_capabilities": {},
    }
    (package_root / "robot.package.yaml").write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ImportFailure):
        RobotImporter(REPO_ROOT).validate_package(package_root)
