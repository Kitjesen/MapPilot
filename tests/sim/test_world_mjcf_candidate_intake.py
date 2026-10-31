"""Public static-MJCF candidate intake, never world qualification."""

import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from sim.catalog import CatalogResolver
from sim.catalog.importers import CatalogPromoter, ImportCode, ImportFailure, WorldImporter


@pytest.fixture
def candidate_request(tmp_path):
    source = tmp_path / "input"
    source.mkdir()
    (source / "LICENSE.txt").write_text("project-owned fixture, not Fab license evidence\n", encoding="utf-8")
    (source / "world.xml").write_text(
        '<mujoco><compiler angle="radian"/>'
        '<option timestep="0.001" integrator="implicitfast" solver="Newton" iterations="100" gravity="0 0 -9.81"/>'
        '<worldbody><body name="machine" pos="0 0 3"><geom name="hot" type="box" size="1 1 .1"/></body>'
        '<geom name="unknown" type="sphere" pos="4 0 0" size=".2"/></worldbody></mujoco>',
        encoding="utf-8",
    )
    (source / "bindings.json").write_text(
        json.dumps(
            [
                {"source_id": "source/geometry/1", "component_source_id": "source/machine", "mjcf_geom": "hot"},
                {"source_id": "source/geometry/2", "component_source_id": "source/other", "mjcf_geom": "unknown"},
            ]
        ),
        encoding="utf-8",
    )
    (source / "assumptions.json").write_text('{"basis":"author_assumptions_not_field_measurements"}', encoding="utf-8")
    return {
        "package": {"id": "source_candidate", "version": "0.1.0"},
        "source": {
            "path": str(source),
            "provenance": {
                "owner": "LingTu tests",
                "license": "LicenseRef-Test",
                "license_file": "LICENSE.txt",
                "source_uri": "file://test-source-candidate",
            },
        },
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "mjcf": "world.xml",
        "source_bindings": "bindings.json",
        "visual": {"binding": "WorldVisual:SourceCandidate", "level": "/Game/SourceCandidate"},
        "measurements": {
            "thermal_surfaces": {"hot": {"temperature_k": 365, "emissivity": 0.9}},
            "semantic_labels": {"hot": {"class_id": 102, "instance_id": 2102}},
            "acoustic_sources": [
                {
                    "source_id": "engine",
                    "body_name": "machine",
                    "pressure_pa": [0, 0.1, 0],
                    "sample_rate_hz": 16000,
                    "start_sim_time_ns": 0,
                    "reference_distance_m": 0.1,
                }
            ],
        },
    }


def test_public_mjcf_intake_preserves_source_and_returns_nonpromotable_candidate(tmp_path, candidate_request):
    source = Path(candidate_request["source"]["path"])
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    draft = WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=tmp_path / "draft")
    assert draft.state == "candidate"
    assert draft.qualification_path is None
    assert draft.provenance_path.is_file()
    assert {p.name: p.read_bytes() for p in source.iterdir()} == before
    for name, content in before.items():
        assert (draft.package_root / "source" / name).read_bytes() == content
    manifest = yaml.safe_load(draft.manifest_path.read_text(encoding="utf-8"))
    assert manifest["measurements"] == candidate_request["measurements"]
    assert "projection" not in manifest["visual"]
    assert not (draft.package_root / "visual").exists()
    assert not (tmp_path / "sim/packages").exists()
    provenance = json.loads(draft.provenance_path.read_text(encoding="utf-8"))
    assert provenance["provenance"]["owner"] == "LingTu tests"
    assert provenance["source_bindings"] == "source/bindings.json"
    assert provenance["state"] == "candidate"
    assert provenance["pending"] == ["visual_alignment", "navigation_spawn", "world_qualification"]
    assert CatalogResolver(tmp_path, [draft.package_root]).find_package(draft.reference).data == manifest
    assert json.loads((draft.root / "draft.json").read_text())["state"] == "candidate"
    with pytest.raises(ImportFailure) as failure:
        CatalogPromoter(tmp_path).promote(draft)
    assert failure.value.code == ImportCode.PROMOTION_INVALID


@pytest.mark.parametrize(
    "case",
    [
        "empty_bindings",
        "duplicate_source_id",
        "duplicate_geom",
        "missing_geom",
        "nonexistent_geom",
        "missing_component_id",
        "unnamed_geom",
        "empty_measurements",
        "missing_thermal",
        "empty_acoustic",
        "dynamic_joint",
        "fused_bodies",
        "include",
    ],
)
def test_candidate_rejects_incomplete_bindings_measurements_and_nonstatic_input(tmp_path, candidate_request, case):
    source = Path(candidate_request["source"]["path"])
    bindings = json.loads((source / "bindings.json").read_text())
    xml = (source / "world.xml").read_text()
    if case == "empty_bindings":
        bindings = []
    elif case == "duplicate_source_id":
        bindings[1]["source_id"] = bindings[0]["source_id"]
    elif case == "duplicate_geom":
        bindings.append(dict(bindings[0], source_id="another-source"))
    elif case == "missing_geom":
        bindings.pop()
    elif case == "nonexistent_geom":
        bindings[1]["mjcf_geom"] = "absent"
    elif case == "missing_component_id":
        del bindings[0]["component_source_id"]
    elif case == "unnamed_geom":
        xml = xml.replace('name="unknown" ', "")
    elif case == "empty_measurements":
        candidate_request["measurements"] = {}
    elif case == "missing_thermal":
        del candidate_request["measurements"]["thermal_surfaces"]
    elif case == "empty_acoustic":
        candidate_request["measurements"]["acoustic_sources"] = []
    elif case == "dynamic_joint":
        xml = xml.replace('<body name="machine" pos="0 0 3">', '<body name="machine" pos="0 0 3"><freejoint/>')
    elif case == "fused_bodies":
        xml = xml.replace('angle="radian"', 'angle="radian" fusestatic="true"')
    elif case == "include":
        (source / "part.xml").write_text('<mujoco><geom name="extra" type="sphere" size=".1"/></mujoco>')
        bindings.append({"source_id": "third-source", "component_source_id": "third-component", "mjcf_geom": "extra"})
        xml = xml.replace("</worldbody>", '<include file="part.xml"/></worldbody>')
    (source / "bindings.json").write_text(json.dumps(bindings))
    (source / "world.xml").write_text(xml)
    with pytest.raises(ImportFailure):
        WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=tmp_path / "rejected")
    assert not (tmp_path / "rejected").exists()


def test_candidate_cannot_be_written_to_active_catalog(tmp_path, candidate_request):
    target = tmp_path / "sim/packages/worlds/source_candidate/0.1.0"
    with pytest.raises(ImportFailure) as failure:
        WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=target)
    assert failure.value.code == ImportCode.PROMOTION_INVALID
    assert not (tmp_path / "sim/packages").exists()


def test_acoustic_source_requires_existing_device_geometry_not_an_empty_body(tmp_path, candidate_request):
    xml_path = Path(candidate_request["source"]["path"]) / "world.xml"
    xml_path.write_text(xml_path.read_text().replace("</worldbody>", '<body name="empty"/></worldbody>'))
    candidate_request["measurements"]["acoustic_sources"][0]["body_name"] = "empty"
    with pytest.raises(ImportFailure, match=r"source.*geom"):
        WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=tmp_path / "empty-source")


def test_candidate_retry_is_idempotent_but_changed_measurements_do_not_overwrite(tmp_path, candidate_request):
    importer = WorldImporter(tmp_path)
    target = tmp_path / "draft"
    first = importer.import_mjcf_candidate(candidate_request, draft_root=target)
    before = first.manifest_path.read_bytes()
    repeated = importer.import_mjcf_candidate(candidate_request, draft_root=target)
    assert repeated.to_dict() == first.to_dict()
    candidate_request["measurements"]["thermal_surfaces"]["hot"]["temperature_k"] = 400
    with pytest.raises(ImportFailure) as failure:
        importer.import_mjcf_candidate(candidate_request, draft_root=target)
    assert failure.value.code == ImportCode.PROMOTION_CONFLICT
    assert first.manifest_path.read_bytes() == before
    assert json.loads((target / "draft.json").read_text())["state"] == "candidate"


@pytest.mark.parametrize("generator", ["flexcomp", "replicate"])
def test_candidate_rejects_compiler_generated_dynamics_or_renamed_bindings(tmp_path, candidate_request, generator):
    import mujoco

    source = Path(candidate_request["source"]["path"])
    xml = (source / "world.xml").read_text()
    if generator == "flexcomp":
        xml = xml.replace(
            "</worldbody>",
            '<flexcomp name="soft" type="grid" dim="3" count="2 2 2" spacing=".1 .1 .1" mass="1"/></worldbody>',
        )
    else:
        xml = xml.replace('<body name="machine"', '<replicate count="1"><body name="machine"')
        xml = xml.replace("</body>", "</body></replicate>")
    model = mujoco.MjModel.from_xml_string(xml)
    assert model.ngeom == 2
    if generator == "flexcomp":
        assert model.nq == model.nv == 24
    else:
        assert model.geom("hot0").id >= 0 and model.body("machine0").id > 0
    (source / "world.xml").write_text(xml)
    with pytest.raises(ImportFailure) as failure:
        WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=tmp_path / "generated")
    assert failure.value.code == ImportCode.MODEL_INVALID
    assert not (tmp_path / "generated").exists()


def test_candidate_records_compiled_static_state_names_and_measurement_ids(tmp_path, candidate_request):
    import mujoco

    draft = WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=tmp_path / "verified")
    report = json.loads(draft.provenance_path.read_text())["mujoco_compile"]
    model = mujoco.MjModel.from_xml_path(str(draft.package_root / "source/world.xml"))
    assert report["nq"] == report["nv"] == model.nq == model.nv == 0
    assert set(report["geom_names"]) == {"hot", "unknown"}
    assert report["body_names"] == ["machine"]
    assert report["measurement_bindings"] == {
        "thermal_surfaces": {"hot": model.geom("hot").id},
        "semantic_labels": {"hot": model.geom("hot").id},
        "acoustic_sources": {"engine": model.body("machine").id},
    }


def test_truncated_candidate_xml_is_quarantined_as_model_invalid_without_staging_leak(tmp_path, candidate_request):
    xml_path = Path(candidate_request["source"]["path"]) / "world.xml"
    truncated = '<mujoco><worldbody><body name="machine">'
    xml_path.write_text(truncated)
    target = tmp_path / "truncated"
    with pytest.raises(ImportFailure) as failure:
        WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=target)
    assert failure.value.code == ImportCode.MODEL_INVALID
    assert not target.exists()
    assert list(tmp_path.glob(".truncated.staging-*")) == []
    quarantines = list((tmp_path / "sim/imports/world/quarantine").glob("*"))
    assert len(quarantines) == 1
    diagnostic = json.loads((quarantines[0] / "quarantine/failure.json").read_text())
    assert diagnostic["code"] == ImportCode.MODEL_INVALID.value
    assert xml_path.read_text() == truncated


@pytest.mark.parametrize("relative", [False, True])
def test_candidate_source_root_reparse_is_rejected_before_resolution(tmp_path, candidate_request, relative):
    source = Path(candidate_request["source"]["path"])
    before = {path.name: path.read_bytes() for path in source.iterdir()}
    junction = tmp_path / "source-junction"
    if os.name == "nt":
        import _winapi

        _winapi.CreateJunction(str(source), str(junction))
    else:
        junction.symlink_to(source, target_is_directory=True)
    candidate_request["source"]["path"] = junction.name if relative else str(junction)
    try:
        with pytest.raises(ImportFailure) as failure:
            WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=tmp_path / "redirected")
        assert failure.value.code == ImportCode.UNSAFE_SOURCE
        assert not (tmp_path / "redirected").exists()
        assert list(tmp_path.glob(".redirected.staging-*")) == []
        quarantines = list((tmp_path / "sim/imports/world/quarantine").glob("*"))
        assert len(quarantines) == 1
        assert {path.name: path.read_bytes() for path in source.iterdir()} == before
    finally:
        if os.name == "nt":
            junction.rmdir()
        else:
            junction.unlink()


@pytest.mark.parametrize("generated", ["flexcomp", "replicate"])
def test_public_candidate_checks_actual_compiler_result_not_only_xml_preflight(
    tmp_path, candidate_request, monkeypatch, generated
):
    import mujoco

    xml = (Path(candidate_request["source"]["path"]) / "world.xml").read_text()
    if generated == "flexcomp":
        xml = xml.replace(
            "</worldbody>",
            '<flexcomp name="soft" type="grid" dim="3" count="2 2 2" spacing=".1 .1 .1" mass="1"/></worldbody>',
        )
    else:
        xml = xml.replace('<body name="machine"', '<replicate count="1"><body name="machine"')
        xml = xml.replace("</body>", "</body></replicate>")
    compiled = mujoco.MjModel.from_xml_string(xml)
    # Substitute only the external compiler boundary; the returned model is real
    # MuJoCo state. The public request itself still contains static, named XML.
    monkeypatch.setattr(mujoco, "MjModel", SimpleNamespace(from_xml_path=lambda _: compiled))
    with pytest.raises(ImportFailure) as failure:
        WorldImporter(tmp_path).import_mjcf_candidate(candidate_request, draft_root=tmp_path / "compiled-mismatch")
    assert failure.value.code == ImportCode.MODEL_INVALID
    assert not (tmp_path / "compiled-mismatch").exists()
