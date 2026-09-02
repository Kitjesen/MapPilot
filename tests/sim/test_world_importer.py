# ruff: noqa: S101, S314
"""Contracts for deterministic world imports."""

from __future__ import annotations

import copy
import json
import struct
import threading
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest
import yaml

from sim.catalog.resolver import CatalogError, CatalogResolver
from sim.catalog.importers import ImportCode, ImportFailure, WorldImporter


def _source_tree(root: Path) -> Path:
    source = root / "source"
    source.mkdir()
    (source / "LICENSE.txt").write_text("project owned test asset\n", encoding="utf-8")
    samples = [0, 16_384, 32_768, 65_535]
    (source / "height.r16").write_bytes(struct.pack("<4H", *samples))
    (source / "mesh.obj").write_text(
        "o prop\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n",
        encoding="ascii",
    )
    return source


def _request(source: Path) -> dict:
    return {
        "schema": "lingtu.sim.world-import-request.v1",
        "package": {"id": "field", "version": "1.0.0", "description": "Imported test field"},
        "source": {
            "path": str(source),
            "provenance": {
                "owner": "LingTu tests",
                "license": "LicenseRef-Test",
                "license_file": "LICENSE.txt",
                "source_uri": "file://test",
                "third_party_assets": [],
            },
        },
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "heightmap": {
            "path": "height.r16",
            "width": 2,
            "height": 2,
            "extent_m": [2.0, 2.0],
            "elevation_min_m": -1.0,
            "elevation_max_m": 1.0,
        },
        "mesh": {"path": "mesh.obj", "collision": True},
        "visual": {"binding": "WorldVisual:ImportedField", "level": "/Game/RobotSim/Maps/ImportedField"},
        "spawn": {
            "position_m": [-1.0, 1.0, -1.0],
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "height_tolerance_m": 1e-6,
        },
        "entities": [
            {
                "entity_id": "rock_01",
                "entity_type": "prop",
                "authority": "mujoco",
                "initial_transform": {
                    "position_m": [0.0, 0.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
                "physics_proxy": "mujoco",
                "semantic_class": "rock",
                "collision": True,
            }
        ],
        "bounds": {"min_m": [-1.0, -1.0, -1.0], "max_m": [1.0, 1.0, 1.0]},
    }


def _quarantine_roots(repo_root: Path) -> list[Path]:
    return list((repo_root / "sim" / "imports" / "world" / "quarantine").glob("*"))


def test_world_importer_builds_qualified_catalog_world_with_internal_artifacts(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)

    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    assert draft.state == "qualified"
    assert draft.package_root == tmp_path / "draft" / "package"
    assert (draft.package_root / "world.xml").is_file()
    assert (draft.package_root / "artifacts" / "heightfield_f32.bin").is_file()
    assert (draft.package_root / "artifacts" / "heightfield_r16.png").is_file()
    assert (draft.package_root / "artifacts" / "terrain.obj").is_file()
    assert (draft.package_root / "visual" / "world.visual-projection.json").is_file()

    option = ET.parse(draft.package_root / "world.xml").getroot().find("option")
    assert option is not None
    assert option.attrib == {
        "gravity": "0 0 -9.81",
        "timestep": "0.002",
        "integrator": "RK4",
        "solver": "Newton",
        "iterations": "100",
    }

    manifest = yaml.safe_load((draft.package_root / "world.package.yaml").read_text(encoding="utf-8"))
    assert b"\r\n" not in (draft.package_root / "world.package.yaml").read_bytes()
    assert manifest["visual"] == {
        "binding": "WorldVisual:ImportedField",
        "level": "/Game/RobotSim/Maps/ImportedField",
        "projection": "visual/world.visual-projection.json",
    }
    assert manifest["provenance"] == manifest["content"]["provenance"]
    assert manifest["content"]["visual_projection"] == {"path": "visual/world.visual-projection.json"}
    content_records = [
        {"path": path.relative_to(draft.package_root).as_posix(), "size": path.stat().st_size}
        for path in draft.package_root.rglob("*")
        if path.is_file() and path.name != "world.package.yaml"
    ]
    content_records.sort(key=lambda item: item["path"])
    assert manifest["content"]["files"] == content_records
    record = CatalogResolver(tmp_path, (draft.package_root,)).find_package("field@1.0.0", kind="world")
    assert record.data["content"] == manifest["content"]
    qualification = json.loads(draft.qualification_path.read_text(encoding="utf-8"))
    assert qualification["package"] == {"kind": "world", "id": "field", "version": "1.0.0"}
    assert qualification["provenance"] == {"path": "provenance/world.provenance.json"}
    assert {check["id"] for check in qualification["checks"]} == {
        "schema",
        "mujoco_compile",
        "visual_projection",
        "entity_graph",
        "provenance",
    }
    assert qualification["checks"][0]["evidence"][0]["path"].startswith("evidence/1.0.0/")

    projection = json.loads(
        (draft.package_root / "visual" / "world.visual-projection.json").read_text(encoding="utf-8")
    )
    assert projection["units"] == {"length": "m", "up_axis": "Z", "handedness": "RH"}
    assert projection["terrain"]["physics_bounds_m"] == projection["terrain"]["visual_bounds_m"]
    assert projection["spawn_alignment"] == {"position_m": [-1.0, 1.0, -1.0], "aligned_to_heightmap": True}

    nrow, ncol, *samples = struct.unpack(
        "<ii4f", (draft.package_root / "artifacts" / "heightfield_f32.bin").read_bytes()
    )
    assert [nrow, ncol] == [2, 2]
    assert samples == pytest.approx([32_768 / 65_535, 1.0, 0.0, 16_384 / 65_535])


def test_world_importer_applies_explicit_1ms_physics_policy_to_mjcf_and_manifest(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    request["physics"] = {
        "timestep_s": 0.001,
        "integrator": "implicitfast",
        "solver": "cg",
        "iterations": 64,
        "gravity": [0.0, 0.0, -3.7],
    }

    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    manifest = yaml.safe_load(draft.manifest_path.read_text(encoding="utf-8"))
    policy = manifest["physics"]["global_policy"]
    assert policy == {
        "timestep_s": 0.001,
        "integrator": "implicitfast",
        "solver": "cg",
        "iterations": 64,
        "gravity_mps2": [0.0, 0.0, -3.7],
    }
    option = ET.parse(draft.package_root / "world.xml").getroot().find("option")
    assert option is not None
    assert option.attrib == {
        "gravity": "0 0 -3.7",
        "timestep": "0.001",
        "integrator": "implicitfast",
        "solver": "CG",
        "iterations": "64",
    }


@pytest.mark.parametrize(
    ("physics", "context"),
    [
        ({"timestep_s": 0.0}, "physics.timestep_s"),
        ({"timestep_s": -0.001}, "physics.timestep_s"),
        ({"integrator": "verlet"}, "physics.integrator"),
        ({"solver": "qp"}, "physics.solver"),
        ({"iterations": 0}, "physics.iterations"),
        ({"gravity": [0.0, 0.0]}, "physics.gravity"),
    ],
)
def test_world_importer_rejects_invalid_physics_policy_values(
    tmp_path: Path, physics: dict, context: str
) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    request["physics"] = physics

    with pytest.raises(ImportFailure) as exc_info:
        WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    assert exc_info.value.context == context
    assert any((path / "quarantine" / "failure.json").is_file() for path in _quarantine_roots(tmp_path))
    assert not (tmp_path / "draft" / "package").exists()


def test_world_importer_emits_compilable_absolute_hfield_datum(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    request["heightmap"].update({"elevation_min_m": 2.0, "elevation_max_m": 4.0})
    request["spawn"]["position_m"][2] = 2.0
    request["bounds"].update({"min_m": [-1.0, -1.0, 2.0], "max_m": [1.0, 1.0, 4.0]})
    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    root = ET.parse(draft.package_root / "world.xml").getroot()
    hfield = root.find("./asset/hfield")
    geom = root.find("./worldbody/geom")
    assert hfield is not None
    assert geom is not None
    assert hfield.attrib["content_type"] == "image/vnd.mujoco.hfield"
    assert [float(value) for value in geom.attrib["pos"].split()] == [0.0, 0.0, 2.0]
    size = [float(value) for value in hfield.attrib["size"].split()]
    assert size[2] == pytest.approx(2.0)
    assert size[3] >= 0.0
    assert size[3] != pytest.approx(-2.0)

    mujoco = pytest.importorskip("mujoco")
    mujoco.MjModel.from_xml_path(str(draft.package_root / "world.xml"))


def test_world_importer_mujoco_surface_uses_the_actual_sampled_elevation_range(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    numpy = pytest.importorskip("numpy")
    source = _source_tree(tmp_path)
    samples = (10_000, 11_000, 12_000, 13_000, 14_000, 15_000, 16_000, 17_000, 18_000)
    (source / "height.r16").write_bytes(struct.pack("<9H", *samples))
    request = _request(source)
    request.pop("mesh")
    request["heightmap"].update(
        {
            "width": 3,
            "height": 3,
            "elevation_min_m": -10.0,
            "elevation_max_m": 70.0,
        }
    )
    expected_min_m = -10.0 + min(samples) / 65_535 * 80.0
    expected_max_m = -10.0 + max(samples) / 65_535 * 80.0
    expected_center_m = -10.0 + samples[4] / 65_535 * 80.0
    request["spawn"]["position_m"] = [0.0, 0.0, expected_center_m]
    request["bounds"] = {"min_m": [-1.0, -1.0, -10.0], "max_m": [1.0, 1.0, 70.0]}

    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")
    model = mujoco.MjModel.from_xml_path(str(draft.package_root / "world.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hfield_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "field_heightfield")
    terrain_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "terrain_collision")
    hit_geom_id = numpy.array([-1], dtype=numpy.int32)
    ray_origin_z_m = 100.0
    distance_m = mujoco.mj_ray(
        model,
        data,
        numpy.array([0.0, 0.0, ray_origin_z_m]),
        numpy.array([0.0, 0.0, -1.0]),
        None,
        True,
        -1,
        hit_geom_id,
    )

    assert model.geom_pos[terrain_geom_id, 2] == pytest.approx(expected_min_m)
    assert model.hfield_size[hfield_id, 2] == pytest.approx(expected_max_m - expected_min_m)
    assert hit_geom_id[0] == terrain_geom_id
    assert ray_origin_z_m - distance_m == pytest.approx(expected_center_m, abs=1.0e-3)


def test_world_importer_flat_hfield_uses_explicit_positive_scale_without_height_drift(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    numpy = pytest.importorskip("numpy")
    source = _source_tree(tmp_path)
    sample = 30_000
    (source / "height.r16").write_bytes(struct.pack("<9H", *(sample for _ in range(9))))
    request = _request(source)
    request.pop("mesh")
    request["heightmap"].update(
        {
            "width": 3,
            "height": 3,
            "elevation_min_m": -10.0,
            "elevation_max_m": 70.0,
        }
    )
    expected_height_m = -10.0 + sample / 65_535 * 80.0
    request["spawn"]["position_m"] = [0.0, 0.0, expected_height_m]
    request["bounds"] = {"min_m": [-1.0, -1.0, -10.0], "max_m": [1.0, 1.0, 70.0]}

    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")
    alignment = json.loads((draft.package_root / "artifacts/alignment-report.json").read_text(encoding="utf-8"))
    normalization = alignment["physics"]["normalization"]
    model = mujoco.MjModel.from_xml_path(str(draft.package_root / "world.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hfield_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "field_heightfield")
    terrain_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "terrain_collision")
    hit_geom_id = numpy.array([-1], dtype=numpy.int32)
    ray_origin_z_m = 100.0
    distance_m = mujoco.mj_ray(
        model,
        data,
        numpy.array([0.0, 0.0, ray_origin_z_m]),
        numpy.array([0.0, 0.0, -1.0]),
        None,
        True,
        -1,
        hit_geom_id,
    )

    assert normalization["sampled_elevation_range_m"] == 0.0
    assert normalization["flat_field_epsilon_m"] == pytest.approx(1.0e-6)
    assert model.hfield_size[hfield_id, 2] == pytest.approx(1.0e-6)
    assert model.geom_pos[terrain_geom_id, 2] == pytest.approx(expected_height_m)
    assert hit_geom_id[0] == terrain_geom_id
    assert ray_origin_z_m - distance_m == pytest.approx(expected_height_m, abs=1.0e-6)


def test_world_importer_materializes_one_collision_body_for_each_mujoco_entity(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    root = ET.parse(draft.package_root / "world.xml").getroot()
    bodies = root.findall("./worldbody/body")
    assert [body.attrib["name"] for body in bodies] == ["entity_rock_01"]
    assert [geom.attrib["name"] for geom in bodies[0].findall("geom")] == ["entity_rock_01_collision"]
    assert bodies[0].find("geom").attrib["type"] == "box"


def test_world_importer_preserves_declared_entity_collision_geometry(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    request["entities"][0]["geometry"] = {
        "shape": "cylinder",
        "radius_m": 0.2,
        "half_height_m": 0.35,
    }

    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    root = ET.parse(draft.package_root / "world.xml").getroot()
    geom = root.find("./worldbody/body/geom")
    assert geom is not None
    assert geom.attrib["type"] == "cylinder"
    assert [float(value) for value in geom.attrib["size"].split()] == [0.2, 0.35]

    manifest = yaml.safe_load(draft.manifest_path.read_text(encoding="utf-8"))
    assert manifest["entities"][0]["geometry"] == request["entities"][0]["geometry"]


def test_world_importer_projects_runtime_entities_with_exact_geometry_and_material(
    tmp_path: Path,
) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    request["entities"][0]["geometry"] = {
        "shape": "cylinder",
        "radius_m": 0.2,
        "half_height_m": 0.35,
    }
    request["entities"][0]["visual"] = {
        "mode": "runtime",
        "material": {
            "key": "safety_orange",
            "base_color_rgba": [0.82, 0.32, 0.06, 1.0],
            "metallic": 0.0,
            "roughness": 0.4,
        },
    }

    draft = WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    projection = json.loads(
        (draft.package_root / "visual" / "world.visual-projection.json").read_text(
            encoding="utf-8"
        )
    )
    assert projection["entities"] == [
        {
            "entity_id": "rock_01",
            "semantic_class": "rock",
            "authority": "mujoco",
            "transform": {
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "geometry": {
                "shape": "cylinder",
                "radius_m": 0.2,
                "half_height_m": 0.35,
            },
            "unreal": {
                "representation": "static_mesh",
                "component_class": "/Script/Engine.StaticMeshComponent",
                "asset_path": "/Engine/BasicShapes/Cylinder.Cylinder",
                "dimensions_m": [0.4, 0.4, 0.7],
            },
            "material": {
                "source": "world_package",
                "key": "safety_orange",
                "pbr": {
                    "base_color_rgba": [0.82, 0.32, 0.06, 1.0],
                    "metallic": 0.0,
                    "roughness": 0.4,
                },
            },
        }
    ]


def test_world_importer_uses_package_identity_and_publish_is_no_overwrite(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    importer = WorldImporter(tmp_path)
    first = importer.import_world(request, draft_root=tmp_path / "first")
    repeated = importer.import_world(request, draft_root=tmp_path / "first")
    assert repeated.import_id == first.import_id

    (tmp_path / "sentinel.txt").write_text("keep me\n", encoding="utf-8")
    (source / "height.r16").write_bytes(struct.pack("<4H", 0, 16_384, 32_768, 65_534))
    second = importer.import_world(request, draft_root=tmp_path / "second")
    assert second.import_id == first.import_id

    with pytest.raises(ImportFailure) as exc_info:
        importer.import_world(request, draft_root=tmp_path / "first")
    assert exc_info.value.code == ImportCode.PROMOTION_CONFLICT
    assert (tmp_path / "sentinel.txt").read_text(encoding="utf-8") == "keep me\n"
    assert (tmp_path / "first" / "package" / "world.package.yaml").is_file()


@pytest.mark.parametrize("occupied", [False, True])
def test_world_import_publish_never_replaces_competing_directory(tmp_path: Path, occupied: bool) -> None:
    importer = WorldImporter(tmp_path)
    staging = tmp_path / "staging"
    target = tmp_path / "target"
    staging.mkdir()
    (staging / "incoming.txt").write_text("incoming\n", encoding="utf-8")
    target.mkdir()
    if occupied:
        (target / "competitor.txt").write_text("competitor\n", encoding="utf-8")

    with pytest.raises(ImportFailure) as exc_info:
        importer._publish_staging(staging, target)

    assert exc_info.value.code == ImportCode.PROMOTION_CONFLICT
    assert target.is_dir()
    assert not (target / "incoming.txt").exists()
    assert (target / "competitor.txt").exists() is occupied
    assert (staging / "incoming.txt").read_text(encoding="utf-8") == "incoming\n"


def test_world_import_dangling_target_symlink_is_not_followed(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    target = tmp_path / "draft"
    missing = tmp_path / "must-not-be-created"
    try:
        target.symlink_to(missing, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"directory symlinks are unavailable: {exc}")

    with pytest.raises(ImportFailure) as exc_info:
        WorldImporter(tmp_path).import_world(request, draft_root=target)

    assert exc_info.value.code == ImportCode.PROMOTION_CONFLICT
    assert target.is_symlink()
    assert not missing.exists()
    quarantine_roots = _quarantine_roots(tmp_path)
    assert len(quarantine_roots) == 1
    assert (quarantine_roots[0] / "quarantine" / "failure.json").is_file()


def test_world_import_publish_race_cannot_redirect_quarantine(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    importer = WorldImporter(tmp_path)
    target = tmp_path / "draft"
    redirect = tmp_path / "competitor-owned"
    redirect.mkdir()
    (redirect / "sentinel.txt").write_text("untouched\n", encoding="utf-8")
    original_publish = importer._publish_staging

    def competing_publish(staging: Path, publication_target: Path) -> None:
        try:
            publication_target.symlink_to(redirect, target_is_directory=True)
        except OSError as exc:
            pytest.skip(f"directory symlinks are unavailable: {exc}")
        original_publish(staging, publication_target)

    monkeypatch.setattr(importer, "_publish_staging", competing_publish)
    with pytest.raises(ImportFailure) as exc_info:
        importer.import_world(request, draft_root=target)

    assert exc_info.value.code == ImportCode.PROMOTION_CONFLICT
    assert (redirect / "sentinel.txt").read_text(encoding="utf-8") == "untouched\n"
    assert sorted(path.name for path in redirect.iterdir()) == ["sentinel.txt"]
    quarantine_roots = _quarantine_roots(tmp_path)
    assert len(quarantine_roots) == 8
    assert all(path.is_symlink() for path in quarantine_roots)
    assert not any((path / "quarantine" / "failure.json").exists() for path in quarantine_roots)


def test_world_import_retries_when_side_quarantine_target_is_replaced(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    importer = WorldImporter(tmp_path)
    target = tmp_path / "draft"
    target.mkdir()
    redirect = tmp_path / "competitor-owned"
    redirect.mkdir()
    (redirect / "sentinel.txt").write_text("untouched\n", encoding="utf-8")
    original_publish = importer._publish_staging
    raced = False

    def replace_publication_target(staging: Path, publication_target: Path) -> None:
        nonlocal raced
        if raced:
            original_publish(staging, publication_target)
            return
        raced = True
        try:
            publication_target.symlink_to(redirect, target_is_directory=True)
        except OSError as exc:
            pytest.skip(f"directory symlinks are unavailable: {exc}")
        original_publish(staging, publication_target)

    monkeypatch.setattr(importer, "_publish_staging", replace_publication_target)
    failure = ImportFailure("competing target", code=ImportCode.PROMOTION_CONFLICT)
    importer._quarantine(target, {"schema": "test"}, failure)

    assert sorted(path.name for path in redirect.iterdir()) == ["sentinel.txt"]
    quarantine_roots = _quarantine_roots(tmp_path)
    assert len(quarantine_roots) == 2
    assert sum((path / "quarantine" / "failure.json").is_file() for path in quarantine_roots) == 1


def test_world_import_serializes_same_identity_and_is_idempotent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    importer = WorldImporter(tmp_path)
    target = tmp_path / "draft"
    original = importer._import_normalized
    state_lock = threading.Lock()
    active = 0
    max_active = 0

    def observed_import(*args: object, **kwargs: object):
        nonlocal active, max_active
        with state_lock:
            active += 1
            max_active = max(max_active, active)
        try:
            time.sleep(0.05)
            return original(*args, **kwargs)
        finally:
            with state_lock:
                active -= 1

    monkeypatch.setattr(importer, "_import_normalized", observed_import)
    with ThreadPoolExecutor(max_workers=2) as pool:
        drafts = list(pool.map(lambda _: importer.import_world(request, draft_root=target), range(2)))

    assert max_active == 1
    assert drafts[0].import_id == drafts[1].import_id


def test_world_import_is_byte_identical_for_same_request(tmp_path: Path) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    importer = WorldImporter(tmp_path)

    first = importer.import_world(request, draft_root=tmp_path / "one")
    second = importer.import_world(request, draft_root=tmp_path / "two")

    first_files = {
        path.relative_to(first.package_root).as_posix(): path.read_bytes()
        for path in sorted(first.package_root.rglob("*"))
        if path.is_file()
    }
    second_files = {
        path.relative_to(second.package_root).as_posix(): path.read_bytes()
        for path in sorted(second.package_root.rglob("*"))
        if path.is_file()
    }
    assert first_files == second_files


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        (lambda data: data["mesh"].update({"collision": False}), ImportCode.MODEL_INVALID),
        (lambda data: data["entities"].append(copy.deepcopy(data["entities"][0])), ImportCode.MODEL_INVALID),
        (
            lambda data: data.update({"bounds": {"min_m": [0.0, 0.0, 0.0], "max_m": [1.0, 1.0, 1.0]}}),
            ImportCode.PROJECTION_INVALID,
        ),
        (lambda data: data["spawn"].update({"position_m": [9.0, 0.0, 0.0]}), ImportCode.WORLD_ALIGNMENT_INVALID),
        (
            lambda data: data.update({"units": {"length": "cm", "up_axis": "Z", "handedness": "RH"}}),
            ImportCode.UNIT_AMBIGUOUS,
        ),
        (lambda data: data["source"]["provenance"].pop("license_file"), ImportCode.INVALID_REQUEST),
        (lambda data: data["visual"].update({"level": "not-a-game-path"}), ImportCode.INVALID_REQUEST),
    ],
)
def test_world_importer_rejects_unsafe_or_ambiguous_inputs(tmp_path: Path, mutation, code: ImportCode) -> None:
    source = _source_tree(tmp_path)
    request = _request(source)
    mutation(request)

    with pytest.raises(ImportFailure) as exc_info:
        WorldImporter(tmp_path).import_world(request, draft_root=tmp_path / "draft")

    assert exc_info.value.code == code
    assert any((path / "quarantine" / "failure.json").is_file() for path in _quarantine_roots(tmp_path))
    assert not (tmp_path / "draft" / "package").exists()


def test_world_package_v1_schema_keeps_checked_in_canonical_manifests_valid() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    importer = WorldImporter(repo_root)
    manifests = sorted((repo_root / "sim" / "packages" / "worlds").glob("*/world.package.yaml"))

    assert manifests
    for manifest_path in manifests:
        importer._schema_gate(
            yaml.safe_load(manifest_path.read_text(encoding="utf-8")),
            "world.v1.json",
            f"canonical manifest {manifest_path.name}",
            ImportCode.MODEL_INVALID,
        )


def test_world_package_v1_schema_rejects_a_partial_enriched_envelope() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    manifest_path = repo_root / "sim" / "packages" / "worlds" / "open_field" / "world.package.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["visual"]["projection"] = "visual/world.visual-projection.json"

    with pytest.raises(ImportFailure, match="required property"):
        WorldImporter(repo_root)._schema_gate(
            manifest,
            "world.v1.json",
            "partial enriched manifest",
            ImportCode.MODEL_INVALID,
        )


def test_catalog_resolver_rejects_tampered_projection_structure(tmp_path: Path) -> None:
    draft = WorldImporter(tmp_path).import_world(_request(_source_tree(tmp_path)), draft_root=tmp_path / "draft")
    projection_path = draft.package_root / "visual" / "world.visual-projection.json"
    projection = json.loads(projection_path.read_text(encoding="utf-8"))
    projection["binding"] = "WorldVisual:Tampered"
    projection_path.write_text(json.dumps(projection, sort_keys=True), encoding="utf-8")

    with pytest.raises(CatalogError, match=r"binding does not match"):
        CatalogResolver(tmp_path, (draft.package_root,))


def test_catalog_resolver_does_not_rehash_importer_metadata(tmp_path: Path) -> None:
    draft = WorldImporter(tmp_path).import_world(_request(_source_tree(tmp_path)), draft_root=tmp_path / "draft")
    draft.provenance_path.write_text(
        draft.provenance_path.read_text(encoding="utf-8") + "\n",
        encoding="utf-8",
    )
    manifest = yaml.safe_load(draft.manifest_path.read_text(encoding="utf-8"))
    manifest["content"]["digest"] = "0" * 64
    draft.manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")

    record = CatalogResolver(tmp_path, (draft.package_root,)).find_package("field@1.0.0", kind="world")

    assert record.ref == "field@1.0.0"
