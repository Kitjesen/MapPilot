# ruff: noqa: S101

"""Collision-proxy contracts for the deterministic Forest_HF world."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import shutil
import struct
from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml

from sim.catalog import CatalogResolver
from sim.tools.worlds.forest_hf.generate import TerrainSpec, generate_forest_hf
from sim.tools.worlds.forest_hf.materialize_proxies import (
    main as materialize_proxies_main,
)
from sim.tools.worlds.forest_hf.materialize_proxies import (
    materialize_worldpackage_collision_proxies,
)
from sim.tools.worlds.forest_hf.proxies import (
    MAX_PROXY_COUNT,
    MAX_ROCK_PROXIES_PER_CELL,
    MAX_TREE_PROXIES_PER_CELL,
    build_collision_proxy_artifacts,
    collision_clearance_evidence,
    merge_collision_proxies_into_world,
    write_collision_proxy_artifacts,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
BUILDER_PATH = (
    REPO_ROOT / "sim/runtime/visual/RobotSimUE/Scripts/build_forest_hf.py"
)
PACKAGE_ROOT = REPO_ROOT / "sim/packages/worlds/forest_hf/2.0.0"
ROUTES = json.loads(
    (
        REPO_ROOT
        / "sim"
        / "packages"
        / "worlds"
        / "forest_hf"
        / "2.0.0"
        / "routes"
        / "forest.routes.json"
    ).read_text(encoding="utf-8")
)


def _builder():
    spec = importlib.util.spec_from_file_location(
        "lingtu_forest_hf_collision_builder", BUILDER_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _bake_contract() -> dict[str, object]:
    sources = tuple(
        json.loads(path.read_text(encoding="utf-8"))
        for path in (
            PACKAGE_ROOT / "visual/ue_projection.json",
            PACKAGE_ROOT / "terrain.recipe.json",
            PACKAGE_ROOT / "routes/forest.routes.json",
        )
    )
    return _builder().build_offline_bake_contract(*sources)


def _sparse_production_package(tmp_path: Path) -> Path:
    generated = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=17))
    root = generated.package_root
    generated_root = root / "generated"
    resolution = 4033
    sample_count = resolution * resolution
    raw_path = generated_root / "heightfield_u16.raw"
    with raw_path.open("wb") as stream:
        stream.seek(sample_count * 2 - 1)
        stream.write(b"\0")
    f32_path = generated_root / "heightfield_f32.bin"
    with f32_path.open("wb") as stream:
        stream.write(struct.pack("<ii", resolution, resolution))
        stream.seek(8 + sample_count * 4 - 1)
        stream.write(b"\0")

    from sim.tools.worlds.forest_hf import generate as generator

    (root / "physics/forest_hf.xml").write_bytes(
        generator._mjcf(
            elevation_origin_m=-10.0,
            elevation_scale_m=1.0e-6,
        )
    )

    asset_manifest_path = generated_root / "asset-manifest.json"
    asset_manifest = json.loads(asset_manifest_path.read_text(encoding="utf-8"))
    asset_manifest["validation_resolution"] = False
    asset_manifest["canonical_source"].update(
        {
            "dimensions_px": [resolution, resolution],
            "sample_min_u16": 0,
            "sample_max_u16": 0,
            "sampled_elevation_min_m": -10.0,
            "sampled_elevation_max_m": -10.0,
            "sampled_elevation_range_m": 0.0,
            "mujoco_elevation_scale_m": 1.0e-6,
        }
    )
    for record in asset_manifest["artifacts"]:
        artifact = root / record["path"]
        record["bytes"] = artifact.stat().st_size
        record["sha256"] = hashlib.sha256(artifact.read_bytes()).hexdigest()
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in asset_manifest["artifacts"]
    ]
    asset_manifest["asset_set_sha256"] = hashlib.sha256(
        (json.dumps(identity, sort_keys=True, separators=(",", ":")) + "\n").encode()
    ).hexdigest()
    asset_manifest_path.write_bytes(_builder().canonical_json_bytes(asset_manifest))

    projection_assets = (
        generator._projection_asset(
            root,
            "generated/heightfield_u16.raw",
            "canonical_u16_source",
            collision=False,
        ),
        generator._projection_asset(
            root,
            "generated/heightfield_f32.bin",
            "mujoco_heightfield",
            collision=True,
        ),
        generator._projection_asset(
            root,
            "generated/heightfield_r16.png",
            "unreal_heightfield",
            collision=False,
        ),
    )
    generator._write_canonical_package(
        root,
        spec=TerrainSpec(),
        projection_assets=projection_assets,
    )
    return root


def _placements() -> dict[str, object]:
    instances: list[dict[str, object]] = []
    dressing: list[dict[str, object]] = []
    for cell_x in range(-3, 4):
        for cell_y in range(-3, 4):
            base_x = cell_x * 100.0 + 45.0
            base_y = cell_y * 100.0 + 35.0
            for slot in range(7):
                instances.append(
                    {
                        "stable_id": f"forest.tree.{cell_x + 3:02d}.{cell_y + 3:02d}.{slot:02d}",
                        "kind": "tree",
                        "position_m": [base_x + slot, base_y + slot * 0.5, 12.0],
                        "scale_xyz": [1.0, 1.0, 1.0],
                    }
                )
            for slot in range(4):
                dressing.append(
                    {
                        "stable_id": f"forest.rock.{cell_x + 3:02d}.{cell_y + 3:02d}.{slot:02d}",
                        "kind": "rock",
                        "asset_slot_id": "forest.asset.boulder",
                        "position_m": [base_x - slot, base_y - slot * 0.5, 12.0],
                        "scale": 1.0,
                    }
                )
    return {"instances": instances, "dressing": dressing}


def _distance_to_segment(
    point: tuple[float, float], start: list[float], end: list[float]
) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.dist(point, start)
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator
    ratio = min(1.0, max(0.0, ratio))
    return math.dist(point, (start[0] + ratio * dx, start[1] + ratio * dy))


def test_proxy_contract_is_reproducible_bounded_and_uses_stable_source_ids() -> None:
    first = build_collision_proxy_artifacts(_placements(), ROUTES)
    second = build_collision_proxy_artifacts(_placements(), ROUTES)

    assert first == second
    manifest = first.manifest
    assert manifest["authority"] == {
        "physics": "mujoco",
        "raycast": "mujoco",
        "unreal": "visual_only_no_collision",
        "render_meshes_are_colliders": False,
    }
    selection = manifest["selection"]
    assert selection["maximum_proxy_count"] == MAX_PROXY_COUNT
    assert selection["selected_proxy_count"] == len(manifest["proxies"])
    assert selection["selected_tree_proxy_count"] > 0
    assert selection["selected_rock_proxy_count"] > 0
    assert len(manifest["proxies"]) <= MAX_PROXY_COUNT
    assert {proxy["kind"] for proxy in manifest["proxies"]} == {"tree", "rock"}
    assert all(proxy["source_stable_id"].startswith("forest.") for proxy in manifest["proxies"])

    counts: dict[tuple[int, int, str], int] = {}
    for proxy in manifest["proxies"]:
        cell_x, cell_y = proxy["cell_xy"]
        key = cell_x, cell_y, proxy["kind"]
        counts[key] = counts.get(key, 0) + 1
    for (*_cell, kind), count in counts.items():
        limit = MAX_TREE_PROXIES_PER_CELL if kind == "tree" else MAX_ROCK_PROXIES_PER_CELL
        assert count <= limit


def test_every_proxy_preserves_route_spawn_and_goal_clearance() -> None:
    artifacts = build_collision_proxy_artifacts(_placements(), ROUTES)
    evidence = collision_clearance_evidence(artifacts.manifest["proxies"], ROUTES)

    assert evidence["qualified"] is True
    assert evidence["minimum_route_margin_m"] >= 0.0
    assert evidence["minimum_spawn_margin_m"] >= 0.0
    assert evidence["minimum_goal_margin_m"] >= 0.0

    for proxy in artifacts.manifest["proxies"]:
        x, y, _z = proxy["position_m"]
        radius = proxy["footprint_radius_m"]
        for route in ROUTES["routes"]:
            limit = (
                route["road_width_m"] / 2.0
                + route["navigation_limits"]["clearance_each_side_m"]
                + radius
            )
            distance = min(
                _distance_to_segment((x, y), start, end)
                for start, end in zip(
                    route["centerline_xy_m"], route["centerline_xy_m"][1:]
                )
            )
            assert distance >= limit
        for marker, radius_field in (
            (ROUTES["spawn"], "minimum_clear_radius_m"),
            (ROUTES["goal"], "acceptance_radius_m"),
        ):
            assert math.dist((x, y), marker["position_xy_m"]) >= (
                marker[radius_field] + 2.0 + radius
            )


def test_mjcf_contains_only_simplified_tree_and_rock_primitives(tmp_path: Path) -> None:
    artifacts = build_collision_proxy_artifacts(_placements(), ROUTES)
    root = ElementTree.fromstring(artifacts.mjcf)  # noqa: S314 - generated locally
    geoms = root.findall("./worldbody/body/geom")

    assert len(geoms) == len(artifacts.manifest["proxies"])
    assert {geom.attrib["type"] for geom in geoms} == {"cylinder", "ellipsoid"}
    assert all(geom.attrib["contype"] == geom.attrib["conaffinity"] == "1" for geom in geoms)
    mjcf_path, manifest_path = write_collision_proxy_artifacts(tmp_path, artifacts)
    assert mjcf_path.read_bytes() == artifacts.mjcf
    assert json.loads(manifest_path.read_text(encoding="utf-8")) == artifacts.manifest


def test_unreal_placeholder_placements_fail_closed_without_canonical_height() -> None:
    source = [
        {
            "source_stable_id": "forest.foliage.x00.y00.00",
            "mesh_slot": "forest.asset.pine",
            "unreal_transform": {
                "location_cm": [1000.0, -2000.0, 0.0],
                "scale_xyz": [1.0, 1.0, 1.0],
            },
        },
        {
            "source_stable_id": "forest.boulder.x00.y00.00",
            "mesh_slot": "forest.asset.boulder",
            "unreal_transform": {
                "location_cm": [2000.0, -3000.0, 0.0],
                "scale_xyz": [1.0, 1.0, 1.0],
            },
        },
    ]

    with pytest.raises(ValueError, match="canonical terrain height sampler"):
        build_collision_proxy_artifacts(source, ROUTES)

    artifacts = build_collision_proxy_artifacts(source, ROUTES, height_at=lambda _x, _y: 4.5)
    assert all(proxy["position_m"][2] > 4.5 for proxy in artifacts.manifest["proxies"])


def test_tampered_routes_and_duplicate_ids_fail_closed() -> None:
    routes = json.loads(json.dumps(ROUTES))
    routes["routes"][0]["road_width_m"] += 1.0
    with pytest.raises(ValueError, match="digest"):
        build_collision_proxy_artifacts(_placements(), routes)

    duplicate = [
        {"stable_id": "forest.tree.same", "kind": "tree", "position_m": [0.0, 0.0, 0.0]},
        {"stable_id": "forest.tree.same", "kind": "tree", "position_m": [1.0, 1.0, 0.0]},
    ]
    with pytest.raises(ValueError, match="duplicate"):
        build_collision_proxy_artifacts(duplicate, ROUTES)

    tree_only = build_collision_proxy_artifacts(
        [
            {
                "stable_id": "forest.tree.only",
                "kind": "tree",
                "position_m": [900.0, 900.0, 0.0],
            }
        ],
        ROUTES,
    )
    assert tree_only.manifest["source"][
        "authoritative_collision_candidate_classes"
    ] == ["tree"]
    assert {proxy["kind"] for proxy in tree_only.manifest["proxies"]} == {"tree"}


def test_unreal_bake_v2_drives_stable_tree_proxy_identity_and_xy_position() -> None:
    contract = _bake_contract()
    artifacts = build_collision_proxy_artifacts(
        contract, ROUTES, height_at=lambda _x, _y: 4.25
    )
    manifest = artifacts.manifest

    assert manifest["source"]["source_type"] == "unreal_offline_bake"
    assert manifest["source"]["bake_contract_digest"] == contract["content_digest"]
    assert manifest["source"][
        "authoritative_collision_candidate_classes"
    ] == ["tree"]
    assert manifest["selection"]["input_candidate_count"] == 18_707
    assert manifest["selection"]["selected_rock_proxy_count"] == 0
    assert manifest["selection"]["selected_tree_proxy_count"] > 0

    source_by_id = {
        instance["source_stable_id"]: instance
        for cell in contract["authoring_density_cells"]
        for group in cell["hism_groups"]
        for instance in group["instances"]
    }
    proxy = manifest["proxies"][0]
    source = source_by_id[proxy["source_stable_id"]]
    source_location = source["unreal_transform"]["location_cm"]
    assert proxy["position_m"][:2] == pytest.approx(
        [source_location[0] / 100.0, -source_location[1] / 100.0]
    )
    expected_proxy_id = (
        "forest.physics.proxy."
        + hashlib.sha256(proxy["source_stable_id"].encode()).hexdigest()[:20]
    )
    assert proxy["stable_id"] == expected_proxy_id


def test_unreal_bake_v2_tampering_fails_closed() -> None:
    contract = _bake_contract()
    instance = contract["authoring_density_cells"][0]["hism_groups"][0][
        "instances"
    ][0]
    instance["unreal_transform"]["location_cm"][0] += 1.0

    with pytest.raises(ValueError, match="content digest"):
        build_collision_proxy_artifacts(
            contract, ROUTES, height_at=lambda _x, _y: 0.0
        )

    contract = _bake_contract()
    group = contract["authoring_density_cells"][0]["hism_groups"][0]
    group["collision_profile"] = "BlockAll"
    body = {key: value for key, value in contract.items() if key != "content_digest"}
    contract["content_digest"] = _builder()._content_digest(body)
    with pytest.raises(ValueError, match="NoCollision"):
        build_collision_proxy_artifacts(
            contract, ROUTES, height_at=lambda _x, _y: 0.0
        )


def test_generated_proxy_world_compiles_in_mujoco_when_available(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    artifacts = build_collision_proxy_artifacts(_placements(), ROUTES)
    mjcf_path, _manifest_path = write_collision_proxy_artifacts(tmp_path, artifacts)

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))

    assert model.ngeom == len(artifacts.manifest["proxies"])


def test_proxy_body_merges_into_the_authoritative_forest_world() -> None:
    artifacts = build_collision_proxy_artifacts(_placements(), ROUTES)
    base = b"""<?xml version='1.0' encoding='utf-8'?>
<mujoco model="forest_hf"><worldbody><geom name="terrain" type="plane" size="1 1 0.1"/></worldbody></mujoco>
"""

    merged = merge_collision_proxies_into_world(base, artifacts)
    root = ElementTree.fromstring(merged)  # noqa: S314 - generated locally

    assert root.attrib["model"] == "forest_hf_2_0_0_with_collision_proxies"
    assert root.find("./worldbody/geom[@name='terrain']") is not None
    assert len(root.findall("./worldbody/body[@name='forest_collision_proxies']/geom")) == len(
        artifacts.manifest["proxies"]
    )


def test_materializer_binds_layout_terrain_routes_and_mujoco_compile(
    tmp_path: Path,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    generated = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=33))

    result = materialize_worldpackage_collision_proxies(generated.package_root)
    evidence = result.evidence
    proxy_manifest = json.loads(result.proxy_manifest_path.read_text(encoding="utf-8"))

    assert evidence["qualification"] == {
        "state": "PREVIEW_ONLY",
        "qualified_for_worldpackage_promotion": False,
        "blockers": ["canonical heightfield uses validation_resolution"],
        "validation_resolution": True,
    }
    assert evidence["same_source"]["terrain_asset_set_sha256"] == generated.asset_set_sha256
    assert evidence["same_source"]["blender_layout_digest"] == proxy_manifest["source"][
        "layout_digest"
    ]
    assert evidence["height_alignment"]["qualified"] is True
    assert evidence["route_clearance"]["qualified"] is True
    assert evidence["authority"]["unreal_collision_profile"] == "NoCollision"
    assert evidence["authority"]["mujoco_physics_authority"] is True
    assert evidence["mujoco_compile"]["qualified"] is True
    assert evidence["performance_budget"]["selected_proxy_count"] <= MAX_PROXY_COUNT
    assert result.layout_path.is_file()
    assert result.evidence_path.is_file()
    assert result.merged_world_path.is_file()

    model = mujoco.MjModel.from_xml_path(str(result.merged_world_path))
    assert model.ngeom == evidence["mujoco_compile"]["model_ngeom"]
    assert model.ngeom == evidence["performance_budget"]["selected_proxy_count"] + 1


def test_materializer_never_promotes_preview_or_uncompiled_assets(tmp_path: Path) -> None:
    generated = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=17))
    manifest_before = (generated.package_root / "world.package.yaml").read_bytes()

    result = materialize_worldpackage_collision_proxies(
        generated.package_root,
        compile_mujoco=False,
    )

    assert result.evidence["qualification"] == {
        "state": "BLOCKED",
        "qualified_for_worldpackage_promotion": False,
        "blockers": [
            "canonical heightfield uses validation_resolution",
            "merged MuJoCo world has no successful compile evidence",
        ],
        "validation_resolution": True,
    }
    assert "promotion" not in result.evidence
    assert (generated.package_root / "world.package.yaml").read_bytes() == manifest_before
    assert (
        materialize_proxies_main(
            [
                "--package-root",
                str(generated.package_root),
                "--skip-mujoco-compile",
                "--require-qualified",
            ]
        )
        == 2
    )


def test_materializer_binds_unreal_bake_contract_to_package_sources(
    tmp_path: Path,
) -> None:
    pytest.importorskip("mujoco")
    generated = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=33))
    contract = _bake_contract()
    contract_path = tmp_path / "forest-bake.json"
    contract_path.write_bytes(_builder().canonical_json_bytes(contract))

    result = materialize_worldpackage_collision_proxies(
        generated.package_root,
        placement_contract_path=contract_path,
    )
    evidence = result.evidence
    proxy_manifest = json.loads(result.proxy_manifest_path.read_text(encoding="utf-8"))

    assert evidence["qualification"]["state"] == "PREVIEW_ONLY"
    assert evidence["same_source"]["placement_schema"] == (
        "lingtu.sim.unreal-forest-offline-bake.v2"
    )
    assert evidence["same_source"]["bake_contract_digest"] == contract[
        "content_digest"
    ]
    assert proxy_manifest["source"]["bake_contract_digest"] == contract[
        "content_digest"
    ]
    assert proxy_manifest["source"][
        "authoritative_collision_candidate_classes"
    ] == ["tree"]
    assert evidence["height_alignment"]["sample_count"] == 18_707
    assert evidence["mujoco_compile"]["qualified"] is True
    assert result.layout_path.name == "forest-placement-contract.json"


def test_materializer_rejects_rehashed_bake_bound_to_other_sources(
    tmp_path: Path,
) -> None:
    generated = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=17))
    contract = _bake_contract()
    contract["source_digests"]["terrain_recipe"] = "0" * 64
    body = {key: value for key, value in contract.items() if key != "content_digest"}
    contract["content_digest"] = _builder()._content_digest(body)
    contract_path = tmp_path / "tampered-source-bake.json"
    contract_path.write_bytes(_builder().canonical_json_bytes(contract))

    with pytest.raises(ValueError, match="source digest mismatch: terrain_recipe"):
        materialize_worldpackage_collision_proxies(
            generated.package_root,
            placement_contract_path=contract_path,
            compile_mujoco=False,
        )


def test_full_resolution_materialization_promotes_manifest_and_resolves_session_plan(
    tmp_path: Path,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    package_root = _sparse_production_package(tmp_path)
    contract = _bake_contract()
    contract_path = tmp_path / "forest-bake.json"
    contract_path.write_bytes(_builder().canonical_json_bytes(contract))

    result = materialize_worldpackage_collision_proxies(
        package_root,
        placement_contract_path=contract_path,
    )

    manifest = yaml.safe_load(
        (package_root / "world.package.yaml").read_text(encoding="utf-8")
    )
    assert result.evidence["qualification"] == {
        "state": "QUALIFIED",
        "qualified_for_worldpackage_promotion": True,
        "blockers": [],
        "validation_resolution": False,
    }
    assert result.evidence["promotion"]["applied"] is True
    assert result.evidence["promotion"]["manifest"] == "world.package.yaml"
    assert (
        result.evidence["promotion"]["physics_mjcf"]
        == "physics/forest_hf.with-collision-proxies.xml"
    )
    assert result.evidence["mujoco_compile"]["heightfield_raycast"] == {
        "qualified": True,
        "sample_count": 5,
        "maximum_error_m": pytest.approx(0.0, abs=1.0e-6),
        "tolerance_m": 0.001,
        "source": "generated/heightfield_u16.raw",
    }
    assert manifest["physics"]["mjcf"] == (
        "physics/forest_hf.with-collision-proxies.xml"
    )
    declared_files = {record["path"] for record in manifest["content"]["files"]}
    assert {
        "generated/forest-placement-contract.json",
        "generated/forest_collision_proxies.manifest.json",
        "generated/forest_collision_proxies.xml",
        "generated/forest_collision_promotion.evidence.json",
        "physics/forest_hf.with-collision-proxies.xml",
    } <= declared_files

    shutil.copytree(
        REPO_ROOT / "sim/robots/omni_cart",
        tmp_path / "sim/robots/omni_cart",
    )
    session_path = tmp_path / "forest.session.yaml"
    session_path.write_text(
        """schema: lingtu.sim.session.v1
session_id: forest_hf_collision_promotion
mujoco_version: 3.10.0
seed: 20260813
world: forest_hf@2.0.0
robots:
  - instance_id: cart_01
    package: omni_cart@1.0.0
    controller: null
    sensor_rig: null
    spawn:
      position_m: [-760.0, -650.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
runtime:
  backend: mujoco
  mode: headless
  required_bindings: [physics]
""",
        encoding="utf-8",
    )
    resolved = CatalogResolver.from_repository(tmp_path).resolve(session_path)
    planned_mjcf = tmp_path / resolved.physics_plan["world"]["mjcf"]
    assert planned_mjcf.resolve() == result.merged_world_path.resolve()
    assert mujoco.MjModel.from_xml_path(str(planned_mjcf)).ngeom == 801
    assert (
        materialize_proxies_main(
            [
                "--package-root",
                str(package_root),
                "--placement-contract",
                str(contract_path),
                "--require-qualified",
            ]
        )
        == 0
    )


def test_failed_production_catalog_validation_rolls_back_manifest_commit(
    tmp_path: Path,
) -> None:
    pytest.importorskip("mujoco")
    package_root = _sparse_production_package(tmp_path)
    contract_path = tmp_path / "forest-bake.json"
    contract_path.write_bytes(_builder().canonical_json_bytes(_bake_contract()))
    manifest_path = package_root / "world.package.yaml"
    projection_path = package_root / "visual/world.visual-projection.json"
    manifest_before = manifest_path.read_bytes()
    projection_before = projection_path.read_bytes()
    nested = package_root / "invalid-nested-package"
    nested.mkdir()
    (nested / "robot.package.yaml").write_text("schema: invalid\n", encoding="utf-8")

    with pytest.raises(Exception, match="missing required key"):
        materialize_worldpackage_collision_proxies(
            package_root,
            placement_contract_path=contract_path,
        )

    assert manifest_path.read_bytes() == manifest_before
    assert projection_path.read_bytes() == projection_before
    for relative in (
        "generated/forest-placement-contract.json",
        "generated/forest_collision_proxies.manifest.json",
        "generated/forest_collision_proxies.xml",
        "generated/forest_collision_promotion.evidence.json",
        "physics/forest_hf.with-collision-proxies.xml",
    ):
        assert not (package_root / relative).exists()
