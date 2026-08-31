# ruff: noqa: S101

"""Contracts for the fail-closed Forest_HF 2.0 Unreal builder."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import struct
from pathlib import Path

import pytest

from sim.tools.worlds.forest_hf.generate import TerrainSpec, generate_forest_hf

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "sim/runtime/visual/RobotSimUE/Scripts/build_forest_hf.py"
PACKAGE_ROOT = REPO_ROOT / "sim/packages/worlds/forest_hf/2.0.0"


def _builder():
    spec = importlib.util.spec_from_file_location("lingtu_forest_hf_builder", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sources():
    return tuple(
        json.loads(path.read_text(encoding="utf-8"))
        for path in (
            PACKAGE_ROOT / "visual/ue_projection.json",
            PACKAGE_ROOT / "terrain.recipe.json",
            PACKAGE_ROOT / "routes/forest.routes.json",
        )
    )


def _flat_materialized_package(tmp_path: Path) -> Path:
    result = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=17))
    generated = result.package_root / "generated"
    raw_path = generated / "heightfield_u16.raw"
    raw_path.write_bytes(struct.pack("<289H", *([32768] * 289)))
    manifest_path = generated / "asset-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for record in manifest["artifacts"]:
        artifact = result.package_root / record["path"]
        payload = artifact.read_bytes()
        record["bytes"] = len(payload)
        record["sha256"] = hashlib.sha256(payload).hexdigest()
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in manifest["artifacts"]
    ]
    manifest["asset_set_sha256"] = hashlib.sha256(
        (json.dumps(identity, sort_keys=True, separators=(",", ":")) + "\n").encode()
    ).hexdigest()
    manifest_path.write_bytes(_builder().canonical_json_bytes(manifest))
    return result.package_root


def test_builder_targets_exact_v2_partitioned_two_kilometre_map() -> None:
    builder = _builder()
    assert builder.WORLD_PACKAGE == "forest_hf@2.0.0"
    assert builder.MAP_PATH == "/Game/RobotSim/Maps/Forest_HF_2km"
    assert builder.WORLD_SIZE_M == 2000.0
    assert builder.AUTHORING_DENSITY_CELL_SIZE_M == 100.0
    assert builder.AUTHORING_DENSITY_GRID_SIZE == 20
    assert builder.UE_PCG_PARTITION_CELL_SIZE_M == 256.0
    assert builder.UE_PCG_PARTITION_GRID_SIZE == 8
    assert builder.FOLIAGE_CANDIDATES_PER_AUTHORING_CELL == 48


def test_builder_uses_runtime_coordinate_contract() -> None:
    builder = _builder()
    assert builder.source_to_unreal_location_cm((1.5, -2.0, 0.25)) == (150.0, 200.0, 25.0)


def test_offline_bake_is_deterministic_and_source_digest_bound() -> None:
    builder = _builder()
    first = builder.build_offline_bake_contract(*_sources())
    second = builder.build_offline_bake_contract(*_sources())
    assert builder.canonical_json_bytes(first) == builder.canonical_json_bytes(second)
    assert first["schema"] == "lingtu.sim.unreal-forest-offline-bake.v2"
    assert first["target"]["map"] == "/Game/RobotSim/Maps/Forest_HF_2km"
    assert len(first["authoring_density_cells"]) == 400
    assert first["tree_scale_contract"] == {
        "mother_mesh_height_m": 2.0,
        "forest_target_height_m": 9.0,
        "pcg_scale_factor": 4.5,
        "application": "multiply_existing_instance_scale_variation",
    }
    assert first["foliage_density_contract"] == {
        "authoring_grid": {
            "role": "deterministic_density_sampling_only",
            "cell_size_m": 100.0,
            "grid_size": [20, 20],
            "cell_count": 400,
            "controls_ue_partitioning": False,
        },
        "candidate_points_per_authoring_cell": 48,
        "candidate_point_budget": 19200,
        "accepted_point_count": 18707,
        "rejected_point_count": 493,
        "accepted_points_per_square_kilometre": 4676.75,
        "rejection_policy": "route_spawn_goal_clearance",
    }
    assert all(len(value) == 64 for value in first["source_digests"].values())
    assert len(first["content_digest"]) == 64


def test_density_authoring_grid_is_not_the_ue_pcg_partition_grid() -> None:
    builder = _builder()
    contract = builder.build_offline_bake_contract(*_sources())
    authoring_grid = contract["foliage_density_contract"]["authoring_grid"]
    pcg_partition = contract["target"]["world_partition"]["pcg_partition"]

    assert authoring_grid == {
        "role": "deterministic_density_sampling_only",
        "cell_size_m": 100.0,
        "grid_size": [20, 20],
        "cell_count": 400,
        "controls_ue_partitioning": False,
    }
    assert pcg_partition == {
        "role": "ue_open_world_pcg_partition",
        "cell_size_m": 256.0,
        "grid_size": [8, 8],
        "cell_count": 64,
        "coverage_size_m": [2048.0, 2048.0],
    }
    assert authoring_grid["cell_size_m"] * authoring_grid["grid_size"][0] == 2000.0
    assert pcg_partition["cell_size_m"] * pcg_partition["grid_size"][0] == 2048.0
    assert authoring_grid["cell_size_m"] != pcg_partition["cell_size_m"]
    assert all(
        cell["grid_contract"] == "deterministic_density_authoring"
        for cell in contract["authoring_density_cells"]
    )


def test_every_baked_instance_is_stable_hism_metadata_and_no_collision() -> None:
    builder = _builder()
    contract = builder.build_offline_bake_contract(*_sources())
    instances = []
    for cell in contract["authoring_density_cells"]:
        assert cell["stable_id"].startswith("forest.cell.")
        for group in cell["hism_groups"]:
            assert group["component_class"] == "HierarchicalInstancedStaticMeshComponent"
            assert group["collision_profile"] == "NoCollision"
            assert group["collision_enabled"] is False
            assert group["simulate_physics"] is False
            assert group["can_ever_affect_navigation"] is False
            assert group["nanite"]["policy"] == "asset_opt_in_after_validation"
            assert group["lod"]["required"] is True
            assert group["cull_distance_cm"] == [2500.0, 12000.0]
            instances.extend(group["instances"])
    assert len(instances) == 18707
    assert len({item["stable_entity_id"] for item in instances}) == len(instances)
    assert {item["mesh_slot"] for item in instances} == {
        "forest.asset.birch",
        "forest.asset.pine",
    }


def test_materialized_heightfield_preserves_exact_mesh_slot_source_bindings(tmp_path: Path) -> None:
    builder = _builder()
    projection, terrain, routes = _sources()
    contract = builder.build_offline_bake_contract(projection, terrain, routes)
    points = builder.build_create_points_json_params(
        contract,
        terrain,
        _flat_materialized_package(tmp_path),
    )
    assert builder.canonical_json_bytes(points) == builder.canonical_json_bytes(
        builder.build_create_points_json_params(contract, terrain, _flat_materialized_package(tmp_path / "again"))
    )
    assert points["schema"] == builder.CREATE_POINTS_SCHEMA
    assert points["world_package"] == builder.WORLD_PACKAGE
    assert points["seed"] == builder.FIXED_SEED
    assert points["contract_content_digest"] == contract["content_digest"]
    assert points["selection_policy"] == builder.EXACT_MESH_SLOT_SELECTION_POLICY
    assert points["point_count"] == 18707
    assert points["slot_counts"] == {
        "forest.asset.birch": 9469,
        "forest.asset.pine": 9238,
    }
    body = dict(points)
    del body["content_digest"]
    assert points["content_digest"] == builder._content_digest(body)

    instances_by_slot = {
        slot: [
            instance
            for cell in contract["authoring_density_cells"]
            for group in cell["hism_groups"]
            if group["mesh_slot"] == slot
            for instance in group["instances"]
        ]
        for slot in sorted(points["slot_counts"])
    }
    assert [group["mesh_slot"] for group in points["groups"]] == sorted(instances_by_slot)
    assert builder.MOTHER_TREE_HEIGHT_M == 2.0
    assert builder.FOREST_TREE_TARGET_HEIGHT_M == 9.0
    assert builder.TREE_SCALE_FACTOR == 4.5
    for group in points["groups"]:
        slot = group["mesh_slot"]
        point_records = group["json_params"]["pointsToCreate"]
        bindings = group["source_bindings"]
        instances = instances_by_slot[slot]
        assert group["point_count"] == len(point_records) == len(bindings) == len(instances)
        assert group["json_params"]["coordinateSpace"] == "World"
        assert group["json_params"]["bCullPointsOutsideVolume"] is True
        assert [point["metadataEntry"] for point in point_records] == list(range(len(point_records)))
        for index, (point, binding, instance) in enumerate(zip(point_records, bindings, instances)):
            assert binding == {
                "metadataEntry": index,
                "mesh_slot": slot,
                "source_stable_id": instance["source_stable_id"],
            }
            transform = point["transform"]
            source_transform = instance["unreal_transform"]
            assert transform["location"] == {
                "x": source_transform["location_cm"][0],
                "y": source_transform["location_cm"][1],
                "z": pytest.approx(3000.061, abs=0.001),
            }
            assert transform["rotation"] == dict(zip(("x", "y", "z"), source_transform["rotation_deg"]))
            assert transform["scale"] == {
                axis: source_transform["scale_xyz"][axis_index] * builder.TREE_SCALE_FACTOR
                for axis_index, axis in enumerate(("x", "y", "z"))
            }
            assert point["density"] == 1.0
            assert point["boundsMin"] == {"x": -50.0, "y": -50.0, "z": 0.0}
            assert point["boundsMax"] == {"x": 50.0, "y": 50.0, "z": 200.0}
            assert "bounds" not in point
            assert point["color"] == {"x": 1.0, "y": 1.0, "z": 1.0, "w": 1.0}
            assert point["steepness"] == 0.0
            assert 0 <= point["seed"] <= 2_147_483_647


def test_height_sampling_respects_north_to_south_rows_and_elevation_range() -> None:
    builder = _builder()
    raw = struct.pack("<4H", 0, 65535, 16384, 32768)

    assert builder._height_sample_m(raw, 2, 2, -10.0, 70.0, -1000.0, 1000.0) == -10.0
    assert builder._height_sample_m(raw, 2, 2, -10.0, 70.0, 1000.0, 1000.0) == 70.0
    assert builder._height_sample_m(raw, 2, 2, -10.0, 70.0, -1000.0, -1000.0) == pytest.approx(
        10.000305,
        abs=0.000001,
    )


def test_materialized_heightfield_digest_drift_fails_closed(tmp_path: Path) -> None:
    builder = _builder()
    projection, terrain, routes = _sources()
    package = _flat_materialized_package(tmp_path)
    raw_path = package / "generated/heightfield_u16.raw"
    raw_path.write_bytes(raw_path.read_bytes() + b"drift")

    with pytest.raises(ValueError, match=r"heightfield_u16\.raw byte count mismatch"):
        builder.build_create_points_json_params(
            builder.build_offline_bake_contract(projection, terrain, routes),
            terrain,
            package,
        )


def test_cli_optional_pcg_output_preserves_default_contract_output(tmp_path: Path) -> None:
    builder = _builder()
    package = _flat_materialized_package(tmp_path)
    contract_output = tmp_path / "contract.json"
    points_output = tmp_path / "create-points.json"

    assert builder.main(
        [
            "--validate-only",
            "--contract-output",
            str(contract_output),
            "--materialized-root",
            str(package),
            "--pcg-points-output",
            str(points_output),
        ]
    ) == 0
    assert json.loads(contract_output.read_text(encoding="utf-8"))["schema"].endswith(".v2")
    params = json.loads(points_output.read_text(encoding="utf-8"))
    assert params["selection_policy"] == builder.EXACT_MESH_SLOT_SELECTION_POLICY
    assert params["point_count"] == 18707


def test_builder_rejects_tree_slots_absent_from_blender_asset_library(monkeypatch) -> None:
    builder = _builder()
    monkeypatch.setattr(builder, "_TREE_ASSET_SLOTS", ("forest.asset.oak",))

    with pytest.raises(ValueError, match="not present in the Blender asset library"):
        builder.build_offline_bake_contract(*_sources())


@pytest.mark.parametrize("source_index", [0, 1, 2])
def test_builder_fails_closed_on_package_or_digest_drift(source_index: int) -> None:
    builder = _builder()
    sources = list(_sources())
    sources[source_index] = copy.deepcopy(sources[source_index])
    sources[source_index]["world_package"] = "forest_hf@1.0.0"
    with pytest.raises(ValueError, match=r"exact package forest_hf@2\.0\.0"):
        builder.build_offline_bake_contract(*sources)

    projection, terrain, routes = _sources()
    routes["routes"][0]["road_width_m"] = 11.0
    with pytest.raises(ValueError, match="routes digest"):
        builder.build_offline_bake_contract(projection, terrain, routes)


def test_pcg_authoring_fails_closed_without_repo_supported_adapter() -> None:
    builder = _builder()
    with pytest.raises(RuntimeError, match="PCG Python authoring is disabled"):
        builder.require_supported_ue_authoring_adapter()
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    assert "unreal.PCG" not in source
    assert 'PCG_AUTHORING_MODE = "offline_baked_hism"' in source
    assert 'PHYSICS_AUTHORITY = "mujoco"' in source
    assert "forest_hf@1.0.0" not in source
