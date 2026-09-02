# ruff: noqa: S101

"""Contracts for the deterministic 2 km Forest_HF WorldPackage."""

from __future__ import annotations

import hashlib
import json
import math
import struct
from itertools import pairwise
from pathlib import Path

import pytest
import yaml

from sim.catalog import CatalogResolver
from sim.catalog.importers.heightmap import build_heightmap_artifacts
from sim.tools.worlds.forest_hf.generate import (
    DEFAULT_SEED,
    PRODUCTION_RESOLUTION_PX,
    TerrainSpec,
    _road_influence_q16,
    _terrain_sample,
    generate_forest_hf,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / "sim/packages/worlds/forest_hf/2.0.0"


def _route_metrics(
    spec: TerrainSpec,
    route: dict[str, object],
) -> dict[str, float]:
    spacing_m = spec.extent_m / (spec.resolution_px - 1)
    half_width_m = float(route["road_width_m"]) / 2.0
    lateral_steps = max(2, math.ceil((2.0 * half_width_m - 2.0 * spacing_m) / spacing_m))
    lateral_offsets_m = tuple(
        -half_width_m + spacing_m + (2.0 * half_width_m - 2.0 * spacing_m) * step / lateral_steps
        for step in range(lateral_steps + 1)
    )
    maximum_grade = 0.0
    maximum_cross_slope = 0.0
    maximum_step_m = 0.0

    def canonical_sample(x_m: float, y_m: float) -> tuple[float, float, float]:
        column = round((x_m + spec.extent_m / 2.0) / spacing_m)
        row = round((spec.extent_m / 2.0 - y_m) / spacing_m)
        canonical_x_m = -spec.extent_m / 2.0 + column * spacing_m
        canonical_y_m = spec.extent_m / 2.0 - row * spacing_m
        sample = _terrain_sample(spec, DEFAULT_SEED, column, row)
        elevation_m = (
            sample / 65_535 * (spec.elevation_max_m - spec.elevation_min_m)
            + spec.elevation_min_m
        )
        return canonical_x_m, canonical_y_m, elevation_m

    centerline = route["centerline_xy_m"]
    for start, end in pairwise(centerline):
        delta_x = float(end[0]) - float(start[0])
        delta_y = float(end[1]) - float(start[1])
        length_m = math.hypot(delta_x, delta_y)
        tangent_x, tangent_y = delta_x / length_m, delta_y / length_m
        normal_x, normal_y = -tangent_y, tangent_x
        steps = max(1, math.ceil(length_m / spacing_m))
        prior_by_offset: dict[float, tuple[float, float, float]] = {}
        for step in range(steps + 1):
            amount = step / steps
            center_x = float(start[0]) + delta_x * amount
            center_y = float(start[1]) + delta_y * amount
            cross_section = []
            for offset_m in lateral_offsets_m:
                point = canonical_sample(
                    center_x + normal_x * offset_m,
                    center_y + normal_y * offset_m,
                )
                cross_section.append(point)
                prior = prior_by_offset.get(offset_m)
                if prior is not None:
                    travel_m = math.hypot(point[0] - prior[0], point[1] - prior[1])
                    if travel_m > 0.0:
                        elevation_delta_m = abs(point[2] - prior[2])
                        maximum_grade = max(maximum_grade, elevation_delta_m / travel_m)
                        maximum_step_m = max(maximum_step_m, elevation_delta_m)
                prior_by_offset[offset_m] = point
            cross_run_m = math.hypot(
                cross_section[-1][0] - cross_section[0][0],
                cross_section[-1][1] - cross_section[0][1],
            )
            maximum_cross_slope = max(
                maximum_cross_slope,
                abs(cross_section[-1][2] - cross_section[0][2]) / cross_run_m,
            )
            for left, right in pairwise(cross_section):
                cross_step_m = abs(right[2] - left[2])
                cross_spacing_m = math.hypot(right[0] - left[0], right[1] - left[1])
                if cross_spacing_m > 0.0:
                    maximum_cross_slope = max(maximum_cross_slope, cross_step_m / cross_spacing_m)
                    maximum_step_m = max(maximum_step_m, cross_step_m)
    return {
        "maximum_grade": maximum_grade,
        "maximum_cross_slope": maximum_cross_slope,
        "maximum_step_m": maximum_step_m,
    }


def _generated_payloads(root: Path) -> dict[str, bytes]:
    package = root / "sim/packages/worlds/forest_hf/2.0.0"
    return {
        path.relative_to(package).as_posix(): path.read_bytes()
        for path in sorted(package.rglob("*"))
        if path.is_file()
    }


def test_checked_in_package_declares_the_production_world_contract() -> None:
    manifest = yaml.safe_load((PACKAGE_ROOT / "world.package.yaml").read_text(encoding="utf-8"))
    recipe = json.loads((PACKAGE_ROOT / "terrain.recipe.json").read_text(encoding="utf-8"))
    routes = json.loads((PACKAGE_ROOT / "routes/forest.routes.json").read_text(encoding="utf-8"))
    projection = json.loads((PACKAGE_ROOT / "visual/ue_projection.json").read_text(encoding="utf-8"))
    canonical_projection = json.loads(
        (PACKAGE_ROOT / "visual/world.visual-projection.json").read_text(encoding="utf-8")
    )

    assert manifest["id"] == "forest_hf"
    assert manifest["version"] == "2.0.0"
    assert recipe["seed"] == DEFAULT_SEED == 20260813
    assert recipe["source_grid"]["dimensions_px"] == [4033, 4033]
    assert recipe["source_grid"]["sample_type"] == "uint16"
    assert recipe["extent_m"] == [2000.0, 2000.0]
    assert recipe["canonical_source"] == "generated/heightfield_u16.raw"
    assert recipe["facets"]["mujoco"]["format"] == "mujoco_hfield_f32"
    normalization = recipe["facets"]["mujoco"]["normalization"]
    assert normalization["compiler_behavior"] == "affine_sample_min_max_to_unit_interval"
    assert normalization["production_sample_min_u16"] == 7_125
    assert normalization["production_sample_max_u16"] == 32_896
    assert normalization["geom_origin_z_m"] == pytest.approx(-1.3023575188830385)
    assert normalization["hfield_elevation_scale_m"] == pytest.approx(31.459220263981077)
    assert recipe["facets"]["unreal"] == {
        "actor_type": "Landscape",
        "axis_mapping": ["x", "-y", "z"],
        "components_xy": [64, 64],
        "format": "ue_landscape_r16_png",
        "heightmap": "generated/heightfield_r16.png",
        "location_z_cm": 3000.0,
        "quads_per_section": 63,
        "scale_cm": pytest.approx([200000.0 / 4032.0, 200000.0 / 4032.0, 15.625]),
        "sections_per_component": 1,
        "world_partition": True,
    }
    assert routes["extent_m"] == [2000.0, 2000.0]
    assert projection["terrain"]["source_resolution_px"] == [4033, 4033]
    assert projection["terrain"]["actor_type"] == "Landscape"
    assert projection["terrain"]["world_partition"] is True
    assert "source_mesh" not in projection["terrain"]
    assert manifest["visual"]["projection"] == "visual/world.visual-projection.json"
    assert canonical_projection["schema"] == "lingtu.sim.world-visual-projection.v1"
    assert canonical_projection["level"] == "/Game/RobotSim/Maps/Forest_HF_2km"
    assert canonical_projection["terrain"]["grid_px"] == [4033, 4033]
    assert not (PACKAGE_ROOT / "generated").exists()


def test_checked_in_metadata_matches_the_generator(tmp_path: Path) -> None:
    generated = generate_forest_hf(tmp_path, emit_artifacts=False)

    assert _generated_payloads(REPO_ROOT) == _generated_payloads(tmp_path)
    assert generated.asset_set_sha256 is None
    assert not (generated.package_root / "generated").exists()


def test_catalog_resolver_accepts_the_canonical_package() -> None:
    resolver = CatalogResolver(REPO_ROOT, (PACKAGE_ROOT,))
    record = resolver.find_package("forest_hf@2.0.0", kind="world")

    assert record.data["content"]["files"]
    assert record.data["visual"]["projection"] == "visual/world.visual-projection.json"


def test_session_resolution_uses_the_declared_world_mjcf(
    tmp_path: Path,
) -> None:
    session_path = tmp_path / "session.yaml"
    session_path.write_text(
        yaml.safe_dump(
            {
                "schema": "lingtu.sim.session.v1",
                "session_id": "forest_hf_missing_terrain",
                "mujoco_version": "3.10.0",
                "seed": 20260819,
                "world": "forest_hf@2.0.0",
                "robots": [
                    {
                        "instance_id": "cart_01",
                        "package": "omni_cart@1.0.0",
                        "controller": None,
                        "sensor_rig": None,
                        "spawn": {
                            "position_m": [-760.0, -650.0, 0.0],
                            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                        },
                    }
                ],
                "runtime": {
                    "backend": "mujoco",
                    "mode": "headless",
                    "required_bindings": ["physics"],
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(session_path)

    assert resolved.session == yaml.safe_load(session_path.read_text(encoding="utf-8"))
    assert resolved.physics_plan["schema"] == "lingtu.sim.physics-plan.v1"
    assert resolved.physics_plan["world"] == {
        "package": {
            "id": "forest_hf",
            "version": "2.0.0",
            "kind": "world",
            "manifest": "sim/packages/worlds/forest_hf/2.0.0/world.package.yaml",
        },
        "mjcf": "sim/packages/worlds/forest_hf/2.0.0/physics/forest_hf.xml",
    }
    assert (REPO_ROOT / resolved.physics_plan["world"]["mjcf"]).is_file()


def test_routes_have_stable_ids_and_explicit_navigation_limits() -> None:
    routes = json.loads((PACKAGE_ROOT / "routes/forest.routes.json").read_text(encoding="utf-8"))
    ids = [route["stable_id"] for route in routes["routes"]]

    assert ids == [
        "forest.route.primary_loop.v1",
        "forest.route.branch_north.v1",
        "forest.route.branch_east.v1",
        "forest.route.branch_southwest.v1",
    ]
    assert len(ids) == len(set(ids))
    assert routes["spawn"]["stable_id"] == "forest.spawn.trailhead.v1"
    assert routes["goal"]["stable_id"] == "forest.goal.overlook.v1"
    assert routes["spawn"]["route_id"] == routes["goal"]["route_id"] == ids[0]
    for route in routes["routes"]:
        limits = route["navigation_limits"]
        assert limits == {
            "clearance_each_side_m": 2.0,
            "maximum_cross_slope": 0.08,
            "maximum_grade": 0.12,
            "maximum_step_m": 0.18,
            "minimum_drivable_width_m": 6.0,
        }
        assert route["road_width_m"] >= 10.0
        assert len(route["centerline_xy_m"]) >= 3
    identity = {"routes": routes["routes"], "spawn": routes["spawn"], "goal": routes["goal"]}
    digest = hashlib.sha256(
        (json.dumps(identity, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    ).hexdigest()
    assert routes["route_contract_sha256"] == digest


def test_production_grid_route_corridors_meet_declared_navigation_limits() -> None:
    routes = json.loads((PACKAGE_ROOT / "routes/forest.routes.json").read_text(encoding="utf-8"))
    spec = TerrainSpec(resolution_px=PRODUCTION_RESOLUTION_PX)

    for route in routes["routes"]:
        metrics = _route_metrics(spec, route)
        limits = route["navigation_limits"]
        assert metrics["maximum_grade"] <= limits["maximum_grade"]
        assert metrics["maximum_cross_slope"] <= limits["maximum_cross_slope"]
        assert metrics["maximum_step_m"] <= limits["maximum_step_m"]


def test_route_shoulders_fall_off_smoothly_outside_the_full_road_width() -> None:
    route = json.loads((PACKAGE_ROOT / "routes/forest.routes.json").read_text(encoding="utf-8"))["routes"][1]
    start, end = route["centerline_xy_m"][:2]
    delta_x = float(end[0]) - float(start[0])
    delta_y = float(end[1]) - float(start[1])
    length_m = math.hypot(delta_x, delta_y)
    normal_x, normal_y = -delta_y / length_m, delta_x / length_m
    center_x = (float(start[0]) + float(end[0])) / 2.0
    center_y = (float(start[1]) + float(end[1])) / 2.0
    half_width_m = float(route["road_width_m"]) / 2.0
    influences = [
        _road_influence_q16(
            center_x + normal_x * (half_width_m + shoulder_offset_m),
            center_y + normal_y * (half_width_m + shoulder_offset_m),
        )
        for shoulder_offset_m in (0.0, 4.0, 8.0, 12.0, 16.0)
    ]

    assert influences[0] == 65_536
    assert influences[-1] == 0
    assert influences == sorted(influences, reverse=True)
    edge_drop = influences[0] - influences[1]
    middle_drop = influences[1] - influences[2]
    outer_drop = influences[-2] - influences[-1]
    assert edge_drop < middle_drop
    assert outer_drop < middle_drop


def test_ue_is_visual_only_and_mujoco_owns_physics_and_raycast() -> None:
    projection = json.loads((PACKAGE_ROOT / "visual/ue_projection.json").read_text(encoding="utf-8"))
    recipe = json.loads((PACKAGE_ROOT / "terrain.recipe.json").read_text(encoding="utf-8"))

    assert projection["runtime_contract"] == {
        "classification": "VisualOnly",
        "collision_enabled": False,
        "collision_profile": "NoCollision",
        "generate_overlap_events": False,
        "physics_authority": "mujoco",
        "raycast_authority": "mujoco",
        "simulate_physics": False,
    }
    assert recipe["authority"] == {
        "physics": "mujoco",
        "raycast": "mujoco",
        "unreal": "visual_only_no_collision",
    }


def test_small_grid_generation_is_reproducible_and_hashes_exact_bytes(tmp_path: Path) -> None:
    spec = TerrainSpec(resolution_px=33)
    first = tmp_path / "first"
    second = tmp_path / "second"
    first_result = generate_forest_hf(first, spec=spec)
    second_result = generate_forest_hf(second, spec=spec)

    assert first_result.asset_set_sha256 == second_result.asset_set_sha256
    assert _generated_payloads(first) == _generated_payloads(second)

    artifact_manifest = json.loads(
        (first_result.package_root / "generated/asset-manifest.json").read_text(encoding="utf-8")
    )
    assert artifact_manifest["canonical_source"]["sample_type"] == "uint16"
    assert artifact_manifest["canonical_source"]["dimensions_px"] == [33, 33]
    for record in artifact_manifest["artifacts"]:
        payload = (first_result.package_root / record["path"]).read_bytes()
        assert record["bytes"] == len(payload)
        assert record["sha256"] == hashlib.sha256(payload).hexdigest()


def test_generated_mujoco_f32_is_a_lossless_projection_of_the_u16_source(tmp_path: Path) -> None:
    result = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=17))
    generated = result.package_root / "generated"
    source = generated / "heightfield_u16.raw"
    physics = generated / "heightfield_f32.bin"
    samples = struct.unpack(f"<{source.stat().st_size // 2}H", source.read_bytes())
    nrow, ncol = struct.unpack_from("<ii", physics.read_bytes())
    values = struct.unpack_from(f"<{nrow * ncol}f", physics.read_bytes(), 8)
    expected = [
        samples[row * ncol + column] / 65_535
        for row in reversed(range(nrow))
        for column in range(ncol)
    ]

    assert [nrow, ncol] == [17, 17]
    assert list(values) == pytest.approx(expected, abs=1e-7)
    xml = (result.package_root / "physics/forest_hf.xml").read_text(encoding="utf-8")
    assert 'file="../generated/heightfield_f32.bin"' in xml
    assert 'type="hfield"' in xml


def test_written_u16_grid_uses_the_same_route_carving_as_production_point_sampling(tmp_path: Path) -> None:
    spec = TerrainSpec(resolution_px=65)
    result = generate_forest_hf(tmp_path, spec=spec)
    source = result.package_root / "generated/heightfield_u16.raw"
    samples = struct.unpack(f"<{spec.resolution_px**2}H", source.read_bytes())
    expected = [
        _terrain_sample(spec, DEFAULT_SEED, column, row)
        for row in range(spec.resolution_px)
        for column in range(spec.resolution_px)
    ]

    assert list(samples) == expected


def test_streamed_small_grid_is_byte_equivalent_and_omits_the_unreal_obj(tmp_path: Path) -> None:
    spec = TerrainSpec(resolution_px=17)
    result = generate_forest_hf(tmp_path / "streamed", spec=spec)
    generated = result.package_root / "generated"
    expected_samples = tuple(
        _terrain_sample(spec, DEFAULT_SEED, column, row)
        for row in range(spec.resolution_px)
        for column in range(spec.resolution_px)
    )
    expected_raw = struct.pack(f"<{len(expected_samples)}H", *expected_samples)
    sequence_root = tmp_path / "sequence"
    build_heightmap_artifacts(
        samples=expected_samples,
        width=spec.resolution_px,
        height=spec.resolution_px,
        extent_m=(spec.extent_m, spec.extent_m),
        elevation_min_m=spec.elevation_min_m,
        elevation_max_m=spec.elevation_max_m,
        spawn_xy_m=(-760.0, -650.0),
        artifact_root=sequence_root,
        mesh_name="Forest_HF_2km_Terrain",
        emit_unreal_obj=False,
    )

    assert (generated / "heightfield_u16.raw").read_bytes() == expected_raw
    assert (generated / "heightfield_u16.raw").stat().st_size == spec.resolution_px**2 * 2
    assert (generated / "heightfield_f32.bin").stat().st_size == 8 + spec.resolution_px**2 * 4
    for name in ("heightfield_f32.bin", "heightfield_r16.png"):
        assert (generated / name).read_bytes() == (sequence_root / name).read_bytes()
        assert (generated / name).stat().st_size == (sequence_root / name).stat().st_size
    assert not (generated / "terrain.obj").exists()

    artifact_manifest = json.loads((generated / "asset-manifest.json").read_text(encoding="utf-8"))
    records = {record["path"]: record for record in artifact_manifest["artifacts"]}
    assert set(records) == {
        "generated/alignment-report.json",
        "generated/heightfield_f32.bin",
        "generated/heightfield_r16.png",
        "generated/heightfield_u16.raw",
    }
    assert "generated/terrain.obj" not in records
    for relative, record in records.items():
        path = result.package_root / relative
        assert record["bytes"] == path.stat().st_size
        assert record["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()


def test_generated_world_compiles_in_mujoco_when_available(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    result = generate_forest_hf(tmp_path, spec=TerrainSpec(resolution_px=17))

    model = mujoco.MjModel.from_xml_path(str(result.package_root / "physics/forest_hf.xml"))
    hfield_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "forest_terrain")
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "forest_terrain")

    assert hfield_id >= 0
    assert geom_id >= 0
    assert model.hfield_nrow[hfield_id] == model.hfield_ncol[hfield_id] == 17
    artifact_manifest = json.loads(
        (result.package_root / "generated/asset-manifest.json").read_text(encoding="utf-8")
    )
    canonical = artifact_manifest["canonical_source"]
    assert model.hfield_size[hfield_id].tolist() == pytest.approx(
        [1000.0, 1000.0, canonical["sampled_elevation_range_m"], 10.0]
    )
    assert model.geom_pos[geom_id, 2] == pytest.approx(canonical["sampled_elevation_min_m"])
    assert model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_HFIELD


def test_generated_mujoco_surface_matches_canonical_u16_heights_at_grid_points(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    numpy = pytest.importorskip("numpy")
    spec = TerrainSpec(resolution_px=17)
    result = generate_forest_hf(tmp_path, spec=spec)
    source = result.package_root / "generated/heightfield_u16.raw"
    samples = struct.unpack(f"<{spec.resolution_px**2}H", source.read_bytes())
    model = mujoco.MjModel.from_xml_path(str(result.package_root / "physics/forest_hf.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    spacing_m = spec.extent_m / (spec.resolution_px - 1)

    for row, column in ((1, 1), (4, 3), (8, 8), (12, 13), (15, 15)):
        x_m = -spec.extent_m / 2.0 + column * spacing_m
        y_m = spec.extent_m / 2.0 - row * spacing_m
        expected_z_m = (
            samples[row * spec.resolution_px + column]
            / 65_535
            * (spec.elevation_max_m - spec.elevation_min_m)
            + spec.elevation_min_m
        )
        ray_origin_z_m = spec.elevation_max_m + 10.0
        geom_id = numpy.array([-1], dtype=numpy.int32)
        distance_m = mujoco.mj_ray(
            model,
            data,
            numpy.array([x_m, y_m, ray_origin_z_m]),
            numpy.array([0.0, 0.0, -1.0]),
            None,
            True,
            -1,
            geom_id,
        )

        assert geom_id[0] >= 0
        assert ray_origin_z_m - distance_m == pytest.approx(expected_z_m, abs=1.0e-3)
