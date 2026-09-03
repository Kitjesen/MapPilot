"""Generate the deterministic 2 km Forest_HF same-source terrain package."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from dataclasses import dataclass
from functools import cache
from itertools import pairwise
from pathlib import Path
from typing import Sequence

from sim.catalog.importers.heightmap import build_heightmap_artifacts_from_u16_file

DEFAULT_SEED = 20260813
PRODUCTION_RESOLUTION_PX = 4033
WORLD_VERSION = "2.0.0"
_U16_MAX = 65_535
_U64_MASK = (1 << 64) - 1
_ROAD_SURFACE_U16 = 20_000
_ROAD_SHOULDER_WIDTH_M = 16.0
_PRODUCTION_SAMPLE_MIN_U16 = 7_125
_PRODUCTION_SAMPLE_MAX_U16 = 32_896
_PRODUCTION_MUJOCO_ORIGIN_Z_M = -10.0 + _PRODUCTION_SAMPLE_MIN_U16 / _U16_MAX * 80.0
_PRODUCTION_MUJOCO_ELEVATION_SCALE_M = (
    (_PRODUCTION_SAMPLE_MAX_U16 - _PRODUCTION_SAMPLE_MIN_U16) / _U16_MAX * 80.0
)


@dataclass(frozen=True)
class TerrainSpec:
    """Physical terrain contract; production uses an Unreal-native 4033 grid."""

    resolution_px: int = PRODUCTION_RESOLUTION_PX
    extent_m: float = 2000.0
    elevation_min_m: float = -10.0
    elevation_max_m: float = 70.0

    def validate(self) -> None:
        """Reject terrain variants that would change the package contract."""

        if (
            isinstance(self.resolution_px, bool)
            or not isinstance(self.resolution_px, int)
            or self.resolution_px < 2
        ):
            raise ValueError("resolution_px must be an integer of at least 2")
        if self.extent_m != 2000.0:
            raise ValueError("Forest_HF extent is fixed at exactly 2000 metres")
        if self.elevation_max_m <= self.elevation_min_m:
            raise ValueError("elevation range must be positive")


@dataclass(frozen=True)
class GeneratedForestWorld:
    """Paths and deterministic identity returned by generation."""

    package_root: Path
    asset_set_sha256: str | None


def _canonical_json(value: object) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, separators=(",", ": "), allow_nan=False)
        + "\n"
    ).encode("utf-8")


def _identity_json(value: object) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _splitmix64(value: int) -> int:
    value = (value + 0x9E3779B97F4A7C15) & _U64_MASK
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & _U64_MASK
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & _U64_MASK
    return value ^ (value >> 31)


def _lattice(seed: int, x: int, y: int) -> int:
    mixed = seed ^ ((x * 0xD6E8FEB86659FD93) & _U64_MASK) ^ ((y * 0xA5A3564E27F8862B) & _U64_MASK)
    return (_splitmix64(mixed) >> 48) & _U16_MAX


def _smooth_q16(value: int) -> int:
    squared = value * value // 65_536
    return squared * (196_608 - 2 * value) // 65_536


def _lerp(start: int, end: int, amount_q16: int) -> int:
    return start + (end - start) * amount_q16 // 65_536


def _value_noise(seed: int, x: int, y: int, cell_size: int) -> int:
    lattice_x, lattice_y = x // cell_size, y // cell_size
    fraction_x = (x % cell_size) * 65_536 // cell_size
    fraction_y = (y % cell_size) * 65_536 // cell_size
    north = _lerp(
        _lattice(seed, lattice_x, lattice_y),
        _lattice(seed, lattice_x + 1, lattice_y),
        _smooth_q16(fraction_x),
    )
    south = _lerp(
        _lattice(seed, lattice_x, lattice_y + 1),
        _lattice(seed, lattice_x + 1, lattice_y + 1),
        _smooth_q16(fraction_x),
    )
    return _lerp(north, south, _smooth_q16(fraction_y))


def _base_terrain_sample(spec: TerrainSpec, seed: int, column: int, row: int) -> int:
    resolution = spec.resolution_px
    coarse = max(2, (resolution - 1) // 8)
    medium = max(2, (resolution - 1) // 24)
    broad = _value_noise(seed ^ 0xF012E57, column, row, coarse) - 32_768
    detail = _value_noise(seed ^ 0xBADC0DE, column, row, medium) - 32_768
    # Keep the 2 km navigation terrain rolling rather than mountainous;
    # the full encoded vertical range remains explicit in the contract.
    value = 20_000 + broad * 11_000 // 32_768 + detail * 2_500 // 32_768
    return max(2_048, min(45_000, value))


@cache
def _route_segments() -> tuple[tuple[float, float, float, float, float], ...]:
    segments = []
    for route in _route_contract()["routes"]:
        half_width_m = float(route["road_width_m"]) / 2.0
        centerline = route["centerline_xy_m"]
        for start, end in pairwise(centerline):
            segments.append((float(start[0]), float(start[1]), float(end[0]), float(end[1]), half_width_m))
    return tuple(segments)


def _point_segment_distance_squared(
    x_m: float,
    y_m: float,
    start_x_m: float,
    start_y_m: float,
    end_x_m: float,
    end_y_m: float,
) -> float:
    delta_x = end_x_m - start_x_m
    delta_y = end_y_m - start_y_m
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared == 0.0:
        return (x_m - start_x_m) ** 2 + (y_m - start_y_m) ** 2
    amount = max(
        0.0,
        min(1.0, ((x_m - start_x_m) * delta_x + (y_m - start_y_m) * delta_y) / length_squared),
    )
    nearest_x = start_x_m + amount * delta_x
    nearest_y = start_y_m + amount * delta_y
    return (x_m - nearest_x) ** 2 + (y_m - nearest_y) ** 2


def _corridor_influence_q16(distance_squared_m: float, half_width_m: float) -> int:
    if distance_squared_m <= half_width_m * half_width_m:
        return 65_536
    outer_radius_m = half_width_m + _ROAD_SHOULDER_WIDTH_M
    if distance_squared_m >= outer_radius_m * outer_radius_m:
        return 0
    distance_m = math.sqrt(distance_squared_m)
    shoulder_amount_q16 = round((distance_m - half_width_m) / _ROAD_SHOULDER_WIDTH_M * 65_536)
    return 65_536 - _smooth_q16(max(0, min(65_536, shoulder_amount_q16)))


def _road_influence_q16(x_m: float, y_m: float) -> int:
    influence = 0
    for start_x, start_y, end_x, end_y, half_width_m in _route_segments():
        distance_squared_m = _point_segment_distance_squared(
            x_m,
            y_m,
            start_x,
            start_y,
            end_x,
            end_y,
        )
        influence = max(influence, _corridor_influence_q16(distance_squared_m, half_width_m))
        if influence == 65_536:
            break
    return influence


def _terrain_sample(spec: TerrainSpec, seed: int, column: int, row: int) -> int:
    """Return one canonical grid sample without allocating the production grid."""

    spacing_m = spec.extent_m / (spec.resolution_px - 1)
    x_m = -spec.extent_m / 2.0 + column * spacing_m
    y_m = spec.extent_m / 2.0 - row * spacing_m
    base = _base_terrain_sample(spec, seed, column, row)
    return _lerp(base, _ROAD_SURFACE_U16, _road_influence_q16(x_m, y_m))


def _terrain_row_samples(spec: TerrainSpec, seed: int, row: int) -> tuple[int, ...]:
    """Build one canonical north-to-south row while bounding route work to that row."""

    resolution = spec.resolution_px
    spacing_m = spec.extent_m / (resolution - 1)
    half_extent_m = spec.extent_m / 2.0
    y_m = half_extent_m - row * spacing_m
    influences = [0] * resolution
    for start_x, start_y, end_x, end_y, half_width_m in _route_segments():
        outer_radius_m = half_width_m + _ROAD_SHOULDER_WIDTH_M
        if y_m < min(start_y, end_y) - outer_radius_m or y_m > max(start_y, end_y) + outer_radius_m:
            continue
        minimum_x = min(start_x, end_x) - outer_radius_m
        maximum_x = max(start_x, end_x) + outer_radius_m
        minimum_column = max(0, math.floor((minimum_x + half_extent_m) / spacing_m))
        maximum_column = min(resolution - 1, math.ceil((maximum_x + half_extent_m) / spacing_m))
        for column in range(minimum_column, maximum_column + 1):
            x_m = -half_extent_m + column * spacing_m
            influence = _corridor_influence_q16(
                _point_segment_distance_squared(
                    x_m,
                    y_m,
                    start_x,
                    start_y,
                    end_x,
                    end_y,
                ),
                half_width_m,
            )
            if influence > influences[column]:
                influences[column] = influence
    return tuple(
        _lerp(_base_terrain_sample(spec, seed, column, row), _ROAD_SURFACE_U16, influences[column])
        for column in range(resolution)
    )


def _write_u16_source(path: Path, *, spec: TerrainSpec, seed: int) -> None:
    """Stream the canonical little-endian u16 grid one bounded row at a time."""

    row_struct = struct.Struct(f"<{spec.resolution_px}H")
    with path.open("wb") as stream:
        for row in range(spec.resolution_px):
            stream.write(row_struct.pack(*_terrain_row_samples(spec, seed, row)))


def _route_contract() -> dict[str, object]:
    limits = {
        "minimum_drivable_width_m": 6.0,
        "maximum_grade": 0.12,
        "maximum_cross_slope": 0.08,
        "maximum_step_m": 0.18,
        "clearance_each_side_m": 2.0,
    }
    routes = [
        {
            "stable_id": "forest.route.primary_loop.v1",
            "kind": "primary_loop",
            "road_width_m": 12.0,
            "centerline_xy_m": [
                [-760.0, -650.0], [-250.0, -790.0], [360.0, -720.0], [760.0, -340.0],
                [790.0, 260.0], [420.0, 710.0], [-180.0, 800.0], [-700.0, 500.0],
                [-820.0, -80.0], [-760.0, -650.0],
            ],
            "navigation_limits": limits,
        },
        {
            "stable_id": "forest.route.branch_north.v1",
            "kind": "branch",
            "road_width_m": 10.0,
            "centerline_xy_m": [[-180.0, 800.0], [-120.0, 900.0], [40.0, 950.0]],
            "navigation_limits": limits,
        },
        {
            "stable_id": "forest.route.branch_east.v1",
            "kind": "branch",
            "road_width_m": 10.0,
            "centerline_xy_m": [[790.0, 260.0], [890.0, 360.0], [950.0, 500.0]],
            "navigation_limits": limits,
        },
        {
            "stable_id": "forest.route.branch_southwest.v1",
            "kind": "branch",
            "road_width_m": 10.0,
            "centerline_xy_m": [[-760.0, -650.0], [-880.0, -760.0], [-950.0, -850.0]],
            "navigation_limits": limits,
        },
    ]
    spawn = {
        "stable_id": "forest.spawn.trailhead.v1",
        "route_id": "forest.route.primary_loop.v1",
        "position_xy_m": [-760.0, -650.0],
        "heading_deg": -15.0,
        "minimum_clear_radius_m": 8.0,
    }
    goal = {
        "stable_id": "forest.goal.overlook.v1",
        "route_id": "forest.route.primary_loop.v1",
        "position_xy_m": [420.0, 710.0],
        "acceptance_radius_m": 5.0,
    }
    identity = {"routes": routes, "spawn": spawn, "goal": goal}
    return {
        "schema": "lingtu.sim.forest-route-contract.v1",
        "world_package": "forest_hf@2.0.0",
        "extent_m": [2000.0, 2000.0],
        **identity,
        "route_contract_sha256": _sha256(_identity_json(identity)),
    }


def _terrain_recipe(routes: dict[str, object]) -> dict[str, object]:
    contract = {
        "extent_m": [2000.0, 2000.0],
        "source_grid": {
            "dimensions_px": [PRODUCTION_RESOLUTION_PX, PRODUCTION_RESOLUTION_PX],
            "sample_type": "uint16",
            "endianness": "little",
            "row_order": "north_to_south",
        },
        "elevation_range_m": [-10.0, 70.0],
        "canonical_source": "generated/heightfield_u16.raw",
        "route_contract_sha256": routes["route_contract_sha256"],
    }
    return {
        "schema": "lingtu.sim.forest-terrain-recipe.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": DEFAULT_SEED,
        "generator": "sim.tools.worlds.forest_hf.generate",
        **contract,
        "facets": {
            "mujoco": {
                "format": "mujoco_hfield_f32",
                "path": "generated/heightfield_f32.bin",
                "row_order": "south_to_north",
                "normalization": {
                    "compiler_behavior": "affine_sample_min_max_to_unit_interval",
                    "production_sample_min_u16": _PRODUCTION_SAMPLE_MIN_U16,
                    "production_sample_max_u16": _PRODUCTION_SAMPLE_MAX_U16,
                    "geom_origin_z_m": _PRODUCTION_MUJOCO_ORIGIN_Z_M,
                    "hfield_elevation_scale_m": _PRODUCTION_MUJOCO_ELEVATION_SCALE_M,
                },
            },
            "unreal": {
                "format": "ue_landscape_r16_png",
                "heightmap": "generated/heightfield_r16.png",
                "actor_type": "Landscape",
                "world_partition": True,
                "components_xy": [64, 64],
                "sections_per_component": 1,
                "quads_per_section": 63,
                "scale_cm": [200000.0 / 4032.0, 200000.0 / 4032.0, 15.625],
                "location_z_cm": 3000.0,
                "axis_mapping": ["x", "-y", "z"],
            },
        },
        "authority": {"physics": "mujoco", "raycast": "mujoco", "unreal": "visual_only_no_collision"},
        "production_contract_sha256": _sha256(_identity_json(contract)),
    }


def _ue_projection(recipe: dict[str, object]) -> dict[str, object]:
    return {
        "schema": "lingtu.sim.forest-ue-projection.v1",
        "world_package": "forest_hf@2.0.0",
        "binding": "WorldVisual:ForestHF",
        "target_level": "/Game/RobotSim/Maps/Forest_HF_2km",
        "terrain": {
            "source_heightmap": "generated/heightfield_r16.png",
            "actor_type": "Landscape",
            "world_partition": True,
            "source_resolution_px": [PRODUCTION_RESOLUTION_PX, PRODUCTION_RESOLUTION_PX],
            "extent_cm": [200000.0, 200000.0],
            "components_xy": [64, 64],
            "sections_per_component": 1,
            "quads_per_section": 63,
            "scale_cm": [200000.0 / 4032.0, 200000.0 / 4032.0, 15.625],
            "location_z_cm": 3000.0,
            "axis_mapping": ["x", "-y", "z"],
            "production_contract_sha256": recipe["production_contract_sha256"],
        },
        "runtime_contract": {
            "classification": "VisualOnly",
            "collision_profile": "NoCollision",
            "collision_enabled": False,
            "simulate_physics": False,
            "generate_overlap_events": False,
            "physics_authority": "mujoco",
            "raycast_authority": "mujoco",
        },
    }


def _world_manifest(
    *,
    records: Sequence[dict[str, object]],
) -> bytes:
    lines = [
        "schema: lingtu.sim.world-package.v1",
        "id: forest_hf",
        "version: 2.0.0",
        "kind: world",
        "description: Deterministic 2 km same-source forest terrain with MuJoCo authority.",
        "physics:",
        "  mjcf: physics/forest_hf.xml",
        "  global_policy:",
        "    timestep_s: 0.001",
        "    integrator: rk4",
        "    solver: newton",
        "    iterations: 100",
        "    gravity_mps2: [0.0, 0.0, -9.81]",
        "visual:",
        "  binding: WorldVisual:ForestHF",
        "  level: /Game/RobotSim/Maps/Forest_HF_2km",
        "  projection: visual/world.visual-projection.json",
        "entities: []",
        "provenance:",
        "  path: provenance/generation.json",
        "content:",
        "  files:",
    ]
    for record in records:
        lines.extend(
            (
                f"  - path: {record['path']}",
                f"    size: {record['size']}",
                f"    sha256: {record['sha256']}",
            )
        )
    lines.extend(
        (
            "  provenance:",
            "    path: provenance/generation.json",
            "  visual_projection:",
            "    path: visual/world.visual-projection.json",
        )
    )
    return ("\n".join(lines) + "\n").encode()


def _mjcf(
    *,
    elevation_origin_m: float = _PRODUCTION_MUJOCO_ORIGIN_Z_M,
    elevation_scale_m: float = _PRODUCTION_MUJOCO_ELEVATION_SCALE_M,
) -> bytes:
    return (
        '<mujoco model="forest_hf_2_0_0">\n'
        '  <compiler angle="radian" autolimits="true"/>\n'
        '  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4" solver="Newton" iterations="100"/>\n'
        "  <asset>\n"
        '    <hfield name="forest_terrain" file="../generated/heightfield_f32.bin" '
        "content_type=\"image/vnd.mujoco.hfield\" "
        f'size="1000 1000 {elevation_scale_m:.17g} 10"/>\n'
        "  </asset>\n"
        "  <worldbody>\n"
        '    <geom name="forest_terrain" type="hfield" hfield="forest_terrain" '
        f'pos="0 0 {elevation_origin_m:.17g}" '
        'contype="1" conaffinity="1" condim="3" friction="0.9 0.02 0.001"/>\n'
        "  </worldbody>\n"
        "</mujoco>\n"
    ).encode()


def _static_payloads() -> dict[Path, bytes]:
    routes = _route_contract()
    recipe = _terrain_recipe(routes)
    projection = _ue_projection(recipe)
    provenance = {
        "schema": "lingtu.sim.procedural-world-provenance.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": DEFAULT_SEED,
        "owner": "LingTu project",
        "license": "LicenseRef-LingTu-Project-Owned",
        "third_party_assets": [],
        "route_contract_sha256": routes["route_contract_sha256"],
        "production_contract_sha256": recipe["production_contract_sha256"],
        "generated_binaries_checked_in": False,
    }
    return {
        Path("physics/forest_hf.xml"): _mjcf(),
        Path("routes/forest.routes.json"): _canonical_json(routes),
        Path("terrain.recipe.json"): _canonical_json(recipe),
        Path("visual/ue_projection.json"): _canonical_json(projection),
        Path("provenance/generation.json"): _canonical_json(provenance),
    }


def _file_record(package_root: Path, relative: str) -> dict[str, object]:
    path = package_root / relative
    return {"path": relative, "size": path.stat().st_size, "sha256": _sha256_file(path)}


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _package_records(package_root: Path, *, excluded: set[str]) -> list[dict[str, object]]:
    return [
        _file_record(package_root, path.relative_to(package_root).as_posix())
        for path in sorted(package_root.rglob("*"))
        if path.is_file() and path.relative_to(package_root).as_posix() not in excluded
    ]


def _projection_asset(package_root: Path, relative: str, role: str, *, collision: bool) -> dict[str, object]:
    record = _file_record(package_root, relative)
    return {
        "role": role,
        "path": record["path"],
        "bytes": record["size"],
        "collision": collision,
    }


def _spawn_height_m(spec: TerrainSpec, seed: int) -> float:
    column = round((-760.0 + spec.extent_m / 2.0) / (spec.extent_m / (spec.resolution_px - 1)))
    row = round((spec.extent_m / 2.0 - (-650.0)) / (spec.extent_m / (spec.resolution_px - 1)))
    sample = _terrain_sample(spec, seed, column, row)
    return sample / _U16_MAX * (spec.elevation_max_m - spec.elevation_min_m) + spec.elevation_min_m


def _write_canonical_package(
    package_root: Path,
    *,
    spec: TerrainSpec,
    projection_assets: Sequence[dict[str, object]],
) -> None:
    manifest_path = "world.package.yaml"
    projection_path = "visual/world.visual-projection.json"
    spacing_m = spec.extent_m / (spec.resolution_px - 1)
    bounds = {
        "min_m": [-spec.extent_m / 2.0, -spec.extent_m / 2.0, spec.elevation_min_m],
        "max_m": [spec.extent_m / 2.0, spec.extent_m / 2.0, spec.elevation_max_m],
    }
    projection_body = {
        "schema": "lingtu.sim.world-visual-projection.v1",
        "package": {
            "id": "forest_hf",
            "version": WORLD_VERSION,
            "manifest": manifest_path,
            "provenance": "provenance/generation.json",
        },
        "binding": "WorldVisual:ForestHF",
        "level": "/Game/RobotSim/Maps/Forest_HF_2km",
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "terrain": {
            "grid_px": [spec.resolution_px, spec.resolution_px],
            "extent_m": [spec.extent_m, spec.extent_m],
            "sample_spacing_m": [spacing_m, spacing_m],
            "physics_bounds_m": bounds,
            "visual_bounds_m": bounds,
            "assets": list(projection_assets),
        },
        "entities": [],
        "spawn_alignment": {
            "position_m": [-760.0, -650.0, _spawn_height_m(spec, DEFAULT_SEED)],
            "aligned_to_heightmap": True,
        },
    }
    projection_target = package_root / projection_path
    projection_target.parent.mkdir(parents=True, exist_ok=True)
    projection_target.write_bytes(_canonical_json(projection_body))
    records = _package_records(package_root, excluded={manifest_path})
    (package_root / manifest_path).write_bytes(
        _world_manifest(records=records)
    )


def generate_forest_hf(
    repo_root: Path | str,
    *,
    seed: int = DEFAULT_SEED,
    spec: TerrainSpec = TerrainSpec(),
    emit_artifacts: bool = True,
) -> GeneratedForestWorld:
    """Generate Forest_HF metadata and, unless disabled, same-source facets."""

    if seed != DEFAULT_SEED:
        raise ValueError(f"Forest_HF 2.0.0 seed is fixed at {DEFAULT_SEED}")
    spec.validate()
    package_root = Path(repo_root).resolve() / "sim/packages/worlds/forest_hf/2.0.0"
    for relative, payload in _static_payloads().items():
        target = package_root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    if not emit_artifacts:
        projection_assets = (
            _projection_asset(package_root, "terrain.recipe.json", "canonical_u16_source_contract", collision=False),
            _projection_asset(package_root, "physics/forest_hf.xml", "mujoco_f32_projection_contract", collision=True),
            _projection_asset(package_root, "visual/ue_projection.json", "unreal_visual_projection_contract", collision=False),
        )
        _write_canonical_package(package_root, spec=spec, projection_assets=projection_assets)
        return GeneratedForestWorld(package_root=package_root, asset_set_sha256=None)

    generated_root = package_root / "generated"
    generated_root.mkdir(parents=True, exist_ok=True)
    source_path = generated_root / "heightfield_u16.raw"
    _write_u16_source(source_path, spec=spec, seed=seed)
    artifacts = build_heightmap_artifacts_from_u16_file(
        source_path=source_path,
        width=spec.resolution_px,
        height=spec.resolution_px,
        extent_m=(spec.extent_m, spec.extent_m),
        elevation_min_m=spec.elevation_min_m,
        elevation_max_m=spec.elevation_max_m,
        spawn_xy_m=(-760.0, -650.0),
        artifact_root=generated_root,
        mesh_name="Forest_HF_2km_Terrain",
        endian="little",
        emit_unreal_obj=False,
    )
    if spec.resolution_px == PRODUCTION_RESOLUTION_PX and (
        artifacts.sample_min_u16 != _PRODUCTION_SAMPLE_MIN_U16
        or artifacts.sample_max_u16 != _PRODUCTION_SAMPLE_MAX_U16
    ):
        raise RuntimeError(
            "Forest_HF production terrain extrema changed; refresh the metadata-only MuJoCo normalization contract"
        )
    (package_root / "physics/forest_hf.xml").write_bytes(
        _mjcf(
            elevation_origin_m=artifacts.sampled_elevation_min_m,
            elevation_scale_m=artifacts.mujoco_elevation_scale_m,
        )
    )
    (generated_root / "alignment-report.json").write_bytes(_canonical_json(artifacts.alignment))
    records = []
    for path in sorted(generated_root.iterdir(), key=lambda item: item.name):
        if path.name == "asset-manifest.json" or not path.is_file():
            continue
        records.append(
            {
                "path": f"generated/{path.name}",
                "bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        )
    identity = [{"path": record["path"], "sha256": record["sha256"]} for record in records]
    asset_set_sha256 = _sha256(_identity_json(identity))
    manifest = {
        "schema": "lingtu.sim.forest-asset-manifest.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": seed,
        "validation_resolution": spec.resolution_px != PRODUCTION_RESOLUTION_PX,
        "canonical_source": {
            "path": "generated/heightfield_u16.raw",
            "sample_type": "uint16",
            "endianness": "little",
            "dimensions_px": [spec.resolution_px, spec.resolution_px],
            "extent_m": [spec.extent_m, spec.extent_m],
            "sample_min_u16": artifacts.sample_min_u16,
            "sample_max_u16": artifacts.sample_max_u16,
            "sampled_elevation_min_m": artifacts.sampled_elevation_min_m,
            "sampled_elevation_max_m": artifacts.sampled_elevation_max_m,
            "sampled_elevation_range_m": artifacts.sampled_elevation_range_m,
            "mujoco_elevation_scale_m": artifacts.mujoco_elevation_scale_m,
        },
        "artifacts": records,
        "asset_set_sha256": asset_set_sha256,
    }
    (generated_root / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    projection_assets = (
        _projection_asset(package_root, "generated/heightfield_u16.raw", "canonical_u16_source", collision=False),
        _projection_asset(package_root, "generated/heightfield_f32.bin", "mujoco_heightfield", collision=True),
        _projection_asset(package_root, "generated/heightfield_r16.png", "unreal_heightfield", collision=False),
    )
    _write_canonical_package(package_root, spec=spec, projection_assets=projection_assets)
    return GeneratedForestWorld(package_root=package_root, asset_set_sha256=asset_set_sha256)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--resolution", type=int, default=PRODUCTION_RESOLUTION_PX)
    parser.add_argument("--metadata-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the package generator command-line interface."""

    args = _build_parser().parse_args(argv)
    result = generate_forest_hf(
        args.repo_root,
        spec=TerrainSpec(resolution_px=args.resolution),
        emit_artifacts=not args.metadata_only,
    )
    print(json.dumps({"package_root": str(result.package_root), "asset_set_sha256": result.asset_set_sha256}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
