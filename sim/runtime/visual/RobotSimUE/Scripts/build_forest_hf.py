"""Emit the deterministic Forest_HF 2.0 Unreal editor authoring contract.

The contract binds the canonical UE projection, terrain recipe, and routes to a
fixed-seed, per-authoring-density-cell offline HISM foliage bake. MuJoCo remains
the sole physics and raycast authority; every proposed Unreal actor is
VisualOnly/NoCollision.

Editor mutation is intentionally delegated to ``build_forest_hf_mcp.py``.
This offline emitter remains side-effect free and fails closed if invoked as
an Unreal editor mutation script directly.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import re
import struct
from pathlib import Path
from typing import Any, Mapping, Sequence

try:  # Unreal is optional for validation and structural tests.
    import unreal  # type: ignore[import-not-found]
except ModuleNotFoundError:  # pragma: no cover - normal Python path.
    unreal = None


REPO_ROOT = Path(__file__).resolve().parents[5]
PACKAGE_ROOT = REPO_ROOT / "sim" / "packages" / "worlds" / "forest_hf" / "2.0.0"
UE_PROJECTION_PATH = PACKAGE_ROOT / "visual" / "ue_projection.json"
TERRAIN_RECIPE_PATH = PACKAGE_ROOT / "terrain.recipe.json"
ROUTES_PATH = PACKAGE_ROOT / "routes" / "forest.routes.json"
DEFAULT_CONTRACT_PATH = REPO_ROOT / "build" / "forest-hf-2km" / "unreal" / "forest-hf.bake.json"

MAP_PATH = "/Game/RobotSim/Maps/Forest_HF_2km"
WORLD_PACKAGE = "forest_hf@2.0.0"
WORLD_BINDING = "WorldVisual:ForestHF"
WORLD_ASSET_ROOT = "/Game/RobotSim/Worlds/ForestHF2km"
PHYSICS_AUTHORITY = "mujoco"
PCG_AUTHORING_MODE = "offline_baked_hism"
FIXED_SEED = 20260813
CREATE_POINTS_SCHEMA = "lingtu.sim.unreal-forest-create-points.v2"
EXACT_MESH_SLOT_SELECTION_POLICY = "exact_mesh_slot_branches"
WORLD_SIZE_M = 2000.0
AUTHORING_DENSITY_CELL_SIZE_M = 100.0
AUTHORING_DENSITY_GRID_SIZE = 20
UE_PCG_PARTITION_CELL_SIZE_M = 256.0
UE_PCG_PARTITION_GRID_SIZE = 8
WORLD_MIN_M = -1000.0
WORLD_MAX_M = 1000.0
FOLIAGE_CANDIDATES_PER_AUTHORING_CELL = 48
MOTHER_TREE_HEIGHT_M = 2.0
FOREST_TREE_TARGET_HEIGHT_M = 9.0
TREE_SCALE_FACTOR = FOREST_TREE_TARGET_HEIGHT_M / MOTHER_TREE_HEIGHT_M

_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_TREE_ASSET_SLOTS = ("forest.asset.birch", "forest.asset.pine")
_KNOWN_ASSET_SLOTS = frozenset(("forest.asset.birch", "forest.asset.boulder", "forest.asset.pine"))


def _validate_tree_asset_slots() -> None:
    unknown = set(_TREE_ASSET_SLOTS) - _KNOWN_ASSET_SLOTS
    if unknown:
        raise ValueError(f"Forest_HF tree asset slots are not present in the Blender asset library: {sorted(unknown)}")


def canonical_json_bytes(value: object) -> bytes:
    """Return indented canonical JSON used for byte-stable evidence files."""

    return (
        json.dumps(value, sort_keys=True, indent=2, separators=(",", ": "), allow_nan=False)
        + "\n"
    ).encode("utf-8")


def identity_digest(value: object) -> str:
    """Hash compact canonical JSON with the generator's trailing-newline contract."""

    payload = (
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _content_digest(value: object) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"could not load Forest_HF input {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeError(f"Forest_HF JSON root must be an object: {path}")
    return value


def _finite_vector(value: object, size: int, field: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != size:
        raise ValueError(f"{field} must contain exactly {size} numbers")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{field} must contain finite numbers")
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{field} must contain finite numbers")
        result.append(number)
    return tuple(result)


def source_to_unreal_location_cm(position_m: Sequence[float]) -> tuple[float, float, float]:
    """Convert MuJoCo RH Z-up metres to Unreal LH Z-up centimetres."""

    x, y, z = _finite_vector(position_m, 3, "position_m")
    return (100.0 * x, -100.0 * y, 100.0 * z)


def _validate_sources(
    projection: Mapping[str, object],
    terrain: Mapping[str, object],
    routes: Mapping[str, object],
) -> dict[str, str]:
    expected_schemas = {
        "projection": "lingtu.sim.forest-ue-projection.v1",
        "terrain": "lingtu.sim.forest-terrain-recipe.v1",
        "routes": "lingtu.sim.forest-route-contract.v1",
    }
    for name, document in (("projection", projection), ("terrain", terrain), ("routes", routes)):
        if document.get("schema") != expected_schemas[name]:
            raise ValueError(f"Forest_HF {name} schema/version is unsupported")
        if document.get("world_package") != WORLD_PACKAGE:
            raise ValueError(f"Forest_HF {name} must target exact package {WORLD_PACKAGE}")
    if projection.get("target_level") != MAP_PATH or projection.get("binding") != WORLD_BINDING:
        raise ValueError("Forest_HF UE projection targets the wrong map or binding")
    if terrain.get("seed") != FIXED_SEED:
        raise ValueError(f"Forest_HF seed must remain fixed at {FIXED_SEED}")
    if terrain.get("extent_m") != [WORLD_SIZE_M, WORLD_SIZE_M] or routes.get("extent_m") != [
        WORLD_SIZE_M,
        WORLD_SIZE_M,
    ]:
        raise ValueError("Forest_HF terrain/routes must declare the exact 2 km extent")

    route_identity = {name: routes.get(name) for name in ("routes", "spawn", "goal")}
    actual_route_digest = identity_digest(route_identity)
    if routes.get("route_contract_sha256") != actual_route_digest:
        raise ValueError("Forest_HF routes digest does not cover the current route content")
    if terrain.get("route_contract_sha256") != actual_route_digest:
        raise ValueError("Forest_HF terrain recipe is not bound to the current routes")
    terrain_identity = {
        name: terrain.get(name)
        for name in (
            "extent_m",
            "source_grid",
            "elevation_range_m",
            "canonical_source",
            "route_contract_sha256",
        )
    }
    actual_production_digest = identity_digest(terrain_identity)
    if terrain.get("production_contract_sha256") != actual_production_digest:
        raise ValueError("Forest_HF terrain production digest is invalid")
    projection_terrain = projection.get("terrain")
    if not isinstance(projection_terrain, Mapping):
        raise ValueError("Forest_HF UE projection terrain contract is missing")
    if projection_terrain.get("production_contract_sha256") != actual_production_digest:
        raise ValueError("Forest_HF UE projection is not bound to the terrain recipe")
    if projection_terrain.get("axis_mapping") != ["x", "-y", "z"]:
        raise ValueError("Forest_HF UE projection has the wrong coordinate mapping")
    if projection_terrain.get("extent_cm") != [200000.0, 200000.0]:
        raise ValueError("Forest_HF UE projection has the wrong centimetre extent")
    runtime = projection.get("runtime_contract")
    if not isinstance(runtime, Mapping) or runtime != {
        "classification": "VisualOnly",
        "collision_enabled": False,
        "collision_profile": "NoCollision",
        "generate_overlap_events": False,
        "physics_authority": "mujoco",
        "raycast_authority": "mujoco",
        "simulate_physics": False,
    }:
        raise ValueError("Forest_HF UE runtime authority contract is not fail-closed")
    return {
        "ue_projection": _content_digest(projection),
        "terrain_recipe": _content_digest(terrain),
        "routes": _content_digest(routes),
        "production_contract": actual_production_digest,
        "route_contract": actual_route_digest,
    }


def _stable_unit(seed: int, identity: str) -> float:
    raw = hashlib.sha256(f"{seed}:{identity}".encode()).digest()[:8]
    return int.from_bytes(raw, "big") / float(2**64 - 1)


def _distance_to_segment(
    point: tuple[float, float], start: Sequence[float], end: Sequence[float]
) -> float:
    ax, ay = _finite_vector(start, 2, "route start")
    bx, by = _finite_vector(end, 2, "route end")
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, (ax, ay))
    ratio = min(1.0, max(0.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / length_squared))
    return math.dist(point, (ax + ratio * dx, ay + ratio * dy))


def _route_clear(point: tuple[float, float], routes: Mapping[str, object]) -> bool:
    for route in routes["routes"]:  # type: ignore[index]
        width = float(route["road_width_m"]) / 2.0 + 2.0
        points = route["centerline_xy_m"]
        if any(
            _distance_to_segment(point, a, b) < width
            for a, b in itertools.pairwise(points)
        ):
            return False
    for marker, radius_name in ((routes["spawn"], "minimum_clear_radius_m"), (routes["goal"], "acceptance_radius_m")):
        if math.dist(point, tuple(marker["position_xy_m"])) < float(marker[radius_name]) + 2.0:
            return False
    return True


def _authoring_cell_instances(
    cell_x: int, cell_y: int, routes: Mapping[str, object]
) -> list[dict[str, object]]:
    _validate_tree_asset_slots()
    instances: list[dict[str, object]] = []
    cell_min_x = WORLD_MIN_M + cell_x * AUTHORING_DENSITY_CELL_SIZE_M
    cell_min_y = WORLD_MIN_M + cell_y * AUTHORING_DENSITY_CELL_SIZE_M
    for slot in range(FOLIAGE_CANDIDATES_PER_AUTHORING_CELL):
        identity = f"cell:{cell_x}:{cell_y}:slot:{slot}"
        x = cell_min_x + 5.0 + 90.0 * _stable_unit(FIXED_SEED, identity + ":x")
        y = cell_min_y + 5.0 + 90.0 * _stable_unit(FIXED_SEED, identity + ":y")
        if not _route_clear((x, y), routes):
            continue
        mesh_slot = _TREE_ASSET_SLOTS[
            int(_stable_unit(FIXED_SEED, identity + ":species") * len(_TREE_ASSET_SLOTS))
        ]
        yaw = 360.0 * _stable_unit(FIXED_SEED, identity + ":yaw")
        scale = 0.8 + 0.4 * _stable_unit(FIXED_SEED, identity + ":scale")
        stable_id = f"forest.foliage.x{cell_x:02d}.y{cell_y:02d}.{slot:02d}"
        instances.append(
            {
                "source_stable_id": stable_id,
                "stable_entity_id": f"forest.ue.{hashlib.sha256(stable_id.encode()).hexdigest()[:20]}",
                "mesh_slot": mesh_slot,
                "unreal_transform": {
                    "location_cm": list(source_to_unreal_location_cm((x, y, 0.0))),
                    "rotation_deg": [0.0, 0.0, -yaw],
                    "scale_xyz": [scale, scale, scale],
                },
            }
        )
    return instances


def build_offline_bake_contract(
    projection: Mapping[str, object],
    terrain: Mapping[str, object],
    routes: Mapping[str, object],
) -> dict[str, object]:
    """Build deterministic density-authoring HISM metadata from canonical inputs."""

    source_digests = _validate_sources(projection, terrain, routes)
    authoring_cells: list[dict[str, object]] = []
    for cell_x in range(AUTHORING_DENSITY_GRID_SIZE):
        for cell_y in range(AUTHORING_DENSITY_GRID_SIZE):
            instances = _authoring_cell_instances(cell_x, cell_y, routes)
            groups: list[dict[str, object]] = []
            for mesh_slot in sorted({str(item["mesh_slot"]) for item in instances}):
                group_instances = [item for item in instances if item["mesh_slot"] == mesh_slot]
                groups.append(
                    {
                        "stable_id": f"forest.hism.x{cell_x:02d}.y{cell_y:02d}.{mesh_slot.rsplit('.', 1)[-1]}",
                        "mesh_slot": mesh_slot,
                        "component_class": "HierarchicalInstancedStaticMeshComponent",
                        "classification": "VisualOnly",
                        "collision_profile": "NoCollision",
                        "collision_enabled": False,
                        "generate_overlap_events": False,
                        "simulate_physics": False,
                        "can_ever_affect_navigation": False,
                        "mobility": "Static",
                        "nanite": {"policy": "asset_opt_in_after_validation", "required": False},
                        "lod": {"required": True, "policy": "source_lod_chain_or_validated_nanite"},
                        "cull_distance_cm": [2500.0, 12000.0],
                        "instances": group_instances,
                    }
                )
            authoring_cells.append(
                {
                    "stable_id": f"forest.cell.x{cell_x:02d}.y{cell_y:02d}",
                    "grid_contract": "deterministic_density_authoring",
                    "grid_index": [cell_x, cell_y],
                    "data_layer": "Forest_HF_2km_OfflineBakedFoliage",
                    "hism_groups": groups,
                }
            )
    accepted_point_count = sum(
        len(group["instances"])
        for cell in authoring_cells
        for group in cell["hism_groups"]  # type: ignore[index]
    )
    body: dict[str, object] = {
        "schema": "lingtu.sim.unreal-forest-offline-bake.v2",
        "world_package": WORLD_PACKAGE,
        "world_binding": WORLD_BINDING,
        "seed": FIXED_SEED,
        "generation": {
            "mode": PCG_AUTHORING_MODE,
            "deterministic": True,
            "runtime_generation": False,
            "per_authoring_density_cell_bake": True,
        },
        "tree_scale_contract": {
            "mother_mesh_height_m": MOTHER_TREE_HEIGHT_M,
            "forest_target_height_m": FOREST_TREE_TARGET_HEIGHT_M,
            "pcg_scale_factor": TREE_SCALE_FACTOR,
            "application": "multiply_existing_instance_scale_variation",
        },
        "foliage_density_contract": {
            "authoring_grid": {
                "role": "deterministic_density_sampling_only",
                "cell_size_m": AUTHORING_DENSITY_CELL_SIZE_M,
                "grid_size": [AUTHORING_DENSITY_GRID_SIZE, AUTHORING_DENSITY_GRID_SIZE],
                "cell_count": AUTHORING_DENSITY_GRID_SIZE**2,
                "controls_ue_partitioning": False,
            },
            "candidate_points_per_authoring_cell": FOLIAGE_CANDIDATES_PER_AUTHORING_CELL,
            "candidate_point_budget": FOLIAGE_CANDIDATES_PER_AUTHORING_CELL
            * AUTHORING_DENSITY_GRID_SIZE**2,
            "accepted_point_count": accepted_point_count,
            "rejected_point_count": FOLIAGE_CANDIDATES_PER_AUTHORING_CELL
            * AUTHORING_DENSITY_GRID_SIZE**2
            - accepted_point_count,
            "accepted_points_per_square_kilometre": accepted_point_count
            / (WORLD_SIZE_M / 1000.0) ** 2,
            "rejection_policy": "route_spawn_goal_clearance",
        },
        "target": {
            "map": MAP_PATH,
            "asset_root": WORLD_ASSET_ROOT,
            "world_partition": {
                "enabled": True,
                "world_size_m": WORLD_SIZE_M,
                "pcg_partition": {
                    "role": "ue_open_world_pcg_partition",
                    "cell_size_m": UE_PCG_PARTITION_CELL_SIZE_M,
                    "grid_size": [UE_PCG_PARTITION_GRID_SIZE, UE_PCG_PARTITION_GRID_SIZE],
                    "cell_count": UE_PCG_PARTITION_GRID_SIZE**2,
                    "coverage_size_m": [
                        UE_PCG_PARTITION_CELL_SIZE_M * UE_PCG_PARTITION_GRID_SIZE,
                        UE_PCG_PARTITION_CELL_SIZE_M * UE_PCG_PARTITION_GRID_SIZE,
                    ],
                },
            },
        },
        "coordinate_contract": {
            "source_frame": "mujoco_rh_z_up_m",
            "target_frame": "unreal_lh_z_up_cm",
            "position_mapping": "(x,y,z)m -> (100*x,-100*y,100*z)cm",
        },
        "authority": {
            "physics": PHYSICS_AUTHORITY,
            "raycast": PHYSICS_AUTHORITY,
            "render_actors": "VisualOnly",
            "collision_profile": "NoCollision",
            "render_meshes_are_colliders": False,
        },
        "source_digests": source_digests,
        "terrain_sources": projection["terrain"],
        "route_stable_ids": [route["stable_id"] for route in routes["routes"]],  # type: ignore[index]
        "authoring_density_cells": authoring_cells,
    }
    return {**body, "content_digest": _content_digest(body)}


def _validate_materialized_terrain(
    terrain: Mapping[str, object], materialized_root: Path
) -> tuple[bytes, int, int, float, float]:
    root = materialized_root.resolve()
    source_grid = terrain.get("source_grid")
    if not isinstance(source_grid, Mapping) or source_grid.get("row_order") != "north_to_south":
        raise ValueError("Forest_HF terrain source row order must be north_to_south")
    materialized_recipe = _load_json_object(root / "terrain.recipe.json")
    if materialized_recipe != terrain:
        raise ValueError("materialized Forest_HF terrain recipe differs from the requested recipe")
    manifest = _load_json_object(root / "generated" / "asset-manifest.json")
    if manifest.get("schema") != "lingtu.sim.forest-asset-manifest.v1":
        raise ValueError("materialized Forest_HF asset manifest schema/version is unsupported")
    if manifest.get("world_package") != WORLD_PACKAGE or manifest.get("seed") != FIXED_SEED:
        raise ValueError("materialized Forest_HF assets target the wrong package or seed")
    canonical = manifest.get("canonical_source")
    if not isinstance(canonical, Mapping):
        raise ValueError("materialized Forest_HF canonical source record is missing")
    if canonical.get("path") != terrain.get("canonical_source"):
        raise ValueError("materialized Forest_HF canonical source path differs from terrain recipe")
    if canonical.get("sample_type") != "uint16" or canonical.get("endianness") != "little":
        raise ValueError("materialized Forest_HF canonical heightfield must be little-endian uint16")
    dimensions = canonical.get("dimensions_px")
    if isinstance(dimensions, (str, bytes)) or not isinstance(dimensions, Sequence) or len(dimensions) != 2:
        raise ValueError("materialized Forest_HF canonical dimensions are invalid")
    width, height = dimensions
    if isinstance(width, bool) or not isinstance(width, int) or width < 2:
        raise ValueError("materialized Forest_HF canonical width is invalid")
    if isinstance(height, bool) or not isinstance(height, int) or height < 2:
        raise ValueError("materialized Forest_HF canonical height is invalid")
    if canonical.get("extent_m") != terrain.get("extent_m"):
        raise ValueError("materialized Forest_HF extent differs from terrain recipe")

    records = manifest.get("artifacts")
    if isinstance(records, (str, bytes)) or not isinstance(records, Sequence) or not records:
        raise ValueError("materialized Forest_HF artifact records are missing")
    identities: list[dict[str, str]] = []
    raw_payload: bytes | None = None
    raw_relative = str(terrain["canonical_source"])
    for index, record in enumerate(records):
        if not isinstance(record, Mapping):
            raise ValueError(f"materialized Forest_HF artifact record {index} is invalid")
        relative = record.get("path")
        expected_bytes = record.get("bytes")
        expected_sha256 = record.get("sha256")
        if not isinstance(relative, str) or Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError(f"materialized Forest_HF artifact path {index} is invalid")
        if isinstance(expected_bytes, bool) or not isinstance(expected_bytes, int) or expected_bytes < 0:
            raise ValueError(f"materialized Forest_HF artifact byte count {index} is invalid")
        if not isinstance(expected_sha256, str) or _SHA256.fullmatch(expected_sha256) is None:
            raise ValueError(f"materialized Forest_HF artifact SHA256 {index} is invalid")
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError as error:
            raise ValueError(f"materialized Forest_HF artifact escapes package root: {relative}") from error
        try:
            actual_bytes = path.stat().st_size
        except OSError as error:
            raise ValueError(f"materialized Forest_HF artifact is missing: {relative}") from error
        if actual_bytes != expected_bytes:
            raise ValueError(
                f"materialized Forest_HF {Path(relative).name} byte count mismatch: "
                f"expected={expected_bytes}, actual={actual_bytes}"
            )
        actual_sha256 = _file_sha256(path)
        if actual_sha256 != expected_sha256:
            raise ValueError(f"materialized Forest_HF {Path(relative).name} SHA256 mismatch")
        identities.append({"path": relative, "sha256": actual_sha256})
        if relative == raw_relative:
            raw_payload = path.read_bytes()
    identities.sort(key=lambda item: item["path"])
    if manifest.get("asset_set_sha256") != identity_digest(identities):
        raise ValueError("materialized Forest_HF asset-set digest is invalid")
    if raw_payload is None:
        raise ValueError("materialized Forest_HF manifest does not contain heightfield_u16.raw")
    if len(raw_payload) != width * height * 2:
        raise ValueError("materialized Forest_HF heightfield_u16.raw dimensions disagree with its bytes")
    elevation_min, elevation_max = _finite_vector(
        terrain.get("elevation_range_m"), 2, "terrain.elevation_range_m"
    )
    if elevation_max <= elevation_min:
        raise ValueError("Forest_HF elevation range must be ordered")
    return raw_payload, width, height, elevation_min, elevation_max


def _height_sample_m(
    raw: bytes,
    width: int,
    height: int,
    elevation_min_m: float,
    elevation_max_m: float,
    x_m: float,
    y_m: float,
) -> float:
    column = min(width - 1.0, max(0.0, (x_m - WORLD_MIN_M) / WORLD_SIZE_M * (width - 1)))
    # The canonical raw is north-to-south: positive source Y is the first row.
    row = min(height - 1.0, max(0.0, (WORLD_MAX_M - y_m) / WORLD_SIZE_M * (height - 1)))
    c0, r0 = int(math.floor(column)), int(math.floor(row))
    c1, r1 = min(c0 + 1, width - 1), min(r0 + 1, height - 1)
    tx, ty = column - c0, row - r0

    def elevation(sample_row: int, sample_column: int) -> float:
        sample = struct.unpack_from("<H", raw, 2 * (sample_row * width + sample_column))[0]
        return elevation_min_m + sample / 65535.0 * (elevation_max_m - elevation_min_m)

    north = elevation(r0, c0) * (1.0 - tx) + elevation(r0, c1) * tx
    south = elevation(r1, c0) * (1.0 - tx) + elevation(r1, c1) * tx
    return north * (1.0 - ty) + south * ty


def _terrain_steepness(
    raw: bytes,
    width: int,
    height: int,
    elevation_min_m: float,
    elevation_max_m: float,
    x_m: float,
    y_m: float,
) -> float:
    dx = WORLD_SIZE_M / (width - 1)
    dy = WORLD_SIZE_M / (height - 1)
    left = _height_sample_m(raw, width, height, elevation_min_m, elevation_max_m, x_m - dx, y_m)
    right = _height_sample_m(raw, width, height, elevation_min_m, elevation_max_m, x_m + dx, y_m)
    south = _height_sample_m(raw, width, height, elevation_min_m, elevation_max_m, x_m, y_m - dy)
    north = _height_sample_m(raw, width, height, elevation_min_m, elevation_max_m, x_m, y_m + dy)
    dz_dx = (right - left) / (2.0 * dx)
    dz_dy = (north - south) / (2.0 * dy)
    steepness = math.atan(math.hypot(dz_dx, dz_dy)) / (math.pi / 2.0)
    return 0.0 if steepness < 1.0e-12 else min(1.0, steepness)


def build_create_points_json_params(
    contract: Mapping[str, object],
    terrain: Mapping[str, object],
    materialized_root: Path,
) -> dict[str, object]:
    """Project deterministic instances into exact per-slot UE PCG point branches."""

    if contract.get("world_package") != WORLD_PACKAGE or contract.get("seed") != FIXED_SEED:
        raise ValueError("Forest_HF PCG points require the exact v2 fixed-seed bake contract")
    contract_content_digest = contract.get("content_digest")
    contract_body = dict(contract)
    contract_body.pop("content_digest", None)
    if (
        not isinstance(contract_content_digest, str)
        or _SHA256.fullmatch(contract_content_digest) is None
        or contract_content_digest != _content_digest(contract_body)
    ):
        raise ValueError("Forest_HF PCG points require a valid bake contract content_digest")
    raw, width, height, elevation_min, elevation_max = _validate_materialized_terrain(
        terrain, materialized_root
    )
    grouped_points: dict[str, list[dict[str, object]]] = {}
    grouped_bindings: dict[str, list[dict[str, object]]] = {}
    authoring_cells = contract.get("authoring_density_cells")
    if not isinstance(authoring_cells, Sequence):
        raise ValueError("Forest_HF bake contract authoring density cells are missing")
    for cell in authoring_cells:
        for group in cell["hism_groups"]:  # type: ignore[index]
            mesh_slot = str(group["mesh_slot"])
            if mesh_slot not in _TREE_ASSET_SLOTS:
                raise ValueError(f"Forest_HF PCG points contain an unsupported tree slot: {mesh_slot}")
            points = grouped_points.setdefault(mesh_slot, [])
            bindings = grouped_bindings.setdefault(mesh_slot, [])
            for instance in group["instances"]:
                transform = instance["unreal_transform"]
                location_cm = _finite_vector(transform["location_cm"], 3, "instance.location_cm")
                source_x_m = location_cm[0] / 100.0
                source_y_m = -location_cm[1] / 100.0
                height_m = _height_sample_m(
                    raw, width, height, elevation_min, elevation_max, source_x_m, source_y_m
                )
                stable_id = str(instance["source_stable_id"])
                rotation = _finite_vector(transform["rotation_deg"], 3, "instance.rotation_deg")
                source_scale = _finite_vector(transform["scale_xyz"], 3, "instance.scale_xyz")
                metadata_entry = len(points)
                points.append(
                    {
                        "transform": {
                            "location": {
                                "x": location_cm[0],
                                "y": location_cm[1],
                                "z": 100.0 * height_m,
                            },
                            "rotation": {
                                "x": rotation[0],
                                "y": rotation[1],
                                "z": rotation[2],
                            },
                            "scale": {
                                "x": TREE_SCALE_FACTOR * source_scale[0],
                                "y": TREE_SCALE_FACTOR * source_scale[1],
                                "z": TREE_SCALE_FACTOR * source_scale[2],
                            },
                        },
                        "density": 1.0,
                        "boundsMin": {"x": -50.0, "y": -50.0, "z": 0.0},
                        "boundsMax": {"x": 50.0, "y": 50.0, "z": 200.0},
                        "color": {"x": 1.0, "y": 1.0, "z": 1.0, "w": 1.0},
                        "steepness": _terrain_steepness(
                            raw,
                            width,
                            height,
                            elevation_min,
                            elevation_max,
                            source_x_m,
                            source_y_m,
                        ),
                        "seed": int.from_bytes(hashlib.sha256(stable_id.encode()).digest()[:4], "big")
                        & 0x7FFFFFFF,
                        "metadataEntry": metadata_entry,
                    }
                )
                bindings.append(
                    {
                        "metadataEntry": metadata_entry,
                        "mesh_slot": mesh_slot,
                        "source_stable_id": stable_id,
                    }
                )

    if set(grouped_points) != set(_TREE_ASSET_SLOTS):
        raise ValueError("Forest_HF PCG points must contain exactly the canonical tree slots")
    groups = [
        {
            "mesh_slot": mesh_slot,
            "point_count": len(grouped_points[mesh_slot]),
            "source_bindings": grouped_bindings[mesh_slot],
            "json_params": {
                "pointsToCreate": grouped_points[mesh_slot],
                "coordinateSpace": "World",
                "bCullPointsOutsideVolume": True,
            },
        }
        for mesh_slot in sorted(grouped_points)
    ]
    body: dict[str, object] = {
        "schema": CREATE_POINTS_SCHEMA,
        "world_package": WORLD_PACKAGE,
        "seed": FIXED_SEED,
        "contract_content_digest": contract_content_digest,
        "selection_policy": EXACT_MESH_SLOT_SELECTION_POLICY,
        "point_count": sum(len(points) for points in grouped_points.values()),
        "slot_counts": {
            mesh_slot: len(grouped_points[mesh_slot]) for mesh_slot in sorted(grouped_points)
        },
        "groups": groups,
    }
    return {**body, "content_digest": _content_digest(body)}


def require_supported_ue_authoring_adapter() -> None:
    """Keep this portable contract emitter separate from editor mutation."""

    raise RuntimeError(
        "Direct Forest_HF PCG Python authoring is disabled. Use the reviewed "
        "build_forest_hf_mcp.py adapter with an exact v2 Create Points wrapper."
    )


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract-output", type=Path, default=DEFAULT_CONTRACT_PATH)
    parser.add_argument("--materialized-root", type=Path)
    parser.add_argument("--pcg-points-output", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Validate canonical inputs, emit the contract, and guard editor mutation."""

    args = _parse_args(argv)
    if (args.materialized_root is None) != (args.pcg_points_output is None):
        raise ValueError("--materialized-root and --pcg-points-output must be provided together")
    terrain = _load_json_object(TERRAIN_RECIPE_PATH)
    contract = build_offline_bake_contract(
        _load_json_object(UE_PROJECTION_PATH),
        terrain,
        _load_json_object(ROUTES_PATH),
    )
    output = args.contract_output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(canonical_json_bytes(contract))
    print(f"LINGTU_FOREST_HF_CONTRACT={output}")
    print(f"LINGTU_FOREST_HF_CONTENT_DIGEST={contract['content_digest']}")
    if args.materialized_root is not None:
        json_params = build_create_points_json_params(contract, terrain, args.materialized_root)
        points_output = args.pcg_points_output.resolve()
        points_output.parent.mkdir(parents=True, exist_ok=True)
        points_output.write_bytes(canonical_json_bytes(json_params))
        print(f"LINGTU_FOREST_HF_PCG_POINTS={points_output}")
        print(f"LINGTU_FOREST_HF_PCG_POINT_COUNT={json_params['point_count']}")
    if unreal is not None and not args.validate_only:
        require_supported_ue_authoring_adapter()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
