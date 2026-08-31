"""Build bounded MuJoCo collision proxies from Forest_HF visual placements."""

from __future__ import annotations

import copy
import hashlib
import itertools
import json
import math
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence
from xml.etree import ElementTree

WORLD_PACKAGE = "forest_hf@2.0.0"
DEFAULT_SEED = 20260813
CELL_SIZE_M = 100.0
MAX_TREE_PROXIES_PER_CELL = 2
MAX_ROCK_PROXIES_PER_CELL = 1
MAX_PROXY_COUNT = 1200

_STABLE_ID = re.compile(r"^[A-Za-z0-9_.:-]+$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_TREE_SLOTS = frozenset({"forest.asset.birch", "forest.asset.pine"})
_ROCK_SLOTS = frozenset({"forest.asset.boulder"})
_LAYOUT_SCHEMA = "lingtu.sim.forest-layout.v1"
_UNREAL_BAKE_SCHEMA = "lingtu.sim.unreal-forest-offline-bake.v2"


@dataclass(frozen=True)
class CollisionProxyArtifacts:
    """Byte-stable MJCF and manifest generated from one placement source."""

    mjcf: bytes
    manifest: dict[str, object]


def _canonical_bytes(value: object) -> bytes:
    return (
        json.dumps(
            value,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _identity_bytes(value: object) -> bytes:
    return (
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")


def _digest(value: object) -> str:
    return hashlib.sha256(_identity_bytes(value)).hexdigest()


def _document_digest(value: object) -> str:
    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _content_digest(value: object) -> str:
    payload = (
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def _vector(value: object, size: int, field: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{field} must contain exactly {size} finite numbers")
    if len(value) != size:
        raise ValueError(f"{field} must contain exactly {size} finite numbers")
    return tuple(_finite_number(item, field) for item in value)


def _stable_unit(seed: int, identity: str) -> float:
    raw = hashlib.sha256(f"{seed}:{identity}".encode()).digest()[:8]
    return int.from_bytes(raw, "big") / float(2**64 - 1)


def _distance_to_segment(
    point: tuple[float, float], start: object, end: object
) -> float:
    ax, ay = _vector(start, 2, "route segment start")
    bx, by = _vector(end, 2, "route segment end")
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, (ax, ay))
    ratio = ((point[0] - ax) * dx + (point[1] - ay) * dy) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    return math.dist(point, (ax + ratio * dx, ay + ratio * dy))


def _validate_routes(routes: Mapping[str, object]) -> str:
    if routes.get("schema") != "lingtu.sim.forest-route-contract.v1":
        raise ValueError("Forest_HF collision proxies require route contract v1")
    if routes.get("world_package") != WORLD_PACKAGE:
        raise ValueError(f"route contract must target {WORLD_PACKAGE}")
    route_items = routes.get("routes")
    spawn = routes.get("spawn")
    goal = routes.get("goal")
    if not isinstance(route_items, Sequence) or not route_items:
        raise ValueError("route contract must contain routes")
    if not isinstance(spawn, Mapping) or not isinstance(goal, Mapping):
        raise ValueError("route contract must contain spawn and goal")
    identity = {"routes": route_items, "spawn": spawn, "goal": goal}
    digest = _digest(identity)
    if routes.get("route_contract_sha256") != digest:
        raise ValueError("route contract digest does not match route content")
    return digest


def _iter_placements(
    source: Mapping[str, object] | Sequence[Mapping[str, object]],
) -> Iterable[Mapping[str, object]]:
    if isinstance(source, Mapping):
        instances = source.get("instances")
        dressing = source.get("dressing")
        if isinstance(instances, Sequence) and isinstance(dressing, Sequence):
            yield from (item for item in instances if isinstance(item, Mapping))
            yield from (item for item in dressing if isinstance(item, Mapping))
            return
        cells = source.get("authoring_density_cells", source.get("cells"))
        if isinstance(cells, Sequence):
            for cell in cells:
                if not isinstance(cell, Mapping):
                    continue
                groups = cell.get("hism_groups", ())
                if not isinstance(groups, Sequence):
                    continue
                for group in groups:
                    if not isinstance(group, Mapping):
                        continue
                    group_instances = group.get("instances", ())
                    if isinstance(group_instances, Sequence):
                        yield from (
                            item for item in group_instances if isinstance(item, Mapping)
                        )
            return
        raise ValueError("placement source has no supported instances collection")
    yield from source


def _validate_layout_contract(source: Mapping[str, object]) -> dict[str, object]:
    if source.get("world_package") != WORLD_PACKAGE:
        raise ValueError("collision proxy layout world package mismatch")
    if source.get("seed") != DEFAULT_SEED:
        raise ValueError("collision proxy layout seed mismatch")
    layout_digest = source.get("digest")
    layout_body = {key: value for key, value in source.items() if key != "digest"}
    if not isinstance(layout_digest, str) or layout_digest != _document_digest(
        layout_body
    ):
        raise ValueError("collision proxy layout digest mismatch")
    return {
        "source_type": "blender_layout",
        "placement_schema": _LAYOUT_SCHEMA,
        "placement_contract_digest": layout_digest,
        "layout_schema": _LAYOUT_SCHEMA,
        "layout_digest": layout_digest,
    }


def _validate_unreal_bake_contract(source: Mapping[str, object]) -> dict[str, object]:
    if source.get("world_package") != WORLD_PACKAGE:
        raise ValueError("collision proxy bake contract world package mismatch")
    if source.get("seed") != DEFAULT_SEED:
        raise ValueError("collision proxy bake contract seed mismatch")
    contract_digest = source.get("content_digest")
    body = {key: value for key, value in source.items() if key != "content_digest"}
    if (
        not isinstance(contract_digest, str)
        or not _SHA256.fullmatch(contract_digest)
        or contract_digest != _content_digest(body)
    ):
        raise ValueError("collision proxy bake contract content digest mismatch")

    authority = source.get("authority")
    required_authority = {
        "physics": "mujoco",
        "raycast": "mujoco",
        "render_actors": "VisualOnly",
        "collision_profile": "NoCollision",
        "render_meshes_are_colliders": False,
    }
    if not isinstance(authority, Mapping) or any(
        authority.get(key) != value for key, value in required_authority.items()
    ):
        raise ValueError("collision proxy bake contract violates authority isolation")

    source_digests = source.get("source_digests")
    required_digests = {
        "ue_projection",
        "terrain_recipe",
        "routes",
        "production_contract",
        "route_contract",
    }
    if not isinstance(source_digests, Mapping) or any(
        not isinstance(source_digests.get(key), str)
        or not _SHA256.fullmatch(str(source_digests[key]))
        for key in required_digests
    ):
        raise ValueError("collision proxy bake contract source digests are invalid")

    cells = source.get("authoring_density_cells")
    if (
        not isinstance(cells, Sequence)
        or isinstance(cells, (str, bytes))
        or not cells
    ):
        raise ValueError("collision proxy bake contract has no authoring density cells")
    seen_cells: set[str] = set()
    seen_groups: set[str] = set()
    seen_instances: set[str] = set()
    candidate_classes: set[str] = set()
    candidate_count = 0
    required_group_authority = {
        "classification": "VisualOnly",
        "collision_profile": "NoCollision",
        "collision_enabled": False,
        "simulate_physics": False,
        "can_ever_affect_navigation": False,
        "generate_overlap_events": False,
    }
    for cell_index, cell in enumerate(cells):
        if not isinstance(cell, Mapping):
            raise TypeError(
                f"bake authoring_density_cells[{cell_index}] must be an object"
            )
        cell_id = cell.get("stable_id")
        if not isinstance(cell_id, str) or not _STABLE_ID.fullmatch(cell_id):
            raise ValueError("bake authoring density cell requires a safe stable ID")
        if cell_id in seen_cells:
            raise ValueError(f"duplicate bake authoring density cell ID: {cell_id}")
        seen_cells.add(cell_id)
        groups = cell.get("hism_groups")
        if not isinstance(groups, Sequence) or isinstance(groups, (str, bytes)):
            raise ValueError(f"bake authoring density cell {cell_id} has invalid HISM groups")
        for group_index, group in enumerate(groups):
            if not isinstance(group, Mapping):
                raise TypeError(
                    f"bake cell {cell_id} HISM group {group_index} must be an object"
                )
            group_id = group.get("stable_id")
            if not isinstance(group_id, str) or not _STABLE_ID.fullmatch(group_id):
                raise ValueError("bake HISM group requires a safe stable ID")
            if group_id in seen_groups:
                raise ValueError(f"duplicate bake HISM group ID: {group_id}")
            seen_groups.add(group_id)
            if any(
                group.get(key) != value
                for key, value in required_group_authority.items()
            ):
                raise ValueError(f"bake HISM group {group_id} violates NoCollision")
            slot = group.get("mesh_slot")
            if slot not in _TREE_SLOTS | _ROCK_SLOTS:
                raise ValueError(f"bake HISM group {group_id} has unsupported mesh slot")
            group_kind = "tree" if slot in _TREE_SLOTS else "rock"
            instances = group.get("instances")
            if not isinstance(instances, Sequence) or isinstance(
                instances, (str, bytes)
            ):
                raise ValueError(f"bake HISM group {group_id} has invalid instances")
            for instance_index, instance in enumerate(instances):
                if not isinstance(instance, Mapping):
                    raise TypeError(
                        f"bake HISM group {group_id} instance {instance_index} must be an object"
                    )
                source_id = _source_id(instance)
                if source_id in seen_instances:
                    raise ValueError(f"duplicate collision candidate stable ID: {source_id}")
                seen_instances.add(source_id)
                if instance.get("mesh_slot") != slot:
                    raise ValueError(
                        f"bake instance {source_id} mesh slot differs from its HISM group"
                    )
                transform = instance.get("unreal_transform")
                if not isinstance(transform, Mapping):
                    raise ValueError(f"bake instance {source_id} has no Unreal transform")
                _vector(transform.get("location_cm"), 3, "unreal_transform.location_cm")
                _vector(transform.get("rotation_deg"), 3, "unreal_transform.rotation_deg")
                _scale(instance)
                candidate_classes.add(group_kind)
                candidate_count += 1
    if candidate_count == 0:
        raise ValueError("collision proxy bake contract has no collision candidates")
    return {
        "source_type": "unreal_offline_bake",
        "placement_schema": _UNREAL_BAKE_SCHEMA,
        "placement_contract_digest": contract_digest,
        "bake_contract_digest": contract_digest,
        "authoritative_collision_candidate_classes": sorted(candidate_classes),
        "source_candidate_count": candidate_count,
    }


def validate_placement_contract(source: Mapping[str, object]) -> dict[str, object]:
    """Validate a versioned visual-placement contract and return its identity."""

    schema = source.get("schema")
    if schema == _LAYOUT_SCHEMA:
        return _validate_layout_contract(source)
    if schema == _UNREAL_BAKE_SCHEMA:
        return _validate_unreal_bake_contract(source)
    raise ValueError("collision proxy placement contract schema is unsupported")


def _kind(item: Mapping[str, object]) -> str | None:
    explicit = item.get("kind")
    if explicit == "tree":
        return "tree"
    if explicit == "rock":
        return "rock"
    slot = item.get("asset_slot_id", item.get("mesh_slot"))
    if slot in _TREE_SLOTS:
        return "tree"
    if slot in _ROCK_SLOTS:
        return "rock"
    return None


def _source_id(item: Mapping[str, object]) -> str:
    value = item.get("source_stable_id", item.get("stable_id"))
    if not isinstance(value, str) or not _STABLE_ID.fullmatch(value):
        raise ValueError("collision candidate requires a safe stable source ID")
    return value


def _position(
    item: Mapping[str, object],
    *,
    height_at: Callable[[float, float], float] | None,
) -> tuple[float, float, float]:
    position = item.get("position_m")
    if position is not None:
        return _vector(position, 3, "position_m")
    transform = item.get("unreal_transform")
    if not isinstance(transform, Mapping):
        raise ValueError("collision candidate must define position_m or unreal_transform")
    location_cm = _vector(transform.get("location_cm"), 3, "unreal_transform.location_cm")
    x, y = location_cm[0] / 100.0, -location_cm[1] / 100.0
    if height_at is None:
        raise ValueError(
            "Unreal placeholder placements require a canonical terrain height sampler"
        )
    z = _finite_number(height_at(x, y), "height_at result")
    return x, y, z


def _scale(item: Mapping[str, object]) -> float:
    value = item.get("scale")
    if value is not None:
        result = _finite_number(value, "scale")
    else:
        transform = item.get("unreal_transform")
        if isinstance(transform, Mapping) and transform.get("scale_xyz") is not None:
            values = _vector(transform["scale_xyz"], 3, "unreal_transform.scale_xyz")
            result = sum(values) / 3.0
        else:
            values = _vector(item.get("scale_xyz", [1.0, 1.0, 1.0]), 3, "scale_xyz")
            result = sum(values) / 3.0
    if not 0.25 <= result <= 4.0:
        raise ValueError("collision candidate scale must be within [0.25, 4.0]")
    return result


def _proxy_geometry(
    kind: str, source_id: str, scale: float
) -> tuple[str, tuple[float, ...], float]:
    if kind == "tree":
        radius = scale * (0.16 + 0.12 * _stable_unit(DEFAULT_SEED, source_id + ":radius"))
        half_height = scale * (
            1.6 + 0.6 * _stable_unit(DEFAULT_SEED, source_id + ":half-height")
        )
        return "cylinder", (radius, half_height), radius
    x_radius = scale * (0.45 + 0.45 * _stable_unit(DEFAULT_SEED, source_id + ":x"))
    y_radius = scale * (0.40 + 0.40 * _stable_unit(DEFAULT_SEED, source_id + ":y"))
    z_radius = scale * (0.30 + 0.45 * _stable_unit(DEFAULT_SEED, source_id + ":z"))
    return "ellipsoid", (x_radius, y_radius, z_radius), max(x_radius, y_radius)


def _is_clear(
    point: tuple[float, float], radius: float, routes: Mapping[str, object]
) -> bool:
    for route in routes["routes"]:  # type: ignore[index]
        if not isinstance(route, Mapping):
            raise ValueError("route entries must be objects")
        points = route.get("centerline_xy_m")
        limits = route.get("navigation_limits")
        if not isinstance(points, Sequence) or len(points) < 2:
            raise ValueError("route centerline must contain at least two points")
        if not isinstance(limits, Mapping):
            raise ValueError("route navigation limits are missing")
        clearance = _finite_number(
            limits.get("clearance_each_side_m"),
            "navigation_limits.clearance_each_side_m",
        )
        corridor_radius = _finite_number(route.get("road_width_m"), "road_width_m") / 2.0
        corridor_radius += clearance + radius
        if any(
            _distance_to_segment(point, start, end) < corridor_radius
            for start, end in itertools.pairwise(points)
        ):
            return False
    markers = (
        (routes["spawn"], "minimum_clear_radius_m"),
        (routes["goal"], "acceptance_radius_m"),
    )
    for marker, radius_field in markers:
        if not isinstance(marker, Mapping):
            raise ValueError("spawn and goal must be objects")
        center = _vector(marker.get("position_xy_m"), 2, "marker.position_xy_m")
        marker_radius = _finite_number(marker.get(radius_field), radius_field)
        if math.dist(point, center) < marker_radius + 2.0 + radius:
            return False
    return True


def _format_numbers(values: Sequence[float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def _mjcf(proxies: Sequence[Mapping[str, object]]) -> bytes:
    root = ElementTree.Element("mujoco", {"model": "forest_hf_collision_proxies"})
    ElementTree.SubElement(root, "compiler", {"angle": "radian"})
    worldbody = ElementTree.SubElement(root, "worldbody")
    body = ElementTree.SubElement(worldbody, "body", {"name": "forest_collision_proxies"})
    for proxy in proxies:
        ElementTree.SubElement(
            body,
            "geom",
            {
                "name": str(proxy["stable_id"]),
                "type": str(proxy["shape"]),
                "pos": _format_numbers(proxy["position_m"]),  # type: ignore[arg-type]
                "size": _format_numbers(proxy["size_m"]),  # type: ignore[arg-type]
                "contype": "1",
                "conaffinity": "1",
                "group": "3",
                "rgba": "0.25 0.18 0.10 0.0",
            },
        )
    ElementTree.indent(root, space="  ")
    return ElementTree.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"


def collision_clearance_evidence(
    proxies: Sequence[Mapping[str, object]], routes: Mapping[str, object]
) -> dict[str, object]:
    """Measure the clearance margin already enforced by proxy generation."""

    route_digest = _validate_routes(routes)
    if not proxies:
        raise ValueError("collision clearance evidence requires at least one proxy")
    route_margins: list[float] = []
    marker_margins: dict[str, list[float]] = {"spawn": [], "goal": []}
    for proxy in proxies:
        x, y, _z = _vector(proxy.get("position_m"), 3, "proxy.position_m")
        radius = _finite_number(
            proxy.get("footprint_radius_m"), "proxy.footprint_radius_m"
        )
        for route in routes["routes"]:  # type: ignore[index]
            if not isinstance(route, Mapping):
                raise ValueError("route entries must be objects")
            points = route.get("centerline_xy_m")
            limits = route.get("navigation_limits")
            if not isinstance(points, Sequence) or len(points) < 2:
                raise ValueError("route centerline must contain at least two points")
            if not isinstance(limits, Mapping):
                raise ValueError("route navigation limits are missing")
            required = (
                _finite_number(route.get("road_width_m"), "road_width_m") / 2.0
                + _finite_number(
                    limits.get("clearance_each_side_m"),
                    "navigation_limits.clearance_each_side_m",
                )
                + radius
            )
            actual = min(
                _distance_to_segment((x, y), start, end)
                for start, end in itertools.pairwise(points)
            )
            route_margins.append(actual - required)
        for marker_name, radius_field in (
            ("spawn", "minimum_clear_radius_m"),
            ("goal", "acceptance_radius_m"),
        ):
            marker = routes[marker_name]
            if not isinstance(marker, Mapping):
                raise ValueError("spawn and goal must be objects")
            center = _vector(marker.get("position_xy_m"), 2, "marker.position_xy_m")
            required = (
                _finite_number(marker.get(radius_field), radius_field) + 2.0 + radius
            )
            marker_margins[marker_name].append(math.dist((x, y), center) - required)

    minimum_route_margin = min(route_margins)
    minimum_spawn_margin = min(marker_margins["spawn"])
    minimum_goal_margin = min(marker_margins["goal"])
    qualified = min(
        minimum_route_margin,
        minimum_spawn_margin,
        minimum_goal_margin,
    ) >= -1e-9
    return {
        "qualified": qualified,
        "proxy_count": len(proxies),
        "tested_route_count": len(routes["routes"]),  # type: ignore[arg-type,index]
        "route_contract_sha256": route_digest,
        "route_rule": "road_width/2 + declared side clearance + proxy footprint radius",
        "marker_rule": "declared radius + 2m safety margin + proxy footprint radius",
        "minimum_route_margin_m": round(minimum_route_margin, 6),
        "minimum_spawn_margin_m": round(minimum_spawn_margin, 6),
        "minimum_goal_margin_m": round(minimum_goal_margin, 6),
    }


def merge_collision_proxies_into_world(
    base_world_mjcf: bytes, artifacts: CollisionProxyArtifacts
) -> bytes:
    """Return one complete Forest_HF world with the generated proxy body attached."""

    base_root = ElementTree.fromstring(base_world_mjcf)  # noqa: S314 - local generated XML
    if base_root.tag != "mujoco":
        raise ValueError("base Forest_HF physics document must be an MJCF mujoco root")
    worldbodies = base_root.findall("./worldbody")
    if len(worldbodies) != 1:
        raise ValueError("base Forest_HF physics document must contain one worldbody")
    if base_root.find("./worldbody/body[@name='forest_collision_proxies']") is not None:
        raise ValueError("base Forest_HF physics document already contains collision proxies")

    proxy_root = ElementTree.fromstring(artifacts.mjcf)  # noqa: S314 - local generated XML
    proxy_body = proxy_root.find("./worldbody/body[@name='forest_collision_proxies']")
    if proxy_body is None:
        raise ValueError("collision proxy MJCF is missing its proxy body")
    worldbodies[0].append(copy.deepcopy(proxy_body))
    base_root.set("model", "forest_hf_2_0_0_with_collision_proxies")
    ElementTree.indent(base_root, space="  ")
    return (
        ElementTree.tostring(base_root, encoding="utf-8", xml_declaration=True)
        + b"\n"
    )


def build_collision_proxy_artifacts(
    placement_source: Mapping[str, object] | Sequence[Mapping[str, object]],
    routes: Mapping[str, object],
    *,
    height_at: Callable[[float, float], float] | None = None,
) -> CollisionProxyArtifacts:
    """Create a stable, route-clear, performance-bounded MuJoCo proxy set."""

    _validate_routes(routes)
    placement_contract: dict[str, object] = {}
    if isinstance(placement_source, Mapping) and placement_source.get("schema") is not None:
        placement_contract = validate_placement_contract(placement_source)
    candidates: list[dict[str, object]] = []
    source_candidates: list[dict[str, object]] = []
    seen_ids: set[str] = set()
    rejected_for_clearance = 0
    for item in _iter_placements(placement_source):
        kind = _kind(item)
        if kind is None:
            continue
        source_id = _source_id(item)
        if source_id in seen_ids:
            raise ValueError(f"duplicate collision candidate stable ID: {source_id}")
        seen_ids.add(source_id)
        x, y, ground_z = _position(item, height_at=height_at)
        scale = _scale(item)
        shape, size, footprint_radius = _proxy_geometry(kind, source_id, scale)
        source_candidates.append(
            {
                "source_stable_id": source_id,
                "kind": kind,
                "ground_position_m": [round(x, 6), round(y, 6), round(ground_z, 6)],
                "shape": shape,
                "size_m": [round(value, 6) for value in size],
            }
        )
        if not _is_clear((x, y), footprint_radius, routes):
            rejected_for_clearance += 1
            continue
        vertical_radius = size[1] if kind == "tree" else size[2]
        stable_id = f"forest.physics.proxy.{hashlib.sha256(source_id.encode()).hexdigest()[:20]}"
        candidates.append(
            {
                "stable_id": stable_id,
                "source_stable_id": source_id,
                "kind": kind,
                "shape": shape,
                "position_m": [round(x, 6), round(y, 6), round(ground_z + vertical_radius, 6)],
                "size_m": [round(value, 6) for value in size],
                "footprint_radius_m": round(footprint_radius, 6),
                "cell_xy": [math.floor(x / CELL_SIZE_M), math.floor(y / CELL_SIZE_M)],
                "selection_rank": hashlib.sha256(
                    f"{DEFAULT_SEED}:{source_id}:collision-selection".encode()
                ).hexdigest(),
            }
        )

    by_cell_kind: dict[tuple[int, int, str], list[dict[str, object]]] = {}
    for candidate in candidates:
        cell_x, cell_y = candidate["cell_xy"]  # type: ignore[misc]
        key = int(cell_x), int(cell_y), str(candidate["kind"])
        by_cell_kind.setdefault(key, []).append(candidate)
    selected: list[dict[str, object]] = []
    for key in sorted(by_cell_kind):
        group = sorted(by_cell_kind[key], key=lambda item: str(item["selection_rank"]))
        limit = MAX_TREE_PROXIES_PER_CELL if key[2] == "tree" else MAX_ROCK_PROXIES_PER_CELL
        selected.extend(group[:limit])
    selected = sorted(selected, key=lambda item: str(item["stable_id"]))
    if len(selected) > MAX_PROXY_COUNT:
        selected = sorted(selected, key=lambda item: str(item["selection_rank"]))[
            :MAX_PROXY_COUNT
        ]
        selected.sort(key=lambda item: str(item["stable_id"]))

    authoritative_kinds = {str(item["kind"]) for item in source_candidates}
    if not authoritative_kinds:
        raise ValueError("Forest_HF placement source has no collision candidates")
    selected_kinds = {str(item["kind"]) for item in selected}
    missing_kinds = sorted(authoritative_kinds - selected_kinds)
    if missing_kinds:
        raise ValueError(
            "Forest_HF collision qualification omitted authoritative collision "
            f"candidate classes: {missing_kinds}"
        )

    for proxy in selected:
        proxy.pop("selection_rank")
    mjcf = _mjcf(selected)
    source_identity = sorted(
        source_candidates, key=lambda value: str(value["source_stable_id"])
    )
    source_contract: dict[str, object] = {
        "stable_id_contract": "source_stable_id or stable_id from visual placement source",
        "placement_digest": _digest(source_identity),
        "authoritative_collision_candidate_classes": sorted(authoritative_kinds),
    }
    source_contract.update(placement_contract)
    clearance = collision_clearance_evidence(selected, routes)
    body: dict[str, object] = {
        "schema": "lingtu.sim.forest-collision-proxies.v1",
        "world_package": WORLD_PACKAGE,
        "seed": DEFAULT_SEED,
        "authority": {
            "physics": "mujoco",
            "raycast": "mujoco",
            "unreal": "visual_only_no_collision",
            "render_meshes_are_colliders": False,
        },
        "selection": {
            "algorithm": "sha256_rank_per_100m_cell_v1",
            "cell_size_m": CELL_SIZE_M,
            "maximum_tree_proxies_per_cell": MAX_TREE_PROXIES_PER_CELL,
            "maximum_rock_proxies_per_cell": MAX_ROCK_PROXIES_PER_CELL,
            "maximum_proxy_count": MAX_PROXY_COUNT,
            "input_candidate_count": len(source_candidates),
            "eligible_candidate_count": len(candidates),
            "rejected_for_route_spawn_goal_clearance": rejected_for_clearance,
            "selected_proxy_count": len(selected),
            "selected_tree_proxy_count": sum(
                item["kind"] == "tree" for item in selected
            ),
            "selected_rock_proxy_count": sum(
                item["kind"] == "rock" for item in selected
            ),
        },
        "clearance": clearance,
        "source": source_contract,
        "artifact": {
            "path": "generated/forest_collision_proxies.xml",
            "format": "mjcf",
            "bytes": len(mjcf),
            "sha256": hashlib.sha256(mjcf).hexdigest(),
        },
        "proxies": selected,
    }
    manifest = {**body, "content_digest": _digest(body)}
    return CollisionProxyArtifacts(mjcf=mjcf, manifest=manifest)


def write_collision_proxy_artifacts(
    output_root: Path | str, artifacts: CollisionProxyArtifacts
) -> tuple[Path, Path]:
    """Write the generated proxy fragment and its canonical manifest."""

    root = Path(output_root)
    root.mkdir(parents=True, exist_ok=True)
    mjcf_path = root / "forest_collision_proxies.xml"
    manifest_path = root / "forest_collision_proxies.manifest.json"
    _atomic_write(mjcf_path, artifacts.mjcf)
    _atomic_write(manifest_path, _canonical_bytes(artifacts.manifest))
    return mjcf_path, manifest_path
