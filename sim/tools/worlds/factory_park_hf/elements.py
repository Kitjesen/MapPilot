"""Compile safe element batches into FactoryPark_HF layout objects.

The public interface deliberately accepts plain mappings and returns immutable,
normalized source-frame geometry. Blender, Unreal, and MuJoCo continue to
consume the expanded world layout rather than learning this authoring format.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from typing import Mapping, Sequence

ELEMENT_BATCH_SCHEMA = "lingtu.sim.factory-park-element-batch.v1"
_SAFE_ID = re.compile(r"[a-z][a-z0-9_]{0,63}")
_BATCH_FIELDS = frozenset({"schema", "batch_id", "description", "elements"})
_ELEMENT_FIELDS = frozenset(
    {"instance_key", "element_type", "surface_id", "position_xy_m", "yaw_deg"}
)


@dataclass(frozen=True)
class _ElementTemplate:
    semantic_class: str
    shape: str
    material: str
    collision: bool
    visual_only: bool
    allowed_surface_classes: tuple[str, ...]
    size_m: tuple[float, float, float] | None = None
    radius_m: float | None = None
    half_height_m: float | None = None


@dataclass(frozen=True)
class CompiledElementBatch:
    """Canonical element geometry and its content identity."""

    batch_id: str
    digest: str
    objects: tuple[dict[str, object], ...]


_OPERATIONS_SURFACES = (
    "container_yard",
    "gravel_test_area",
    "loading_dock",
    "parking_area",
    "road",
)

_CATALOG = {
    "equipment_cabinet": _ElementTemplate(
        semantic_class="equipment_cabinet",
        shape="box",
        material="warehouse_cladding",
        collision=True,
        visual_only=False,
        allowed_surface_classes=_OPERATIONS_SURFACES,
        size_m=(1.2, 0.55, 1.8),
    ),
    "fire_cabinet": _ElementTemplate(
        semantic_class="fire_cabinet",
        shape="box",
        material="container_red",
        collision=True,
        visual_only=False,
        allowed_surface_classes=_OPERATIONS_SURFACES,
        size_m=(0.8, 0.35, 1.2),
    ),
    "industrial_drum": _ElementTemplate(
        semantic_class="industrial_drum",
        shape="cylinder",
        material="container_blue",
        collision=True,
        visual_only=False,
        allowed_surface_classes=_OPERATIONS_SURFACES,
        radius_m=0.29,
        half_height_m=0.45,
    ),
    "jersey_barrier": _ElementTemplate(
        semantic_class="jersey_barrier",
        shape="box",
        material="concrete",
        collision=True,
        visual_only=False,
        allowed_surface_classes=_OPERATIONS_SURFACES,
        size_m=(2.0, 0.55, 0.8),
    ),
    "lane_marker": _ElementTemplate(
        semantic_class="lane_marker",
        shape="box",
        material="road_marking",
        collision=False,
        visual_only=True,
        allowed_surface_classes=("container_yard", "parking_area", "road"),
        size_m=(2.5, 0.15, 0.012),
    ),
    "pallet_stack": _ElementTemplate(
        semantic_class="pallet_stack",
        shape="box",
        material="pallet_wood",
        collision=True,
        visual_only=False,
        allowed_surface_classes=("container_yard", "loading_dock"),
        size_m=(1.2, 1.0, 0.45),
    ),
    "safety_bollard": _ElementTemplate(
        semantic_class="safety_bollard",
        shape="cylinder",
        material="painted_steel",
        collision=True,
        visual_only=False,
        allowed_surface_classes=_OPERATIONS_SURFACES,
        radius_m=0.12,
        half_height_m=0.55,
    ),
    "safety_sign": _ElementTemplate(
        semantic_class="safety_sign",
        shape="box",
        material="painted_steel",
        collision=True,
        visual_only=False,
        allowed_surface_classes=_OPERATIONS_SURFACES,
        size_m=(0.8, 0.12, 1.8),
    ),
    "traffic_cone": _ElementTemplate(
        semantic_class="traffic_cone",
        shape="cylinder",
        material="container_orange",
        collision=True,
        visual_only=False,
        allowed_surface_classes=_OPERATIONS_SURFACES,
        radius_m=0.2,
        half_height_m=0.35,
    ),
    "wheel_stop": _ElementTemplate(
        semantic_class="wheel_stop",
        shape="box",
        material="curb_concrete",
        collision=True,
        visual_only=False,
        allowed_surface_classes=("container_yard", "parking_area", "road"),
        size_m=(1.8, 0.2, 0.15),
    ),
}


def element_catalog() -> dict[str, dict[str, object]]:
    """Return the supported element templates as serializable declarations."""

    catalog: dict[str, dict[str, object]] = {}
    for element_type, template in sorted(_CATALOG.items()):
        item: dict[str, object] = {
            "semantic_class": template.semantic_class,
            "shape": template.shape,
            "material": template.material,
            "authority": "PhysicsShared" if template.collision else "VisualOnly",
            "allowed_surface_classes": list(template.allowed_surface_classes),
        }
        if template.shape == "box":
            item["size_m"] = list(template.size_m or ())
        else:
            item["radius_m"] = template.radius_m
            item["half_height_m"] = template.half_height_m
        catalog[element_type] = item
    return catalog


def _canonical_json(value: object) -> bytes:
    return (
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


def _find_surface(layout: Mapping[str, object], surface_id: str) -> Mapping[str, object]:
    objects = layout.get("objects")
    if isinstance(objects, (str, bytes)) or not isinstance(objects, Sequence):
        raise TypeError("expanded layout objects must be an array")
    matches = [item for item in objects if isinstance(item, Mapping) and item.get("id") == surface_id]
    if len(matches) != 1:
        raise ValueError(f"element surface_id {surface_id!r} must resolve exactly once")
    return matches[0]


def _validate_support_footprint(
    *,
    template: _ElementTemplate,
    position_xy: Sequence[object],
    yaw_deg: float,
    surface: Mapping[str, object],
) -> None:
    if surface.get("shape") != "box":
        raise ValueError("element support surface must be a box")
    surface_position = surface.get("position_m")
    surface_size = surface.get("size_m")
    if not isinstance(surface_position, Sequence) or len(surface_position) != 3:
        raise ValueError("element support surface position_m must contain three values")
    if not isinstance(surface_size, Sequence) or len(surface_size) != 3:
        raise ValueError("element support surface size_m must contain three values")

    x_m, y_m = (float(value) for value in position_xy)
    surface_x, surface_y = float(surface_position[0]), float(surface_position[1])
    surface_yaw = float(surface.get("yaw_deg", 0.0))
    if not all(math.isfinite(value) for value in (x_m, y_m, yaw_deg, surface_x, surface_y, surface_yaw)):
        raise ValueError("element placement values must be finite")

    surface_yaw_rad = math.radians(surface_yaw)
    offset_x = x_m - surface_x
    offset_y = y_m - surface_y
    local_x = math.cos(surface_yaw_rad) * offset_x + math.sin(surface_yaw_rad) * offset_y
    local_y = -math.sin(surface_yaw_rad) * offset_x + math.cos(surface_yaw_rad) * offset_y
    if template.shape == "cylinder":
        extent_x = extent_y = float(template.radius_m or 0.0)
    else:
        size_x, size_y, _ = template.size_m or (0.0, 0.0, 0.0)
        relative_yaw = math.radians(yaw_deg - surface_yaw)
        extent_x = abs(math.cos(relative_yaw)) * size_x / 2.0 + abs(math.sin(relative_yaw)) * size_y / 2.0
        extent_y = abs(math.sin(relative_yaw)) * size_x / 2.0 + abs(math.cos(relative_yaw)) * size_y / 2.0
    half_surface_x = float(surface_size[0]) / 2.0
    half_surface_y = float(surface_size[1]) / 2.0
    if abs(local_x) + extent_x > half_surface_x or abs(local_y) + extent_y > half_surface_y:
        raise ValueError("element footprint does not fit inside support surface")


def _validate_spawn_clearance(
    *,
    template: _ElementTemplate,
    position_xy: Sequence[object],
    layout: Mapping[str, object],
) -> None:
    if not template.collision:
        return
    spawn = layout.get("spawn")
    if spawn is None:
        return
    if not isinstance(spawn, Mapping):
        raise TypeError("expanded layout spawn must be an object")
    spawn_position = spawn.get("position_m")
    if not isinstance(spawn_position, Sequence) or len(spawn_position) != 3:
        raise ValueError("expanded layout spawn.position_m must contain three values")
    clearance = float(spawn.get("clearance_radius_m", 0.0))
    if clearance < 0 or not math.isfinite(clearance):
        raise ValueError("expanded layout spawn clearance must be finite and non-negative")
    if template.shape == "cylinder":
        footprint_radius = float(template.radius_m or 0.0)
    else:
        size_x, size_y, _ = template.size_m or (0.0, 0.0, 0.0)
        footprint_radius = math.hypot(size_x / 2.0, size_y / 2.0)
    distance = math.hypot(
        float(position_xy[0]) - float(spawn_position[0]),
        float(position_xy[1]) - float(spawn_position[1]),
    )
    if distance < clearance + footprint_radius:
        raise ValueError("PhysicsShared element enters robot spawn clearance")


def compile_element_batch(
    batch: Mapping[str, object],
    layout: Mapping[str, object],
) -> CompiledElementBatch:
    """Compile one declarative batch against an expanded FactoryPark_HF layout."""

    if batch.get("schema") != ELEMENT_BATCH_SCHEMA:
        raise ValueError(f"element batch schema must be {ELEMENT_BATCH_SCHEMA!r}")
    unsupported_batch_fields = sorted(set(batch) - _BATCH_FIELDS)
    if unsupported_batch_fields:
        raise ValueError(f"element batch has unsupported fields {unsupported_batch_fields}")
    batch_id = str(batch.get("batch_id", "")).strip()
    if not batch_id:
        raise ValueError("element batch_id cannot be empty")
    if _SAFE_ID.fullmatch(batch_id) is None:
        raise ValueError("batch_id must match [a-z][a-z0-9_]{0,63}")
    raw_elements = batch.get("elements")
    if isinstance(raw_elements, (str, bytes)) or not isinstance(raw_elements, Sequence):
        raise TypeError("element batch elements must be an array")
    if not raw_elements:
        raise ValueError("element batch elements cannot be empty")

    layout_objects = layout.get("objects")
    if isinstance(layout_objects, (str, bytes)) or not isinstance(layout_objects, Sequence):
        raise TypeError("expanded layout objects must be an array")
    allocated_ids = {
        str(item.get("id"))
        for item in layout_objects
        if isinstance(item, Mapping) and item.get("id") is not None
    }
    objects: list[dict[str, object]] = []
    for index, raw in enumerate(raw_elements):
        if not isinstance(raw, Mapping):
            raise TypeError(f"elements[{index}] must be an object")
        unsupported_element_fields = sorted(set(raw) - _ELEMENT_FIELDS)
        if unsupported_element_fields:
            raise ValueError(
                f"elements[{index}] has unsupported fields {unsupported_element_fields}"
            )
        instance_key = str(raw.get("instance_key", "")).strip()
        element_type = str(raw.get("element_type", "")).strip()
        surface_id = str(raw.get("surface_id", "")).strip()
        template = _CATALOG.get(element_type)
        if template is None:
            raise ValueError(f"unknown element_type {element_type!r}")
        surface = _find_surface(layout, surface_id)
        surface_class = str(surface.get("semantic_class", ""))
        if surface_class not in template.allowed_surface_classes:
            raise ValueError(
                f"element_type {element_type!r} is not allowed on surface class "
                f"{surface_class!r}"
            )
        position_xy = raw["position_xy_m"]
        if not instance_key:
            raise ValueError(f"elements[{index}].instance_key cannot be empty")
        if _SAFE_ID.fullmatch(instance_key) is None:
            raise ValueError("instance_key must match [a-z][a-z0-9_]{0,63}")
        stable_id = f"element__{batch_id}__{instance_key}"
        if stable_id in allocated_ids:
            raise ValueError(f"duplicate stable id {stable_id!r}")
        allocated_ids.add(stable_id)
        if not isinstance(position_xy, Sequence) or len(position_xy) != 2:
            raise ValueError(f"elements[{index}].position_xy_m must contain two values")
        yaw_deg = float(raw.get("yaw_deg", 0.0))
        _validate_support_footprint(
            template=template,
            position_xy=position_xy,
            yaw_deg=yaw_deg,
            surface=surface,
        )
        _validate_spawn_clearance(template=template, position_xy=position_xy, layout=layout)
        surface_position = surface["position_m"]
        surface_size = surface["size_m"]
        element_half_height = (
            template.half_height_m
            if template.shape == "cylinder"
            else template.size_m[2] / 2.0  # type: ignore[index]
        )
        surface_top = float(surface_position[2]) + float(surface_size[2]) / 2.0  # type: ignore[index]
        item: dict[str, object] = {
            "id": stable_id,
            "semantic_class": template.semantic_class,
            "shape": template.shape,
            "position_m": [
                float(position_xy[0]),
                float(position_xy[1]),
                round(surface_top + element_half_height, 9),
            ],
            "yaw_deg": yaw_deg,
            "material": template.material,
            "collision": template.collision,
            "visual_only": template.visual_only,
            "element_provenance": {
                "batch_id": batch_id,
                "element_type": element_type,
                "instance_key": instance_key,
                "surface_id": surface_id,
            },
        }
        if template.shape == "box":
            item["size_m"] = list(template.size_m or ())
        else:
            item["radius_m"] = template.radius_m
            item["half_height_m"] = template.half_height_m
        objects.append(item)

    digest = hashlib.sha256(
        _canonical_json({"batch_id": batch_id, "objects": objects})
    ).hexdigest()
    return CompiledElementBatch(batch_id=batch_id, digest=digest, objects=tuple(objects))
