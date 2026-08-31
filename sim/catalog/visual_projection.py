"""Compile instance-agnostic Robot Visual Projection v1 contracts."""

from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Any

from .visual_binding import VisualBindingError


class VisualProjectionError(VisualBindingError):
    """Raised when a Robot Visual Projection cannot be compiled safely."""


_MESH_COMPONENT_CLASS = "/Script/Engine.StaticMeshComponent"
_PRIMITIVE_ASSETS = {
    "box": "/Engine/BasicShapes/Cube.Cube",
    "cylinder": "/Engine/BasicShapes/Cylinder.Cylinder",
    "sphere": "/Engine/BasicShapes/Sphere.Sphere",
}
_PRIMITIVE_SIZE_COUNTS = {"box": 3, "cylinder": 2, "sphere": 1, "capsule": 2}
_QUATERNION_NORM_TOLERANCE = 1e-6
_DEFAULT_METALLIC = 0.0
_DEFAULT_ROUGHNESS = 0.65


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise VisualProjectionError(f"{context} must be a mapping")
    return dict(value)


def _required_string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise VisualProjectionError(f"{context} must be a non-empty string")
    return value.strip()


def _vector(value: Any, size: int, context: str, *, positive: bool = False) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
        raise VisualProjectionError(f"{context} must contain exactly {size} values")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise VisualProjectionError(f"{context}[{index}] must be finite numeric data")
        number = float(item)
        if not math.isfinite(number) or (positive and number <= 0.0):
            qualifier = "finite positive" if positive else "finite"
            raise VisualProjectionError(f"{context}[{index}] must be {qualifier} numeric data")
        result.append(number)
    return result


def _unit_interval(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VisualProjectionError(f"{context} must be finite numeric data in [0, 1]")
    number = float(value)
    if not math.isfinite(number) or number < 0.0 or number > 1.0:
        raise VisualProjectionError(f"{context} must be finite numeric data in [0, 1]")
    return number


def _rgba(value: Any, context: str) -> list[float]:
    rgba = _vector(value, 4, context)
    for index, component in enumerate(rgba):
        if component < 0.0 or component > 1.0:
            raise VisualProjectionError(f"{context}[{index}] must be in [0, 1]")
    return rgba


def _unit_quaternion(value: Any, context: str) -> list[float]:
    """Validate a local visual quaternion as finite and normalized within 1e-6."""

    quaternion = _vector(value, 4, context)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if abs(norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
        raise VisualProjectionError(
            f"{context} must be a normalized unit quaternion within {_QUATERNION_NORM_TOLERANCE:g}"
        )
    return quaternion


def _component_material(visual: Mapping[str, Any], context: str) -> dict[str, Any]:
    material_key = visual.get("material")
    rgba = visual.get("rgba")
    if rgba is not None:
        return {
            "source": "mjcf_geom_rgba",
            "key": None,
            "pbr": {
                "base_color_rgba": _rgba(rgba, f"{context}.rgba"),
                "metallic": _DEFAULT_METALLIC,
                "roughness": _DEFAULT_ROUGHNESS,
            },
        }
    if material_key is not None:
        material_name = _required_string(material_key, f"{context}.material")
        material_rgba = visual.get("material_rgba")
        if material_rgba is not None:
            return {
                "source": "mjcf_material_rgba",
                "key": material_name,
                "pbr": {
                    "base_color_rgba": _rgba(material_rgba, f"{context}.material_rgba"),
                    "metallic": _DEFAULT_METALLIC,
                    "roughness": _DEFAULT_ROUGHNESS,
                },
            }
    return {
        "source": "compiler_default",
        "key": None,
        "pbr": {
            "base_color_rgba": [0.5, 0.5, 0.5, 1.0],
            "metallic": _DEFAULT_METALLIC,
            "roughness": _DEFAULT_ROUGHNESS,
        },
    }


def _relative_posix_path(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise VisualProjectionError(f"{context} must be a non-empty string")
    result = value
    path = PurePosixPath(result)
    parts = result.split("/")
    if (
        result != result.strip()
        or any(char.isspace() for char in result)
        or "\\" in result
        or path.is_absolute()
        or "//" in result
        or any(part in ("", ".", "..") for part in parts)
        or re.match(r"^[A-Za-z]:", result) is not None
    ):
        raise VisualProjectionError(f"{context} must be a safe asset-root-relative POSIX path")
    return result


def _cooked_asset_path(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise VisualProjectionError(f"{context} must be a non-empty string")
    result = value
    parts = result.split("/")
    if (
        result != result.strip()
        or any(char.isspace() for char in result)
        or "\\" in result
        or not result.startswith("/Game/")
        or "//" in result
        or result.endswith("/")
        or any(part in ("", ".", "..") for part in parts[2:])
    ):
        raise VisualProjectionError(f"{context} must be a cooked /Game/ Unreal asset path")
    return result


def _validate_manifest_schema(visual_manifest: Mapping[str, Any]) -> None:
    schema = _required_string(visual_manifest.get("schema"), "visual_manifest.schema")
    if schema != "lingtu.sim.robot-visual-manifest.v1":
        raise VisualProjectionError(f"unsupported visual manifest schema {schema!r}")


def _component_geometry(visual: Mapping[str, Any], context: str) -> dict[str, Any]:
    geometry = _mapping(visual.get("geometry"), f"{context}.geometry")
    kind = _required_string(geometry.get("kind"), f"{context}.geometry.kind")
    if kind == "mesh":
        expected = {"kind", "source_mesh", "mesh", "scale"}
        if set(geometry) != expected:
            raise VisualProjectionError(f"{context}.geometry has invalid mesh fields")
        return {
            "kind": kind,
            "source_mesh": _relative_posix_path(geometry["source_mesh"], f"{context}.geometry.source_mesh"),
            "mesh": _required_string(geometry["mesh"], f"{context}.geometry.mesh"),
            "scale": _vector(geometry["scale"], 3, f"{context}.geometry.scale"),
        }
    if kind == "primitive":
        expected = {"kind", "primitive", "size"}
        if set(geometry) != expected:
            raise VisualProjectionError(f"{context}.geometry has invalid primitive fields")
        primitive = _required_string(geometry["primitive"], f"{context}.geometry.primitive")
        size_count = _PRIMITIVE_SIZE_COUNTS.get(primitive)
        if size_count is None:
            raise VisualProjectionError(f"{context}.geometry.primitive {primitive!r} is unsupported")
        return {
            "kind": kind,
            "primitive": primitive,
            "size": _vector(geometry["size"], size_count, f"{context}.geometry.size", positive=True),
        }
    raise VisualProjectionError(f"{context}.geometry.kind {kind!r} is unsupported")


def _unreal_representation(geometry: Mapping[str, Any], cooked_asset: str | None, context: str) -> dict[str, Any]:
    if geometry["kind"] == "mesh":
        if cooked_asset is None:
            raise VisualProjectionError(f"{context} is missing an explicit cooked Unreal asset mapping")
        return {
            "representation": "static_mesh",
            "component_class": _MESH_COMPONENT_CLASS,
            "asset_path": _cooked_asset_path(cooked_asset, f"{context}.asset_path"),
        }

    primitive = geometry["primitive"]
    size = geometry["size"]
    if primitive == "box":
        dimensions = [2.0 * value for value in size]
        representation = {
            "representation": "static_mesh",
            "component_class": _MESH_COMPONENT_CLASS,
            "asset_path": _PRIMITIVE_ASSETS[primitive],
            "dimensions_m": dimensions,
        }
    elif primitive == "cylinder":
        radius, half_height = size
        representation = {
            "representation": "static_mesh",
            "component_class": _MESH_COMPONENT_CLASS,
            "asset_path": _PRIMITIVE_ASSETS[primitive],
            "dimensions_m": [2.0 * radius, 2.0 * radius, 2.0 * half_height],
        }
    elif primitive == "sphere":
        diameter = 2.0 * size[0]
        representation = {
            "representation": "static_mesh",
            "component_class": _MESH_COMPONENT_CLASS,
            "asset_path": _PRIMITIVE_ASSETS[primitive],
            "dimensions_m": [diameter, diameter, diameter],
        }
    else:
        radius, cylinder_half_length = size
        representation = {
            "representation": "component",
            "component_class": "/Script/Engine.CapsuleComponent",
            "radius_m": radius,
            "cylinder_half_length_m": cylinder_half_length,
            "capsule_half_height_m": radius + cylinder_half_length,
            "dimensions_m": [
                2.0 * radius,
                2.0 * radius,
                2.0 * (radius + cylinder_half_length),
            ],
        }
    return representation


def _source_provenance(
    visual_manifest: Mapping[str, Any],
    geometry: Mapping[str, Any],
) -> dict[str, Any]:
    package = _mapping(visual_manifest.get("package"), "visual_manifest.package")
    provenance: dict[str, Any] = {
        "manifest_schema": "lingtu.sim.robot-visual-manifest.v1",
        "package": {
            "id": _required_string(package.get("id"), "visual_manifest.package.id"),
            "version": _required_string(package.get("version"), "visual_manifest.package.version"),
            "manifest": _relative_posix_path(package.get("manifest"), "visual_manifest.package.manifest"),
        },
        "mjcf": {
            "path": _relative_posix_path(visual_manifest.get("mjcf"), "visual_manifest.mjcf"),
        },
    }
    if geometry["kind"] == "mesh":
        provenance["geometry_source"] = {
            "kind": "mesh",
            "source_mesh": geometry["source_mesh"],
            "mesh": geometry["mesh"],
        }
    else:
        provenance["geometry_source"] = {
            "kind": "primitive",
            "primitive": geometry["primitive"],
        }
    return provenance


@dataclass(frozen=True)
class RobotVisualProjection:
    """Instance-agnostic visual projection document.

    The compatibility name is retained for callers that compile robot visual
    manifests; validation also uses this immutable value for generic entities.
    """

    body: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return dict(self.body)


def compile_robot_visual_projection(
    visual_manifest: Mapping[str, Any],
    asset_bindings: Mapping[str, str] | None = None,
) -> RobotVisualProjection:
    """Compile one manifest into a deterministic Unreal-facing visual projection."""

    if not isinstance(visual_manifest, Mapping):
        raise VisualProjectionError("visual_manifest must be a mapping")
    _validate_manifest_schema(visual_manifest)
    bindings = {} if asset_bindings is None else dict(asset_bindings)
    for key, value in bindings.items():
        if not isinstance(key, str) or not key.strip() or not isinstance(value, str) or not value.strip():
            raise VisualProjectionError("asset_bindings keys and values must be non-empty strings")

    package = _mapping(visual_manifest.get("package"), "visual_manifest.package")
    binding = _required_string(visual_manifest.get("binding"), "visual_manifest.binding")
    visuals = visual_manifest.get("visuals")
    if not isinstance(visuals, Sequence) or isinstance(visuals, (str, bytes)):
        raise VisualProjectionError("visual_manifest.visuals must be an array")

    components: list[dict[str, Any]] = []
    seen_visual_frames: set[str] = set()
    seen_visual_ids: set[tuple[str, str]] = set()
    for index, raw_visual in enumerate(visuals):
        visual = _mapping(raw_visual, f"visual_manifest.visuals[{index}]")
        context = f"visual_manifest.visuals[{index}]"
        local_body_id = _required_string(visual.get("body"), f"{context}.body")
        body_frame_id = _required_string(visual.get("body_frame_id"), f"{context}.body_frame_id")
        visual_id = _required_string(visual.get("visual_id"), f"{context}.visual_id")
        visual_frame_id = _required_string(visual.get("visual_frame_id"), f"{context}.visual_frame_id")
        if visual_frame_id in seen_visual_frames or (local_body_id, visual_id) in seen_visual_ids:
            raise VisualProjectionError(f"duplicate stable visual ID {visual_frame_id!r}")
        seen_visual_frames.add(visual_frame_id)
        seen_visual_ids.add((local_body_id, visual_id))
        asset_key = _required_string(visual.get("asset_key"), f"{context}.asset_key")
        geometry = _component_geometry(visual, context)
        cooked_asset = None
        if geometry["kind"] == "mesh":
            cooked_asset = bindings.get(asset_key) or bindings.get(geometry["mesh"])
            if cooked_asset is None:
                raise VisualProjectionError(
                    f"mesh visual {asset_key!r} has no explicit cooked Unreal asset mapping"
                )
        material = visual.get("material")
        if material is not None:
            material = _required_string(material, f"{context}.material")
        component = {
            "local_body_id": local_body_id,
            "body_frame_id": body_frame_id,
            "visual_id": visual_id,
            "visual_frame_id": visual_frame_id,
            "asset_key": asset_key,
            "geometry": geometry,
            "unreal": _unreal_representation(geometry, cooked_asset, f"visual {asset_key!r}"),
            "local_transform": {
                "position_m": _vector(visual.get("pos"), 3, f"{context}.pos"),
                "quaternion_wxyz": _unit_quaternion(visual.get("quat"), f"{context}.quat"),
                "scale": geometry.get("scale", [1.0, 1.0, 1.0]),
            },
            "material_key": material,
            "material": _component_material(visual, context),
            "source": _source_provenance(visual_manifest, geometry),
        }
        components.append(component)

    components.sort(key=lambda item: (item["local_body_id"], item["visual_frame_id"], item["asset_key"]))
    body = {
        "schema": "lingtu.sim.robot-visual-projection.v1",
        "binding": binding,
        "package": {
            "id": _required_string(package.get("id"), "visual_manifest.package.id"),
            "version": _required_string(package.get("version"), "visual_manifest.package.version"),
            "manifest": _relative_posix_path(package.get("manifest"), "visual_manifest.package.manifest"),
        },
        "mjcf": {
            "path": _relative_posix_path(visual_manifest.get("mjcf"), "visual_manifest.mjcf"),
        },
        "components": components,
    }
    return RobotVisualProjection(body=body)


def _exact_fields(value: Mapping[str, Any], expected: set[str], context: str) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise VisualProjectionError(
            f"{context} fields do not match the contract; missing={missing}, extra={extra}"
        )


def _projection_package(value: Any, context: str) -> dict[str, Any]:
    package = _mapping(value, context)
    _exact_fields(package, {"id", "version", "manifest"}, context)
    return {
        "id": _required_string(package["id"], f"{context}.id"),
        "version": _required_string(package["version"], f"{context}.version"),
        "manifest": _relative_posix_path(package["manifest"], f"{context}.manifest"),
    }


def _projection_mjcf(value: Any, context: str) -> dict[str, Any]:
    mjcf = _mapping(value, context)
    _exact_fields(mjcf, {"path"}, context)
    return {
        "path": _relative_posix_path(mjcf["path"], f"{context}.path"),
    }


def _reject_instance_identity(value: Any, context: str = "projection") -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            if key == "instance_id":
                raise VisualProjectionError(f"{context} must not contain instance_id")
            _reject_instance_identity(item, f"{context}.{key}")
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, item in enumerate(value):
            _reject_instance_identity(item, f"{context}[{index}]")


def _validated_unreal(
    value: Any,
    geometry: Mapping[str, Any],
    context: str,
) -> dict[str, Any]:
    unreal = _mapping(value, context)
    if geometry["kind"] == "mesh":
        _exact_fields(
            unreal,
            {"representation", "component_class", "asset_path"},
            context,
        )
        if unreal.get("representation") != "static_mesh":
            raise VisualProjectionError(f"{context}.representation must be 'static_mesh'")
        if unreal.get("component_class") != _MESH_COMPONENT_CLASS:
            raise VisualProjectionError(f"{context}.component_class does not match mesh geometry")
        return {
            "representation": "static_mesh",
            "component_class": _MESH_COMPONENT_CLASS,
            "asset_path": _cooked_asset_path(unreal.get("asset_path"), f"{context}.asset_path"),
        }

    expected = _unreal_representation(geometry, None, context)
    _exact_fields(unreal, set(expected), context)
    normalized: dict[str, Any] = {}
    for key, expected_value in expected.items():
        if key == "dimensions_m":
            normalized[key] = _vector(unreal.get(key), 3, f"{context}.{key}", positive=True)
        elif key in (
            "radius_m",
            "cylinder_half_length_m",
            "capsule_half_height_m",
        ):
            normalized[key] = _vector([unreal.get(key)], 1, f"{context}.{key}", positive=True)[0]
        else:
            normalized[key] = unreal.get(key)
        if not _representation_values_equal(normalized[key], expected_value):
            raise VisualProjectionError(
                f"{context}.{key} does not agree with {geometry['primitive']!r} geometry"
            )
    return normalized


def _representation_values_equal(left: Any, right: Any) -> bool:
    if isinstance(left, float) and isinstance(right, (int, float)):
        return math.isclose(left, float(right), rel_tol=0.0, abs_tol=1e-12)
    if isinstance(left, list) and isinstance(right, list) and len(left) == len(right):
        return all(
            math.isclose(float(left_item), float(right_item), rel_tol=0.0, abs_tol=1e-12)
            for left_item, right_item in zip(left, right, strict=True)
        )
    return bool(left == right)


def _validated_source(
    value: Any,
    geometry: Mapping[str, Any],
    package: Mapping[str, Any],
    mjcf: Mapping[str, Any],
    manifest_schema: str,
    context: str,
) -> dict[str, Any]:
    source = _mapping(value, context)
    _exact_fields(
        source,
        {"manifest_schema", "package", "mjcf", "geometry_source"},
        context,
    )
    if source.get("manifest_schema") != manifest_schema:
        raise VisualProjectionError(f"{context}.manifest_schema is unsupported")
    source_package = _projection_package(source.get("package"), f"{context}.package")
    source_mjcf = _projection_mjcf(source.get("mjcf"), f"{context}.mjcf")
    if source_package != package or source_mjcf != mjcf:
        raise VisualProjectionError(f"{context} provenance does not match projection package/MJCF")

    geometry_source = _mapping(source.get("geometry_source"), f"{context}.geometry_source")
    if geometry["kind"] == "mesh":
        _exact_fields(
            geometry_source,
            {"kind", "source_mesh", "mesh"},
            f"{context}.geometry_source",
        )
        expected_geometry_source = {
            "kind": "mesh",
            "source_mesh": geometry["source_mesh"],
            "mesh": geometry["mesh"],
        }
    else:
        _exact_fields(
            geometry_source,
            {"kind", "primitive"},
            f"{context}.geometry_source",
        )
        expected_geometry_source = {
            "kind": "primitive",
            "primitive": geometry["primitive"],
        }
    if geometry_source != expected_geometry_source:
        raise VisualProjectionError(f"{context}.geometry_source does not match geometry")
    return {
        "manifest_schema": manifest_schema,
        "package": dict(package),
        "mjcf": dict(mjcf),
        "geometry_source": expected_geometry_source,
    }


def _validated_material(value: Any, context: str) -> dict[str, Any]:
    material = _mapping(value, context)
    _exact_fields(material, {"source", "key", "pbr"}, context)
    source = _required_string(material.get("source"), f"{context}.source")
    if source not in {"mjcf_geom_rgba", "mjcf_material_rgba", "compiler_default"}:
        raise VisualProjectionError(f"{context}.source is unsupported")
    raw_key = material.get("key")
    key = None if raw_key is None else _required_string(raw_key, f"{context}.key")
    if source == "mjcf_material_rgba" and key is None:
        raise VisualProjectionError(f"{context}.key must identify the named MJCF material")
    if source != "mjcf_material_rgba" and key is not None:
        raise VisualProjectionError(f"{context}.key must be null unless source is mjcf_material_rgba")
    pbr = _mapping(material.get("pbr"), f"{context}.pbr")
    _exact_fields(
        pbr,
        {"base_color_rgba", "metallic", "roughness"},
        f"{context}.pbr",
    )
    return {
        "source": source,
        "key": key,
        "pbr": {
            "base_color_rgba": _rgba(
                pbr.get("base_color_rgba"),
                f"{context}.pbr.base_color_rgba",
            ),
            "metallic": _unit_interval(pbr.get("metallic"), f"{context}.pbr.metallic"),
            "roughness": _unit_interval(pbr.get("roughness"), f"{context}.pbr.roughness"),
        },
    }


def _validated_component(
    value: Any,
    index: int,
    package: Mapping[str, Any],
    mjcf: Mapping[str, Any],
    manifest_schema: str,
) -> dict[str, Any]:
    context = f"projection.components[{index}]"
    component = _mapping(value, context)
    component_fields = {
        "local_body_id",
        "body_frame_id",
        "visual_id",
        "visual_frame_id",
        "asset_key",
        "geometry",
        "unreal",
        "local_transform",
        "material_key",
        "material",
        "source",
    }
    _exact_fields(component, component_fields, context)
    geometry = _component_geometry(component, context)
    local_transform = _mapping(component.get("local_transform"), f"{context}.local_transform")
    _exact_fields(
        local_transform,
        {"position_m", "quaternion_wxyz", "scale"},
        f"{context}.local_transform",
    )
    expected_scale = geometry.get("scale", [1.0, 1.0, 1.0])
    normalized_scale = _vector(
        local_transform.get("scale"), 3, f"{context}.local_transform.scale"
    )
    if normalized_scale != expected_scale:
        raise VisualProjectionError(f"{context}.local_transform.scale does not match geometry")
    material = component.get("material_key")
    if material is not None:
        material = _required_string(material, f"{context}.material_key")
    normalized = {
        "local_body_id": _required_string(component.get("local_body_id"), f"{context}.local_body_id"),
        "body_frame_id": _required_string(component.get("body_frame_id"), f"{context}.body_frame_id"),
        "visual_id": _required_string(component.get("visual_id"), f"{context}.visual_id"),
        "visual_frame_id": _required_string(
            component.get("visual_frame_id"), f"{context}.visual_frame_id"
        ),
        "asset_key": _required_string(component.get("asset_key"), f"{context}.asset_key"),
        "geometry": geometry,
        "unreal": _validated_unreal(component.get("unreal"), geometry, f"{context}.unreal"),
        "local_transform": {
            "position_m": _vector(
                local_transform.get("position_m"), 3, f"{context}.local_transform.position_m"
            ),
            "quaternion_wxyz": _unit_quaternion(
                local_transform.get("quaternion_wxyz"),
                f"{context}.local_transform.quaternion_wxyz",
            ),
            "scale": normalized_scale,
        },
        "material_key": material,
    }
    normalized["material"] = _validated_material(
        component.get("material"), f"{context}.material"
    )
    normalized["source"] = _validated_source(
        component.get("source"),
        geometry,
        package,
        mjcf,
        manifest_schema,
        f"{context}.source",
    )
    return normalized


def _validate_visual_projection(
    document: Mapping[str, Any],
    *,
    projection_schema: str,
    manifest_schema: str,
) -> RobotVisualProjection:
    if not isinstance(document, Mapping):
        raise VisualProjectionError("projection must be a mapping")
    _reject_instance_identity(document)
    projection = dict(document)
    _exact_fields(
        projection,
        {"schema", "binding", "package", "mjcf", "components"},
        "projection",
    )
    if projection.get("schema") != projection_schema:
        raise VisualProjectionError("projection.schema is unsupported")
    package = _projection_package(projection.get("package"), "projection.package")
    mjcf = _projection_mjcf(projection.get("mjcf"), "projection.mjcf")
    raw_components = projection.get("components")
    if (
        not isinstance(raw_components, Sequence)
        or isinstance(raw_components, (str, bytes))
        or not raw_components
    ):
        raise VisualProjectionError("projection.components must be a non-empty array")
    components = [
        _validated_component(item, index, package, mjcf, manifest_schema)
        for index, item in enumerate(raw_components)
    ]
    visual_frames: set[str] = set()
    local_visual_ids: set[tuple[str, str]] = set()
    for component in components:
        frame = component["visual_frame_id"]
        local_id = (component["local_body_id"], component["visual_id"])
        if frame in visual_frames or local_id in local_visual_ids:
            raise VisualProjectionError(f"duplicate local visual ID {frame!r}")
        visual_frames.add(frame)
        local_visual_ids.add(local_id)
    sorted_components = sorted(
        components,
        key=lambda item: (item["local_body_id"], item["visual_frame_id"], item["asset_key"]),
    )
    if components != sorted_components:
        raise VisualProjectionError("projection.components must use deterministic canonical ordering")
    body = {
        "schema": projection_schema,
        "binding": _required_string(projection.get("binding"), "projection.binding"),
        "package": package,
        "mjcf": mjcf,
        "components": components,
    }
    return RobotVisualProjection(body=body)


def validate_robot_visual_projection(document: Mapping[str, Any]) -> RobotVisualProjection:
    """Validate and normalize one complete Robot Visual Projection v1 document."""

    return _validate_visual_projection(
        document,
        projection_schema="lingtu.sim.robot-visual-projection.v1",
        manifest_schema="lingtu.sim.robot-visual-manifest.v1",
    )


def validate_entity_visual_projection(document: Mapping[str, Any]) -> RobotVisualProjection:
    """Validate a generic ScenarioPackage-owned Entity Visual Projection v1."""

    return _validate_visual_projection(
        document,
        projection_schema="lingtu.sim.entity-visual-projection.v1",
        manifest_schema="lingtu.sim.scenario-package.v1",
    )


def validate_robot_visual_projection_matches_manifest(
    document: Mapping[str, Any],
    visual_manifest: Mapping[str, Any],
) -> RobotVisualProjection:
    """Validate a projection against a freshly compiled Robot Visual Manifest.

    Mesh projections may bind manifest visuals to legitimate cooked Unreal assets,
    so this check reuses the validated mesh asset paths as the compile-time
    binding map. Every manifest-derived component field must still match the
    freshly compiled projection one-to-one, and deleted or altered components
    are rejected.
    """

    projection = validate_robot_visual_projection(document).to_dict()
    asset_bindings = {
        component["asset_key"]: component["unreal"]["asset_path"]
        for component in projection["components"]
        if component["geometry"]["kind"] == "mesh"
    }
    try:
        expected = compile_robot_visual_projection(visual_manifest, asset_bindings).to_dict()
    except VisualProjectionError as exc:
        raise VisualProjectionError(
            "projection components do not match freshly compiled visual manifest"
        ) from exc
    if projection != expected:
        raise VisualProjectionError(
            "projection components do not match freshly compiled visual manifest"
        )
    return RobotVisualProjection(body=projection)
