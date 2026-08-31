"""Compile deterministic robot visual binding manifests from RobotPackage MJCF."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class VisualBindingError(ValueError):
    """Raised when a robot visual manifest cannot be compiled safely."""


_PACKAGE_SUFFIXES = (".package.yaml", ".package.yml")
_ASSET_TOKEN_SAFE = frozenset("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.-")
_UNSUPPORTED_ORIENTATION_ATTRIBUTES = ("euler", "axisangle", "xyaxes", "zaxis")
_PRIMITIVE_TYPES = frozenset(("box", "cylinder", "sphere", "capsule"))
_V1_DEFAULT_SENSITIVE_ATTRIBUTES = frozenset(
    (
        "type",
        "mesh",
        "size",
        "pos",
        "quat",
        "fromto",
        "euler",
        "axisangle",
        "xyaxes",
        "zaxis",
        "rgba",
        "material",
        "group",
    )
)


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise VisualBindingError(f"{context} must be a mapping")
    return dict(value)


def _string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise VisualBindingError(f"{context} must be a non-empty string")
    return value.strip()


def _finite_vector(text: str | None, size: int, default: tuple[float, ...], context: str) -> list[float]:
    if text is None:
        return list(default)
    parts = text.split()
    if len(parts) != size:
        raise VisualBindingError(f"{context} must contain exactly {size} values")
    result: list[float] = []
    for index, part in enumerate(parts):
        try:
            value = float(part)
        except ValueError as exc:
            raise VisualBindingError(f"{context}[{index}] must be finite numeric data") from exc
        if not math.isfinite(value):
            raise VisualBindingError(f"{context}[{index}] must be finite numeric data")
        result.append(value)
    return result


def _positive_vector(text: str | None, size: int, context: str) -> list[float]:
    if text is None:
        raise VisualBindingError(f"{context} must contain exactly {size} values")
    result = _finite_vector(text, size, (), context)
    if any(value <= 0.0 for value in result):
        raise VisualBindingError(f"{context} must contain finite positive values")
    return result


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise VisualBindingError(f"cannot read RobotPackage manifest {path}: {exc}") from exc
    return _mapping(value, str(path))


def _package_manifest_path(package_dir: Path) -> Path:
    candidates: list[Path] = []
    for suffix in _PACKAGE_SUFFIXES:
        candidates.extend(sorted(package_dir.glob(f"*{suffix}")))
    if not candidates:
        raise VisualBindingError(f"RobotPackage manifest does not exist in {package_dir}")
    if len(candidates) > 1:
        names = ", ".join(path.name for path in candidates)
        raise VisualBindingError(f"RobotPackage directory has multiple manifests: {names}")
    return candidates[0]


def _relative_to_package(path: Path, package_dir: Path, context: str) -> str:
    try:
        return path.resolve().relative_to(package_dir.resolve()).as_posix()
    except ValueError as exc:
        raise VisualBindingError(f"{context} escapes robot package: {path}") from exc


def _package_directory(package_dir: Path, value: Any, context: str) -> Path:
    relative = _string(value, context)
    parts = relative.split("/")
    if (
        not parts
        or any(not part or part in {".", ".."} for part in parts)
        or "\\" in relative
        or relative.startswith("/")
        or ":" in relative
        or any(character.isspace() for character in relative)
    ):
        raise VisualBindingError(f"{context} escapes robot package: {relative}")

    current = package_dir
    for part in parts:
        current /= part
        if current.is_symlink():
            raise VisualBindingError(f"{context} must not traverse symbolic links: {relative}")
    candidate = current.resolve()
    _relative_to_package(candidate, package_dir, context)
    if not candidate.is_dir():
        raise VisualBindingError(f"{context} does not exist: {relative}")
    return candidate


def _repository_asset_root(package_dir: Path) -> Path:
    if (
        len(package_dir.parents) >= 4
        and package_dir.parents[1].name == "packages"
        and package_dir.parents[2].name == "sim"
    ):
        return package_dir.parents[3]
    return package_dir


def _asset_root_for_mjcf(package_dir: Path, repository_asset_root: Path, mjcf: Path) -> Path:
    try:
        mjcf.resolve().relative_to(package_dir.resolve())
        return package_dir
    except ValueError:
        return repository_asset_root


def _relative_to_asset_root(path: Path, asset_root: Path, context: str) -> str:
    try:
        return path.resolve().relative_to(asset_root.resolve()).as_posix()
    except ValueError as exc:
        raise VisualBindingError(f"{context} escapes robot package asset root: {path}") from exc


def _asset_path(asset_root: Path, base_dir: Path, value: str, context: str) -> Path:
    candidate = (base_dir / value).resolve()
    _relative_to_asset_root(candidate, asset_root, context)
    if not candidate.is_file():
        raise VisualBindingError(f"{context} does not exist: {value}")
    return candidate


def _asset_token(value: str) -> str:
    """Return a deterministic, reversible token without changing runtime frame names."""

    encoded: list[str] = []
    for char in value:
        if char in _ASSET_TOKEN_SAFE:
            encoded.append(char)
        else:
            encoded.extend(f"%{byte:02X}" for byte in char.encode("utf-8"))
    return "".join(encoded)


@dataclass(frozen=True)
class RobotVisualManifest:
    """In-memory Robot Visual Manifest v1."""

    body: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return dict(self.body)


def compile_robot_visual_manifest(package_dir: Path, mjcf_path: Path | None = None) -> RobotVisualManifest:
    """Compile a Robot Visual Manifest v1 from a RobotPackage directory and MJCF."""

    package_dir = Path(package_dir).resolve()
    if not package_dir.is_dir():
        raise VisualBindingError(f"RobotPackage directory does not exist: {package_dir}")

    repository_asset_root = _repository_asset_root(package_dir)
    manifest_path = _package_manifest_path(package_dir)
    package = _load_yaml(manifest_path)
    if _string(package.get("kind"), f"{manifest_path}.kind") != "robot":
        raise VisualBindingError(f"{manifest_path}.kind must be 'robot'")
    visual_config = _mapping(package.get("visual"), f"{manifest_path}.visual")
    binding = _string(visual_config.get("binding"), f"{manifest_path}.visual.binding")
    visual_mesh_root = None
    if "source_mesh_root" in visual_config:
        visual_mesh_root = _package_directory(
            package_dir,
            visual_config["source_mesh_root"],
            f"{manifest_path}.visual.source_mesh_root",
        )

    if mjcf_path is None:
        physics = _mapping(package.get("physics"), f"{manifest_path}.physics")
        mjcf = _asset_path(
            repository_asset_root,
            manifest_path.parent,
            _string(physics.get("mjcf"), f"{manifest_path}.physics.mjcf"),
            "physics.mjcf",
        )
    else:
        mjcf = Path(mjcf_path).resolve()
        _relative_to_asset_root(mjcf, repository_asset_root, "mjcf_path")
        if not mjcf.is_file():
            raise VisualBindingError(f"mjcf_path does not exist: {mjcf}")
    asset_root = _asset_root_for_mjcf(package_dir, repository_asset_root, mjcf)
    _relative_to_asset_root(mjcf, asset_root, "mjcf")

    root = _parse_mjcf(mjcf)
    mesh_assets = _collect_mesh_assets(
        root,
        asset_root,
        mjcf,
        package_dir=package_dir,
        visual_mesh_root=visual_mesh_root,
    )
    materials = _collect_materials(root, mjcf)
    default_geoms = _collect_default_geoms(root)
    visuals = _collect_body_visuals(
        root,
        mesh_assets,
        materials,
        default_geoms,
        binding,
        asset_root,
    )
    if not visuals:
        raise VisualBindingError("MJCF has no supported visual geoms")
    body = {
        "schema": "lingtu.sim.robot-visual-manifest.v1",
        "package": {
            "id": _string(package.get("id"), f"{manifest_path}.id"),
            "version": _string(package.get("version"), f"{manifest_path}.version"),
            "manifest": _relative_to_package(manifest_path, package_dir, "manifest"),
        },
        "binding": binding,
        "mjcf": _relative_to_asset_root(mjcf, asset_root, "mjcf"),
        "visuals": visuals,
    }
    return RobotVisualManifest(body=body)


def _parse_mjcf(path: Path) -> ET.Element:
    try:
        return ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise VisualBindingError(f"cannot parse MuJoCo XML {path}: {exc}") from exc


def _collect_mesh_assets(
    root: ET.Element,
    asset_root: Path,
    mjcf: Path,
    *,
    package_dir: Path,
    visual_mesh_root: Path | None,
) -> dict[str, dict[str, Any]]:
    compiler = root.find("compiler")
    meshdir_text = compiler.attrib.get("meshdir", ".") if compiler is not None else "."
    meshdir = (mjcf.parent / meshdir_text).resolve()
    _relative_to_asset_root(meshdir, asset_root, "compiler.meshdir")

    meshes: dict[str, dict[str, Any]] = {}
    for mesh in root.findall("./asset/mesh"):
        name = _string(mesh.attrib.get("name"), f"{mjcf}.asset.mesh.name")
        if name in meshes:
            raise VisualBindingError(f"duplicate mesh asset {name!r}")
        file_name = _string(mesh.attrib.get("file"), f"{mjcf}.asset.mesh[{name}].file")
        physics_source = _asset_path(asset_root, meshdir, file_name, f"mesh asset {name!r}")
        source = physics_source
        source_root = asset_root
        if visual_mesh_root is not None:
            source = _asset_path(
                package_dir,
                visual_mesh_root,
                file_name,
                f"visual mesh asset {name!r}",
            )
            _relative_to_asset_root(
                source,
                visual_mesh_root,
                f"visual mesh asset {name!r}",
            )
            source_root = package_dir
        meshes[name] = {
            "source": source,
            "source_root": source_root,
            "scale": _finite_vector(mesh.attrib.get("scale"), 3, (1.0, 1.0, 1.0), f"{mjcf}.asset.mesh[{name}].scale"),
        }
    return meshes


def _collect_materials(root: ET.Element, mjcf: Path) -> dict[str, dict[str, Any]]:
    materials: dict[str, dict[str, Any]] = {}
    for material in root.findall("./asset/material"):
        name = _string(material.attrib.get("name"), f"{mjcf}.asset.material.name")
        if name in materials:
            raise VisualBindingError(f"duplicate material asset {name!r}")
        rgba = material.attrib.get("rgba")
        materials[name] = {
            "rgba": None if rgba is None else _finite_vector(rgba, 4, (), f"{mjcf}.asset.material[{name}].rgba")
        }
    return materials


def _collect_default_geoms(root: ET.Element) -> dict[str | None, dict[str, str]]:
    defaults: dict[str | None, dict[str, str]] = {}

    def visit(element: ET.Element, inherited: Mapping[str, str]) -> None:
        effective = dict(inherited)
        geom = element.find("geom")
        if geom is not None:
            effective.update(geom.attrib)
        class_name = element.attrib.get("class")
        if class_name in defaults:
            raise VisualBindingError(f"duplicate default class {class_name!r}")
        defaults[class_name] = effective
        for child in element.findall("default"):
            visit(child, effective)

    root_default = root.find("default")
    if root_default is not None:
        visit(root_default, {})
    return defaults


def _explicit_int(value: str | None, context: str) -> int | None:
    if value is None:
        return None
    try:
        result = int(value)
    except ValueError as exc:
        raise VisualBindingError(f"{context} must be integer data") from exc
    if str(result) != value.strip():
        raise VisualBindingError(f"{context} must be integer data")
    return result


def _effective_alpha(
    geom: ET.Element,
    materials: Mapping[str, Mapping[str, Any]],
    context: str,
) -> float | None:
    rgba = geom.attrib.get("rgba")
    if rgba is not None:
        return _finite_vector(rgba, 4, (), f"{context}.rgba")[3]
    material_name = geom.attrib.get("material")
    if material_name is None:
        return None
    material = materials.get(material_name)
    if material is None:
        raise VisualBindingError(f"{context} references unknown material asset {material_name!r}")
    material_rgba = material.get("rgba")
    return None if material_rgba is None else float(material_rgba[3])


def _selected_default_class(
    body: ET.Element,
    geom: ET.Element,
    parents: Mapping[ET.Element, ET.Element],
) -> str | None:
    if "class" in geom.attrib:
        return geom.attrib["class"]
    current: ET.Element | None = body
    while current is not None:
        if current.tag == "body" and "childclass" in current.attrib:
            return current.attrib["childclass"]
        current = parents.get(current)
    return None


def _reject_ambiguous_defaults(
    body: ET.Element,
    geom: ET.Element,
    default_geoms: Mapping[str | None, Mapping[str, str]],
    parents: Mapping[ET.Element, ET.Element],
    context: str,
) -> None:
    selected_class = _selected_default_class(body, geom, parents)
    if selected_class is not None and selected_class not in default_geoms:
        raise VisualBindingError(f"{context} references unknown default class {selected_class!r}")
    inherited = default_geoms.get(selected_class, {})
    ambiguous = sorted(
        attribute
        for attribute in inherited
        if attribute in _V1_DEFAULT_SENSITIVE_ATTRIBUTES and attribute not in geom.attrib
    )
    if ambiguous:
        joined = ", ".join(ambiguous)
        raise VisualBindingError(
            f"{context} depends on default-inherited visual fields ({joined}); raw-MJCF v1 requires them explicit"
        )


def _collect_body_visuals(
    root: ET.Element,
    mesh_assets: Mapping[str, Mapping[str, Any]],
    materials: Mapping[str, Mapping[str, Any]],
    default_geoms: Mapping[str | None, Mapping[str, str]],
    binding: str,
    asset_root: Path,
) -> list[dict[str, Any]]:
    parents = {child: parent for parent in root.iter() for child in parent}
    seen_bodies: set[str] = set()
    visuals: list[dict[str, Any]] = []
    body_geoms: list[tuple[ET.Element, list[tuple[ET.Element, str]]]] = []

    for body in root.iter("body"):
        supported: list[tuple[ET.Element, str]] = []
        for geom in (child for child in body if child.tag == "geom"):
            geom_type = geom.attrib.get("type")
            has_mesh = geom.attrib.get("mesh") is not None
            if has_mesh:
                if geom_type not in (None, "mesh"):
                    raise VisualBindingError(
                        f"body {body.attrib.get('name', '<unnamed>')!r} geom has mesh with "
                        f"incompatible type {geom_type!r}"
                    )
                supported.append((geom, "mesh"))
                continue
            if geom_type is None:
                raise VisualBindingError(
                    f"body {body.attrib.get('name', '<unnamed>')!r} geom.type must be explicit for raw-MJCF visual v1"
                )
            if geom_type == "mesh":
                supported.append((geom, "mesh"))
            elif geom_type in _PRIMITIVE_TYPES:
                supported.append((geom, "primitive"))
            else:
                raise VisualBindingError(
                    f"body {body.attrib.get('name', '<unnamed>')!r} has unsupported visual geom type {geom_type!r}"
                )
        body_geoms.append((body, supported))

    model_has_mesh_visuals = any(kind == "mesh" for _body, geoms in body_geoms for _geom, kind in geoms)

    for body, supported_geoms in body_geoms:
        name = body.attrib.get("name")
        if not name:
            if supported_geoms:
                raise VisualBindingError("supported visual geom must belong to a named body")
            continue
        body_name = _string(name, "body.name")
        if body_name in seen_bodies:
            raise VisualBindingError(f"duplicate named body {body_name!r}")
        seen_bodies.add(body_name)

        per_body_tokens: set[str] = set()
        kind_counts: dict[str, int] = {}
        for geom, kind in supported_geoms:
            geom_name = geom.attrib.get("name")
            geom_label = geom_name or geom.attrib.get("mesh") or geom.attrib.get("type") or "geom"
            context = f"body {body_name!r} geom[{geom_label}]"
            if "fromto" in geom.attrib:
                raise VisualBindingError(f"{context} uses unsupported fromto geometry")
            for attribute in _UNSUPPORTED_ORIENTATION_ATTRIBUTES:
                if attribute in geom.attrib:
                    raise VisualBindingError(f"{context} uses unsupported orientation attribute {attribute!r}")

            group = _explicit_int(geom.attrib.get("group"), f"{context}.group")
            alpha = _effective_alpha(geom, materials, context)
            if (alpha is not None and alpha <= 0.0) or (group is not None and group >= 3):
                continue

            if kind == "primitive" and model_has_mesh_visuals:
                contype = _explicit_int(geom.attrib.get("contype"), f"{context}.contype")
                conaffinity = _explicit_int(geom.attrib.get("conaffinity"), f"{context}.conaffinity")
                if contype != 0 or conaffinity != 0:
                    continue

            _reject_ambiguous_defaults(
                body,
                geom,
                default_geoms,
                parents,
                context,
            )

            mesh_name = None
            asset = None
            primitive = None
            if kind == "mesh":
                mesh_name = _string(geom.attrib.get("mesh"), f"body {body_name!r} geom.mesh")
                asset = mesh_assets.get(mesh_name)
                if asset is None:
                    raise VisualBindingError(f"body {body_name!r} references unknown mesh asset {mesh_name!r}")
            else:
                primitive = _string(geom.attrib.get("type"), f"body {body_name!r} geom.type")
                if primitive not in _PRIMITIVE_TYPES:
                    raise VisualBindingError(f"body {body_name!r} has unsupported visual geom type {primitive!r}")

            if geom_name is not None:
                base_token = _asset_token(_string(geom_name, f"body {body_name!r} geom.name"))
                token = base_token
                if token in per_body_tokens:
                    raise VisualBindingError(f"duplicate stable visual ID {body_name}/visual/{token!s}")
            else:
                base_token_source = mesh_name if kind == "mesh" else primitive
                if base_token_source is None:
                    raise VisualBindingError(f"{context} has no stable visual token source")
                base_token = _asset_token(base_token_source)
                kind_counts[base_token] = kind_counts.get(base_token, 0) + 1
                ordinal = kind_counts[base_token]
                token = base_token if ordinal == 1 else f"{base_token}_{ordinal}"
                while token in per_body_tokens:
                    ordinal += 1
                    kind_counts[base_token] = ordinal
                    token = f"{base_token}_{ordinal}"
            per_body_tokens.add(token)

            body_frame_id = body_name
            visual_frame_id = f"{_asset_token(body_name)}/visual/{token}"
            pos = _finite_vector(geom.attrib.get("pos"), 3, (0.0, 0.0, 0.0), f"{context}.pos")
            quat = _finite_vector(geom.attrib.get("quat"), 4, (1.0, 0.0, 0.0, 0.0), f"{context}.quat")
            visual: dict[str, Any] = {
                "body": body_name,
                "body_frame_id": body_frame_id,
                "visual_id": token,
                "visual_frame_id": visual_frame_id,
                "asset_key": f"{binding}/{visual_frame_id}",
                "geom": geom_name,
                "material": geom.attrib.get("material"),
                "rgba": None
                if geom.attrib.get("rgba") is None
                else _finite_vector(geom.attrib.get("rgba"), 4, (), f"{context}.rgba"),
                "material_rgba": None,
                "pos": pos,
                "quat": quat,
            }
            if visual["material"] is not None:
                material_asset = materials.get(visual["material"])
                if material_asset is None:
                    raise VisualBindingError(f"{context} references unknown material asset {visual['material']!r}")
                material_rgba = material_asset.get("rgba")
                visual["material_rgba"] = None if material_rgba is None else list(material_rgba)
            if kind == "mesh":
                assert mesh_name is not None and asset is not None
                source_mesh = _relative_to_asset_root(
                    Path(asset["source"]),
                    Path(asset.get("source_root", asset_root)),
                    f"mesh asset {mesh_name!r}",
                )
                visual.update(
                    {
                        "source_mesh": source_mesh,
                        "mesh": mesh_name,
                        "scale": list(asset["scale"]),
                        "geometry": {
                            "kind": "mesh",
                            "source_mesh": source_mesh,
                            "mesh": mesh_name,
                            "scale": list(asset["scale"]),
                        },
                    }
                )
            else:
                assert primitive is not None
                size_count = {"box": 3, "cylinder": 2, "sphere": 1, "capsule": 2}[primitive]
                size = _positive_vector(geom.attrib.get("size"), size_count, f"{context}.size")
                visual.update(
                    {
                        "primitive": primitive,
                        "size": size,
                        "geometry": {"kind": "primitive", "primitive": primitive, "size": size},
                    }
                )
            visuals.append(visual)

    return sorted(
        visuals,
        key=lambda item: (
            item["body"],
            item["visual_frame_id"],
            item["geometry"]["kind"],
            item.get("mesh", ""),
            item.get("primitive", ""),
        ),
    )
