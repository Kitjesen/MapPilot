"""Build offline visual render plans from the RobotLab URDF source library."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Sequence

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_INVENTORY = REPOSITORY_ROOT / "sim/.imports/robot_lab/source-library.json"
DEFAULT_PLAN = (
    REPOSITORY_ROOT / "build/robot-library-import-20261016/robot-library-render-plan.json"
)
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "build/robot-library-previews-20261016"
ROBOTS_PATH = Path("source/robot_lab/data/Robots")
IDENTITY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


class PreviewPlanError(RuntimeError):
    """Raised when a source URDF cannot produce truthful visual geometry."""


def _numbers(value: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if value is None:
        return default
    parsed = tuple(float(item) for item in value.split())
    if len(parsed) != count or not all(math.isfinite(item) for item in parsed):
        raise PreviewPlanError(f"expected {count} finite numbers, found {value!r}")
    return parsed


def _multiply(
    left: tuple[tuple[float, ...], ...], right: tuple[tuple[float, ...], ...]
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][axis] * right[axis][column] for axis in range(4)) for column in range(4))
        for row in range(4)
    )


def _origin(element: ET.Element | None) -> tuple[tuple[float, ...], ...]:
    if element is None:
        return IDENTITY
    x, y, z = _numbers(element.attrib.get("xyz"), 3, (0.0, 0.0, 0.0))
    roll, pitch, yaw = _numbers(element.attrib.get("rpy"), 3, (0.0, 0.0, 0.0))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return (
        (*rotation[0], x),
        (*rotation[1], y),
        (*rotation[2], z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _pose(matrix: tuple[tuple[float, ...], ...]) -> tuple[list[float], list[list[float]]]:
    position = [matrix[row][3] for row in range(3)]
    rotation = [[matrix[row][column] for column in range(3)] for row in range(3)]
    return position, rotation


def _material_role(link_name: str, visual_name: str, mesh_name: str = "") -> str:
    identity = f"{link_name} {visual_name} {mesh_name}".lower()
    if any(token in identity for token in ("camera", "lidar", "sensor")):
        return "sensor"
    if any(token in identity for token in ("wheel", "tire", "foot", "sole", "rubber")):
        return "rubber"
    if any(token in identity for token in ("thigh", "calf", "shank", "forearm")):
        return "graphite"
    return "silver"


def _link_transforms(root: ET.Element) -> dict[str, tuple[tuple[float, ...], ...]]:
    links = {link.attrib["name"] for link in root.findall("link")}
    children: dict[str, list[tuple[str, tuple[tuple[float, ...], ...]]]] = {}
    child_links: set[str] = set()
    for joint in root.findall("joint"):
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if parent_element is None or child_element is None:
            raise PreviewPlanError("joint is missing parent or child")
        parent = parent_element.attrib.get("link")
        child = child_element.attrib.get("link")
        if parent not in links or child not in links:
            raise PreviewPlanError(f"joint references an unknown link: {parent!r} -> {child!r}")
        children.setdefault(parent, []).append((child, _origin(joint.find("origin"))))
        child_links.add(child)

    transforms: dict[str, tuple[tuple[float, ...], ...]] = {}

    def visit(link_name: str, transform: tuple[tuple[float, ...], ...]) -> None:
        if link_name in transforms:
            raise PreviewPlanError(f"link tree contains a cycle or duplicate parent: {link_name}")
        transforms[link_name] = transform
        for child_name, joint_transform in children.get(link_name, []):
            visit(child_name, _multiply(transform, joint_transform))

    for root_link in sorted(links - child_links):
        visit(root_link, IDENTITY)
    if transforms.keys() != links:
        missing = sorted(links - transforms.keys())
        raise PreviewPlanError(f"link tree has no reachable root for: {missing}")
    return transforms


def _reference_map(model: dict[str, Any], urdf_path: str) -> dict[str, dict[str, str]]:
    entry = next((item for item in model["urdf_entries"] if item["path"] == urdf_path), None)
    if entry is None:
        raise PreviewPlanError(f"presentation URDF is not inventoried: {urdf_path}")
    return {item["uri"]: item for item in entry["references"]}


def _geometry_record(
    geometry: ET.Element,
    *,
    name: str,
    link_name: str,
    transform: tuple[tuple[float, ...], ...],
    references: dict[str, dict[str, str]],
    source_root: Path,
) -> dict[str, Any]:
    children = list(geometry)
    if len(children) != 1:
        raise PreviewPlanError(f"{name}: visual geometry must contain exactly one shape")
    shape = children[0]
    position, rotation = _pose(transform)
    record: dict[str, Any] = {
        "name": name,
        "type": shape.tag,
        "position": position,
        "rotation": rotation,
        "size": [0.0, 0.0, 0.0],
        "material": _material_role(link_name, name, shape.attrib.get("filename", "")),
    }
    if shape.tag == "mesh":
        uri = shape.attrib.get("filename")
        reference = references.get(uri or "")
        if uri is None or reference is None or reference.get("status") != "resolved":
            raise PreviewPlanError(f"{name}: visual mesh is not resolved: {uri!r}")
        mesh_path = (source_root / ROBOTS_PATH / reference["path"]).resolve()
        if not mesh_path.is_file():
            raise PreviewPlanError(f"{name}: resolved visual mesh is missing: {mesh_path}")
        record["mesh"] = str(mesh_path)
        record["scale"] = list(_numbers(shape.attrib.get("scale"), 3, (1.0, 1.0, 1.0)))
        record["material"] = _material_role(link_name, name, mesh_path.name)
    elif shape.tag == "box":
        dimensions = _numbers(shape.attrib.get("size"), 3, (0.0, 0.0, 0.0))
        if any(value <= 0.0 for value in dimensions):
            raise PreviewPlanError(f"{name}: box dimensions must be positive")
        record["size"] = [value / 2.0 for value in dimensions]
    elif shape.tag == "cylinder":
        radius = float(shape.attrib.get("radius", "0"))
        length = float(shape.attrib.get("length", "0"))
        if radius <= 0.0 or length <= 0.0:
            raise PreviewPlanError(f"{name}: cylinder dimensions must be positive")
        record["size"] = [radius, length / 2.0, 0.0]
    elif shape.tag == "sphere":
        radius = float(shape.attrib.get("radius", "0"))
        if radius <= 0.0:
            raise PreviewPlanError(f"{name}: sphere radius must be positive")
        record["size"] = [radius, 0.0, 0.0]
    else:
        raise PreviewPlanError(f"{name}: unsupported URDF visual geometry {shape.tag!r}")
    return record


def _robot_plan(
    presentation: dict[str, str],
    *,
    model: dict[str, Any],
    source_root: Path,
    output_dir: Path,
) -> dict[str, Any]:
    urdf_relative = presentation["urdf"]
    urdf = (source_root / urdf_relative).resolve()
    if not urdf.is_file():
        raise PreviewPlanError(f"presentation URDF is missing: {urdf}")
    root = ET.parse(urdf).getroot()
    transforms = _link_transforms(root)
    references = _reference_map(model, urdf_relative)
    geometries: list[dict[str, Any]] = []
    for link in root.findall("link"):
        link_name = link.attrib["name"]
        for visual_index, visual in enumerate(link.findall("visual")):
            geometry = visual.find("geometry")
            if geometry is None:
                raise PreviewPlanError(f"{link_name}: visual is missing geometry")
            visual_name = visual.attrib.get("name") or f"visual_{visual_index:02d}"
            geometries.append(
                _geometry_record(
                    geometry,
                    name=f"{link_name}_{visual_name}",
                    link_name=link_name,
                    transform=_multiply(transforms[link_name], _origin(visual.find("origin"))),
                    references=references,
                    source_root=source_root,
                )
            )
    if not geometries:
        raise PreviewPlanError(f"{presentation['id']}: URDF has no visual geometry")
    filename = f"robot-{presentation['id']}.png"
    return {
        "id": presentation["id"],
        "version": presentation["version"],
        "label": presentation["label"],
        "filename": filename,
        "output": str((output_dir / filename).resolve()),
        "width": 1400,
        "height": 1200,
        "geometries": geometries,
        "source_urdf": urdf_relative,
        "pose": "urdf_declared_zero",
        "kind": "blender_geometry_render",
        "note": "Visual-only URDF geometry at declared zero joint pose; no collision or runtime claim.",
    }


def build_render_plan(
    inventory_path: Path,
    plan_path: Path,
    output_dir: Path,
    *,
    source_root: Path | None = None,
) -> dict[str, Any]:
    inventory_path = inventory_path.resolve()
    inventory = json.loads(inventory_path.read_text(encoding="utf-8"))
    if inventory.get("schema") != "lingtu.sim.robot-source-library.v1":
        raise PreviewPlanError(f"unsupported source inventory: {inventory_path}")
    source_root = (source_root or inventory_path.parent).resolve()
    models = {model["source_directory"]: model for model in inventory["models"]}
    robots = [
        _robot_plan(
            presentation,
            model=models[presentation["source_path"]],
            source_root=source_root,
            output_dir=output_dir.resolve(),
        )
        for presentation in inventory["presentation_entries"]
    ]
    document = {"schema": "lingtu.sim.robot-selection-render-plan.v1", "robots": robots}
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return document


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--source-root", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    document = build_render_plan(
        args.inventory,
        args.plan,
        args.output_dir,
        source_root=args.source_root,
    )
    print(json.dumps({"robots": len(document["robots"]), "plan": str(args.plan.resolve())}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
