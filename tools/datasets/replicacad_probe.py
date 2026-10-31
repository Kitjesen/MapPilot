#!/usr/bin/env python3
"""Convert one official ReplicaCAD scene into a geometry-bound LingTu graph.

ReplicaCAD provides scene-instance transforms, object semantic IDs, GLB assets,
collision assets, and an official navmesh.  This adapter walks the complete
GLB node hierarchy, reads POSITION bounds/vertices without adding a mesh
dependency, and computes near relations from map-frame object centers.  It is
an offline dataset probe; it does not authorize motion.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Any


_JSON_CHUNK = 0x4E4F534A


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _quat_rotate(q: list[float], point: list[float]) -> list[float]:
    x, y, z, w = q
    px, py, pz = point
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return [
        px + w * tx + y * tz - z * ty,
        py + w * ty + z * tx - x * tz,
        pz + w * tz + x * ty - y * tx,
    ]


def _transform_point(
    point: list[float],
    *,
    scale: list[float] | None = None,
    rotation: list[float] | None = None,
    translation: list[float] | None = None,
) -> list[float]:
    value = [point[i] * (scale[i] if scale else 1.0) for i in range(3)]
    if rotation:
        value = _quat_rotate(rotation, value)
    if translation:
        value = [value[i] + translation[i] for i in range(3)]
    return value


def _matrix_multiply(lhs: list[float], rhs: list[float]) -> list[float]:
    """Multiply two row-major 4x4 affine matrices."""
    return [
        sum(lhs[row * 4 + k] * rhs[k * 4 + col] for k in range(4))
        for row in range(4)
        for col in range(4)
    ]


def _node_matrix(node: dict[str, Any]) -> list[float]:
    """Return a glTF node's local TRS/matrix transform.

    glTF stores an explicit matrix in column-major order.  ReplicaCAD stage
    assets use a rotated scene root and rotated child meshes; walking only the
    mesh node (the previous implementation) dropped the root transform and
    changed the stage up axis.
    """
    explicit = node.get("matrix")
    if explicit is not None:
        if len(explicit) != 16:
            raise ValueError("glTF node matrix must contain 16 values")
        return [float(explicit[col * 4 + row]) for row in range(4) for col in range(4)]

    scale = [float(value) for value in node.get("scale", [1.0, 1.0, 1.0])]
    rotation = [float(value) for value in node.get("rotation", [0.0, 0.0, 0.0, 1.0])]
    translation = [float(value) for value in node.get("translation", [0.0, 0.0, 0.0])]
    x, y, z, w = rotation
    return [
        (1.0 - 2.0 * (y * y + z * z)) * scale[0],
        (2.0 * (x * y - z * w)) * scale[1],
        (2.0 * (x * z + y * w)) * scale[2],
        translation[0],
        (2.0 * (x * y + z * w)) * scale[0],
        (1.0 - 2.0 * (x * x + z * z)) * scale[1],
        (2.0 * (y * z - x * w)) * scale[2],
        translation[1],
        (2.0 * (x * z - y * w)) * scale[0],
        (2.0 * (y * z + x * w)) * scale[1],
        (1.0 - 2.0 * (x * x + y * y)) * scale[2],
        translation[2],
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def _matrix_point(matrix: list[float], point: list[float]) -> list[float]:
    return [
        matrix[row * 4] * point[0]
        + matrix[row * 4 + 1] * point[1]
        + matrix[row * 4 + 2] * point[2]
        + matrix[row * 4 + 3]
        for row in range(3)
    ]


def _accessor_indices(
    document: dict[str, Any], binary: bytes, accessor_index: int
) -> list[int]:
    """Read a scalar index accessor used by TRIANGLES primitives."""
    accessor = document["accessors"][int(accessor_index)]
    if accessor.get("type") != "SCALAR":
        raise ValueError("triangle index accessor must be SCALAR")
    formats = {5121: ("<B", 1), 5123: ("<H", 2), 5125: ("<I", 4)}
    fmt, component_size = formats.get(int(accessor.get("componentType", 0)), (None, 0))
    if fmt is None:
        raise ValueError("unsupported triangle index component type")
    view = document["bufferViews"][int(accessor["bufferView"])]
    offset = int(view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
    stride = int(view.get("byteStride", component_size))
    return [
        int(struct.unpack_from(fmt, binary, offset + index * stride)[0])
        for index in range(int(accessor.get("count", 0)))
    ]


def _sample_floor_triangles(
    document: dict[str, Any],
    binary: bytes,
    node_index: int,
    node_matrix: list[float],
    spacing_m: float,
) -> list[list[float]]:
    """Rasterize floor triangles so large sparse floor faces cover the map."""
    node = document["nodes"][node_index]
    if "floor" not in str(node.get("name", "")).lower():
        return []
    mesh = document["meshes"][int(node["mesh"])]
    accessors = document["accessors"]
    views = document["bufferViews"]
    sampled: list[list[float]] = []
    for primitive in mesh.get("primitives", []):
        if int(primitive.get("mode", 4)) != 4 or primitive.get("indices") is None:
            continue
        position_index = primitive.get("attributes", {}).get("POSITION")
        if position_index is None:
            continue
        position = accessors[int(position_index)]
        view = views[int(position["bufferView"])]
        offset = int(view.get("byteOffset", 0)) + int(position.get("byteOffset", 0))
        stride = int(view.get("byteStride", 12))
        local_points = [
            _matrix_point(
                node_matrix,
                list(struct.unpack_from("<fff", binary, offset + index * stride)),
            )
            for index in range(int(position.get("count", 0)))
        ]
        indices = _accessor_indices(document, binary, int(primitive["indices"]))
        for first in range(0, len(indices) - 2, 3):
            a, b, c = (local_points[indices[first + step]] for step in range(3))
            edge = max(math.dist(a, b), math.dist(a, c), math.dist(b, c))
            steps = max(1, int(math.ceil(edge / max(spacing_m, 1e-3))))
            for i in range(steps + 1):
                for j in range(steps + 1 - i):
                    u = i / steps
                    v = j / steps
                    w = 1.0 - u - v
                    sampled.append([u * a[k] + v * b[k] + w * c[k] for k in range(3)])
    return sampled


def _mesh_nodes(document: dict[str, Any]) -> list[tuple[int, list[float]]]:
    """Yield each mesh node with its complete scene-root transform."""
    nodes = document.get("nodes", [])
    children = {
        child
        for node in nodes
        for child in node.get("children", [])
    }
    roots = [index for index in range(len(nodes)) if index not in children]
    visited: set[int] = set()
    result: list[tuple[int, list[float]]] = []

    def visit(index: int, parent: list[float]) -> None:
        if index in visited:
            return
        visited.add(index)
        node = nodes[index]
        world = _matrix_multiply(parent, _node_matrix(node))
        if node.get("mesh") is not None:
            result.append((index, world))
        for child in node.get("children", []):
            visit(int(child), world)

    identity = [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]
    for root in roots:
        visit(root, identity)
    for index in range(len(nodes)):
        if index not in visited:
            visit(index, identity)
    return result


def _to_lingtu_map(point: list[float]) -> list[float]:
    """Convert ReplicaCAD's Y-up coordinates to LingTu's Z-up map frame."""
    return [point[0], point[2], point[1]]


def _read_glb(path: Path) -> tuple[dict[str, Any], bytes]:
    payload = path.read_bytes()
    if payload[:4] != b"glTF":
        raise ValueError(f"not a GLB file: {path}")
    offset = 12
    document: dict[str, Any] | None = None
    binary = b""
    while offset + 8 <= len(payload):
        size, chunk_type = struct.unpack_from("<II", payload, offset)
        chunk = payload[offset + 8 : offset + 8 + size]
        if chunk_type == _JSON_CHUNK:
            document = json.loads(chunk.rstrip(b" \t\r\n\x00"))
        elif chunk_type == 0x004E4942:
            binary = chunk
        offset += 8 + size
    if document is None:
        raise ValueError(f"GLB JSON chunk missing: {path}")
    return document, binary


def _glb_bounds(path: Path) -> tuple[list[float], list[float], int]:
    """Return world-local AABB and estimated vertex count from a GLB asset."""
    document, _ = _read_glb(path)

    points: list[list[float]] = []
    accessors = document.get("accessors", [])
    meshes = document.get("meshes", [])
    for mesh in meshes:
        for primitive in mesh.get("primitives", []):
            accessor_index = primitive.get("attributes", {}).get("POSITION")
            if accessor_index is None:
                continue
            accessor = accessors[int(accessor_index)]
            if accessor.get("type") != "VEC3" or accessor.get("componentType") != 5126:
                continue
            minimum = accessor.get("min")
            maximum = accessor.get("max")
            if minimum and maximum:
                for x in (minimum[0], maximum[0]):
                    for y in (minimum[1], maximum[1]):
                        for z in (minimum[2], maximum[2]):
                            points.append([float(x), float(y), float(z)])

    if not points:
        raise ValueError(f"GLB has no POSITION bounds: {path}")

    transformed: list[list[float]] = []
    for node_index, node_matrix in _mesh_nodes(document):
        node = document["nodes"][node_index]
        mesh_index = node["mesh"]
        mesh = meshes[int(mesh_index)]
        for primitive in mesh.get("primitives", []):
            accessor_index = primitive.get("attributes", {}).get("POSITION")
            if accessor_index is None:
                continue
            accessor = accessors[int(accessor_index)]
            minimum = accessor.get("min")
            maximum = accessor.get("max")
            if not minimum or not maximum:
                continue
            corners = [
                [float(x), float(y), float(z)]
                for x in (minimum[0], maximum[0])
                for y in (minimum[1], maximum[1])
                for z in (minimum[2], maximum[2])
            ]
            for corner in corners:
                transformed.append(_matrix_point(node_matrix, corner))

    points = transformed or points
    lower = [min(point[i] for point in points) for i in range(3)]
    upper = [max(point[i] for point in points) for i in range(3)]
    vertex_count = sum(
        int(accessors[int(primitive.get("attributes", {}).get("POSITION"))].get("count", 0))
        for mesh in meshes
        for primitive in mesh.get("primitives", [])
        if primitive.get("attributes", {}).get("POSITION") is not None
    )
    return lower, upper, vertex_count


def _glb_positions(
    path: Path,
    *,
    stride: int = 1,
    floor_spacing_m: float | None = None,
) -> list[list[float]]:
    """Read transformed POSITION vertices and optionally rasterize floor faces."""
    document, binary = _read_glb(path)
    accessors = document.get("accessors", [])
    views = document.get("bufferViews", [])
    meshes = document.get("meshes", [])
    positions: list[list[float]] = []
    stride = max(1, int(stride))
    for node_index, node_matrix in _mesh_nodes(document):
        node = document["nodes"][node_index]
        mesh_index = node["mesh"]
        mesh = meshes[int(mesh_index)]
        for primitive in mesh.get("primitives", []):
            accessor_index = primitive.get("attributes", {}).get("POSITION")
            if accessor_index is None:
                continue
            accessor = accessors[int(accessor_index)]
            if accessor.get("type") != "VEC3" or accessor.get("componentType") != 5126:
                continue
            view = views[int(accessor["bufferView"])]
            offset = int(view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
            step = int(view.get("byteStride", 12))
            count = int(accessor.get("count", 0))
            for index in range(0, count, stride):
                base = offset + index * step
                point = list(struct.unpack_from("<fff", binary, base))
                positions.append(_matrix_point(node_matrix, point))
        if floor_spacing_m is not None and floor_spacing_m > 0.0:
            positions.extend(
                _sample_floor_triangles(
                    document,
                    binary,
                    node_index,
                    node_matrix,
                    floor_spacing_m,
                )
            )
    return positions


def write_scene_pcd(
    root: Path,
    scene_id: str,
    output: Path,
    *,
    stride: int = 4,
    floor_spacing_m: float | None = None,
) -> dict[str, Any]:
    """Export ReplicaCAD render geometry as an ASCII PCD in LingTu's Z-up frame."""
    scene_path = root / "configs" / "scenes" / f"{scene_id}.scene_instance.json"
    scene = _load_json(scene_path)
    stage_template = str(scene["stage_instance"]["template_name"])
    stage_config_path = root / "configs" / f"{stage_template}.stage_config.json"
    if stage_config_path.is_file():
        stage_config = _load_json(stage_config_path)
        stage_path = (stage_config_path.parent / str(stage_config["render_asset"])).resolve()
    else:
        # The compact local cache keeps the official stage GLB but omits its
        # optional stage config; the scene template still gives its filename.
        stage_path = (root / f"{stage_template}.glb").resolve()
    if not stage_path.is_file():
        raise FileNotFoundError(stage_path)
    points = [
        _to_lingtu_map(point)
        for point in _glb_positions(
            stage_path,
            stride=stride,
            floor_spacing_m=floor_spacing_m,
        )
    ]
    for item in scene.get("object_instances", []):
        template = str(item["template_name"])
        config_path, config = _template_config(root, template)
        asset_path = (config_path.parent / str(config["render_asset"])).resolve()
        object_rotation = [float(value) for value in item.get("rotation", [0, 0, 0, 1])]
        translation = [float(value) for value in item.get("translation", [0, 0, 0])]
        for point in _glb_positions(asset_path, stride=stride):
            transformed = _transform_point(point, rotation=object_rotation, translation=translation)
            points.append(_to_lingtu_map(transformed))
    output.parent.mkdir(parents=True, exist_ok=True)
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA ascii",
        ]
    )
    body = "\n".join(f"{x:.7g} {y:.7g} {z:.7g}" for x, y, z in points)
    output.write_text(header + "\n" + body + "\n", encoding="ascii")
    return {
        "output": output.as_posix(),
        "scene": scene_id,
        "point_count": len(points),
        "source_up_axis": "y",
        "native_frame": "map (z-up)",
        "point_stride": stride,
        "floor_spacing_m": floor_spacing_m,
    }


def _template_config(root: Path, template: str) -> tuple[Path, dict[str, Any]]:
    path = root / "configs" / f"{template}.object_config.json"
    if not path.is_file():
        raise FileNotFoundError(path)
    return path, _load_json(path)


def _label_by_id(root: Path) -> dict[int, str]:
    payload = _load_json(root / "configs" / "ssd" / "replicaCAD_semantic_lexicon.json")
    return {int(item["id"]): str(item["name"]) for item in payload.get("classes", [])}


def convert_scene(root: Path, scene_id: str = "apt_0", near_radius_m: float = 2.0) -> dict[str, Any]:
    scene_path = root / "configs" / "scenes" / f"{scene_id}.scene_instance.json"
    scene = _load_json(scene_path)
    labels = _label_by_id(root)
    nodes: list[dict[str, Any]] = []
    by_label: dict[str, list[dict[str, Any]]] = {}
    missing_assets: list[str] = []

    for index, item in enumerate(scene.get("object_instances", [])):
        template = str(item["template_name"])
        try:
            config_path, config = _template_config(root, template)
        except FileNotFoundError:
            missing_assets.append(f"{template} (config)")
            continue
        asset_rel = str(config["render_asset"])
        asset_path = (config_path.parent / asset_rel).resolve()
        if not asset_path.is_file():
            missing_assets.append(f"{template} (mesh)")
            continue
        lower, upper, vertex_count = _glb_bounds(asset_path)
        object_rotation = [float(value) for value in item.get("rotation", [0, 0, 0, 1])]
        translation = [float(value) for value in item.get("translation", [0, 0, 0])]
        source_corners = [
            _transform_point(
                [x, y, z],
                rotation=object_rotation,
                translation=translation,
            )
            for x in (lower[0], upper[0])
            for y in (lower[1], upper[1])
            for z in (lower[2], upper[2])
        ]
        corners = [_to_lingtu_map(point) for point in source_corners]
        bbox_min = [min(point[i] for point in corners) for i in range(3)]
        bbox_max = [max(point[i] for point in corners) for i in range(3)]
        center = [(bbox_min[i] + bbox_max[i]) / 2.0 for i in range(3)]
        semantic_id = int(config.get("semantic_id", -1))
        label = labels.get(semantic_id, template.rsplit("/", 1)[-1].removeprefix("frl_apartment_"))
        node = {
            "id": f"object:{scene_id}:{index}",
            "kind": "object",
            "label": label,
            "semantic_id": semantic_id,
            "geometry_binding": {
                "status": "bound",
                "asset": asset_path.as_posix(),
                "collision_asset": str(config.get("collision_asset", "")),
                "map_frame": "map",
                "source_frame": "replicacad_map",
                "source_up_axis": "y",
                "axis_transform": "[x, y, z] -> [x, z, y]",
                "bbox_min": bbox_min,
                "bbox_max": bbox_max,
                "center": center,
                "vertex_count": vertex_count,
            },
            "position": {"x": center[0], "y": center[1], "z": center[2]},
            "motion_type": item.get("motion_type", "UNKNOWN"),
        }
        nodes.append(node)
        by_label.setdefault(label, []).append(node)

    relations: list[dict[str, Any]] = []
    for table in by_label.get("table", []):
        for box in by_label.get("box", []):
            distance = math.dist(table["geometry_binding"]["center"], box["geometry_binding"]["center"])
            if distance <= near_radius_m:
                relations.append(
                    {
                        "subject": box["id"],
                        "predicate": "near",
                        "object": table["id"],
                        "distance_m": distance,
                        "evidence": "ReplicaCAD map-frame geometry centers",
                    }
                )

    # ReplicaCAD's apt_0 scene has no room annotations.  Keep the hierarchy
    # honest: expose the known scene/floor and semantic object groups, while
    # explicitly leaving room segmentation unavailable for a later RGB-D/VLM
    # pass (the HOV-SG floor -> room -> object level cannot be invented here).
    floor_id = 0
    groups: list[dict[str, Any]] = []
    hierarchy_edges: list[dict[str, Any]] = []
    for label, members in sorted(by_label.items()):
        group_id = f"label:{label}"
        groups.append(
            {
                "group_id": group_id,
                "name": label,
                "object_ids": [node["id"] for node in members],
                "source": "ReplicaCAD semantic label",
            }
        )
        hierarchy_edges.append(
            {
                "parent_type": "floor",
                "parent_id": floor_id,
                "child_type": "group",
                "child_id": group_id,
            }
        )
        hierarchy_edges.extend(
            {
                "parent_type": "group",
                "parent_id": group_id,
                "child_type": "object",
                "child_id": node["id"],
            }
            for node in members
        )

    navmesh = root / "navmeshes" / f"{scene_id}.navmesh"
    instance_records = [
        {
            "instance_id": node["id"],
            "map_identity": f"replicacad:{scene_id}",
            "geometry_ref": {
                "asset": node["geometry_binding"]["asset"],
                "bbox_min": node["geometry_binding"]["bbox_min"],
                "bbox_max": node["geometry_binding"]["bbox_max"],
            },
            "observation_refs": [f"dataset:{scene_id}:instance:{node['id'].rsplit(':', 1)[-1]}"],
            "semantic_evidence_refs": [
                f"replicacad:semantic_id:{node['semantic_id']}"
            ],
            "confidence": 1.0,
            "lifecycle": "static_dataset",
            "persistence_status": "dataset_artifact_only",
        }
        for node in nodes
    ]
    return {
        "schema_version": "lingtu.semantic_scene_graph.v1",
        "graph_level": "hierarchical",
        "graph_version": "3.0",
        "map_id": f"replicacad:{scene_id}",
        "content_epoch": 1,
        "floors": [
            {
                "floor_id": floor_id,
                "room_ids": [],
                "object_ids": [node["id"] for node in nodes],
                "source": "ReplicaCAD scene instance",
            }
        ],
        "rooms": [],
        "groups": groups,
        "hierarchy_edges": hierarchy_edges,
        "hierarchy_metadata": {
            "room_segmentation": "unavailable",
            "room_source": None,
            "next_evidence": "RGB-D/VLM room observations",
        },
        "source": {
            "dataset": "ReplicaCAD",
            "license": "see LICENSE.txt (CC BY-NC 4.0 text; upstream README says CC BY 4.0)",
            "scene": scene_id,
            "scene_config": scene_path.as_posix(),
            "coordinate_frame": "replicacad_map (Y-up)",
            "native_frame": "map (Z-up)",
            "axis_transform": "[x, y, z] -> [x, z, y]",
        },
        "frame_id": "map",
        "nodes": nodes,
        "instance_records": instance_records,
        "relations": relations,
        "geometry_binding": {
            "status": "bound",
            "frame_id": "map",
            "method": "ReplicaCAD scene/object transforms + hierarchical GLB POSITION bounds + Y-up to Z-up",
            "navmesh": navmesh.as_posix(),
            "navmesh_present": navmesh.is_file(),
            "motion_authorization": False,
        },
        "stats": {
            "node_count": len(nodes),
            "relation_count": len(relations),
            "geometry_bound_nodes": sum(node["geometry_binding"]["status"] == "bound" for node in nodes),
            "scene_instances": len(scene.get("object_instances", [])),
            "missing_assets": len(missing_assets),
        },
        "missing_assets": missing_assets,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_dir", type=Path)
    parser.add_argument("--scene", default="apt_0")
    parser.add_argument("--near-radius-m", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pcd-output", type=Path, help="also export scene render geometry as native-frame PCD")
    parser.add_argument("--pcd-stride", type=int, default=4)
    parser.add_argument(
        "--floor-spacing-m",
        type=float,
        default=None,
        help="sample floor triangles at approximately this spacing before PCD export",
    )
    args = parser.parse_args()
    result = convert_scene(args.dataset_dir, args.scene, args.near_radius_m)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report: dict[str, Any] = {"output": args.output.as_posix(), "scene": args.scene, **result["stats"]}
    if args.pcd_output:
        report["pcd"] = write_scene_pcd(
            args.dataset_dir,
            args.scene,
            args.pcd_output,
            stride=args.pcd_stride,
            floor_spacing_m=args.floor_spacing_m,
        )
    print(json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
