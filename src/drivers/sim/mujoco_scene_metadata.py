"""MuJoCo scene metadata extraction for simulation acceptance gates."""

from __future__ import annotations

import math
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


_GROUND_NAME_TOKENS = ("ground", "floor", "road", "start_disk", "goal_disk")
_OVERHEAD_NAME_TOKENS = ("roof", "beam", "pipe", "light")


def _floats(text: str | None, *, default: tuple[float, ...] = ()) -> list[float]:
    if text is None:
        return list(default)
    out: list[float] = []
    for item in str(text).split():
        try:
            value = float(item)
        except ValueError:
            return list(default)
        if not math.isfinite(value):
            return list(default)
        out.append(value)
    return out


def _identity3() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _matmul3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [
            a[row][0] * b[0][col]
            + a[row][1] * b[1][col]
            + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def _matvec3(a: list[list[float]], v: list[float]) -> list[float]:
    return [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ]


def _rot_x(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rot_y(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rot_z(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _rotation_from_euler(elem: ET.Element) -> list[list[float]]:
    values = _floats(elem.get("euler"), default=())
    if len(values) < 3:
        return _identity3()
    rx = _rot_x(values[0])
    ry = _rot_y(values[1])
    rz = _rot_z(values[2])
    return _matmul3(rz, _matmul3(ry, rx))


def _rotation_from_quat(elem: ET.Element) -> list[list[float]]:
    values = _floats(elem.get("quat"), default=())
    if len(values) < 4:
        return _identity3()
    w, x, y, z = values[:4]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        return _identity3()
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]


def _local_rotation(elem: ET.Element) -> list[list[float]]:
    if elem.get("quat") is not None:
        return _rotation_from_quat(elem)
    return _rotation_from_euler(elem)


def mujoco_parent_map(root: ET.Element) -> dict[ET.Element, ET.Element]:
    """Return child->parent links for MuJoCo XML elements."""

    return {child: parent for parent in root.iter() for child in parent}


def _ancestor_chain(
    elem: ET.Element,
    parent_map: dict[ET.Element, ET.Element],
) -> list[ET.Element]:
    chain: list[ET.Element] = [elem]
    parent = parent_map.get(elem)
    while parent is not None:
        if parent.tag == "worldbody":
            break
        if parent.tag == "body":
            chain.append(parent)
        parent = parent_map.get(parent)
    chain.reverse()
    return chain


def mujoco_geom_name_chain(
    geom: ET.Element,
    parent_map: dict[ET.Element, ET.Element],
) -> tuple[str, ...]:
    """Return geom plus ancestor body names, closest geom first."""

    names: list[str] = []
    name = str(geom.get("name") or "")
    if name:
        names.append(name)
    parent = parent_map.get(geom)
    while parent is not None:
        if parent.tag == "worldbody":
            break
        if parent.tag == "body":
            body_name = str(parent.get("name") or "")
            if body_name:
                names.append(body_name)
        parent = parent_map.get(parent)
    return tuple(names)


def mujoco_geom_world_pose(
    geom: ET.Element,
    parent_map: dict[ET.Element, ET.Element],
) -> tuple[list[float], list[list[float]]]:
    """Return geom world position and rotation from nested MuJoCo transforms."""

    position = [0.0, 0.0, 0.0]
    rotation = _identity3()
    for elem in _ancestor_chain(geom, parent_map):
        local_pos = _floats(elem.get("pos"), default=(0.0, 0.0, 0.0))
        if len(local_pos) < 3:
            local_pos = [0.0, 0.0, 0.0]
        offset = _matvec3(rotation, [float(v) for v in local_pos[:3]])
        position = [
            position[0] + offset[0],
            position[1] + offset[1],
            position[2] + offset[2],
        ]
        rotation = _matmul3(rotation, _local_rotation(elem))
    return position, rotation


def oriented_box_aabb_half_size(
    half_size: list[float],
    rotation: list[list[float]],
) -> list[float]:
    """Return the axis-aligned half size enclosing an oriented box."""

    return [
        sum(abs(rotation[row][col]) * float(half_size[col]) for col in range(3))
        for row in range(3)
    ]


def _is_static_collision_geom(elem: ET.Element, name: str) -> bool:
    if elem.get("type", "box") not in {"box", "cylinder"}:
        return False
    if elem.get("contype") == "0" and elem.get("conaffinity") == "0":
        return False
    lowered = name.lower()
    if any(token in lowered for token in _GROUND_NAME_TOKENS):
        return False
    if any(token in lowered for token in _OVERHEAD_NAME_TOKENS):
        return False
    return True


def extract_robot_height_obstacle_boxes(
    scene_xml: Path,
    *,
    robot_min_z: float = 0.05,
    robot_max_z: float = 1.25,
) -> list[dict[str, Any]]:
    """Return box-like obstacles that overlap the robot body height.

    The native local-planner gate expects simple axis-aligned boxes in metadata.
    This helper keeps scene parsing in ``src`` so script gates do not each invent
    their own world parser. Cylinders are conservatively approximated as square
    boxes in XY.
    """

    scene_xml = Path(scene_xml)
    root = ET.parse(scene_xml).getroot()
    parent_map = mujoco_parent_map(root)
    obstacles: list[dict[str, Any]] = []
    for index, geom in enumerate(root.findall(".//geom")):
        name = str(geom.get("name") or f"geom_{index}")
        if not _is_static_collision_geom(geom, name):
            continue
        pos, rotation = mujoco_geom_world_pose(geom, parent_map)
        size = _floats(geom.get("size"), default=())
        geom_type = geom.get("type", "box")
        if len(pos) < 3 or not size:
            continue
        if geom_type == "box":
            if len(size) < 3:
                continue
            half_size = [abs(float(size[0])), abs(float(size[1])), abs(float(size[2]))]
            half_size = oriented_box_aabb_half_size(half_size, rotation)
        else:
            radius = abs(float(size[0]))
            half_z = abs(float(size[1])) if len(size) > 1 else radius
            half_size = [radius, radius, half_z]
        center_z = float(pos[2])
        bottom = center_z - half_size[2]
        top = center_z + half_size[2]
        if top < float(robot_min_z) or bottom > float(robot_max_z):
            continue
        obstacles.append(
            {
                "name": name,
                "type": geom_type,
                "position": [float(pos[0]), float(pos[1]), center_z],
                "half_size": half_size,
                "floor_id": 0,
            }
        )
    return obstacles


def write_scene_obstacle_metadata(
    *,
    scene_xml: Path,
    output: Path,
    source_map_metadata: Path | None = None,
) -> dict[str, Any]:
    """Write a native-gate compatible metadata file for a MuJoCo scene."""

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    obstacles = extract_robot_height_obstacle_boxes(scene_xml)
    payload: dict[str, Any] = {
        "schema_version": "lingtu.mujoco_scene_obstacle_metadata.v1",
        "scene_xml": str(Path(scene_xml)),
        "source_map_metadata": str(source_map_metadata or ""),
        "obstacles": obstacles,
        "obstacle_count": len(obstacles),
    }
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
