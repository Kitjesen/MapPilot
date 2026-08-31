"""Presentation dressing for the formal MuJoCo CMU/SCAN comparison scene.

The source comparison world remains the authority for collision and sensor
geometry.  This module adds only lights, materials, cameras, and group-5
display geometry.  LingTu's product MID-360 mask observes groups 0 and 1, so
the floor markings do not enter the comparison point cloud or saved map.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

COMPARISON_MODEL = "lingtu_cmu_scan_compare"
COMPARISON_OBSTACLE = "dynamic_compare_obstacle"
DISPLAY_GEOM_GROUP = "5"

_MATERIALS = {
    "comparison_floor": {
        "texture": "comparison_floor_grid",
        "texrepeat": "5 3",
        "texuniform": "true",
        "rgba": "0.38 0.39 0.38 1",
        "specular": "0.08",
        "shininess": "0.12",
        "reflectance": "0.04",
    },
    "comparison_boundary": {
        "rgba": "0.17 0.20 0.21 1",
        "specular": "0.16",
        "shininess": "0.24",
    },
    "comparison_hazard": {
        "rgba": "0.58 0.09 0.07 1",
        "specular": "0.12",
        "shininess": "0.18",
    },
    "comparison_start": {"rgba": "0.12 0.22 0.23 1"},
    "comparison_route": {"rgba": "0.38 0.36 0.29 1"},
    "comparison_warning": {"rgba": "0.34 0.19 0.07 1"},
    "comparison_goal": {"rgba": "0.07 0.23 0.17 1"},
    "comparison_inlay": {"rgba": "0.08 0.10 0.11 1"},
    "comparison_backdrop": {"rgba": "0.12 0.13 0.13 1"},
}


def is_comparison_scene(path: str | Path) -> bool:
    """Return whether *path* is the formal CMU/SCAN comparison world."""

    try:
        root = ET.parse(Path(path)).getroot()  # noqa: S314 - trusted repo-local MJCF
    except (ET.ParseError, OSError):
        return False
    if root.attrib.get("model") == COMPARISON_MODEL:
        return True
    worldbody = root.find("worldbody")
    return worldbody is not None and any(
        geom.attrib.get("name") == COMPARISON_OBSTACLE
        for geom in worldbody.findall("geom")
    )


def _upsert(parent: ET.Element, tag: str, name: str | None = None) -> ET.Element:
    for child in parent.findall(tag):
        if name is None or child.attrib.get("name") == name:
            return child
    attrib = {"name": name} if name is not None else {}
    return ET.SubElement(parent, tag, attrib)


def _set_material(geom: ET.Element, material: str) -> None:
    geom.set("material", material)
    geom.attrib.pop("rgba", None)


def _display_geom(
    worldbody: ET.Element,
    *,
    name: str,
    geom_type: str = "box",
    pos: str,
    size: str,
    material: str,
    euler: str | None = None,
) -> ET.Element:
    attrib = {
        "name": name,
        "type": geom_type,
        "pos": pos,
        "size": size,
        "material": material,
        "group": DISPLAY_GEOM_GROUP,
        "contype": "0",
        "conaffinity": "0",
    }
    if euler is not None:
        attrib["euler"] = euler
    return ET.SubElement(worldbody, "geom", attrib)


def _display_segment(
    worldbody: ET.Element,
    *,
    name: str,
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    material: str,
) -> ET.Element:
    dx = float(end[0] - start[0])
    dy = float(end[1] - start[1])
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise ValueError(f"display segment {name!r} has zero length")
    center_x = (start[0] + end[0]) * 0.5
    center_y = (start[1] + end[1]) * 0.5
    yaw_deg = math.degrees(math.atan2(dy, dx))
    return _display_geom(
        worldbody,
        name=name,
        pos=f"{center_x:.3f} {center_y:.3f} 0.005",
        size=f"{length * 0.5:.3f} {width * 0.5:.3f} 0.005",
        material=material,
        euler=f"0 0 {yaw_deg:.3f}",
    )


def _add_visual_rig(root: ET.Element, worldbody: ET.Element) -> None:
    statistic = _upsert(root, "statistic")
    statistic.set("center", "1.5 0 0.25")
    statistic.set("extent", "4.8")

    visual = _upsert(root, "visual")
    global_visual = _upsert(visual, "global")
    global_visual.attrib.update(
        {"azimuth": "135", "elevation": "-48", "offwidth": "1920", "offheight": "1080"}
    )
    quality = _upsert(visual, "quality")
    quality.attrib.update({"shadowsize": "4096", "offsamples": "4"})
    map_visual = _upsert(visual, "map")
    map_visual.attrib.update(
        {"znear": "0.02", "zfar": "32", "fogstart": "8", "fogend": "18", "shadowscale": "0.7"}
    )
    headlight = _upsert(visual, "headlight")
    headlight.attrib.update(
        {
            "ambient": "0.20 0.22 0.22",
            "diffuse": "0.62 0.62 0.59",
            "specular": "0.08 0.08 0.07",
            "active": "1",
        }
    )
    rgba = _upsert(visual, "rgba")
    rgba.attrib.update({"haze": "0.18 0.20 0.20 1"})

    asset = _upsert(root, "asset")
    sky = _upsert(asset, "texture", "comparison_sky")
    sky.attrib.update(
        {
            "type": "skybox",
            "builtin": "gradient",
            "rgb1": "0.08 0.10 0.11",
            "rgb2": "0.31 0.33 0.32",
            "width": "512",
            "height": "3072",
        }
    )
    floor_texture = _upsert(asset, "texture", "comparison_floor_grid")
    floor_texture.attrib.update(
        {
            "type": "2d",
            "builtin": "checker",
            "rgb1": "0.31 0.32 0.31",
            "rgb2": "0.34 0.35 0.34",
            "mark": "edge",
            "markrgb": "0.26 0.27 0.27",
            "width": "512",
            "height": "512",
        }
    )
    for name, attrib in _MATERIALS.items():
        material = _upsert(asset, "material", name)
        material.attrib.update(attrib)

    for light in list(worldbody.findall("light")):
        if str(light.attrib.get("name") or "").startswith("comparison_"):
            worldbody.remove(light)
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "comparison_key",
            "pos": "-1.2 -2.8 5.8",
            "dir": "0.34 0.42 -1",
            "directional": "true",
            "castshadow": "true",
            "diffuse": "0.92 0.89 0.82",
            "specular": "0.12 0.12 0.10",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "comparison_fill",
            "pos": "4.2 2.8 3.2",
            "dir": "-0.46 -0.34 -1",
            "directional": "true",
            "castshadow": "false",
            "diffuse": "0.36 0.42 0.44",
            "specular": "0.04 0.05 0.05",
        },
    )

    camera = _upsert(worldbody, "camera", "comparison_overview")
    camera.attrib.update(
        {
            "mode": "fixed",
            "pos": "1.5 -5.6 4.2",
            "xyaxes": "1 0 0 0 0.620 0.785",
            "fovy": "42",
        }
    )


def _add_floor_language(worldbody: ET.Element) -> None:
    for geom in list(worldbody.findall("geom")):
        if str(geom.attrib.get("name") or "").startswith("comparison_display_"):
            worldbody.remove(geom)

    # A non-sensed studio deck replaces the robot asset's generic blue void.
    _display_geom(
        worldbody,
        name="comparison_display_backdrop",
        pos="1.5 0 -0.075",
        size="30.0 30.0 0.015",
        material="comparison_backdrop",
    )

    # Start and calibration bay: one robot-length of quiet space with a crosshair.
    _display_geom(
        worldbody,
        name="comparison_display_start_pad",
        pos="0 0 0.004",
        size="0.46 0.62 0.004",
        material="comparison_start",
    )
    _display_segment(
        worldbody,
        name="comparison_display_calibration_axis_x",
        start=(-0.28, 0.0),
        end=(0.28, 0.0),
        width=0.035,
        material="comparison_inlay",
    )
    _display_segment(
        worldbody,
        name="comparison_display_calibration_axis_y",
        start=(0.0, -0.34),
        end=(0.0, 0.34),
        width=0.035,
        material="comparison_inlay",
    )
    _display_segment(
        worldbody,
        name="comparison_display_start_separator",
        start=(0.55, -1.18),
        end=(0.55, 1.18),
        width=0.025,
        material="comparison_route",
    )

    # A deliberately open bypass band around the single programmed red obstacle.
    _display_geom(
        worldbody,
        name="comparison_display_obstacle_apron",
        pos="1.4 0 0.003",
        size="0.43 0.47 0.003",
        material="comparison_warning",
    )
    _display_geom(
        worldbody,
        name="comparison_display_obstacle_inlay",
        pos="1.4 0 0.006",
        size="0.31 0.35 0.003",
        material="comparison_inlay",
    )
    for side, y in (("left", 1.05), ("right", -1.05)):
        for index, x in enumerate((0.82, 1.12, 1.42, 1.72, 2.02)):
            _display_segment(
                worldbody,
                name=f"comparison_display_bypass_{side}_{index}",
                start=(x - 0.09, y),
                end=(x + 0.09, y),
                width=0.035,
                material="comparison_route",
            )

    # Channel and boundary cues: visual floor edge, separate from rail collision.
    for side, y in (("left", 1.72), ("right", -1.72)):
        _display_segment(
            worldbody,
            name=f"comparison_display_boundary_{side}",
            start=(0.62, y),
            end=(3.55, y),
            width=0.045,
            material="comparison_boundary",
        )
    _display_segment(
        worldbody,
        name="comparison_display_channel_separator",
        start=(2.14, -1.18),
        end=(2.14, 1.18),
        width=0.025,
        material="comparison_route",
    )
    for index, x in enumerate((2.30, 2.55, 2.80)):
        _display_segment(
            worldbody,
            name=f"comparison_display_goal_approach_{index}",
            start=(x - 0.08, 0.0),
            end=(x + 0.08, 0.0),
            width=0.03,
            material="comparison_route",
        )

    # Goal ring: a restrained teal target with a dark center for legibility.
    _display_geom(
        worldbody,
        name="comparison_display_goal_outer",
        geom_type="cylinder",
        pos="3.0 0 0.004",
        size="0.36 0.004",
        material="comparison_goal",
    )
    _display_geom(
        worldbody,
        name="comparison_display_goal_inner",
        geom_type="cylinder",
        pos="3.0 0 0.009",
        size="0.24 0.005",
        material="comparison_inlay",
    )


def build_comparison_scene(base_scene: str | Path, output: str | Path) -> Path:
    """Create the effective formal comparison world without changing physics."""

    base_path = Path(base_scene).expanduser().resolve()
    output_path = Path(output).expanduser().resolve()
    tree = ET.parse(base_path)  # noqa: S314 - trusted repo-local MJCF
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"MuJoCo scene has no worldbody: {base_path}")
    if not is_comparison_scene(base_path):
        raise ValueError(f"not the formal CMU/SCAN comparison scene: {base_path}")

    floor = next(
        (geom for geom in worldbody.findall("geom") if geom.attrib.get("name") == "floor"),
        None,
    )
    if floor is not None:
        _set_material(floor, "comparison_floor")
    for name in ("left_rail", "right_rail"):
        geom = next(
            (item for item in worldbody.findall("geom") if item.attrib.get("name") == name),
            None,
        )
        if geom is not None:
            _set_material(geom, "comparison_boundary")
    obstacle = next(
        (
            geom
            for geom in worldbody.findall("geom")
            if geom.attrib.get("name") == COMPARISON_OBSTACLE
        ),
        None,
    )
    if obstacle is None:
        raise ValueError(f"comparison scene is missing {COMPARISON_OBSTACLE!r}")
    _set_material(obstacle, "comparison_hazard")

    _add_visual_rig(root, worldbody)
    _add_floor_language(worldbody)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path
