"""Reusable Blender presentation helpers for the Forest_HF world.

The pure specification functions in this module intentionally import without
``bpy``.  Blender is passed into the authoring functions by the caller so the
contracts remain testable with ordinary Python.  Everything created here is a
project-owned procedural visual and never a physics or navigation authority.
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any

VISUALS_VERSION = 2
CLASSIFICATION = "VisualOnly"
COLLISION_PROFILE = "NoCollision"
PROJECT_LICENSE = "LicenseRef-LingTu-Project-Owned"
PROJECT_SOURCE = "repo://sim/tools/worlds/forest_hf/blender_visuals.py"

MetadataCallback = Callable[[Any, str, str], None]

_GRASS_BLADE_HEIGHTS_M = (0.15, 0.24, 0.18, 0.31, 0.21, 0.27, 0.12, 0.29, 0.19)
_FERN_FROND_HEIGHTS_M = (0.29, 0.24, 0.32, 0.27, 0.30, 0.22, 0.31, 0.26)
_FERN_STATION_RATIOS = (0.0, 0.24, 0.50, 0.76, 1.0)
_FERN_TIP_HEIGHT_SCALE = 0.88


def _fern_station_height(height_m: float, station_index: int) -> float:
    ratio = _FERN_STATION_RATIOS[station_index]
    height = height_m * math.sin(ratio * math.pi * 0.58)
    if station_index == len(_FERN_STATION_RATIOS) - 1:
        height *= _FERN_TIP_HEIGHT_SCALE
    return height


_GRASS_REFERENCE_HEIGHT_M = max(_GRASS_BLADE_HEIGHTS_M)
_FERN_REFERENCE_HEIGHT_M = max(
    _fern_station_height(height, station)
    for height in _FERN_FROND_HEIGHTS_M
    for station in range(len(_FERN_STATION_RATIOS))
)


class _ExplicitReviewCameraSpecs(dict[str, dict[str, Any]]):
    """Keep opt-in hero framing out of legacy/default camera iteration."""

    def __iter__(self):  # type: ignore[override]
        return (key for key in super().__iter__() if key != "hero_patrol")


def procedural_material_specs() -> dict[str, dict[str, Any]]:
    """Return stable, engine-neutral contracts for project-owned materials."""

    def spec(
        palette: Sequence[str],
        roughness: float,
        pattern: str,
        **surface: float,
    ) -> dict[str, Any]:
        return {
            "source": PROJECT_SOURCE,
            "license": PROJECT_LICENSE,
            "generator": "lingtu_png_pbr",
            "external_assets": [],
            "palette": list(palette),
            "roughness": roughness,
            "pattern": pattern,
            **surface,
        }

    return {
        "pine_bark": spec(("#2B2018", "#55402D"), 0.88, "vertical_bark_ridges"),
        "pine_needles": spec(("#142B20", "#31533A"), 0.76, "needle_canopy_noise"),
        "birch_bark": spec(("#D8D1C1", "#3A332C"), 0.68, "horizontal_birch_scars"),
        "birch_leaves": spec(("#355A2B", "#77904A"), 0.72, "leaf_canopy_noise"),
        "fern": spec(("#24472B", "#71844A"), 0.79, "fern_leaf_gradient"),
        "grass": spec(("#304426", "#788052"), 0.82, "wet_grass_gradient"),
        "muddy_trail": spec(
            ("#704625", "#D09658"),
            0.86,
            "wet_mud_ruts_and_puddles",
            normal_strength=0.05,
        ),
        "marker_amber": spec(
            ("#8A4A0A", "#F29A18"),
            0.42,
            "inspection_marker_paint",
            emission_strength=2.0,
        ),
        "forest_ground": spec(
            ("#3B402E", "#66644A"),
            0.88,
            "moss_leaf_litter",
            normal_strength=0.08,
        ),
        "rock": spec(("#343A34", "#667064"), 0.82, "weathered_rock"),
    }


def procedural_asset_specs() -> dict[str, dict[str, Any]]:
    """Describe the reusable low-poly asset slots created by this module."""

    common = {
        "classification": CLASSIFICATION,
        "collision_profile": COLLISION_PROFILE,
        "simulate_physics": False,
        "can_ever_affect_navigation": False,
        "source": PROJECT_SOURCE,
        "license": PROJECT_LICENSE,
        "units": "metres",
        "up_axis": "+Z",
    }
    return {
        "pine": {
            **common,
            "semantic_class": "tree.pine",
            "object_name": "LT_Forest_Asset_Pine",
            "reference_height_m": 8.0,
        },
        "birch": {
            **common,
            "semantic_class": "tree.birch",
            "object_name": "LT_Forest_Asset_Birch",
            "reference_height_m": 7.0,
        },
        "fern": {
            **common,
            "semantic_class": "understory.fern",
            "object_name": "LT_Forest_Asset_Fern",
            "reference_height_m": _FERN_REFERENCE_HEIGHT_M,
        },
        "grass": {
            **common,
            "semantic_class": "understory.grass",
            "object_name": "LT_Forest_Asset_Grass",
            "reference_height_m": _GRASS_REFERENCE_HEIGHT_M,
        },
        "inspection_marker": {
            **common,
            "semantic_class": "inspection.marker",
            "object_name": "LT_Forest_Asset_InspectionMarker",
            "reference_height_m": 1.35,
            "geometry": {
                "pole_height_m": 1.35,
                "plate_width_m": 0.70,
                "plate_height_m": 0.32,
            },
        },
    }


def morning_fog_spec() -> dict[str, Any]:
    """Return the fixed art-direction contract for wet, foggy dawn lighting."""

    return {
        "world_color": [0.11, 0.105, 0.095, 1.0],
        "world_strength": 0.10,
        "fog_color": [0.52, 0.51, 0.47, 1.0],
        "fog_density": 0.00028,
        "sky": {
            "model": "nishita",
            "sun_disc": True,
            "air_density": 1.0,
            "dust_density": 1.35,
            "ozone_density": 1.0,
        },
        "clouds": {
            "coverage": 0.52,
            "density": 0.38,
            "scale": 1.8,
            "detail": 5.0,
            "roughness": 0.72,
            "color": [0.56, 0.57, 0.56, 1.0],
        },
        "sun": {
            "azimuth_deg": 132.0,
            "altitude_deg": 24.0,
            "energy": 3.2,
            "angle_deg": 2.5,
            "color": [1.0, 0.72, 0.48],
        },
        "fill": {
            "energy": 260.0,
            "color": [0.60, 0.73, 0.82],
            "size_m": 18.0,
        },
    }


def _sample_route_frame(
    points: Sequence[tuple[float, float]],
    distance_m: float,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    """Return route point, forward tangent, and left normal at one distance."""

    if len(points) < 2:
        points = ((-58.0, -48.0), (8.0, -7.0))
    remaining = max(0.0, float(distance_m))
    for start, end in itertools.pairwise(points):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        if remaining <= length:
            amount = remaining / length
            tangent = (dx / length, dy / length)
            return (
                (start[0] + dx * amount, start[1] + dy * amount),
                tangent,
                (-tangent[1], tangent[0]),
            )
        remaining -= length
    start, end = points[-2], points[-1]
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = max(math.hypot(dx, dy), 1e-9)
    tangent = (dx / length, dy / length)
    return end, tangent, (-tangent[1], tangent[0])


def _first_route_corner_distance(
    points: Sequence[tuple[float, float]],
    *,
    minimum_turn_deg: float = 8.0,
) -> float | None:
    """Return accumulated route distance at the first meaningful turn."""

    accumulated = 0.0
    for index in range(1, len(points) - 1):
        previous, corner, following = points[index - 1], points[index], points[index + 1]
        incoming = (corner[0] - previous[0], corner[1] - previous[1])
        outgoing = (following[0] - corner[0], following[1] - corner[1])
        incoming_length = math.hypot(*incoming)
        outgoing_length = math.hypot(*outgoing)
        if incoming_length <= 1e-9:
            continue
        accumulated += incoming_length
        if outgoing_length <= 1e-9:
            continue
        cosine = (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]) / (
            incoming_length * outgoing_length
        )
        turn_deg = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        if turn_deg >= minimum_turn_deg:
            return accumulated
    return None


def review_robot_grounding_offset(
    min_world_z: float,
    trail_z: float,
    clearance: float = 0.01,
) -> float:
    """Return the root Z delta that places the lowest mesh point on the trail."""

    return float(trail_z) + float(clearance) - float(min_world_z)


def hero_patrol_composition(layout: Mapping[str, Any]) -> dict[str, Any]:
    """Describe the review-only Thunder patrol composition along the route."""

    trail = layout.get("trail", {})
    raw_points = trail.get("centerline_m", []) if isinstance(trail, Mapping) else []
    points = [tuple(float(value) for value in point[:2]) for point in raw_points if len(point) >= 2]
    corner_distance = _first_route_corner_distance(points)
    if corner_distance is None:
        camera_distance, robot_distance, target_distance = 58.0, 74.0, 92.0
    else:
        camera_distance = max(0.0, corner_distance - 39.0)
        robot_distance = max(0.0, corner_distance - 14.0)
        target_distance = corner_distance + 21.0
    camera_point, _camera_tangent, camera_left = _sample_route_frame(points, camera_distance)
    target_point, _target_tangent, _target_left = _sample_route_frame(points, target_distance)
    robot_point, robot_tangent, _robot_left = _sample_route_frame(points, robot_distance)
    camera_location = [
        camera_point[0] + camera_left[0] * 4.2,
        camera_point[1] + camera_left[1] * 4.2,
        1.55,
    ]
    return {
        "camera_location_m": camera_location,
        "camera_target_m": [target_point[0], target_point[1], 0.75],
        "robot_location_m": [robot_point[0], robot_point[1], 0.0],
        "robot_yaw_deg": math.degrees(math.atan2(robot_tangent[1], robot_tangent[0])),
        "distance_along_route_m": camera_distance,
        "robot_distance_along_route_m": robot_distance,
        "target_distance_along_route_m": target_distance,
        "corner_distance_along_route_m": corner_distance,
    }


def review_camera_specs(layout: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    """Create deterministic review-camera specifications from a forest layout."""

    trail = layout.get("trail", {})
    raw_points = trail.get("centerline_m", []) if isinstance(trail, Mapping) else []
    points = [tuple(float(value) for value in point[:2]) for point in raw_points if len(point) >= 2]
    if len(points) > 2 and math.dist(points[0], points[-1]) <= 1e-6:
        points = points[:-1]
    start = points[0] if points else (-58.0, -48.0)
    forward = points[1] if len(points) > 1 else (8.0, -7.0)
    end = points[-1] if points else (56.0, 48.0)
    extent = layout.get("extent_m", [160.0, 160.0])
    width = float(extent[0]) if isinstance(extent, Sequence) else 160.0
    height = float(extent[1]) if isinstance(extent, Sequence) else 160.0
    scene_radius = max(width, height)
    clip_end = max(1000.0, scene_radius * 3.5)
    dx, dy = forward[0] - start[0], forward[1] - start[1]
    segment_length = max(math.hypot(dx, dy), 1e-9)
    tx, ty = dx / segment_length, dy / segment_length
    nx, ny = -ty, tx
    if points:
        hero_points = [points[0]]
        hero_length = 0.0
        hero_limit = 420.0 if scene_radius >= 500.0 else float("inf")
        hero_clipped = False
        for point_a, point_b in itertools.pairwise(points):
            segment_length = math.dist(point_a, point_b)
            remaining = hero_limit - hero_length
            if segment_length > remaining:
                t = remaining / max(segment_length, 1e-9)
                hero_points.append(
                    (
                        point_a[0] + (point_b[0] - point_a[0]) * t,
                        point_a[1] + (point_b[1] - point_a[1]) * t,
                    )
                )
                hero_clipped = True
                break
            hero_points.append(point_b)
            hero_length += segment_length
        route_min_x = min(point[0] for point in hero_points)
        route_max_x = max(point[0] for point in hero_points)
        route_min_y = min(point[1] for point in hero_points)
        route_max_y = max(point[1] for point in hero_points)
        aerial_center = ((route_min_x + route_max_x) * 0.5, (route_min_y + route_max_y) * 0.5)
        route_span = max(route_max_x - route_min_x, route_max_y - route_min_y)
        if hero_clipped:
            aerial_scale = min(500.0, max(350.0, route_span * 1.25))
        else:
            aerial_scale = max(100.0, min(scene_radius * 1.92, route_span * 1.18))
    else:
        aerial_center = (0.0, 0.0)
        aerial_scale = scene_radius * 1.8
        hero_points = [end]
    hero_end = hero_points[-1]
    ridge_end = hero_end if bool(trail.get("closed_loop")) else end
    natural_composition = "shoulder_blend_m" in trail
    robot_location = (
        [start[0] + nx * 0.45, start[1] + ny * 0.45, 1.10]
        if natural_composition
        else [start[0], start[1], 1.25]
    )
    aerial_location = (
        [
            aerial_center[0] - aerial_scale * 0.36,
            aerial_center[1] - aerial_scale * 0.27,
            max(65.0, aerial_scale * 0.60),
        ]
        if natural_composition
        else [aerial_center[0], aerial_center[1], max(180.0, aerial_scale * 1.15)]
    )
    hero = hero_patrol_composition(layout)
    return _ExplicitReviewCameraSpecs({
        "establishing": {
            "location_m": [start[0] - tx * 48.0 + nx * 58.0, start[1] - ty * 48.0 + ny * 58.0, 42.0],
            "target_m": [start[0] + tx * 175.0, start[1] + ty * 175.0, 5.0],
            "lens_mm": 38.0,
            "projection": "perspective",
            "clip_start_m": 15.0,
            "clip_end_m": clip_end,
        },
        "robot_eye": {
            "location_m": robot_location,
            "target_m": [start[0] + tx * 56.0, start[1] + ty * 56.0, 1.05],
            "lens_mm": 40.0 if natural_composition else 34.0,
            "projection": "perspective",
            "clip_start_m": 0.25,
            "clip_end_m": clip_end,
        },
        "ridge": {
            "location_m": [ridge_end[0] - 70.0, ridge_end[1] - 55.0, 28.0],
            "target_m": [ridge_end[0], ridge_end[1], 4.0],
            "lens_mm": 46.0,
            "projection": "perspective",
            "clip_start_m": 15.0,
            "clip_end_m": clip_end,
        },
        "aerial": {
            "location_m": aerial_location,
            "target_m": [aerial_center[0], aerial_center[1], 0.0],
            "lens_mm": 50.0,
            "projection": "perspective" if natural_composition else "orthographic",
            "ortho_scale_m": aerial_scale,
            "clip_start_m": 20.0,
            "clip_end_m": clip_end,
        },
        "hero_patrol": {
            "location_m": hero["camera_location_m"],
            "target_m": hero["camera_target_m"],
            "lens_mm": 55.0,
            "projection": "perspective",
            "clip_start_m": 0.25,
            "clip_end_m": clip_end,
        },
    })


def inspection_marker_placements(
    layout: Mapping[str, Any],
    *,
    first_distance_m: float = 25.0,
    spacing_m: float = 25.0,
) -> list[dict[str, Any]]:
    """Sample deterministic marker positions by distance along the route."""

    trail = layout.get("trail", {})
    raw_points = trail.get("centerline_m", []) if isinstance(trail, Mapping) else []
    points = [tuple(float(value) for value in point[:2]) for point in raw_points if len(point) >= 2]
    if len(points) < 2 or first_distance_m < 0.0 or spacing_m <= 0.0:
        return []
    if bool(trail.get("closed_loop")) and math.dist(points[0], points[-1]) > 1e-6:
        points.append(points[0])
    segments = [
        (start, end, math.dist(start, end))
        for start, end in itertools.pairwise(points)
    ]
    total_length = sum(length for _, _, length in segments)
    placements: list[dict[str, Any]] = []
    distance = first_distance_m
    while distance < total_length - 1e-6:
        remaining = distance
        for start, end, length in segments:
            if remaining <= length or math.isclose(remaining, length):
                ratio = 0.0 if length <= 1e-9 else remaining / length
                placements.append(
                    {
                        "distance_along_route_m": distance,
                        "position_m": [
                            start[0] + (end[0] - start[0]) * ratio,
                            start[1] + (end[1] - start[1]) * ratio,
                        ],
                        "tangent_m": [end[0] - start[0], end[1] - start[1]],
                    }
                )
                break
            remaining -= length
        distance += spacing_m
    return placements


def _hex_rgb(value: str) -> tuple[float, float, float, float]:
    value = value.removeprefix("#")
    rgb = tuple(int(value[index : index + 2], 16) / 255.0 for index in (0, 2, 4))
    return (*rgb, 1.0)


def _material(blender: Any, name: str, base: str, roughness: float) -> tuple[Any, Any, Any]:
    material = blender.data.materials.get(name) or blender.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    shader = nodes.new("ShaderNodeBsdfPrincipled")
    shader.inputs["Base Color"].default_value = _hex_rgb(base)
    shader.inputs["Roughness"].default_value = roughness
    material.node_tree.links.new(shader.outputs["BSDF"], output.inputs["Surface"])
    material["source"] = PROJECT_SOURCE
    material["license"] = PROJECT_LICENSE
    material["external_assets"] = False
    return material, nodes, shader


def build_procedural_materials(blender: Any) -> dict[str, Any]:
    """Build all forest materials from project-owned Blender nodes."""

    specs = procedural_material_specs()
    materials: dict[str, Any] = {}
    for material_id, spec in specs.items():
        material, nodes, shader = _material(
            blender,
            f"M_Forest_{material_id}",
            spec["palette"][0],
            float(spec["roughness"]),
        )
        texcoord = nodes.new("ShaderNodeTexCoord")
        mapping = nodes.new("ShaderNodeMapping")
        noise = nodes.new("ShaderNodeTexNoise")
        ramp = nodes.new("ShaderNodeValToRGB")
        bump = nodes.new("ShaderNodeBump")
        noise.inputs["Scale"].default_value = 5.5
        noise.inputs["Detail"].default_value = 4.0
        noise.inputs["Roughness"].default_value = 0.72
        ramp.color_ramp.elements[0].color = _hex_rgb(spec["palette"][0])
        ramp.color_ramp.elements[1].color = _hex_rgb(spec["palette"][1])
        bump.inputs["Strength"].default_value = 0.24
        bump.inputs["Distance"].default_value = 0.11
        if material_id == "birch_bark":
            mapping.inputs["Scale"].default_value = (2.0, 2.0, 14.0)
            noise.inputs["Scale"].default_value = 3.2
            ramp.color_ramp.elements[0].position = 0.35
            ramp.color_ramp.elements[1].position = 0.58
        elif material_id == "muddy_trail":
            mapping.inputs["Scale"].default_value = (1.4, 7.5, 2.0)
            noise.inputs["Scale"].default_value = 2.8
            shader.inputs["Metallic"].default_value = 0.0
            shader.inputs["Roughness"].default_value = 0.38
            bump.inputs["Strength"].default_value = float(spec["normal_strength"])
        elif material_id == "marker_amber":
            emission = shader.inputs.get("Emission Color") or shader.inputs.get("Emission")
            if emission is not None:
                emission.default_value = _hex_rgb(spec["palette"][-1])
            emission_strength = shader.inputs.get("Emission Strength")
            if emission_strength is not None:
                emission_strength.default_value = float(spec["emission_strength"])
        elif material_id in {"fern", "grass"}:
            mapping.inputs["Scale"].default_value = (2.0, 2.0, 5.0)
        material.node_tree.links.new(texcoord.outputs["Generated"], mapping.inputs["Vector"])
        material.node_tree.links.new(mapping.outputs["Vector"], noise.inputs["Vector"])
        material.node_tree.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
        material.node_tree.links.new(ramp.outputs["Color"], shader.inputs["Base Color"])
        material.node_tree.links.new(noise.outputs["Fac"], bump.inputs["Height"])
        material.node_tree.links.new(bump.outputs["Normal"], shader.inputs["Normal"])
        material["pattern"] = spec["pattern"]
        materials[material_id] = material
    return materials


def _move_to_collection(obj: Any, collection: Any) -> None:
    for owner in list(obj.users_collection):
        owner.objects.unlink(obj)
    collection.objects.link(obj)


def _mark_visual(obj: Any, semantic_id: str, category: str, callback: MetadataCallback | None) -> None:
    obj["classification"] = CLASSIFICATION
    obj["collision_profile"] = COLLISION_PROFILE
    obj["simulate_physics"] = False
    obj["can_ever_affect_navigation"] = False
    obj["semantic_id"] = semantic_id
    obj["visual_category"] = category
    if callback is not None:
        callback(obj, semantic_id, category)


def _parent(child: Any, root: Any) -> None:
    child.parent = root
    child.matrix_parent_inverse = root.matrix_world.inverted()


def _empty_root(blender: Any, collection: Any, name: str) -> Any:
    root = blender.data.objects.new(name, None)
    collection.objects.link(root)
    root.empty_display_type = "PLAIN_AXES"
    return root


def create_pine_template(
    blender: Any,
    collection: Any,
    materials: Mapping[str, Any],
    metadata_callback: MetadataCallback | None = None,
) -> Any:
    """Create a grounded, tiered low-poly conifer template."""

    root = _empty_root(blender, collection, procedural_asset_specs()["pine"]["object_name"])
    blender.ops.mesh.primitive_cylinder_add(vertices=10, radius=0.27, depth=5.6, location=(0, 0, 2.8))
    trunk = blender.context.object
    trunk.name = "LT_Forest_Pine_Trunk"
    trunk.data.materials.append(materials["pine_bark"])
    _move_to_collection(trunk, collection)
    _parent(trunk, root)
    for index, (z, radius, depth) in enumerate(((3.1, 1.85, 2.7), (4.45, 1.45, 2.5), (5.75, 1.0, 2.2))):
        blender.ops.mesh.primitive_cone_add(vertices=10, radius1=radius, radius2=0.08, depth=depth, location=(0, 0, z))
        crown = blender.context.object
        crown.name = f"LT_Forest_Pine_Crown_{index:02d}"
        crown.rotation_euler[2] = index * 0.31
        crown.data.materials.append(materials["pine_needles"])
        _move_to_collection(crown, collection)
        _parent(crown, root)
    _mark_visual(root, "forest.asset.pine", "tree", metadata_callback)
    return root


def create_birch_template(
    blender: Any,
    collection: Any,
    materials: Mapping[str, Any],
    metadata_callback: MetadataCallback | None = None,
) -> Any:
    """Create a recognizable white-trunk birch with an airy crown."""

    root = _empty_root(blender, collection, procedural_asset_specs()["birch"]["object_name"])
    blender.ops.mesh.primitive_cylinder_add(vertices=12, radius=0.22, depth=5.8, location=(0, 0, 2.9))
    trunk = blender.context.object
    trunk.name = "LT_Forest_Birch_WhiteTrunk"
    trunk.data.materials.append(materials["birch_bark"])
    _move_to_collection(trunk, collection)
    _parent(trunk, root)
    crown_specs = ((-0.35, 0.0, 5.1, 1.0), (0.35, 0.15, 5.5, 1.15), (0.0, -0.28, 6.1, 0.9))
    for index, (x, y, z, radius) in enumerate(crown_specs):
        blender.ops.mesh.primitive_ico_sphere_add(subdivisions=1, radius=radius, location=(x, y, z))
        crown = blender.context.object
        crown.name = f"LT_Forest_Birch_Crown_{index:02d}"
        crown.scale = (1.0, 0.82, 1.18)
        crown.data.materials.append(materials["birch_leaves"])
        _move_to_collection(crown, collection)
        _parent(crown, root)
    _mark_visual(root, "forest.asset.birch", "tree", metadata_callback)
    return root


def grass_clump_mesh_data() -> dict[str, list[tuple[float, ...]]]:
    """Return a short, irregular clump whose blades taper to single tips."""

    angles_deg = (4.0, 39.0, 83.0, 119.0, 171.0, 214.0, 252.0, 301.0, 337.0)
    radii = (0.015, 0.052, 0.031, 0.061, 0.025, 0.047, 0.057, 0.020, 0.041)
    widths = (0.014, 0.019, 0.016, 0.021, 0.017, 0.020, 0.013, 0.018, 0.015)
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    uvs: list[tuple[float, float]] = []
    for index, (height, angle_deg, radius, half_width) in enumerate(
        zip(_GRASS_BLADE_HEIGHTS_M, angles_deg, radii, widths, strict=True)
    ):
        angle = math.radians(angle_deg)
        direction = (math.cos(angle), math.sin(angle))
        tangent = (-direction[1], direction[0])
        base_center = (direction[0] * radius, direction[1] * radius)
        bend = 0.035 + height * (0.16 + 0.025 * (index % 3))
        curve = (-0.010, 0.006, 0.014)[index % 3]
        middle_center = (
            base_center[0] + direction[0] * bend * 0.48 + tangent[0] * curve,
            base_center[1] + direction[1] * bend * 0.48 + tangent[1] * curve,
        )
        tip = (
            base_center[0] + direction[0] * bend + tangent[0] * curve * 1.7,
            base_center[1] + direction[1] * bend + tangent[1] * curve * 1.7,
            height,
        )
        base = len(vertices)
        vertices.extend(
            (
                (
                    base_center[0] - tangent[0] * half_width,
                    base_center[1] - tangent[1] * half_width,
                    0.0,
                ),
                (
                    base_center[0] + tangent[0] * half_width,
                    base_center[1] + tangent[1] * half_width,
                    0.0,
                ),
                (
                    middle_center[0] + tangent[0] * half_width * 0.58,
                    middle_center[1] + tangent[1] * half_width * 0.58,
                    height * 0.55,
                ),
                tip,
                (
                    middle_center[0] - tangent[0] * half_width * 0.58,
                    middle_center[1] - tangent[1] * half_width * 0.58,
                    height * 0.55,
                ),
            )
        )
        faces.append(tuple(range(base, base + 5)))
        uvs.extend(((0.0, 0.0), (1.0, 0.0), (0.82, 0.55), (0.5, 1.0), (0.18, 0.55)))
    return {"vertices": vertices, "faces": faces, "uvs": uvs}


def fern_clump_mesh_data() -> dict[str, list[tuple[float, ...]]]:
    """Return low, radially spreading fern fronds made from tapered segments."""

    frond_lengths = (0.43, 0.36, 0.48, 0.39, 0.45, 0.34, 0.46, 0.38)
    angles_deg = (7.0, 49.0, 91.0, 138.0, 184.0, 226.0, 278.0, 326.0)
    width_ratios = (0.020, 0.052, 0.045, 0.030, 0.0)
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    uvs: list[tuple[float, float]] = []
    for index, (length, height, angle_deg) in enumerate(
        zip(frond_lengths, _FERN_FROND_HEIGHTS_M, angles_deg, strict=True)
    ):
        angle = math.radians(angle_deg)
        direction = (math.cos(angle), math.sin(angle))
        tangent = (-direction[1], direction[0])
        root_radius = 0.018 + 0.008 * (index % 3)
        root = (direction[0] * root_radius, direction[1] * root_radius)
        station_indices: list[tuple[int, int]] = []
        for station, (ratio, half_width) in enumerate(
            zip(_FERN_STATION_RATIOS, width_ratios, strict=True)
        ):
            sideways = math.sin(ratio * math.pi) * (0.010 if index % 2 else -0.008)
            center = (
                root[0] + direction[0] * length * ratio + tangent[0] * sideways,
                root[1] + direction[1] * length * ratio + tangent[1] * sideways,
            )
            # Ferns rise gently from the crown, then flatten and droop at the tip.
            z = _fern_station_height(height, station)
            left = len(vertices)
            vertices.append(
                (center[0] - tangent[0] * half_width, center[1] - tangent[1] * half_width, z)
            )
            uvs.append((0.0, ratio))
            right = len(vertices)
            vertices.append(
                (center[0] + tangent[0] * half_width, center[1] + tangent[1] * half_width, z)
            )
            uvs.append((1.0, ratio))
            station_indices.append((left, right))
        for (left_a, right_a), (left_b, right_b) in itertools.pairwise(station_indices):
            faces.append((left_a, right_a, right_b, left_b))
    return {"vertices": vertices, "faces": faces, "uvs": uvs}


def _mesh_object_from_data(
    blender: Any,
    name: str,
    mesh_data: Mapping[str, Sequence[tuple[float, ...]]],
) -> Any:
    mesh = blender.data.meshes.new(f"{name}_Mesh")
    mesh.from_pydata(mesh_data["vertices"], [], mesh_data["faces"])
    mesh.update()
    uv_layer = mesh.uv_layers.new(name="UVMap")
    vertex_uvs = mesh_data["uvs"]
    for loop in mesh.loops:
        uv_layer.data[loop.index].uv = vertex_uvs[loop.vertex_index]
    return blender.data.objects.new(name, mesh)


def create_fern_template(
    blender: Any,
    collection: Any,
    materials: Mapping[str, Any],
    metadata_callback: MetadataCallback | None = None,
) -> Any:
    """Create a radial, broad-leaf low-poly fern."""

    fern = _mesh_object_from_data(blender, "LT_Forest_Fern", fern_clump_mesh_data())
    fern.data.materials.append(materials["fern"])
    collection.objects.link(fern)
    _mark_visual(fern, "forest.asset.fern", "understory", metadata_callback)
    return fern


def create_grass_template(
    blender: Any,
    collection: Any,
    materials: Mapping[str, Any],
    metadata_callback: MetadataCallback | None = None,
) -> Any:
    """Create a compact wet-grass clump."""

    grass = _mesh_object_from_data(blender, "LT_Forest_Grass", grass_clump_mesh_data())
    grass.data.materials.append(materials["grass"])
    collection.objects.link(grass)
    _mark_visual(grass, "forest.asset.grass", "understory", metadata_callback)
    return grass


def create_inspection_marker_template(
    blender: Any,
    collection: Any,
    materials: Mapping[str, Any],
    metadata_callback: MetadataCallback | None = None,
) -> Any:
    """Create an amber trail marker readable by robot-height cameras."""

    root = _empty_root(blender, collection, "LT_Forest_InspectionMarker_Root")
    geometry = procedural_asset_specs()["inspection_marker"]["geometry"]
    pole_height = float(geometry["pole_height_m"])
    blender.ops.mesh.primitive_cylinder_add(
        vertices=10,
        radius=0.045,
        depth=pole_height,
        location=(0, 0, pole_height * 0.5),
    )
    pole = blender.context.object
    pole.name = "LT_Forest_InspectionMarker_Pole"
    pole.data.materials.append(materials["marker_amber"])
    _move_to_collection(pole, collection)
    _parent(pole, root)
    plate_width = float(geometry["plate_width_m"])
    plate_height = float(geometry["plate_height_m"])
    blender.ops.mesh.primitive_cube_add(
        size=1.0,
        location=(0, 0, pole_height),
        scale=(plate_width * 0.5, 0.055, plate_height * 0.5),
    )
    plate = blender.context.object
    plate.name = "LT_Forest_InspectionMarker_Plate"
    plate.data.materials.append(materials["marker_amber"])
    _move_to_collection(plate, collection)
    _parent(plate, root)
    _mark_visual(root, "forest.asset.inspection_marker", "marker", metadata_callback)
    return root


def build_visual_templates(
    blender: Any,
    collection: Any,
    materials: Mapping[str, Any],
    metadata_callback: MetadataCallback | None = None,
) -> dict[str, Any]:
    """Create the complete reusable visual template set."""

    return {
        "pine": create_pine_template(blender, collection, materials, metadata_callback),
        "birch": create_birch_template(blender, collection, materials, metadata_callback),
        "fern": create_fern_template(blender, collection, materials, metadata_callback),
        "grass": create_grass_template(blender, collection, materials, metadata_callback),
        "inspection_marker": create_inspection_marker_template(blender, collection, materials, metadata_callback),
    }


def _configure_physical_sky_node(sky: Any, spec: Mapping[str, Any]) -> None:
    """Map the Nishita art direction onto the active Blender sky API."""

    last_error: Exception | None = None
    for sky_type in ("NISHITA", "MULTIPLE_SCATTERING"):
        try:
            sky.sky_type = sky_type
        except (TypeError, ValueError) as exc:
            last_error = exc
        else:
            break
    else:
        raise RuntimeError("Blender exposes no supported physical sky model") from last_error

    sky.sun_disc = spec["sky"]["sun_disc"]
    sky.sun_elevation = math.radians(spec["sun"]["altitude_deg"])
    sky.sun_rotation = math.radians(spec["sun"]["azimuth_deg"])
    sky.air_density = spec["sky"]["air_density"]
    particle_density = spec["sky"]["dust_density"]
    if hasattr(sky, "aerosol_density"):
        sky.aerosol_density = particle_density
    elif hasattr(sky, "dust_density"):
        sky.dust_density = particle_density
    else:
        raise RuntimeError(
            "Blender physical sky exposes neither aerosol_density nor dust_density"
        )
    sky.ozone_density = spec["sky"]["ozone_density"]


def setup_morning_fog(blender: Any, collection: Any) -> dict[str, Any]:
    """Install fog-volume, sun, and cool fill light from :func:`morning_fog_spec`."""

    from mathutils import Vector  # type: ignore[import-not-found]

    spec = morning_fog_spec()
    world = blender.context.scene.world or blender.data.worlds.new("Forest_Morning_World")
    blender.context.scene.world = world
    world.use_nodes = True
    background = world.node_tree.nodes.get("Background")
    background.inputs["Color"].default_value = spec["world_color"]
    background.inputs["Strength"].default_value = spec["world_strength"]

    volume = blender.data.materials.get("M_Forest_MorningFog") or blender.data.materials.new("M_Forest_MorningFog")
    volume.use_nodes = True
    nodes = volume.node_tree.nodes
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    principled = nodes.new("ShaderNodeVolumePrincipled")
    principled.inputs["Color"].default_value = spec["fog_color"]
    principled.inputs["Density"].default_value = spec["fog_density"]
    volume.node_tree.links.new(principled.outputs["Volume"], output.inputs["Volume"])
    blender.ops.mesh.primitive_cube_add(size=1.0, scale=(1.0, 1.0, 1.0), location=(0, 0, 10.0))
    fog = blender.context.object
    fog.name = "LT_Forest_MorningFog"
    fog.data.materials.append(volume)
    _move_to_collection(fog, collection)
    _mark_visual(fog, "forest.atmosphere.morning_fog", "atmosphere", None)

    world_nodes = world.node_tree.nodes
    world_links = world.node_tree.links
    sky = world_nodes.new("ShaderNodeTexSky")
    sky.name = "LT_Forest_ProceduralSky"
    _configure_physical_sky_node(sky, spec)

    coordinates = world_nodes.new("ShaderNodeTexCoord")
    coordinates.name = "LT_Forest_CloudCoordinates"
    cloud_noise = world_nodes.new("ShaderNodeTexNoise")
    cloud_noise.name = "LT_Forest_ProceduralClouds"
    cloud_noise.noise_dimensions = "3D"
    cloud_noise.inputs["Scale"].default_value = spec["clouds"]["scale"]
    cloud_noise.inputs["Detail"].default_value = spec["clouds"]["detail"]
    cloud_noise.inputs["Roughness"].default_value = spec["clouds"]["roughness"]
    cloud_ramp = world_nodes.new("ShaderNodeValToRGB")
    cloud_ramp.name = "LT_Forest_CloudCoverage"
    cloud_ramp.color_ramp.elements[0].position = spec["clouds"]["coverage"] - 0.08
    cloud_ramp.color_ramp.elements[1].position = spec["clouds"]["coverage"] + 0.08
    cloud_ramp.color_ramp.elements[1].color = (
        *spec["clouds"]["color"][:3],
        spec["clouds"]["density"],
    )
    cloud_mix = world_nodes.new("ShaderNodeMixRGB")
    cloud_mix.name = "LT_Forest_SkyCloudMix"
    cloud_mix.blend_type = "MIX"
    cloud_mix.inputs[2].default_value = spec["clouds"]["color"]
    world_links.new(coordinates.outputs["Normal"], cloud_noise.inputs["Vector"])
    world_links.new(cloud_noise.outputs["Fac"], cloud_ramp.inputs["Fac"])
    world_links.new(cloud_ramp.outputs["Color"], cloud_mix.inputs["Fac"])
    world_links.new(sky.outputs["Color"], cloud_mix.inputs[1])
    world_links.new(cloud_mix.outputs["Color"], background.inputs["Color"])

    sun_data = blender.data.lights.new("LT_Forest_DawnSun", "SUN")
    sun_data.energy = spec["sun"]["energy"]
    sun_data.angle = math.radians(spec["sun"]["angle_deg"])
    sun_data.color = spec["sun"]["color"]
    sun = blender.data.objects.new("LT_Forest_DawnSun", sun_data)
    collection.objects.link(sun)
    azimuth = math.radians(spec["sun"]["azimuth_deg"])
    altitude = math.radians(spec["sun"]["altitude_deg"])
    direction = Vector(
        (
            math.cos(altitude) * math.cos(azimuth),
            math.cos(altitude) * math.sin(azimuth),
            math.sin(altitude),
        )
    )
    sun.rotation_euler = (-direction).to_track_quat("-Z", "Y").to_euler()

    fill_data = blender.data.lights.new("LT_Forest_CoolFill", "AREA")
    fill_data.energy = spec["fill"]["energy"]
    fill_data.color = spec["fill"]["color"]
    fill_data.shape = "DISK"
    fill_data.size = spec["fill"]["size_m"]
    fill = blender.data.objects.new("LT_Forest_CoolFill", fill_data)
    fill.location = (-28.0, -35.0, 24.0)
    fill.rotation_euler = (math.radians(28.0), 0.0, math.radians(-38.0))
    collection.objects.link(fill)
    return {"fog": fog, "sun": sun, "fill": fill}


def create_review_cameras(blender: Any, collection: Any, layout: Mapping[str, Any]) -> dict[str, Any]:
    """Create review cameras while keeping camera logic outside the main author."""

    from mathutils import Vector  # type: ignore[import-not-found]

    cameras: dict[str, Any] = {}
    for camera_id, spec in review_camera_specs(layout).items():
        data = blender.data.cameras.new(f"Camera_Forest_{camera_id}")
        data.lens = spec["lens_mm"]
        data.clip_start = spec["clip_start_m"]
        data.clip_end = spec["clip_end_m"]
        if spec["projection"] == "orthographic":
            data.type = "ORTHO"
            data.ortho_scale = spec["ortho_scale_m"]
        camera = blender.data.objects.new(f"Camera_Forest_{camera_id}", data)
        camera.location = spec["location_m"]
        direction = Vector(spec["target_m"]) - camera.location
        camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
        collection.objects.link(camera)
        cameras[camera_id] = camera
    return cameras


__all__ = [
    "CLASSIFICATION",
    "COLLISION_PROFILE",
    "PROJECT_LICENSE",
    "PROJECT_SOURCE",
    "VISUALS_VERSION",
    "build_procedural_materials",
    "build_visual_templates",
    "create_birch_template",
    "create_fern_template",
    "create_grass_template",
    "create_inspection_marker_template",
    "create_pine_template",
    "create_review_cameras",
    "fern_clump_mesh_data",
    "grass_clump_mesh_data",
    "hero_patrol_composition",
    "inspection_marker_placements",
    "morning_fog_spec",
    "procedural_asset_specs",
    "procedural_material_specs",
    "review_camera_specs",
    "setup_morning_fog",
]
