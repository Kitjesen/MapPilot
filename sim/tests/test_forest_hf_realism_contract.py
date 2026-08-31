# ruff: noqa: S101

"""Deterministic acceptance contracts for the next Forest_HF realism pass."""

from __future__ import annotations

import math
import sys
from itertools import pairwise
from types import SimpleNamespace
from typing import Any

import pytest

from sim.tools.worlds.forest_hf import blender_author as author
from sim.tools.worlds.forest_hf import blender_visuals as visuals


def _recipe() -> dict[str, Any]:
    return {
        "seed": 5808,
        "extent_m": [80.0, 60.0],
        "tree_count": 96,
        "trail": {
            "centerline_m": [[-36.0, 0.0], [0.0, 4.0], [36.0, 0.0]],
            "clearance_radius_m": 3.0,
            "surface_width_m": 3.2,
            "shoulder_blend_m": 0.8,
        },
        "species": [
            {
                "id": "pine",
                "weight": 0.7,
                "height_m": [5.0, 9.0],
                "crown_radius_m": [1.2, 2.1],
                "material": "pine_bark_needles",
            },
            {
                "id": "birch",
                "weight": 0.3,
                "height_m": [4.0, 7.0],
                "crown_radius_m": [1.0, 1.8],
                "material": "birch_bark_leaves",
            },
        ],
    }


def _long_hero_route_recipe() -> dict[str, Any]:
    recipe = _recipe()
    recipe["extent_m"] = [230.0, 75.0]
    recipe["tree_count"] = 360
    recipe["trail"]["centerline_m"] = [[-210.0, 0.0], [210.0, 0.0]]
    return recipe


def _distance_to_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    squared_length = dx * dx + dy * dy
    if squared_length == 0.0:
        return math.dist(point, start)
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared_length
    amount = min(1.0, max(0.0, projection))
    return math.dist(point, (start[0] + amount * dx, start[1] + amount * dy))


def _distance_to_trail(point: list[float], centerline: list[list[float]]) -> float:
    return min(
        _distance_to_segment(tuple(point[:2]), tuple(start), tuple(end))
        for start, end in pairwise(centerline)
    )


def _distance_along_trail(point: list[float], centerline: list[list[float]]) -> float:
    best_distance = float("inf")
    best_along = 0.0
    accumulated = 0.0
    for start, end in pairwise(centerline):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        projection = (
            0.0
            if segment_length == 0.0
            else min(
                1.0,
                max(
                    0.0,
                    ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                    / segment_length**2,
                ),
            )
        )
        closest = (start[0] + projection * dx, start[1] + projection * dy)
        distance = math.dist(point[:2], closest)
        if distance < best_distance:
            best_distance = distance
            best_along = accumulated + projection * segment_length
        accumulated += segment_length
    return best_along


def _pearson_correlation(left: list[float], right: list[float]) -> float:
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    covariance = sum((x - left_mean) * (y - right_mean) for x, y in zip(left, right))
    denominator = math.sqrt(
        sum((value - left_mean) ** 2 for value in left)
        * sum((value - right_mean) ** 2 for value in right)
    )
    return covariance / denominator


def test_most_tree_canopies_form_a_dense_belt_beside_the_route() -> None:
    recipe = _recipe()

    layout = author.generate_forest_layout(recipe)
    belt_members = [
        tree
        for tree in layout["instances"]
        if _distance_to_trail(tree["position_m"], recipe["trail"]["centerline_m"])
        - tree["crown_radius_m"]
        - recipe["trail"]["clearance_radius_m"]
        <= 18.0
    ]

    assert len(belt_members) / len(layout["instances"]) >= 0.65


def test_route_belt_trees_grow_in_local_clusters() -> None:
    layout = author.generate_forest_layout(_recipe())
    trees = layout["instances"]
    clustered = [
        tree
        for tree in trees
        if sum(
            math.dist(tree["position_m"][:2], other["position_m"][:2]) <= 6.0
            for other in trees
            if other is not tree
        )
        >= 1
    ]

    assert len(clustered) / len(trees) >= 0.75


def test_each_tree_species_has_three_organic_scale_and_yaw_variants() -> None:
    layout = author.generate_forest_layout(_recipe())

    for species_id in {tree["species_id"] for tree in layout["instances"]}:
        trees = [tree for tree in layout["instances"] if tree["species_id"] == species_id]
        assert len({tree["variant_id"] for tree in trees}) >= 3
        assert len({round(tree["scale_xyz"][2], 2) for tree in trees}) >= 3
        assert len({math.floor((tree["yaw_deg"] % 360.0) / 90.0) for tree in trees}) >= 3


def test_hero_trees_fill_six_readable_bands_without_a_linear_scatter() -> None:
    recipe = _recipe()
    layout = author.generate_forest_layout(recipe)
    start_x = recipe["trail"]["centerline_m"][0][0]
    end_x = recipe["trail"]["centerline_m"][-1][0]
    hero_trees = [
        tree
        for tree in layout["instances"]
        if _distance_to_trail(tree["position_m"], recipe["trail"]["centerline_m"])
        - tree["crown_radius_m"]
        - recipe["trail"]["clearance_radius_m"]
        <= 18.0
    ]
    band_counts = {
        (depth_band, lateral_band): sum(
            min(2, int(3.0 * (tree["position_m"][0] - start_x) / (end_x - start_x))) == depth_band
            and (1 if tree["position_m"][1] >= 2.0 else -1) == lateral_band
            for tree in hero_trees
            if start_x <= tree["position_m"][0] <= end_x
        )
        for depth_band in range(3)
        for lateral_band in (-1, 1)
    }
    along_route = [tree["position_m"][0] for tree in hero_trees]
    lateral_offset = [tree["position_m"][1] for tree in hero_trees]

    assert all(count >= 8 for count in band_counts.values())
    assert abs(_pearson_correlation(along_route, lateral_offset)) <= 0.35


def test_layout_preserves_tight_trail_and_soft_shoulder_dimensions() -> None:
    recipe = _recipe()

    layout = author.generate_forest_layout(recipe)

    assert layout["trail"]["surface_width_m"] == pytest.approx(3.2)
    assert layout["trail"]["shoulder_blend_m"] == pytest.approx(0.8)


def _capture_trail_mesh(
    monkeypatch: pytest.MonkeyPatch,
    layout: dict[str, Any],
    *,
    height_at: Any | None = None,
) -> tuple[Any, dict[str, object]]:
    class FakeUvLayers:
        def __init__(self, mesh: Any) -> None:
            self.mesh = mesh

        def new(self, *, name: str) -> Any:
            return SimpleNamespace(
                name=name,
                data=[SimpleNamespace(uv=None) for _loop in self.mesh.loops],
            )

    class FakeMesh:
        def __init__(self) -> None:
            self.vertices: list[SimpleNamespace] = []
            self.loops: list[SimpleNamespace] = []
            self.materials: list[object] = []
            self.uv_layers = FakeUvLayers(self)
            self.attributes: dict[str, Any] = {}

        def from_pydata(
            self,
            vertices: list[tuple[float, float, float]],
            _edges: list[object],
            faces: list[tuple[int, ...]],
        ) -> None:
            self.vertices = [SimpleNamespace(co=SimpleNamespace(x=x, y=y, z=z)) for x, y, z in vertices]
            self.loops = [
                SimpleNamespace(index=index, vertex_index=vertex_index)
                for index, vertex_index in enumerate(vertex for face in faces for vertex in face)
            ]

            class FakeAttributes(dict[str, Any]):
                def new(attributes, *, name: str, type: str, domain: str) -> Any:
                    attribute = SimpleNamespace(
                        name=name,
                        type=type,
                        domain=domain,
                        data=[SimpleNamespace(value=None) for _vertex in self.vertices],
                    )
                    attributes[name] = attribute
                    return attribute

            self.attributes = FakeAttributes()

    mesh = FakeMesh()
    obj: dict[str, object] = {}
    blender = SimpleNamespace(
        data=SimpleNamespace(
            meshes=SimpleNamespace(new=lambda _name: mesh),
            objects=SimpleNamespace(new=lambda _name, _mesh: obj),
        )
    )
    collection = SimpleNamespace(objects=SimpleNamespace(link=lambda _obj: None))
    monkeypatch.setattr(author, "_require_bpy", lambda: blender)
    recipe = {"terrain_height_mode": "flat"}
    if height_at is not None:
        recipe = {"terrain_height_mode": "canonical_package"}
        monkeypatch.setattr(author, "_canonical_height_sampler", lambda _recipe: height_at)
    author._add_trail(recipe, layout, collection, object())
    return mesh, obj


def test_rendered_trail_uses_the_declared_tight_surface_width(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    layout = author.generate_forest_layout(_recipe())

    mesh, _obj = _capture_trail_mesh(monkeypatch, layout)
    surface_indices = (1, 2) if "TrailBlend" in mesh.attributes else (0, 1)
    rendered_width_m = math.dist(
        (mesh.vertices[surface_indices[0]].co.x, mesh.vertices[surface_indices[0]].co.y),
        (mesh.vertices[surface_indices[1]].co.x, mesh.vertices[surface_indices[1]].co.y),
    )

    assert rendered_width_m == pytest.approx(layout["trail"]["surface_width_m"])


def test_rendered_trail_resamples_to_four_metre_chords(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    layout = author.generate_forest_layout(_recipe())

    mesh, _obj = _capture_trail_mesh(monkeypatch, layout)
    section_size = 4 if "TrailBlend" in mesh.attributes else 2
    centers = [
        (
            sum(mesh.vertices[index + offset].co.x for offset in range(section_size)) / section_size,
            sum(mesh.vertices[index + offset].co.y for offset in range(section_size)) / section_size,
        )
        for index in range(0, len(mesh.vertices), section_size)
    ]

    assert max(math.dist(first, second) for first, second in pairwise(centers)) <= 4.0


def test_each_trail_edge_samples_terrain_and_hugs_it_within_three_centimetres(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    layout = author.generate_forest_layout(_recipe())

    def sloped_terrain(x_m: float, y_m: float) -> float:
        return 0.04 * x_m + 0.07 * y_m

    mesh, _obj = _capture_trail_mesh(monkeypatch, layout, height_at=sloped_terrain)
    gaps_m = [
        vertex.co.z - sloped_terrain(vertex.co.x, vertex.co.y)
        for vertex in mesh.vertices
    ]

    assert all(0.0 <= gap_m <= 0.03 for gap_m in gaps_m)


def test_robot_eye_foreground_contains_dense_ground_cover() -> None:
    layout = author.generate_forest_layout(_recipe())
    camera_xy = visuals.review_camera_specs(layout)["robot_eye"]["location_m"][:2]
    nearby_ground_cover = [
        item
        for item in layout["dressing"]
        if item["kind"] == "understory"
        and math.dist(item["position_m"][:2], camera_xy) <= 12.0
    ]

    assert len(nearby_ground_cover) >= 12


def test_morning_fog_keeps_the_route_readable_in_daylight() -> None:
    lighting = visuals.morning_fog_spec()

    assert 0.00022 <= lighting["fog_density"] <= 0.00032
    assert 3.0 <= lighting["sun"]["energy"] <= 4.5


def test_physical_sky_with_explicit_sun_uses_low_world_strength() -> None:
    lighting = visuals.morning_fog_spec()

    assert lighting["sky"]["sun_disc"] is True
    assert lighting["sun"]["energy"] > 0.0
    assert 0.06 <= lighting["world_strength"] <= 0.14


def test_muddy_trail_base_palette_is_darker_than_the_old_orange_brown() -> None:
    old_rgb = tuple(int("A06B3D"[index : index + 2], 16) for index in (0, 2, 4))
    new_hex = visuals.procedural_material_specs()["muddy_trail"]["palette"][0].removeprefix("#")
    new_rgb = tuple(int(new_hex[index : index + 2], 16) for index in (0, 2, 4))

    def luminance(rgb: tuple[int, int, int]) -> float:
        return 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]

    assert luminance(new_rgb) < luminance(old_rgb)


def test_scene_exposure_preserves_highlight_and_shadow_detail(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    scene = SimpleNamespace(
        render=SimpleNamespace(image_settings=SimpleNamespace()),
        view_settings=SimpleNamespace(),
        collection=SimpleNamespace(children=SimpleNamespace(link=lambda _collection: None)),
    )
    background = SimpleNamespace(
        inputs={
            "Color": SimpleNamespace(default_value=None),
            "Strength": SimpleNamespace(default_value=None),
        }
    )
    world = SimpleNamespace(use_nodes=False, node_tree=SimpleNamespace(nodes={"Background": background}))
    blender = SimpleNamespace(
        ops=SimpleNamespace(wm=SimpleNamespace(read_factory_settings=lambda **_kwargs: None)),
        context=SimpleNamespace(scene=scene),
        data=SimpleNamespace(
            worlds=SimpleNamespace(new=lambda _name: world),
            collections=SimpleNamespace(new=lambda name: SimpleNamespace(name=name)),
        ),
    )
    monkeypatch.setattr(author, "_require_bpy", lambda: blender)

    author._setup_scene(640, 360, 8)

    assert -0.10 <= scene.view_settings.exposure <= 0.35


def test_robot_eye_camera_frames_a_medium_range_trail_corridor() -> None:
    camera = visuals.review_camera_specs(author.generate_forest_layout(_recipe()))["robot_eye"]
    target_distance_m = math.dist(camera["location_m"], camera["target_m"])

    assert 35.0 <= target_distance_m <= 60.0


def test_ridge_camera_uses_the_hero_prefix_instead_of_the_closed_loop_endpoint() -> None:
    layout = {
        "extent_m": [1000.0, 1000.0],
        "trail": {
            "closed_loop": True,
            "centerline_m": [
                [0.0, 0.0],
                [300.0, 0.0],
                [600.0, 0.0],
                [600.0, 600.0],
                [0.0, 600.0],
                [0.0, 0.0],
            ],
        },
    }

    ridge = visuals.review_camera_specs(layout)["ridge"]

    assert ridge["target_m"][:2] == pytest.approx([420.0, 0.0])
    assert ridge["location_m"][:2] == pytest.approx([350.0, -55.0])


def test_ground_cover_populates_the_trail_edge_without_navigation_collision() -> None:
    layout = author.generate_forest_layout(_recipe())
    half_surface_width_m = layout["trail"]["surface_width_m"] * 0.5
    edge_dressing = [
        item
        for item in layout["dressing"]
        if item["kind"] == "understory"
        and 2.4
        <= _distance_to_trail(item["position_m"], layout["trail"]["centerline_m"])
        - half_surface_width_m
        <= 4.0
    ]

    assert len(edge_dressing) >= 24
    assert all(item["collision"] is False for item in edge_dressing)
    assert all(item["physics_representation"] == "none" for item in edge_dressing)


def test_morning_fog_is_subtle_enough_to_keep_depth_readable() -> None:
    atmosphere = visuals.morning_fog_spec()

    assert 0.00022 <= atmosphere["fog_density"] <= 0.00032


def test_morning_atmosphere_declares_sky_and_cloud_layers() -> None:
    atmosphere = visuals.morning_fog_spec()

    assert atmosphere["sky"]["model"]
    assert atmosphere["sky"]["sun_disc"] is True
    assert 0.35 <= atmosphere["clouds"]["coverage"] <= 0.7
    assert 0.0 < atmosphere["clouds"]["density"] <= 1.0


def test_robot_eye_camera_uses_natural_trail_composition() -> None:
    layout = author.generate_forest_layout(_recipe())
    camera = visuals.review_camera_specs(layout)["robot_eye"]
    start = layout["trail"]["centerline_m"][0]
    forward = layout["trail"]["centerline_m"][1]
    dx, dy = forward[0] - start[0], forward[1] - start[1]
    length = math.hypot(dx, dy)
    nx, ny = -dy / length, dx / length
    lateral_offset_m = abs(
        (camera["location_m"][0] - start[0]) * nx
        + (camera["location_m"][1] - start[1]) * ny
    )
    look_distance_m = math.dist(camera["location_m"][:2], camera["target_m"][:2])

    assert 38.0 <= camera["lens_mm"] <= 42.0
    assert 0.35 <= lateral_offset_m <= 0.55
    assert 1.05 <= camera["location_m"][2] <= 1.15
    assert 50.0 <= look_distance_m <= 65.0


def test_aerial_camera_is_perspective_with_an_oblique_overlook() -> None:
    camera = visuals.review_camera_specs(author.generate_forest_layout(_recipe()))["aerial"]
    horizontal_m = math.dist(camera["location_m"][:2], camera["target_m"][:2])
    vertical_m = abs(camera["location_m"][2] - camera["target_m"][2])
    depression_deg = math.degrees(math.atan2(vertical_m, horizontal_m))

    assert camera["projection"] == "perspective"
    assert 48.0 <= depression_deg <= 58.0


def test_review_camera_specs_use_scene_scale_appropriate_near_clipping() -> None:
    cameras = visuals.review_camera_specs(author.generate_forest_layout(_recipe()))

    assert cameras["robot_eye"]["clip_start_m"] >= 0.2
    assert cameras["establishing"]["clip_start_m"] >= 15.0
    assert cameras["ridge"]["clip_start_m"] >= 15.0
    assert cameras["aerial"]["clip_start_m"] >= 20.0


def test_create_review_cameras_applies_declared_near_clipping(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    camera_data: dict[str, Any] = {}

    class FakeDirection:
        def to_track_quat(self, _track: str, _up: str) -> Any:
            return SimpleNamespace(to_euler=lambda: (0.0, 0.0, 0.0))

    class FakeVector:
        def __init__(self, _values: Any) -> None:
            pass

        def __sub__(self, _other: Any) -> FakeDirection:
            return FakeDirection()

    def new_camera(name: str) -> Any:
        data = SimpleNamespace(
            lens=None,
            clip_start=None,
            clip_end=None,
            type="PERSP",
            ortho_scale=None,
        )
        camera_data[name] = data
        return data

    blender = SimpleNamespace(
        data=SimpleNamespace(
            cameras=SimpleNamespace(new=new_camera),
            objects=SimpleNamespace(
                new=lambda name, data: SimpleNamespace(name=name, data=data, location=None),
            ),
        ),
    )
    collection = SimpleNamespace(objects=SimpleNamespace(link=lambda _camera: None))
    monkeypatch.setitem(sys.modules, "mathutils", SimpleNamespace(Vector=FakeVector))

    expected = visuals.review_camera_specs(author.generate_forest_layout(_recipe()))
    visuals.create_review_cameras(blender, collection, author.generate_forest_layout(_recipe()))

    for camera_id, spec in expected.items():
        assert camera_data[f"Camera_Forest_{camera_id}"].clip_start == spec["clip_start_m"]


def _hero_route_bin_counts_by_side() -> dict[int, list[int]]:
    recipe = _long_hero_route_recipe()
    layout = author.generate_forest_layout(recipe)
    counts = {-1: [0] * 14, 1: [0] * 14}
    for tree in layout["instances"]:
        position = tree["position_m"]
        inside_hero_belt = (
            _distance_to_trail(position, layout["trail"]["centerline_m"])
            - tree["crown_radius_m"]
            - layout["trail"]["clearance_radius_m"]
            <= 18.0
        )
        distance_along_m = _distance_along_trail(position, layout["trail"]["centerline_m"])
        if not inside_hero_belt or distance_along_m > 420.0:
            continue
        side = 1 if position[1] >= 0.0 else -1
        bin_index = min(13, int(distance_along_m // 30.0))
        counts[side][bin_index] += 1
    return counts


def test_each_side_of_hero_route_has_at_least_three_canopy_windows() -> None:
    counts_by_side = _hero_route_bin_counts_by_side()

    assert all(sum(count <= 2 for count in counts) >= 3 for counts in counts_by_side.values())


def test_each_side_of_hero_route_retains_at_least_four_dense_tree_clusters() -> None:
    counts_by_side = _hero_route_bin_counts_by_side()

    assert all(sum(count >= 8 for count in counts) >= 4 for counts in counts_by_side.values())


def test_trail_cross_sections_include_the_declared_shoulder_width(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    layout = author.generate_forest_layout(_recipe())

    mesh, _obj = _capture_trail_mesh(monkeypatch, layout)

    assert len(mesh.vertices) % 4 == 0
    expected_outer_width_m = (
        layout["trail"]["surface_width_m"]
        + 2.0 * layout["trail"]["shoulder_blend_m"]
    )
    for index in range(0, len(mesh.vertices), 4):
        outer_width_m = math.dist(
            (mesh.vertices[index].co.x, mesh.vertices[index].co.y),
            (mesh.vertices[index + 3].co.x, mesh.vertices[index + 3].co.y),
        )
        assert outer_width_m == pytest.approx(expected_outer_width_m)


def test_trail_cross_sections_expose_outer_to_inner_blend_weights(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    layout = author.generate_forest_layout(_recipe())

    mesh, _obj = _capture_trail_mesh(monkeypatch, layout)

    trail_blend = mesh.attributes["TrailBlend"]
    assert trail_blend.domain == "POINT"
    assert [item.value for item in trail_blend.data[:4]] == pytest.approx([0.0, 1.0, 1.0, 0.0])


def test_first_120_metres_have_collision_free_shoulder_debris() -> None:
    layout = author.generate_forest_layout(_recipe())
    shoulder_debris = [
        item
        for item in layout["dressing"]
        if item["kind"] == "shoulder_debris"
        and _distance_along_trail(item["position_m"], layout["trail"]["centerline_m"])
        <= 120.0
    ]

    assert len(shoulder_debris) >= 16
    assert all(item["classification"] == "VisualOnly" for item in shoulder_debris)
    assert all(item["collision_profile"] == "NoCollision" for item in shoulder_debris)
    assert all(item["collision"] is False for item in shoulder_debris)


def _pixel(pixels: list[float], size: int, x: int, y: int) -> tuple[float, float, float, float]:
    offset = 4 * (y * size + x)
    return tuple(pixels[offset : offset + 4])


def _is_leaf_litter_brown(color: tuple[float, float, float, float]) -> bool:
    red, green, blue, _alpha = color
    return (
        red > green > blue
        and red <= green * 1.65
        and green >= blue * 1.25
    )


def test_forest_ground_basecolor_contains_a_continuous_tan_leaf_litter_field() -> None:
    size = 65
    base = author.outdoor_material_base_color("forest_ground", "#303D1C")

    pixels = author._procedural_pixels(
        size,
        5808,
        "basecolor",
        base,
        material_id="forest_ground",
    )
    unique_tile_pixels = [
        _pixel(pixels, size, x, y)
        for y in range(size - 1)
        for x in range(size - 1)
    ]
    leaf_litter_fraction = sum(
        _is_leaf_litter_brown(color)
        for color in unique_tile_pixels
    ) / len(unique_tile_pixels)

    assert 0.08 <= leaf_litter_fraction <= 0.35


def test_forest_ground_leaf_litter_regions_are_multicolour_not_flat_blocks() -> None:
    size = 65
    denominator = size - 1
    base = author.outdoor_material_base_color("forest_ground", "#303D1C")
    pixels = author._procedural_pixels(
        size,
        5808,
        "basecolor",
        base,
        material_id="forest_ground",
    )
    warm_pixels = [
        _pixel(pixels, size, x, y)
        for y in range(denominator)
        for x in range(denominator)
        if _is_leaf_litter_brown(_pixel(pixels, size, x, y))
    ]
    distinct_warm_colors = {
        tuple(round(channel, 5) for channel in color[:3]) for color in warm_pixels
    }

    assert len(distinct_warm_colors) >= 32
    for y in range(denominator - 3):
        for x in range(denominator - 3):
            block = [
                _pixel(pixels, size, block_x, block_y)
                for block_y in range(y, y + 4)
                for block_x in range(x, x + 4)
            ]
            if all(_is_leaf_litter_brown(color) for color in block):
                assert len({tuple(color[:3]) for color in block}) > 1


def test_forest_ground_neighbour_transitions_have_no_block_boundary_jumps() -> None:
    size = 65
    denominator = size - 1
    base = author.outdoor_material_base_color("forest_ground", "#303D1C")
    pixels = author._procedural_pixels(
        size,
        5808,
        "basecolor",
        base,
        material_id="forest_ground",
    )
    neighbour_differences = []
    for y in range(denominator):
        for x in range(denominator):
            color = _pixel(pixels, size, x, y)
            for neighbour_x, neighbour_y in ((x + 1, y), (x, y + 1)):
                if neighbour_x >= denominator or neighbour_y >= denominator:
                    continue
                neighbour = _pixel(pixels, size, neighbour_x, neighbour_y)
                neighbour_differences.append(
                    max(abs(left - right) for left, right in zip(color[:3], neighbour[:3]))
                )
    neighbour_differences.sort()
    percentile_99 = neighbour_differences[math.ceil(len(neighbour_differences) * 0.99) - 1]

    assert percentile_99 < 0.03
    assert max(neighbour_differences) < 0.08


def test_forest_ground_main_albedo_is_low_saturation_olive_brown() -> None:
    red, green, blue = author.outdoor_material_base_color("forest_ground", "#303D1C")
    saturation = (max(red, green, blue) - min(red, green, blue)) / max(red, green, blue)

    assert green > red > blue
    assert saturation <= 0.55


def test_forest_ground_warm_fleck_texture_is_tileable() -> None:
    size = 65
    base = author.outdoor_material_base_color("forest_ground", "#303D1C")
    pixels = author._procedural_pixels(
        size,
        5808,
        "basecolor",
        base,
        material_id="forest_ground",
    )

    for index in range(size):
        assert _pixel(pixels, size, 0, index) == pytest.approx(
            _pixel(pixels, size, size - 1, index),
            abs=1e-7,
        )
        assert _pixel(pixels, size, index, 0) == pytest.approx(
            _pixel(pixels, size, index, size - 1),
            abs=1e-7,
        )
