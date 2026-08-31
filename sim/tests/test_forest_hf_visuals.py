# ruff: noqa: S101

"""Pure contract tests for the richer Forest_HF Blender visuals."""

from __future__ import annotations

import importlib
import itertools
import math
import sys
from types import SimpleNamespace

import pytest


def _visuals():
    return importlib.import_module("sim.tools.worlds.forest_hf.blender_visuals")


def _face_vertices(
    mesh: dict[str, list[tuple[float, float, float]] | list[tuple[int, ...]]],
) -> list[list[tuple[float, float, float]]]:
    vertices = mesh["vertices"]
    return [[vertices[index] for index in face] for face in mesh["faces"]]


class _Blender52SkyNode(SimpleNamespace):
    @property
    def sky_type(self) -> str:
        return self._sky_type

    @sky_type.setter
    def sky_type(self, value: str) -> None:
        if value not in {
            "SINGLE_SCATTERING",
            "MULTIPLE_SCATTERING",
            "PREETHAM",
            "HOSEK_WILKIE",
        }:
            raise ValueError(f"unsupported Blender 5.2 sky type: {value}")
        self._sky_type = value


def test_visual_helpers_import_without_bpy() -> None:
    visuals = _visuals()

    assert visuals.VISUALS_VERSION == 2
    assert visuals.CLASSIFICATION == "VisualOnly"
    assert visuals.COLLISION_PROFILE == "NoCollision"


def test_assets_are_project_owned_visual_only_contracts() -> None:
    visuals = _visuals()

    specs = visuals.procedural_asset_specs()

    assert set(specs) == {"pine", "birch", "fern", "grass", "inspection_marker"}
    assert specs["birch"]["semantic_class"] == "tree.birch"
    assert specs["pine"]["object_name"] == "LT_Forest_Asset_Pine"
    assert specs["birch"]["object_name"] == "LT_Forest_Asset_Birch"
    for spec in specs.values():
        assert spec["classification"] == "VisualOnly"
        assert spec["collision_profile"] == "NoCollision"
        assert spec["simulate_physics"] is False
        assert spec["can_ever_affect_navigation"] is False
        assert spec["source"].startswith("repo://")
        assert spec["license"] == "LicenseRef-LingTu-Project-Owned"


@pytest.mark.parametrize(
    ("asset_id", "mesh_factory_name"),
    (("grass", "grass_clump_mesh_data"), ("fern", "fern_clump_mesh_data")),
)
def test_understory_reference_height_matches_authored_mesh(
    asset_id: str,
    mesh_factory_name: str,
) -> None:
    visuals = _visuals()
    mesh = getattr(visuals, mesh_factory_name)()
    authored_height_m = max(vertex[2] for vertex in mesh["vertices"])

    assert visuals.procedural_asset_specs()[asset_id]["reference_height_m"] == pytest.approx(
        authored_height_m
    )


def test_grass_clump_uses_short_non_uniform_blade_heights() -> None:
    mesh = _visuals().grass_clump_mesh_data()
    blade_heights = [max(vertex[2] for vertex in face) for face in _face_vertices(mesh)]
    height_steps = {
        round(second - first, 5)
        for first, second in itertools.pairwise(sorted(set(blade_heights)))
    }

    assert min(blade_heights) >= 0.12
    assert max(blade_heights) <= 0.32
    assert len({round(height, 4) for height in blade_heights}) >= 5
    assert len(height_steps) >= 2


def test_every_grass_blade_converges_to_one_tip() -> None:
    mesh = _visuals().grass_clump_mesh_data()

    assert all(
        sum(math.isclose(vertex[2], max(point[2] for point in face)) for vertex in face)
        == 1
        for face in _face_vertices(mesh)
    )


def test_fern_clump_stays_below_half_metre_scale() -> None:
    mesh = _visuals().fern_clump_mesh_data()
    vertices = mesh["vertices"]

    assert 0.18 <= max(vertex[2] for vertex in vertices) <= 0.55


def test_fern_fronds_spread_outward_instead_of_forming_sword_leaves() -> None:
    mesh = _visuals().fern_clump_mesh_data()
    vertices = mesh["vertices"]
    height_m = max(vertex[2] for vertex in vertices)
    radial_reach_m = max(math.hypot(vertex[0], vertex[1]) for vertex in vertices)

    assert radial_reach_m >= height_m * 0.75


def test_materials_use_only_project_owned_procedural_sources() -> None:
    visuals = _visuals()

    specs = visuals.procedural_material_specs()

    assert "muddy_trail" in specs
    assert "wet_mud" in specs["muddy_trail"]["pattern"]
    assert set(specs["birch_bark"]["palette"]) == {"#D8D1C1", "#3A332C"}
    for spec in specs.values():
        assert spec["generator"] == "lingtu_png_pbr"
        assert spec["external_assets"] == []
        assert spec["source"].startswith("repo://")
        assert spec["license"] == "LicenseRef-LingTu-Project-Owned"


def test_muddy_trail_uses_bright_brown_albedo_with_gentle_normals() -> None:
    visuals = _visuals()

    muddy = visuals.procedural_material_specs()["muddy_trail"]
    colors = [
        tuple(int(color[index : index + 2], 16) / 255.0 for index in (1, 3, 5))
        for color in muddy["palette"]
    ]

    assert all(red > green > blue for red, green, blue in colors)
    assert max(sum(color) / 3.0 for color in colors) >= 0.42
    assert 0.0 < muddy["normal_strength"] <= 0.06


def test_forest_ground_uses_subtle_normals_for_aerial_readability() -> None:
    visuals = _visuals()

    ground = visuals.procedural_material_specs()["forest_ground"]

    assert 0.0 < ground["normal_strength"] <= 0.10


def test_marker_material_is_bright_amber_and_emissive() -> None:
    visuals = _visuals()

    marker = visuals.procedural_material_specs()["marker_amber"]
    bright = marker["palette"][-1]
    red, green, blue = (
        int(bright[index : index + 2], 16) / 255.0 for index in (1, 3, 5)
    )

    assert red >= 0.85
    assert 0.45 <= green < red
    assert blue <= 0.2
    assert marker["emission_strength"] >= 1.0


def test_marker_asset_declares_robot_readable_plate_geometry() -> None:
    visuals = _visuals()

    marker = visuals.procedural_asset_specs()["inspection_marker"]

    assert marker["geometry"]["pole_height_m"] >= 1.0
    assert marker["geometry"]["plate_width_m"] >= 0.35
    assert marker["geometry"]["plate_height_m"] >= 0.2


def test_review_camera_specs_follow_layout_trail_and_extent() -> None:
    visuals = _visuals()
    layout = {
        "extent_m": [100.0, 80.0],
        "trail": {"centerline_m": [[-40.0, -20.0], [5.0, 3.0], [40.0, 20.0]]},
    }

    cameras = visuals.review_camera_specs(layout)

    assert set(cameras) == {"establishing", "robot_eye", "ridge", "aerial"}
    assert cameras["robot_eye"]["location_m"][2] == 1.25
    assert cameras["ridge"]["target_m"][:2] == [40.0, 20.0]
    assert cameras["aerial"]["projection"] == "orthographic"
    assert cameras["aerial"]["target_m"][:2] == pytest.approx([0.0, 0.0])
    assert cameras["aerial"]["ortho_scale_m"] < 2.0 * max(layout["extent_m"])
    assert all(
        camera["clip_end_m"] >= 3.0 * max(layout["extent_m"])
        for camera in cameras.values()
    )


def test_review_cameras_ignore_the_duplicate_endpoint_of_a_closed_trail() -> None:
    visuals = _visuals()
    layout = {
        "extent_m": [1000.0, 1000.0],
        "trail": {
            "centerline_m": [
                [-760.0, -650.0],
                [420.0, 710.0],
                [-700.0, 500.0],
                [-760.0, -650.0],
            ]
        },
    }

    cameras = visuals.review_camera_specs(layout)

    assert cameras["ridge"]["target_m"][:2] == [-700.0, 500.0]
    assert cameras["ridge"]["location_m"][:2] != cameras["robot_eye"]["location_m"][:2]


def test_aerial_camera_focuses_hero_route_instead_of_leaving_world_border() -> None:
    visuals = _visuals()
    layout = {
        "extent_m": [1000.0, 1000.0],
        "trail": {
            "centerline_m": [[-80.0, -40.0], [-10.0, 5.0], [70.0, 50.0]]
        },
    }

    aerial = visuals.review_camera_specs(layout)["aerial"]

    assert aerial["location_m"][:2] == pytest.approx([-5.0, 5.0])
    assert aerial["target_m"][:2] == pytest.approx([-5.0, 5.0])
    assert 100.0 <= aerial["ortho_scale_m"] <= 250.0


def test_large_closed_route_aerial_frames_only_start_and_first_leg() -> None:
    visuals = _visuals()
    layout = {
        "extent_m": [2000.0, 2000.0],
        "trail": {
            "centerline_m": [
                [-760.0, -650.0],
                [-360.0, -420.0],
                [620.0, 720.0],
                [740.0, -620.0],
                [-760.0, -650.0],
            ]
        },
    }

    aerial = visuals.review_camera_specs(layout)["aerial"]
    start = layout["trail"]["centerline_m"][0]
    first_leg = layout["trail"]["centerline_m"][1]
    expected_center = [(start[0] + first_leg[0]) / 2.0, (start[1] + first_leg[1]) / 2.0]

    assert aerial["target_m"][:2] == pytest.approx(expected_center, abs=25.0)
    assert aerial["location_m"][:2] == pytest.approx(expected_center, abs=25.0)
    assert 350.0 <= aerial["ortho_scale_m"] <= 500.0


def test_inspection_markers_start_ahead_and_keep_fixed_route_spacing() -> None:
    visuals = _visuals()
    layout = {
        "extent_m": [1000.0, 1000.0],
        "trail": {
            "centerline_m": [[0.0, 0.0], [40.0, 0.0], [40.0, 80.0]]
        },
    }

    markers = visuals.inspection_marker_placements(layout)
    route_distances = [float(marker["distance_along_route_m"]) for marker in markers]

    assert 20.0 <= route_distances[0] <= 30.0
    assert all(
        math.isclose(second - first, 25.0, abs_tol=1e-6)
        for first, second in itertools.pairwise(route_distances)
    )


def test_morning_fog_contract_is_cool_dawn_not_physics() -> None:
    visuals = _visuals()

    spec = visuals.morning_fog_spec()

    assert spec["fog_density"] > 0.0
    assert spec["fog_density"] <= 0.003
    assert spec["sun"]["altitude_deg"] == 24.0
    assert spec["sun"]["azimuth_deg"] == 132.0
    assert spec["fill"]["color"][2] > spec["fill"]["color"][0]


def test_morning_fog_maps_nishita_intent_to_a_blender_52_sky_type(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    visuals = _visuals()
    supported_sky_types = {
        "SINGLE_SCATTERING",
        "MULTIPLE_SCATTERING",
        "PREETHAM",
        "HOSEK_WILKIE",
    }
    selected_sky_types: list[str] = []

    class StopAfterCompatibleSky(RuntimeError):
        pass

    class SkyNode:
        name = ""

        @property
        def sky_type(self) -> str:
            return selected_sky_types[-1]

        @sky_type.setter
        def sky_type(self, value: str) -> None:
            if value not in supported_sky_types:
                raise ValueError(f"unsupported Blender 5.2 sky type: {value}")
            selected_sky_types.append(value)
            raise StopAfterCompatibleSky

    class MaterialNodes:
        def clear(self) -> None:
            return None

        def new(self, node_type: str) -> SimpleNamespace:
            if node_type == "ShaderNodeVolumePrincipled":
                return SimpleNamespace(
                    inputs={
                        "Color": SimpleNamespace(default_value=None),
                        "Density": SimpleNamespace(default_value=None),
                    },
                    outputs={"Volume": object()},
                )
            return SimpleNamespace(inputs={"Volume": object()})

    class WorldNodes:
        def get(self, _name: str) -> SimpleNamespace:
            return SimpleNamespace(
                inputs={
                    "Color": SimpleNamespace(default_value=None),
                    "Strength": SimpleNamespace(default_value=None),
                }
            )

        def new(self, node_type: str) -> object:
            assert node_type == "ShaderNodeTexSky"
            return SkyNode()

    material_nodes = MaterialNodes()
    material_tree = SimpleNamespace(
        nodes=material_nodes,
        links=SimpleNamespace(new=lambda *_args: None),
    )
    world_nodes = WorldNodes()
    world_tree = SimpleNamespace(
        nodes=world_nodes,
        links=SimpleNamespace(new=lambda *_args: None),
    )
    world = SimpleNamespace(use_nodes=False, node_tree=world_tree)
    material = SimpleNamespace(use_nodes=False, node_tree=material_tree)
    fog = SimpleNamespace(
        name="",
        data=SimpleNamespace(materials=[]),
    )
    blender = SimpleNamespace(
        context=SimpleNamespace(scene=SimpleNamespace(world=world), object=fog),
        data=SimpleNamespace(
            worlds=SimpleNamespace(new=lambda _name: world),
            materials=SimpleNamespace(
                get=lambda _name: material,
                new=lambda _name: material,
            ),
        ),
        ops=SimpleNamespace(
            mesh=SimpleNamespace(primitive_cube_add=lambda **_kwargs: None)
        ),
    )
    monkeypatch.setitem(sys.modules, "mathutils", SimpleNamespace(Vector=object))
    monkeypatch.setattr(visuals, "_move_to_collection", lambda *_args: None)
    monkeypatch.setattr(visuals, "_mark_visual", lambda *_args: None)

    assert visuals.morning_fog_spec()["sky"]["model"] == "nishita"
    with pytest.raises(StopAfterCompatibleSky):
        visuals.setup_morning_fog(blender, SimpleNamespace())

    assert selected_sky_types and selected_sky_types[0] in supported_sky_types


def test_blender_52_sky_maps_dust_intent_to_aerosol_density() -> None:
    visuals = _visuals()
    spec = visuals.morning_fog_spec()
    sky = _Blender52SkyNode(aerosol_density=0.0)

    visuals._configure_physical_sky_node(sky, spec)

    assert sky.sky_type == "MULTIPLE_SCATTERING"
    assert sky.aerosol_density == spec["sky"]["dust_density"]


def test_physical_sky_fails_when_particle_density_property_is_unavailable() -> None:
    visuals = _visuals()
    sky = _Blender52SkyNode()

    with pytest.raises(
        RuntimeError,
        match="neither aerosol_density nor dust_density",
    ):
        visuals._configure_physical_sky_node(sky, visuals.morning_fog_spec())


def test_fog_volume_is_created_at_unit_scale_before_world_sizing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    visuals = _visuals()
    cube_calls: list[dict[str, object]] = []

    class StopAfterCube(RuntimeError):
        pass

    def primitive_cube_add(**kwargs: object) -> None:
        cube_calls.append(kwargs)
        raise StopAfterCube

    class FakeNodes:
        def clear(self) -> None:
            return None

        def get(self, _name: str) -> SimpleNamespace:
            return SimpleNamespace(
                inputs={
                    "Color": SimpleNamespace(default_value=None),
                    "Strength": SimpleNamespace(default_value=None),
                }
            )

        def new(self, node_type: str) -> SimpleNamespace:
            if node_type == "ShaderNodeVolumePrincipled":
                return SimpleNamespace(
                    inputs={
                        "Color": SimpleNamespace(default_value=None),
                        "Density": SimpleNamespace(default_value=None),
                    },
                    outputs={"Volume": object()},
                )
            return SimpleNamespace(inputs={"Volume": object()})

    nodes = FakeNodes()
    node_tree = SimpleNamespace(
        nodes=nodes,
        links=SimpleNamespace(new=lambda *_args: None),
    )
    world = SimpleNamespace(use_nodes=False, node_tree=node_tree)
    material = SimpleNamespace(use_nodes=False, node_tree=node_tree)
    blender = SimpleNamespace(
        context=SimpleNamespace(scene=SimpleNamespace(world=world)),
        data=SimpleNamespace(
            worlds=SimpleNamespace(new=lambda _name: world),
            materials=SimpleNamespace(
                get=lambda _name: material,
                new=lambda _name: material,
            ),
        ),
        ops=SimpleNamespace(mesh=SimpleNamespace(primitive_cube_add=primitive_cube_add)),
    )
    monkeypatch.setitem(sys.modules, "mathutils", SimpleNamespace(Vector=object))

    with pytest.raises(StopAfterCube):
        visuals.setup_morning_fog(blender, SimpleNamespace())

    assert cube_calls == [{"size": 1.0, "scale": (1.0, 1.0, 1.0), "location": (0, 0, 10.0)}]
