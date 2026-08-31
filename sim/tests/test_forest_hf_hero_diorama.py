# ruff: noqa: S101

"""Review-only contracts for the standalone Forest_HF hero diorama."""

from __future__ import annotations

import importlib
import math
import os
from collections import Counter
from itertools import pairwise
from pathlib import Path
from types import ModuleType
from typing import Any, cast

import pytest


def _subject() -> ModuleType:
    return importlib.import_module(
        "sim.tools.worlds.forest_hf.blender_hero_diorama"
    )


def _external_asset_slots() -> dict[str, dict[str, object]]:
    return {
        "pine": {
            "path": "conditioned/pine/forest_pine_lod2.glb",
            "bytes": 3_239_260,
            "sha256": "3" * 64,
            "embed_depth_m": 0.24,
        },
        "birch": {
            "path": "conditioned/birch/forest_birch_lod2.glb",
            "bytes": 3_110_916,
            "sha256": "4" * 64,
            "embed_depth_m": 0.22,
        },
        "boulder": {
            "path": "conditioned/boulder/forest_boulder_lod2.glb",
            "bytes": 4_086_904,
            "sha256": "f" * 64,
            "embed_depth_m": 0.10,
        },
    }


def _plan(*, include_thunder: bool = False) -> dict[str, Any]:
    return cast(
        dict[str, Any],
        _subject().build_hero_diorama_plan(
            _external_asset_slots(),
            seed=5808,
            include_thunder=include_thunder,
        ),
    )


def _turn_signs(centerline: list[list[float]]) -> set[int]:
    signs = set()
    for first, corner, last in zip(centerline, centerline[1:], centerline[2:]):
        incoming = (corner[0] - first[0], corner[1] - first[1])
        outgoing = (last[0] - corner[0], last[1] - corner[1])
        cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
        if not math.isclose(cross, 0.0, abs_tol=1e-9):
            signs.add(1 if cross > 0.0 else -1)
    return signs


def _distance_to_segment(
    point: list[float],
    start: list[float],
    end: list[float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    squared_length = dx * dx + dy * dy
    if squared_length == 0.0:
        return math.dist(point[:2], start[:2])
    amount = min(
        1.0,
        max(
            0.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / squared_length,
        ),
    )
    closest = [start[0] + amount * dx, start[1] + amount * dy]
    return math.dist(point[:2], closest)


def _distance_to_centerline(point: list[float], centerline: list[list[float]]) -> float:
    return min(
        _distance_to_segment(point, start, end)
        for start, end in pairwise(centerline)
    )


def _percentile_90(values: list[float]) -> float:
    ordered = sorted(values)
    return ordered[math.ceil(len(ordered) * 0.9) - 1]


def _hex_rgb(value: str) -> tuple[int, int, int]:
    normalized = value.removeprefix("#")
    assert len(normalized) == 6
    return tuple(int(normalized[offset : offset + 2], 16) for offset in (0, 2, 4))


def test_review_plan_is_byte_stable_for_the_same_seed_and_asset_identities() -> None:
    first = _plan(include_thunder=True)
    second = _plan(include_thunder=True)

    assert first == second


def test_review_plan_builds_a_local_one_hundred_twenty_metre_s_curve() -> None:
    plan = _plan()
    centerline = plan["trail"]["centerline_m"]
    length_m = sum(math.dist(first, second) for first, second in pairwise(centerline))

    assert plan["extent_m"] == pytest.approx([120.0, 100.0])
    assert 100.0 <= length_m <= 140.0
    assert _turn_signs(centerline) == {-1, 1}
    assert plan["trail"]["surface_width_m"] == pytest.approx(3.0, abs=0.4)


def test_review_plan_populates_dense_pine_and_birch_near_mid_far_layers() -> None:
    trees = _plan()["trees"]
    counts = Counter((tree["depth_band"], tree["species_id"]) for tree in trees)

    assert len(trees) >= 90
    for depth_band in ("near", "mid", "far"):
        assert counts[(depth_band, "pine")] >= 8
        assert counts[(depth_band, "birch")] >= 4


def test_review_tree_distribution_forms_local_clusters_instead_of_a_grid() -> None:
    trees = _plan()["trees"]
    nearest_neighbour_m = [
        min(
            math.dist(tree["position_m"][:2], other["position_m"][:2])
            for other in trees
            if other is not tree
        )
        for tree in trees
    ]
    clustered_fraction = sum(distance <= 8.0 for distance in nearest_neighbour_m) / len(trees)
    rounded_positions = {
        (round(tree["position_m"][0], 3), round(tree["position_m"][1], 3))
        for tree in trees
    }

    assert clustered_fraction >= 0.80
    assert len(rounded_positions) == len(trees)
    assert len({round(distance, 2) for distance in nearest_neighbour_m}) >= 12


def test_review_plan_preserves_conditioned_external_asset_identities() -> None:
    slots = _external_asset_slots()
    assets = _plan()["external_asset_slots"]

    assert assets == slots


def test_review_plan_dresses_the_corridor_with_boulders_ferns_and_grass() -> None:
    dressing = _plan()["dressing"]
    counts = Counter(item["kind"] for item in dressing)

    assert counts["boulder"] >= 8
    assert counts["fern"] >= 24
    assert counts["grass"] >= 48
    assert all(item["kind"] != "fallen_log" for item in dressing)


def test_review_plan_uses_dense_clustered_short_grass() -> None:
    grass = [item for item in _plan()["dressing"] if item["kind"] == "grass"]

    assert len(grass) >= 800
    assert len({item["patch_id"] for item in grass}) >= 24
    assert all(item["patch_id"] is not None for item in grass)


def test_review_plan_uses_a_bounded_clustered_fern_population() -> None:
    ferns = [item for item in _plan()["dressing"] if item["kind"] == "fern"]

    assert 180 <= len(ferns) <= 260
    assert len({item["patch_id"] for item in ferns}) >= 18
    assert all(item["patch_id"] is not None for item in ferns)


def test_review_plan_keeps_ninety_percent_of_grass_below_thirty_five_centimetres() -> None:
    visuals = importlib.import_module("sim.tools.worlds.forest_hf.blender_visuals")
    template_height_m = max(
        vertex[2] for vertex in visuals.grass_clump_mesh_data()["vertices"]
    )
    world_heights_m = [
        template_height_m * float(item["scale"][2])
        for item in _plan()["dressing"]
        if item["kind"] == "grass"
    ]

    assert _percentile_90(world_heights_m) <= 0.35


def test_review_plan_keeps_ninety_percent_of_ferns_below_sixty_centimetres() -> None:
    visuals = importlib.import_module("sim.tools.worlds.forest_hf.blender_visuals")
    template_height_m = max(
        vertex[2] for vertex in visuals.fern_clump_mesh_data()["vertices"]
    )
    world_heights_m = [
        template_height_m * float(item["scale"][2])
        for item in _plan()["dressing"]
        if item["kind"] == "fern"
    ]

    assert _percentile_90(world_heights_m) <= 0.60


def test_hero_ground_surface_contract_uses_centimetre_metre_and_multi_metre_detail() -> None:
    contract = _subject().hero_ground_surface_contract()
    scales_m = sorted(float(layer["scale_m"]) for layer in contract["detail_layers"])

    assert any(0.01 <= scale <= 0.09 for scale in scales_m)
    assert any(0.1 <= scale <= 1.9 for scale in scales_m)
    assert any(scale >= 2.0 for scale in scales_m)


def test_hero_ground_surface_contract_uses_rough_forest_soil() -> None:
    contract = _subject().hero_ground_surface_contract()

    assert 0.78 <= float(contract["roughness"]) <= 0.94


def test_hero_ground_surface_contract_limits_moss_to_sparse_patches() -> None:
    contract = _subject().hero_ground_surface_contract()
    mask = contract["moss_mask"]

    assert float(contract["surface_classes"]["moss"]["coverage"]) <= 0.15
    assert 0.55 <= float(mask["low_threshold"]) < float(mask["high_threshold"]) <= 0.95
    assert (float(mask["low_threshold"]) + float(mask["high_threshold"])) / 2.0 >= 0.65


def test_hero_ground_surface_contract_uses_warm_soil_humus_and_olive_moss() -> None:
    classes = _subject().hero_ground_surface_contract()["surface_classes"]
    moist_soil = _hex_rgb(str(classes["moist_soil"]["color"]))
    humus = _hex_rgb(str(classes["forest_humus"]["color"]))
    moss = _hex_rgb(str(classes["moss"]["color"]))

    assert moist_soil[0] > moist_soil[1] > moist_soil[2]
    assert humus[0] > humus[1] > humus[2]
    assert moss[1] > moss[0] > moss[2]
    assert moss[1] - moss[0] <= 32


def test_hero_ground_detail_counts_make_litter_and_tree_contact_readable() -> None:
    counts = _subject().hero_ground_detail_counts()

    assert counts["wet_leaves"] >= 8000
    assert counts["fallen_twigs"] >= 240
    assert counts["tree_base_moss"] >= 150


def test_hero_ground_noise_frequencies_are_reciprocals_of_metre_scales() -> None:
    subject = _subject()
    contract = subject.hero_ground_surface_contract()
    frequencies = subject.hero_ground_noise_frequencies(contract)

    assert frequencies == pytest.approx(
        {
            str(layer["layer_id"]): 1.0 / float(layer["scale_m"])
            for layer in contract["detail_layers"]
        }
    )


def test_hero_ground_noise_rejects_normalized_generated_coordinates() -> None:
    contract = _subject().hero_ground_surface_contract()

    assert contract["noise_coordinate_input"] in {"Object", "Position"}
    assert str(contract["noise_coordinate_input"]).casefold() != "generated"


def test_trail_shoulders_fade_from_opaque_inner_edge_to_transparent_outer_edge() -> None:
    mesh = _subject().trail_shoulder_mesh_data(_plan())
    weights = mesh["blend_weights"]

    assert mesh["faces"]
    assert len(weights) == len(mesh["vertices"])
    assert set(weights[0::2]) == {0.0}
    assert set(weights[1::2]) == {1.0}


def test_trail_shoulders_transition_between_zero_point_seven_and_one_point_three_metres() -> None:
    mesh = _subject().trail_shoulder_mesh_data(_plan())

    assert all(0.7 <= float(width) <= 1.3 for width in mesh["side_widths_m"])


def test_trail_shoulder_mesh_has_one_inner_and_outer_vertex_per_side_station() -> None:
    plan = _plan()
    mesh = _subject().trail_shoulder_mesh_data(plan)
    station_count = len(plan["trail"]["centerline_m"])
    segment_count = station_count - 1

    assert len(mesh["vertices"]) == 4 * station_count
    assert len(set(mesh["vertices"])) == len(mesh["vertices"])
    for side_faces in (
        mesh["faces"][:segment_count],
        mesh["faces"][segment_count:],
    ):
        assert len(set().union(*map(set, side_faces))) == 2 * station_count


def test_adjacent_trail_shoulder_faces_share_one_complete_station_edge() -> None:
    plan = _plan()
    mesh = _subject().trail_shoulder_mesh_data(plan)
    segment_count = len(plan["trail"]["centerline_m"]) - 1

    for side_faces in (
        mesh["faces"][:segment_count],
        mesh["faces"][segment_count:],
    ):
        assert all(
            len(set(first).intersection(second)) == 2
            for first, second in pairwise(side_faces)
        )


def test_trail_shoulder_material_contract_fades_outer_to_inner_alpha() -> None:
    contract = _subject().trail_shoulder_surface_contract()

    assert contract["outer_alpha"] == 0.0
    assert contract["inner_alpha"] == 1.0


def test_trail_shoulder_material_noise_uses_spatial_coordinates() -> None:
    contract = _subject().trail_shoulder_surface_contract()

    assert contract["noise_coordinate_input"] in {"Object", "Position"}
    assert str(contract["noise_coordinate_input"]).casefold() != "generated"


def test_hero_trail_surface_contract_uses_object_local_metres() -> None:
    contract = _subject().hero_trail_surface_contract()

    assert contract["coordinate_space"] == "object_local_metres"
    assert contract["noise_coordinate_input"] == "Object"
    assert str(contract["noise_coordinate_input"]).casefold() != "generated"


def test_hero_trail_surface_contract_has_three_mud_detail_scales() -> None:
    contract = _subject().hero_trail_surface_contract()
    scales_m = sorted(float(layer["scale_m"]) for layer in contract["detail_layers"])
    surface_language = " ".join(
        f'{layer["layer_id"]} {layer["role"]}' for layer in contract["detail_layers"]
    ).casefold()

    assert any(0.01 <= scale <= 0.09 for scale in scales_m)
    assert any(0.10 <= scale <= 1.90 for scale in scales_m)
    assert any(scale >= 2.0 for scale in scales_m)
    assert "mud" in surface_language
    assert "wet" in surface_language or "damp" in surface_language


def test_hero_trail_surface_contract_keeps_wet_mud_rough_but_varied() -> None:
    roughness_range = _subject().hero_trail_surface_contract()["roughness_range"]

    assert 0.62 <= float(roughness_range[0]) < float(roughness_range[1]) <= 0.94


def test_hero_lighting_contract_uses_a_soft_low_sun() -> None:
    sun = _subject().hero_lighting_contract()["sun"]

    assert 5.0 <= float(sun["angle_deg"]) <= 7.0


def test_hero_lighting_contract_balances_world_fill_and_exposure() -> None:
    contract = _subject().hero_lighting_contract()

    assert 0.14 <= float(contract["world_strength"]) <= 0.22
    assert 110.0 <= float(contract["fill"]["energy"]) <= 180.0
    assert 0.08 <= float(contract["exposure"]) <= 0.25


def test_hero_lighting_contract_uses_neutral_non_blue_fill() -> None:
    color = [float(channel) for channel in _subject().hero_lighting_contract()["fill"]["color"]]

    assert len(color) == 3
    assert max(color) - min(color) <= 0.10
    assert color[2] <= color[0] * 1.05


def test_optional_thunder_is_grounded_as_a_non_physical_review_actor() -> None:
    without_robot = _plan(include_thunder=False)
    with_robot = _plan(include_thunder=True)
    robot = with_robot["review_actor"]

    assert without_robot["review_actor"] is None
    assert robot["asset_id"] == "ThunderV4_v1.0.3"
    assert robot["grounding_policy"] == "lowest_vertex_to_terrain"
    assert 0.0 <= robot["ground_clearance_m"] <= 0.03
    assert robot["classification"] == "VisualOnly"
    assert robot["collision_profile"] == "NoCollision"
    assert robot["simulate_physics"] is False


def test_review_plan_has_one_off_axis_hero_camera() -> None:
    cameras = _plan(include_thunder=True)["cameras"]
    camera = cameras[0]

    assert len(cameras) == 1
    assert camera["camera_id"] == "hero_diorama"
    assert camera["projection"] == "perspective"
    assert 45.0 <= camera["lens_mm"] <= 55.0
    assert abs(camera["location_m"][1] - camera["target_m"][1]) >= 3.0


def test_review_plan_is_visual_only_and_cannot_mutate_runtime_authority() -> None:
    plan = _plan(include_thunder=True)
    contract = plan["runtime_contract"]
    scene_items = [*plan["trees"], *plan["dressing"], plan["review_actor"]]

    assert plan["review_only"] is True
    assert plan["modifies_production_world"] is False
    assert plan["modifies_physics"] is False
    assert contract == {
        "classification": "VisualOnly",
        "collision_profile": "NoCollision",
        "physics_authority": "MuJoCo",
    }
    assert all(item["classification"] == "VisualOnly" for item in scene_items)
    assert all(item["collision_profile"] == "NoCollision" for item in scene_items)


def test_review_plan_outputs_only_one_png_and_one_review_manifest() -> None:
    outputs = _plan(include_thunder=True)["outputs"]

    assert outputs == [
        {
            "artifact_role": "hero_diorama_preview",
            "filename": "forest_hf.hero-diorama.png",
        },
        {
            "artifact_role": "review_manifest",
            "filename": "forest_hf.hero-diorama.review.json",
        },
    ]
    assert not any(
        output["filename"].lower().endswith((".blend", ".fbx", ".glb", ".gltf"))
        for output in outputs
    )
    assert all(
        output["filename"] not in {"world.package.yaml", "authoring.manifest.json"}
        for output in outputs
    )


def test_trail_mesh_quads_all_have_upward_winding() -> None:
    mesh = _subject().trail_mesh_data(_plan())
    vertices = mesh["vertices"]

    assert mesh["faces"]
    for face in mesh["faces"]:
        assert len(face) == 4
        signed_double_area = sum(
            vertices[first][0] * vertices[second][1]
            - vertices[second][0] * vertices[first][1]
            for first, second in pairwise([*face, face[0]])
        )
        assert signed_double_area > 0.0


def test_every_tree_clears_the_trail_surface_and_trunk_buffer() -> None:
    plan = _plan()
    half_width_m = float(plan["trail"]["surface_width_m"]) * 0.5
    required_clearance_m = half_width_m + 2.0

    assert all(
        _distance_to_centerline(tree["position_m"], plan["trail"]["centerline_m"])
        >= required_clearance_m
        for tree in plan["trees"]
    )


def test_every_dressing_item_clears_the_trail_by_its_visual_footprint() -> None:
    plan = _plan()
    half_width_m = float(plan["trail"]["surface_width_m"]) * 0.5
    footprint_clearance_m = {"boulder": 0.75, "fern": 0.15, "grass": 0.10}

    assert all(
        _distance_to_centerline(item["position_m"], plan["trail"]["centerline_m"])
        >= half_width_m + footprint_clearance_m[item["kind"]]
        for item in plan["dressing"]
    )


def test_no_two_trees_overlap_within_one_point_two_five_metres() -> None:
    trees = _plan()["trees"]

    assert min(
        math.dist(first["position_m"][:2], second["position_m"][:2])
        for index, first in enumerate(trees)
        for second in trees[index + 1 :]
    ) >= 1.25


def test_review_output_resolver_accepts_a_new_contained_review_directory(
    tmp_path: Path,
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    output = Path("build/forest-hf/review/hero-v001")

    resolved = _subject().resolve_review_output_dir(repo_root, output)

    assert resolved == repo_root / output
    assert resolved.is_absolute()
    assert not resolved.exists()


def test_default_output_directory_is_inside_the_review_namespace() -> None:
    assert _subject().DEFAULT_OUTPUT_DIR.parts[:3] == ("build", "forest-hf", "review")


def test_public_builder_rejects_an_external_output_before_dcc_work(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    with pytest.raises(ValueError, match=r"contained|review"):
        _subject().build_hero_diorama(
            {},
            tmp_path,
            tmp_path / "outside" / "hero",
            repo_root=repo_root,
        )


def test_review_output_resolver_rejects_an_absolute_path_outside_the_repo(
    tmp_path: Path,
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    with pytest.raises(ValueError, match=r"contained|review"):
        _subject().resolve_review_output_dir(repo_root, tmp_path / "foreign" / "hero")


def test_review_output_resolver_rejects_parent_traversal(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    with pytest.raises(ValueError, match=r"\.\.|contained|review"):
        _subject().resolve_review_output_dir(
            repo_root,
            Path("build/forest-hf/review/../escaped-hero"),
        )


def test_review_output_resolver_rejects_a_linked_path_component(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    review_root = repo_root / "build" / "forest-hf" / "review"
    review_root.mkdir(parents=True)
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    linked = review_root / "linked"
    try:
        os.symlink(foreign, linked, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"directory symlinks unavailable on this host: {exc}")

    with pytest.raises(ValueError, match=r"link|reparse"):
        _subject().resolve_review_output_dir(
            repo_root,
            Path("build/forest-hf/review/linked/hero-v001"),
        )


def test_review_output_resolver_rejects_an_existing_output_directory(
    tmp_path: Path,
) -> None:
    repo_root = tmp_path / "repo"
    output = repo_root / "build" / "forest-hf" / "review" / "hero-v001"
    output.mkdir(parents=True)

    with pytest.raises(FileExistsError, match=r"new|exist"):
        _subject().resolve_review_output_dir(repo_root, output)
