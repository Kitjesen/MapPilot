# ruff: noqa: S101

"""Review-only hero patrol contracts for Forest_HF Blender authoring."""

from __future__ import annotations

import hashlib
import json
import math
from itertools import pairwise
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from sim.tools.worlds.forest_hf import blender_author as author
from sim.tools.worlds.forest_hf import blender_visuals as visuals

_RUNTIME_INDEX = Path(
    "build/unreal-assets/thunderv4-v103-runtime-60k-fbx/asset-index.json"
)


def _layout() -> dict[str, Any]:
    return {
        "extent_m": [200.0, 160.0],
        "trail": {
            "surface_width_m": 3.2,
            "shoulder_blend_m": 0.8,
            "centerline_m": [[-80.0, -20.0], [-20.0, 10.0], [70.0, 38.0]],
        },
    }


def _first_turn_distance_m(
    points: list[list[float]],
    *,
    minimum_turn_deg: float,
) -> float:
    accumulated = 0.0
    for previous, corner, following in zip(points, points[1:], points[2:]):
        incoming = (corner[0] - previous[0], corner[1] - previous[1])
        outgoing = (following[0] - corner[0], following[1] - corner[1])
        incoming_length = math.hypot(*incoming)
        outgoing_length = math.hypot(*outgoing)
        cosine = (
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
        ) / (incoming_length * outgoing_length)
        turn_deg = math.degrees(math.acos(min(1.0, max(-1.0, cosine))))
        accumulated += incoming_length
        if turn_deg >= minimum_turn_deg:
            return accumulated
    raise AssertionError(f"fixture route has no turn of at least {minimum_turn_deg} degrees")


def _project_distance_along_route_m(
    point: list[float],
    centerline: list[list[float]],
) -> float:
    best_distance = float("inf")
    best_along = 0.0
    accumulated = 0.0
    for start, end in pairwise(centerline):
        dx, dy = end[0] - start[0], end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        amount = min(
            1.0,
            max(
                0.0,
                ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                / segment_length**2,
            ),
        )
        closest = (start[0] + amount * dx, start[1] + amount * dy)
        distance = math.dist(point[:2], closest)
        if distance < best_distance:
            best_distance = distance
            best_along = accumulated + amount * segment_length
        accumulated += segment_length
    return best_along


def _write_robot_index(root: Path, *, asset_count: int = 21) -> Path:
    assets = []
    for index in range(asset_count):
        payload = f"fbx-{index:02d}".encode()
        filename = f"part_{index:02d}.fbx"
        (root / filename).write_bytes(payload)
        assets.append(
            {
                "asset_name": f"part_{index:02d}_visual",
                "fbx": filename,
                "fbx_bytes": len(payload),
                "fbx_sha256": hashlib.sha256(payload).hexdigest(),
            }
        )
    path = root / "asset-index.json"
    path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.fbx-asset-index.v1",
                "source_axis": "mujoco_rh_z_up_m",
                "fbx_axis": {"forward": "-Y", "up": "Z", "unit": "m"},
                "assets": assets,
            }
        ),
        encoding="utf-8",
    )
    return path


def test_hero_patrol_camera_uses_the_declared_route_frame() -> None:
    layout = _layout()
    composition = visuals.hero_patrol_composition(layout)
    camera = visuals.review_camera_specs(layout)["hero_patrol"]

    assert camera["location_m"] == pytest.approx(composition["camera_location_m"])
    assert camera["target_m"] == pytest.approx(composition["camera_target_m"])
    assert camera["lens_mm"] == 55.0
    assert camera["projection"] == "perspective"


def test_hero_patrol_composition_exposes_camera_target_and_robot_pose() -> None:
    layout = _layout()

    composition = visuals.hero_patrol_composition(layout)
    camera = visuals.review_camera_specs(layout)["hero_patrol"]

    assert composition["camera_location_m"] == pytest.approx(camera["location_m"])
    assert composition["camera_target_m"] == pytest.approx(camera["target_m"])
    assert len(composition["robot_location_m"]) == 3
    assert math.isfinite(composition["robot_yaw_deg"])
    assert composition["distance_along_route_m"] == pytest.approx(
        _project_distance_along_route_m(
            composition["camera_location_m"], layout["trail"]["centerline_m"]
        )
    )


def test_hero_patrol_composition_straddles_first_readable_production_turn() -> None:
    layout = author.load_forest_recipe(author.DEFAULT_RECIPE)
    centerline = layout["trail"]["centerline_m"]
    corner_distance_m = _first_turn_distance_m(centerline, minimum_turn_deg=8.0)

    composition = visuals.hero_patrol_composition(layout)
    camera_distance_m = _project_distance_along_route_m(
        composition["camera_location_m"], centerline
    )
    robot_distance_m = _project_distance_along_route_m(
        composition["robot_location_m"], centerline
    )
    target_distance_m = _project_distance_along_route_m(
        composition["camera_target_m"], centerline
    )

    assert camera_distance_m < robot_distance_m < corner_distance_m < target_distance_m
    assert camera_distance_m == pytest.approx(corner_distance_m - 39.0, abs=1.5)
    assert robot_distance_m == pytest.approx(corner_distance_m - 14.0, abs=1.5)
    assert target_distance_m == pytest.approx(corner_distance_m + 21.0, abs=1.5)
    assert 50.0 <= math.dist(
        composition["camera_location_m"], composition["camera_target_m"]
    ) <= 65.0


def test_review_robot_grounding_offset_places_lowest_vertex_at_clearance() -> None:
    offset_m = visuals.review_robot_grounding_offset(
        min_world_z=-0.237,
        trail_z=1.425,
        clearance=0.01,
    )

    assert offset_m == pytest.approx(1.672, abs=1e-12)


def test_review_actor_collection_is_excluded_from_scene_exports(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeObject:
        def __init__(self, *, selected: bool, hidden: bool) -> None:
            self.selected = selected
            self.hidden = hidden

        def hide_set(self, hidden: bool) -> None:
            self.hidden = hidden

        def select_set(self, selected: bool) -> None:
            self.selected = selected

    scene_names = ("Terrain", "Trail", "Trees", "Dressing", "Lighting", "Cameras")
    scene_objects = [FakeObject(selected=False, hidden=True) for _name in scene_names]
    review_robot = FakeObject(selected=True, hidden=False)
    collections = {
        **{
            name: SimpleNamespace(all_objects=[obj])
            for name, obj in zip(scene_names, scene_objects)
        },
        "ReviewActors": SimpleNamespace(all_objects=[review_robot]),
    }
    all_objects = [*scene_objects, review_robot]
    fake_bpy = SimpleNamespace(
        ops=SimpleNamespace(
            object=SimpleNamespace(
                select_all=lambda **_kwargs: [obj.select_set(False) for obj in all_objects]
            )
        )
    )
    monkeypatch.setattr(author, "bpy", fake_bpy)

    author._select_scene_export_objects(collections)

    assert all(item.selected and not item.hidden for item in scene_objects)
    assert review_robot.selected is False
    assert review_robot.hidden is True


def test_runtime_robot_index_loads_exactly_twenty_one_review_only_assets() -> None:
    contract = author.load_review_robot_asset_index(_RUNTIME_INDEX)

    assert len(contract["assets"]) == 21
    assert all(item["classification"] == "VisualOnly" for item in contract["assets"])
    assert all(item["collision_profile"] == "NoCollision" for item in contract["assets"])
    assert all(item["simulate_physics"] is False for item in contract["assets"])


def test_review_robot_index_rejects_any_asset_count_other_than_twenty_one(
    tmp_path: Path,
) -> None:
    index_path = _write_robot_index(tmp_path, asset_count=20)

    with pytest.raises(ValueError, match="exactly 21"):
        author.load_review_robot_asset_index(index_path)


def test_review_robot_index_rejects_declared_byte_count_mismatch(tmp_path: Path) -> None:
    index_path = _write_robot_index(tmp_path)
    document = json.loads(index_path.read_text(encoding="utf-8"))
    document["assets"][0]["fbx_bytes"] += 1
    index_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="bytes"):
        author.load_review_robot_asset_index(index_path)


def test_review_robot_index_rejects_declared_sha256_mismatch(tmp_path: Path) -> None:
    index_path = _write_robot_index(tmp_path)
    document = json.loads(index_path.read_text(encoding="utf-8"))
    document["assets"][0]["fbx_sha256"] = "0" * 64
    index_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="sha256"):
        author.load_review_robot_asset_index(index_path)


def test_review_robot_asset_is_opt_in_at_the_cli_boundary(tmp_path: Path) -> None:
    default_args = author.parse_cli_args(["--repo-root", str(tmp_path), "--validate-only"])
    opted_in_args = author.parse_cli_args(
        [
            "--repo-root",
            str(tmp_path),
            "--review-robot-asset-index",
            str(_RUNTIME_INDEX.resolve()),
            "--validate-only",
        ]
    )

    assert default_args.review_robot_asset_index is None
    assert opted_in_args.review_robot_asset_index == _RUNTIME_INDEX.resolve()


def test_single_hero_render_plan_keeps_an_independent_output() -> None:
    cameras = visuals.review_camera_specs(_layout())

    targets = author.review_render_targets(cameras, selected_camera="hero_patrol")

    assert targets == [
        {
            "camera_id": "hero_patrol",
            "artifact_role": "hero_patrol_preview",
            "filename": "forest_hf.hero-patrol.png",
        }
    ]


def test_review_camera_cli_selects_only_the_hero_render(tmp_path: Path) -> None:
    args = author.parse_cli_args(
        [
            "--repo-root",
            str(tmp_path),
            "--review-camera",
            "hero_patrol",
            "--validate-only",
        ]
    )

    assert args.review_camera == "hero_patrol"
