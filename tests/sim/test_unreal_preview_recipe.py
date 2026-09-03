
from __future__ import annotations

import math
from pathlib import Path

import pytest
from sim.runtime.visual.RobotSimUE.Scripts.build_thunderv4_preview import (
    _robot_material_key,
    preview_post_process_profile,
)
from sim.tools.assets.build_unreal_preview_recipe import (
    UnrealPreviewError,
    build_unreal_preview_recipe,
)


@pytest.mark.parametrize(
    ("asset_name", "material_key", "expected"),
    [
        ("base_link_visual", "body_white", "body_shell"),
        ("FR_hip_visual", "body_white", "joint_alloy"),
        ("FR_thigh_visual", "body_white", "limb_alloy"),
        ("FR_foot_visual", "wheel_black", "rubber"),
        ("lidar1_link_visual", "body_white", "sensor"),
        ("camera2_link_visual", "body_white", "sensor"),
    ],
)
def test_preview_material_classifier_separates_production_robot_surfaces(
    asset_name: str,
    material_key: str,
    expected: str,
) -> None:
    assert _robot_material_key({"asset_name": asset_name, "material_key": material_key}) == expected


def test_preview_post_process_preserves_dark_robot_materials() -> None:
    assert preview_post_process_profile() == {
        "auto_exposure_min_brightness": 0.58,
        "auto_exposure_max_brightness": 0.58,
        "auto_exposure_bias": -2.0,
        "bloom_intensity": 0.05,
    }


def test_builds_unreal_component_recipe_from_visual_manifest_and_truth_snapshot() -> None:
    source_hash = "a" * 64
    manifest = {
        "schema": "lingtu.sim.robot-visual-manifest.v1",
        "digest": "b" * 64,
        "binding": "RobotVisual:TracerBot",
        "visuals": [
            {
                "body": "base",
                "body_frame_id": "base",
                "visual_frame_id": "base/visual/base_visual",
                "asset_key": "RobotVisual:TracerBot/base/visual/base_visual",
                "source_mesh": "meshes/base.stl",
                "source_sha256": source_hash,
                "mesh": "base_visual",
                "geom": None,
                "material": "body_white",
                "pos": [0.0, 0.0, 0.0],
                "quat": [1.0, 0.0, 0.0, 0.0],
                "scale": [1.0, 1.0, 1.0],
            }
        ],
    }
    asset_index = {
        "schema": "lingtu.sim.fbx-asset-index.v1",
        "assets": [
            {
                "asset_name": "base_visual",
                "fbx": "base_visual.fbx",
                "source": "base.stl",
                "source_sha256": source_hash,
            }
        ],
    }
    half_sqrt = math.sqrt(0.5)
    snapshot = {
        "schema": "lingtu.sim.truth-snapshot.v1",
        "model_generation": 7,
        "reset_generation": 2,
        "sequence": 11,
        "sim_time_ns": 50_000_000,
        "bodies": [
            {
                "body_id": 1,
                "name": "base",
                "position_m": [1.0, 2.0, 3.0],
                "quaternion_wxyz": [half_sqrt, 0.0, 0.0, half_sqrt],
            }
        ],
    }

    recipe = build_unreal_preview_recipe(
        manifest,
        asset_index,
        snapshot,
        instance_id="tracer_01",
        destination_path="/Game/RobotSim/Robots/TracerBot/Meshes",
    )

    assert recipe["schema"] == "lingtu.sim.unreal-preview-recipe.v1"
    assert recipe["instance_id"] == "tracer_01"
    assert recipe["model_generation"] == 7
    assert recipe["reset_generation"] == 2
    assert recipe["bodies"] == [
        {
            "stable_id": "tracer_01/base",
            "location_cm": [100.0, -200.0, 300.0],
            "quaternion_xyzw": [0.0, 0.0, -half_sqrt, half_sqrt],
        }
    ]
    assert len(recipe["components"]) == 1
    assert recipe["components"][0] == {
        "stable_id": "tracer_01/base/visual/base_visual",
        "body_frame_id": "tracer_01/base",
        "asset_key": "RobotVisual:TracerBot/base/visual/base_visual",
        "asset_name": "base_visual",
        "unreal_asset": "/Game/RobotSim/Robots/TracerBot/Meshes/base_visual.base_visual",
        "material_key": "body_white",
        "link_to_mesh": {
            "location_cm": [0.0, -0.0, 0.0],
            "quaternion_xyzw": [-0.0, 0.0, -0.0, 1.0],
            "scale": [1.0, 1.0, 1.0],
        },
        "location_cm": [100.0, -200.0, 300.0],
        "quaternion_xyzw": [0.0, 0.0, -half_sqrt, half_sqrt],
        "scale": [1.0, 1.0, 1.0],
    }


def test_selects_one_robot_by_stable_id_from_a_multi_robot_snapshot() -> None:
    source_hash = "a" * 64
    manifest = {
        "schema": "lingtu.sim.robot-visual-manifest.v1",
        "digest": "b" * 64,
        "binding": "RobotVisual:TracerBot",
        "visuals": [
            {
                "body": "base",
                "body_frame_id": "base",
                "visual_frame_id": "base/visual/base_visual",
                "asset_key": "RobotVisual:TracerBot/base/visual/base_visual",
                "source_sha256": source_hash,
                "mesh": "base_visual",
                "pos": [0.0, 0.0, 0.0],
                "quat": [1.0, 0.0, 0.0, 0.0],
                "scale": [1.0, 1.0, 1.0],
            }
        ],
    }
    asset_index = {
        "schema": "lingtu.sim.fbx-asset-index.v1",
        "assets": [
            {
                "asset_name": "base_visual",
                "source_sha256": source_hash,
            }
        ],
    }
    snapshot = {
        "schema": "lingtu.sim.truth-snapshot.v1",
        "session_id": "c" * 64,
        "model_generation": 4,
        "reset_generation": 0,
        "bodies": [
            {
                "name": "thunder_01__base",
                "stable_id": "thunder_01/base",
                "instance_id": "thunder_01",
                "frame_id": "base",
                "position_m": [1.0, 2.0, 3.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            {
                "name": "cart_01__base",
                "stable_id": "cart_01/base",
                "instance_id": "cart_01",
                "frame_id": "base",
                "position_m": [9.0, 9.0, 9.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
        ],
    }

    recipe = build_unreal_preview_recipe(
        manifest,
        asset_index,
        snapshot,
        instance_id="thunder_01",
        destination_path="/Game/RobotSim/Meshes",
    )

    assert recipe["session_id"] == "c" * 64
    assert recipe["components"][0]["body_frame_id"] == "thunder_01/base"
    assert recipe["components"][0]["location_cm"] == [100.0, -200.0, 300.0]


def test_component_exposes_pose_invariant_link_to_mesh_transform() -> None:
    source_hash = "a" * 64
    half_sqrt = math.sqrt(0.5)
    manifest = {
        "schema": "lingtu.sim.robot-visual-manifest.v1",
        "digest": "b" * 64,
        "binding": "RobotVisual:TracerBot",
        "visuals": [
            {
                "body": "base",
                "body_frame_id": "base",
                "visual_frame_id": "base/visual/base_visual",
                "asset_key": "RobotVisual:TracerBot/base/visual/base_visual",
                "source_sha256": source_hash,
                "mesh": "base_visual",
                "material": "body_white",
                "pos": [0.25, -0.5, 0.75],
                "quat": [0.5, 0.5, 0.5, 0.5],
                "scale": [1.25, 0.75, 1.5],
            }
        ],
    }
    asset_index = {
        "schema": "lingtu.sim.fbx-asset-index.v1",
        "assets": [{"asset_name": "base_visual", "source_sha256": source_hash}],
    }

    def snapshot(position_m: list[float], quaternion_wxyz: list[float]) -> dict:
        return {
            "schema": "lingtu.sim.truth-snapshot.v1",
            "model_generation": 1,
            "reset_generation": 0,
            "bodies": [
                {
                    "name": "base",
                    "position_m": position_m,
                    "quaternion_wxyz": quaternion_wxyz,
                }
            ],
        }

    nominal_recipe = build_unreal_preview_recipe(
        manifest,
        asset_index,
        snapshot([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        instance_id="tracer_01",
        destination_path="/Game/RobotSim/Meshes",
    )
    live_recipe = build_unreal_preview_recipe(
        manifest,
        asset_index,
        snapshot([1.0, 2.0, 3.0], [half_sqrt, 0.0, 0.0, half_sqrt]),
        instance_id="tracer_01",
        destination_path="/Game/RobotSim/Meshes",
    )

    expected_link_to_mesh = {
        "location_cm": [25.0, 50.0, 75.0],
        "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
        "scale": [1.25, 0.75, 1.5],
    }
    nominal_component = nominal_recipe["components"][0]
    live_component = live_recipe["components"][0]
    assert nominal_component.get("link_to_mesh") == expected_link_to_mesh
    assert live_component.get("link_to_mesh") == expected_link_to_mesh
    assert nominal_component["link_to_mesh"] == live_component["link_to_mesh"]

    body_world = live_recipe["bodies"][0]
    link_to_mesh = live_component["link_to_mesh"]
    bx, by, bz, bw = body_world["quaternion_xyzw"]
    lx, ly, lz, lw = link_to_mesh["quaternion_xyzw"]
    ox, oy, oz = link_to_mesh["location_cm"]
    tx = 2.0 * (by * oz - bz * oy)
    ty = 2.0 * (bz * ox - bx * oz)
    tz = 2.0 * (bx * oy - by * ox)
    rotated_offset = [
        ox + bw * tx + (by * tz - bz * ty),
        oy + bw * ty + (bz * tx - bx * tz),
        oz + bw * tz + (bx * ty - by * tx),
    ]
    composed_location = [
        body_world["location_cm"][axis] + rotated_offset[axis] for axis in range(3)
    ]
    composed_quaternion = [
        bw * lx + bx * lw + by * lz - bz * ly,
        bw * ly - bx * lz + by * lw + bz * lx,
        bw * lz + bx * ly - by * lx + bz * lw,
        bw * lw - bx * lx - by * ly - bz * lz,
    ]

    assert composed_location == pytest.approx(live_component["location_cm"])
    assert composed_quaternion == pytest.approx(live_component["quaternion_xyzw"])
