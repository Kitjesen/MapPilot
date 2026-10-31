"""Actual-model pose tests for the offline body-envelope experiment."""

import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from tools.bench.body_model import BodyModel

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "web/public/assets/robots/go2/urdf/go2_mid360_flatpack.urdf"


def test_imports_every_collision_and_mount():
    body = BodyModel(URDF, [])
    geometry = body.geometry([0, 0, 0.35, 0])
    assert body.report["source_collision_count"] == 34
    assert body.report["imported_collision_count"] == len(geometry) == 34
    assert len(body.report["joint_angles"]) == 12
    assert "livox_frame" in {item["link"] for item in geometry}
    assert "mid360_lid" in {item["link"] for item in geometry}


def test_wall_collision_reports_the_environment_pair():
    body = BodyModel(URDF, [{"name": "front_wall", "center": [0.35, 0, 0.4],
                             "half_size": [0.05, 1, 0.4]}])
    distance, link, obstacle = body.clearance([0, 0, 0.35, 0])
    assert distance < 0
    assert link
    assert obstacle == "front_wall"


def test_corridor_passes_aligned_and_rejects_crosswise_pose():
    body = BodyModel(URDF, [
        {"name": "left", "center": [0, 0.35, 0.4], "half_size": [2, 0.05, 0.4]},
        {"name": "right", "center": [0, -0.35, 0.4], "half_size": [2, 0.05, 0.4]},
    ])
    # Inner wall Y is .300 m; the nearest foot sphere reaches .164 m.
    assert body.clearance([0, 0, 0.35, 0])[0] == pytest.approx(0.136, abs=1e-6)
    assert body.clearance([0, 0, 0.35, math.pi / 2])[0] < 0


def test_raised_payload_is_checked_above_the_main_body():
    body = BodyModel(URDF, [{"name": "overhead", "center": [0.24, 0, 0.57],
                             "half_size": [0.04, 0.04, 0.02]}])
    distance, link, obstacle = body.clearance([0, 0, 0.35, 0])
    assert distance < 0
    assert link == "livox_frame"
    assert obstacle == "overhead"


def test_pose_moves_robot_and_preserves_obstacles():
    body = BodyModel(URDF, [{"name": "wall", "center": [0.8, 0, 0.4],
                             "half_size": [0.05, 1, 0.4]}])
    at_origin = body.clearance([0, 0, 0.35, 0])[0]
    moved = body.clearance([0.2, 0, 0.35, 0])[0]
    assert moved == pytest.approx(at_origin - 0.2, abs=1e-6)
    original = body.geometry([0, 0, 0.35, 0])
    translated = body.geometry([0.2, -0.1, 0.5, 0])
    assert np.allclose(np.array(translated[0]["center"]) - original[0]["center"], [0.2, -0.1, 0.15])


def test_simple_primitive_urdf_without_joints(tmp_path):
    urdf = tmp_path / "proxy.urdf"
    urdf.write_text('<robot name="proxy"><link name="base"><collision>'
                    '<geometry><cylinder radius="0.44" length="0.35" />'
                    '</geometry></collision></link></robot>')
    body = BodyModel(urdf, [{"name": "wall", "center": [0.5, 0, 0],
                             "half_size": [0.05, 1, 1]}])
    assert body.report["joint_angles"] == {}
    assert body.clearance([0, 0, 0, 0])[0] == pytest.approx(0.01, abs=1e-6)
    geom = body.geometry([0, 0, 0, 0])[0]
    assert geom["type"] == "cylinder"
    assert geom["size"] == pytest.approx([0.44, 0.175, 0])


def test_collision_mesh_is_never_silently_dropped(tmp_path):
    urdf = tmp_path / "mesh.urdf"
    urdf.write_text('<robot name="mesh"><link name="base"><collision>'
                    '<geometry><mesh filename="missing.stl" /></geometry>'
                    '</collision></link></robot>')
    with pytest.raises(ValueError, match="not discarded"):
        BodyModel(urdf, [])


def test_finite_door_jamb_has_proven_positive_gap(tmp_path):
    urdf = tmp_path / "twin.urdf"
    urdf.write_text('<robot name="twin"><link name="base">'
                    '<collision><origin xyz="0.19 0 0" /><geometry>'
                    '<cylinder radius="0.25" length="0.35" /></geometry></collision>'
                    '<collision><origin xyz="-0.19 0 0" /><geometry>'
                    '<cylinder radius="0.25" length="0.35" /></geometry></collision>'
                    '</link></robot>')
    obstacle = {"name": "jamb", "center": [0, -1.3, 0.75], "half_size": [0.1, 1, 0.75]}
    body = BodyModel(urdf, [obstacle])
    distance, _, name = body.clearance([-0.5, 0, 0.35, 0], cutoff=0.5)
    exact_gap = math.hypot(0.31 - 0.1, 0.3) - 0.25
    assert distance == pytest.approx(0.05, abs=1e-6)
    assert 0 < distance <= exact_gap
    assert name == "jamb"
    # Moving within .02 m of the jamb fails the parent's .02 m margin.
    assert body.clearance([-0.19, -0.04, 0.35, 0])[0] < 0.02
    # A larger lateral move really overlaps; the AABB stage cannot pass it.
    assert body.clearance([0, -0.1, 0.35, 0])[0] < 0
