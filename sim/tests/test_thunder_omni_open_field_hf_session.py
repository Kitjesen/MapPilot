# ruff: noqa: S101
from __future__ import annotations

from pathlib import Path

import yaml

from sim.catalog import CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim"
    / "scenarios"
    / "catalog"
    / "thunder_omni_open_field_hf"
    / "session.yaml"
)


def test_thunder_omni_open_field_hf_compiles_one_visual_runtime_session() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)

    assert resolved.session == yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    assert resolved.visual_plan["world"]["level"] == "/Game/RobotSim/Maps/OpenFieldRuntime"
    assert [robot["instance_id"] for robot in resolved.physics_plan["robots"]] == [
        "thunder_01",
        "cart_01",
    ]
    assert [robot["binding"] for robot in resolved.visual_plan["robots"]] == [
        "RobotVisual:ThunderV4",
        "RobotVisual:OmniCart",
    ]
    assert [controller["instance_id"] for controller in resolved.control_plan["controllers"]] == [
        "thunder_01",
        "cart_01",
    ]


def test_thunder_omni_open_field_hf_namespaces_each_sensor_stream() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)

    streams = resolved.sensor_plan["streams"]
    assert {kind: len(entries) for kind, entries in streams.items()} == {
        "rgb": 2,
        "depth": 2,
        "imu": 2,
        "mid360": 2,
        "truth_odom": 2,
    }
    for entries in streams.values():
        frame_ids = [entry["frame_id"] for entry in entries]
        assert len(frame_ids) == len(set(frame_ids))
        assert {frame_id.split("/", 1)[0] for frame_id in frame_ids} == {
            "thunder_01",
            "cart_01",
        }
