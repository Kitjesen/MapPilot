# ruff: noqa: S101
from __future__ import annotations

import json
from pathlib import Path

import yaml

from sim.catalog import CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "omni_cart_factory_park_hf"
    / "session.yaml"
)


def test_omni_cart_factory_park_session_resolves_as_an_unreal_sensor_session() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)

    assert resolved.session == yaml.safe_load(SESSION.read_text(encoding="utf-8"))


def test_omni_cart_factory_park_session_exposes_the_navigation_sensor_contract() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)

    streams = resolved.sensor_plan["streams"]
    assert set(streams) == {"rgb", "depth", "imu", "mid360", "truth_odom"}
    assert {kind: len(entries) for kind, entries in streams.items()} == {
        "rgb": 1,
        "depth": 1,
        "imu": 1,
        "mid360": 1,
        "truth_odom": 1,
    }
    assert {
        field: streams["rgb"][0][field]
        for field in ("width", "height", "rate_hz")
    } == {
        "width": 640,
        "height": 480,
        "rate_hz": 30,
    }
    assert streams["rgb"][0]["frame_id"] == "cart_01/front_camera"
    assert streams["rgb"][0]["parent_frame_id"] == "cart_01/base_link"
    assert streams["rgb"][0]["extrinsic"] == {
        "position_m": [0.38, 0.0, 0.34],
        "quaternion_wxyz": [0.5, -0.5, 0.5, -0.5],
    }
    assert streams["depth"][0]["width"] == 640
    assert streams["depth"][0]["height"] == 480
    assert streams["depth"][0]["rate_hz"] == 30
    assert streams["mid360"][0]["raycast_frame_stable_id"] == "cart_01/lidar_link"
    assert streams["truth_odom"][0]["estimator_input"] is False


def test_omni_cart_factory_park_bundle_is_driven_only_by_omni_cart_plans(
    tmp_path: Path,
) -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle_dir = resolved.write_bundle(tmp_path / "bundle")

    physics = json.loads((bundle_dir / "physics.plan.json").read_text(encoding="utf-8"))
    visual = json.loads((bundle_dir / "visual.plan.json").read_text(encoding="utf-8"))
    sensors = json.loads((bundle_dir / "sensor.plan.json").read_text(encoding="utf-8"))
    control = json.loads((bundle_dir / "control.plan.json").read_text(encoding="utf-8"))
    projection_path = REPO_ROOT / visual["robots"][0]["projection"]["path"]
    projection = json.loads(projection_path.read_text(encoding="utf-8"))

    assert [(robot["instance_id"], robot["package"]["id"]) for robot in physics["robots"]] == [
        ("cart_01", "omni_cart")
    ]
    assert physics["robots"][0]["model"]["attach_root"] == "base_link"
    assert [(robot["instance_id"], robot["binding"]) for robot in visual["robots"]] == [
        ("cart_01", "RobotVisual:OmniCart")
    ]
    assert len(projection["components"]) == 5
    assert control["controllers"] == []
    assert control["command_channels"] == []

    runtime_module_plans = json.dumps(
        [physics, visual, sensors, control],
        ensure_ascii=False,
        sort_keys=True,
    ).lower()
    assert "thunder" not in runtime_module_plans
    assert "quadruped" not in runtime_module_plans
