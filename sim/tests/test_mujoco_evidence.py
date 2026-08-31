"""Behavior tests for MuJoCo acceptance evidence."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from lingtu.sim.stop import PROCESS_LAUNCH_ID_ENV
from sim.scripts.mujoco.evidence import (
    FEEDER_STATUS_FILENAME,
    FEEDER_STATUS_SCHEMA,
    SimFeederStatusError,
    load_feeder_status,
    publish_feeder_status,
)


def test_feeder_status_accepts_normal_json_and_binds_the_launch(tmp_path: Path) -> None:
    payload = {
        "schema": FEEDER_STATUS_SCHEMA,
        "product": "teleop_avoid",
        "product_session_id": "a" * 32,
        "process": "mujoco_feeder",
        "state": "stopped",
        "sequence": 2,
        "updated_wall_ns": 1,
        "window_s": 4.0,
        "streams": {
            "imu": {
                "expected_hz": 100.0,
                "scheduled_count": 400,
                "published_count": 400,
                "dropped_count": 0,
                "actual_hz": 100.0,
                "max_schedule_lateness_ms": 1.0,
            }
        },
    }
    published = publish_feeder_status(
        session_root=tmp_path,
        payload=payload,
        environment={PROCESS_LAUNCH_ID_ENV: "launch-1"},
    )
    path = tmp_path / FEEDER_STATUS_FILENAME
    path.write_text(json.dumps(published, indent=2), encoding="utf-8")

    loaded = load_feeder_status(
        session_root=tmp_path,
        product="teleop_avoid",
        product_session_id="a" * 32,
        process="mujoco_feeder",
        launch_id="launch-1",
    )
    assert loaded == published

    with pytest.raises(SimFeederStatusError):
        load_feeder_status(
            session_root=tmp_path,
            product="teleop_avoid",
            product_session_id="a" * 32,
            process="mujoco_feeder",
            launch_id="stale-launch",
        )


def test_running_feeder_status_allows_inflight_sensor_records(tmp_path: Path) -> None:
    payload = {
        "schema": FEEDER_STATUS_SCHEMA,
        "product": "teleop_avoid",
        "product_session_id": "a" * 32,
        "process": "mujoco_feeder",
        "state": "running",
        "sequence": 3,
        "updated_wall_ns": 1,
        "window_s": 4.0,
        "streams": {
            "imu": {
                "expected_hz": 200.0,
                "scheduled_count": 802,
                "published_count": 800,
                "dropped_count": 0,
                "actual_hz": 200.0,
                "max_schedule_lateness_ms": 1.0,
            }
        },
    }

    publish_feeder_status(
        session_root=tmp_path,
        payload=payload,
        environment={PROCESS_LAUNCH_ID_ENV: "launch-1"},
    )

    payload["state"] = "stopped"
    with pytest.raises(SimFeederStatusError, match="counts are inconsistent"):
        publish_feeder_status(
            session_root=tmp_path,
            payload=payload,
            environment={PROCESS_LAUNCH_ID_ENV: "launch-1"},
        )
