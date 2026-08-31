"""Behavior tests for terminal zero confirmation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from lingtu.sim.stop import (
    MOTION_STOP_SCHEMA,
    PROCESS_LAUNCH_ID_ENV,
    SimStopEvidenceError,
    load_motion_stop_evidence,
    process_launch_id,
    publish_motion_stop_evidence,
)


def _payload(launch_id: str = "launch-1") -> dict[str, Any]:
    return {
        "schema": MOTION_STOP_SCHEMA,
        "product_session_id": "a" * 32,
        "product": "teleop_avoid",
        "process": "driver_bridge",
        "outcome": "zero_applied",
        "launch_id": launch_id,
        "bridge_boot_id": "bridge-1",
        "controller_boot_id": "controller-1",
        "bridge_command_seq": 1,
        "applied_step_seq": 2,
        "command_kind": "deactivate_zero",
        "walk_x": 0.0,
        "walk_y": 0.0,
        "walk_z": 0.0,
        "terminal_ack": True,
    }


def _load(path: Path, launch_id: str = "launch-1") -> dict[str, Any]:
    return dict(
        load_motion_stop_evidence(
            session_root=path.parent,
            target=path.name,
            product_session_id="a" * 32,
            product="teleop_avoid",
            process="driver_bridge",
            launch_id=launch_id,
        )
    )


def test_motion_stop_evidence_accepts_zero_ack_and_normal_json(tmp_path: Path) -> None:
    evidence = tmp_path / "stop.json"
    payload = _payload()
    evidence.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    assert _load(evidence) == payload


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("outcome", "process_exited"),
        ("command_kind", "nav_cmd"),
        ("terminal_ack", False),
        ("bridge_command_seq", 0),
        ("walk_x", 0.1),
    ),
)
def test_motion_stop_evidence_rejects_nonzero_or_unconfirmed_ack(
    tmp_path: Path,
    field: str,
    value: Any,
) -> None:
    evidence = tmp_path / "stop.json"
    payload = _payload()
    payload[field] = value
    evidence.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(SimStopEvidenceError):
        _load(evidence)


def test_motion_stop_evidence_is_bound_to_session_and_launch(tmp_path: Path) -> None:
    evidence = tmp_path / "stop.json"
    evidence.write_text(json.dumps(_payload("old-launch")), encoding="utf-8")

    with pytest.raises(SimStopEvidenceError):
        _load(evidence, "new-launch")


def test_publish_motion_stop_evidence_is_atomic_and_launch_bound(tmp_path: Path) -> None:
    payload = _payload()
    payload.pop("launch_id")

    published = publish_motion_stop_evidence(
        session_root=tmp_path,
        target="stop.json",
        payload=payload,
        environment={PROCESS_LAUNCH_ID_ENV: "launch-1"},
    )

    assert published == _payload()
    assert _load(tmp_path / "stop.json") == published
    assert not tuple(tmp_path.glob(".*.tmp"))


def test_process_launch_id_comes_from_environment() -> None:
    assert process_launch_id({PROCESS_LAUNCH_ID_ENV: "launch-1"}) == "launch-1"
    with pytest.raises(SimStopEvidenceError):
        process_launch_id({})
