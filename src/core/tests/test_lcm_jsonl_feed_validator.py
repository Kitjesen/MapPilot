from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from tools.validate.validate_lcm_jsonl_feed import validate_feed

from compat.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, endpoint_contract
from compat.lcm.endpoint_codec import dumps_endpoint_message
from core.msgs.geometry import Pose, Twist
from core.msgs.nav import Odometry
from core.runtime_interface import TOPICS

REPO_ROOT = Path(__file__).resolve().parents[3]


def _write_jsonl(path: Path, records: list[Any]) -> None:
    lines = [
        item if isinstance(item, str) else json.dumps(item, sort_keys=True)
        for item in records
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_validate_lcm_jsonl_feed_accepts_required_field_inputs(tmp_path) -> None:
    feed = tmp_path / "field-inputs.jsonl"
    _write_jsonl(
        feed,
        [
            "# operator comment",
            {"sleep_sec": 0.0},
            {
                "topic": TOPICS.lidar_scan,
                "message": {
                    "points": [[0.0, 0.0, 0.0, 1.0]],
                    "frame_id": "livox_frame",
                    "ts": 1.0,
                },
            },
            {
                "topic": TOPICS.imu,
                "message": {
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
                    "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
                    "frame_id": "lidar_link",
                    "ts": 1.0,
                },
            },
            {
                "topic": TOPICS.odometry,
                "message": {
                    "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.0}},
                    "twist": {"linear": {"x": 0.1}},
                    "frame_id": "odom",
                    "child_frame_id": "base_link",
                    "ts": 1.0,
                },
            },
        ],
    )

    report = validate_feed(feed, require_field_inputs=True)

    assert report["ok"] is True
    assert report["checked_records"] == 3
    assert report["skipped_records"] == 2
    assert report["topics"][TOPICS.lidar_scan] == 1
    assert report["frames"][TOPICS.lidar_scan]["livox_frame"] == 1
    assert report["frames"][f"{TOPICS.odometry}:child_frame_id"]["base_link"] == 1


def test_validate_lcm_jsonl_feed_rejects_invalid_frame(tmp_path) -> None:
    feed = tmp_path / "bad-frame.jsonl"
    _write_jsonl(
        feed,
        [
            {
                "topic": TOPICS.lidar_scan,
                "message": {
                    "points": [[0.0, 0.0, 0.0]],
                    "frame_id": "map",
                },
            }
        ],
    )

    report = validate_feed(feed)

    assert report["ok"] is False
    assert any("frame_id 'map' not allowed" in blocker for blocker in report["blockers"])


def test_validate_lcm_jsonl_feed_requires_field_inputs(tmp_path) -> None:
    feed = tmp_path / "missing-imu.jsonl"
    _write_jsonl(
        feed,
        [
            {
                "topic": TOPICS.lidar_scan,
                "message": {
                    "points": [[0.0, 0.0, 0.0]],
                    "frame_id": "lidar_link",
                },
            }
        ],
    )

    report = validate_feed(feed, require_field_inputs=True)

    assert report["ok"] is False
    assert f"missing required endpoint input topic: {TOPICS.imu}" in report["blockers"]


def test_validate_lcm_jsonl_feed_accepts_endpoint_envelopes(tmp_path) -> None:
    feed = tmp_path / "envelope.jsonl"
    contract = endpoint_contract(THUNDER_FIELD_LCM_CONTRACT_NAME)
    binding = contract.binding_for_topic(TOPICS.odometry)
    odometry = Odometry(
        pose=Pose(3.0, 4.0, 0.0),
        twist=Twist(),
        frame_id="odom",
        child_frame_id="body",
    )
    feed.write_text(
        dumps_endpoint_message(binding, odometry).decode("utf-8") + "\n",
        encoding="utf-8",
    )

    report = validate_feed(feed)

    assert report["ok"] is True
    assert report["topics"] == {TOPICS.odometry: 1}


def test_validate_lcm_jsonl_feed_cli_json_exit_code(tmp_path) -> None:
    feed = tmp_path / "missing-required.jsonl"
    _write_jsonl(
        feed,
        [
            {
                "topic": TOPICS.lidar_scan,
                "message": {"points": [[0.0, 0.0, 0.0]], "frame_id": "lidar_link"},
            }
        ],
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/validate/validate_lcm_jsonl_feed.py",
            str(feed),
            "--require-field-inputs",
            "--json",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 1
    payload = json.loads(result.stdout)
    assert payload["ok"] is False
    assert f"missing required endpoint input topic: {TOPICS.imu}" in payload["blockers"]
