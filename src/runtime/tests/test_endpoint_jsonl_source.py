from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from runtime.endpoints.dds.contracts import THUNDER_DDS_CONTRACT_NAME, endpoint_contract
from runtime.endpoints.dds.endpoint_runner import _load_source
from runtime.adapters.endpoint_sources.endpoint_codec import dumps_endpoint_message
from runtime.adapters.endpoint_sources.jsonl import JsonlEndpointSource, create
from runtime.msgs.geometry import Pose, Twist
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.runtime_interface import TOPICS


class _FakeEndpointService:
    def __init__(self) -> None:
        self.contract = endpoint_contract(THUNDER_DDS_CONTRACT_NAME)
        self.published: list[tuple[str, Any]] = []

    def publish_to_lingtu(self, topic: str, msg: Any) -> None:
        self.published.append((topic, msg))


def _write_jsonl(path: Path, records: list[Any]) -> None:
    lines = [
        item if isinstance(item, str) else json.dumps(item, sort_keys=True)
        for item in records
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_jsonl_source_publishes_sensor_and_localization_records(tmp_path) -> None:
    replay = tmp_path / "endpoint.jsonl"
    _write_jsonl(
        replay,
        [
            {
                "topic": TOPICS.lidar_scan,
                "message": {
                    "points": [[0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.2, 0.8]],
                    "frame_id": "lidar_link",
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
                    "pose": {
                        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "twist": {
                        "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
                    },
                    "frame_id": "odom",
                    "child_frame_id": "body",
                    "ts": 1.0,
                },
            },
            {
                "topic": TOPICS.localization_health,
                "message": {"state": "TRACKING", "quality": 0.9},
            },
            {
                "topic": TOPICS.localization_quality,
                "value": 0.9,
            },
        ],
    )
    service = _FakeEndpointService()
    source = JsonlEndpointSource(path=replay)

    source.start(service)

    assert [topic for topic, _ in service.published] == [
        TOPICS.lidar_scan,
        TOPICS.imu,
        TOPICS.odometry,
        TOPICS.localization_health,
        TOPICS.localization_quality,
    ]
    assert isinstance(service.published[0][1], PointCloud2)
    assert service.published[0][1].num_points == 2
    assert isinstance(service.published[1][1], Imu)
    assert isinstance(service.published[2][1], Odometry)
    assert service.published[3][1]["state"] == "TRACKING"
    assert service.published[4][1] == 0.9
    assert source.health()["published"][TOPICS.odometry] == 1


def test_jsonl_source_accepts_endpoint_envelopes(tmp_path) -> None:
    replay = tmp_path / "endpoint-envelope.jsonl"
    contract = endpoint_contract(THUNDER_DDS_CONTRACT_NAME)
    binding = contract.binding_for_topic(TOPICS.odometry)
    odometry = Odometry(
        pose=Pose(3.0, 4.0, 0.0),
        twist=Twist(),
        ts=5.0,
        frame_id="odom",
        child_frame_id="body",
    )
    replay.write_text(
        dumps_endpoint_message(binding, odometry).decode("utf-8") + "\n",
        encoding="utf-8",
    )
    service = _FakeEndpointService()
    source = JsonlEndpointSource(path=replay)

    source.start(service)

    assert len(service.published) == 1
    assert service.published[0][0] == TOPICS.odometry
    assert service.published[0][1].x == 3.0
    assert service.published[0][1].y == 4.0


def test_jsonl_source_accepts_utf8_bom_files(tmp_path) -> None:
    replay = tmp_path / "bom.jsonl"
    replay.write_text(
        f'\ufeff{{"topic": "{TOPICS.localization_quality}", "value": 0.8}}\n',
        encoding="utf-8",
    )
    service = _FakeEndpointService()
    source = JsonlEndpointSource(path=replay)

    source.start(service)

    assert service.published == [(TOPICS.localization_quality, 0.8)]
    assert source.health()["errors"] == {}


def test_jsonl_source_factory_reads_env(monkeypatch, tmp_path) -> None:
    replay = tmp_path / "endpoint.jsonl"
    replay.write_text("", encoding="utf-8")
    monkeypatch.setenv("LINGTU_ENDPOINT_JSONL_PATH", str(replay))
    monkeypatch.setenv("LINGTU_ENDPOINT_JSONL_RATE_HZ", "2.5")

    source = create()

    assert source.health()["name"] == "jsonl"
    assert source.health()["path"] == str(replay)
    assert source.health()["rate_hz"] == 2.5


def test_endpoint_runner_accepts_builtin_jsonl_source_alias(monkeypatch, tmp_path) -> None:
    replay = tmp_path / "endpoint.jsonl"
    replay.write_text("", encoding="utf-8")
    monkeypatch.setenv("LINGTU_ENDPOINT_JSONL_PATH", str(replay))

    source = _load_source("jsonl")

    assert source.health()["name"] == "jsonl"
    assert source.health()["role"] == "sensor_localization_jsonl"
