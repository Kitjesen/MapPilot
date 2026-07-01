from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from runtime.adapters.lcm.contracts import (
    THUNDER_FIELD_LCM_CONTRACT_NAME,
)
from runtime.adapters.lcm.endpoint_runner import LCMEndpointEvent, run_endpoint_service
from runtime.runtime_interface import TOPICS

ROOT = Path(__file__).resolve().parents[3]


class _FakeEndpointService:
    last: "_FakeEndpointService | None" = None

    def __init__(
        self,
        *,
        endpoint_contract_name: str,
        on_lingtu_message: Any,
        transport_factory: Any = None,
        transport_strategy: str = "lcm",
    ) -> None:
        del transport_factory
        self.endpoint_contract_name = endpoint_contract_name
        self.on_lingtu_message = on_lingtu_message
        self.transport_strategy = transport_strategy
        self.started = False
        self.start_count = 0
        self.stop_count = 0
        _FakeEndpointService.last = self

    def start(self) -> None:
        self.started = True
        self.start_count += 1

    def stop(self) -> None:
        self.started = False
        self.stop_count += 1

    def health(self) -> dict[str, Any]:
        return {
            "service": "lcm_endpoint_service",
            "endpoint_contract": self.endpoint_contract_name,
            "transport": self.transport_strategy,
            "started": self.started,
            "publish_counts": {},
            "receive_counts": {},
        }


class _FakeSource:
    def __init__(self, name: str) -> None:
        self.name = name
        self.started = False
        self.stopped = False
        self.services: list[Any] = []
        self.events: list[LCMEndpointEvent] = []

    def start(self, service: Any) -> None:
        self.started = True
        self.services.append(service)

    def stop(self) -> None:
        self.stopped = True

    def health(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "started": self.started,
            "stopped": self.stopped,
        }

    def on_lingtu_message(self, event: LCMEndpointEvent) -> None:
        self.events.append(event)


def _args(**overrides: Any) -> argparse.Namespace:
    values = {
        "contract": THUNDER_FIELD_LCM_CONTRACT_NAME,
        "source": "",
        "heartbeat_sec": 0.0,
        "max_heartbeats": None,
        "once": False,
        "describe": False,
        "transport": "lcm",
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_endpoint_runner_describe_does_not_start_transport() -> None:
    result = run_endpoint_service(
        _args(describe=True),
        service_factory=_FakeEndpointService,
    )

    service = _FakeEndpointService.last
    assert service is not None
    assert result["ok"] is True
    assert result["mode"] == "describe"
    assert result["health"]["started"] is False
    assert service.start_count == 0
    assert service.stop_count == 0


def test_endpoint_runner_once_starts_and_stops_service() -> None:
    result = run_endpoint_service(
        _args(once=True),
        service_factory=_FakeEndpointService,
    )

    service = _FakeEndpointService.last
    assert service is not None
    assert result["ok"] is True
    assert result["mode"] == "once"
    assert result["health"]["started"] is True
    assert service.start_count == 1
    assert service.stop_count == 1


def test_endpoint_runner_smoke_source_can_publish_without_lcm_dependency() -> None:
    proc = subprocess.run(
        [
            sys.executable,
            "scripts/deploy/thunder/run_lcm_endpoint_service.py",
            "--transport",
            "local",
            "--source",
            "smoke",
            "--once",
            "--json",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    data = json.loads(proc.stdout)
    assert data["ok"] is True
    assert data["mode"] == "once"
    assert data["transport_strategy"] == "local"
    assert data["source_attached"] is True
    assert data["source_health"]["name"] == "smoke"
    assert data["source_health"]["hardware"] is False
    assert data["source_health"]["published"] == 7
    assert data["health"]["publish_counts"][TOPICS.lidar_scan] == 1
    assert data["health"]["publish_counts"][TOPICS.odometry] == 1


def test_endpoint_runner_module_describes_contract_without_lcm_dependency() -> None:
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT / "src") + os.pathsep + env.get("PYTHONPATH", "")

    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "runtime.adapters.lcm.endpoint_runner",
            "--describe",
            "--json",
        ],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    data = json.loads(proc.stdout)
    assert data["ok"] is True
    assert data["health"]["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME
    assert data["health"]["started"] is False
    assert TOPICS.cmd_vel in data["health"]["lingtu_to_endpoint_channels"]


def test_endpoint_runner_can_attach_multiple_sources(monkeypatch) -> None:
    from runtime.adapters.lcm import endpoint_runner

    sources = [_FakeSource("control"), _FakeSource("localization")]
    monkeypatch.setattr(endpoint_runner, "_load_sources", lambda spec: sources)

    result = endpoint_runner.run_endpoint_service(
        _args(once=True, source="control,localization"),
        service_factory=_FakeEndpointService,
    )

    service = _FakeEndpointService.last
    assert service is not None
    assert result["ok"] is True
    assert result["source_attached"] is True
    assert result["source_health"] == {
        "sources": ["control", "localization"],
        "count": 2,
    }
    assert [item["name"] for item in result["source_healths"]] == [
        "control",
        "localization",
    ]
    assert all(source.started for source in sources)
    assert all(source.stopped for source in sources)
    assert all(source.services == [service] for source in sources)


def test_endpoint_runner_forwards_lingtu_messages_to_all_sources() -> None:
    from runtime.adapters.lcm.endpoint_runner import _notify_sources

    sources = [_FakeSource("control"), _FakeSource("observer")]
    event = LCMEndpointEvent(
        topic=TOPICS.cmd_vel,
        channel="LINGTU_NAV_CMD_VEL",
        schema="twist2d_v1",
        message={"linear_x": 0.2, "angular_z": -0.1},
        ts=123.0,
    )

    _notify_sources(sources, event)

    assert sources[0].events == [event]
    assert sources[1].events == [event]


def test_endpoint_runner_loads_comma_separated_builtin_sources() -> None:
    from runtime.adapters.lcm.endpoint_runner import _load_sources

    sources = _load_sources("smoke,builtin:smoke")

    assert [source.health()["name"] for source in sources] == ["smoke", "smoke"]


def test_endpoint_runner_expands_thunder_field_source_group(monkeypatch) -> None:
    from runtime.adapters.lcm import endpoint_runner

    monkeypatch.delenv("LINGTU_ENDPOINT_JSONL_PATH", raising=False)
    monkeypatch.delenv("LINGTU_THUNDER_JSONL_PATH", raising=False)
    monkeypatch.delenv("LINGTU_ENDPOINT_JSONL_COMMAND", raising=False)

    assert endpoint_runner._expand_source_specs(["thunder_field"]) == ["thunder_brainstem"]


def test_endpoint_runner_expands_thunder_field_source_group_with_jsonl(monkeypatch) -> None:
    from runtime.adapters.lcm import endpoint_runner

    monkeypatch.setenv("LINGTU_ENDPOINT_JSONL_COMMAND", "provider --jsonl")

    assert endpoint_runner._expand_source_specs(["thunder_field"]) == [
        "thunder_brainstem",
        "jsonl",
    ]


def test_endpoint_runner_loads_thunder_field_group_without_jsonl_config(monkeypatch) -> None:
    from runtime.adapters.lcm import endpoint_runner

    monkeypatch.delenv("LINGTU_ENDPOINT_JSONL_PATH", raising=False)
    monkeypatch.delenv("LINGTU_THUNDER_JSONL_PATH", raising=False)
    monkeypatch.delenv("LINGTU_ENDPOINT_JSONL_COMMAND", raising=False)

    sources = endpoint_runner._load_sources("thunder_field")

    assert [source.health()["name"] for source in sources] == ["thunder_brainstem"]
    assert sources[0].health()["hardware"] is True


def test_endpoint_runner_accepts_builtin_thunder_brainstem_source() -> None:
    from runtime.adapters.lcm.endpoint_runner import _load_source

    source = _load_source("thunder_brainstem")

    assert source.health()["name"] == "thunder_brainstem"
    assert source.health()["role"] == "brainstem_command_sink"


def test_deploy_wrapper_describes_contract_without_ros_environment() -> None:
    proc = subprocess.run(
        [
            sys.executable,
            "scripts/deploy/thunder/run_lcm_endpoint_service.py",
            "--describe",
            "--json",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    data = json.loads(proc.stdout)
    assert data["ok"] is True
    assert data["health"]["transport"] == "lcm"
    assert data["health"]["started"] is False
