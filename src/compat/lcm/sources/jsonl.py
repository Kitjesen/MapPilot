"""JSONL endpoint source for no-ROS localization and sensor feeds."""

from __future__ import annotations

import json
import logging
import os
import shlex
import subprocess
import threading
import time
from collections import Counter
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from core.msgs.geometry import PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.nav import Path as NavPath
from core.msgs.sensor import Imu, PointCloud2

from ..contracts import LCM_PAYLOAD_FORMAT, LCMEndpointBinding
from ..endpoint_codec import loads_endpoint_message
from ..endpoint_service import LCMEndpointEvent, LCMEndpointService

logger = logging.getLogger(__name__)


class JsonlEndpointSource:
    """Publish normalized endpoint records from JSONL files or process output."""

    name = "jsonl"

    def __init__(
        self,
        *,
        path: str | Path | None = None,
        command: str | Sequence[str] | None = None,
        loop: bool = False,
        rate_hz: float = 0.0,
        shell: bool = False,
        encoding: str = "utf-8-sig",
    ) -> None:
        """Create a JSONL endpoint source."""

        self._path = Path(path).expanduser() if path else None
        self._command = command
        self._loop = bool(loop)
        self._rate_hz = max(0.0, float(rate_hz))
        self._shell = bool(shell)
        self._encoding = str(encoding or "utf-8-sig")
        self._service: LCMEndpointService | None = None
        self._process: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._started = False
        self._published: Counter[str] = Counter()
        self._received: Counter[str] = Counter()
        self._errors: Counter[str] = Counter()
        self._records = 0
        self._last_publish_ts = 0.0
        self._last_receive_ts = 0.0

    def start(self, service: LCMEndpointService) -> None:
        """Attach to the endpoint service and begin publishing JSONL records."""

        if self._started:
            return
        self._service = service
        self._stop.clear()
        self._started = True
        if self._command:
            self._start_process_reader()
        elif self._path is not None:
            if self._loop:
                self._thread = threading.Thread(target=self._run_file_loop, daemon=True)
                self._thread.start()
            else:
                self._publish_file_once()
        else:
            self._started = False
            raise ValueError(
                "jsonl endpoint source requires LINGTU_ENDPOINT_JSONL_PATH "
                "or LINGTU_ENDPOINT_JSONL_COMMAND"
            )

    def stop(self) -> None:
        """Stop reading records and release any child process."""

        self._stop.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2.0)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._process = None
        self._thread = None
        self._service = None
        self._started = False

    def on_lingtu_message(self, event: LCMEndpointEvent) -> None:
        """Record LingTu-to-endpoint outputs observed by this source."""

        self._received[event.topic] += 1
        self._last_receive_ts = event.ts

    def health(self) -> Mapping[str, Any]:
        """Return source status and publication counters."""

        running = self._process is not None and self._process.poll() is None
        return {
            "name": self.name,
            "hardware": False,
            "role": "sensor_localization_jsonl",
            "started": self._started,
            "path": str(self._path) if self._path is not None else "",
            "command": _public_command(self._command),
            "process_running": running,
            "process_pid": self._process.pid if running and self._process else None,
            "loop": self._loop,
            "rate_hz": self._rate_hz,
            "records": self._records,
            "published": dict(self._published),
            "received": dict(self._received),
            "errors": dict(self._errors),
            "last_publish_ts": self._last_publish_ts,
            "last_receive_ts": self._last_receive_ts,
        }

    def _start_process_reader(self) -> None:
        """Launch a JSONL producer process and read stdout in a thread."""

        args: str | Sequence[str]
        if isinstance(self._command, str):
            args = self._command if self._shell else shlex.split(self._command)
        else:
            args = list(self._command or ())
        if not args:
            raise ValueError("jsonl endpoint source command is empty")
        self._process = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            encoding=self._encoding,
            shell=self._shell,
        )
        self._thread = threading.Thread(target=self._read_process_stdout, daemon=True)
        self._thread.start()

    def _read_process_stdout(self) -> None:
        """Read records from the configured process stdout."""

        process = self._process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            if self._stop.is_set():
                break
            self._publish_line(line)

    def _run_file_loop(self) -> None:
        """Replay the configured JSONL file until stopped."""

        while not self._stop.is_set():
            self._publish_file_once()

    def _publish_file_once(self) -> None:
        """Publish all records from the configured JSONL file once."""

        if self._path is None:
            raise ValueError("jsonl endpoint source path is not configured")
        with self._path.open("r", encoding=self._encoding) as handle:
            for line in handle:
                if self._stop.is_set():
                    break
                self._publish_line(line)

    def _publish_line(self, line: str) -> None:
        """Decode and publish one JSONL record."""

        raw = line.lstrip("\ufeff").strip()
        if not raw or raw.startswith("#"):
            return
        try:
            record = json.loads(raw)
            self._publish_record(record)
        except Exception as exc:
            self._errors[type(exc).__name__] += 1
            logger.exception("JSONL endpoint source dropped record: %s", exc)
        finally:
            if self._rate_hz > 0:
                time.sleep(1.0 / self._rate_hz)

    def _publish_record(self, record: Mapping[str, Any]) -> None:
        """Convert one decoded JSON record into an endpoint publication."""

        delay = record.get("sleep_sec", record.get("delay_sec"))
        if delay is not None:
            time.sleep(max(0.0, float(delay)))
            return

        service = self._service
        if service is None:
            raise RuntimeError("jsonl endpoint source is not attached")
        topic = str(record.get("topic") or "")
        if not topic:
            raise ValueError("jsonl endpoint record missing topic")
        binding = service.contract.binding_for_topic(topic)
        if binding.direction != "endpoint_to_lingtu":
            raise ValueError(f"{topic} is not an endpoint-to-LingTu topic")

        message = message_from_record(binding, record)
        service.publish_to_lingtu(topic, message)
        self._records += 1
        self._published[topic] += 1
        self._last_publish_ts = time.time()


def create(**overrides: Any) -> JsonlEndpointSource:
    """Factory used by endpoint runner ``--source jsonl``."""

    path = overrides.pop("path", None) or _env_str(
        "LINGTU_ENDPOINT_JSONL_PATH",
        _env_str("LINGTU_THUNDER_JSONL_PATH", ""),
    )
    command = overrides.pop("command", None) or _env_str(
        "LINGTU_ENDPOINT_JSONL_COMMAND",
        "",
    )
    loop = bool(overrides.pop("loop", _env_bool("LINGTU_ENDPOINT_JSONL_LOOP", False)))
    rate_hz = float(overrides.pop("rate_hz", _env_float("LINGTU_ENDPOINT_JSONL_RATE_HZ", 0.0)))
    shell = bool(overrides.pop("shell", _env_bool("LINGTU_ENDPOINT_JSONL_COMMAND_SHELL", False)))
    return JsonlEndpointSource(
        path=path or None,
        command=command or None,
        loop=loop,
        rate_hz=rate_hz,
        shell=shell,
        **overrides,
    )


def message_from_record(binding: LCMEndpointBinding, record: Mapping[str, Any]) -> Any:
    """Build the contract message represented by a JSONL record."""

    if record.get("format") == LCM_PAYLOAD_FORMAT and "payload" in record:
        return loads_endpoint_message(binding, json.dumps(record).encode("utf-8"))
    if "payload" in record:
        envelope = {
            "format": LCM_PAYLOAD_FORMAT,
            "topic": binding.topic,
            "schema": str(record.get("schema") or binding.schema),
            "payload": record["payload"],
        }
        return loads_endpoint_message(binding, json.dumps(envelope).encode("utf-8"))
    if "message" in record:
        return _message_from_plain_payload(binding.schema, record["message"])
    if "value" in record:
        return _message_from_plain_payload(binding.schema, record["value"])
    raise ValueError(f"jsonl endpoint record for {binding.topic} missing payload/message")


def _message_from_record(binding: LCMEndpointBinding, record: Mapping[str, Any]) -> Any:
    """Backward-compatible private wrapper for older imports."""

    return message_from_record(binding, record)


def _message_from_plain_payload(schema: str, payload: Any) -> Any:
    """Convert a plain JSON payload into a core endpoint message."""

    if schema == "lingtu.sensor.point_cloud2.v1":
        return _point_cloud_from_plain_payload(payload)
    if schema == "lingtu.sensor.imu.v1":
        return _imu_from_plain_payload(payload)
    if schema == "lingtu.nav.odometry.v1":
        return Odometry.from_dict(dict(payload or {}))
    if schema == "lingtu.nav.path.v1":
        return NavPath.from_dict(dict(payload or {}))
    if schema == "lingtu.geometry.pose_stamped.v1":
        return PoseStamped.from_dict(dict(payload or {}))
    if schema == "lingtu.geometry.twist.v1":
        return Twist.from_dict(dict(payload or {}))
    if schema == "lingtu.status.localization_quality.v1":
        return float(payload)
    if schema == "lingtu.status.localization_health.v1":
        return dict(payload or {})
    if schema in {"lingtu.control.cancel.v1", "lingtu.control.instruction.v1"}:
        return str(payload or "")
    return payload


def _point_cloud_from_plain_payload(payload: Any) -> PointCloud2:
    """Create a point cloud from JSON arrays or a point-cloud mapping."""

    if isinstance(payload, Mapping):
        if "payload" in payload:
            return _point_cloud_from_plain_payload(payload["payload"])
        if "points" in payload:
            return PointCloud2(
                points=payload.get("points") or [],
                ts=float(payload.get("ts", 0.0) or time.time()),
                frame_id=str(payload.get("frame_id") or "map"),
            )
    return PointCloud2(points=payload or [])


def _imu_from_plain_payload(payload: Any) -> Imu:
    """Create an IMU message from a JSON mapping."""

    if not isinstance(payload, Mapping):
        raise ValueError("IMU JSONL payload must be a mapping")
    return Imu(
        orientation=Quaternion.from_dict(dict(payload.get("orientation") or {})),
        angular_velocity=Vector3.from_dict(dict(payload.get("angular_velocity") or {})),
        linear_acceleration=Vector3.from_dict(dict(payload.get("linear_acceleration") or {})),
        ts=float(payload.get("ts", 0.0) or time.time()),
        frame_id=str(payload.get("frame_id") or "imu_link"),
    )


def _public_command(command: str | Sequence[str] | None) -> str:
    """Return a redacted command string for health output."""

    if command is None:
        return ""
    text = command if isinstance(command, str) else " ".join(str(part) for part in command)
    tokens = text.split()
    redacted = [
        "<redacted>" if any(marker in token.lower() for marker in ("token", "secret", "key="))
        else token
        for token in tokens
    ]
    return " ".join(redacted)


def _env_str(name: str, default: str) -> str:
    """Read a non-empty string environment variable."""

    value = os.getenv(name)
    return str(value).strip() if value not in (None, "") else default


def _env_float(name: str, default: float) -> float:
    """Read a float environment variable."""

    value = os.getenv(name)
    if value in (None, ""):
        return float(default)
    return float(str(value).strip())


def _env_bool(name: str, default: bool) -> bool:
    """Read a boolean environment variable."""

    value = os.getenv(name)
    if value in (None, ""):
        return bool(default)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}
