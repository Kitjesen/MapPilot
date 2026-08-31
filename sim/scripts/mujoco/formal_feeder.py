"""Product-owned MuJoCo physics and native sensor feeder.

The process is launched and stopped only by the simulation supervisor.  It
consumes identity from the direct-child environment, connects to the two
Product-owned native endpoints, and is the sole owner of MuJoCo stepping.
"""

# ruff: noqa: E402 - direct script execution must establish repository roots first.

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import secrets
import shutil
import signal
import sys
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Iterator, Mapping, Sequence, cast


def _prepare_direct_script_import_path() -> None:
    if __package__ not in {None, ""}:
        return
    repository_root = Path(__file__).resolve().parents[3]
    source_root = repository_root / "src"
    script_root = Path(__file__).resolve().parent
    retained = [entry for entry in sys.path if Path(entry or ".").resolve() != script_root]
    sys.path[:] = [str(repository_root), str(source_root), *retained]


_prepare_direct_script_import_path()

from sim.engine.core.engine import VelocityCommand
from sim.engine.core.sensor import CameraConfig
from sim.runtime.scenario.runtime import ScenarioClock, ScenarioRuntime
from sim.runtime.windows_timing import deadline_waiter
from sim.scripts.mujoco.driver_bridge_session import (
    DriverBridgeCommand,
    DriverBridgeSession,
    DriverBridgeStoppedEvidence,
)
from sim.scripts.mujoco.evidence import (
    FEEDER_STATUS_SCHEMA,
    MAX_ABS_TILT_RAD,
    MAX_BASE_HEIGHT_M,
    MAX_BASE_HEIGHT_SPAN_M,
    MIN_BASE_HEIGHT_M,
    MOTION_EVIDENCE_SCHEMA,
    publish_feeder_status,
    publish_motion_evidence,
)
from sim.scripts.mujoco.native_runtime_endpoint import (
    CAMERA_ROLE,
    IMU_ROLE,
    LIDAR_ROLE,
    DriverBridgeClient,
    SensorPublisherClient,
)
from sim.scripts.mujoco.native_sensor_records import (
    CAMERA_RECORD_HEADER,
    encode_camera_depth,
    encode_camera_intrinsics,
    encode_camera_rgb,
    encode_imu,
    encode_odom_prior,
    encode_registered_cloud,
    encode_scan,
)

from drivers.real.camera.shm import ShmFrameWriter, StreamKind
from drivers.sim.mujoco.runtime import (
    DEFAULT_MID360_PATTERN,
    DEFAULT_MID360_SAMPLES_PER_FRAME,
    build_engine,
    draw_navigation_paths,
    focus_presentation_viewer,
    launch_presentation_viewer,
)
from drivers.sim.mujoco.sensors import (
    navigation_fixture_registered_body_points,
    sensor_specific_force_body,
    world_xyzi_to_body_xyzi,
    world_xyzi_to_sensor_xyzi,
)
from lingtu.run_plan import RunPlan
from lingtu.sim.viewer_input import ViewerInput, viewer_input_from_run_plan
from lingtu.sim.readiness import SIM_FEEDER_SCHEMA, validate_feeder_readiness
from lingtu.sim.stop import MOTION_STOP_SCHEMA, publish_motion_stop_evidence
from lingtu.switch_contracts import is_product_session_id
from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import POINT_DTYPE, Imu, LivoxPointFrame
from runtime.runtime_interface import TOPICS, topic_default_frame_id

_PROCESS_NAME = "mujoco_feeder"
_READINESS_NAME = "mujoco_feeder.ready.json"
_MOTION_EVIDENCE_NAME = "mujoco_feeder.motion.json"
_ENDPOINT_TIMEOUT_S = 2.0
_HEARTBEAT_PERIOD_S = 0.05
_SCENARIO_SNAPSHOT_NAME = "scenario.current.json"
_SCENARIO_SNAPSHOT_PERIOD_NS = 50_000_000
_UINT32_MAX = (1 << 32) - 1
_REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
_SENSOR_ENDPOINTS = (
    (LIDAR_ROLE, "lidar.ready.json"),
    (IMU_ROLE, "imu.ready.json"),
    (CAMERA_ROLE, "camera.ready.json"),
)


def _read_navigation_status(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


class FormalFeederError(RuntimeError):
    """The formal feeder could not preserve its runtime contract."""


def _load_run_plan(environment: Mapping[str, str]) -> tuple[RunPlan, Path, str]:
    run_plan_value = str(environment.get("LINGTU_RUN_PLAN") or "")
    product_session_id = str(environment.get("LINGTU_PRODUCT_SESSION_ID") or "")
    if not run_plan_value or run_plan_value != run_plan_value.strip():
        raise FormalFeederError("LINGTU_RUN_PLAN is required")
    run_plan_path = Path(run_plan_value)
    if not run_plan_path.is_absolute():
        raise FormalFeederError("LINGTU_RUN_PLAN must be an absolute path")
    if not is_product_session_id(product_session_id):
        raise FormalFeederError("LINGTU_PRODUCT_SESSION_ID is invalid")
    return RunPlan.load(run_plan_path), run_plan_path.parent, product_session_id


def _publish_session_bytes(session_root: Path, filename: str, payload: bytes) -> Path:
    if not filename or Path(filename).name != filename:
        raise FormalFeederError("invalid session artifact filename")
    destination = session_root / filename
    temporary = destination.with_name(f".{filename}.{secrets.token_hex(8)}.tmp")
    try:
        temporary.write_bytes(payload)
        os.chmod(temporary, 0o600)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination


def _session_id(value: str) -> str:
    if not value or value != value.strip():
        raise argparse.ArgumentTypeError("value must be non-empty trimmed text")
    return value


def _nonnegative_int(value: str) -> int:
    if not value.isascii() or not value.isdecimal():
        raise argparse.ArgumentTypeError("value must be a canonical non-negative integer")
    return int(value)


def _positive_int(value: str) -> int:
    parsed = _nonnegative_int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def _finite_positive(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be a finite positive number") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be a finite positive number")
    return parsed


def _frequency(minimum: float, maximum: float):
    def parse(value: str) -> float:
        parsed = _finite_positive(value)
        if not minimum <= parsed <= maximum:
            raise argparse.ArgumentTypeError(f"frequency must be in [{minimum:g}, {maximum:g}] Hz")
        return parsed

    return parse


def _zero_frequency(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("first-version odom prior rate must be 0") from exc
    if not math.isfinite(parsed) or parsed != 0.0 or math.copysign(1.0, parsed) < 0.0:
        raise argparse.ArgumentTypeError("first-version odom prior rate must be 0")
    return 0.0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lingtu-mujoco-feeder",
        description="Run the Product-owned MuJoCo physics and native sensor feeder.",
    )
    parser.add_argument("--session-id", type=_session_id)
    parser.add_argument("--model-generation", type=_nonnegative_int)
    parser.add_argument("--reset-generation", type=_nonnegative_int)
    parser.add_argument("--world")
    parser.add_argument("--start", default="")
    parser.add_argument("--drive-mode", choices=("kinematic", "policy"), default="policy")
    parser.add_argument("--mujoco-memory", default="")
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument(
        "--mid360-samples-per-frame",
        type=_positive_int,
        default=DEFAULT_MID360_SAMPLES_PER_FRAME,
    )
    parser.add_argument(
        "--lidar-backend",
        choices=("mujoco_lidar", "ray_caster_lidar"),
        default="mujoco_lidar",
    )
    parser.add_argument(
        "--mujoco-lidar-backend",
        choices=("cpu", "taichi", "warp", "jax"),
        default="cpu",
    )
    parser.add_argument("--policy-path", type=Path)
    parser.add_argument("--max-linear-mps", type=_finite_positive, default=1.0)
    parser.add_argument("--max-angular-rps", type=_finite_positive, default=1.0)
    parser.add_argument("--imu-hz", type=_frequency(50.0, 500.0))
    parser.add_argument("--lidar-hz", type=_frequency(1.0, 30.0))
    parser.add_argument("--odom-prior-hz", type=_zero_frequency, default=0.0)
    parser.add_argument("--max-points", type=_positive_int, default=100_000)
    parser.add_argument(
        "--viewer",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Open the passive MuJoCo viewer; MUJOCO_VIEWER=1 enables it for a Product child.",
    )
    parser.add_argument("--viewer-hz", type=_finite_positive, default=30.0)
    return parser


def _request_stop(
    stop_event: threading.Event,
    _signum: int,
    _frame: Any,
) -> None:
    """Signal callback: record intent only; all shutdown work stays in main."""

    stop_event.set()


@contextmanager
def _installed_stop_handlers(stop_event: threading.Event) -> Iterator[None]:
    installed: list[tuple[signal.Signals, Any]] = []
    candidates = [signal.SIGTERM, signal.SIGINT]
    if hasattr(signal, "SIGBREAK"):
        candidates.append(signal.SIGBREAK)

    def handle(signum: int, frame: Any) -> None:
        _request_stop(stop_event, signum, frame)

    try:
        for candidate in candidates:
            previous = signal.getsignal(candidate)
            signal.signal(candidate, handle)
            installed.append((candidate, previous))
        yield
    finally:
        for candidate, previous in reversed(installed):
            signal.signal(candidate, previous)


class _Services:
    """Small dependency boundary for platform I/O and deterministic tests."""

    @staticmethod
    def resolve_artifact(value: str) -> Path:
        candidate = (_REPOSITORY_ROOT / value).resolve()
        try:
            candidate.relative_to(_REPOSITORY_ROOT)
        except ValueError as exc:
            raise FormalFeederError("RunPlan simulation artifact escapes repository") from exc
        if not candidate.is_file():
            raise FormalFeederError(f"RunPlan simulation artifact is missing: {value}")
        return candidate

    @staticmethod
    def resolve_directory(value: str) -> Path:
        candidate = (_REPOSITORY_ROOT / value).resolve()
        try:
            candidate.relative_to(_REPOSITORY_ROOT)
        except ValueError as exc:
            raise FormalFeederError("RunPlan simulation directory escapes repository") from exc
        if not candidate.is_dir():
            raise FormalFeederError(f"RunPlan simulation directory is missing: {value}")
        return candidate

    @classmethod
    def snapshot_artifacts(
        cls,
        session_root: Path,
        config: _RuntimeConfig,
    ) -> tuple[Path, Path, Path]:
        required = (config.world, config.robot.policy)
        snapshot_root = session_root / (f".formal-feeder-artifacts-{secrets.token_hex(16)}")
        snapshot_root.mkdir(mode=0o700)
        package_relative = PurePosixPath(config.robot.package_root)
        model_relative = PurePosixPath(config.robot.model)
        if (
            package_relative.is_absolute()
            or ".." in package_relative.parts
            or "\\" in config.robot.package_root
            or model_relative.parts[: len(package_relative.parts)] != package_relative.parts
        ):
            raise FormalFeederError("RunPlan robot package path is unsafe")
        package_source = cls.resolve_directory(config.robot.package_root)
        package_destination = snapshot_root / Path(*package_relative.parts)
        shutil.copytree(package_source, package_destination, copy_function=shutil.copy2)
        copied_packages = {package_relative}
        kinematic_entities = tuple(getattr(config, "kinematic_entities", ()))
        for entity in kinematic_entities:
            entity_package = PurePosixPath(entity.package_root)
            if entity_package in copied_packages:
                continue
            shutil.copytree(
                cls.resolve_directory(entity.package_root),
                snapshot_root / Path(*entity_package.parts),
                copy_function=shutil.copy2,
            )
            copied_packages.add(entity_package)
        for relative in sorted(required):
            relative_path = PurePosixPath(relative)
            if relative_path.is_absolute() or ".." in relative_path.parts or "\\" in relative:
                raise FormalFeederError(f"RunPlan simulation artifact path is unsafe: {relative}")
            source = cls.resolve_artifact(relative)
            destination = snapshot_root / Path(*relative_path.parts)
            destination.parent.mkdir(parents=True, exist_ok=True)
            with source.open("rb") as reader, destination.open("xb") as writer:
                while chunk := reader.read(1024 * 1024):
                    writer.write(chunk)
                writer.flush()
                os.fsync(writer.fileno())
        world_path = snapshot_root / Path(*PurePosixPath(config.world).parts)
        if kinematic_entities:
            world_path = cls.compose_scenario_world(
                world_path,
                tuple(
                    (
                        entity,
                        snapshot_root / Path(*PurePosixPath(entity.model).parts),
                    )
                    for entity in kinematic_entities
                ),
            )
        return (
            world_path,
            snapshot_root / Path(*PurePosixPath(config.robot.model).parts),
            snapshot_root / Path(*PurePosixPath(config.robot.policy).parts),
        )

    @staticmethod
    def compose_scenario_world(
        world_path: Path,
        entities: tuple[tuple[_KinematicEntityConfig, Path], ...],
    ) -> Path:
        import mujoco

        spec = mujoco.MjSpec.from_file(str(world_path))
        for entity, model_path in entities:
            child = mujoco.MjSpec.from_file(str(model_path))
            frame = spec.worldbody.add_frame(
                name=f"{entity.entity_id}__frame",
                pos=entity.position_m,
                quat=entity.quaternion_wxyz,
            )
            spec.attach(child, prefix=f"{entity.entity_id}__", frame=frame)
        destination = world_path.with_name(f"{world_path.stem}.scenario.xml")
        destination.write_text(spec.to_xml(), encoding="utf-8")
        for name, payload in spec.assets.items():
            relative = PurePosixPath(name)
            if relative.is_absolute() or ".." in relative.parts:
                raise FormalFeederError(f"MuJoCo scenario asset path is invalid: {name}")
            target = destination.parent / Path(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
        return destination

    @staticmethod
    def build_engine(**kwargs: Any) -> Any:
        return build_engine(**kwargs)

    @staticmethod
    def connect_sensor(
        path: Path,
        *,
        role: str,
        product_session_id: str,
        timeout_s: float,
    ) -> SensorPublisherClient:
        return SensorPublisherClient.connect(
            path,
            role=role,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )

    @staticmethod
    def connect_driver(
        path: Path,
        *,
        product_session_id: str,
        timeout_s: float,
    ) -> DriverBridgeClient:
        return DriverBridgeClient.connect(
            path,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )

    @staticmethod
    def create_camera_writer(
        path: Path,
        *,
        stream_kind: StreamKind,
        slot_capacity: int,
    ) -> ShmFrameWriter:
        return ShmFrameWriter(
            path,
            stream_kind=stream_kind,
            slot_capacity=slot_capacity,
        )

    @staticmethod
    def create_session(
        driver: DriverBridgeClient,
        *,
        expected_product_session_id: str,
    ) -> DriverBridgeSession:
        return DriverBridgeSession(
            send_line=driver.send_line,
            recv_line=driver.recv_line,
            close=driver.close,
            expected_product_session_id=expected_product_session_id,
            operation_timeout_s=_ENDPOINT_TIMEOUT_S,
        )

    @staticmethod
    def driver_input_available(driver: DriverBridgeClient) -> bool:
        return driver.input_available()

    def command_available(
        self,
        session: DriverBridgeSession,
        driver: DriverBridgeClient,
    ) -> bool:
        if getattr(session, "_queued_command", None) is not None:
            return True
        return self.driver_input_available(driver)

    @staticmethod
    def monotonic() -> float:
        # CPython may expose a 15.625 ms ``monotonic`` clock on Windows,
        # which cannot schedule the Product's 200 Hz physics/IMU deadline.
        # ``perf_counter`` is monotonic too, but uses the high-resolution
        # performance counter required by this timing loop.
        return time.perf_counter()

    @staticmethod
    def wall_time() -> float:
        return time.time()

    @staticmethod
    def wait(stop_event: threading.Event, timeout_s: float) -> bool:
        return stop_event.wait(timeout_s)

    @staticmethod
    @contextmanager
    def wait_scope(
        stop_event: threading.Event,
    ) -> Iterator[Callable[[float], bool]]:
        with deadline_waiter(stop_event) as wait:
            yield wait

    @staticmethod
    def publish_readiness(
        session_root: Path,
        target: str,
        payload: dict[str, Any],
    ) -> Path:
        document = validate_feeder_readiness(payload)
        return _publish_session_bytes(
            session_root,
            target,
            json.dumps(document, allow_nan=False, separators=(",", ":")).encode(),
        )

    @staticmethod
    def publish_evidence(
        *,
        session_root: Path,
        target: str,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        return dict(
            publish_motion_stop_evidence(
                session_root=session_root,
                target=target,
                payload=payload,
                bound_publisher=lambda name, raw: _publish_session_bytes(session_root, name, raw),
            )
        )

    @staticmethod
    def publish_motion(
        *,
        session_root: Path,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        return dict(
            publish_motion_evidence(
                session_root=session_root,
                payload=payload,
                bound_publisher=lambda name, raw: _publish_session_bytes(session_root, name, raw),
            )
        )

    @staticmethod
    def publish_status(
        *,
        session_root: Path,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        return dict(
            publish_feeder_status(
                session_root=session_root,
                payload=payload,
                bound_publisher=lambda name, raw: _publish_session_bytes(session_root, name, raw),
            )
        )

    @staticmethod
    def publish_scenario(session_root: Path, payload: Mapping[str, Any]) -> Path:
        return _publish_session_bytes(
            session_root,
            _SCENARIO_SNAPSHOT_NAME,
            json.dumps(dict(payload), allow_nan=False, separators=(",", ":")).encode(),
        )


_SERVICES = _Services()


@dataclass(frozen=True)
class _Tick:
    now_s: float
    due_s: float
    skipped: int


@dataclass
class _StreamStats:
    expected_hz: float
    scheduled_count: int = 0
    published_count: int = 0
    dropped_count: int = 0
    max_lateness_s: float = 0.0
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    def due(self, *, count: int, dropped: int, lateness_s: float) -> None:
        if count <= 0 or dropped < 0 or dropped > count:
            raise FormalFeederError("sensor schedule accounting is invalid")
        with self._lock:
            self.scheduled_count += count
            self.dropped_count += dropped
            self.max_lateness_s = max(self.max_lateness_s, max(0.0, lateness_s))

    def published(self) -> None:
        with self._lock:
            outstanding = self.scheduled_count - self.published_count - self.dropped_count
            if outstanding < 1:
                raise FormalFeederError("sensor publish accounting is invalid")
            self.published_count += 1

    def drop_current(self) -> None:
        with self._lock:
            outstanding = self.scheduled_count - self.published_count - self.dropped_count
            if outstanding < 1:
                raise FormalFeederError("sensor drop accounting is invalid")
            self.dropped_count += 1

    def drop_pending(self) -> None:
        with self._lock:
            outstanding = self.scheduled_count - self.published_count - self.dropped_count
            if outstanding > 0:
                self.dropped_count += outstanding

    def drop_one_if_pending(self) -> None:
        with self._lock:
            outstanding = self.scheduled_count - self.published_count - self.dropped_count
            if outstanding > 0:
                self.dropped_count += 1

    def payload(self, window_s: float) -> dict[str, Any]:
        with self._lock:
            actual_hz = self.published_count / window_s if window_s > 0.0 else 0.0
            return {
                "expected_hz": self.expected_hz,
                "scheduled_count": self.scheduled_count,
                "published_count": self.published_count,
                "dropped_count": self.dropped_count,
                "actual_hz": actual_hz,
                "max_schedule_lateness_ms": self.max_lateness_s * 1000.0,
            }


class _Status:
    _PUBLISH_PERIOD_S = 1.0

    def __init__(
        self,
        *,
        services: _Services,
        session_root: Path,
        product_session_id: str,
        product: str,
        started_s: float,
        streams: Mapping[str, _StreamStats],
    ) -> None:
        self._services = services
        self._session_root = session_root
        self._product_session_id = product_session_id
        self._product = product
        self._started_s = started_s
        self._streams = dict(streams)
        self._sequence = 0
        self._next_publish_s = started_s
        self._window_s: float | None = None

    def begin(self, started_s: float) -> None:
        if not math.isfinite(started_s):
            raise FormalFeederError("status monotonic clock is invalid")
        self._started_s = started_s
        self._next_publish_s = started_s
        self._window_s = None

    def running(self, now_s: float, *, force: bool = False) -> None:
        if not force and now_s + 1e-12 < self._next_publish_s:
            return
        self._next_publish_s = now_s + self._PUBLISH_PERIOD_S
        self._publish("running", now_s)

    def finish(self, state: str, now_s: float) -> None:
        if self._window_s is None:
            self._window_s = max(0.0, now_s - self._started_s)
        self._publish(state, now_s)

    def _publish(self, state: str, now_s: float) -> None:
        window_s = self._window_s if self._window_s is not None else max(0.0, now_s - self._started_s)
        payload = {
            "schema": FEEDER_STATUS_SCHEMA,
            "product_session_id": self._product_session_id,
            "product": self._product,
            "process": _PROCESS_NAME,
            "state": state,
            "sequence": self._sequence,
            "updated_wall_ns": int(self._services.wall_time() * 1_000_000_000),
            "window_s": window_s,
            "streams": {name: stats.payload(window_s) for name, stats in sorted(self._streams.items())},
        }
        try:
            self._services.publish_status(session_root=self._session_root, payload=payload)
        except Exception as exc:
            _report_failure(FormalFeederError(f"feeder status publish failed: {exc}"))
            return
        self._sequence += 1


@dataclass
class _AbsoluteDeadline:
    period_s: float
    next_due_s: float

    @classmethod
    def start(cls, *, period_s: float, now_s: float) -> _AbsoluteDeadline:
        return cls(period_s=period_s, next_due_s=now_s + period_s)

    def wait_next(
        self,
        services: _Services,
        stop_event: threading.Event,
        *,
        wait: Callable[[float], bool] | None = None,
    ) -> _Tick | None:
        due_s = self.next_due_s
        now = services.monotonic()
        if not math.isfinite(now):
            raise FormalFeederError("monotonic clock is invalid")
        if now < self.next_due_s:
            wait_for = wait or (lambda timeout_s: services.wait(stop_event, timeout_s))
            if wait_for(self.next_due_s - now):
                return None
            now = services.monotonic()
            if not math.isfinite(now):
                raise FormalFeederError("monotonic clock is invalid")
        if stop_event.is_set():
            return None
        # Advance exactly one scheduled physics slot. If Windows delayed this
        # wake-up, subsequent calls return immediately until simulated time has
        # caught up instead of deleting physics and IMU samples.
        self.next_due_s = due_s + self.period_s
        return _Tick(now_s=now, due_s=due_s, skipped=0)


def _position(state: Any) -> tuple[float, float, float]:
    try:
        values = tuple(float(value) for value in state.position)
    except (AttributeError, TypeError, ValueError) as exc:
        raise FormalFeederError("MuJoCo state position is invalid") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise FormalFeederError("MuJoCo state position is invalid")
    return values


def _pose_metrics(state: Any) -> tuple[float, float, float]:
    position = _position(state)
    try:
        quaternion = tuple(float(value) for value in state.orientation)
    except (AttributeError, TypeError, ValueError) as exc:
        raise FormalFeederError("MuJoCo state orientation is invalid") from exc
    if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
        raise FormalFeederError("MuJoCo state orientation is invalid")
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise FormalFeederError("MuJoCo state orientation is invalid")
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_sine)
    return position[2], abs(roll), abs(pitch)


def _yaw(state: Any) -> float:
    try:
        x, y, z, w = (float(value) for value in state.orientation)
    except (AttributeError, TypeError, ValueError) as exc:
        raise FormalFeederError("MuJoCo state orientation is invalid") from exc
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise FormalFeederError("MuJoCo state orientation is invalid")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise FormalFeederError("MuJoCo state orientation is invalid")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _require_stable_pose(state: Any) -> tuple[float, float, float]:
    height, abs_roll, abs_pitch = _pose_metrics(state)
    if (
        height < MIN_BASE_HEIGHT_M
        or height > MAX_BASE_HEIGHT_M
        or abs_roll > MAX_ABS_TILT_RAD
        or abs_pitch > MAX_ABS_TILT_RAD
    ):
        raise FormalFeederError("MuJoCo base pose is outside the stability gate")
    return height, abs_roll, abs_pitch


def _report_failure(exc: Exception) -> None:
    reason = " ".join(str(exc).split())[:512] or "unspecified"
    print(
        f"formal_feeder_failed type={type(exc).__name__} reason={reason}",
        file=sys.stderr,
        flush=True,
    )


@dataclass
class _PhysicalMotionEvidence:
    start_position: tuple[float, float, float] | None = None
    end_position: tuple[float, float, float] | None = None
    path_length_xy_m: float = 0.0
    nonzero_command_count: int = 0
    nonzero_physics_steps: int = 0
    first_bridge_command_seq: int = 0
    last_bridge_command_seq: int = 0
    first_motion_step_seq: int = 0
    last_motion_step_seq: int = 0
    first_producer_boot_id: str = ""
    last_producer_boot_id: str = ""
    first_output_sequence: int = 0
    last_output_sequence: int = 0
    pose_sample_count: int = 0
    min_base_height_m: float = math.inf
    max_base_height_m: float = -math.inf
    max_abs_roll_rad: float = 0.0
    max_abs_pitch_rad: float = 0.0
    start_yaw_rad: float | None = None
    end_yaw_rad: float | None = None
    trajectory: list[list[float]] = field(default_factory=list)
    last_trace_step_seq: int = 0
    last_pose_step_seq: int = 0

    def record_trace(self, state: Any, step_seq: int, *, force: bool = False) -> None:
        position = _position(state)
        yaw = _yaw(state)
        if self.trajectory and not force:
            previous = self.trajectory[-1]
            step_gap = step_seq - self.last_trace_step_seq
            distance = math.hypot(position[0] - previous[1], position[1] - previous[2])
            if step_gap < 20 and distance < 0.025:
                return
        self.trajectory.append(
            [float(step_seq), position[0], position[1], position[2], yaw]
        )
        self.last_trace_step_seq = step_seq

    def observe_pose(self, state: Any) -> None:
        height, abs_roll, abs_pitch = _require_stable_pose(state)
        self.pose_sample_count += 1
        self.min_base_height_m = min(self.min_base_height_m, height)
        self.max_base_height_m = max(self.max_base_height_m, height)
        self.max_abs_roll_rad = max(self.max_abs_roll_rad, abs_roll)
        self.max_abs_pitch_rad = max(self.max_abs_pitch_rad, abs_pitch)
        if self.max_base_height_m - self.min_base_height_m > MAX_BASE_HEIGHT_SPAN_M:
            raise FormalFeederError(
                "MuJoCo base height span exceeds the stability gate: "
                f"min={self.min_base_height_m:.6f} "
                f"max={self.max_base_height_m:.6f} current={height:.6f}"
            )

    def observe(
        self,
        command: DriverBridgeCommand,
        *,
        before: Any,
        after: Any,
        step_seq: int,
    ) -> None:
        try:
            self.observe_pose(after)
        except FormalFeederError as exc:
            raise FormalFeederError(
                f"{exc}; step={step_seq} kind={command.kind} "
                f"walk=({command.walk_x:.6f},{command.walk_y:.6f},"
                f"{command.walk_z:.6f})"
            ) from exc
        nonzero_nav = command.kind == "nav" and not (
            command.walk_x == 0.0 and command.walk_y == 0.0 and command.walk_z == 0.0
        )
        start = _position(before)
        end = _position(after)
        self.last_pose_step_seq = step_seq
        if self.start_position is None:
            if not nonzero_nav:
                return
            self.start_position = start
            self.start_yaw_rad = _yaw(before)
            self.first_bridge_command_seq = command.bridge_command_seq
            self.first_motion_step_seq = step_seq
            self.first_producer_boot_id = command.producer_boot_id
            self.first_output_sequence = command.output_sequence
            self.record_trace(before, max(0, step_seq - 1), force=True)
        self.path_length_xy_m += math.hypot(end[0] - start[0], end[1] - start[1])
        self.end_position = end
        self.end_yaw_rad = _yaw(after)
        self.record_trace(after, step_seq)
        if not nonzero_nav:
            return
        if command.bridge_command_seq != self.last_bridge_command_seq:
            self.nonzero_command_count += 1
        self.nonzero_physics_steps += 1
        self.last_bridge_command_seq = command.bridge_command_seq
        self.last_motion_step_seq = step_seq
        self.last_producer_boot_id = command.producer_boot_id
        self.last_output_sequence = command.output_sequence

    def payload(
        self,
        *,
        product_session_id: str,
        product: str,
        stopped: DriverBridgeStoppedEvidence,
    ) -> dict[str, Any]:
        if self.pose_sample_count <= 0:
            raise FormalFeederError("MuJoCo pose evidence is empty")
        observed = self.start_position is not None and self.end_position is not None
        if observed:
            assert self.start_position is not None and self.end_position is not None
            displacement = math.hypot(
                self.end_position[0] - self.start_position[0],
                self.end_position[1] - self.start_position[1],
            )
            start_position: list[float] | None = list(self.start_position)
            end_position: list[float] | None = list(self.end_position)
            if (
                not self.trajectory
                or math.hypot(
                    self.end_position[0] - self.trajectory[-1][1],
                    self.end_position[1] - self.trajectory[-1][2],
                )
                > 1e-9
            ):
                self.trajectory.append(
                    [
                        float(self.last_pose_step_seq),
                        self.end_position[0],
                        self.end_position[1],
                        self.end_position[2],
                        float(self.end_yaw_rad or 0.0),
                    ]
                )
        else:
            displacement = 0.0
            start_position = None
            end_position = None
        return {
            "schema": MOTION_EVIDENCE_SCHEMA,
            "product_session_id": product_session_id,
            "product": product,
            "process": _PROCESS_NAME,
            "bridge_boot_id": stopped.bridge_boot_id,
            "controller_boot_id": stopped.controller_boot_id,
            "terminal_bridge_command_seq": stopped.bridge_command_seq,
            "terminal_applied_step_seq": stopped.applied_step_seq,
            "commanded_motion_observed": bool(observed),
            "nonzero_command_count": self.nonzero_command_count,
            "nonzero_physics_steps": self.nonzero_physics_steps,
            "first_bridge_command_seq": self.first_bridge_command_seq,
            "last_bridge_command_seq": self.last_bridge_command_seq,
            "first_motion_step_seq": self.first_motion_step_seq,
            "last_motion_step_seq": self.last_motion_step_seq,
            "first_producer_boot_id": self.first_producer_boot_id,
            "last_producer_boot_id": self.last_producer_boot_id,
            "first_output_sequence": self.first_output_sequence,
            "last_output_sequence": self.last_output_sequence,
            "start_position_m": start_position,
            "end_position_m": end_position,
            "net_displacement_xy_m": float(displacement),
            "path_length_xy_m": float(self.path_length_xy_m),
            "pose_sample_count": self.pose_sample_count,
            "min_base_height_m": float(self.min_base_height_m),
            "max_base_height_m": float(self.max_base_height_m),
            "max_abs_roll_rad": float(self.max_abs_roll_rad),
            "max_abs_pitch_rad": float(self.max_abs_pitch_rad),
            "start_yaw_rad": self.start_yaw_rad,
            "end_yaw_rad": self.end_yaw_rad,
            "trajectory": self.trajectory,
        }


@dataclass(frozen=True)
class _Subscan:
    monotonic_s: float
    sensor_points: Any
    world_points: Any


@dataclass(frozen=True)
class _LidarFrame:
    snapshot: Any
    state: Any
    frame_start_s: float
    monotonic_s: float
    wall_s: float
    sequence: int
    registered_sequence: int | None


_LIDAR_STOP = object()


@dataclass(frozen=True)
class _RobotRuntimeConfig:
    instance_id: str
    package_root: str
    model: str
    initial_keyframe: str | None
    base_body: str
    lidar_body: str
    lidar_site: str
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    policy: str
    policy_hz: float
    actuator_names: tuple[str, ...]


@dataclass(frozen=True)
class _KinematicEntityConfig:
    entity_id: str
    package_root: str
    model: str
    attach_root: str
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class _RuntimeConfig:
    session_id: str
    model_generation: int
    reset_generation: int
    world: str
    physics_timestep_s: float
    step_period_s: float
    imu_hz: float | None
    lidar_hz: float | None
    sensors_enabled: bool
    camera_enabled: bool
    camera_hz: float | None
    sensor_roles: tuple[str, ...]
    publish_odom_prior: bool
    publish_registered_cloud_fixture: bool
    navigation_fixture_raw_overlay: bool
    viewer_enabled: bool
    robot: _RobotRuntimeConfig
    scenario_plan: Mapping[str, Any] | None
    kinematic_entities: tuple[_KinematicEntityConfig, ...]


class _ScenarioFeed:
    """Apply and publish one ScenarioRuntime snapshot from MuJoCo sim time."""

    def __init__(
        self,
        *,
        engine: Any,
        runtime: ScenarioRuntime,
        services: _Services,
        session_root: Path,
        config: _RuntimeConfig,
    ) -> None:
        self._engine = engine
        self._runtime = runtime
        self._services = services
        self._session_root = session_root
        self._config = config
        self._next_publish_ns = 0
        self._last_snapshot: Any | None = None

    def start(self) -> None:
        self._apply(0)

    def before_step(self, period_s: float) -> None:
        sim_time_ns = int(round((float(self._engine.sim_time) + period_s) * 1_000_000_000))
        self._apply(sim_time_ns)

    def _apply(self, sim_time_ns: int) -> None:
        snapshot = self._last_snapshot
        if snapshot is None or snapshot.sim_time_ns != sim_time_ns:
            snapshot = self._runtime.snapshot(
                ScenarioClock(
                    session_id=self._config.session_id,
                    model_generation=self._config.model_generation,
                    reset_generation=self._config.reset_generation,
                    sim_time_ns=sim_time_ns,
                )
            )
            self._last_snapshot = snapshot
        self._engine.apply_scenario_snapshot(snapshot)
        if sim_time_ns >= self._next_publish_ns:
            self._services.publish_scenario(self._session_root, snapshot.to_dict())
            self._next_publish_ns = sim_time_ns + _SCENARIO_SNAPSHOT_PERIOD_NS


@dataclass(frozen=True)
class _CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


class _CameraPipeline:
    """One Product-gated RGB-D stream on the existing native seam."""

    _NAME = "front_camera"
    _FRAME_ID = "camera_link"
    _WIDTH = 640
    _HEIGHT = 480

    def __init__(
        self,
        *,
        services: _Services,
        client: SensorPublisherClient,
        engine: Any,
        session_root: Path,
        started_s: float,
        started_wall_s: float,
        rate_hz: float,
        stats: _StreamStats | None = None,
    ) -> None:
        if not math.isfinite(started_s):
            raise FormalFeederError("camera monotonic clock is invalid")
        if not math.isfinite(started_wall_s) or started_wall_s <= 0.0:
            raise FormalFeederError("camera wall clock is invalid")
        self._client = client
        self._engine = engine
        self._started_s = started_s
        self._started_wall_s = started_wall_s
        self._period_s = 1.0 / _positive_finite(rate_hz, "camera rate_hz")
        self._stats = stats or _StreamStats(expected_hz=rate_hz)
        self._next_due_s = started_s + self._period_s
        self._sequence = 0
        self._intrinsics_published = False
        self._writers: dict[str, ShmFrameWriter] = {}
        try:
            self._writers["color"] = services.create_camera_writer(
                session_root / "camera_color.shm",
                stream_kind=StreamKind.COLOR,
                slot_capacity=self._WIDTH * self._HEIGHT * 3,
            )
            self._writers["depth"] = services.create_camera_writer(
                session_root / "camera_depth.shm",
                stream_kind=StreamKind.DEPTH,
                slot_capacity=self._WIDTH * self._HEIGHT * 2,
            )
            self._writers["info"] = services.create_camera_writer(
                session_root / "camera_info.shm",
                stream_kind=StreamKind.INFO,
                slot_capacity=1,
            )
        except Exception:
            for writer in self._writers.values():
                writer.close()
            raise

    def publish_initial(self) -> None:
        self._publish(self._started_wall_s)

    def begin(self, *, started_s: float, started_wall_s: float) -> None:
        if not math.isfinite(started_s):
            raise FormalFeederError("camera monotonic clock is invalid")
        if not math.isfinite(started_wall_s) or started_wall_s <= 0.0:
            raise FormalFeederError("camera wall clock is invalid")
        self._started_s = started_s
        self._started_wall_s = started_wall_s
        self._next_due_s = started_s + self._period_s

    def publish_due(
        self,
        *,
        monotonic_s: float,
        observed_s: float | None = None,
    ) -> None:
        if not math.isfinite(monotonic_s) or monotonic_s < self._started_s:
            raise FormalFeederError("camera monotonic time is invalid")
        observed = monotonic_s if observed_s is None else observed_s
        if not math.isfinite(observed) or observed < self._started_s:
            raise FormalFeederError("camera observation time is invalid")
        if monotonic_s + 1e-12 < self._next_due_s:
            return
        skipped = max(
            0,
            math.floor((monotonic_s - self._next_due_s) / self._period_s),
        )
        self._stats.due(
            count=skipped + 1,
            dropped=skipped,
            lateness_s=max(0.0, monotonic_s - self._next_due_s),
        )
        self._next_due_s += (skipped + 1) * self._period_s
        wall_s = self._started_wall_s + (observed - self._started_s)
        try:
            self._publish(wall_s)
        except Exception:
            self._stats.drop_current()
            raise
        self._stats.published()

    def _publish(self, wall_s: float) -> None:
        frame = self._engine.get_camera_data(self._NAME)
        if frame is None:
            raise FormalFeederError("MuJoCo camera returned no RGB-D frame")
        try:
            rgb = np.asarray(frame.rgb)
            depth = np.asarray(frame.depth)
            fx, fy, cx, cy = (float(value) for value in frame.intrinsics)
        except (AttributeError, TypeError, ValueError) as exc:
            raise FormalFeederError("MuJoCo camera frame is invalid") from exc
        if rgb.shape != (self._HEIGHT, self._WIDTH, 3):
            raise FormalFeederError("MuJoCo camera RGB shape drifted")
        if depth.shape != (self._HEIGHT, self._WIDTH):
            raise FormalFeederError("MuJoCo camera depth shape drifted")
        intrinsics = _CameraIntrinsics(
            self._WIDTH,
            self._HEIGHT,
            fx,
            fy,
            cx,
            cy,
        )
        next_sequence = self._sequence
        try:
            info = None
            if not self._intrinsics_published:
                info = encode_camera_intrinsics(
                    intrinsics,
                    timestamp_s=wall_s,
                    sequence=next_sequence,
                )
                next_sequence = _next_sequence(next_sequence, "camera")
            rgb_record = encode_camera_rgb(
                rgb,
                intrinsics=intrinsics,
                timestamp_s=wall_s,
                sequence=next_sequence,
            )
            next_sequence = _next_sequence(next_sequence, "camera")
            depth_record = encode_camera_depth(
                depth,
                intrinsics=intrinsics,
                timestamp_s=wall_s,
                sequence=next_sequence,
            )
            next_sequence = _next_sequence(next_sequence, "camera")
        except (TypeError, ValueError) as exc:
            raise FormalFeederError(f"MuJoCo camera frame is invalid: {exc}") from exc
        timestamp_ns = int(wall_s * 1_000_000_000)
        calibration = {
            "timestamp_ns": timestamp_ns,
            "width": self._WIDTH,
            "height": self._HEIGHT,
            "frame_id": self._FRAME_ID,
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
        }
        if not self._intrinsics_published:
            assert info is not None
            self._client.write(info.wire)
            self._writers["info"].publish(
                **calibration,
                stride=0,
                encoding="camera_info",
                payload=b"",
            )
            self._intrinsics_published = True
        self._client.write(rgb_record.wire)
        self._client.write(depth_record.wire)
        self._writers["color"].publish(
            **calibration,
            stride=self._WIDTH * 3,
            encoding="rgb8",
            payload=rgb_record.payload[CAMERA_RECORD_HEADER.size :],
        )
        self._writers["depth"].publish(
            **calibration,
            stride=self._WIDTH * 2,
            encoding="16UC1",
            payload=depth_record.payload[CAMERA_RECORD_HEADER.size :],
        )
        self._sequence = next_sequence

    def close(self) -> None:
        failed = False
        for writer in self._writers.values():
            try:
                writer.close()
            except Exception:
                failed = True
        if failed:
            raise FormalFeederError("camera SHM writer cleanup failed")


class _RecordPublisher:
    """Bounded single-writer queue for one native sensor endpoint."""

    _QUEUE_LIMIT = 512
    _SHUTDOWN_TIMEOUT_S = 4.0

    def __init__(
        self,
        *,
        name: str,
        client: SensorPublisherClient,
        stats: _StreamStats | None,
        write_lock: Any | None = None,
    ) -> None:
        self._name = name
        self._client = client
        self._stats = stats
        self._write_lock = write_lock
        self._records: queue.Queue[bytes | object] = queue.Queue(
            maxsize=self._QUEUE_LIMIT
        )
        self._failure_lock = threading.Lock()
        self._failure: BaseException | None = None
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name=f"mujoco-{name}-publisher",
            daemon=True,
        )
        self._thread.start()

    def enqueue(self, wire: bytes) -> None:
        try:
            if self._closed:
                raise FormalFeederError(f"{self._name} publisher is closed")
            self.raise_if_failed()
            self._records.put_nowait(wire)
        except Exception as exc:
            if self._stats is not None:
                self._stats.drop_one_if_pending()
            if isinstance(exc, queue.Full):
                raise FormalFeederError(
                    f"{self._name} publisher queue is full"
                ) from exc
            raise

    def raise_if_failed(self) -> None:
        with self._failure_lock:
            failure = self._failure
        if failure is not None:
            raise FormalFeederError(
                f"{self._name} publisher failed: {failure}"
            ) from failure

    def close(self) -> None:
        if self._closed:
            self.raise_if_failed()
            return
        self._closed = True
        if self._thread.is_alive():
            try:
                self._records.put(_LIDAR_STOP, timeout=self._SHUTDOWN_TIMEOUT_S)
            except queue.Full as exc:
                raise FormalFeederError(
                    f"{self._name} publisher queue did not drain"
                ) from exc
            self._thread.join(timeout=self._SHUTDOWN_TIMEOUT_S)
        if self._thread.is_alive():
            raise FormalFeederError(f"{self._name} publisher did not stop")
        self.raise_if_failed()

    def _run(self) -> None:
        while True:
            record = self._records.get()
            if record is _LIDAR_STOP:
                return
            try:
                if not isinstance(record, bytes):
                    raise FormalFeederError(
                        f"{self._name} publisher received an invalid record"
                    )
                if self._write_lock is None:
                    self._client.write(record)
                else:
                    with self._write_lock:
                        self._client.write(record)
                if self._stats is not None:
                    self._stats.published()
            except BaseException as exc:
                with self._failure_lock:
                    if self._failure is None:
                        self._failure = exc
                if self._stats is not None:
                    self._stats.drop_pending()
                return


class _LidarPublisher:
    """Scan immutable MuJoCo snapshots without stalling the physics clock."""

    _QUEUE_LIMIT = 512
    _SHUTDOWN_TIMEOUT_S = 4.0

    def __init__(
        self,
        *,
        client: SensorPublisherClient,
        engine: Any,
        samples_per_frame: int,
        max_points: int,
        publish_registered_cloud_fixture: bool,
        navigation_fixture_raw_overlay: bool,
        stats: _StreamStats,
    ) -> None:
        self._client = client
        self._engine = engine
        self._samples_per_frame = samples_per_frame
        self._max_points = max_points
        self._publish_registered_cloud_fixture = publish_registered_cloud_fixture
        self._navigation_fixture_raw_overlay = navigation_fixture_raw_overlay
        self._stats = stats
        self._write_lock = threading.Lock()
        self._odom_publisher = _RecordPublisher(
            name="odom",
            client=client,
            stats=None,
            write_lock=self._write_lock,
        )
        self._tasks: queue.Queue[_LidarFrame | object] = queue.Queue(
            maxsize=self._QUEUE_LIMIT
        )
        self._failure_lock = threading.Lock()
        self._failure: BaseException | None = None
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name="mujoco-lidar-publisher",
            daemon=True,
        )
        self._thread.start()

    def enqueue_odom(self, wire: bytes) -> None:
        self._odom_publisher.enqueue(wire)

    def enqueue_frame(self, frame: _LidarFrame) -> None:
        try:
            self._enqueue(frame)
        except Exception:
            self._stats.drop_one_if_pending()
            raise

    def raise_if_failed(self) -> None:
        self._odom_publisher.raise_if_failed()
        with self._failure_lock:
            failure = self._failure
        if failure is not None:
            raise FormalFeederError(f"LiDAR publisher failed: {failure}") from failure

    def close(self) -> None:
        if self._closed:
            self.raise_if_failed()
            return
        self._closed = True
        odom_failure: BaseException | None = None
        try:
            self._odom_publisher.close()
        except BaseException as exc:
            odom_failure = exc
        if self._thread.is_alive():
            try:
                self._tasks.put(_LIDAR_STOP, timeout=self._SHUTDOWN_TIMEOUT_S)
            except queue.Full as exc:
                raise FormalFeederError("LiDAR publisher queue did not drain") from exc
            self._thread.join(timeout=self._SHUTDOWN_TIMEOUT_S)
        if self._thread.is_alive():
            raise FormalFeederError("LiDAR publisher did not stop")
        self.raise_if_failed()
        if odom_failure is not None:
            raise FormalFeederError(
                f"odometry publisher did not stop: {odom_failure}"
            ) from odom_failure

    def _enqueue(self, task: _LidarFrame) -> None:
        if self._closed:
            raise FormalFeederError("LiDAR publisher is closed")
        self.raise_if_failed()
        try:
            self._tasks.put_nowait(task)
        except queue.Full as exc:
            raise FormalFeederError("LiDAR publisher queue is full") from exc

    def _run(self) -> None:
        while True:
            task = self._tasks.get()
            if task is _LIDAR_STOP:
                return
            try:
                if isinstance(task, _LidarFrame):
                    self._publish_frame(task)
                else:
                    raise FormalFeederError("LiDAR publisher received an invalid task")
            except BaseException as exc:
                with self._failure_lock:
                    if self._failure is None:
                        self._failure = exc
                if isinstance(task, _LidarFrame):
                    self._stats.drop_pending()
                return

    def _publish_frame(self, task: _LidarFrame) -> None:
        world_points = _bounded_xyzi(
            self._engine.get_lidar_points_from_snapshot(
                task.snapshot,
                sample_count=self._samples_per_frame,
            ),
            self._max_points,
        )
        sensor_points = _bounded_xyzi(
            world_xyzi_to_sensor_xyzi(
                self._engine,
                world_points,
                data=task.snapshot,
            ),
            self._max_points,
        )
        scan = _Subscan(task.frame_start_s, sensor_points, world_points)
        frame = _build_livox_frame(
            (scan,),
            frame_start_s=task.frame_start_s,
            frame_timestamp_ns=int(
                max(
                    0.0,
                    task.wall_s
                    - max(0.0, task.monotonic_s - task.frame_start_s),
                )
                * 1_000_000_000
            ),
            sequence=task.sequence,
            max_points=self._max_points,
            scan_duration_ns=int(1_000_000_000 / self._stats.expected_hz),
        )
        scan_wire = encode_scan(frame).wire
        registered_wire: bytes | None = None
        if self._publish_registered_cloud_fixture:
            if task.registered_sequence is None:
                raise FormalFeederError("registered cloud sequence is missing")
            body_points, _ = navigation_fixture_registered_body_points(
                world_xyzi_to_body_xyzi(task.state, world_points),
                task.state,
                max_points=self._max_points,
                raw_overlay_enabled=self._navigation_fixture_raw_overlay,
            )
            registered_wire = encode_registered_cloud(
                body_points,
                timestamp_ns=int(task.wall_s * 1_000_000_000),
                sequence=task.registered_sequence,
            ).wire
        with self._write_lock:
            self._client.write(scan_wire)
            if registered_wire is not None:
                self._client.write(registered_wire)
        self._stats.published()


class _SensorPipeline:
    """One absolute-deadline IMU/LiDAR stream with honest frame drops."""

    def __init__(
        self,
        *,
        services: _Services,
        lidar: SensorPublisherClient,
        imu: SensorPublisherClient,
        engine: Any,
        imu_hz: float,
        lidar_hz: float,
        samples_per_frame: int,
        max_points: int,
        publish_odom_prior: bool,
        publish_registered_cloud_fixture: bool,
        navigation_fixture_raw_overlay: bool,
        started_s: float,
        started_wall_s: float,
        imu_stats: _StreamStats | None = None,
        lidar_stats: _StreamStats | None = None,
    ) -> None:
        self._services = services
        self._lidar = lidar
        self._imu = imu
        self._engine = engine
        self._imu_period_s = 1.0 / imu_hz
        self._lidar_period_s = 1.0 / lidar_hz
        self._imu_stats = imu_stats or _StreamStats(expected_hz=imu_hz)
        self._lidar_stats = lidar_stats or _StreamStats(expected_hz=lidar_hz)
        self._next_lidar_s = started_s + self._lidar_period_s
        if not math.isfinite(started_wall_s) or started_wall_s <= 0.0:
            raise FormalFeederError("wall clock is invalid")
        self._started_s = started_s
        self._started_wall_s = started_wall_s
        self._samples_per_frame = max(1, samples_per_frame)
        self._max_points = max_points
        self._publish_odom_prior = publish_odom_prior
        self._publish_registered_cloud_fixture = publish_registered_cloud_fixture
        self._navigation_fixture_raw_overlay = navigation_fixture_raw_overlay
        self._imu_sequence = 0
        self._odom_sequence = 0
        self._lidar_sequence = 0
        self._registered_sequence = 0
        self._imu_publisher = _RecordPublisher(
            name="imu",
            client=imu,
            stats=self._imu_stats,
        )
        self._lidar_publisher = _LidarPublisher(
            client=lidar,
            engine=engine,
            samples_per_frame=self._samples_per_frame,
            max_points=self._max_points,
            publish_registered_cloud_fixture=publish_registered_cloud_fixture,
            navigation_fixture_raw_overlay=navigation_fixture_raw_overlay,
            stats=self._lidar_stats,
        )

    def publish_step(self, state: Any, *, tick: _Tick) -> None:
        self._imu_publisher.raise_if_failed()
        self._lidar_publisher.raise_if_failed()
        monotonic_s = tick.due_s
        if not math.isfinite(monotonic_s) or monotonic_s < self._started_s:
            raise FormalFeederError("sensor monotonic time is invalid")
        wall_s = self._started_wall_s + (tick.now_s - self._started_s)
        imu = Imu(
            orientation=Quaternion(
                float(state.orientation[0]),
                float(state.orientation[1]),
                float(state.orientation[2]),
                float(state.orientation[3]),
            ),
            angular_velocity=Vector3(
                float(state.imu_gyro[0]),
                float(state.imu_gyro[1]),
                float(state.imu_gyro[2]),
            ),
            linear_acceleration=Vector3(*(float(value) for value in sensor_specific_force_body(state))),
            ts=wall_s,
            frame_id=topic_default_frame_id(TOPICS.imu),
        )
        self._imu_stats.due(
            count=tick.skipped + 1,
            dropped=tick.skipped,
            lateness_s=max(0.0, tick.now_s - tick.due_s),
        )
        self._imu_publisher.enqueue(
            encode_imu(imu, sequence=self._imu_sequence).wire
        )
        self._imu_sequence = _next_sequence(self._imu_sequence, "IMU")
        if self._publish_odom_prior:
            self._lidar_publisher.enqueue_odom(
                encode_odom_prior(
                    state,
                    timestamp_s=wall_s,
                    sequence=self._odom_sequence,
                ).wire
            )
            self._odom_sequence = _next_sequence(self._odom_sequence, "odometry")

        if monotonic_s + 1e-12 < self._next_lidar_s:
            return

        due_s = self._next_lidar_s
        due_count = max(
            1,
            math.floor((monotonic_s - due_s) / self._lidar_period_s) + 1,
        )
        self._lidar_stats.due(
            count=due_count,
            dropped=due_count if due_count > 1 else 0,
            lateness_s=max(0.0, monotonic_s - due_s),
        )
        if due_count > 1:
            for _ in range(due_count):
                self._lidar_sequence = _next_sequence(self._lidar_sequence, "LiDAR")
            self._next_lidar_s += due_count * self._lidar_period_s
            return

        frame_start_s = self._next_lidar_s - self._lidar_period_s
        try:
            snapshot = self._engine.capture_lidar_snapshot()
        except Exception:
            self._lidar_stats.drop_current()
            raise
        self._lidar_publisher.enqueue_frame(
            _LidarFrame(
                snapshot=snapshot,
                state=state,
                frame_start_s=frame_start_s,
                monotonic_s=monotonic_s,
                wall_s=wall_s,
                sequence=self._lidar_sequence,
                registered_sequence=(
                    self._registered_sequence
                    if self._publish_registered_cloud_fixture
                    else None
                ),
            )
        )
        self._lidar_sequence = _next_sequence(self._lidar_sequence, "LiDAR")
        if self._publish_registered_cloud_fixture:
            self._registered_sequence = _next_sequence(self._registered_sequence, "registered cloud")
        self._next_lidar_s += self._lidar_period_s

    def close(self) -> None:
        failures: list[Exception] = []
        for publisher in (self._imu_publisher, self._lidar_publisher):
            try:
                publisher.close()
            except Exception as exc:
                failures.append(exc)
        if failures:
            raise FormalFeederError("sensor publisher cleanup failed") from failures[0]


def _next_sequence(value: int, label: str) -> int:
    if value >= _UINT32_MAX:
        raise FormalFeederError(f"{label} sequence exhausted")
    return value + 1


def _bounded_xyzi(points: Any, maximum: int) -> Any:
    array = np.asarray(points, dtype=np.float32)
    if array.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if array.ndim != 2 or array.shape[1] < 3:
        raise FormalFeederError("MuJoCo LiDAR returned an invalid point array")
    if array.shape[1] == 3:
        array = np.column_stack((array, np.full((len(array),), 100.0, dtype=np.float32)))
    else:
        array = array[:, :4]
    if not bool(np.isfinite(array).all()):
        raise FormalFeederError("MuJoCo LiDAR returned non-finite points")
    if len(array) > maximum:
        stride = math.ceil(len(array) / maximum)
        array = array[::stride][:maximum]
    return array.astype(np.float32, copy=False)


def _build_livox_frame(
    subscans: Sequence[_Subscan],
    *,
    frame_start_s: float,
    frame_timestamp_ns: int,
    sequence: int,
    max_points: int,
    scan_duration_ns: int,
) -> LivoxPointFrame:
    nonempty = [sample for sample in subscans if len(sample.sensor_points)]
    if nonempty:
        points = np.concatenate([sample.sensor_points for sample in nonempty], axis=0)
        offsets = np.concatenate(
            [
                np.full(
                    (len(sample.sensor_points),),
                    max(0.0, sample.monotonic_s - frame_start_s),
                    dtype=np.float64,
                )
                for sample in nonempty
            ]
        )
    else:
        points = np.zeros((0, 4), dtype=np.float32)
        offsets = np.zeros((0,), dtype=np.float64)
    if len(points) > max_points:
        stride = math.ceil(len(points) / max_points)
        points = points[::stride][:max_points]
        offsets = offsets[::stride][:max_points]
    raw = np.zeros(len(points), dtype=POINT_DTYPE)
    if len(points):
        raw["x"] = points[:, 0]
        raw["y"] = points[:, 1]
        raw["z"] = points[:, 2]
        raw["intensity"] = points[:, 3]
        offset_ns = np.minimum(
            np.asarray(offsets * 1_000_000_000, dtype=np.uint64),
            max(0, scan_duration_ns - 1),
        )
        raw["offset_time_ns"] = offset_ns.astype(np.uint32)
    return LivoxPointFrame(
        points=raw,
        timestamp_ns=frame_timestamp_ns,
        sequence=sequence,
    )


def _velocity(
    command: DriverBridgeCommand,
    *,
    max_linear_mps: float,
    max_angular_rps: float,
) -> VelocityCommand:
    values = (command.walk_x, command.walk_y, command.walk_z)
    if not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in values):
        raise FormalFeederError("driver command is outside normalized bounds")
    return VelocityCommand(
        linear_x=command.walk_x * max_linear_mps,
        linear_y=command.walk_y * max_linear_mps,
        angular_z=command.walk_z * max_angular_rps,
    )


def _required_mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise FormalFeederError(f"RunPlan {field} must be an object")
    return value


def _required_text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise FormalFeederError(f"RunPlan {field} must be a non-empty string")
    return value


def _positive_finite(value: Any, field: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, int | float)
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise FormalFeederError(f"RunPlan {field} must be positive finite")
    return float(value)


def _finite_vector(value: Any, size: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise FormalFeederError(f"RunPlan {field} must contain {size} numbers")
    if any(
        isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)) for item in value
    ):
        raise FormalFeederError(f"RunPlan {field} must contain finite numbers")
    return tuple(float(item) for item in value)


def _single_sensor_stream(
    sensor_plan: dict[str, Any],
    stream_class: str,
) -> tuple[dict[str, Any], float]:
    streams = _required_mapping(
        sensor_plan.get("streams"),
        "launch.simulation.sensor_plan.streams",
    )
    values = streams.get(stream_class)
    if not isinstance(values, list) or len(values) != 1:
        raise FormalFeederError(f"RunPlan sensor_plan must declare exactly one {stream_class} stream")
    stream = _required_mapping(
        values[0],
        f"launch.simulation.sensor_plan.streams.{stream_class}[0]",
    )
    rate = _positive_finite(
        stream.get("rate_hz"),
        f"sensor_plan {stream_class} rate_hz",
    )
    return stream, rate


def _single_robot_config(
    *,
    physics_plan: dict[str, Any],
    control_plan: dict[str, Any],
    sensor_plan: dict[str, Any],
    sensors_enabled: bool,
) -> _RobotRuntimeConfig:
    robots = physics_plan.get("robots")
    if not isinstance(robots, list) or len(robots) != 1:
        raise FormalFeederError("formal feeder requires exactly one physics robot")
    robot = _required_mapping(robots[0], "physics_plan.robots[0]")
    package = _required_mapping(robot.get("package"), "physics_plan.robots[0].package")
    package_manifest = PurePosixPath(
        _required_text(package.get("manifest"), "physics_plan.robots[0].package.manifest")
    )
    instance_id = _required_text(
        robot.get("instance_id"),
        "physics_plan.robots[0].instance_id",
    )
    model = _required_mapping(robot.get("model"), "physics_plan.robots[0].model")
    spawn = _required_mapping(robot.get("spawn"), "physics_plan.robots[0].spawn")
    frames = robot.get("frames")
    if not isinstance(frames, list):
        raise FormalFeederError("RunPlan physics robot frames must be a list")
    body_names = [
        _required_text(frame.get("name"), "physics robot body frame name")
        for raw_frame in frames
        for frame in [_required_mapping(raw_frame, "physics robot frame")]
        if frame.get("role") == "body"
    ]
    if len(body_names) != 1:
        raise FormalFeederError("RunPlan physics robot must declare exactly one body frame")
    attach_root = _required_text(
        model.get("attach_root"),
        "physics robot model attach_root",
    )
    if attach_root != body_names[0]:
        raise FormalFeederError("RunPlan robot attach_root does not match its body frame")
    declared_frames = {
        _required_text(frame.get("name"), "physics robot frame name"): frame
        for raw_frame in frames
        for frame in [_required_mapping(raw_frame, "physics robot frame")]
    }

    controllers = control_plan.get("controllers")
    if not isinstance(controllers, list) or len(controllers) != 1:
        raise FormalFeederError("formal feeder requires exactly one robot controller")
    controller = _required_mapping(controllers[0], "control_plan.controllers[0]")
    if controller.get("instance_id") != instance_id:
        raise FormalFeederError("RunPlan controller does not own the physics robot")
    policy = _required_mapping(controller.get("policy"), "control_plan controller policy")
    timing = _required_mapping(controller.get("timing"), "control_plan controller timing")
    actuators = controller.get("actuator_channels")
    if (
        not isinstance(actuators, list)
        or not actuators
        or any(not isinstance(value, str) or not value or value != value.strip() for value in actuators)
        or len(set(actuators)) != len(actuators)
    ):
        raise FormalFeederError("RunPlan controller actuator_channels are invalid")

    lidar_body = ""
    lidar_site = ""
    if sensors_enabled:
        imu_stream, _ = _single_sensor_stream(sensor_plan, "imu")
        lidar_stream, _ = _single_sensor_stream(sensor_plan, "mid360")
        if imu_stream.get("instance_id") != instance_id or lidar_stream.get("instance_id") != instance_id:
            raise FormalFeederError("RunPlan sensor streams do not belong to the physics robot")
        lidar_frame = _required_text(
            lidar_stream.get("frame_id"),
            "sensor_plan mid360 frame_id",
        )
        namespace_prefix = f"{instance_id}/"
        if not lidar_frame.startswith(namespace_prefix):
            raise FormalFeederError("RunPlan mid360 frame is outside the robot namespace")
        lidar_body = lidar_frame[len(namespace_prefix) :]
        raycast_frame = _required_text(
            lidar_stream.get("raycast_frame_stable_id"),
            "sensor_plan mid360 raycast_frame_stable_id",
        )
        if not raycast_frame.startswith(namespace_prefix):
            raise FormalFeederError("RunPlan mid360 raycast frame is outside the robot namespace")
        lidar_site = raycast_frame[len(namespace_prefix) :]
    else:
        origins = [
            (
                _required_text(frame.get("name"), "physics sensor origin frame"),
                _required_text(
                    frame.get("parent_frame"),
                    "physics sensor origin parent",
                ),
            )
            for raw_frame in frames
            for frame in [_required_mapping(raw_frame, "physics robot frame")]
            if frame.get("role") == "sensor_origin"
        ]
        if len(origins) != 1:
            raise FormalFeederError("RunPlan physics robot must declare one sensor origin for engine binding")
        lidar_site, lidar_body = origins[0]
    if lidar_body not in declared_frames:
        raise FormalFeederError("RunPlan LiDAR body is not declared by the physics robot")
    site_frame = declared_frames.get(lidar_site)
    if site_frame is None or site_frame.get("role") != "sensor_origin":
        raise FormalFeederError("RunPlan LiDAR raycast frame is not a declared sensor origin")
    if site_frame.get("parent_frame") != lidar_body:
        raise FormalFeederError("RunPlan LiDAR raycast frame is not attached to the LiDAR body")

    position = _finite_vector(
        spawn.get("position_m"),
        3,
        "physics_plan robot spawn.position_m",
    )
    orientation = _finite_vector(
        spawn.get("quaternion_wxyz"),
        4,
        "physics_plan robot spawn.quaternion_wxyz",
    )
    quaternion_norm = math.sqrt(sum(value * value for value in orientation))
    if not math.isfinite(quaternion_norm) or abs(quaternion_norm - 1.0) > 1e-6:
        raise FormalFeederError("RunPlan robot spawn quaternion must be normalized")
    return _RobotRuntimeConfig(
        instance_id=instance_id,
        package_root=package_manifest.parent.as_posix(),
        model=_required_text(model.get("mjcf"), "physics robot model mjcf"),
        initial_keyframe=(
            None
            if model.get("initial_keyframe") is None
            else _required_text(
                model.get("initial_keyframe"),
                "physics robot model initial_keyframe",
            )
        ),
        base_body=body_names[0],
        lidar_body=lidar_body,
        lidar_site=lidar_site,
        position_m=(position[0], position[1], position[2]),
        quaternion_wxyz=(
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        ),
        policy=_required_text(policy.get("artifact"), "controller policy artifact"),
        policy_hz=_positive_finite(
            timing.get("inference_hz"),
            "controller timing.inference_hz",
        ),
        actuator_names=tuple(actuators),
    )


def _run_plan_runtime_config(
    args: argparse.Namespace,
    plan: RunPlan,
) -> _RuntimeConfig:
    override_fields = {
        "session_id": "--session-id",
        "model_generation": "--model-generation",
        "reset_generation": "--reset-generation",
        "world": "--world",
        "imu_hz": "--imu-hz",
        "lidar_hz": "--lidar-hz",
    }
    supplied = [option for field, option in override_fields.items() if getattr(args, field) is not None]
    if str(args.start):
        supplied.append("--start")
    if args.policy_path is not None:
        supplied.append("--policy-path")
    if supplied:
        raise FormalFeederError(
            "Product RunPlan child rejects runtime identity or physics overrides: " + ", ".join(sorted(supplied))
        )
    if plan.env != "sim" or plan.controller != "subprocess":
        raise FormalFeederError("formal feeder RunPlan identity does not match direct-child identity")
    simulation = plan.simulation
    session = _required_mapping(
        simulation.get("session"),
        "launch.simulation.session",
    )
    runtime = _required_mapping(
        session.get("runtime"),
        "launch.simulation.session.runtime",
    )
    runtime_mode = _required_text(runtime.get("mode"), "simulation runtime mode")
    physics_plan = _required_mapping(
        simulation.get("physics_plan"),
        "launch.simulation.physics_plan",
    )
    session_id = _required_text(physics_plan.get("session_id"), "physics_plan session_id")
    world_entry = _required_mapping(
        physics_plan.get("world"),
        "launch.simulation.physics_plan.world",
    )
    world = _required_text(
        world_entry.get("mjcf"),
        "physics_plan world mjcf",
    )
    global_policy = _required_mapping(
        physics_plan.get("global_policy"),
        "launch.simulation.physics_plan.global_policy",
    )
    physics_timestep_s = _positive_finite(
        global_policy.get("timestep_s"),
        "physics_plan global_policy.timestep_s",
    )
    scenario_plan = simulation.get("scenario_plan")
    if scenario_plan is None:
        model_generation = 0
        reset_generation = 0
        kinematic_entities: tuple[_KinematicEntityConfig, ...] = ()
    else:
        scenario = _required_mapping(
            scenario_plan,
            "launch.simulation.scenario_plan",
        )
        raw_model_generation = scenario.get("model_generation")
        raw_reset_generation = scenario.get("reset_generation")
        if (
            isinstance(raw_model_generation, bool)
            or not isinstance(raw_model_generation, int)
            or raw_model_generation < 0
            or isinstance(raw_reset_generation, bool)
            or not isinstance(raw_reset_generation, int)
            or raw_reset_generation < 0
        ):
            raise FormalFeederError("RunPlan scenario generations are invalid")
        model_generation = raw_model_generation
        reset_generation = raw_reset_generation
        runtime = ScenarioRuntime.from_plan(scenario)
        bindings = dict(runtime.mujoco_kinematic_bindings)
        raw_entities = physics_plan.get("kinematic_entities", [])
        if not isinstance(raw_entities, list):
            raise FormalFeederError("physics_plan kinematic_entities must be a list")
        parsed_entities: list[_KinematicEntityConfig] = []
        for index, raw_entity in enumerate(raw_entities):
            entity = _required_mapping(
                raw_entity,
                f"physics_plan.kinematic_entities[{index}]",
            )
            entity_id = _required_text(entity.get("entity_id"), "kinematic entity_id")
            model = _required_mapping(entity.get("model"), "kinematic entity model")
            attach_root = _required_text(model.get("attach_root"), "kinematic attach_root")
            if bindings.get(entity_id) != f"{entity_id}/{attach_root}":
                raise FormalFeederError("scenario and physics kinematic entity bindings disagree")
            package = _required_mapping(entity.get("package"), "kinematic entity package")
            manifest = PurePosixPath(
                _required_text(package.get("manifest"), "kinematic entity package manifest")
            )
            transform = _required_mapping(
                entity.get("initial_transform"),
                "kinematic entity initial_transform",
            )
            position = _finite_vector(
                transform.get("position_m"),
                3,
                "kinematic entity position_m",
            )
            quaternion = _finite_vector(
                transform.get("quaternion_wxyz"),
                4,
                "kinematic entity quaternion_wxyz",
            )
            parsed_entities.append(
                _KinematicEntityConfig(
                    entity_id=entity_id,
                    package_root=manifest.parent.as_posix(),
                    model=_required_text(model.get("mjcf"), "kinematic entity model mjcf"),
                    attach_root=attach_root,
                    position_m=(position[0], position[1], position[2]),
                    quaternion_wxyz=(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                )
            )
        if set(bindings) != {entity.entity_id for entity in parsed_entities}:
            raise FormalFeederError("scenario and physics kinematic entity sets disagree")
        kinematic_entities = tuple(parsed_entities)
    sensor_plan = _required_mapping(
        simulation.get("sensor_plan"),
        "launch.simulation.sensor_plan",
    )
    control_plan = _required_mapping(
        simulation.get("control_plan"),
        "launch.simulation.control_plan",
    )
    sensors_enabled = plan.has_process("lidar")
    camera_setting = plan.host_config.get("enable_camera", False)
    if not isinstance(camera_setting, bool):
        raise FormalFeederError("host_config enable_camera must be bool")
    camera_provider = plan.has_process("camera")
    if camera_setting and not camera_provider:
        raise FormalFeederError("host_config enable_camera requires a camera process provider")
    if camera_setting:
        _, rgb_hz = _single_sensor_stream(sensor_plan, "rgb")
        _, depth_hz = _single_sensor_stream(sensor_plan, "depth")
        if rgb_hz != depth_hz:
            raise FormalFeederError("sensor_plan RGB and depth rate_hz must match")
        camera_hz = rgb_hz
    else:
        camera_hz = None
    if plan.has_process("slam") and not sensors_enabled:
        raise FormalFeederError("simulation truth localization requires lidar")
    required_roles = (
        *((LIDAR_ROLE, IMU_ROLE) if sensors_enabled else ()),
        *((CAMERA_ROLE,) if camera_setting else ()),
    )
    process_names = {process.name for process in plan.processes}
    missing_roles = [role for role in required_roles if role not in process_names]
    if missing_roles:
        raise FormalFeederError("RunPlan is missing required sensor endpoint: " + ", ".join(missing_roles))
    slam_colocated = bool(
        plan.has_process("slam")
        and plan.process("lidar").name == plan.process("slam").name
    )
    slam_config = plan.native_process_environment.get("LINGTU_SLAM_CONFIG", "").replace("\\", "/")
    publish_odom_prior = bool(
        slam_colocated
        or (
            plan.has_process("slam")
            and slam_config.endswith("/sim_mid360.yaml")
        )
    )
    robot = _single_robot_config(
        physics_plan=physics_plan,
        control_plan=control_plan,
        sensor_plan=sensor_plan,
        sensors_enabled=sensors_enabled,
    )
    navigation_fixture_raw_overlay = True
    if sensors_enabled:
        _, imu_hz = _single_sensor_stream(sensor_plan, "imu")
        lidar_stream, lidar_hz = _single_sensor_stream(sensor_plan, "mid360")
        raw_overlay = lidar_stream.get("navigation_fixture_raw_overlay", True)
        if not isinstance(raw_overlay, bool):
            raise FormalFeederError("sensor_plan mid360 navigation_fixture_raw_overlay must be bool")
        navigation_fixture_raw_overlay = raw_overlay
        step_period_s = 1.0 / imu_hz
    else:
        imu_hz = None
        lidar_hz = None
        step_period_s = physics_timestep_s
    return _RuntimeConfig(
        session_id=session_id,
        model_generation=model_generation,
        reset_generation=reset_generation,
        world=world,
        physics_timestep_s=physics_timestep_s,
        step_period_s=step_period_s,
        imu_hz=imu_hz,
        lidar_hz=lidar_hz,
        sensors_enabled=sensors_enabled,
        camera_enabled=camera_setting,
        camera_hz=camera_hz,
        sensor_roles=required_roles,
        publish_odom_prior=publish_odom_prior,
        publish_registered_cloud_fixture=slam_colocated,
        navigation_fixture_raw_overlay=navigation_fixture_raw_overlay,
        viewer_enabled=runtime_mode == "preview",
        robot=robot,
        scenario_plan=(None if scenario_plan is None else scenario),
        kinematic_entities=kinematic_entities,
    )


def _step(
    engine: Any,
    velocity: VelocityCommand,
    *,
    period_s: float,
    step_seq: int,
    scenario: _ScenarioFeed | None = None,
) -> tuple[Any, int]:
    if step_seq >= (1 << 64) - 1:
        raise FormalFeederError("physics step sequence exhausted")
    next_step = step_seq + 1
    if scenario is not None:
        scenario.before_step(period_s)
    state = engine.step_sensor_tick(velocity, period_s)
    return state, next_step


def _readiness_payload(
    *,
    product_session_id: str,
    product: str,
    config: _RuntimeConfig,
) -> dict[str, Any]:
    endpoints = [{"protocol": "driver-v2", "role": "driver_bridge"}]
    endpoints.extend({"protocol": "ltu1-v1", "role": role} for role in config.sensor_roles)
    return {
        "backend": "mujoco",
        "endpoints": endpoints,
        "env": "sim",
        "product_session_id": product_session_id,
        "first_physics_step_applied": True,
        "model_generation": config.model_generation,
        "process": _PROCESS_NAME,
        "product": product,
        "protocol": "mujoco-feeder-v1",
        "ready": True,
        "reset_generation": config.reset_generation,
        "role": _PROCESS_NAME,
        "schema": SIM_FEEDER_SCHEMA,
        "session_id": config.session_id,
    }


def _stop_payload(
    *,
    product_session_id: str,
    product: str,
    stopped: DriverBridgeStoppedEvidence,
) -> dict[str, Any]:
    return {
        "schema": MOTION_STOP_SCHEMA,
        "product_session_id": product_session_id,
        "product": product,
        "process": _PROCESS_NAME,
        "outcome": "zero_applied",
        "bridge_boot_id": stopped.bridge_boot_id,
        "controller_boot_id": stopped.controller_boot_id,
        "bridge_command_seq": stopped.bridge_command_seq,
        "applied_step_seq": stopped.applied_step_seq,
        "command_kind": stopped.kind,
        "walk_x": stopped.walk_x,
        "walk_y": stopped.walk_y,
        "walk_z": stopped.walk_z,
        "terminal_ack": stopped.terminal_ack,
    }


def _shutdown(
    *,
    session: DriverBridgeSession,
    engine: Any,
    current_velocity: VelocityCommand,
    current_command: DriverBridgeCommand,
    previous_state: Any,
    motion: _PhysicalMotionEvidence,
    pending_ready: bool,
    step_seq: int,
    period_s: float,
    max_linear_mps: float,
    max_angular_rps: float,
    scenario: _ScenarioFeed | None = None,
    emergency: bool = False,
) -> tuple[DriverBridgeStoppedEvidence, int]:
    if pending_ready:
        state, step_seq = _step(
            engine,
            VelocityCommand() if emergency else current_velocity,
            period_s=period_s,
            step_seq=step_seq,
            scenario=scenario,
        )
        session.heartbeat(step_seq=step_seq)
        session.confirm_ready()
        if not emergency:
            motion.observe(
                current_command,
                before=previous_state,
                after=state,
                step_seq=step_seq,
            )
        previous_state = state

    while True:
        command = session.begin_deactivate()
        current_command = command
        if emergency and command.kind != "deactivate_zero":
            raise FormalFeederError("emergency shutdown refuses to physically replay pending navigation")
        velocity = _velocity(
            command,
            max_linear_mps=max_linear_mps,
            max_angular_rps=max_angular_rps,
        )
        state, step_seq = _step(
            engine,
            velocity,
            period_s=period_s,
            step_seq=step_seq,
            scenario=scenario,
        )
        session.complete_step(command, step_seq=step_seq)
        if not emergency:
            motion.observe(
                current_command,
                before=previous_state,
                after=state,
                step_seq=step_seq,
            )
        previous_state = state
        if command.kind == "deactivate_zero":
            break
        if command.kind != "nav":
            raise FormalFeederError("shutdown received a non-drainable command")
    return session.wait_stopped(), step_seq


def _execute(
    args: argparse.Namespace,
    plan: RunPlan,
    session_root: Path,
    product_session_id: str,
    stop_event: threading.Event,
    services: _Services,
) -> int:
    engine: Any | None = None
    viewer: Any | None = None
    viewer_input: ViewerInput | None = None
    sensors: dict[str, SensorPublisherClient] = {}
    camera: _CameraPipeline | None = None
    sensor_pipeline: _SensorPipeline | None = None
    scenario: _ScenarioFeed | None = None
    status: _Status | None = None
    driver: DriverBridgeClient | None = None
    session: DriverBridgeSession | None = None
    current_velocity: VelocityCommand | None = None
    current_command: DriverBridgeCommand | None = None
    previous_state: Any | None = None
    motion: _PhysicalMotionEvidence | None = None
    pending_ready = False
    step_seq = 0
    period_s: float | None = None
    max_linear_mps = float(args.max_linear_mps)
    max_angular_rps = float(args.max_angular_rps)
    protocol_active = False
    stop_completed = False
    result = 1
    try:
        config = _run_plan_runtime_config(args, plan)
        period_s = config.step_period_s
        world_path, robot_path, policy_path = services.snapshot_artifacts(
            session_root,
            config,
        )
        engine = services.build_engine(
            world=world_path,
            drive_mode=str(args.drive_mode),
            start=list(config.robot.position_m),
            start_orientation_wxyz=list(config.robot.quaternion_wxyz),
            mujoco_memory=str(args.mujoco_memory),
            robot_xml=robot_path,
            base_body_name=config.robot.base_body,
            lidar_body_name=config.robot.lidar_body,
            lidar_site_name=config.robot.lidar_site,
            physics_timestep_s=config.physics_timestep_s,
            controller_actuator_names=list(config.robot.actuator_names),
            mid360_pattern=args.mid360_pattern,
            mid360_samples_per_frame=int(args.mid360_samples_per_frame),
            lidar_backend=str(args.lidar_backend),
            mujoco_lidar_backend=str(args.mujoco_lidar_backend),
            require_product_lidar_backend=True,
            policy_path=policy_path,
            policy_freq_hz=config.robot.policy_hz,
            max_linear_vel=float(args.max_linear_mps),
            max_angular_vel=float(args.max_angular_rps),
            initial_keyframe=config.robot.initial_keyframe,
            camera_configs=(
                [
                    CameraConfig(
                        name="front_camera",
                        width=640,
                        height=480,
                        fovy=60.0,
                        render_depth=True,
                        fps=float(cast(float, config.camera_hz)),
                    )
                ]
                if config.camera_enabled
                else None
            ),
        )
        if config.scenario_plan is not None:
            scenario = _ScenarioFeed(
                engine=engine,
                runtime=ScenarioRuntime.from_plan(config.scenario_plan),
                services=services,
                session_root=session_root,
                config=config,
            )
            scenario.start()
        viewer_requested = args.viewer
        if viewer_requested is None:
            viewer_requested = config.viewer_enabled
        if viewer_requested:
            viewer = launch_presentation_viewer(engine.model, engine.data)
            focus_presentation_viewer(viewer, config.robot.position_m, initialize=True)
            viewer.sync()
            viewer_input = viewer_input_from_run_plan(plan)
            if viewer_input is not None:
                viewer_input.start()
        endpoint_files = dict(_SENSOR_ENDPOINTS)
        for role in config.sensor_roles:
            sensors[role] = services.connect_sensor(
                session_root / endpoint_files[role],
                role=role,
                product_session_id=product_session_id,
                timeout_s=_ENDPOINT_TIMEOUT_S,
            )
        driver = services.connect_driver(
            session_root / "driver.ready.json",
            product_session_id=product_session_id,
            timeout_s=_ENDPOINT_TIMEOUT_S,
        )
        session = services.create_session(
            driver,
            expected_product_session_id=product_session_id,
        )

        started_s = services.monotonic()
        started_wall_s = services.wall_time()
        streams: dict[str, _StreamStats] = {}
        if config.sensors_enabled:
            if config.imu_hz is None or config.lidar_hz is None:
                raise FormalFeederError("enabled sensor pipeline has no RunPlan rates")
            streams["imu"] = _StreamStats(float(config.imu_hz))
            streams["lidar"] = _StreamStats(float(config.lidar_hz))
        if config.camera_enabled:
            if config.camera_hz is None:
                raise FormalFeederError("enabled camera pipeline has no RunPlan rate")
            streams["camera_rgbd"] = _StreamStats(float(config.camera_hz))
        status = _Status(
            services=services,
            session_root=session_root,
            product_session_id=product_session_id,
            product=plan.product,
            started_s=started_s,
            streams=streams,
        )
        if config.camera_enabled:
            camera = _CameraPipeline(
                services=services,
                client=sensors[CAMERA_ROLE],
                engine=engine,
                session_root=session_root,
                started_s=started_s,
                started_wall_s=started_wall_s,
                rate_hz=streams["camera_rgbd"].expected_hz,
                stats=streams["camera_rgbd"],
            )
            camera.publish_initial()

        activation = session.activate()
        current_velocity = _velocity(
            activation,
            max_linear_mps=max_linear_mps,
            max_angular_rps=max_angular_rps,
        )
        state, step_seq = _step(
            engine,
            current_velocity,
            period_s=config.step_period_s,
            step_seq=0,
            scenario=scenario,
        )
        _require_stable_pose(state)
        session.complete_step(activation, step_seq=step_seq)
        state, step_seq = _step(
            engine,
            current_velocity,
            period_s=config.step_period_s,
            step_seq=step_seq,
            scenario=scenario,
        )
        _require_stable_pose(state)
        session.heartbeat(step_seq=step_seq)
        session.confirm_ready()
        protocol_active = True
        current_command = activation
        previous_state = state
        motion = _PhysicalMotionEvidence()
        motion.observe_pose(state)

        services.publish_readiness(
            session_root,
            _READINESS_NAME,
            _readiness_payload(
                product_session_id=product_session_id,
                product=plan.product,
                config=config,
            ),
        )
        started_s = services.monotonic()
        started_wall_s = services.wall_time()
        status.begin(started_s)
        if camera is not None:
            camera.begin(started_s=started_s, started_wall_s=started_wall_s)
        status.running(started_s, force=True)
        deadline = _AbsoluteDeadline.start(
            period_s=config.step_period_s,
            now_s=started_s,
        )
        viewer_period_s = 1.0 / float(args.viewer_hz)
        next_viewer_s = started_s
        next_heartbeat_s = started_s + _HEARTBEAT_PERIOD_S
        sensor_pipeline = (
            _SensorPipeline(
                services=services,
                lidar=sensors[LIDAR_ROLE],
                imu=sensors[IMU_ROLE],
                engine=engine,
                imu_hz=streams["imu"].expected_hz,
                lidar_hz=streams["lidar"].expected_hz,
                samples_per_frame=int(args.mid360_samples_per_frame),
                max_points=int(args.max_points),
                publish_odom_prior=config.publish_odom_prior,
                publish_registered_cloud_fixture=(
                    config.publish_registered_cloud_fixture
                ),
                navigation_fixture_raw_overlay=(config.navigation_fixture_raw_overlay),
                started_s=started_s,
                started_wall_s=started_wall_s,
                imu_stats=streams["imu"],
                lidar_stats=streams["lidar"],
            )
            if config.sensors_enabled
            else None
        )
        with services.wait_scope(stop_event) as wait:
            while not stop_event.is_set():
                if viewer is not None and not viewer.is_running():
                    stop_event.set()
                    break
                tick = deadline.wait_next(services, stop_event, wait=wait)
                if tick is None:
                    break
                command: DriverBridgeCommand | None = None
                if not pending_ready and services.command_available(session, driver):
                    command = session.receive_command()
                    current_command = command
                    current_velocity = _velocity(
                        command,
                        max_linear_mps=max_linear_mps,
                        max_angular_rps=max_angular_rps,
                    )
                state, step_seq = _step(
                    engine,
                    current_velocity,
                    period_s=config.step_period_s,
                    step_seq=step_seq,
                    scenario=scenario,
                )
                if command is not None:
                    session.complete_step(command, step_seq=step_seq)
                    pending_ready = True
                elif pending_ready or tick.now_s + 1e-12 >= next_heartbeat_s:
                    session.heartbeat(step_seq=step_seq)
                    next_heartbeat_s = tick.now_s + _HEARTBEAT_PERIOD_S
                    if pending_ready:
                        session.confirm_ready()
                        pending_ready = False
                motion.observe(
                    current_command,
                    before=previous_state,
                    after=state,
                    step_seq=step_seq,
                )
                previous_state = state
                if viewer is not None and tick.now_s + 1e-12 >= next_viewer_s:
                    draw_navigation_paths(
                        viewer,
                        _read_navigation_status(session_root / "nav.status.json"),
                    )
                    focus_presentation_viewer(viewer, state.position)
                    viewer.sync()
                    next_viewer_s = tick.now_s + viewer_period_s
                if sensor_pipeline is not None:
                    sensor_pipeline.publish_step(state, tick=tick)
                if camera is not None:
                    camera.publish_due(
                        monotonic_s=tick.due_s,
                        observed_s=tick.now_s,
                    )
                status.running(tick.now_s)

        measurement_end_s = services.monotonic()
        stopped, step_seq = _shutdown(
            session=session,
            engine=engine,
            current_velocity=current_velocity,
            current_command=current_command,
            previous_state=previous_state,
            motion=motion,
            pending_ready=pending_ready,
            step_seq=step_seq,
            period_s=config.step_period_s,
            max_linear_mps=max_linear_mps,
            max_angular_rps=max_angular_rps,
            scenario=scenario,
        )
        stop_completed = True
        if sensor_pipeline is not None:
            sensor_pipeline.close()
        status.finish("stopped", measurement_end_s)
        services.publish_evidence(
            session_root=session_root,
            target="mujoco_feeder.stop.json",
            payload=_stop_payload(
                product_session_id=product_session_id,
                product=plan.product,
                stopped=stopped,
            ),
        )
        services.publish_motion(
            session_root=session_root,
            payload=motion.payload(
                product_session_id=product_session_id,
                product=plan.product,
                stopped=stopped,
            ),
        )
        result = 0
    except Exception as exc:
        _report_failure(exc)
        failure_end_s = services.monotonic()
        if (
            protocol_active
            and not stop_completed
            and session is not None
            and engine is not None
            and current_velocity is not None
            and current_command is not None
            and previous_state is not None
            and motion is not None
            and period_s is not None
        ):
            try:
                stopped, step_seq = _shutdown(
                    session=session,
                    engine=engine,
                    current_velocity=current_velocity,
                    current_command=current_command,
                    previous_state=previous_state,
                    motion=motion,
                    pending_ready=pending_ready,
                    step_seq=step_seq,
                    period_s=period_s,
                    max_linear_mps=max_linear_mps,
                    max_angular_rps=max_angular_rps,
                    scenario=scenario,
                    emergency=True,
                )
                stop_completed = True
                services.publish_evidence(
                    session_root=session_root,
                    target="mujoco_feeder.stop.json",
                    payload=_stop_payload(
                        product_session_id=product_session_id,
                        product=plan.product,
                        stopped=stopped,
                    ),
                )
            except Exception as shutdown_exc:
                _report_failure(FormalFeederError(f"emergency shutdown failed: {shutdown_exc}"))
        if status is not None:
            try:
                status.finish("failed", failure_end_s)
            except Exception as status_exc:
                _report_failure(FormalFeederError(f"failed feeder status could not be built: {status_exc}"))
        result = 1
    finally:
        cleanup_failed = False
        if viewer_input is not None:
            try:
                viewer_input.close()
            except Exception:
                cleanup_failed = True
        if viewer is not None:
            try:
                viewer.close()
            except Exception:
                cleanup_failed = True
        if sensor_pipeline is not None:
            try:
                sensor_pipeline.close()
            except Exception:
                cleanup_failed = True
        if session is not None:
            try:
                session.close()
            except Exception:
                cleanup_failed = True
        elif driver is not None:
            try:
                driver.close()
            except Exception:
                cleanup_failed = True
        if camera is not None:
            try:
                camera.close()
            except Exception:
                cleanup_failed = True
        for sensor in sensors.values():
            try:
                sensor.close()
            except Exception:
                cleanup_failed = True
        if engine is not None:
            try:
                engine.close()
            except Exception:
                cleanup_failed = True
        if cleanup_failed:
            if status is not None:
                try:
                    status.finish("failed", services.monotonic())
                except Exception as status_exc:
                    _report_failure(FormalFeederError(f"cleanup failure status could not be built: {status_exc}"))
            result = 1
    return result


def main(argv: Sequence[str] | None = None) -> int:
    """Run one formal, Supervisor-owned MuJoCo feeder process."""

    try:
        args = _parser().parse_args(argv)
    except SystemExit as exc:
        return int(cast(int, exc.code))
    stop_event = threading.Event()
    with _installed_stop_handlers(stop_event):
        try:
            plan, session_root, product_session_id = _load_run_plan(os.environ)
        except Exception:
            return 1
        return _execute(
            args,
            plan,
            session_root,
            product_session_id,
            stop_event,
            _SERVICES,
        )


def _reject_duplicate_json_pairs(
    pairs: list[tuple[str, Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate RunPlan JSON key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"invalid RunPlan JSON constant: {value}")


if __name__ == "__main__":
    raise SystemExit(main())
