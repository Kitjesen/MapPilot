"""Validate one resolved session bundle and allocate run-local resources."""

from __future__ import annotations

import json
import os
import re
import stat
import uuid
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Literal, Mapping

import yaml
from sim.contracts import physics_sensor_timebase_violation

STATIC_PLAN_CONTRACTS: tuple[tuple[str, str], ...] = (
    ("physics.plan.json", "lingtu.sim.physics-plan.v1"),
    ("visual.plan.json", "lingtu.sim.visual-plan.v1"),
    ("sensor.plan.json", "lingtu.sim.sensor-plan.v1"),
    ("control.plan.json", "lingtu.sim.control-plan.v1"),
    ("transport.intent.json", "lingtu.sim.transport-intent.v1"),
)
STATIC_PLAN_FILES = tuple(filename for filename, _schema in STATIC_PLAN_CONTRACTS)
_V2_PLAN_SCHEMAS = {
    **dict(STATIC_PLAN_CONTRACTS),
    "physics.plan.json": "lingtu.sim.physics-plan.v2",
    "visual.plan.json": "lingtu.sim.visual-plan.v2",
}
_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_RESOURCE_KEY_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
_SHM_NAME_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,254}\Z")
_COORDINATOR_BOOT_ID = str(uuid.uuid4())
_REPARSE_POINT = 0x0400
_ALLOCATION_KEYS = {
    "schema",
    "run_id",
    "session_id",
    "artifact_root",
    "boot_id",
    "dds_domain",
    "ports",
    "shm",
    "log_dir",
}
_TERMINAL_RUNTIME_STATES = {"STOPPED", "FAILED"}


class RunAllocationErrorCode(str, Enum):
    """Machine-readable failures exposed by the RunAllocation interface."""

    BUNDLE_NOT_FOUND = "bundle_not_found"
    BUNDLE_ARTIFACT_MISSING = "bundle_artifact_missing"
    BUNDLE_ARTIFACT_INVALID = "bundle_artifact_invalid"
    SESSION_ID_MISMATCH = "session_id_mismatch"
    ALLOCATION_INVALID = "allocation_invalid"
    RESOURCE_CONFLICT = "resource_conflict"
    ALLOCATION_EXISTS = "allocation_exists"
    IO_ERROR = "io_error"


class RunAllocationError(RuntimeError):
    """Stable machine-readable failure at the RunAllocation seam."""

    def __init__(
        self,
        code: RunAllocationErrorCode,
        message: str,
        *,
        artifact: str | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.artifact = artifact

    def to_dict(self) -> dict[str, str | None]:
        """Return the fixed v1 error document."""

        return {
            "schema": "lingtu.sim.run-allocation-error.v1",
            "code": self.code.value,
            "message": self.message,
            "artifact": self.artifact,
        }


@dataclass(frozen=True)
class ResolvedSessionBundle:
    """Validated deterministic inputs consumed by the Runtime Coordinator."""

    bundle_dir: Path
    repo_root: Path | None
    session_id: str
    session_spec: Mapping[str, Any]
    plans: Mapping[str, Mapping[str, Any]]


@dataclass(frozen=True)
class RunAllocation:
    """Resources for one run, deliberately excluded from ``session_id``."""

    run_id: str
    run_dir: Path
    artifact_root: Path
    log_dir: Path
    ports: Mapping[str, int]
    shm: Mapping[str, str]
    session_id: str
    boot_id: str
    dds_domain: int

    @property
    def path(self) -> Path:
        """Return the serialized allocation path."""

        return self.run_dir / "run-allocation.json"

    @property
    def shared_memory(self) -> Mapping[str, str]:
        """Compatibility name used by the existing PhysicsHost seam."""

        return self.shm

    def child_environment(
        self,
        base: Mapping[str, str] | None = None,
    ) -> dict[str, str]:
        """Return an isolated environment for every process owned by this run."""

        environment = dict(os.environ if base is None else base)
        environment["LINGTU_HOST_BOOT_ID"] = self.boot_id
        return environment

    def to_dict(self) -> dict[str, Any]:
        """Return the language-neutral v1 allocation document."""

        return {
            "schema": "lingtu.sim.run-allocation.v1",
            "run_id": self.run_id,
            "session_id": self.session_id,
            "artifact_root": str(self.artifact_root),
            "boot_id": self.boot_id,
            "dds_domain": self.dds_domain,
            "ports": dict(self.ports),
            "shm": dict(self.shm),
            "log_dir": str(self.log_dir),
        }

    def to_json(self) -> str:
        """Serialize deterministically for disk and contract tests."""

        return (
            json.dumps(
                self.to_dict(),
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n"
        )


def _strict_json(text: str) -> Any:
    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def reject_constant(value: str) -> Any:
        raise ValueError(f"non-finite JSON value: {value}")

    return json.loads(
        text,
        object_pairs_hook=object_from_pairs,
        parse_constant=reject_constant,
    )


def _is_reparse_metadata(metadata: os.stat_result) -> bool:
    if stat.S_ISLNK(metadata.st_mode):
        return True
    return bool(getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT)


def _read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = _strict_json(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_MISSING,
            f"required bundle artifact is missing: {path.name}",
            artifact=path.name,
        ) from exc
    except (OSError, UnicodeError, ValueError) as exc:
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            f"cannot read bundle artifact: {path.name}",
            artifact=path.name,
        ) from exc
    if type(value) is not dict:
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            f"bundle artifact must contain a JSON object: {path.name}",
            artifact=path.name,
        )
    return value


def _invalid_allocation(message: str, *, artifact: str | None = None) -> RunAllocationError:
    return RunAllocationError(
        RunAllocationErrorCode.ALLOCATION_INVALID,
        message,
        artifact=artifact,
    )


def _validated_id(value: Any, field: str, pattern: re.Pattern[str]) -> str:
    if not isinstance(value, str) or pattern.fullmatch(value) is None:
        raise _invalid_allocation(f"{field} has an invalid value")
    return value


def _validated_dds_domain(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 232:
        raise _invalid_allocation("dds_domain must be an integer in [0, 232]")
    return value


def _validated_ports(value: Any) -> dict[str, int]:
    if not isinstance(value, Mapping):
        raise _invalid_allocation("ports must be an object")
    result: dict[str, int] = {}
    for key, port in value.items():
        name = _validated_id(key, "ports key", _RESOURCE_KEY_RE)
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise _invalid_allocation(f"ports.{name} must be an integer in [1, 65535]")
        result[name] = port
    if len(set(result.values())) != len(result):
        raise RunAllocationError(
            RunAllocationErrorCode.RESOURCE_CONFLICT,
            "port values conflict within the requested allocation",
        )
    return result


def _validated_shm(value: Any) -> dict[str, str]:
    if not isinstance(value, Mapping):
        raise _invalid_allocation("shm must be an object")
    result: dict[str, str] = {}
    for key, shm_name in value.items():
        name = _validated_id(key, "shm key", _RESOURCE_KEY_RE)
        if (
            not isinstance(shm_name, str)
            or not shm_name
            or shm_name != shm_name.strip()
            or len(shm_name) > 255
            or "\x00" in shm_name
            or "\r" in shm_name
            or "\n" in shm_name
        ):
            raise _invalid_allocation(f"shm.{name} has an invalid value")
        result[name] = shm_name
    if len(set(result.values())) != len(result):
        raise RunAllocationError(
            RunAllocationErrorCode.RESOURCE_CONFLICT,
            "shm names conflict within the requested allocation",
        )
    return result


def _validated_absolute_normalized_path(value: Any, field: str) -> Path:
    if not isinstance(value, str) or not value or value != value.strip():
        raise _invalid_allocation(f"{field} must be a non-empty absolute normalized path")
    if "\x00" in value or "\r" in value or "\n" in value:
        raise _invalid_allocation(f"{field} must be a non-empty absolute normalized path")
    path = Path(value)
    if not path.is_absolute():
        raise _invalid_allocation(f"{field} must be a non-empty absolute normalized path")
    normalized = path.resolve()
    if str(normalized) != value:
        raise _invalid_allocation(f"{field} must be a non-empty absolute normalized path")
    return normalized


def _require_plain_canonical_directory(path: Path, field: str) -> None:
    """Reject a caller-owned directory if any path indirection is visible."""

    try:
        metadata = os.lstat(path)
        resolved = path.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise _invalid_allocation(
            f"{field} must be an absolute canonical plain directory",
            artifact=str(path),
        ) from exc
    if not stat.S_ISDIR(metadata.st_mode) or _is_reparse_metadata(metadata):
        raise _invalid_allocation(
            f"{field} must be an absolute canonical plain directory",
            artifact=str(path),
        )
    canonical = os.path.normcase(os.path.normpath(os.fspath(resolved)))
    requested = os.path.normcase(os.path.normpath(os.fspath(path)))
    if not path.is_absolute() or canonical != requested:
        raise _invalid_allocation(
            f"{field} must be an absolute canonical plain directory",
            artifact=str(path),
        )


def _validate_camera_shm_name(sensor_id: str, shm_name: str) -> None:
    if _SHM_NAME_RE.fullmatch(shm_name) is None:
        raise _invalid_allocation(f"shm.{sensor_id} has an invalid camera_shm name")


def _camera_shm_sensor_ids(sensor_plan: Mapping[str, Any]) -> tuple[str, ...]:
    streams = sensor_plan.get("streams")
    if not isinstance(streams, Mapping):
        return ()
    sensor_ids: list[str] = []
    seen: set[str] = set()
    for group_name, group_streams in streams.items():
        if not isinstance(group_streams, list):
            continue
        for stream_index, stream in enumerate(group_streams):
            if type(stream) is not dict or stream.get("transport") != "camera_shm":
                continue
            sensor_id = stream.get("sensor_id")
            if not isinstance(sensor_id, str) or _RESOURCE_KEY_RE.fullmatch(sensor_id) is None:
                raise _invalid_allocation(
                    (
                        "sensor.plan.json streams."
                        f"{group_name}[{stream_index}].sensor_id has an invalid value"
                    ),
                    artifact="sensor.plan.json",
                )
            if sensor_id not in seen:
                sensor_ids.append(sensor_id)
                seen.add(sensor_id)
    return tuple(sensor_ids)


def _camera_shm_name(run_id: str, sensor_id: str) -> str:
    return f"lingtu.sim.camera_shm.{run_id}.{sensor_id}"


def _with_camera_shm_defaults(
    *,
    run_id: str,
    explicit_shm: Mapping[str, str],
    sensor_plan: Mapping[str, Any],
) -> dict[str, str]:
    resolved = dict(explicit_shm)
    camera_sensor_ids = _camera_shm_sensor_ids(sensor_plan)
    for sensor_id in camera_sensor_ids:
        resolved.setdefault(sensor_id, _camera_shm_name(run_id, sensor_id))
    resolved_shm = _validated_shm(resolved)
    for sensor_id in camera_sensor_ids:
        _validate_camera_shm_name(sensor_id, resolved_shm[sensor_id])
    return resolved_shm


def _existing_allocation(path: Path) -> dict[str, Any]:
    return load_run_allocation(path).to_dict()


def _allocation_is_released(path: Path, allocation: Mapping[str, Any]) -> bool:
    """Return whether an atomic runtime manifest proves terminal release."""

    manifest_path = path.parent / "session.runtime.json"
    try:
        manifest = _strict_json(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError):
        return False
    if (
        type(manifest) is not dict
        or manifest.get("schema") != "lingtu.sim.session-runtime.v1"
        or manifest.get("state") not in _TERMINAL_RUNTIME_STATES
        or manifest.get("run_id") != allocation.get("run_id")
        or manifest.get("session_id") != allocation.get("session_id")
    ):
        return False
    runtime_allocation = manifest.get("allocation")
    if type(runtime_allocation) is not dict:
        return False
    return (
        runtime_allocation.get("run_dir") == str(path.parent.resolve())
        and runtime_allocation.get("log_dir") == allocation.get("log_dir")
        and runtime_allocation.get("boot_id") == allocation.get("boot_id")
        and runtime_allocation.get("dds_domain") == allocation.get("dds_domain")
        and runtime_allocation.get("ports") == allocation.get("ports")
        and runtime_allocation.get("shm") == allocation.get("shm")
    )


def _reject_reserved_resources(
    run_root: Path,
    *,
    dds_domain: int,
    ports: Mapping[str, int],
    shm: Mapping[str, str],
) -> None:
    requested_ports = set(ports.values())
    requested_shm = set(shm.values())
    for path in sorted(run_root.glob("*/run-allocation.json")):
        existing = _existing_allocation(path)
        if _allocation_is_released(path, existing):
            continue
        existing_run = existing.get("run_id")
        if existing.get("dds_domain") == dds_domain:
            raise RunAllocationError(
                RunAllocationErrorCode.RESOURCE_CONFLICT,
                f"dds_domain {dds_domain} conflicts with run {existing_run!r}",
                artifact=str(path),
            )
        existing_ports = existing.get("ports")
        if type(existing_ports) is not dict:
            raise _invalid_allocation(
                f"existing run allocation has invalid ports: {path}",
                artifact=str(path),
            )
        port_conflicts = requested_ports.intersection(existing_ports.values())
        if port_conflicts:
            port = min(port_conflicts)
            raise RunAllocationError(
                RunAllocationErrorCode.RESOURCE_CONFLICT,
                f"port {port} conflicts with run {existing_run!r}",
                artifact=str(path),
            )
        existing_shm = existing.get("shm")
        if type(existing_shm) is not dict:
            raise _invalid_allocation(
                f"existing run allocation has invalid shm: {path}",
                artifact=str(path),
            )
        shm_conflicts = requested_shm.intersection(existing_shm.values())
        if shm_conflicts:
            name = sorted(shm_conflicts)[0]
            raise RunAllocationError(
                RunAllocationErrorCode.RESOURCE_CONFLICT,
                f"shm name {name!r} conflicts with run {existing_run!r}",
                artifact=str(path),
            )


def load_run_allocation(path: Path) -> RunAllocation:
    """Load and validate one serialized v1 allocation document."""

    allocation_path = Path(path).resolve()
    if allocation_path.is_dir():
        allocation_path = allocation_path / "run-allocation.json"
    try:
        document = _strict_json(allocation_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise _invalid_allocation(
            f"cannot read run allocation: {allocation_path}",
            artifact=str(allocation_path),
        ) from exc
    if type(document) is not dict or set(document) != _ALLOCATION_KEYS:
        raise _invalid_allocation(
            "run allocation must contain exactly the v1 fields",
            artifact=str(allocation_path),
        )
    if document.get("schema") != "lingtu.sim.run-allocation.v1":
        raise _invalid_allocation(
            "run allocation has an unsupported schema",
            artifact=str(allocation_path),
        )
    run_id = _validated_id(document.get("run_id"), "run_id", _RUN_ID_RE)
    boot_id = _validated_id(document.get("boot_id"), "boot_id", _RUN_ID_RE)
    session_id = document.get("session_id")
    if not isinstance(session_id, str) or not session_id.strip():
        raise _invalid_allocation("session_id must be non-empty")
    dds_domain = _validated_dds_domain(document.get("dds_domain"))
    ports = _validated_ports(document.get("ports"))
    shm = _validated_shm(document.get("shm"))
    artifact_root = _validated_absolute_normalized_path(
        document.get("artifact_root"),
        "artifact_root",
    )
    log_dir = _validated_absolute_normalized_path(document.get("log_dir"), "log_dir")
    return RunAllocation(
        run_id=run_id,
        run_dir=allocation_path.parent,
        artifact_root=artifact_root,
        log_dir=log_dir.resolve(),
        ports=ports,
        shm=shm,
        session_id=session_id,
        boot_id=boot_id,
        dds_domain=dds_domain,
    )


def _validate_bundle_sensor_timebase(
    physics_plan: Mapping[str, Any],
    sensor_plan: Mapping[str, Any],
) -> None:
    streams = sensor_plan.get("streams")
    if type(streams) is not dict:
        return
    physics_streams = [
        stream
        for declarations in streams.values()
        if isinstance(declarations, list)
        for stream in declarations
        if isinstance(stream, Mapping) and stream.get("owner") == "physics"
    ]
    if not physics_streams:
        return

    global_policy = physics_plan.get("global_policy")
    if not isinstance(global_policy, Mapping):
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            "physics.plan.json must declare global_policy for physics-owned sensors",
            artifact="physics.plan.json",
        )
    timestep_s = global_policy.get("timestep_s")
    try:
        violation = physics_sensor_timebase_violation(
            timestep_s=timestep_s,
            streams=physics_streams,
        )
    except (TypeError, ValueError) as exc:
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            f"bundle physics sensor timebase is invalid: {exc}",
            artifact="sensor.plan.json",
        ) from exc
    if violation is None:
        return
    raise RunAllocationError(
        RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
        f"physics-owned sensor {violation.sensor_id!r} at "
        f"{float(violation.rate_hz):g} Hz cannot sample exactly on physics "
        f"timestep {float(violation.timestep_s):g} s",
        artifact="sensor.plan.json",
    )


def load_resolved_session_bundle(
    bundle_dir: Path,
    *,
    repo_root: Path | None = None,
) -> ResolvedSessionBundle:
    """Load and cross-check every deterministic artifact in one bundle."""

    resolved_dir = Path(bundle_dir).resolve()
    if not resolved_dir.is_dir():
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_NOT_FOUND,
            f"resolved session bundle does not exist: {resolved_dir}",
        )
    resolved_repo_root = Path(repo_root).resolve() if repo_root is not None else None
    if resolved_repo_root is not None and not resolved_repo_root.is_dir():
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            f"repository root does not exist: {resolved_repo_root}",
        )
    try:
        session_spec = yaml.safe_load((resolved_dir / "session.yaml").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            f"cannot read session.yaml: {exc}",
            artifact="session.yaml",
        ) from exc
    if not isinstance(session_spec, Mapping):
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            "session.yaml must contain an object",
            artifact="session.yaml",
        )
    session_id = session_spec.get("session_id")
    if not isinstance(session_id, str) or not session_id.strip():
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            "session.yaml has an invalid session_id",
            artifact="session.yaml",
        )
    plans: dict[str, Mapping[str, Any]] = {}
    plan_schemas = _V2_PLAN_SCHEMAS
    for filename in STATIC_PLAN_FILES:
        plan = _read_json_object(resolved_dir / filename)
        if plan.get("schema") not in {dict(STATIC_PLAN_CONTRACTS)[filename], plan_schemas[filename]}:
            raise RunAllocationError(
                RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
                f"{filename} has an unsupported schema",
                artifact=filename,
            )
        plan_session_id = plan.get("session_id")
        if not isinstance(plan_session_id, str) or not plan_session_id.strip():
            raise RunAllocationError(
                RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
                f"{filename} has an invalid session_id",
                artifact=filename,
            )
        if plan_session_id != session_id:
            raise RunAllocationError(
                RunAllocationErrorCode.SESSION_ID_MISMATCH,
                f"{filename} session_id does not match session.yaml",
                artifact=filename,
            )
        plans[filename] = plan
    _validate_bundle_sensor_timebase(
        plans["physics.plan.json"],
        plans["sensor.plan.json"],
    )
    boundary = plans["transport.intent.json"].get("allocation_boundary")
    if boundary != {"owner": "RunAllocation", "runtime_values_external": True}:
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            "transport.intent.json has an invalid allocation_boundary",
            artifact="transport.intent.json",
        )
    return ResolvedSessionBundle(
        bundle_dir=resolved_dir,
        repo_root=resolved_repo_root,
        session_id=session_id,
        session_spec=session_spec,
        plans=plans,
    )


def _allocation_exists_error(run_id: str, run_dir: Path) -> RunAllocationError:
    return RunAllocationError(
        RunAllocationErrorCode.ALLOCATION_EXISTS,
        f"run allocation already exists: {run_id}",
        artifact=str(run_dir / "run-allocation.json"),
    )


def _plain_run_directory_metadata(path: Path, run_id: str) -> os.stat_result:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError as exc:
        raise _allocation_exists_error(run_id, path) from exc
    except OSError as exc:
        raise RunAllocationError(
            RunAllocationErrorCode.IO_ERROR,
            f"cannot inspect existing run directory: {run_id}",
            artifact=str(path),
        ) from exc
    if not stat.S_ISDIR(metadata.st_mode) or _is_reparse_metadata(metadata):
        raise _allocation_exists_error(run_id, path)
    return metadata


def _metadata_identity(metadata: os.stat_result) -> tuple[int, int]:
    return metadata.st_dev, metadata.st_ino


@dataclass
class _AdoptedRunDirectory:
    path: Path
    run_id: str
    identity: tuple[int, int]
    descriptor: int | None
    logs_created: bool = False
    screenshots_created: bool = False
    allocation_created: bool = False
    allocation_identity: tuple[int, int] | None = None

    def verify_identity(self) -> None:
        named = _plain_run_directory_metadata(self.path, self.run_id)
        if _metadata_identity(named) != self.identity:
            raise _allocation_exists_error(self.run_id, self.path)
        if self.descriptor is not None:
            try:
                opened = os.fstat(self.descriptor)
            except OSError as exc:
                raise RunAllocationError(
                    RunAllocationErrorCode.IO_ERROR,
                    f"cannot inspect held run directory: {self.run_id}",
                    artifact=str(self.path),
                ) from exc
            if (
                not stat.S_ISDIR(opened.st_mode)
                or _metadata_identity(opened) != self.identity
            ):
                raise _allocation_exists_error(self.run_id, self.path)

    def require_empty(self) -> None:
        self.verify_identity()
        if self._entry_names():
            raise _allocation_exists_error(self.run_id, self.path)

    def create_log_directory(self) -> None:
        self.verify_identity()
        try:
            if self.descriptor is None:
                (self.path / "logs").mkdir()
            else:
                os.mkdir("logs", mode=0o700, dir_fd=self.descriptor)
        except FileExistsError as exc:
            raise _allocation_exists_error(self.run_id, self.path) from exc
        except OSError as exc:
            raise RunAllocationError(
                RunAllocationErrorCode.IO_ERROR,
                f"cannot create run directories for {self.run_id}",
                artifact=str(self.path),
            ) from exc
        self.logs_created = True
        self.verify_identity()

    def create_screenshot_directory(self) -> None:
        self.verify_identity()
        try:
            if self.descriptor is None:
                (self.path / "screenshots").mkdir()
            else:
                os.mkdir("screenshots", mode=0o700, dir_fd=self.descriptor)
        except FileExistsError as exc:
            raise _allocation_exists_error(self.run_id, self.path) from exc
        except OSError as exc:
            raise RunAllocationError(
                RunAllocationErrorCode.IO_ERROR,
                f"cannot create run directories for {self.run_id}",
                artifact=str(self.path),
            ) from exc
        self.screenshots_created = True
        self.verify_identity()

    def write_allocation(self, allocation: RunAllocation) -> None:
        self.verify_identity()
        try:
            if self.descriptor is None:
                with allocation.path.open(
                    "x",
                    encoding="utf-8",
                    newline="",
                ) as handle:
                    self.allocation_created = True
                    self.allocation_identity = _metadata_identity(
                        os.fstat(handle.fileno())
                    )
                    handle.write(allocation.to_json())
                    handle.flush()
                    os.fsync(handle.fileno())
            else:
                flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
                file_descriptor = os.open(
                    allocation.path.name,
                    flags,
                    0o600,
                    dir_fd=self.descriptor,
                )
                self.allocation_created = True
                self.allocation_identity = _metadata_identity(
                    os.fstat(file_descriptor)
                )
                with os.fdopen(
                    file_descriptor,
                    "w",
                    encoding="utf-8",
                    newline="",
                ) as handle:
                    handle.write(allocation.to_json())
                    handle.flush()
                    os.fsync(handle.fileno())
        except FileExistsError as exc:
            raise _allocation_exists_error(self.run_id, self.path) from exc
        except OSError as exc:
            raise RunAllocationError(
                RunAllocationErrorCode.IO_ERROR,
                f"cannot write run allocation for {self.run_id}",
                artifact=str(allocation.path),
            ) from exc
        self.verify_identity()

    def verify_materialized(self) -> None:
        self.verify_identity()
        if self._entry_names() != {"logs", "screenshots", "run-allocation.json"}:
            raise _allocation_exists_error(self.run_id, self.path)
        try:
            logs = self._entry_metadata("logs")
            screenshots = self._entry_metadata("screenshots")
            allocation = self._entry_metadata("run-allocation.json")
        except OSError as exc:
            raise _allocation_exists_error(self.run_id, self.path) from exc
        if not stat.S_ISDIR(logs.st_mode) or _is_reparse_metadata(logs):
            raise _allocation_exists_error(self.run_id, self.path)
        if not stat.S_ISDIR(screenshots.st_mode) or _is_reparse_metadata(screenshots):
            raise _allocation_exists_error(self.run_id, self.path)
        if (
            not stat.S_ISREG(allocation.st_mode)
            or _is_reparse_metadata(allocation)
            or _metadata_identity(allocation) != self.allocation_identity
        ):
            raise _allocation_exists_error(self.run_id, self.path)

    def rollback(self) -> None:
        if self.allocation_created and self.allocation_identity is not None:
            try:
                metadata = self._entry_metadata("run-allocation.json")
                if (
                    stat.S_ISREG(metadata.st_mode)
                    and not _is_reparse_metadata(metadata)
                    and _metadata_identity(metadata) == self.allocation_identity
                ):
                    self._unlink("run-allocation.json")
            except OSError:
                pass
        if self.screenshots_created:
            try:
                metadata = self._entry_metadata("screenshots")
                if stat.S_ISDIR(metadata.st_mode) and not _is_reparse_metadata(metadata):
                    self._rmdir("screenshots")
            except OSError:
                pass
        if self.logs_created:
            try:
                metadata = self._entry_metadata("logs")
                if stat.S_ISDIR(metadata.st_mode) and not _is_reparse_metadata(metadata):
                    self._rmdir("logs")
            except OSError:
                pass

    def _entry_names(self) -> set[str]:
        target: int | Path = self.path if self.descriptor is None else self.descriptor
        try:
            with os.scandir(target) as entries:
                return {entry.name for entry in entries}
        except OSError as exc:
            raise RunAllocationError(
                RunAllocationErrorCode.IO_ERROR,
                f"cannot inspect existing run directory: {self.run_id}",
                artifact=str(self.path),
            ) from exc

    def _entry_metadata(self, name: str) -> os.stat_result:
        if self.descriptor is None:
            return os.lstat(self.path / name)
        return os.stat(name, dir_fd=self.descriptor, follow_symlinks=False)

    def _unlink(self, name: str) -> None:
        if self.descriptor is None:
            (self.path / name).unlink()
        else:
            os.unlink(name, dir_fd=self.descriptor)

    def _rmdir(self, name: str) -> None:
        if self.descriptor is None:
            (self.path / name).rmdir()
        else:
            os.rmdir(name, dir_fd=self.descriptor)


@contextmanager
def _hold_adopted_run_directory(
    path: Path,
    run_id: str,
    *,
    trusted_root: Path | None = None,
) -> Iterator[_AdoptedRunDirectory]:
    chain_root = path.parent if trusted_root is None else trusted_root
    with _hold_trusted_directory_chain(
        chain_root,
        path,
        run_id,
        preserve_final_allocation_exists=trusted_root is None,
    ) as descriptor:
        opened = (
            _plain_run_directory_metadata(path, run_id)
            if descriptor is None
            else os.fstat(descriptor)
        )
        identity = _metadata_identity(opened)
        named = _plain_run_directory_metadata(path, run_id)
        if _metadata_identity(named) != identity:
            raise _allocation_exists_error(run_id, path)
        guard = _AdoptedRunDirectory(path, run_id, identity, descriptor)
        committed = False
        try:
            guard.verify_identity()
            guard.require_empty()
            yield guard
            guard.verify_materialized()
            guard.verify_identity()
            committed = True
        finally:
            if not committed:
                guard.rollback()


@contextmanager
def _hold_trusted_directory_chain(
    trusted_root: Path,
    target: Path,
    run_id: str,
    *,
    preserve_final_allocation_exists: bool = False,
) -> Iterator[int | None]:
    """Pin every owned directory from ``trusted_root`` through ``target``.

    Windows handles deliberately omit ``FILE_SHARE_DELETE`` so an inspected
    ancestor cannot be replaced while allocation files are materialized.  On
    POSIX, every child is opened relative to its already-open parent and the
    final descriptor is used for writes.
    """

    root = Path(os.path.abspath(os.fspath(trusted_root)))
    destination = Path(os.path.abspath(os.fspath(target)))
    try:
        relative = destination.relative_to(root)
    except ValueError as exc:
        raise _invalid_allocation(
            "trusted run path is outside its owned root",
            artifact=str(destination),
        ) from exc

    if os.name == "nt":
        handles: list[int] = []
        current = root
        try:
            for part in (None, *relative.parts):
                if part is not None:
                    current = current / part
                try:
                    handles.append(_open_windows_run_directory(current, run_id))
                except RunAllocationError as exc:
                    if preserve_final_allocation_exists and current == destination:
                        raise
                    raise _invalid_allocation(
                        f"trusted run path contains a reparse point or non-directory: {current}",
                        artifact=str(current),
                    ) from exc
            yield None
        finally:
            for handle in reversed(handles):
                _close_windows_handle(handle)
        return

    flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptors: list[int] = []
    try:
        try:
            descriptor = os.open(root, flags)
        except OSError as exc:
            raise _invalid_allocation(
                f"trusted run root is not a plain directory: {root}",
                artifact=str(root),
            ) from exc
        descriptors.append(descriptor)
        for index, part in enumerate(relative.parts):
            try:
                descriptor = os.open(part, flags, dir_fd=descriptor)
            except OSError as exc:
                if (
                    preserve_final_allocation_exists
                    and index == len(relative.parts) - 1
                ):
                    raise _allocation_exists_error(run_id, destination) from exc
                raise _invalid_allocation(
                    f"trusted run path contains a link or non-directory: {destination}",
                    artifact=str(destination),
                ) from exc
            descriptors.append(descriptor)
        for descriptor in descriptors:
            metadata = os.fstat(descriptor)
            if not stat.S_ISDIR(metadata.st_mode) or _is_reparse_metadata(metadata):
                raise _invalid_allocation(
                    f"trusted run path contains a reparse point or non-directory: {destination}",
                    artifact=str(destination),
                )
        yield descriptors[-1]
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _open_windows_run_directory(path: Path, run_id: str) -> int:
    import ctypes
    from ctypes import wintypes

    file_list_directory = 0x0001
    file_read_attributes = 0x0080
    file_share_read = 0x00000001
    file_share_write = 0x00000002
    open_existing = 3
    file_attribute_directory = 0x00000010
    file_flag_backup_semantics = 0x02000000
    file_flag_open_reparse_point = 0x00200000

    class ByHandleFileInformation(ctypes.Structure):
        _fields_ = [
            ("dwFileAttributes", wintypes.DWORD),
            ("ftCreationTime", wintypes.FILETIME),
            ("ftLastAccessTime", wintypes.FILETIME),
            ("ftLastWriteTime", wintypes.FILETIME),
            ("dwVolumeSerialNumber", wintypes.DWORD),
            ("nFileSizeHigh", wintypes.DWORD),
            ("nFileSizeLow", wintypes.DWORD),
            ("nNumberOfLinks", wintypes.DWORD),
            ("nFileIndexHigh", wintypes.DWORD),
            ("nFileIndexLow", wintypes.DWORD),
        ]

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    create_file = kernel32.CreateFileW
    create_file.argtypes = (
        wintypes.LPCWSTR,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.LPVOID,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.HANDLE,
    )
    create_file.restype = wintypes.HANDLE
    get_information = kernel32.GetFileInformationByHandle
    get_information.argtypes = (
        wintypes.HANDLE,
        ctypes.POINTER(ByHandleFileInformation),
    )
    get_information.restype = wintypes.BOOL

    handle = create_file(
        os.fspath(path),
        file_list_directory | file_read_attributes,
        file_share_read | file_share_write,
        None,
        open_existing,
        file_flag_backup_semantics | file_flag_open_reparse_point,
        None,
    )
    if handle == ctypes.c_void_p(-1).value:
        error = ctypes.get_last_error()
        raise RunAllocationError(
            RunAllocationErrorCode.IO_ERROR,
            f"cannot hold existing run directory: {run_id}",
            artifact=str(path),
        ) from OSError(error, os.strerror(error))

    information = ByHandleFileInformation()
    if not get_information(handle, ctypes.byref(information)):
        error = ctypes.get_last_error()
        _close_windows_handle(handle)
        raise RunAllocationError(
            RunAllocationErrorCode.IO_ERROR,
            f"cannot inspect held run directory: {run_id}",
            artifact=str(path),
        ) from OSError(error, os.strerror(error))
    if (
        not information.dwFileAttributes & file_attribute_directory
        or information.dwFileAttributes & _REPARSE_POINT
    ):
        _close_windows_handle(handle)
        raise _allocation_exists_error(run_id, path)
    return int(handle)


def _close_windows_handle(handle: int) -> None:
    import ctypes
    from ctypes import wintypes

    close_handle = ctypes.WinDLL("kernel32", use_last_error=True).CloseHandle
    close_handle.argtypes = (wintypes.HANDLE,)
    close_handle.restype = wintypes.BOOL
    close_handle(handle)


def create_run_allocation(
    bundle_dir: Path,
    run_root: Path,
    *,
    dds_domain: int,
    ports: Mapping[str, int] | None = None,
    shm: Mapping[str, str] | None = None,
    run_id: str | None = None,
    boot_id: str | None = None,
    repo_root: Path | None = None,
    adopt_existing_empty_run_dir: bool = False,
    trusted_root: Path | None = None,
    artifact_root_mode: Literal["repository", "run"] = "repository",
) -> RunAllocation:
    """Validate a bundle and materialize one run without reserving OS resources.

    ``artifact_root_mode="run"`` is the narrow qualification seam: it binds
    the serialized artifact root to the allocator-owned run directory without
    accepting an arbitrary caller path.  The default remains the repository
    root for every existing caller.
    """

    bundle = load_resolved_session_bundle(bundle_dir, repo_root=repo_root)
    if bundle.repo_root is None:
        raise RunAllocationError(
            RunAllocationErrorCode.BUNDLE_ARTIFACT_INVALID,
            "repository root is required to materialize artifact_root",
            artifact="session.yaml",
        )
    requested_root = Path(run_root)
    resolved_root = Path(os.path.abspath(os.fspath(requested_root)))
    resolved_trusted_root = (
        None
        if trusted_root is None
        else Path(os.path.abspath(os.fspath(trusted_root)))
    )
    if resolved_trusted_root is not None:
        try:
            resolved_root.relative_to(resolved_trusted_root)
        except ValueError as exc:
            raise _invalid_allocation(
                "run root is outside its trusted root",
                artifact=str(resolved_root),
            ) from exc
    resolved_run_id = run_id if run_id is not None else f"run-{uuid.uuid4().hex}"
    inherited_boot_id = os.environ.get("LINGTU_HOST_BOOT_ID")
    resolved_boot_id = (
        boot_id
        if boot_id is not None
        else inherited_boot_id or _COORDINATOR_BOOT_ID
    )
    _validated_id(resolved_run_id, "run_id", _RUN_ID_RE)
    _validated_id(resolved_boot_id, "boot_id", _RUN_ID_RE)
    resolved_dds_domain = _validated_dds_domain(dds_domain)
    resolved_ports = _validated_ports({} if ports is None else ports)
    explicit_shm = _validated_shm({} if shm is None else shm)
    resolved_shm = _with_camera_shm_defaults(
        run_id=resolved_run_id,
        explicit_shm=explicit_shm,
        sensor_plan=bundle.plans["sensor.plan.json"],
    )
    run_dir = resolved_root / resolved_run_id
    log_dir = run_dir / "logs"
    if not isinstance(artifact_root_mode, str) or artifact_root_mode not in {
        "repository",
        "run",
    }:
        raise _invalid_allocation("artifact_root_mode must be 'repository' or 'run'")
    if artifact_root_mode == "run" and (
        not requested_root.is_absolute()
        or os.path.normcase(os.fspath(requested_root))
        != os.path.normcase(os.fspath(resolved_root))
    ):
        raise _invalid_allocation(
            "run artifact owner root must be an absolute canonical path",
            artifact=str(requested_root),
        )
    artifact_root = bundle.repo_root if artifact_root_mode == "repository" else run_dir
    try:
        run_dir.relative_to(bundle.bundle_dir)
    except ValueError:
        pass
    else:
        raise _invalid_allocation(
            "run allocation must be materialized outside the resolved session bundle"
        )
    if resolved_trusted_root is None:
        try:
            resolved_root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise RunAllocationError(
                RunAllocationErrorCode.IO_ERROR,
                f"cannot create run root: {resolved_root}",
                artifact=str(resolved_root),
            ) from exc
    elif not adopt_existing_empty_run_dir:
        raise _invalid_allocation(
            "trusted_root is supported only for adoption of a pre-created run directory",
            artifact=str(resolved_root),
        )
    if artifact_root_mode == "run":
        _require_plain_canonical_directory(resolved_root, "run artifact owner root")
    allocation = RunAllocation(
        run_id=resolved_run_id,
        run_dir=run_dir,
        artifact_root=artifact_root,
        log_dir=log_dir,
        ports=resolved_ports,
        shm=resolved_shm,
        session_id=bundle.session_id,
        boot_id=resolved_boot_id,
        dds_domain=resolved_dds_domain,
    )
    try:
        existing_run_dir = os.lstat(run_dir)
    except FileNotFoundError:
        existing_run_dir = None
    except OSError as exc:
        raise RunAllocationError(
            RunAllocationErrorCode.IO_ERROR,
            f"cannot inspect run directory: {resolved_run_id}",
            artifact=str(run_dir),
        ) from exc
    if resolved_trusted_root is not None and existing_run_dir is None:
        raise _invalid_allocation(
            "trusted run directory must be pre-created before adoption",
            artifact=str(run_dir),
        )
    if existing_run_dir is not None and not adopt_existing_empty_run_dir:
        raise _allocation_exists_error(resolved_run_id, run_dir)
    if existing_run_dir is not None:
        with _hold_adopted_run_directory(
            run_dir,
            resolved_run_id,
            trusted_root=resolved_trusted_root,
        ) as adopted:
            _reject_reserved_resources(
                resolved_root,
                dds_domain=resolved_dds_domain,
                ports=resolved_ports,
                shm=resolved_shm,
            )
            adopted.create_log_directory()
            adopted.create_screenshot_directory()
            adopted.write_allocation(allocation)
        return allocation

    _reject_reserved_resources(
        resolved_root,
        dds_domain=resolved_dds_domain,
        ports=resolved_ports,
        shm=resolved_shm,
    )
    try:
        run_dir.mkdir(exist_ok=False)
    except FileExistsError as exc:
        raise _allocation_exists_error(resolved_run_id, run_dir) from exc
    except OSError as exc:
        raise RunAllocationError(
            RunAllocationErrorCode.IO_ERROR,
            f"cannot materialize run allocation for {resolved_run_id}",
            artifact=str(run_dir),
        ) from exc
    with _hold_adopted_run_directory(
        run_dir,
        resolved_run_id,
        trusted_root=resolved_root,
    ) as created:
        created.create_log_directory()
        created.create_screenshot_directory()
        created.write_allocation(allocation)
    return allocation
