"""Server-trusted runtime construction for SimStudio runs."""

from __future__ import annotations

import math
import os
from collections.abc import Callable, Mapping
from dataclasses import InitVar, dataclass, field
from pathlib import Path
from typing import Any, NoReturn

from sim.runtime.control import create_production_components
from sim.runtime.coordinator import (
    InteractiveSimulationSession,
    MujocoProcess,
    RuntimeCoordinator,
)
from sim.runtime.coordinator.external_evidence import ExternalEvidenceWatcher
from sim.runtime.coordinator.live_snapshot import UdpLoopbackSnapshotPublisher
from sim.runtime.coordinator.live_visual import (
    DEFAULT_SNAPSHOT_PORT,
    _validated_unreal_level,
    default_uproject,
)
from sim.runtime.coordinator.run_allocation import (
    ResolvedSessionBundle,
    load_resolved_session_bundle,
)
from sim.runtime.coordinator.session_host import SessionHost
from sim.runtime.coordinator.unreal_process import UnrealProcess
from sim.runtime.recording import CameraShmPayloadSource
from sim.runtime.sensors import (
    ImuEndpointFactory,
    Mid360EndpointFactory,
    SensorEndpointFactory,
    SensorEndpointRouter,
    TruthOdometryEndpointFactory,
)

from .models import BundleRecord, StoreValidationError
from .store import StudioStore

_MUJOCO_HOST_ENV = "LINGTU_SIMSTUDIO_MUJOCO_HOST"
_DDS_DOMAIN_ENV = "LINGTU_SIMSTUDIO_DDS_DOMAIN"
_TRUTH_ODOM_PUBLISHER_ENV = "LINGTU_SIMSTUDIO_TRUTH_ODOM_PUBLISHER"
_IMU_PUBLISHER_ENV = "LINGTU_SIMSTUDIO_IMU_PUBLISHER"
_MID360_PUBLISHER_ENV = "LINGTU_SIMSTUDIO_MID360_PUBLISHER"
_UNREAL_EDITOR_ENV = "LINGTU_SIMSTUDIO_UNREAL_EDITOR"
_VISUAL_SNAPSHOT_PORT_ENV = "LINGTU_SIMSTUDIO_VISUAL_SNAPSHOT_PORT"
_VISUAL_READY_TIMEOUT_ENV = "LINGTU_SIMSTUDIO_VISUAL_READY_TIMEOUT_S"
_MOTION_CAMERA_STABLE_ID_ENV = "LINGTU_SIMSTUDIO_MOTION_CAMERA_STABLE_ID"
DEFAULT_VISUAL_READY_TIMEOUT_S = 600.0


@dataclass(frozen=True)
class TrustedRuntimeConfig:
    """Server-owned runtime facts excluded from HTTP and bundle payloads."""

    repo_root: Path
    mujoco_host: Path
    studio_root: Path | None = None
    dds_domain: int = 0
    ports: Mapping[str, int] = field(default_factory=dict)
    shm: Mapping[str, str] = field(default_factory=dict)
    truth_odom_publisher: Path | None = None
    imu_publisher: Path | None = None
    mid360_publisher: Path | None = None
    truth_odom_parent_frame: str = "map"
    unreal_editor: Path | None = None
    uproject: Path | None = None
    visual_snapshot_port: int = DEFAULT_SNAPSHOT_PORT
    visual_ready_timeout_s: float = DEFAULT_VISUAL_READY_TIMEOUT_S
    motion_camera_stable_id: str | None = None
    external_mujoco_host: InitVar[bool] = False

    def __post_init__(self, external_mujoco_host: bool) -> None:
        repo_root = Path(self.repo_root).resolve()
        object.__setattr__(self, "repo_root", repo_root)
        object.__setattr__(
            self,
            "mujoco_host",
            _resolve_server_path(
                self.mujoco_host,
                repo_root=repo_root,
                field="mujoco_host",
                require_repo_build=not external_mujoco_host,
            ),
        )
        if self.studio_root is not None:
            object.__setattr__(self, "studio_root", Path(self.studio_root).resolve())
        if isinstance(self.dds_domain, bool) or not isinstance(self.dds_domain, int) or self.dds_domain < 0:
            raise ValueError("dds_domain must be a non-negative integer")
        if (
            isinstance(self.visual_snapshot_port, bool)
            or not isinstance(self.visual_snapshot_port, int)
            or not 1 <= self.visual_snapshot_port <= 65535
        ):
            raise ValueError("visual_snapshot_port must be an integer in [1, 65535]")
        object.__setattr__(
            self,
            "visual_ready_timeout_s",
            _validated_positive_finite(
                self.visual_ready_timeout_s,
                "visual_ready_timeout_s",
            ),
        )
        if self.motion_camera_stable_id is not None:
            object.__setattr__(
                self,
                "motion_camera_stable_id",
                _source_token(
                    self.motion_camera_stable_id,
                    "motion_camera_stable_id",
                ),
            )
        object.__setattr__(self, "ports", _validated_ports(self.ports))
        object.__setattr__(self, "shm", _validated_shm(self.shm))
        for field_name in (
            "truth_odom_publisher",
            "imu_publisher",
            "mid360_publisher",
            "unreal_editor",
        ):
            value = getattr(self, field_name)
            if value is not None:
                object.__setattr__(
                    self,
                    field_name,
                    _resolve_server_path(
                        value,
                        repo_root=repo_root,
                        field=field_name,
                        require_repo_build=False,
                    ),
                )
        object.__setattr__(
            self,
            "uproject",
            _resolve_server_path(
                self.uproject or default_uproject(repo_root),
                repo_root=repo_root,
                field="uproject",
                require_repo_build=False,
            ),
        )
        _source_token(self.truth_odom_parent_frame, "truth_odom_parent_frame")

    @classmethod
    def from_repository(
        cls,
        repo_root: Path,
        *,
        studio_root: Path | None = None,
        env: Mapping[str, str] | None = None,
    ) -> TrustedRuntimeConfig:
        """Create trusted defaults from the repository and server environment."""

        root = Path(repo_root).resolve()
        environment = os.environ if env is None else env
        env_mujoco_host = environment.get(_MUJOCO_HOST_ENV)
        mujoco_host = env_mujoco_host or _default_mujoco_host(root)
        return cls(
            repo_root=root,
            studio_root=studio_root,
            mujoco_host=Path(mujoco_host),
            external_mujoco_host=env_mujoco_host is not None,
            dds_domain=_env_int(environment.get(_DDS_DOMAIN_ENV), field=_DDS_DOMAIN_ENV),
            truth_odom_publisher=_optional_env_path(environment.get(_TRUTH_ODOM_PUBLISHER_ENV)),
            imu_publisher=_optional_env_path(environment.get(_IMU_PUBLISHER_ENV)),
            mid360_publisher=_optional_env_path(environment.get(_MID360_PUBLISHER_ENV)),
            unreal_editor=_optional_env_path(environment.get(_UNREAL_EDITOR_ENV)),
            visual_snapshot_port=_env_port(
                environment.get(_VISUAL_SNAPSHOT_PORT_ENV),
                field=_VISUAL_SNAPSHOT_PORT_ENV,
                default=DEFAULT_SNAPSHOT_PORT,
            ),
            visual_ready_timeout_s=_env_positive_float(
                environment.get(_VISUAL_READY_TIMEOUT_ENV),
                field=_VISUAL_READY_TIMEOUT_ENV,
                default=DEFAULT_VISUAL_READY_TIMEOUT_S,
            ),
            motion_camera_stable_id=_optional_env_token(
                environment.get(_MOTION_CAMERA_STABLE_ID_ENV),
                field=_MOTION_CAMERA_STABLE_ID_ENV,
            ),
        )


class RuntimeFactory:
    """Build interactive runtime sessions from trusted server configuration."""

    def __init__(
        self,
        config: TrustedRuntimeConfig,
        *,
        coordinator_type: Callable[..., Any] = RuntimeCoordinator,
        mujoco_process_type: Callable[..., Any] = MujocoProcess,
        session_type: Callable[..., InteractiveSimulationSession] = InteractiveSimulationSession,
        session_host_type: Callable[..., SessionHost] = SessionHost,
        unreal_process_type: Callable[..., Any] = UnrealProcess,
        snapshot_publisher_type: Callable[..., Any] = UdpLoopbackSnapshotPublisher,
        evidence_watcher_type: Callable[..., Any] = ExternalEvidenceWatcher,
        controller_factory: Callable[..., Any] = create_production_components,
        sensor_endpoint_factory: SensorEndpointFactory | None = None,
    ) -> None:
        self.config = config
        self._coordinator_type = coordinator_type
        self._mujoco_process_type = mujoco_process_type
        self._session_type = session_type
        self._session_host_type = session_host_type
        self._unreal_process_type = unreal_process_type
        self._snapshot_publisher_type = snapshot_publisher_type
        self._evidence_watcher_type = evidence_watcher_type
        self._controller_factory = controller_factory
        self._sensor_endpoint_factory = sensor_endpoint_factory

    @classmethod
    def from_repository(
        cls,
        repo_root: Path,
        *,
        studio_root: Path | None = None,
        env: Mapping[str, str] | None = None,
    ) -> RuntimeFactory:
        """Build the production runtime factory for one controlled repository."""

        return cls(TrustedRuntimeConfig.from_repository(repo_root, studio_root=studio_root, env=env))

    def create_session(
        self,
        *,
        bundle: BundleRecord | None = None,
        bundle_record: BundleRecord | None = None,
        launch_profile: str,
        artifact_root: Path | None = None,
        artifact_path: Path | None = None,
        run_id: str,
    ) -> InteractiveSimulationSession:
        """Return a headless interactive session for one Studio run."""

        resolved_bundle = _required_bundle(bundle, bundle_record)
        resolved_artifact_root = _required_artifact_root(
            artifact_root,
            artifact_path,
        )
        if launch_profile not in {"headless", "visual"}:
            raise RuntimeError(f"unsupported launch_profile: {launch_profile}")
        if launch_profile == "visual" and self.config.unreal_editor is None:
            raise RuntimeError(
                "visual launch_profile requires server Unreal executable config "
                f"({_UNREAL_EDITOR_ENV} or TrustedRuntimeConfig.unreal_editor)"
            )
        if self.config.studio_root is None:
            raise RuntimeError("SimStudio runtime factory requires a server-side studio_root")
        run_root = _trusted_artifact_parent(
            resolved_artifact_root,
            studio_root=self.config.studio_root,
            run_id=run_id,
        )
        bundle_dir = self._bundle_dir(resolved_bundle)
        compiled_bundle = load_resolved_session_bundle(
            bundle_dir,
            repo_root=self.config.repo_root,
        )
        _validate_launch_profile(
            compiled_bundle,
            launch_profile=launch_profile,
        )
        if launch_profile == "visual":
            return self._create_visual_session(
                bundle=compiled_bundle,
                artifact_root=resolved_artifact_root,
                run_root=run_root,
                run_id=run_id,
            )
        physics_host = self._mujoco_process_type(self.config.mujoco_host)
        coordinator = self._coordinator_type(
            bundle_dir=bundle_dir,
            repo_root=self.config.repo_root,
            run_root=run_root,
            physics_host=physics_host,
            run_id=run_id,
            adopt_existing_empty_run_dir=True,
            trusted_root=self.config.studio_root,
            dds_domain=self.config.dds_domain,
            ports=self.config.ports,
            shm=self.config.shm,
            controller_factory=self._controller_factory,
            sensor_endpoint_factory=self._sensor_endpoint_factory
            or self._production_sensor_endpoint_factory(physics_host),
        )
        return self._session_type(coordinator)

    def _create_visual_session(
        self,
        *,
        bundle: ResolvedSessionBundle,
        artifact_root: Path,
        run_root: Path,
        run_id: str,
    ) -> InteractiveSimulationSession:
        if self.config.unreal_editor is None:
            raise RuntimeError(
                "visual launch_profile requires server Unreal executable config "
                f"({_UNREAL_EDITOR_ENV} or TrustedRuntimeConfig.unreal_editor)"
            )
        bundle_dir = bundle.bundle_dir
        level = _visual_plan_level(bundle.plans["visual.plan.json"])
        physics_host = self._mujoco_process_type(self.config.mujoco_host)
        ports = dict(self.config.ports)
        ports["visual_snapshot_udp"] = self.config.visual_snapshot_port
        coordinator = self._coordinator_type(
            bundle_dir=bundle_dir,
            repo_root=self.config.repo_root,
            run_root=run_root,
            physics_host=physics_host,
            run_id=run_id,
            adopt_existing_empty_run_dir=True,
            trusted_root=self.config.studio_root,
            dds_domain=self.config.dds_domain,
            ports=ports,
            shm=self.config.shm,
            controller_factory=self._controller_factory,
            sensor_endpoint_factory=self._sensor_endpoint_factory
            or self._production_sensor_endpoint_factory(physics_host),
        )
        run_dir = Path(os.path.abspath(os.fspath(artifact_root)))
        log_dir = run_dir / "logs"
        visual_watcher = self._evidence_watcher_type(
            log_dir / "visual-readiness.json",
            session_id=bundle.session_id,
            model_generation=0,
            reset_generation=0,
            expected_source_id="robotsimue-visual",
        )
        camera_watcher = self._evidence_watcher_type(
            log_dir / "sensor-readiness.json",
            session_id=bundle.session_id,
            model_generation=0,
            reset_generation=0,
            expected_source_id="robotsimue-camera",
        )
        host = self._session_host_type(
            coordinator=coordinator,
            unreal_process=self._unreal_process_type(
                self.config.unreal_editor,
                self.config.uproject,
                level,
                motion_camera_stable_id=self.config.motion_camera_stable_id,
            ),
            publisher=self._snapshot_publisher_type(self.config.visual_snapshot_port),
            evidence_watchers=(visual_watcher, camera_watcher),
            snapshot_port=self.config.visual_snapshot_port,
            ready_timeout_s=self.config.visual_ready_timeout_s,
        )
        return self._session_type(
            host,
            sensor_payload_source=CameraShmPayloadSource(
                sensor_plan=bundle.plans["sensor.plan.json"],
                allocation_provider=lambda: coordinator.allocation,
            ),
        )

    def _bundle_dir(self, bundle: BundleRecord) -> Path:
        relative = bundle.payload.get("bundle_path")
        if not isinstance(relative, str):
            raise StoreValidationError("bundle.payload.bundle_path must be present")
        safe = StudioStore.validate_relative_path(relative, context="bundle.payload.bundle_path")
        if self.config.studio_root is None:
            raise RuntimeError("SimStudio runtime factory requires a server-side studio_root")
        bundle_dir = self.config.studio_root.joinpath(*safe.split("/")).resolve()
        try:
            bundle_dir.relative_to(self.config.studio_root)
        except ValueError as exc:
            raise StoreValidationError("bundle path escapes the Studio store root") from exc
        return bundle_dir

    def _production_sensor_endpoint_factory(self, physics_host: Any) -> SensorEndpointFactory | None:
        factories: list[SensorEndpointFactory] = []
        if self.config.imu_publisher is not None:
            factories.append(ImuEndpointFactory(self.config.imu_publisher))
        if self.config.truth_odom_publisher is not None:
            factories.append(
                TruthOdometryEndpointFactory(
                    self.config.truth_odom_publisher,
                    parent_frame=self.config.truth_odom_parent_frame,
                )
            )
        if self.config.mid360_publisher is not None:
            factories.append(Mid360EndpointFactory(self.config.mid360_publisher, physics_host))
        return SensorEndpointRouter(tuple(factories)) if factories else None


class RuntimeFactoryError(StoreValidationError):
    """Stable machine-readable failure at the Studio runtime factory seam."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        details: Mapping[str, Any],
    ) -> None:
        super().__init__(message)
        self.code = code
        self.details = dict(details)

    def to_dict(self) -> dict[str, Any]:
        """Return the fixed v1 error document."""

        return {
            "schema": "lingtu.sim.studio.runtime-factory-error.v1",
            "code": self.code,
            "message": str(self),
            "details": dict(self.details),
        }


def _validate_launch_profile(
    bundle: ResolvedSessionBundle,
    *,
    launch_profile: str,
) -> None:
    session = bundle.session_spec
    if not isinstance(session, Mapping):
        _raise_invalid_runtime_contract(
            launch_profile=launch_profile,
            field="session",
            mode=None,
            bindings=[],
        )
    runtime = session.get("runtime")
    if not isinstance(runtime, Mapping):
        _raise_invalid_runtime_contract(
            launch_profile=launch_profile,
            field="session.runtime",
            mode=None,
            bindings=[],
        )
    mode = runtime.get("mode")
    if not isinstance(mode, str) or not mode:
        _raise_invalid_runtime_contract(
            launch_profile=launch_profile,
            field="session.runtime.mode",
            mode=None,
            bindings=[],
        )
    bindings = runtime.get("required_bindings")
    normalized_bindings = (
        sorted(item for item in bindings if isinstance(item, str))
        if isinstance(bindings, list)
        else []
    )
    allowed_bindings = {"physics", "visual", "sensors", "control"}
    if (
        not isinstance(bindings, list)
        or not bindings
        or len(normalized_bindings) != len(bindings)
        or len(set(normalized_bindings)) != len(normalized_bindings)
        or "physics" not in normalized_bindings
        or not set(normalized_bindings) <= allowed_bindings
    ):
        _raise_invalid_runtime_contract(
            launch_profile=launch_profile,
            field="session.runtime.required_bindings",
            mode=mode,
            bindings=normalized_bindings,
        )
    expects_visual = launch_profile == "visual"
    if ("visual" in normalized_bindings) is expects_visual:
        return
    raise RuntimeFactoryError(
        "SIMSTUDIO_LAUNCH_PROFILE_MISMATCH",
        (
            f"launch profile {launch_profile!r} is incompatible with session mode "
            f"{mode!r} and required bindings"
        ),
        details={
            "launch_profile": launch_profile,
            "required_bindings": normalized_bindings,
            "session_mode": mode,
        },
    )


def _raise_invalid_runtime_contract(
    *,
    launch_profile: str,
    field: str,
    mode: str | None,
    bindings: list[str],
) -> NoReturn:
    raise RuntimeFactoryError(
        "SIMSTUDIO_RUNTIME_CONTRACT_INVALID",
        f"resolved bundle has invalid {field}",
        details={
            "field": field,
            "launch_profile": launch_profile,
            "required_bindings": bindings,
            "session_mode": mode,
        },
    )


def _default_mujoco_host(repo_root: Path) -> Path:
    suffix = ".exe" if os.name == "nt" else ""
    return repo_root / "build" / "mujoco-runtime-physics-win" / "Release" / f"lingtu_mujoco_headless{suffix}"


def _optional_env_path(value: str | None) -> Path | None:
    return Path(value) if value else None


def _optional_env_token(value: str | None, *, field: str) -> str | None:
    if value is None or value == "":
        return None
    return _source_token(value, field)


def _env_int(value: str | None, *, field: str) -> int:
    if value is None or value == "":
        return 0
    try:
        number = int(value)
    except ValueError as exc:
        raise ValueError(f"{field} must be an integer") from exc
    if number < 0:
        raise ValueError(f"{field} must be non-negative")
    return number


def _env_port(value: str | None, *, field: str, default: int) -> int:
    if value is None or value == "":
        return default
    number = _env_int(value, field=field)
    if number < 1 or number > 65535:
        raise ValueError(f"{field} must be in [1, 65535]")
    return number


def _env_positive_float(value: str | None, *, field: str, default: float) -> float:
    if value is None or value == "":
        return default
    try:
        number = float(value)
    except ValueError as exc:
        raise ValueError(f"{field} must be a positive finite number") from exc
    return _validated_positive_finite(number, field)


def _validated_positive_finite(value: float, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a positive finite number")
    number = float(value)
    if number <= 0 or not math.isfinite(number):
        raise ValueError(f"{field} must be a positive finite number")
    return number


def _visual_plan_level(plan: Mapping[str, Any]) -> str:
    world = plan.get("world")
    if type(world) is not dict:
        raise RuntimeError("visual.plan.json world must be an object")
    try:
        return _validated_unreal_level(world.get("level"), "visual.plan.world.level")
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc


def _required_bundle(
    bundle: BundleRecord | None,
    bundle_record: BundleRecord | None,
) -> BundleRecord:
    resolved = bundle if bundle is not None else bundle_record
    if resolved is None:
        raise TypeError("create_session requires bundle")
    return resolved


def _required_artifact_root(
    artifact_root: Path | None,
    artifact_path: Path | None,
) -> Path:
    resolved = artifact_root if artifact_root is not None else artifact_path
    if resolved is None:
        raise TypeError("create_session requires artifact_root")
    return resolved


def _trusted_artifact_parent(
    artifact_root: Path,
    *,
    studio_root: Path,
    run_id: str,
) -> Path:
    owned_root = Path(os.path.abspath(os.fspath(studio_root)))
    candidate = Path(os.path.abspath(os.fspath(artifact_root)))
    expected = owned_root / "artifacts" / "runs" / run_id
    if candidate != expected:
        raise StoreValidationError(
            "runtime artifact root must be the service-owned path for its run_id"
        )
    run_root = candidate.parent
    StudioStore._assert_no_reparse_components(run_root, below=owned_root)
    return run_root


def _resolve_server_path(
    value: Path,
    *,
    repo_root: Path,
    field: str,
    require_repo_build: bool,
) -> Path:
    path = Path(value)
    resolved = (repo_root / path).resolve() if not path.is_absolute() else path.resolve()
    if require_repo_build:
        try:
            resolved.relative_to(repo_root / "build")
        except ValueError as exc:
            raise ValueError(f"{field} must resolve below the repository build directory") from exc
    return resolved


def _validated_ports(value: Mapping[str, int]) -> dict[str, int]:
    result: dict[str, int] = {}
    for key, port in value.items():
        _source_token(key, "port key")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("ports must contain integer TCP/UDP port values")
        result[key] = port
    return result


def _validated_shm(value: Mapping[str, str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for key, name in value.items():
        _source_token(key, "shm key")
        _source_token(name, "shm name")
        result[key] = name
    return result


def _source_token(value: str, field: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or any(character.isspace() for character in value)
    ):
        raise ValueError(f"{field} must be one non-empty token")
    return value


__all__ = ["RuntimeFactory", "RuntimeFactoryError", "TrustedRuntimeConfig"]
