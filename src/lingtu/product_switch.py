"""Fail-closed product switch transaction for the field runtime."""

# Protocol declarations intentionally keep their signatures compact.
# ruff: noqa: D102

from __future__ import annotations

import json
import math
import os
import posixpath
import re
import subprocess
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

from lingtu.product_lock import resolve_current_run_path, resolve_product_state_dir
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RUN_PLAN_SCHEMA, RunPlan
from lingtu.runtime_parameters import resolve_runtime_parameters
from lingtu.systemd import ProcessReport
from runtime.contracts.product_runtime import (
    PRODUCT_CONTROL_SESSION_HEADER,
    PRODUCT_SESSION_ID_ENV,
)
from runtime.profiles.native_nav_config import NativeNavConfig, compile_native_nav_config
from runtime.profiles.product_lifecycle import (
    ProductLifecycle,
    ProductName,
    product_name,
)

SWITCH_REPORT_SCHEMA = "lingtu.product_switch.v1"
SESSION_EXPLANATION_SCHEMA = "lingtu.session_explanation.v1"
MAP_ACTIVATION_TOKEN_SCHEMA = "lingtu.map_activation.v2"  # noqa: S105
_ACTIVE_MAP_SAVE_STATES = frozenset({"WAITING_SNAPSHOT", "QUEUED", "RUNNING"})
_TERMINAL_MAP_SAVE_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELLED"})
_ACTIVE_RECORDING_STATES = frozenset({"preparing", "recording", "stopping"})
_TERMINAL_RECORDING_STATES = frozenset({"idle", "completed", "failed"})
_MAX_RECORDING_STATUS_BYTES = 5 * 1024 * 1024
_SESSION_ROOT = "/run/lingtu"
_SESSION_ROOT_ENV = "LINGTU_SESSION_ROOT"
_SESSION_ENV_FILE = "session.env"
_LEGACY_SYSTEMD_RUNTIME_ROOT = "/run/systemd/system"
_LEGACY_RUNTIME_DROPINS = {
    "host": "product-mode.conf",
    "nav": "product-mode.conf",
    "slam": "runtime-mode.conf",
    "explore": "product-mode.conf",
}
_NAV_ENV_KEYS = frozenset(
    {
        "LINGTU_PRODUCT",
        "LINGTU_NAV_CONFIG_FINGERPRINT",
        "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M",
        "LINGTU_NAV_CONTROL_MODE",
        "LINGTU_NAV_PUBLISH_CMD_VEL",
        "LINGTU_NAV_CHECK_OBSTACLE",
        "LINGTU_NAV_USE_TRAVERSABILITY_COST",
        "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER",
        "LINGTU_NAV_GOAL_REACHED_M",
        "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M",
        "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS",
        "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS",
        "LINGTU_NAV_WAYPOINT_REACHED_M",
        "LINGTU_TELEOP_LOCAL_PLANNER",
        "LINGTU_TELEOP_PLANNER_HORIZON_M",
        "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG",
    }
)
_RELOCALIZE_RETRY_REASONS = (
    "registered_cloud_unavailable",
    "waiting_for_scan",
    "scan_too_small",
    "cloud_unavailable",
    "scan_unavailable",
    "timeout",
)
_NAV_STATUS_PATH = Path("/dev/shm/lingtu/nav_endpoint_status.json")  # noqa: S108
_EXPLORE_STATUS_PATH = Path("/dev/shm/lingtu/explore_status.json")  # noqa: S108
_SLAM_STATUS_PATH = Path("/tmp/lingtu_slam_status.json")  # noqa: S108
_SLAM_SNAPSHOT_DIR = Path("/dev/shm/lingtu_slam")  # noqa: S108
_RUNTIME_CLEANUP_ROOTS = ("/dev/shm", "/run", "/tmp")  # noqa: S108
_ALREADY_ACTIVE_READINESS_TIMEOUT_S = 1.5
_TELEOP_AUTONOMY_ONLY_BLOCKERS = frozenset(
    {
        "odometry_missing",
        "frame_mismatch_odometry",
        "frame_mismatch_costmap",
        "frame_mismatch_goal",
        "navigation_session_inactive",
        "localization_degraded",
        "localization_lost",
        "localization_relocalizing",
        "localization_initializing",
        "localization_recovery_active",
        "saved_map_relocalization_missing",
        "pose_stale",
        "map_artifact_gate_failed",
        "real_runtime_evidence_missing_or_stale",
        "native_global_planner_missing",
        "native_global_planner_mismatch",
        "native_planner_map_missing",
        "native_teleop_local_planner_disabled",
        "native_obstacle_check_disabled",
        "native_traversability_cost_disabled",
    }
)


@dataclass(frozen=True)
class SwitchRequest:
    """Operator intent for one cold Product transition."""

    target_product: ProductName
    current_product: ProductName | None = None
    map_name: str | None = None
    relocalize: bool = True
    initial_pose: tuple[float, float, float] | None = None
    parameter_overrides: Mapping[str, Any] = field(default_factory=dict)

    @property
    def product_variant(self) -> str | None:
        """Select the internal Explore route without exposing another Product."""

        if self.target_product != "explore":
            return None
        return "map" if str(self.map_name or "").strip() else "live"


@dataclass
class SwitchReport:
    """Machine-readable result and phase journal for a Product switch."""

    current_product: ProductName | None
    target_product: ProductName
    env: str
    product_variant: str | None = None
    dry_run: bool = False
    ok: bool = False
    status: str = "preflight"
    run_plan_path: str | None = None
    fingerprint: str | None = None
    phases: list[str] = field(default_factory=list)
    cleanup: list[str] = field(default_factory=list)
    error: str | None = None
    launch: Mapping[str, Any] | None = None
    run_plan: Mapping[str, Any] | None = None
    native_nav: Mapping[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready switch evidence."""

        return {
            "schema_version": SWITCH_REPORT_SCHEMA,
            "ok": self.ok,
            "status": self.status,
            "dry_run": self.dry_run,
            "current_product": self.current_product,
            "target_product": self.target_product,
            "product_variant": self.product_variant,
            "env": self.env,
            "run_plan_path": self.run_plan_path,
            "fingerprint": self.fingerprint,
            "phases": list(self.phases),
            "cleanup": list(self.cleanup),
            "error": self.error,
            "launch": dict(self.launch) if self.launch is not None else None,
            "run_plan": dict(self.run_plan) if self.run_plan is not None else None,
            "native_nav": dict(self.native_nav) if self.native_nav is not None else None,
        }


@dataclass(frozen=True)
class SessionFile:
    """One ephemeral session file installed for a Product run."""

    path: str


@dataclass(frozen=True)
class SessionStage:
    """Rollback token for the ephemeral files created by one session."""

    files: tuple[SessionFile, ...]


@dataclass(frozen=True)
class MapArtifactIdentity:
    """Stable identity fields exposed by one native map artifact record."""

    artifact_type: str
    uri: str | None = None
    sha256: str | None = None


@dataclass(frozen=True)
class MapIdentity:
    """Exact saved-map identity returned by native mapd."""

    map_id: str
    version_id: str
    frame_id: str
    map_dir: str
    artifacts: tuple[MapArtifactIdentity, ...] = ()


@dataclass(frozen=True)
class MapActivationToken:
    """Opaque native compensation token plus verified map identities."""

    target: MapIdentity
    previous: MapIdentity | None
    changed: bool
    activation_token: str
    schema_version: str = MAP_ACTIVATION_TOKEN_SCHEMA


class _MapRollbackFailed(RuntimeError):
    """A prepared map could not be compensated and must stay fail-closed."""


class SwitchFailed(RuntimeError):
    """Raised after a switch is rejected or forced into a stopped state."""

    def __init__(self, report: SwitchReport):
        super().__init__(report.error or "Product switch failed")
        self.report = report


class SwitchControl(Protocol):
    """ProductControl surface used by the transaction."""

    env: str

    def resolve(
        self,
        product: str | None = None,
        *,
        product_variant: str | None = None,
    ) -> RunPlan: ...

    def _apply_plan_for_switch(
        self,
        path: str | Path,
        *,
        dry_run: bool = False,
    ) -> ProcessReport: ...

    def quiesce_plan(
        self,
        plan: RunPlan,
        *,
        dry_run: bool = False,
    ) -> ProcessReport: ...


class SwitchBackend(Protocol):
    """Deployment and Gateway effects required by ProductControl."""

    def current_product(self) -> ProductName | None: ...

    def assert_map_save_idle(self) -> None: ...

    def assert_recording_idle(self) -> None: ...

    def stop_motion_and_session(self, current_product: ProductName | None) -> None: ...

    def stage_map(self, map_name: str) -> MapActivationToken: ...

    def restore_map(self, token: MapActivationToken) -> None: ...

    def commit_map(self, token: MapActivationToken) -> None: ...

    def stage_session(
        self,
        run_plan_path: Path,
        plan: RunPlan,
        native_environment: Mapping[str, str],
        *,
        slam_mode: str,
        map_path: str,
        map_identity: MapIdentity | None = None,
        product_session_id: str | None = None,
        parameter_overrides: Mapping[str, Any] | None = None,
    ) -> SessionStage | None: ...

    def rollback_session(self, staged: SessionStage) -> None: ...

    def remove_session(self, plan: RunPlan) -> None: ...

    def clear_runtime_status(self) -> None: ...

    def wait_native_nav(
        self,
        native_environment: Mapping[str, str],
        *,
        timeout_s: float,
    ) -> None: ...

    def wait_slam(self, mode: str, *, require_map: bool, timeout_s: float) -> None: ...

    def wait_exploration(
        self,
        route: str,
        *,
        map_identity: MapIdentity | None,
        product_session_id: str,
        timeout_s: float,
        allow_active: bool = False,
    ) -> None: ...

    def start_session(
        self,
        lifecycle: ProductLifecycle,
        *,
        map_name: str,
        relocalize: bool,
        initial_pose: tuple[float, float, float] | None,
    ) -> None: ...

    def wait_navigation(
        self,
        *,
        map_name: str,
        control_mode: str,
        timeout_s: float,
    ) -> None: ...

    def wait_inspection(self, *, timeout_s: float) -> None: ...


class FieldBackend:
    """Linux/systemd/Gateway implementation of the Product switch effects."""

    def __init__(
        self,
        *,
        environment: Mapping[str, str] | None = None,
        gateway_url: str | None = None,
        runner: Callable[..., Any] = subprocess.run,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        self._environment = environment if environment is not None else os.environ
        self._gateway_url = str(gateway_url or self._environment.get("GW") or "http://127.0.0.1:5050").rstrip("/")
        parsed_gateway = urllib.parse.urlsplit(self._gateway_url)
        if parsed_gateway.scheme not in {"http", "https"} or not parsed_gateway.netloc:
            raise ValueError("Gateway URL must be an absolute HTTP(S) URL")
        self._runner = runner
        self._sleep = sleep
        self._monotonic = monotonic
        self._wall_clock = wall_clock
        self._session_root = _session_root(self._environment)

    def current_product(self) -> ProductName | None:
        """Read the current Product, or return None for a clean bootstrap."""

        product = _text(self._environment.get("LINGTU_PRODUCT"))
        if product:
            return product_name(product)
        committed = _current_run_record(None, self._environment)
        committed_product = _text(committed.get("product"))
        if committed_product:
            return product_name(committed_product)
        result = self._run(
            ["systemctl", "show", "lingtu.service", "-p", "Environment", "--value"],
            check=False,
            timeout=5.0,
        )
        managed_environment = {
            key: value
            for item in str(result.stdout or "").split()
            for key, separator, value in (item.partition("="),)
            if separator
        }
        product = _text(managed_environment.get("LINGTU_PRODUCT"))
        return product_name(product) if product else None

    def assert_map_save_idle(self) -> None:
        """Reject a Product switch while a durable SaveMap transaction is active."""

        map_api_key = _text(self._environment.get("LINGTU_MAP_API_KEY") or self._environment.get("LINGTU_API_KEY"))
        headers = {"X-API-Key": map_api_key} if map_api_key is not None else None
        response = self._http(
            "GET",
            "/api/v1/maps/operations?limit=1000",
            timeout_s=5.0,
            headers=headers,
        )
        if response.get("ok") is not True and response.get("success") is not True:
            raise RuntimeError("map_save_status_unavailable: Gateway rejected the SaveMap operation query")

        operations = response.get("operations")
        if operations is None:
            if response.get("count") == 0:
                return
            raise RuntimeError("map_save_status_invalid: SaveMap operation query omitted operations")
        if not isinstance(operations, list):
            raise RuntimeError("map_save_status_invalid: SaveMap operations must be a list")

        active: list[str] = []
        for operation in operations:
            if not isinstance(operation, Mapping):
                raise RuntimeError("map_save_status_invalid: SaveMap operation entry must be an object")
            state = _text(operation.get("state"))
            if state is None:
                raise RuntimeError("map_save_status_invalid: SaveMap operation entry omitted state")
            state = state.upper()
            if state in _TERMINAL_MAP_SAVE_STATES:
                continue
            if state not in _ACTIVE_MAP_SAVE_STATES:
                raise RuntimeError(f"map_save_status_invalid: unsupported SaveMap state {state!r}")
            operation_id = _text(operation.get("operation_id")) or "unknown"
            active.append(f"{operation_id}:{state}")

        if active:
            raise RuntimeError(
                "map_save_in_progress: Product switch is blocked by active SaveMap "
                f"operations ({', '.join(active)}); wait for completion or explicitly "
                "cancel the operation"
            )

    def assert_recording_idle(self) -> None:
        """Reject a Product switch while native evidence recording is active."""

        binary = self._recording_binary()
        root = self._recording_root()
        result = self._run(
            [str(binary), "status", "--root", str(root)],
            check=False,
            timeout=5.0,
        )
        output = str(result.stdout or "")
        if not output or len(output.encode("utf-8", errors="replace")) > _MAX_RECORDING_STATUS_BYTES:
            raise RuntimeError("recording_status_invalid: native recorder returned no bounded status")
        try:
            payload = json.loads(output)
        except json.JSONDecodeError as exc:
            raise RuntimeError("recording_status_invalid: native recorder returned invalid JSON") from exc
        if not isinstance(payload, Mapping) or payload.get("control_version") != 1:
            raise RuntimeError("recording_status_invalid: native recorder control version is unsupported")
        if payload.get("ok") is not True:
            error = payload.get("error")
            code = _text(error.get("code")) if isinstance(error, Mapping) else None
            message = _text(error.get("message")) if isinstance(error, Mapping) else None
            raise RuntimeError(
                f"{code or 'recording_status_unavailable'}: {message or 'native recorder rejected the status query'}"
            )
        state = _text(payload.get("state"))
        healthy = payload.get("healthy")
        if state is None or not isinstance(healthy, bool):
            raise RuntimeError("recording_status_invalid: native recorder omitted state or health")
        if state in _ACTIVE_RECORDING_STATES:
            session = payload.get("session")
            session_id = _text(session.get("session_id")) if isinstance(session, Mapping) else None
            if not healthy:
                raise RuntimeError(
                    "recording_recovery_required: active recording state is unhealthy; "
                    f"session={session_id or 'unknown'} state={state}"
                )
            raise RuntimeError(
                "recording_in_progress: Product switch is blocked by native recording "
                f"session={session_id or 'unknown'} state={state}; stop recording first"
            )
        if state not in _TERMINAL_RECORDING_STATES:
            raise RuntimeError(f"recording_status_invalid: unsupported native recording state {state!r}")
        if int(result.returncode) != 0 and state != "failed":
            raise RuntimeError("recording_status_unavailable: native recorder status command failed")

    def _recording_binary(self) -> Path:
        configured = _text(self._environment.get("LINGTU_RECORDING_BIN"))
        candidates: list[Path] = []
        if configured is not None:
            candidates.append(Path(configured).expanduser())
        configured_dir = _text(self._environment.get("LINGTU_RECORDING_BIN_DIR"))
        if configured_dir is not None:
            candidates.append(Path(configured_dir).expanduser() / "lingtu_recorder")
        candidates.append(Path("/opt/lingtu/current/build/native-recording/lingtu_recorder"))
        for candidate in candidates:
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return candidate.resolve()
        raise RuntimeError("recording_status_unavailable: native lingtu_recorder is not installed")

    def _recording_root(self) -> Path:
        configured = _text(self._environment.get("LINGTU_RECORDING_ROOT"))
        if configured is not None:
            return Path(configured).expanduser()
        home = Path(self._environment.get("HOME") or Path.home())
        return home / "data" / "lingtu" / "recordings"

    def stop_motion_and_session(self, current_product: ProductName | None) -> None:
        """Stop native motion without requiring the Python Host to exist."""

        nav_unit = str(self._environment.get("LINGTU_NAV_SERVICE") or "lingtu-nav-dds.service")
        nav_active = self._unit_active(nav_unit)
        if current_product is not None and not nav_active:
            raise RuntimeError(
                "native navigation stop is unavailable; zero speed, driver ACK, "
                "and odometry stillness were not confirmed"
            )
        if nav_active:
            nav_control = str(
                self._environment.get("LINGTU_NAV_CONTROL_BIN")
                or "/opt/lingtu/current/build/nav_endpoint/lingtu_nav_control"
            )
            command = [nav_control, "stop", "product_mode_switch"]
            domain_id = _text(self._environment.get("LINGTU_DDS_DOMAIN_ID"))
            if domain_id is not None:
                command.extend(("--domain-id", domain_id))
            command.extend(("--timeout-ms", "3000"))
            self._run(command, timeout=8.0)

        control_token = self._product_session_token()
        if current_product is not None and control_token is None:
            raise RuntimeError(
                "active Product session credential is unavailable; refusing an unauthenticated Gateway session shutdown"
            )
        headers = {PRODUCT_CONTROL_SESSION_HEADER: control_token} if control_token is not None else None
        try:
            session = self._http(
                "POST",
                "/api/v1/session/end",
                timeout_s=10.0,
                headers=headers,
            )
        except RuntimeError:
            if current_product is not None:
                raise
            return
        if session.get("ok") is not True or session.get("success") is not True:
            raise RuntimeError("session end was not confirmed")

    def stage_map(self, map_name: str) -> MapActivationToken:
        """Stage one exact active map through native typed DDS control."""

        response = self._mapctl("stage", map_name, timeout_s=20.0)
        target = _native_map_identity(response.get("target"), field_name="target map")
        previous = _native_optional_map_identity(response.get("previous"), field_name="previous map")
        active = _native_map_identity(response.get("active"), field_name="active map")
        activation_token = _required_text(
            response.get("activation_token"),
            "native map activation token",
        )
        if target.map_id != map_name or active != target:
            raise _MapRollbackFailed("native map staging did not activate the exact requested identity")
        return MapActivationToken(
            target=target,
            previous=previous,
            changed=_required_bool(response.get("changed"), "native map changed"),
            activation_token=activation_token,
        )

    def restore_map(self, token: MapActivationToken) -> None:
        """Restore the exact previous identity using mapd's opaque token."""

        _validate_map_activation_token(token)
        response = self._mapctl("restore", token.activation_token, timeout_s=20.0)
        active = _native_optional_map_identity(response.get("active"), field_name="restored active map")
        if active != token.previous:
            raise RuntimeError("native map restore did not recover the prepared previous identity")

    def commit_map(self, token: MapActivationToken) -> None:
        """Verify the exact staged identity before committing the Product."""

        _validate_map_activation_token(token)
        response = self._mapctl("verify", token.activation_token, timeout_s=10.0)
        active = _native_map_identity(response.get("active"), field_name="verified active map")
        if active != token.target:
            raise RuntimeError("native active map identity changed before Product commit")

    def _mapctl(self, operation: str, operand: str, *, timeout_s: float) -> Mapping[str, Any]:
        binary = str(self._environment.get("LINGTU_MAPCTL_BIN") or "/opt/lingtu/current/build/maps/lingtu-mapctl")
        command = [
            binary,
            operation,
            _required_text(operand, f"mapctl {operation} operand"),
            "--map-root",
            str(self._maps_root().expanduser().resolve()),
            "--caller",
            "product-control",
        ]
        domain_id = _text(self._environment.get("LINGTU_DDS_DOMAIN_ID"))
        if domain_id is not None:
            command.extend(("--domain-id", domain_id))
        result = self._run(command, check=False, timeout=timeout_s)
        try:
            payload = json.loads(str(result.stdout or ""))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            detail = _text(result.stderr) or _text(result.stdout) or f"exit {result.returncode}"
            raise RuntimeError(f"native map control returned invalid JSON: {detail}") from exc
        if not isinstance(payload, Mapping):
            raise RuntimeError("native map control response must be an object")
        if (
            result.returncode != 0
            or payload.get("schema_version") != MAP_ACTIVATION_TOKEN_SCHEMA
            or payload.get("operation") != operation
            or payload.get("accepted") is not True
        ):
            detail = _text(payload.get("message")) or _text(result.stderr) or f"exit {result.returncode}"
            raise RuntimeError(f"native map {operation} rejected: {detail}")
        return payload

    def stage_session(
        self,
        run_plan_path: Path,
        plan: RunPlan,
        native_environment: Mapping[str, str],
        *,
        slam_mode: str,
        map_path: str,
        map_identity: MapIdentity | None = None,
        product_session_id: str | None = None,
        parameter_overrides: Mapping[str, Any] | None = None,
    ) -> SessionStage:
        """Atomically publish one boot-scoped Product session environment."""

        session_id = _required_text(
            product_session_id or uuid.uuid4().hex,
            "Product session id",
        )
        session_path = _session_env_path(self._session_root)
        try:
            environment = self._session_environment(
                run_plan_path=run_plan_path,
                plan=plan,
                native_environment=native_environment,
                slam_mode=slam_mode,
                map_path=map_path,
                map_identity=map_identity,
                product_session_id=session_id,
                parameter_overrides=parameter_overrides,
            )
            self._remove_legacy_runtime_config(plan)
            self._install_runtime_file(session_path, _environment_file(environment))
        except Exception as exc:
            # A failed stage must never leave an older Product mode startable.
            try:
                self._remove_runtime_file(session_path, check=False)
            except Exception as cleanup_error:
                raise RuntimeError(f"session staging failed: {exc}; cleanup failed: {cleanup_error}") from exc
            raise
        return SessionStage(files=(SessionFile(path=session_path),))

    def _session_environment(
        self,
        *,
        run_plan_path: Path,
        plan: RunPlan,
        native_environment: Mapping[str, str],
        slam_mode: str,
        map_path: str,
        map_identity: MapIdentity | None,
        product_session_id: str,
        parameter_overrides: Mapping[str, Any] | None,
    ) -> dict[str, str]:
        """Build the only per-session process environment."""

        if plan.schema_version != RUN_PLAN_SCHEMA:
            raise RuntimeError(f"unsupported RunPlan schema: {plan.schema_version!r}")
        host_config = plan.host_config
        endpoint_transport = _text(host_config.get("_endpoint_transport")) or "dds"
        endpoint_contract = _text(host_config.get("_endpoint_contract")) or ""
        command_output_mode = _required_text(
            host_config.get("command_output_mode"),
            "Product host command_output_mode",
        )
        hardware_control_boundary = _required_text(
            host_config.get("hardware_control_boundary"),
            "Product host hardware_control_boundary",
        )
        native = {str(key): str(value) for key, value in native_environment.items()}
        unexpected = sorted(set(native) - _NAV_ENV_KEYS)
        if unexpected:
            raise RuntimeError(f"invalid native navigation environment keys: {unexpected}")
        missing = sorted(_NAV_ENV_KEYS - set(native))
        if missing:
            raise RuntimeError(f"missing native navigation environment keys: {missing}")

        if slam_mode not in {"mapping", "localization", "none"}:
            raise RuntimeError(f"unsupported SLAM mode: {slam_mode}")
        if slam_mode == "localization" and not map_path:
            raise RuntimeError("localization requires a MapService-provided map artifact")

        parameter_set = resolve_runtime_parameters(
            parameter_profile=plan.parameter_profile,
            env_overrides=plan.parameter_overrides,
            session_overrides=parameter_overrides,
            map_publish_hz=_environment_float(
                self._environment,
                "LINGTU_TRAVERSABILITY_PUBLISH_HZ",
                default=10.0,
            ),
            robot_max_speed_mps=_environment_float(
                self._environment,
                "LINGTU_NAV_MAX_SPEED_MPS",
                default=0.4,
            ),
        )
        environment = {
            "LINGTU_PRODUCT": plan.product,
            "LINGTU_ENV": plan.env,
            "LINGTU_RUN_PLAN": str(run_plan_path),
            "LINGTU_RUN_PLAN_FINGERPRINT": plan.fingerprint,
            "LINGTU_SESSION_ROOT": str(run_plan_path.parent),
            "LINGTU_PRODUCT_SESSION_ID": product_session_id,
            "LINGTU_ENDPOINT_TRANSPORT": endpoint_transport,
            "LINGTU_ENDPOINT_CONTRACT": endpoint_contract,
            "LINGTU_COMMAND_OUTPUT_MODE": command_output_mode,
            "LINGTU_HARDWARE_CONTROL_BOUNDARY": hardware_control_boundary,
            "LINGTU_SLAM_MODE": slam_mode,
            "LINGTU_SLAM_MAP": map_path,
            **native,
            **parameter_set.environment(),
        }
        if map_identity is not None:
            environment.update(_map_identity_environment(map_identity))
        if plan.has_process("explore"):
            environment["LINGTU_EXPLORE_ROUTE"] = _explore_route(slam_mode)
        return dict(sorted(environment.items()))

    def rollback_session(self, staged: SessionStage) -> None:
        """Remove files created for a failed Product session."""

        errors: list[str] = []
        for snapshot in reversed(staged.files):
            try:
                self._remove_runtime_file(snapshot.path)
            except Exception as exc:
                errors.append(f"{snapshot.path}: {exc}")
        if errors:
            raise RuntimeError("failed to remove staged runtime config: " + "; ".join(errors))

    def remove_session(self, plan: RunPlan) -> None:
        """Remove the boot-scoped Product session and legacy drop-ins."""

        errors: list[str] = []
        try:
            self._remove_runtime_file(_session_env_path(self._session_root))
        except Exception as exc:
            errors.append(f"session.env: {exc}")
        try:
            self._remove_legacy_runtime_config(plan)
        except Exception as exc:
            errors.append(f"legacy drop-ins: {exc}")
        if errors:
            raise RuntimeError("failed to remove transient runtime config: " + "; ".join(errors))

    def _remove_legacy_runtime_config(self, plan: RunPlan) -> None:
        """Remove pre-session-file overrides left by an older release."""

        removed = False
        for _process_name, unit, name in _legacy_runtime_config_dropins(plan):
            path = _dropin_path(unit, name, runtime_root=_LEGACY_SYSTEMD_RUNTIME_ROOT)
            result = self._sudo(["rm", "-f", path], check=False, timeout=5.0)
            if result.returncode != 0:
                detail = _text(result.stderr) or _text(result.stdout) or str(result.returncode)
                raise RuntimeError(f"could not remove legacy runtime drop-in {path}: {detail}")
            removed = True
        if removed:
            self._sudo(["systemctl", "daemon-reload"], timeout=15.0)

    def _install_runtime_file(self, target: str, content: str) -> None:
        operator_group = str(
            self._environment.get("LINGTU_OPERATOR_GROUP") or self._environment.get("USER") or "sunrise"
        ).strip()
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]{0,31}", operator_group) is None:
            raise RuntimeError("LINGTU_OPERATOR_GROUP is invalid")
        handle, raw_path = tempfile.mkstemp(prefix="lingtu-session-", suffix=".env")
        temp_path = Path(raw_path)
        try:
            with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(content)
            self._sudo(
                [
                    "install",
                    "-D",
                    "-o",
                    "root",
                    "-g",
                    operator_group,
                    "-m",
                    "0640",
                    str(temp_path),
                    target,
                ],
                timeout=10.0,
            )
        finally:
            temp_path.unlink(missing_ok=True)

    def _remove_runtime_file(self, target: str, *, check: bool = True) -> None:
        result = self._sudo(["rm", "-f", target], check=False, timeout=5.0)
        if check and result.returncode != 0:
            detail = _text(result.stderr) or _text(result.stdout) or str(result.returncode)
            raise RuntimeError(f"could not remove runtime file {target}: {detail}")

    def clear_runtime_status(self) -> None:
        """Remove stale endpoint evidence before starting a Product."""

        nav_status = _runtime_cleanup_path(
            self._environment.get("LINGTU_NAV_STATUS_FILE") or _NAV_STATUS_PATH,
            allowed_roots=_RUNTIME_CLEANUP_ROOTS,
        )
        slam_status = _runtime_cleanup_path(
            self._environment.get("LINGTU_SLAM_STATUS_JSON")
            or self._environment.get("LINGTU_SLAM_STATUS_FILE")
            or _SLAM_STATUS_PATH,
            allowed_roots=_RUNTIME_CLEANUP_ROOTS,
        )
        explore_status = _runtime_cleanup_path(
            self._environment.get("LINGTU_EXPLORE_STATUS_FILE") or _EXPLORE_STATUS_PATH,
            allowed_roots=_RUNTIME_CLEANUP_ROOTS,
        )
        snapshot_dir = _runtime_cleanup_path(
            self._environment.get("LINGTU_SLAM_CLOUD_SNAPSHOT_DIR") or _SLAM_SNAPSHOT_DIR,
            allowed_roots=_RUNTIME_CLEANUP_ROOTS,
        )
        self._sudo(
            [
                "rm",
                "-f",
                "--",
                nav_status,
                slam_status,
                f"{slam_status}.tmp",
                explore_status,
                f"{explore_status}.tmp",
            ],
            timeout=5.0,
        )
        self._sudo(["rm", "-rf", "--", snapshot_dir], timeout=5.0)

    def wait_native_nav(
        self,
        native_environment: Mapping[str, str],
        *,
        timeout_s: float,
    ) -> None:
        """Require fresh endpoint evidence for the exact native config."""

        status_path = Path(self._environment.get("LINGTU_NAV_STATUS_FILE") or _NAV_STATUS_PATH)
        deadline = self._monotonic() + timeout_s
        last: Mapping[str, Any] = {}
        while self._monotonic() < deadline:
            last = _read_json(status_path)
            try:
                age_s = self._wall_clock() - float(last.get("stamp_s"))
            except (TypeError, ValueError):
                age_s = math.inf
            native_product = last.get("native_product") if isinstance(last.get("native_product"), Mapping) else {}
            checks = (
                -0.05 <= age_s <= 1.0,
                _text(last.get("control_mode")) == native_environment.get("LINGTU_NAV_CONTROL_MODE"),
                _text(native_product.get("product")) == native_environment.get("LINGTU_PRODUCT"),
                _text(native_product.get("config_fingerprint"))
                == native_environment.get("LINGTU_NAV_CONFIG_FINGERPRINT"),
                _status_bool(last.get("publish_cmd_vel"))
                == _env_bool(native_environment.get("LINGTU_NAV_PUBLISH_CMD_VEL")),
                _status_bool(last.get("check_obstacle"))
                == _env_bool(native_environment.get("LINGTU_NAV_CHECK_OBSTACLE")),
                _status_bool(last.get("use_traversability_cost"))
                == _env_bool(native_environment.get("LINGTU_NAV_USE_TRAVERSABILITY_COST")),
                _status_bool(last.get("allow_teleop_takeover"))
                == _env_bool(native_environment.get("LINGTU_NAV_ALLOW_TELEOP_TAKEOVER")),
                _status_bool(last.get("teleop_local_planner"))
                == _env_bool(native_environment.get("LINGTU_TELEOP_LOCAL_PLANNER")),
            )
            if all(checks):
                return
            self._sleep(0.2)
        raise RuntimeError(f"native navigation config was not confirmed: {dict(last)}")

    def wait_slam(self, mode: str, *, require_map: bool, timeout_s: float) -> None:
        """Wait for fresh SLAM mode and saved-map readiness evidence."""

        status_path = Path(
            self._environment.get("LINGTU_SLAM_STATUS_JSON")
            or self._environment.get("LINGTU_SLAM_STATUS_FILE")
            or _SLAM_STATUS_PATH
        )
        max_age_s = _positive_float(
            self._environment.get("LINGTU_SLAM_STATUS_MAX_AGE_S"),
            1.5,
        )
        deadline = self._monotonic() + timeout_s
        last: Mapping[str, Any] = {}
        while self._monotonic() < deadline:
            last = _read_json(status_path)
            state = str(last.get("state") or "").upper()
            try:
                age_s = self._wall_clock() - float(last.get("snapshot_written_at_s"))
            except (TypeError, ValueError):
                age_s = math.inf
            if (
                last.get("mode") == mode
                and bool(_text(last.get("runtime_instance_id")))
                and last.get("alive") is True
                and -0.05 <= age_s <= max_age_s
                and (not require_map or bool(last.get("map_loaded")))
                and state in {"TRACKING", "MAPPING", "OK"}
            ):
                return
            self._sleep(0.25)
        raise RuntimeError(
            f"SLAM did not become ready: mode={mode} require_map={require_map} max_age_s={max_age_s} last={dict(last)}"
        )

    def wait_exploration(
        self,
        route: str,
        *,
        map_identity: MapIdentity | None,
        product_session_id: str,
        timeout_s: float,
        allow_active: bool = False,
    ) -> None:
        """Require fresh Explore evidence for the exact session binding."""

        expected_route = _required_text(route, "exploration route")
        if expected_route not in {"live", "map"}:
            raise RuntimeError(f"unsupported exploration route: {expected_route}")
        expected_session = _required_text(product_session_id, "Product session ID")
        if expected_route == "map" and map_identity is None:
            raise RuntimeError("saved-map exploration requires an exact map identity")
        if expected_route == "live" and map_identity is not None:
            raise RuntimeError("live exploration must not be bound to a saved map identity")

        status_path = Path(self._environment.get("LINGTU_EXPLORE_STATUS_FILE") or _EXPLORE_STATUS_PATH)
        max_file_age_s = _positive_float(
            self._environment.get("LINGTU_EXPLORE_STATUS_MAX_AGE_S"),
            6.0,
        )
        max_odometry_age_s = _positive_float(
            self._environment.get("LINGTU_EXPLORE_ODOMETRY_MAX_AGE_S"),
            1.0,
        )
        max_snapshot_age_s = _positive_float(
            self._environment.get("LINGTU_EXPLORE_SNAPSHOT_MAX_AGE_S"),
            3.0,
        )
        deadline = self._monotonic() + timeout_s
        last: Mapping[str, Any] = {}
        while self._monotonic() < deadline:
            last = _read_json(status_path)
            try:
                file_age_s = self._wall_clock() - status_path.stat().st_mtime
            except OSError:
                file_age_s = math.inf
            input_status = last.get("input") if isinstance(last.get("input"), Mapping) else {}
            map_status = last.get("map") if isinstance(last.get("map"), Mapping) else {}
            counters = last.get("counters") if isinstance(last.get("counters"), Mapping) else {}
            try:
                odometry_age_s = float(input_status.get("odometry_age_s"))
                snapshot_age_s = float(input_status.get("snapshot_age_s"))
                odometry_messages = int(counters.get("odometry_messages"))
                snapshot_messages = int(counters.get("snapshot_messages"))
            except (TypeError, ValueError):
                odometry_age_s = math.inf
                snapshot_age_s = math.inf
                odometry_messages = 0
                snapshot_messages = 0

            identity_ready = _exploration_map_identity_matches(
                map_status,
                route=expected_route,
                map_identity=map_identity,
                product_session_id=expected_session,
            )
            state = str(last.get("state") or "").lower()
            active = last.get("active") is True
            paused = last.get("paused") is True
            pending_fields_present = "pending_goal" in last and "pending_segment" in last
            pending_goal = last.get("pending_goal")
            pending_segment = last.get("pending_segment")
            idle_ready = (
                not active and not paused and state == "idle" and pending_goal is None and pending_segment is None
            )
            active_ready = (
                allow_active
                and active
                and (
                    (paused and state == "paused")
                    or (
                        not paused
                        and (
                            state
                            in {
                                "complete",
                                "planning",
                                "waiting_segment_snapshot",
                            }
                            or (
                                state in {"dispatching", "executing"}
                                and isinstance(pending_goal, Mapping)
                                and pending_segment is None
                            )
                            or (
                                state
                                in {
                                    "segment_dispatching",
                                    "segment_executing",
                                }
                                and pending_goal is None
                                and isinstance(pending_segment, Mapping)
                            )
                        )
                    )
                )
            )
            checks = (
                last.get("schema_version") == "lingtu.explore.status.v2",
                last.get("endpoint") == "lingtu_explore_dds",
                last.get("route") == expected_route,
                last.get("ready") is True,
                pending_fields_present,
                idle_ready or active_ready,
                -0.05 <= file_age_s <= max_file_age_s,
                -0.05 <= odometry_age_s <= max_odometry_age_s,
                -0.05 <= snapshot_age_s <= max_snapshot_age_s,
                odometry_messages > 0,
                snapshot_messages > 0,
                identity_ready,
            )
            if all(checks):
                return
            self._sleep(0.2)
        raise RuntimeError(
            f"exploration did not become ready for route={expected_route} session={expected_session}: {dict(last)}"
        )

    def start_session(
        self,
        lifecycle: ProductLifecycle,
        *,
        map_name: str,
        relocalize: bool,
        initial_pose: tuple[float, float, float] | None,
    ) -> None:
        """Start the target session and complete localization handover."""

        if lifecycle.session_mode == "none":
            return
        payload: dict[str, Any] = {
            "profile": lifecycle.product,
            "mode": lifecycle.session_mode,
            "product_session": lifecycle.product_session,
            # Product activation and autonomous task execution are separate
            # transactions. The operator starts Explore only after this switch
            # has committed its exact runtime and map identity.
            "start_task": False,
        }
        if map_name:
            payload["map_name"] = map_name
        response = self._http("POST", "/api/v1/session/start", payload, timeout_s=30.0)
        if not _response_ok(response):
            active = self._http("GET", "/api/v1/session", timeout_s=3.0)
            active_mode = _text(active.get("mode"))
            active_map = _text(active.get("active_map") or active.get("map_name"))
            if active_mode != lifecycle.session_mode or (map_name and active_map != map_name):
                raise RuntimeError(f"Product session did not start: {response.get('message') or response}")
        if lifecycle.slam_mode != "localization":
            return
        if relocalize:
            if initial_pose is not None:
                self._relocalize(map_name, initial_pose=initial_pose)
            else:
                self._relocalize(map_name, initial_pose=None)
            return
        if not self._localization_reusable(map_name):
            self._relocalize(map_name, initial_pose=None)

    def wait_navigation(
        self,
        *,
        map_name: str,
        control_mode: str,
        timeout_s: float,
    ) -> None:
        """Require Gateway readiness for the selected map and control mode."""

        deadline = self._monotonic() + timeout_s
        nav: Mapping[str, Any] = {}
        session: Mapping[str, Any] = {}
        while self._monotonic() < deadline:
            nav = self._http("GET", "/api/v1/navigation/status", timeout_s=3.0)
            session = self._http("GET", "/api/v1/session", timeout_s=3.0)
            readiness = nav.get("readiness") if isinstance(nav.get("readiness"), Mapping) else {}
            if control_mode == "autonomy":
                ready = bool(readiness.get("can_accept_goal", nav.get("can_accept_goal", False)))
            else:
                native = (
                    readiness.get("native_endpoint") if isinstance(readiness.get("native_endpoint"), Mapping) else {}
                )
                aggregate_blockers = [
                    str(blocker)
                    for blocker in (
                        *(readiness.get("blockers") or []),
                        *(native.get("blockers") or []),
                    )
                    if str(blocker)
                ]
                ready = bool(native.get("ok", False))
                if control_mode == "teleop_avoid":
                    ready = ready and not aggregate_blockers
                elif control_mode == "teleop":
                    ready = ready and not any(
                        blocker not in _TELEOP_AUTONOMY_ONLY_BLOCKERS for blocker in aggregate_blockers
                    )
            active_map = _text(session.get("active_map") or session.get("saved_active_map"))
            if ready and (not map_name or active_map == map_name):
                return
            self._sleep(0.25)
        raise RuntimeError(f"navigation did not become ready: state={nav.get('state')} map={session.get('active_map')}")

    def wait_inspection(self, *, timeout_s: float) -> None:
        """Require the inspection evidence worker selected by the Product."""

        deadline = self._monotonic() + timeout_s
        last: Mapping[str, Any] = {}
        while self._monotonic() < deadline:
            last = self._http("GET", "/api/v1/inspection/status", timeout_s=3.0)
            status = last.get("status") if isinstance(last.get("status"), Mapping) else {}
            worker = status.get("evidence_worker") if isinstance(status.get("evidence_worker"), Mapping) else {}
            if worker.get("ready") is True:
                return
            self._sleep(0.25)
        raise RuntimeError(f"inspection evidence did not become ready: {dict(last)}")

    def _maps_root(self) -> Path:
        home = Path(self._environment.get("HOME") or Path.home())
        nav_root = _text(self._environment.get("NAV_MAP_DIR"))
        if nav_root:
            return Path(nav_root)
        map_root = _text(self._environment.get("MAP_DIR"))
        default_nova = home / "data" / "nova" / "maps"
        if map_root and Path(map_root).expanduser() != default_nova:
            return Path(map_root)
        if default_nova.is_dir():
            return default_nova
        lingtu_root = home / "data" / "lingtu" / "maps"
        if lingtu_root.is_dir():
            return lingtu_root
        return Path(map_root) if map_root else default_nova

    def _localization_reusable(self, map_name: str) -> bool:
        status = self._http("GET", "/api/v1/localization/status", timeout_s=5.0)
        transform = status.get("map_odom_tf") if isinstance(status.get("map_odom_tf"), Mapping) else {}
        reported_map = _text(status.get("active_map"))
        state = str(status.get("state") or "").upper()
        return bool(
            (status.get("ready") is True or state in {"TRACKING", "READY"})
            and (not reported_map or reported_map == map_name)
            and status.get("map_loaded") is not False
            and status.get("pose_fresh") is not False
            and transform.get("valid") is True
            and _text(transform.get("frame_id") or "map") == "map"
            and _text(transform.get("child_frame_id") or "odom") == "odom"
        )

    def _relocalize(
        self,
        map_name: str,
        *,
        initial_pose: tuple[float, float, float] | None,
    ) -> None:
        attempts = _positive_int(
            self._environment.get(
                "LINGTU_RELOCALIZE_ATTEMPTS" if initial_pose is not None else "LINGTU_GLOBAL_RELOCALIZE_ATTEMPTS"
            ),
            12,
        )
        interval = _positive_float(
            self._environment.get("LINGTU_RELOCALIZE_RETRY_INTERVAL"),
            1.0,
        )
        response: Mapping[str, Any] = {}
        for attempt in range(1, attempts + 1):
            if initial_pose is None:
                response = self._http(
                    "POST",
                    "/api/v1/slam/auto_relocalize",
                    timeout_s=35.0,
                )
            else:
                x, y, yaw = initial_pose
                response = self._http(
                    "POST",
                    "/api/v1/slam/relocalize",
                    {"map_name": map_name, "x": x, "y": y, "yaw": yaw},
                    timeout_s=35.0,
                )
            if _response_ok(response):
                return
            message = str(response.get("message") or "")
            retryable = any(reason in message for reason in _RELOCALIZE_RETRY_REASONS)
            if not retryable or attempt == attempts:
                break
            self._sleep(interval)
        raise RuntimeError(f"saved-map relocalization failed: {response.get('message') or response}")

    def _http(
        self,
        method: str,
        path: str,
        payload: Mapping[str, Any] | None = None,
        *,
        timeout_s: float,
        headers: Mapping[str, str] | None = None,
    ) -> Mapping[str, Any]:
        data = None
        request_headers: dict[str, str] = {}
        operator_api_key = _text(self._environment.get("LINGTU_API_KEY"))
        if operator_api_key is not None:
            request_headers["X-API-Key"] = operator_api_key
        request_headers.update({str(key): str(value) for key, value in (headers or {}).items()})
        if payload is not None:
            data = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
            request_headers["Content-Type"] = "application/json"
        request = urllib.request.Request(  # noqa: S310 - Base URL is validated in __init__.
            f"{self._gateway_url}{path}",
            data=data,
            headers=request_headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(  # noqa: S310 - Request uses the validated base URL.
                request,
                timeout=timeout_s,
            ) as response:
                raw = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Gateway {method} {path} failed: HTTP {exc.code} {detail}") from exc
        except (OSError, urllib.error.URLError) as exc:
            raise RuntimeError(f"Gateway {method} {path} failed: {exc}") from exc
        try:
            decoded = json.loads(raw or "{}")
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Gateway {method} {path} returned invalid JSON") from exc
        if not isinstance(decoded, Mapping):
            raise RuntimeError(f"Gateway {method} {path} returned a non-object response")
        return decoded

    def _product_session_token(self) -> str | None:
        """Read the current boot-scoped session credential without persisting it."""

        direct = _session_token(self._environment.get(PRODUCT_SESSION_ID_ENV))
        if direct is not None:
            return direct
        try:
            content = Path(_session_env_path(self._session_root)).read_text(encoding="utf-8")
        except OSError:
            return None
        prefix = f"{PRODUCT_SESSION_ID_ENV}="
        for line in content.splitlines():
            if not line.startswith(prefix):
                continue
            raw = line[len(prefix) :]
            if len(raw) >= 2 and raw[0] == raw[-1] == '"':
                raw = raw[1:-1].replace('\\"', '"').replace("\\\\", "\\")
            return _session_token(raw)
        return None

    def _unit_available(self, unit: str) -> bool:
        result = self._run(
            ["systemctl", "show", unit, "-p", "LoadState", "--value"],
            check=False,
            timeout=5.0,
        )
        return result.returncode == 0 and _text(result.stdout) not in {None, "not-found"}

    def _unit_active(self, unit: str) -> bool:
        result = self._run(
            ["systemctl", "is-active", "--quiet", _unit_name(unit)],
            check=False,
            timeout=5.0,
        )
        return result.returncode == 0

    def _sudo(
        self,
        command: list[str],
        *,
        timeout: float,
        check: bool = True,
    ) -> Any:
        return self._run(
            ["sudo", "-n", *command],
            timeout=timeout,
            check=check,
        )

    def _run(
        self,
        command: list[str],
        *,
        check: bool = True,
        timeout: float,
    ) -> Any:
        result = self._runner(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        if check and result.returncode != 0:
            detail = _text(result.stderr) or _text(result.stdout) or f"exit {result.returncode}"
            raise RuntimeError(f"command failed: {' '.join(command)}: {detail}")
        return result


def execute_switch(
    control: SwitchControl,
    request: SwitchRequest,
    *,
    backend: SwitchBackend | None = None,
    environment: Mapping[str, str] | None = None,
    state_dir: str | Path | None = None,
    dry_run: bool = False,
    resolved_plan: RunPlan | None = None,
) -> SwitchReport:
    """Compile once, apply one cold-switch transaction, and fail closed."""

    process_environment = environment if environment is not None else os.environ
    backend = backend or FieldBackend(environment=process_environment)
    target_product = product_name(_required_text(request.target_product, "target Product"))
    runtime_env = control.env
    raw_current_product: str | None = request.current_product
    if raw_current_product is None:
        committed = _current_run_record(state_dir, process_environment)
        committed_env = _text(committed.get("env"))
        if committed_env is not None and committed_env != runtime_env:
            raise RuntimeError(f"current Product belongs to Env {committed_env!r}, not {runtime_env!r}")
        raw_current_product = _text(committed.get("product")) or backend.current_product()
    current_product = product_name(raw_current_product) if raw_current_product is not None else None
    report = SwitchReport(
        current_product=current_product,
        target_product=target_product,
        env=runtime_env,
        product_variant=request.product_variant,
        dry_run=dry_run,
    )
    plan: RunPlan | None = None
    map_activation: MapActivationToken | None = None
    session_stage: SessionStage | None = None
    plan_created = False
    plan_committed = False
    map_activation_committed = False
    mutated = False
    try:
        plan = resolved_plan or control.resolve(
            target_product,
            product_variant=request.product_variant,
        )
        if plan.product != target_product:
            raise RuntimeError("resolved RunPlan Product does not match switch request")
        if plan.product_variant != request.product_variant:
            raise RuntimeError("resolved RunPlan variant does not match switch request")
        if plan.env != runtime_env:
            raise RuntimeError("resolved RunPlan Env does not match ProductControl")
        plan.assert_compatible(environment=process_environment)
        target_lifecycle = _lifecycle(plan)
        if plan.process_control != "systemd":
            raise RuntimeError(f"Product {target_product} is controlled by {plan.process_control}, not systemd")
        if not plan.has_process("nav"):
            raise RuntimeError(f"Product {target_product} has no native navigation process")
        native = _native_config(plan)
        required_lifecycle = target_lifecycle.switch_policy
        if required_lifecycle != "cold_restart":
            raise RuntimeError(f"unsupported Product switch lifecycle: {required_lifecycle or 'missing'}")
        initial_pose = _initial_pose(request.initial_pose)
        map_name = _requested_map(target_lifecycle, request.map_name)
        map_path = ""
        report.run_plan = plan.as_dict()
        report.native_nav = native.as_dict()
        report.fingerprint = plan.fingerprint
        report.phases.append("preflight")
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        existing_binding = _committed_explore_binding(
            request,
            plan,
            target_lifecycle,
            map_name=map_name,
            current_product=current_product,
            state_dir=state_dir,
            environment=process_environment,
        )
        if existing_binding is not None:
            existing_plan_path, existing_map_identity, product_session_id = existing_binding
            probe_phase_count = len(report.phases)
            try:
                backend.wait_native_nav(
                    native.environment,
                    timeout_s=_ALREADY_ACTIVE_READINESS_TIMEOUT_S,
                )
                report.phases.append("existing_native_nav_ready")
                if target_lifecycle.slam_mode != "none" and plan.has_process("slam"):
                    backend.wait_slam(
                        target_lifecycle.slam_mode,
                        require_map=target_lifecycle.slam_mode == "localization",
                        timeout_s=_ALREADY_ACTIVE_READINESS_TIMEOUT_S,
                    )
                    report.phases.append("existing_slam_ready")
                backend.wait_exploration(
                    _explore_route(target_lifecycle.slam_mode),
                    map_identity=existing_map_identity,
                    product_session_id=product_session_id,
                    timeout_s=_ALREADY_ACTIVE_READINESS_TIMEOUT_S,
                    allow_active=True,
                )
                report.phases.append("existing_exploration_ready")
            except Exception:
                # Missing or stale evidence means "perform the normal cold
                # switch", never "assume healthy".
                del report.phases[probe_phase_count:]
            else:
                report.run_plan_path = str(existing_plan_path)
                report.phases.append("already_active")
                report.ok = True
                report.status = "already_active"
                return report

        backend.assert_map_save_idle()
        report.phases.append("map_save_idle")

        backend.assert_recording_idle()
        report.phases.append("recording_idle")

        run_plan_path = _run_plan_path(state_dir, process_environment, plan.fingerprint)
        if run_plan_path.exists():
            existing_plan = RunPlan.load(run_plan_path)
            if existing_plan.fingerprint != plan.fingerprint:
                raise RuntimeError("existing RunPlan does not match the resolved fingerprint")
        else:
            plan.write(run_plan_path)
            plan_created = True
        report.run_plan_path = str(run_plan_path)
        report.phases.append("plan_published")

        backend.stop_motion_and_session(current_product)
        report.phases.append("motion_stopped")
        mutated = True
        resolve_current_run_path(
            state_dir,
            environment=process_environment,
        ).unlink(missing_ok=True)
        report.phases.append("previous_run_cleared")
        if map_name:
            map_activation = backend.stage_map(map_name)
            map_path = _pointcloud_artifact_path(map_activation.target)
            report.phases.append("map_prepared")
        product_session_id = uuid.uuid4().hex
        session_stage = backend.stage_session(
            run_plan_path,
            plan,
            native.environment,
            slam_mode=target_lifecycle.slam_mode,
            map_path=map_path,
            map_identity=map_activation.target if map_activation is not None else None,
            product_session_id=product_session_id,
            parameter_overrides=request.parameter_overrides,
        )
        report.phases.append("session_staged")
        backend.clear_runtime_status()
        report.phases.append("stale_status_cleared")

        launch = control._apply_plan_for_switch(run_plan_path)
        report.launch = launch.as_dict()
        report.phases.append("processes_active")
        backend.wait_native_nav(native.environment, timeout_s=10.0)
        report.phases.append("native_nav_ready")
        if target_lifecycle.slam_mode != "none" and plan.has_process("slam"):
            backend.wait_slam(
                target_lifecycle.slam_mode,
                require_map=target_lifecycle.slam_mode == "localization",
                timeout_s=35.0,
            )
            report.phases.append("slam_ready")
        backend.start_session(
            target_lifecycle,
            map_name=map_name,
            relocalize=bool(request.relocalize),
            initial_pose=initial_pose,
        )
        report.phases.append("session_active")
        if target_lifecycle.session_mode == "navigating":
            backend.wait_navigation(
                map_name=map_name,
                control_mode=target_lifecycle.native_control_mode,
                timeout_s=45.0,
            )
            report.phases.append("navigation_ready")
        if target_lifecycle.session_mode == "exploring":
            backend.wait_exploration(
                _explore_route(target_lifecycle.slam_mode),
                map_identity=map_activation.target if map_activation is not None else None,
                product_session_id=product_session_id,
                timeout_s=45.0,
            )
            report.phases.append("exploration_ready")
        if "inspection_evidence_capture_and_result_ack" in plan.required_capabilities:
            backend.wait_inspection(timeout_s=45.0)
            report.phases.append("inspection_ready")
        if map_activation is not None:
            backend.commit_map(map_activation)
        _commit_current_run(
            run_plan_path,
            plan,
            process_environment,
            state_dir,
            product_session_id=product_session_id,
            map_name=map_name or None,
            map_identity=(map_activation.target if map_activation is not None else None),
        )
        plan_committed = True
        map_activation_committed = True
        if map_activation is not None:
            report.phases.append("map_committed")
        report.phases.append("committed")
        report.ok = True
        report.status = "active"
        return report
    except Exception as exc:
        report.error = str(exc) or exc.__class__.__name__
        if isinstance(exc, _MapRollbackFailed):
            report.status = "rollback_failed"
        elif "motion_stopped" in report.phases:
            report.status = "failed_stopped"
        elif "plan_published" in report.phases:
            report.status = "stop_unconfirmed"
        else:
            report.status = "failed"
        if mutated and plan is not None:
            try:
                backend.stop_motion_and_session(target_product)
                report.cleanup.append("motion_session:stopped")
            except Exception as cleanup_error:
                if report.status != "rollback_failed":
                    report.status = "stop_unconfirmed"
                report.cleanup.append(f"motion_session_failed:{cleanup_error}")
            if map_activation is not None and not map_activation_committed:
                try:
                    backend.restore_map(map_activation)
                    report.cleanup.append("map:restored")
                except Exception as cleanup_error:
                    report.status = "rollback_failed"
                    report.cleanup.append(f"map_failed:{cleanup_error}")
            try:
                quiesce = control.quiesce_plan(plan)
                report.cleanup.append(f"processes:{quiesce.status}")
            except Exception as cleanup_error:
                report.cleanup.append(f"processes_failed:{cleanup_error}")
            if session_stage is not None:
                try:
                    backend.rollback_session(session_stage)
                    report.cleanup.append("session:removed")
                except Exception as cleanup_error:
                    report.cleanup.append(f"session_failed:{cleanup_error}")
        if plan_created and not plan_committed and report.run_plan_path:
            try:
                Path(report.run_plan_path).unlink(missing_ok=True)
                report.cleanup.append("plan:removed")
            except OSError as cleanup_error:
                report.cleanup.append(f"plan_failed:{cleanup_error}")
        raise SwitchFailed(report) from exc


def _lifecycle(plan: RunPlan) -> ProductLifecycle:
    payload = plan.lifecycle
    lifecycle_product = product_name(_required_text(payload.get("product"), "RunPlan lifecycle Product"))
    if lifecycle_product != plan.product:
        raise RuntimeError("RunPlan lifecycle Product does not match plan")
    lifecycle_variant = _text(payload.get("product_variant"))
    if lifecycle_variant != plan.product_variant:
        raise RuntimeError("RunPlan lifecycle variant does not match plan")
    raw_candidates = payload.get("hot_switch_candidates")
    if not isinstance(raw_candidates, list | tuple):
        raise RuntimeError("RunPlan lifecycle hot_switch_candidates must be a list")
    return ProductLifecycle(
        product=lifecycle_product,
        label=_required_text(payload.get("label"), "RunPlan lifecycle label"),
        product_mode=_required_text(
            payload.get("product_mode"),
            "RunPlan lifecycle product_mode",
        ),
        product_session=_required_text(
            payload.get("product_session"),
            "RunPlan lifecycle product_session",
        ),
        session_mode=_required_text(
            payload.get("session_mode"),
            "RunPlan lifecycle session_mode",
        ),
        native_control_mode=_required_text(
            payload.get("native_control_mode"),
            "RunPlan lifecycle native_control_mode",
        ),
        slam_mode=_required_text(
            payload.get("slam_mode"),
            "RunPlan lifecycle slam_mode",
        ),
        requires_map=_required_bool(
            payload.get("requires_map"),
            "RunPlan lifecycle requires_map",
        ),
        switch_policy=_required_text(
            payload.get("switch_policy"),
            "RunPlan lifecycle switch_policy",
        ),
        product_variant=lifecycle_variant,
        default_for_session_mode=_required_bool(
            payload.get("default_for_session_mode"),
            "RunPlan lifecycle default_for_session_mode",
        ),
        hot_switch_candidates=frozenset(product_name(str(candidate)) for candidate in raw_candidates),
        online_hot_switch_supported=_required_bool(
            payload.get("online_hot_switch_supported"),
            "RunPlan lifecycle online_hot_switch_supported",
        ),
    )


def _native_config(plan: RunPlan) -> NativeNavConfig:
    return compile_native_nav_config(
        plan.product,
        {
            **plan.host_config,
            "native_control_mode": plan.native_nav.get("control_mode"),
            "native_nav": plan.native_nav,
        },
    )


def _requested_map(
    lifecycle: ProductLifecycle,
    requested_map: str | None,
) -> str:
    map_name = _text(requested_map) or ""
    if lifecycle.requires_map and not map_name:
        raise RuntimeError(f"Product {lifecycle.product} requires a map")
    return map_name


def _pointcloud_artifact(identity: MapIdentity) -> MapArtifactIdentity:
    for artifact in identity.artifacts:
        if artifact.artifact_type.strip().upper() in {
            "POINTCLOUD",
            "POINT_CLOUD",
            "PCD",
        }:
            return artifact
    raise RuntimeError(f"MapService map {identity.map_id!r} has no point-cloud artifact")


def _exploration_map_identity_matches(
    status: Mapping[str, Any],
    *,
    route: str,
    map_identity: MapIdentity | None,
    product_session_id: str,
) -> bool:
    """Match Explore's accepted snapshot to the staged Product session."""

    try:
        reset_epoch = int(status.get("reset_epoch"))
        generation = int(status.get("generation"))
        map_version = int(status.get("map_version"))
    except (TypeError, ValueError):
        return False
    common = (
        _text(status.get("frame_id")) == "map",
        _text(status.get("session_id")) == product_session_id,
        reset_epoch > 0,
        generation > 0,
    )
    if not all(common):
        return False
    if route == "live":
        return bool(
            status.get("live") is True
            and not (_text(status.get("map_id")) or "")
            and map_version == 0
            and not (_text(status.get("artifact_hash")) or "")
        )
    if route != "map" or map_identity is None:
        return False
    pointcloud = _pointcloud_artifact(map_identity)
    return bool(
        status.get("live") is False
        and _text(status.get("map_id")) == map_identity.map_id
        and map_version == _map_version_number(map_identity.version_id)
        and _text(status.get("artifact_hash"))
        == _required_text(pointcloud.sha256, "MapService point-cloud artifact sha256")
    )


def _map_version_number(version_id: str) -> int:
    match = re.search(r":v([1-9][0-9]*)$", _required_text(version_id, "map version ID"))
    if match is None:
        raise RuntimeError(f"invalid MapService version ID: {version_id!r}")
    return int(match.group(1))


def _pointcloud_artifact_path(identity: MapIdentity) -> str:
    artifact = _pointcloud_artifact(identity)
    raw_uri = _required_text(artifact.uri, "MapService point-cloud artifact URI")
    parsed = urllib.parse.urlsplit(raw_uri)
    if parsed.scheme not in {"", "file"} or parsed.query or parsed.fragment:
        raise RuntimeError("SLAM requires a local MapService point-cloud artifact")
    if parsed.scheme == "file" and parsed.netloc not in {"", "localhost"}:
        raise RuntimeError("SLAM does not accept a remote file URI")
    raw_path = urllib.parse.unquote(parsed.path) if parsed.scheme == "file" else raw_uri
    # The control tests run on Windows while mapd returns Linux artifact paths.
    # Treat a leading slash as the authority boundary; do not reinterpret it
    # through the controller host's path rules.
    if not (raw_path.startswith("/") or Path(raw_path).is_absolute()):
        raise RuntimeError("MapService point-cloud artifact path must be absolute")
    return raw_path


def _map_identity_environment(identity: MapIdentity) -> dict[str, str]:
    pointcloud = _pointcloud_artifact(identity)
    return {
        "LINGTU_MAP_ID": identity.map_id,
        "LINGTU_MAP_VERSION": identity.version_id,
        "LINGTU_MAP_FRAME": identity.frame_id,
        "LINGTU_MAP_DIR": identity.map_dir,
        "LINGTU_MAP_POINTCLOUD_SHA256": _required_text(
            pointcloud.sha256,
            "MapService point-cloud artifact sha256",
        ),
    }


def _run_plan_path(
    state_dir: str | Path | None,
    environment: Mapping[str, str],
    fingerprint: str,
) -> Path:
    root = resolve_product_state_dir(state_dir, environment=environment)
    root.mkdir(parents=True, exist_ok=True)
    return root / f"plan-{fingerprint}.json"


def _current_run_record(
    state_dir: str | Path | None,
    environment: Mapping[str, str],
) -> Mapping[str, Any]:
    path = resolve_current_run_path(state_dir, environment=environment)
    if not path.is_file():
        return {}
    payload = _read_json(path)
    if payload.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError(f"current run record has unsupported schema: {path}")
    return payload


def _committed_explore_binding(
    request: SwitchRequest,
    plan: RunPlan,
    lifecycle: ProductLifecycle,
    *,
    map_name: str,
    current_product: ProductName | None,
    state_dir: str | Path | None,
    environment: Mapping[str, str],
) -> tuple[Path, MapIdentity | None, str] | None:
    """Return a proven committed Explore binding, otherwise require restart."""

    if (
        plan.product != "explore"
        or current_product != "explore"
        or request.initial_pose is not None
        or bool(request.parameter_overrides)
    ):
        return None
    try:
        committed = _current_run_record(state_dir, environment)
        required_fields = {
            "schema_version",
            "product",
            "product_variant",
            "env",
            "run_plan_path",
            "fingerprint",
            "product_session_id",
            "map_name",
            "map_identity",
        }
        if not required_fields.issubset(committed):
            return None
        if (
            committed.get("schema_version") != CURRENT_RUN_SCHEMA
            or _text(committed.get("product")) != plan.product
            or _text(committed.get("product_variant")) != plan.product_variant
            or _text(committed.get("env")) != plan.env
            or _text(committed.get("fingerprint")) != plan.fingerprint
        ):
            return None

        run_plan_path = Path(
            _required_text(
                committed.get("run_plan_path"),
                "committed Explore RunPlan path",
            )
        )
        if not run_plan_path.is_file():
            return None
        committed_plan = RunPlan.load(run_plan_path)
        if (
            committed_plan.fingerprint != plan.fingerprint
            or committed_plan.product != plan.product
            or committed_plan.product_variant != plan.product_variant
            or committed_plan.env != plan.env
        ):
            return None

        product_session_id = _required_text(
            committed.get("product_session_id"),
            "committed Explore Product session ID",
        )
        raw_map_identity = committed.get("map_identity")
        if lifecycle.requires_map:
            if _text(committed.get("map_name")) != map_name:
                return None
            map_identity = _committed_map_identity(raw_map_identity)
        else:
            if committed.get("map_name") is not None or raw_map_identity is not None or map_name:
                return None
            map_identity = None
        return run_plan_path, map_identity, product_session_id
    except (OSError, RuntimeError, TypeError, ValueError):
        return None


def _committed_map_identity(value: Any) -> MapIdentity:
    if not isinstance(value, Mapping):
        raise RuntimeError("committed Explore map identity is missing")
    raw_artifacts = value.get("artifacts")
    if not isinstance(raw_artifacts, list) or not raw_artifacts:
        raise RuntimeError("committed Explore map artifacts are missing")
    artifacts: list[MapArtifactIdentity] = []
    for index, raw_artifact in enumerate(raw_artifacts):
        if not isinstance(raw_artifact, Mapping):
            raise RuntimeError(f"committed Explore map artifact {index} must be an object")
        artifacts.append(
            MapArtifactIdentity(
                artifact_type=_required_text(
                    raw_artifact.get("artifact_type"),
                    f"committed Explore map artifact {index} type",
                ),
                uri=_required_text(
                    raw_artifact.get("uri"),
                    f"committed Explore map artifact {index} URI",
                ),
                sha256=_required_text(
                    raw_artifact.get("sha256"),
                    f"committed Explore map artifact {index} sha256",
                ),
            )
        )
    identity = MapIdentity(
        map_id=_required_text(value.get("map_id"), "committed Explore map ID"),
        version_id=_required_text(
            value.get("version_id"),
            "committed Explore map version ID",
        ),
        frame_id=_required_text(
            value.get("frame_id"),
            "committed Explore map frame ID",
        ),
        map_dir=_required_text(
            value.get("map_dir"),
            "committed Explore map directory",
        ),
        artifacts=tuple(artifacts),
    )
    _pointcloud_artifact(identity)
    _map_version_number(identity.version_id)
    return identity


def _commit_current_run(
    run_plan_path: Path,
    plan: RunPlan,
    environment: Mapping[str, str],
    state_dir: str | Path | None,
    *,
    product_session_id: str,
    map_name: str | None,
    map_identity: MapIdentity | None,
) -> None:
    root = resolve_product_state_dir(state_dir, environment=environment)
    root.mkdir(parents=True, exist_ok=True)
    current = resolve_current_run_path(root, environment=environment)
    temp = current.with_name(f".{current.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    payload = {
        "schema_version": CURRENT_RUN_SCHEMA,
        "product": plan.product,
        "product_variant": plan.product_variant,
        "env": plan.env,
        "run_plan_path": str(run_plan_path),
        "fingerprint": plan.fingerprint,
        "product_session_id": product_session_id,
        "map_name": map_name,
        "map_identity": _map_identity_record(map_identity),
        "committed_at": time.time(),
    }
    try:
        temp.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.chmod(temp, 0o600)
        os.replace(temp, current)
    finally:
        temp.unlink(missing_ok=True)


def _map_identity_record(identity: MapIdentity | None) -> dict[str, Any] | None:
    if identity is None:
        return None
    return {
        "map_id": identity.map_id,
        "version_id": identity.version_id,
        "frame_id": identity.frame_id,
        "map_dir": identity.map_dir,
        "artifacts": [
            {
                "artifact_type": artifact.artifact_type,
                "uri": artifact.uri,
                "sha256": artifact.sha256,
            }
            for artifact in identity.artifacts
        ],
    }


def _initial_pose(
    value: tuple[float, float, float] | None,
) -> tuple[float, float, float] | None:
    if value is None:
        return None
    if len(value) != 3:
        raise RuntimeError("initial pose must contain X, Y, and YAW")
    pose = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in pose):
        raise RuntimeError("initial pose must contain finite values")
    return pose


def session_explanation(
    plan: RunPlan,
    *,
    runtime_root: str | Path = _SESSION_ROOT,
) -> dict[str, Any]:
    """Describe the single boot-scoped Product session without side effects."""

    if plan.schema_version != RUN_PLAN_SCHEMA:
        raise RuntimeError(f"unsupported RunPlan schema: {plan.schema_version!r}")
    root = _normalize_session_root(runtime_root)
    legacy_dropins = [
        {
            "process": process_name,
            "unit": unit,
            "name": name,
            "path": _dropin_path(
                unit,
                name,
                runtime_root=_LEGACY_SYSTEMD_RUNTIME_ROOT,
            ),
        }
        for process_name, unit, name in _legacy_runtime_config_dropins(plan)
    ]
    return {
        "schema_version": SESSION_EXPLANATION_SCHEMA,
        "product": plan.product,
        "env": plan.env,
        "run_plan_fingerprint": plan.fingerprint,
        "session_root": root,
        "session_root_environment": _SESSION_ROOT_ENV,
        "session_file": _session_env_path(root),
        "session_environment": {
            "LINGTU_PRODUCT_SESSION_ID": "<generated-per-switch>",
            "LINGTU_RUN_PLAN_FINGERPRINT": plan.fingerprint,
        },
        "persistent_boot_units": [process.target for process in plan.processes if process.lifecycle == "persistent"],
        "selected_mode_units": [process.target for process in plan.processes if process.lifecycle == "mode"],
        "session_guarded_units": [process.target for process in plan.processes if process.lifecycle == "mode"],
        "legacy_dropins": legacy_dropins,
    }


def _environment_file(environment: Mapping[str, str]) -> str:
    lines: list[str] = []
    for key, value in sorted(environment.items()):
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            raise RuntimeError(f"invalid environment key: {key}")
        raw = str(value)
        if any(character in raw for character in ("\x00", "\n", "\r")):
            raise RuntimeError(f"invalid environment value: {key}")
        escaped = raw.replace("\\", "\\\\").replace('"', '\\"')
        lines.append(f'{key}="{escaped}"')
    return "\n".join(lines) + "\n"


def _runtime_cleanup_path(
    value: Any,
    *,
    allowed_roots: tuple[str, ...],
) -> str:
    """Validate a Linux runtime path before passing it to privileged rm."""

    raw = _required_text(value, "runtime cleanup path").replace("\\", "/")
    normalized = posixpath.normpath(raw)
    if not normalized.startswith("/") or normalized in {"/", ".", ".."}:
        raise RuntimeError(f"unsafe runtime cleanup path: {raw}")
    allowed = any(normalized == root or normalized.startswith(f"{root}/") for root in allowed_roots)
    if not allowed or normalized in allowed_roots:
        raise RuntimeError(f"unsafe runtime cleanup path: {raw}")
    parts = tuple(part for part in normalized.split("/") if part)
    if not any(part.startswith("lingtu") for part in parts):
        raise RuntimeError(f"unsafe runtime cleanup path: {raw}")
    return normalized


def _unit_name(value: str) -> str:
    unit = _required_text(value, "systemd unit")
    if not re.fullmatch(r"[A-Za-z0-9_.@:-]+\.service", unit):
        raise RuntimeError(f"invalid systemd unit: {unit}")
    return unit


def _explore_route(slam_mode: str) -> str:
    if slam_mode == "mapping":
        return "live"
    if slam_mode == "localization":
        return "map"
    raise RuntimeError(f"exploration requires mapping or localization SLAM, got: {slam_mode}")


def _legacy_runtime_config_dropins(
    plan: RunPlan,
) -> tuple[tuple[str, str, str], ...]:
    available = {process.name: process for process in plan.available_processes}
    return tuple(
        (process_name, _unit_name(process.target), dropin_name)
        for process_name, dropin_name in _LEGACY_RUNTIME_DROPINS.items()
        if (process := available.get(process_name)) is not None
    )


def _session_root(environment: Mapping[str, str]) -> str:
    configured = environment.get(_SESSION_ROOT_ENV) or _SESSION_ROOT
    return _normalize_session_root(configured)


def _normalize_session_root(value: str | Path) -> str:
    raw = _required_text(value, "Product session root")
    if raw == _SESSION_ROOT:
        return raw
    root = Path(raw).expanduser()
    if not root.is_absolute():
        raise RuntimeError(f"Product session root must be absolute: {raw}")
    normalized = str(root.resolve(strict=False))
    posix_normalized = normalized.replace("\\", "/").rstrip("/") or "/"
    if posix_normalized == "/etc" or posix_normalized.startswith("/etc/"):
        raise RuntimeError("Product session root must not use persistent /etc storage")
    return normalized


def _session_env_path(root: str | Path) -> str:
    normalized = _normalize_session_root(root)
    if normalized.startswith("/"):
        return posixpath.join(normalized, _SESSION_ENV_FILE)
    return str(Path(normalized) / _SESSION_ENV_FILE)


def _dropin_path(
    unit: str,
    name: str,
    *,
    runtime_root: str = _LEGACY_SYSTEMD_RUNTIME_ROOT,
) -> str:
    unit_name = _unit_name(unit)
    dropin_name = _required_text(name, "systemd drop-in name")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*\.conf", dropin_name):
        raise RuntimeError(f"invalid systemd drop-in name: {dropin_name}")
    root = _required_text(runtime_root, "legacy systemd runtime root")
    if root.startswith("/"):
        return posixpath.join(root, f"{unit_name}.d", dropin_name)
    return str(Path(root) / f"{unit_name}.d" / dropin_name)


def _environment_float(
    environment: Mapping[str, str],
    key: str,
    *,
    default: float,
) -> float:
    raw = environment.get(key)
    value = default if raw is None or not str(raw).strip() else float(raw)
    if not math.isfinite(value) or value <= 0.0:
        raise RuntimeError(f"{key} must be finite and positive")
    return value


def _read_json(path: Path) -> Mapping[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, Mapping) else {}


def _native_optional_map_identity(value: Any, *, field_name: str) -> MapIdentity | None:
    if not isinstance(value, Mapping):
        raise RuntimeError(f"native {field_name} identity is missing")
    present = _required_bool(value.get("present"), f"native {field_name} present")
    return _native_map_identity(value, field_name=field_name) if present else None


def _native_map_identity(value: Any, *, field_name: str) -> MapIdentity:
    if not isinstance(value, Mapping) or value.get("present") is not True:
        raise RuntimeError(f"native {field_name} identity is not present")
    raw_artifacts = value.get("artifacts")
    if not isinstance(raw_artifacts, list) or not raw_artifacts:
        raise RuntimeError(f"native {field_name} artifacts must be a non-empty list")
    artifacts: list[MapArtifactIdentity] = []
    for index, artifact in enumerate(raw_artifacts):
        if not isinstance(artifact, Mapping):
            raise RuntimeError(f"native {field_name} artifact {index} must be an object")
        artifacts.append(
            MapArtifactIdentity(
                artifact_type=_required_text(
                    artifact.get("type"),
                    f"native {field_name} artifact {index} type",
                ),
                uri=_required_text(
                    artifact.get("uri"),
                    f"native {field_name} artifact {index} uri",
                ),
                sha256=_required_text(
                    artifact.get("sha256"),
                    f"native {field_name} artifact {index} sha256",
                ),
            )
        )
    return MapIdentity(
        map_id=_required_text(value.get("map_id"), f"native {field_name} map_id"),
        version_id=_required_text(value.get("version_id"), f"native {field_name} version_id"),
        frame_id=_required_text(value.get("frame_id"), f"native {field_name} frame_id"),
        map_dir=_required_text(value.get("map_dir"), f"native {field_name} map_dir"),
        artifacts=tuple(artifacts),
    )


def _validate_map_activation_token(token: MapActivationToken) -> None:
    if token.schema_version != MAP_ACTIVATION_TOKEN_SCHEMA:
        raise RuntimeError(f"unsupported native map token schema: {token.schema_version!r}")
    _required_text(token.activation_token, "native map activation token")
    if token.changed and token.previous == token.target:
        raise RuntimeError("changed native map identities must differ")
    if not token.changed and token.previous != token.target:
        raise RuntimeError("unchanged native map identities must match")


def _response_ok(response: Mapping[str, Any]) -> bool:
    return response.get("success", response.get("ok", False)) is True


def _status_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    text = _text(value)
    if text and text.lower() in {"1", "true", "yes", "on"}:
        return True
    if text and text.lower() in {"0", "false", "no", "off"}:
        return False
    return None


def _env_bool(value: Any) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def _positive_int(value: Any, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed > 0 else default


def _positive_float(value: Any, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) and parsed > 0.0 else default


def _required_bool(value: Any, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise RuntimeError(f"{field_name} must be boolean")
    return value


def _required_text(value: Any, field_name: str) -> str:
    text = _text(value)
    if not text:
        raise RuntimeError(f"{field_name} is required")
    return text


def _text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _session_token(value: Any) -> str | None:
    token = _text(value)
    if token is None:
        return None
    if not re.fullmatch(r"[A-Za-z0-9_.-]{16,128}", token):
        raise RuntimeError("invalid Product session credential")
    return token


__all__ = [
    "MAP_ACTIVATION_TOKEN_SCHEMA",
    "SESSION_EXPLANATION_SCHEMA",
    "SWITCH_REPORT_SCHEMA",
    "FieldBackend",
    "MapActivationToken",
    "MapArtifactIdentity",
    "MapIdentity",
    "SwitchBackend",
    "SwitchFailed",
    "SwitchReport",
    "SwitchRequest",
    "execute_switch",
    "session_explanation",
]
