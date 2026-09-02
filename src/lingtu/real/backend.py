"""Field side effects used by the real Product switch transaction."""

# Protocol declarations intentionally keep their signatures compact.
# ruff: noqa: D102

from __future__ import annotations

import ipaddress
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
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol, TypeGuard

from lingtu.products import ProductLifecycle, ProductName
from lingtu.run_plan import RUN_PLAN_SCHEMA, RunPlan
from lingtu.switch_contracts import (
    MAP_ACTIVATION_TOKEN_SCHEMA,
    MapArtifactIdentity,
    MapIdentity,
    map_identity_environment,
    map_identity_from_native,
    new_product_session_id,
    occupancy_artifact,
    octomap_artifact,
    optional_map_identity_from_native,
    pointcloud_artifact,
)
from runtime.tf import map_from_odom_transform_from_mapping

_ACTIVE_MAP_SAVE_STATES = frozenset({"WAITING_SNAPSHOT", "QUEUED", "RUNNING"})
_TERMINAL_MAP_SAVE_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELLED"})
_ACTIVE_RECORDING_STATES = frozenset({"preparing", "recording", "stopping"})
_TERMINAL_RECORDING_STATES = frozenset({"idle", "completed", "failed"})
_MAX_RECORDING_STATUS_BYTES = 5 * 1024 * 1024
_NAV_STOP_CONFIRMATION_DEFAULT_S = 4.0
_NAV_STOP_CONFIRMATION_RANGE_S = (0.5, 30.0)
_NAV_STOP_CLIENT_MARGIN_DEFAULT_S = 3.0
_NAV_STOP_CLIENT_MARGIN_RANGE_S = (1.0, 10.0)
_NAV_STOP_SUBPROCESS_MARGIN_S = 2.0
_STARTUP_ZERO_ACK_MAX_SEQUENCE_LAG = 2
_SESSION_ROOT = "/run/lingtu"
_SESSION_ROOT_ENV = "LINGTU_SESSION_ROOT"
_SESSION_ENV_FILE = "session.env"
_MAX_SESSION_ENV_BYTES = 256 * 1024
_RELOCALIZE_RETRY_REASONS = (
    "registered_cloud_unavailable",
    "waiting_for_scan",
    "scan_too_small",
    "cloud_unavailable",
    "scan_unavailable",
    "timeout",
)
_NAV_STATUS_PATH = Path("/dev/shm/lingtu/nav_endpoint_status.json")  # noqa: S108
_DRIVER_STATUS_PATH = Path("/dev/shm/lingtu/driver_status.json")  # noqa: S108
_DRIVER_MOTION_PRINCIPAL = "lingtu-driver@robot"
_EXPLORE_STATUS_PATH = Path("/dev/shm/lingtu/explore_status.json")  # noqa: S108
_SLAM_STATUS_PATH = Path("/tmp/lingtu_slam_status.json")  # noqa: S108
_MAPD_STATUS_PATH = Path("/dev/shm/lingtu/mapd_status.json")  # noqa: S108
_SLAM_SNAPSHOT_DIR = Path("/dev/shm/lingtu_slam")  # noqa: S108
_RUNTIME_CLEANUP_ROOTS = ("/dev/shm", "/run", "/tmp")  # noqa: S108
_TELEOP_AVOID_STARTUP_BLOCKERS = frozenset(
    {
        "real_runtime_evidence_missing_or_stale",
        "native_resume_required",
    }
)
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
class SessionFile:
    """One ephemeral session file installed for a Product run."""

    path: str
    previous_content: str | None = None


@dataclass(frozen=True)
class SessionStage:
    """Rollback token for the ephemeral files created by one session."""

    files: tuple[SessionFile, ...]
    environment: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class MapActivationToken:
    """Opaque native compensation token plus verified map identities."""

    target: MapIdentity
    previous: MapIdentity | None
    changed: bool
    activation_token: str
    schema_version: str = MAP_ACTIVATION_TOKEN_SCHEMA



class MapRollbackFailed(RuntimeError):
    """A prepared map could not be compensated and must stay fail-closed."""


class SwitchBackend(Protocol):
    """Deployment and Gateway effects required by ProductControl."""

    def assert_map_save_idle(self) -> None: ...

    def assert_recording_idle(self) -> None: ...

    def stop_motion(self, current_product: ProductName | None) -> None: ...

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
        map_identity: MapIdentity | None = None,
        product_session_id: str | None = None,
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

    def wait_slam(
        self,
        mode: str,
        *,
        require_map: bool,
        require_localization: bool = False,
        timeout_s: float,
    ) -> None: ...

    def wait_exploration(
        self,
        route: str,
        *,
        map_identity: MapIdentity | None,
        product_session_id: str,
        timeout_s: float,
        allow_active: bool = False,
    ) -> None: ...

    def prepare_localization(
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

    def wait_motion_output(self, control_mode: str, *, timeout_s: float) -> None: ...

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

    def assert_map_save_idle(self) -> None:
        """Reject a Product switch while a durable SaveMap transaction is active."""

        map_api_key = _text(self._environment.get("LINGTU_MAP_API_KEY") or self._environment.get("LINGTU_API_KEY"))
        headers = {"X-API-Key": map_api_key} if map_api_key is not None else None
        response = self._http(
            "GET",
            "/api/v1/maps/operations?limit=1000",
            timeout_s=5.0,
            headers=headers,
            allowed_error_statuses=(503,),
        )
        if response.get("_http_status") == 503:
            detail = _text(response.get("detail")) or ""
            if detail.startswith("maps.service is unavailable"):
                return
            raise RuntimeError("map_save_status_unavailable: Gateway map service is unhealthy")
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
        candidates.append(Path("/opt/lingtu/current/bin/lingtu_recorder"))
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

    def stop_motion(self, current_product: ProductName | None) -> None:
        """Stop native motion without requiring the Python Host to exist."""

        nav_unit = str(self._environment.get("LINGTU_NAV_SERVICE") or "lt-nav.service")
        nav_active = self._unit_active(nav_unit)
        if current_product is not None and not nav_active:
            raise RuntimeError(
                "native navigation stop is unavailable; zero speed, driver ACK, "
                "and odometry stillness were not confirmed"
            )
        if nav_active:
            nav_control = str(
                self._environment.get("LINGTU_NAV_CONTROL_BIN")
                or "/opt/lingtu/current/bin/lingtu_nav_control"
            )
            client_timeout_ms, subprocess_timeout_s = _nav_stop_timeout_budget(self._environment)
            command = [nav_control, "stop", "product_mode_switch"]
            domain_id = _text(self._environment.get("LINGTU_DDS_DOMAIN_ID"))
            if domain_id is not None:
                command.extend(("--domain-id", domain_id))
            command.extend(("--timeout-ms", str(client_timeout_ms)))
            self._run(command, timeout=subprocess_timeout_s)

    def stage_map(self, map_name: str) -> MapActivationToken:
        """Stage one exact active map through native typed DDS control."""

        response = self._mapctl("stage", map_name, timeout_s=20.0)
        target = map_identity_from_native(response.get("target"), field_name="target map")
        previous = optional_map_identity_from_native(response.get("previous"), field_name="previous map")
        active = map_identity_from_native(response.get("active"), field_name="active map")
        activation_token = _required_text(
            response.get("activation_token"),
            "native map activation token",
        )
        if target.map_id != map_name or active != target:
            raise MapRollbackFailed("native map staging did not activate the exact requested identity")
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
        active = optional_map_identity_from_native(response.get("active"), field_name="restored active map")
        if active != token.previous:
            raise RuntimeError("native map restore did not recover the prepared previous identity")

    def commit_map(self, token: MapActivationToken) -> None:
        """Verify the exact staged identity before committing the Product."""

        _validate_map_activation_token(token)
        response = self._mapctl("verify", token.activation_token, timeout_s=10.0)
        active = map_identity_from_native(response.get("active"), field_name="verified active map")
        if active != token.target:
            raise RuntimeError("native active map identity changed before Product commit")

    def _mapctl(self, operation: str, operand: str, *, timeout_s: float) -> Mapping[str, Any]:
        binary = str(self._environment.get("LINGTU_MAPCTL_BIN") or "/opt/lingtu/current/bin/lingtu-mapctl")
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
        map_identity: MapIdentity | None = None,
        product_session_id: str | None = None,
    ) -> SessionStage:
        """Atomically publish one boot-scoped Product session environment."""

        session_id = _required_text(
            product_session_id or new_product_session_id(),
            "Product session id",
        )
        session_path = _session_env_path(self._session_root)
        previous_content = self._session_file_snapshot(session_path)
        try:
            environment = self._session_environment(
                run_plan_path=run_plan_path,
                plan=plan,
                native_environment=native_environment,
                slam_mode=slam_mode,
                map_identity=map_identity,
                product_session_id=session_id,
            )
            self._install_runtime_file(session_path, _environment_file(environment))
        except Exception as exc:
            # Restore the exact previous environment when replacement fails.
            try:
                if previous_content is None:
                    self._remove_runtime_file(session_path, check=False)
                else:
                    self._install_runtime_file(session_path, previous_content)
            except Exception as cleanup_error:
                raise RuntimeError(f"session staging failed: {exc}; cleanup failed: {cleanup_error}") from exc
            raise
        return SessionStage(
            files=(
                SessionFile(
                    path=session_path,
                    previous_content=previous_content,
                ),
            ),
            environment=environment,
        )

    @staticmethod
    def _session_file_snapshot(path: str) -> str | None:
        candidate = Path(path)
        if not (candidate.exists() or candidate.is_symlink()):
            return None
        try:
            raw = candidate.read_bytes()
            if len(raw) > _MAX_SESSION_ENV_BYTES:
                raise ValueError("Product session environment exceeds the byte limit")
            return raw.decode("utf-8")
        except (OSError, UnicodeDecodeError, ValueError) as exc:
            raise RuntimeError("existing Product session environment is unavailable") from exc

    def _session_environment(
        self,
        *,
        run_plan_path: Path,
        plan: RunPlan,
        native_environment: Mapping[str, str],
        slam_mode: str,
        map_identity: MapIdentity | None,
        product_session_id: str,
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
        expected_native = plan.native_process_environment
        unexpected = sorted(set(native) - set(expected_native))
        if unexpected:
            raise RuntimeError(f"invalid native navigation environment keys: {unexpected}")
        missing = sorted(set(expected_native) - set(native))
        if missing:
            raise RuntimeError(f"missing native navigation environment keys: {missing}")
        mismatched = sorted(key for key, value in native.items() if value != expected_native[key])
        if mismatched:
            raise RuntimeError(f"native navigation environment values do not match RunPlan: {mismatched}")

        if slam_mode not in {"mapping", "localization", "none"}:
            raise RuntimeError(f"unsupported SLAM mode: {slam_mode}")
        if slam_mode == "localization" and map_identity is None:
            raise RuntimeError("localization requires an exact MapService map identity")
        if slam_mode != "localization" and map_identity is not None:
            raise RuntimeError(f"{slam_mode} must not inherit a saved-map identity")

        map_path = (
            _pointcloud_artifact_path(map_identity, self._maps_root())
            if map_identity is not None
            else ""
        )
        environment = {
            "LINGTU_PRODUCT": plan.product,
            "LINGTU_ENV": plan.env,
            "LINGTU_RUN_PLAN": str(run_plan_path),
            "LINGTU_SESSION_ROOT": str(run_plan_path.parent),
            "LINGTU_PRODUCT_SESSION_ID": product_session_id,
            "LINGTU_ENDPOINT_TRANSPORT": endpoint_transport,
            "LINGTU_ENDPOINT_CONTRACT": endpoint_contract,
            "LINGTU_COMMAND_OUTPUT_MODE": command_output_mode,
            "LINGTU_HARDWARE_CONTROL_BOUNDARY": hardware_control_boundary,
            "LINGTU_SLAM_MODE": slam_mode,
            "LINGTU_SLAM_MAP": map_path,
            # These values deliberately override persistent unit defaults. The
            # navigation runner only re-enables a planner map for localization.
            "OCTOPLANNER_MAP_PATH": "",
            "FAR_OCCUPANCY_PATH": "",
            "EXPLORE_OCCUPANCY_PATH": "",
            **native,
        }
        if map_identity is not None:
            planner = native["NAV_GLOBAL_PLANNER"]
            if planner == "octoplanner3d":
                artifact = octomap_artifact(map_identity)
                planner_environment = {
                    "OCTOPLANNER_MAP_PATH": _local_artifact_path(
                        artifact,
                        purpose="OctoPlanner3D",
                        map_root=self._maps_root(),
                        map_id=map_identity.map_id,
                    )
                }
            elif planner == "far":
                artifact = occupancy_artifact(map_identity)
                planner_environment = {
                    "FAR_OCCUPANCY_PATH": _local_artifact_path(
                        artifact,
                        purpose="FAR",
                        map_root=self._maps_root(),
                        map_id=map_identity.map_id,
                    )
                }
            else:
                raise RuntimeError(f"unsupported native global planner: {planner}")
            environment.update(map_identity_environment(map_identity))
            environment.update(planner_environment)
            if plan.product == "explore":
                environment["EXPLORE_OCCUPANCY_PATH"] = _local_artifact_path(
                    occupancy_artifact(map_identity),
                    purpose="Explore coverage",
                    map_root=self._maps_root(),
                    map_id=map_identity.map_id,
                )
        return dict(sorted(environment.items()))

    def rollback_session(self, staged: SessionStage) -> None:
        """Restore the exact previous Product session environment."""

        errors: list[str] = []
        for snapshot in reversed(staged.files):
            try:
                if snapshot.previous_content is None:
                    self._remove_runtime_file(snapshot.path)
                else:
                    self._install_runtime_file(
                        snapshot.path,
                        snapshot.previous_content,
                    )
            except Exception as exc:
                errors.append(f"{snapshot.path}: {exc}")
        if errors:
            raise RuntimeError("failed to remove staged runtime config: " + "; ".join(errors))

    def remove_session(self, _plan: RunPlan) -> None:
        """Remove the boot-scoped Product session."""

        self._remove_runtime_file(_session_env_path(self._session_root))

    def _install_runtime_file(self, target: str, content: str) -> None:
        operator_group = str(
            self._environment.get("LINGTU_OPERATOR_GROUP") or self._environment.get("USER") or "lingtu"
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
                _text(last.get("local_planner"))
                == native_environment.get("LINGTU_NAV_LOCAL_PLANNER_BACKEND", "cmu"),
                _text(native_product.get("product")) == native_environment.get("LINGTU_PRODUCT"),
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

    def wait_slam(
        self,
        mode: str,
        *,
        require_map: bool,
        require_localization: bool = False,
        timeout_s: float,
    ) -> None:
        """Wait for a tracking frontend and, when requested, saved-map alignment."""

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
            map_odom_tf = last.get("map_odom_tf")
            localized = (
                isinstance(map_odom_tf, Mapping)
                and map_odom_tf.get("valid") is True
            )
            try:
                age_s = self._wall_clock() - float(last.get("snapshot_written_at_s"))
            except (TypeError, ValueError):
                age_s = math.inf
            frontend_ready = (
                last.get("mode") == mode
                and bool(_text(last.get("runtime_instance_id")))
                and last.get("alive") is True
                and -0.05 <= age_s <= max_age_s
                and (not require_map or bool(last.get("map_loaded")))
                and state in {"TRACKING", "MAPPING", "OK"}
            )
            localization_ready = localized and _raw_map_tracking_healthy(last)
            if frontend_ready and (not require_localization or localization_ready):
                return
            self._sleep(0.25)
        raise RuntimeError(
            "SLAM did not become ready: "
            f"mode={mode} require_map={require_map} "
            f"require_localization={require_localization} max_age_s={max_age_s} "
            f"last={dict(last)}"
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
                last.get("product_session_id") == expected_session,
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

    def prepare_localization(
        self,
        lifecycle: ProductLifecycle,
        *,
        map_name: str,
        relocalize: bool,
        initial_pose: tuple[float, float, float] | None,
    ) -> None:
        """Complete localization handover for a saved-map Product."""

        if lifecycle.slam_mode != "localization":
            return
        if relocalize:
            self._relocalize(map_name, initial_pose=initial_pose)
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
            if map_name:
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
                    input_gate = (
                        native.get("input_gate")
                        if isinstance(native.get("input_gate"), Mapping)
                        else {}
                    )
                    loop_health = (
                        native.get("control_loop_health")
                        if isinstance(native.get("control_loop_health"), Mapping)
                        else {}
                    )
                    ready = (
                        native.get("status_available") is True
                        and input_gate.get("ready") is True
                        and loop_health.get("ready") is True
                        and loop_health.get("healthy") is True
                        and not any(
                            blocker not in _TELEOP_AVOID_STARTUP_BLOCKERS
                            for blocker in aggregate_blockers
                        )
                    )
                elif control_mode == "teleop":
                    ready = ready and not any(
                        blocker not in _TELEOP_AUTONOMY_ONLY_BLOCKERS for blocker in aggregate_blockers
                    )
            active_map = _text(session.get("active_map") or session.get("saved_active_map"))
            if ready and (not map_name or active_map == map_name):
                return
            self._sleep(0.25)
        raise RuntimeError(f"navigation did not become ready: state={nav.get('state')} map={session.get('active_map')}")

    def wait_motion_output(self, control_mode: str, *, timeout_s: float) -> None:
        """Require one exact idle-zero acknowledgement through the selected driver.

        This gate runs before the RunPlan is committed.  It therefore reads the
        two native status snapshots directly instead of relying on the active
        Product projection, which intentionally does not exist yet.
        """

        expected_mode = _required_text(control_mode, "native control mode").lower()
        if expected_mode not in {"teleop", "teleop_avoid"}:
            raise RuntimeError(f"motion-output startup gate is unsupported for {expected_mode}")
        nav_path = Path(self._environment.get("LINGTU_NAV_STATUS_FILE") or _NAV_STATUS_PATH)
        driver_path = Path(self._environment.get("LINGTU_DRIVER_STATUS_FILE") or _DRIVER_STATUS_PATH)
        nav_max_age_s = _positive_float(
            self._environment.get("LINGTU_NAV_STATUS_MAX_AGE_S"),
            1.0,
        )
        driver_max_age_s = _positive_float(
            self._environment.get("LINGTU_DRIVER_STATUS_MAX_AGE_S"),
            1.5,
        )
        expected_driver = _expected_driver_configuration(self._session_root)
        deadline = self._monotonic() + timeout_s
        last_reason = "native_status_missing"
        last_evidence: Mapping[str, Any] = {}
        while self._monotonic() < deadline:
            nav = _read_json(nav_path)
            driver = _read_json(driver_path)
            last_reason, last_evidence = motion_output_startup_evidence(
                nav,
                driver,
                expected_mode=expected_mode,
                expected_driver=expected_driver,
                wall_clock_s=self._wall_clock(),
                nav_max_age_s=nav_max_age_s,
                driver_max_age_s=driver_max_age_s,
            )
            if not last_reason:
                return
            self._sleep(0.1)
        raise RuntimeError(
            "field driver did not acknowledge the exact startup zero: "
            f"reason={last_reason} evidence={dict(last_evidence)}"
        )

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
        nav_root = _text(self._environment.get("NAV_MAP_DIR"))
        return Path(nav_root) if nav_root else Path("/var/lib/lingtu/maps")

    def _localization_reusable(self, map_name: str) -> bool:
        status = self._http("GET", "/api/v1/localization/status", timeout_s=5.0)
        transform = map_from_odom_transform_from_mapping(status.get("map_odom_tf"))
        raw = status.get("raw") if isinstance(status.get("raw"), Mapping) else {}
        reported_map = _text(status.get("active_map"))
        state = str(status.get("state") or "").upper()
        return (
            (status.get("ready") is True or state in {"TRACKING", "READY"})
            and (not reported_map or reported_map == map_name)
            and status.get("map_loaded") is not False
            and status.get("pose_fresh") is not False
            and transform is not None
            and _raw_map_tracking_healthy(raw)
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
                payload: dict[str, Any] = {
                    "map_name": map_name,
                    "mode": "global",
                }
            else:
                x, y, yaw = initial_pose
                payload = {
                    "map_name": map_name,
                    "mode": "seeded",
                    "initial_pose": {"x": x, "y": y, "yaw": yaw},
                }
            response = self._http(
                "POST",
                "/api/v1/localization/relocalizations",
                payload,
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
        allowed_error_statuses: tuple[int, ...] = (),
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
        status_code: int | None = None
        try:
            with urllib.request.urlopen(  # noqa: S310 - Request uses the validated base URL.
                request,
                timeout=timeout_s,
            ) as response:
                raw = response.read().decode("utf-8")
                status_code = int(getattr(response, "status", 200) or 200)
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            if exc.code not in allowed_error_statuses:
                raise RuntimeError(f"Gateway {method} {path} failed: HTTP {exc.code} {detail}") from exc
            raw = detail
            status_code = int(exc.code)
        except (OSError, urllib.error.URLError) as exc:
            raise RuntimeError(f"Gateway {method} {path} failed: {exc}") from exc
        try:
            decoded = json.loads(raw or "{}")
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Gateway {method} {path} returned invalid JSON") from exc
        if not isinstance(decoded, Mapping):
            raise RuntimeError(f"Gateway {method} {path} returned a non-object response")
        if status_code is not None and status_code >= 400:
            decoded = {**decoded, "_http_status": status_code}
        return decoded

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




def _exploration_map_identity_matches(
    status: Mapping[str, Any],
    *,
    route: str,
    map_identity: MapIdentity | None,
) -> bool:
    """Match Explore's accepted snapshot to the expected map identity."""

    try:
        reset_epoch = int(status.get("reset_epoch"))
        generation = int(status.get("generation"))
        map_content_epoch = status.get("map_content_epoch")
    except (TypeError, ValueError):
        return False
    if isinstance(map_content_epoch, bool) or not isinstance(map_content_epoch, int):
        return False
    common = (
        _text(status.get("frame_id")) == "map",
        reset_epoch > 0,
        generation > 0,
    )
    if not all(common):
        return False
    if route == "live":
        return (
            status.get("live") is True
            and not (_text(status.get("map_id")) or "")
            and map_content_epoch == 0
        )
    if route != "map" or map_identity is None:
        return False
    return (
        status.get("live") is False
        and _text(status.get("map_id")) == map_identity.map_id
        and map_content_epoch == map_identity.content_epoch
    )




def _pointcloud_artifact_path(identity: MapIdentity, map_root: Path) -> str:
    return _local_artifact_path(
        pointcloud_artifact(identity),
        purpose="SLAM",
        map_root=map_root,
        map_id=identity.map_id,
    )


def _local_artifact_path(
    artifact: MapArtifactIdentity,
    *,
    purpose: str,
    map_root: Path,
    map_id: str,
) -> str:
    raw_uri = _required_text(artifact.uri, f"MapService {purpose} artifact URI")
    parsed = urllib.parse.urlsplit(raw_uri)
    if parsed.scheme not in {"", "file"} or parsed.query or parsed.fragment:
        raise RuntimeError(f"{purpose} requires a local MapService artifact")
    if parsed.scheme == "file" and parsed.netloc not in {"", "localhost"}:
        raise RuntimeError(f"{purpose} does not accept a remote file URI")
    raw_path = urllib.parse.unquote(parsed.path if parsed.scheme == "file" else raw_uri)
    artifact_path = _normalized_map_path(
        raw_path,
        field=f"MapService {purpose} artifact path",
    )
    activated_map_dir = _normalized_map_path(
        (map_root / map_id).as_posix(),
        field="MapService map directory",
    )
    try:
        contained = (
            artifact_path != activated_map_dir
            and posixpath.commonpath((activated_map_dir, artifact_path)) == activated_map_dir
        )
    except ValueError:
        contained = False
    if not contained:
        raise RuntimeError(f"MapService {purpose} artifact must be inside the activated map directory")
    return artifact_path


def _normalized_map_path(value: Any, *, field: str) -> str:
    """Return one canonical absolute robot path without traversal semantics."""

    raw = _required_text(value, field)
    if "\x00" in raw or "\\" in raw or not raw.startswith("/"):
        raise RuntimeError(f"{field} must be absolute POSIX path")
    if any(part in {"", ".", ".."} for part in raw.split("/")[1:]):
        raise RuntimeError(f"{field} must be a normalized absolute POSIX path")
    normalized = posixpath.normpath(raw)
    if normalized != raw or normalized == "/":
        raise RuntimeError(f"{field} must be a normalized absolute POSIX path")
    return normalized


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


def _bounded_environment_float(
    environment: Mapping[str, str],
    key: str,
    *,
    default: float,
    minimum: float,
    maximum: float,
) -> float:
    try:
        value = _environment_float(environment, key, default=default)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{key} must be a finite number") from exc
    if value < minimum or value > maximum:
        raise RuntimeError(f"{key} must be within [{minimum}, {maximum}] seconds")
    return value


def _nav_stop_timeout_budget(environment: Mapping[str, str]) -> tuple[int, float]:
    confirmation_s = _bounded_environment_float(
        environment,
        "LINGTU_NAV_STOP_CONFIRMATION_TIMEOUT_S",
        default=_NAV_STOP_CONFIRMATION_DEFAULT_S,
        minimum=_NAV_STOP_CONFIRMATION_RANGE_S[0],
        maximum=_NAV_STOP_CONFIRMATION_RANGE_S[1],
    )
    margin_s = _bounded_environment_float(
        environment,
        "LINGTU_NAV_STOP_CLIENT_MARGIN_S",
        default=_NAV_STOP_CLIENT_MARGIN_DEFAULT_S,
        minimum=_NAV_STOP_CLIENT_MARGIN_RANGE_S[0],
        maximum=_NAV_STOP_CLIENT_MARGIN_RANGE_S[1],
    )
    client_timeout_ms = math.ceil((confirmation_s + margin_s) * 1000.0)
    subprocess_timeout_s = client_timeout_ms / 1000.0 + _NAV_STOP_SUBPROCESS_MARGIN_S
    return client_timeout_ms, subprocess_timeout_s


def motion_output_startup_evidence(
    nav: Mapping[str, Any],
    driver: Mapping[str, Any],
    *,
    expected_mode: str,
    expected_driver: Mapping[str, str],
    wall_clock_s: float,
    nav_max_age_s: float,
    driver_max_age_s: float,
) -> tuple[str, Mapping[str, Any]]:
    """Return an empty reason only for one physically acknowledged idle zero."""

    final_output = nav.get("final_output") if isinstance(nav.get("final_output"), Mapping) else {}
    driver_control = nav.get("driver_control") if isinstance(nav.get("driver_control"), Mapping) else {}
    final_twist = nav.get("final_cmd_vel") if isinstance(nav.get("final_cmd_vel"), Mapping) else {}
    operator_motion = nav.get("operator_motion") if isinstance(nav.get("operator_motion"), Mapping) else {}
    operator_status = operator_motion.get("status") if isinstance(operator_motion.get("status"), Mapping) else {}
    driver_dds = driver.get("dds") if isinstance(driver.get("dds"), Mapping) else {}
    adapter = driver.get("adapter") if isinstance(driver.get("adapter"), Mapping) else {}
    control = driver.get("control") if isinstance(driver.get("control"), Mapping) else {}
    output_ack = driver.get("output_ack") if isinstance(driver.get("output_ack"), Mapping) else {}
    producer_boot_id = str(final_output.get("producer_boot_id") or "").strip()
    output_sequence = final_output.get("output_sequence")
    accepted_output_sequence = driver_control.get("accepted_output_sequence")
    driver_status_output_sequence = output_ack.get("output_sequence")
    target_host, target_port = _target_endpoint(adapter.get("target"))
    evidence = {
        "nav_schema": nav.get("schema_version"),
        "nav_age_s": _status_age_s(nav, wall_clock_s),
        "control_mode": nav.get("control_mode"),
        "final_cmd_vel": dict(final_twist),
        "operator_motion": dict(operator_motion),
        "producer_boot_id": producer_boot_id,
        "output_sequence": output_sequence,
        "driver_control": dict(driver_control),
        "driver_schema": driver.get("schema_version"),
        "driver_age_s": _status_age_s(driver, wall_clock_s),
        "matched_cmd_vel_writers": driver_dds.get("matched_cmd_vel_writers"),
        "driver_backend": driver.get("backend"),
        "driver_target": adapter.get("target"),
        "driver_protocol": adapter.get("protocol"),
        "driver_host": target_host,
        "driver_port": target_port,
        "driver_owner": adapter.get("control_owner_id"),
        "driver_fsm": control.get("fsm"),
        "driver_output_ack": dict(output_ack),
    }
    if not nav or not driver:
        return "native_status_missing", evidence
    nav_age_s = evidence["nav_age_s"]
    driver_age_s = evidence["driver_age_s"]
    if not isinstance(nav_age_s, float) or not -0.05 <= nav_age_s <= nav_max_age_s:
        return "nav_status_stale", evidence
    if not isinstance(driver_age_s, float) or not -0.05 <= driver_age_s <= driver_max_age_s:
        return "driver_status_stale", evidence
    if nav.get("schema_version") != "lingtu.nav.endpoint.status.v1":
        return "nav_status_schema_mismatch", evidence
    if str(nav.get("control_mode") or "").strip().lower() != expected_mode:
        return "nav_control_mode_mismatch", evidence
    if (
        operator_motion.get("interface_enabled") is not True
        or operator_motion.get("authority_owner") != "native_endpoint"
        or operator_motion.get("control_mode") != expected_mode
        or operator_motion.get("control_ack_scope") != "claim_hold_release"
        or operator_motion.get("sample_evidence") != "status_sequences"
        or operator_motion.get("ack_publish_failed") != 0
        or operator_motion.get("status_publish_failed") != 0
    ):
        return "operator_motion_interface_not_ready", evidence
    if (
        operator_status.get("has_active_authority") is not False
        or operator_status.get("has_active_sample") is not False
        or not _zero_native_twist(
            operator_status.get("final_cmd_vel") if isinstance(operator_status.get("final_cmd_vel"), Mapping) else {}
        )
    ):
        return "operator_motion_not_idle", evidence
    if not _zero_native_twist(final_twist):
        return "startup_output_not_zero", evidence
    expected_backend = expected_driver.get("backend")
    if (
        driver.get("schema_version") != "lingtu.driver.status.v2"
        or driver.get("role") != "driver"
        or driver.get("backend") != expected_backend
    ):
        return "driver_status_schema_mismatch", evidence
    if driver.get("connected") is not True or driver.get("ready") is not True:
        return "driver_not_ready", evidence
    if (
        driver_dds.get("topic") != "/nav/cmd_vel"
        or driver_dds.get("wire_topic") != "rt/nav/cmd_vel"
        or driver_dds.get("cmd_vel_writer_ready") is not True
        or driver_dds.get("matched_cmd_vel_writers") != 1
    ):
        return "cmd_vel_writer_ownership_invalid", evidence
    if expected_backend == "doso":
        expected_host = expected_driver.get("host") or ""
        expected_port = int(expected_driver.get("port") or 0)
        if (
            not _remote_host(target_host)
            or target_port is None
            or (target_host, target_port) != (expected_host, expected_port)
        ):
            return "driver_target_invalid", evidence
        expected_protocol = "brainstem_grpc"
        expected_owner = "grpc"
        expected_owner_id = _DRIVER_MOTION_PRINCIPAL
        expected_lease = True
    elif expected_backend == "go2":
        expected_interface = expected_driver.get("network_interface") or ""
        if adapter.get("target") != f"dds://{expected_interface}/rt/api/sport/request":
            return "driver_target_invalid", evidence
        expected_protocol = "unitree_sdk2"
        expected_owner = "none"
        expected_owner_id = ""
        expected_lease = False
    else:
        return "driver_backend_invalid", evidence
    if (
        adapter.get("protocol") != expected_protocol
        or adapter.get("control_owner") != expected_owner
        or str(adapter.get("control_owner_id") or "") != expected_owner_id
        or control.get("initial_zero_acknowledged") is not True
        or control.get("motors_enabled") is not True
        or control.get("critical_fault") is not False
        or control.get("control_assured") is not True
        or control.get("lease_valid") is not expected_lease
        or str(control.get("fsm") or "").strip().lower() not in {"standing", "walking"}
    ):
        return "driver_control_not_ready", evidence
    if (
        final_output.get("published") is not True
        or not producer_boot_id
        or not _strict_positive_int(output_sequence)
        or driver_control.get("received") is not True
        or driver_control.get("ready") is not True
        or driver_control.get("fresh") is not True
        or driver_control.get("last_command_accepted") is not True
        or driver_control.get("accepted_producer_boot_id") != producer_boot_id
        or not _strict_positive_int(accepted_output_sequence)
        or accepted_output_sequence > output_sequence
        or output_sequence - accepted_output_sequence > _STARTUP_ZERO_ACK_MAX_SEQUENCE_LAG
        or output_ack.get("accepted") is not True
        or output_ack.get("producer_boot_id") != producer_boot_id
        or not _strict_positive_int(driver_status_output_sequence)
        or driver_status_output_sequence > accepted_output_sequence
    ):
        return "startup_zero_ack_identity_mismatch", evidence
    return "", evidence


def _status_age_s(payload: Mapping[str, Any], wall_clock_s: float) -> float | None:
    stamp = payload.get("stamp_s")
    if isinstance(stamp, bool):
        return None
    try:
        parsed = float(stamp)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return wall_clock_s - parsed


def _zero_native_twist(value: Mapping[str, Any]) -> bool:
    components: list[float] = []
    for name in ("vx", "vy", "wz"):
        raw = value.get(name)
        if isinstance(raw, bool):
            return False
        try:
            parsed = float(raw)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(parsed):
            return False
        components.append(parsed)
    return all(abs(component) <= 1e-6 for component in components)


def _strict_positive_int(value: Any) -> TypeGuard[int]:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _target_endpoint(target: Any) -> tuple[str, int | None]:
    value = str(target or "").strip()
    if not value:
        return "", None
    parsed = urllib.parse.urlsplit(value if "://" in value else f"//{value}")
    try:
        port = parsed.port
    except ValueError:
        return "", None
    return parsed.hostname or "", port


def _expected_driver_configuration(session_root: str | Path) -> dict[str, str]:
    """Read the physical driver selection from the trusted Product session."""

    path = Path(_session_env_path(session_root))
    try:
        raw = path.read_bytes()
        if len(raw) > _MAX_SESSION_ENV_BYTES:
            raise ValueError("Product session environment exceeds the byte limit")
        text = raw.decode("utf-8")
    except (OSError, UnicodeError, ValueError) as exc:
        raise RuntimeError(f"Product session driver configuration is unavailable: {path}") from exc

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        match = re.fullmatch(r'([A-Z][A-Z0-9_]*)="((?:[^"\\]|\\.)*)"', line)
        if match is None:
            raise RuntimeError(f"invalid Product session environment line {line_number}: {path}")
        key, escaped = match.groups()
        if key in values:
            raise RuntimeError(f"duplicate Product session environment key: {key}")
        values[key] = escaped.replace('\\"', '"').replace("\\\\", "\\")

    backend = values.get("LINGTU_DRIVER_BACKEND", "").strip()
    if backend == "go2":
        network_interface = values.get("LINGTU_DRIVER_NETWORK_INTERFACE", "").strip()
        if re.fullmatch(r"[A-Za-z0-9_.:-]+", network_interface) is None:
            raise RuntimeError("trusted Product session has invalid Go2 network interface")
        return {
            "backend": backend,
            "network_interface": network_interface,
        }
    if backend == "doso":
        host, port = _target_endpoint(values.get("LINGTU_DRIVER_TARGET"))
        if not _remote_host(host) or port is None:
            raise RuntimeError("trusted Product session has invalid Doso driver target")
        return {"backend": backend, "host": host, "port": str(port)}
    raise RuntimeError(f"trusted Product session has unsupported driver backend: {backend or '<unset>'}")


def _remote_host(host: str) -> bool:
    if not host:
        return False
    try:
        return not ipaddress.ip_address(host).is_loopback
    except ValueError:
        return host.lower() not in {"localhost", "ip6-localhost"}


def _read_json(path: Path) -> Mapping[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, Mapping) else {}


def _raw_map_tracking_healthy(status: Mapping[str, Any]) -> bool:
    tracking = status.get("track_against_map")
    if not isinstance(tracking, Mapping):
        return False
    try:
        successes = int(tracking.get("successes", 0))
    except (TypeError, ValueError):
        return False
    return tracking.get("enabled") is True and successes > 0 and tracking.get("degraded") is False


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


__all__ = [
    "FieldBackend",
    "MapActivationToken",
    "MapRollbackFailed",
    "SessionFile",
    "SessionStage",
    "SwitchBackend",
    "motion_output_startup_evidence",
]
