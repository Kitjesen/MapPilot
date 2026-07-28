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

from lingtu.launcher import LaunchReport
from lingtu.product import (
    PROCESS_CONTRACT_SCHEMA,
    PRODUCT_SCHEMA,
    Product,
    ProductManifest,
)
from runtime.profiles.endpoints import resolve_runtime_run_spec
from runtime.profiles.native_nav_config import NativeNavProfileConfig, native_nav_profile_config
from runtime.profiles.product_mode_contracts import (
    PRODUCT_MODE_CONTRACTS,
    ProductModeContract,
    product_mode_switch_plan,
)
from runtime.profiles.resolver import canonical_profile_name, resolve_runtime_config
from runtime.runtime_switch import validate_runtime_switch

SWITCH_REPORT_SCHEMA = "lingtu.product_switch.v1"
MAP_ACTIVATION_TOKEN_SCHEMA = "lingtu.map_activation.v1"  # noqa: S105
_NAV_ENV_KEYS = frozenset(
    {
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
        "LINGTU_NAV_PROFILE",
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
_SLAM_STATUS_PATH = Path("/tmp/lingtu_slam_status.json")  # noqa: S108
_SLAM_SNAPSHOT_DIR = Path("/dev/shm/lingtu_slam")  # noqa: S108
_RUNTIME_CLEANUP_ROOTS = ("/dev/shm", "/run", "/tmp")  # noqa: S108
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

    target_profile: str
    endpoint: str = "thunder_field"
    current_profile: str | None = None
    map_name: str | None = None
    relocalize: bool = True
    initial_pose: tuple[float, float, float] | None = None


@dataclass
class SwitchReport:
    """Machine-readable result and phase journal for a Product switch."""

    current_profile: str
    target_profile: str
    endpoint: str
    dry_run: bool = False
    ok: bool = False
    status: str = "preflight"
    manifest_path: str | None = None
    fingerprint: str | None = None
    phases: list[str] = field(default_factory=list)
    cleanup: list[str] = field(default_factory=list)
    error: str | None = None
    launch: Mapping[str, Any] | None = None
    product: Mapping[str, Any] | None = None
    native_nav: Mapping[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready switch evidence."""

        return {
            "schema_version": SWITCH_REPORT_SCHEMA,
            "ok": self.ok,
            "status": self.status,
            "dry_run": self.dry_run,
            "current_profile": self.current_profile,
            "target_profile": self.target_profile,
            "endpoint": self.endpoint,
            "manifest_path": self.manifest_path,
            "fingerprint": self.fingerprint,
            "phases": list(self.phases),
            "cleanup": list(self.cleanup),
            "error": self.error,
            "launch": dict(self.launch) if self.launch is not None else None,
            "product": dict(self.product) if self.product is not None else None,
            "native_nav": dict(self.native_nav) if self.native_nav is not None else None,
        }


@dataclass(frozen=True)
class DropinSnapshot:
    """Previous state of one systemd drop-in replaced by a switch."""

    unit: str
    name: str
    content: str | None


@dataclass(frozen=True)
class RuntimeConfigStage:
    """Rollback token for exactly the drop-ins changed by one switch."""

    dropins: tuple[DropinSnapshot, ...]


@dataclass(frozen=True)
class MapArtifactIdentity:
    """Stable identity fields exposed by one native map artifact record."""

    artifact_type: str
    uri: str | None = None
    sha256: str | None = None


@dataclass(frozen=True)
class MapIdentity:
    """Versioned saved-map identity returned by the maps service."""

    map_id: str
    version: int | None = None
    version_id: str | None = None
    frame_id: str | None = None
    map_dir: str | None = None
    artifacts: tuple[MapArtifactIdentity, ...] = ()


@dataclass(frozen=True)
class MapActivationToken:
    """Compensation token for one prepared active-map pointer change."""

    target: MapIdentity
    previous: MapIdentity | None
    changed: bool
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

    def product(self, profile: str | None = None, *, endpoint: str | None = None) -> Product: ...

    def apply_manifest(
        self,
        path: str | Path,
        *,
        dry_run: bool = False,
    ) -> LaunchReport: ...

    def quiesce_product(
        self,
        product: Product | ProductManifest,
        *,
        dry_run: bool = False,
    ) -> LaunchReport: ...


class SwitchBackend(Protocol):
    """Deployment and Gateway effects required by ProductControl."""

    def current_profile(self) -> str: ...

    def resolve_map(self, map_name: str) -> tuple[str, str]: ...

    def stop_motion_and_session(self) -> None: ...

    def stage_map(self, map_name: str) -> MapActivationToken: ...

    def restore_map(self, token: MapActivationToken) -> None: ...

    def commit_map(self, token: MapActivationToken) -> None: ...

    def stage_runtime_config(
        self,
        manifest_path: Path,
        manifest: ProductManifest,
        native_environment: Mapping[str, str],
        *,
        slam_mode: str,
        map_path: str,
    ) -> RuntimeConfigStage | None: ...

    def rollback_runtime_config(self, staged: RuntimeConfigStage) -> None: ...

    def persist_boot_ownership(self, manifest: ProductManifest) -> None: ...

    def clear_runtime_status(self) -> None: ...

    def wait_native_nav(
        self,
        native_environment: Mapping[str, str],
        *,
        timeout_s: float,
    ) -> None: ...

    def wait_slam(self, mode: str, *, require_map: bool, timeout_s: float) -> None: ...

    def start_session(
        self,
        contract: ProductModeContract,
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

    def disable_boot_ownership(self, manifest: ProductManifest) -> None: ...


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

    def current_profile(self) -> str:
        """Read the active profile from the process or managed Host unit."""

        profile = _text(self._environment.get("LINGTU_PROFILE"))
        if profile:
            return canonical_profile_name(profile)
        result = self._run(
            ["systemctl", "show", "lingtu.service", "-p", "Environment", "--value"],
            check=False,
            timeout=5.0,
        )
        for item in str(result.stdout or "").split():
            if item.startswith("LINGTU_PROFILE="):
                value = _text(item.partition("=")[2])
                if value:
                    return canonical_profile_name(value)
        raise RuntimeError("active Product profile is unavailable; pass --current explicitly")

    def resolve_map(self, map_name: str) -> tuple[str, str]:
        """Resolve one map name inside the configured map root."""

        root = self._maps_root().expanduser().resolve()
        raw = Path(str(map_name or "")).expanduser()
        candidate = raw if raw.is_absolute() else root / raw
        try:
            relative = candidate.resolve(strict=False).relative_to(root)
        except (OSError, ValueError) as exc:
            raise RuntimeError(f"unsafe map name: {map_name}") from exc
        if len(relative.parts) != 1 or relative.name in {"", ".", ".."}:
            raise RuntimeError(f"unsafe map name: {map_name}")
        map_path = root / relative.name / "map.pcd"
        if not map_path.is_file():
            raise RuntimeError(f"map.pcd not found: {map_path}")
        return relative.name, str(map_path)

    def stop_motion_and_session(self) -> None:
        """Require native navigation cancellation and session termination."""

        cancel = self._http(
            "POST",
            "/api/v1/navigation/cancel",
            {"reason": "product_mode_switch"},
            timeout_s=8.0,
        )
        command = cancel.get("command") if isinstance(cancel.get("command"), Mapping) else {}
        if cancel.get("ok") is not True or command.get("accepted") is not True:
            raise RuntimeError("navigation cancel was not accepted")
        session = self._http("POST", "/api/v1/session/end", timeout_s=10.0)
        if session.get("ok") is not True or session.get("success") is not True:
            raise RuntimeError("session end was not confirmed")

    def stage_map(self, map_name: str) -> MapActivationToken:
        """Stage the selected map while the old Host is still available."""

        response = self._http(
            "POST",
            "/api/v1/map/stage-for-runtime-switch",
            {"name": map_name},
            timeout_s=20.0,
        )
        if not _response_ok(response):
            transaction = response.get("transaction")
            state = transaction.get("state") if isinstance(transaction, Mapping) else None
            detail = _text(response.get("message")) or f"could not stage navigation map: {map_name}"
            if state == "rollback_failed" or response.get("reason_code") == "map_stage_rollback_failed":
                raise _MapRollbackFailed(detail)
            raise RuntimeError(detail)
        try:
            token = _map_activation_token(response.get("activation_token"))
        except Exception as validation_error:
            raise _MapRollbackFailed(
                "map staging response did not expose a complete typed compensation token: "
                f"{validation_error}"
            ) from validation_error
        try:
            previous_active = _staged_map_previous(response, requested_map=map_name)
            if token.target.map_id != map_name:
                raise RuntimeError(
                    f"map activation token target mismatch: requested={map_name} token={token.target.map_id}"
                )
            token_previous = token.previous.map_id if token.previous is not None else None
            if previous_active != token_previous:
                raise RuntimeError("map activation token previous identity does not match staging evidence")
            return token
        except Exception as validation_error:
            try:
                self._restore_map_token(token)
            except Exception as rollback_error:
                raise _MapRollbackFailed(
                    "map staging response validation failed and active-map compensation failed: "
                    f"{validation_error}; rollback: {rollback_error}"
                ) from validation_error
            raise RuntimeError(
                f"map staging response validation failed after compensation: {validation_error}"
            ) from validation_error

    def restore_map(self, token: MapActivationToken) -> None:
        """Restore and re-verify the exact previous active-map identity."""

        response = self._restore_map_token(token)
        if token.previous is None:
            if _text(response.get("active")) is not None or response.get("restored_identity") is not None:
                raise RuntimeError("active map was not verifiably cleared")
            return
        restored = _map_identity(
            response.get("restored_identity"),
            field_name="restored map",
        )
        if restored != token.previous:
            raise RuntimeError("restored map identity did not match the prepared previous identity")

    def commit_map(self, token: MapActivationToken) -> None:
        """Verify the exact staged target before active Product commit."""

        response = self._http(
            "POST",
            "/api/v1/map/commit-staged-runtime-switch",
            {"name": token.target.map_id},
            timeout_s=10.0,
        )
        if not _response_ok(response):
            raise RuntimeError("staged map commit verification was rejected")
        transaction = response.get("transaction")
        if (
            not isinstance(transaction, Mapping)
            or transaction.get("operation") != "commit_staged_map_for_runtime_switch"
            or transaction.get("state") != "commit_verified"
            or transaction.get("target_map") != token.target.map_id
            or transaction.get("verified") is not True
        ):
            raise RuntimeError("staged map commit response was not verified")
        active = _map_identity(
            response.get("active_identity"),
            field_name="committed target map",
        )
        if active != token.target:
            raise RuntimeError("staged map target identity changed before Product commit")

    def _restore_map_token(self, token: MapActivationToken) -> Mapping[str, Any]:
        wire_token = _map_activation_token_payload(token)
        if _map_activation_token(wire_token) != token:
            raise RuntimeError("map activation token could not be serialized exactly")
        target_map = token.target.map_id
        previous_active = token.previous.map_id if token.previous is not None else None
        response = self._http(
            "POST",
            "/api/v1/map/restore-staged-runtime-switch",
            {"activation_token": wire_token},
            timeout_s=20.0,
        )
        if not _response_ok(response):
            raise RuntimeError("active-map restore was rejected")
        transaction = response.get("transaction")
        if (
            not isinstance(transaction, Mapping)
            or transaction.get("operation") != "restore_staged_map_for_runtime_switch"
            or transaction.get("state") != "rolled_back"
            or transaction.get("target_map") != target_map
            or _text(transaction.get("previous_active")) != previous_active
            or transaction.get("verified") is not True
        ):
            raise RuntimeError("active-map restore response was not verified")
        return response

    def stage_runtime_config(
        self,
        manifest_path: Path,
        manifest: ProductManifest,
        native_environment: Mapping[str, str],
        *,
        slam_mode: str,
        map_path: str,
    ) -> RuntimeConfigStage:
        """Install drop-ins and return a token that can restore their prior state."""

        snapshots: list[DropinSnapshot] = []
        try:
            self._stage_runtime_config(
                manifest_path,
                manifest,
                native_environment,
                slam_mode=slam_mode,
                map_path=map_path,
                snapshots=snapshots,
            )
        except Exception as exc:
            if snapshots:
                try:
                    self.rollback_runtime_config(RuntimeConfigStage(dropins=tuple(snapshots)))
                except Exception as rollback_error:
                    staging_detail = str(exc) or exc.__class__.__name__
                    rollback_detail = str(rollback_error) or rollback_error.__class__.__name__
                    raise RuntimeError(
                        f"runtime config staging failed: {staging_detail}; rollback also failed: {rollback_detail}"
                    ) from exc
            raise
        return RuntimeConfigStage(dropins=tuple(snapshots))

    def _stage_runtime_config(
        self,
        manifest_path: Path,
        manifest: ProductManifest,
        native_environment: Mapping[str, str],
        *,
        slam_mode: str,
        map_path: str,
        snapshots: list[DropinSnapshot],
    ) -> None:
        """Install Host, navigation, and SLAM drop-ins into one rollback set."""

        host_process = manifest.process("host")
        host_target = _unit_name(host_process.target)
        nav_target = _unit_name(manifest.process("nav").target)
        if manifest.schema_version == PROCESS_CONTRACT_SCHEMA:
            if host_process.application != "map_control_plane":
                raise RuntimeError("v5 process-only Product host application must be map_control_plane")
            host_config = host_process.config
            endpoint_transport = _required_text(
                host_config.get("_endpoint_transport"),
                "map_control_plane host _endpoint_transport",
            )
            endpoint_contract = _required_text(
                host_config.get("_endpoint_contract"),
                "map_control_plane host _endpoint_contract",
            )
        elif manifest.schema_version == PRODUCT_SCHEMA:
            host_config = manifest.host_config
            endpoint_transport = _text(host_config.get("_endpoint_transport")) or "dds"
            endpoint_contract = _text(host_config.get("_endpoint_contract")) or ""
        else:
            raise RuntimeError(f"unsupported Product manifest schema: {manifest.schema_version!r}")
        command_output_mode = _required_text(
            host_config.get("command_output_mode"),
            "Product host command_output_mode",
        )
        hardware_control_boundary = _required_text(
            host_config.get("hardware_control_boundary"),
            "Product host hardware_control_boundary",
        )
        host_environment = {
            "LINGTU_PROFILE": manifest.profile,
            "LINGTU_PRODUCT_PROFILE": manifest.profile,
            "LINGTU_ENDPOINT": manifest.endpoint or "",
            "LINGTU_PRODUCT_MANIFEST": str(manifest_path),
            "LINGTU_PRODUCT_FINGERPRINT": manifest.fingerprint,
            "LINGTU_ENDPOINT_TRANSPORT": endpoint_transport,
            "LINGTU_ENDPOINT_CONTRACT": endpoint_contract,
            "LINGTU_COMMAND_OUTPUT_MODE": command_output_mode,
            "LINGTU_HARDWARE_CONTROL_BOUNDARY": hardware_control_boundary,
        }
        self._stage_dropin(
            snapshots,
            host_target,
            "product-mode.conf",
            _dropin(host_environment),
        )

        native = {str(key): str(value) for key, value in native_environment.items()}
        unexpected = sorted(set(native) - _NAV_ENV_KEYS)
        if unexpected:
            raise RuntimeError(f"invalid native navigation environment keys: {unexpected}")
        if set(native) != _NAV_ENV_KEYS:
            missing = sorted(_NAV_ENV_KEYS - set(native))
            raise RuntimeError(f"missing native navigation environment keys: {missing}")
        self._stage_dropin(
            snapshots,
            nav_target,
            "product-mode.conf",
            _dropin(native),
        )

        slam_target = _process_target(manifest.available_processes, "slam")
        if slam_mode in {"mapping", "localization"}:
            if not manifest.has_process("slam") or not slam_target:
                raise RuntimeError(f"Product {manifest.profile} has no SLAM process for {slam_mode}")
            self._stage_dropin(
                snapshots,
                _unit_name(slam_target),
                "runtime-mode.conf",
                _dropin(
                    {
                        "LINGTU_SLAM_MODE": slam_mode,
                        "LINGTU_SLAM_MAP": map_path,
                    }
                ),
            )
        elif slam_mode == "none":
            if slam_target:
                self._stage_dropin_removal(
                    snapshots,
                    _unit_name(slam_target),
                    "runtime-mode.conf",
                )
        else:
            raise RuntimeError(f"unsupported SLAM mode: {slam_mode}")
        self._sudo(["systemctl", "daemon-reload"], timeout=15.0)

    def rollback_runtime_config(self, staged: RuntimeConfigStage) -> None:
        """Restore only drop-ins captured by one runtime staging transaction."""

        errors: list[str] = []
        for snapshot in reversed(staged.dropins):
            try:
                if snapshot.content is None:
                    self._remove_dropin(snapshot.unit, snapshot.name)
                else:
                    self._install_dropin(
                        snapshot.unit,
                        snapshot.name,
                        snapshot.content,
                    )
            except Exception as exc:
                errors.append(f"{snapshot.unit}/{snapshot.name}: {exc}")
        try:
            self._sudo(["systemctl", "daemon-reload"], timeout=15.0)
        except Exception as exc:
            errors.append(f"daemon-reload: {exc}")
        if errors:
            raise RuntimeError("failed to restore staged runtime config: " + "; ".join(errors))

    def persist_boot_ownership(self, manifest: ProductManifest) -> None:
        """Enable only Product-selected units for the next boot."""

        selected = {process.target for process in manifest.processes}
        known = {process.target for process in manifest.available_processes}
        targets = tuple(dict.fromkeys((*known, *manifest.stop_targets)))
        for target in targets:
            unit = _unit_name(target)
            available = self._unit_available(unit)
            if target in selected and not available:
                raise RuntimeError(f"required Product service is not installed: {unit}")
            if not available:
                continue
            action = "enable" if target in selected else "disable"
            self._sudo(["systemctl", action, unit], timeout=20.0)

    def clear_runtime_status(self) -> None:
        """Remove stale navigation and SLAM evidence before starting a Product."""

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
        snapshot_dir = _runtime_cleanup_path(
            self._environment.get("LINGTU_SLAM_CLOUD_SNAPSHOT_DIR") or _SLAM_SNAPSHOT_DIR,
            allowed_roots=_RUNTIME_CLEANUP_ROOTS,
        )
        self._sudo(
            ["rm", "-f", "--", nav_status, slam_status, f"{slam_status}.tmp"],
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
            native_profile = last.get("native_profile") if isinstance(last.get("native_profile"), Mapping) else {}
            checks = (
                -0.05 <= age_s <= 1.0,
                _text(last.get("control_mode")) == native_environment.get("LINGTU_NAV_CONTROL_MODE"),
                _text(native_profile.get("config_fingerprint"))
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

    def start_session(
        self,
        contract: ProductModeContract,
        *,
        map_name: str,
        relocalize: bool,
        initial_pose: tuple[float, float, float] | None,
    ) -> None:
        """Start the target session and complete localization handover."""

        if contract.session_mode == "none":
            return
        payload: dict[str, Any] = {
            "profile": contract.profile,
            "mode": contract.session_mode,
            "product_session": contract.product_session,
        }
        if map_name:
            payload["map_name"] = map_name
        response = self._http("POST", "/api/v1/session/start", payload, timeout_s=30.0)
        if not _response_ok(response):
            active = self._http("GET", "/api/v1/session", timeout_s=3.0)
            active_mode = _text(active.get("mode"))
            active_map = _text(active.get("active_map") or active.get("map_name"))
            if active_mode != contract.session_mode or (map_name and active_map != map_name):
                raise RuntimeError(f"Product session did not start: {response.get('message') or response}")
        if contract.slam_mode != "localization":
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

    def disable_boot_ownership(self, manifest: ProductManifest) -> None:
        """Prevent an incomplete Product from being revived on reboot."""

        targets = tuple(dict.fromkeys(process.target for process in manifest.available_processes)) + tuple(
            target
            for target in manifest.stop_targets
            if target not in {process.target for process in manifest.available_processes}
        )
        errors: list[str] = []
        for target in targets:
            unit = _unit_name(target)
            if not self._unit_available(unit):
                continue
            try:
                self._sudo(["systemctl", "disable", unit], timeout=20.0)
            except Exception as exc:
                errors.append(f"{unit}: {exc}")
        if errors:
            raise RuntimeError("failed to disable Product boot ownership: " + "; ".join(errors))

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
    ) -> Mapping[str, Any]:
        data = None
        headers: dict[str, str] = {}
        if payload is not None:
            data = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(  # noqa: S310 - Base URL is validated in __init__.
            f"{self._gateway_url}{path}",
            data=data,
            headers=headers,
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

    def _stage_dropin(
        self,
        snapshots: list[DropinSnapshot],
        unit: str,
        name: str,
        content: str,
    ) -> None:
        snapshots.append(
            DropinSnapshot(
                unit=unit,
                name=name,
                content=self._read_dropin(unit, name),
            )
        )
        self._install_dropin(unit, name, content)

    def _stage_dropin_removal(
        self,
        snapshots: list[DropinSnapshot],
        unit: str,
        name: str,
    ) -> None:
        snapshots.append(
            DropinSnapshot(
                unit=unit,
                name=name,
                content=self._read_dropin(unit, name),
            )
        )
        self._remove_dropin(unit, name)

    def _read_dropin(self, unit: str, name: str) -> str | None:
        target = _dropin_path(unit, name)
        exists = self._run(
            ["sudo", "-n", "test", "-e", target],
            check=False,
            timeout=5.0,
        )
        if exists.returncode == 1:
            return None
        if exists.returncode != 0:
            detail = _text(exists.stderr) or _text(exists.stdout) or str(exists.returncode)
            raise RuntimeError(f"could not inspect systemd drop-in {target}: {detail}")
        content = self._run(
            ["sudo", "-n", "cat", "--", target],
            timeout=5.0,
        )
        return str(content.stdout or "")

    def _install_dropin(self, unit: str, name: str, content: str) -> None:
        handle, raw_path = tempfile.mkstemp(prefix="lingtu-dropin-", suffix=".conf")
        temp_path = Path(raw_path)
        try:
            with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(content)
            target = _dropin_path(unit, name)
            self._sudo(
                ["install", "-D", "-m", "0644", str(temp_path), target],
                timeout=10.0,
            )
        finally:
            temp_path.unlink(missing_ok=True)

    def _remove_dropin(self, unit: str, name: str) -> None:
        self._sudo(
            ["rm", "-f", _dropin_path(unit, name)],
            timeout=5.0,
        )

    def _unit_available(self, unit: str) -> bool:
        result = self._run(
            ["systemctl", "show", unit, "-p", "LoadState", "--value"],
            check=False,
            timeout=5.0,
        )
        return result.returncode == 0 and _text(result.stdout) not in {None, "not-found"}

    def _sudo(self, command: list[str], *, timeout: float) -> Any:
        return self._run(["sudo", "-n", *command], timeout=timeout)

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
) -> SwitchReport:
    """Compile once, apply one cold-switch transaction, and fail closed."""

    env = environment if environment is not None else os.environ
    backend = backend or FieldBackend(environment=env)
    target = canonical_profile_name(_required_text(request.target_profile, "target profile"))
    endpoint = _required_text(request.endpoint, "endpoint")
    current = canonical_profile_name(request.current_profile or backend.current_profile())
    report = SwitchReport(
        current_profile=current,
        target_profile=target,
        endpoint=endpoint,
        dry_run=dry_run,
    )
    manifest: ProductManifest | None = None
    map_activation: MapActivationToken | None = None
    runtime_config_stage: RuntimeConfigStage | None = None
    manifest_created = False
    manifest_committed = False
    map_activation_committed = False
    mutated = False
    try:
        contract = _contract(target)
        product = control.product(target, endpoint=endpoint)
        if product.process_control != "launcher":
            raise RuntimeError(f"Product {target} is controlled by {product.process_control}, not Launcher")
        if not product.has_process("nav"):
            raise RuntimeError(f"Product {target} has no native navigation process")
        _validate_runtime_boundary(current, endpoint=endpoint)
        _validate_compiled_boundary(product)
        native = _native_config(product)
        plan = product_mode_switch_plan(
            current,
            target,
            product=product.as_dict(),
            native_nav_config=native.as_dict(),
        )
        lifecycle = str(plan.get("required_lifecycle") or "")
        if lifecycle != "cold_restart":
            raise RuntimeError(f"unsupported Product switch lifecycle: {lifecycle or 'missing'}")
        initial_pose = _initial_pose(request.initial_pose)
        map_name, map_path = _map_contract(backend, contract, request.map_name)
        manifest = product.manifest()
        report.product = manifest.as_dict()
        report.native_nav = native.as_dict()
        report.fingerprint = manifest.fingerprint
        report.phases.append("preflight")
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        manifest_path = _manifest_path(state_dir, env, manifest.fingerprint)
        if manifest_path.exists():
            existing_manifest = ProductManifest.load(manifest_path)
            if existing_manifest.fingerprint != manifest.fingerprint:
                raise RuntimeError("existing Product manifest does not match the compiled fingerprint")
        else:
            manifest.write(manifest_path)
            manifest_created = True
        report.manifest_path = str(manifest_path)
        report.phases.append("manifest_published")

        mutated = True
        backend.stop_motion_and_session()
        report.phases.append("motion_stopped")
        if map_name:
            map_activation = backend.stage_map(map_name)
            report.phases.append("map_prepared")
        runtime_config_stage = backend.stage_runtime_config(
            manifest_path,
            manifest,
            native.environment,
            slam_mode=contract.slam_mode,
            map_path=map_path,
        )
        report.phases.append("runtime_config_staged")
        backend.persist_boot_ownership(manifest)
        report.phases.append("boot_ownership_staged")
        backend.clear_runtime_status()
        report.phases.append("stale_status_cleared")

        launch = control.apply_manifest(manifest_path)
        report.launch = launch.as_dict()
        report.phases.append("processes_active")
        backend.wait_native_nav(native.environment, timeout_s=10.0)
        report.phases.append("native_nav_ready")
        if contract.slam_mode != "none" and manifest.has_process("slam"):
            backend.wait_slam(
                contract.slam_mode,
                require_map=contract.slam_mode == "localization",
                timeout_s=35.0,
            )
            report.phases.append("slam_ready")
        backend.start_session(
            contract,
            map_name=map_name,
            relocalize=bool(request.relocalize),
            initial_pose=initial_pose,
        )
        report.phases.append("session_active")
        if contract.session_mode == "navigating":
            backend.wait_navigation(
                map_name=map_name,
                control_mode=contract.native_control_mode,
                timeout_s=45.0,
            )
            report.phases.append("navigation_ready")
        if "inspection_evidence_capture_and_result_ack" in manifest.required_capabilities:
            backend.wait_inspection(timeout_s=45.0)
            report.phases.append("inspection_ready")
        if map_activation is not None:
            backend.commit_map(map_activation)
        _commit_active_manifest(manifest_path, manifest, env, state_dir)
        manifest_committed = True
        map_activation_committed = True
        if map_activation is not None:
            report.phases.append("map_committed")
        report.phases.append("committed")
        report.ok = True
        report.status = "active"
        return report
    except Exception as exc:
        report.error = str(exc) or exc.__class__.__name__
        report.status = "rollback_failed" if isinstance(exc, _MapRollbackFailed) else "failed"
        if mutated and manifest is not None:
            try:
                backend.stop_motion_and_session()
                report.cleanup.append("motion_session:stopped")
            except Exception as cleanup_error:
                report.cleanup.append(f"motion_session_failed:{cleanup_error}")
            if map_activation is not None and not map_activation_committed:
                try:
                    backend.restore_map(map_activation)
                    report.cleanup.append("map:restored")
                except Exception as cleanup_error:
                    report.status = "rollback_failed"
                    report.cleanup.append(f"map_failed:{cleanup_error}")
            try:
                quiesce = control.quiesce_product(manifest)
                report.cleanup.append(f"processes:{quiesce.status}")
            except Exception as cleanup_error:
                report.cleanup.append(f"processes_failed:{cleanup_error}")
            try:
                backend.disable_boot_ownership(manifest)
                report.cleanup.append("boot_ownership:disabled")
            except Exception as cleanup_error:
                report.cleanup.append(f"boot_ownership_failed:{cleanup_error}")
            if runtime_config_stage is not None:
                try:
                    backend.rollback_runtime_config(runtime_config_stage)
                    report.cleanup.append("runtime_config:restored")
                except Exception as cleanup_error:
                    report.cleanup.append(f"runtime_config_failed:{cleanup_error}")
        if manifest_created and not manifest_committed and report.manifest_path:
            try:
                Path(report.manifest_path).unlink(missing_ok=True)
                report.cleanup.append("manifest:removed")
            except OSError as cleanup_error:
                report.cleanup.append(f"manifest_failed:{cleanup_error}")
        raise SwitchFailed(report) from exc


def _contract(profile: str) -> ProductModeContract:
    try:
        return PRODUCT_MODE_CONTRACTS[profile]
    except KeyError as exc:
        raise RuntimeError(f"Product is not operator-switchable: {profile}") from exc


def _validate_runtime_boundary(profile: str, *, endpoint: str) -> None:
    resolved = resolve_runtime_config(profile, runtime_endpoint_name=endpoint)
    spec = resolve_runtime_run_spec(resolved.profile, resolved.config)
    validation = validate_runtime_switch(spec)
    if not validation.ok:
        raise RuntimeError("current runtime boundary is invalid: " + "; ".join(validation.blockers))


def _validate_compiled_boundary(product: Product) -> None:
    spec = resolve_runtime_run_spec(product.profile, product.config)
    validation = validate_runtime_switch(spec)
    if not validation.ok:
        raise RuntimeError("target runtime boundary is invalid: " + "; ".join(validation.blockers))


def _native_config(product: Product) -> NativeNavProfileConfig:
    return native_nav_profile_config(
        product.profile,
        {
            **product.config,
            "native_control_mode": product.native_nav.get("control_mode"),
            "native_nav": product.native_nav,
        },
    )


def _map_contract(
    backend: SwitchBackend,
    contract: ProductModeContract,
    requested_map: str | None,
) -> tuple[str, str]:
    map_name = _text(requested_map) or ""
    if contract.requires_map and not map_name:
        raise RuntimeError(f"Product {contract.profile} requires a map")
    if not map_name:
        return "", ""
    return backend.resolve_map(map_name)


def _manifest_path(
    state_dir: str | Path | None,
    environment: Mapping[str, str],
    fingerprint: str,
) -> Path:
    configured = state_dir or environment.get("LINGTU_PRODUCT_STATE_DIR")
    if configured is None:
        home = Path(environment.get("HOME") or Path.home())
        configured = home / ".local" / "state" / "lingtu"
    root = Path(configured).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    return root / f"product-{fingerprint}.json"


def _commit_active_manifest(
    manifest_path: Path,
    manifest: ProductManifest,
    environment: Mapping[str, str],
    state_dir: str | Path | None,
) -> None:
    root = _manifest_path(state_dir, environment, manifest.fingerprint).parent
    active = root / "active-product.json"
    temp = active.with_name(f".{active.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    payload = {
        "schema_version": "lingtu.active_product.v1",
        "profile": manifest.profile,
        "endpoint": manifest.endpoint,
        "manifest_path": str(manifest_path),
        "fingerprint": manifest.fingerprint,
        "committed_at": time.time(),
    }
    try:
        temp.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.chmod(temp, 0o600)
        os.replace(temp, active)
    finally:
        temp.unlink(missing_ok=True)


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


def _dropin(environment: Mapping[str, str]) -> str:
    lines = ["[Service]"]
    for key, value in sorted(environment.items()):
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            raise RuntimeError(f"invalid environment key: {key}")
        raw = str(value)
        if any(character in raw for character in ("\x00", "\n", "\r")):
            raise RuntimeError(f"invalid environment value: {key}")
        escaped = raw.replace("\\", "\\\\").replace('"', '\\"')
        lines.append(f'Environment="{key}={escaped}"')
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


def _dropin_path(unit: str, name: str) -> str:
    unit_name = _unit_name(unit)
    dropin_name = _required_text(name, "systemd drop-in name")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*\.conf", dropin_name):
        raise RuntimeError(f"invalid systemd drop-in name: {dropin_name}")
    return f"/etc/systemd/system/{unit_name}.d/{dropin_name}"


def _process_target(processes: tuple[Any, ...], name: str) -> str | None:
    for process in processes:
        if process.name == name:
            return process.target
    return None


def _read_json(path: Path) -> Mapping[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, Mapping) else {}


def _staged_map_previous(
    response: Mapping[str, Any],
    *,
    requested_map: str,
) -> str | None:
    transaction = response.get("transaction")
    if not isinstance(transaction, Mapping):
        raise RuntimeError("map staging response did not include a transaction")
    if "previous_active" not in transaction:
        raise RuntimeError("map staging transaction omitted previous_active")
    if (
        transaction.get("operation") != "stage_map_for_runtime_switch"
        or transaction.get("state") != "staged"
        or transaction.get("target_map") != requested_map
        or transaction.get("runtime_consistent") is not False
        or transaction.get("restart_required") is not True
    ):
        raise RuntimeError("map staging response did not confirm the requested prepared state")
    active = _text(response.get("active") or response.get("name"))
    if active != requested_map:
        raise RuntimeError(f"active map mismatch after staging: requested={requested_map} active={active or 'none'}")
    return _text(transaction.get("previous_active"))


def _map_activation_token(value: Any) -> MapActivationToken:
    if not isinstance(value, Mapping):
        raise RuntimeError("map staging response did not include a typed activation token")
    schema_version = _text(value.get("schema_version"))
    if schema_version != MAP_ACTIVATION_TOKEN_SCHEMA:
        raise RuntimeError(f"unsupported map activation token schema: {schema_version!r}")
    changed = value.get("changed")
    if not isinstance(changed, bool):
        raise RuntimeError("map activation token changed flag must be boolean")
    target = _map_identity(value.get("target"), field_name="target map")
    previous_value = value.get("previous")
    previous = None if previous_value is None else _map_identity(previous_value, field_name="previous map")
    if changed and previous is not None and previous.map_id == target.map_id:
        raise RuntimeError("changed map activation identities must name different maps")
    if not changed and previous != target:
        raise RuntimeError("unchanged map activation identities must be exactly equal")
    return MapActivationToken(
        target=target,
        previous=previous,
        changed=changed,
        schema_version=schema_version,
    )


def _map_identity(value: Any, *, field_name: str) -> MapIdentity:
    if not isinstance(value, Mapping):
        raise RuntimeError(f"map activation token {field_name} identity is missing")
    map_id = _required_text(value.get("map_id"), f"{field_name} map_id")
    raw_version = value.get("version")
    try:
        version = int(raw_version)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{field_name} version must be a positive integer") from exc
    if version <= 0:
        raise RuntimeError(f"{field_name} version must be a positive integer")
    version_id = _required_text(value.get("version_id"), f"{field_name} version_id")
    frame_id = _required_text(value.get("frame_id"), f"{field_name} frame_id")
    raw_artifacts = value.get("artifacts")
    if not isinstance(raw_artifacts, list) or not raw_artifacts:
        raise RuntimeError(f"{field_name} artifacts must be a non-empty list")
    artifacts: list[MapArtifactIdentity] = []
    for index, artifact in enumerate(raw_artifacts):
        if not isinstance(artifact, Mapping):
            raise RuntimeError(f"{field_name} artifact {index} must be an object")
        artifacts.append(
            MapArtifactIdentity(
                artifact_type=_required_text(
                    artifact.get("type"),
                    f"{field_name} artifact {index} type",
                ),
                uri=_required_text(artifact.get("uri"), f"{field_name} artifact {index} uri"),
                sha256=_required_text(
                    artifact.get("hash") or artifact.get("sha256"),
                    f"{field_name} artifact {index} hash",
                ),
            )
        )
    return MapIdentity(
        map_id=map_id,
        version=version,
        version_id=version_id,
        frame_id=frame_id,
        map_dir=_text(value.get("map_dir")),
        artifacts=tuple(artifacts),
    )


def _map_activation_token_payload(token: MapActivationToken) -> dict[str, Any]:
    return {
        "schema_version": token.schema_version,
        "changed": token.changed,
        "target": _map_identity_payload(token.target),
        "previous": _map_identity_payload(token.previous) if token.previous is not None else None,
    }


def _map_identity_payload(identity: MapIdentity) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "map_id": identity.map_id,
        "version": identity.version,
        "version_id": identity.version_id,
        "frame_id": identity.frame_id,
        "artifacts": [
            {
                "type": artifact.artifact_type,
                "uri": artifact.uri,
                "hash": artifact.sha256,
            }
            for artifact in identity.artifacts
        ],
    }
    if identity.map_dir is not None:
        payload["map_dir"] = identity.map_dir
    return payload


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
    "MAP_ACTIVATION_TOKEN_SCHEMA",
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
]
