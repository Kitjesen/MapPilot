"""Local runtime entry points for building LingTu systems.

This module is the package-facing boundary for local execution.  It resolves a
product profile, applies robot/runtime endpoint overrides, and builds the
Blueprint system without exposing those lower-level layers to facade classes.
"""

from __future__ import annotations

import threading
import time
import weakref
from dataclasses import dataclass
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Callable, Mapping

if TYPE_CHECKING:
    from runtime.profiles.resolver import ResolvedRuntimeConfig
    from runtime.status_provider import RuntimeStatusProvider


_MAP_RUNTIME_ALIASES = (
    "SlamAdapterModule",
    "camera",
    "maps.service",
    "host.bus",
    "nav.commands",
    "nav.inspection",
    "GatewayModule",
    "MCPServerModule",
    "CameraJpegRelayModule",
)

_MAP_RUNTIME_CRITICAL_ALIASES = frozenset(
    {
        "host.bus",
        "camera",
        "nav.commands",
        "GatewayModule",
    }
)

_MAP_RUNTIME_OPTIONAL_ALIASES = frozenset(_MAP_RUNTIME_ALIASES) - _MAP_RUNTIME_CRITICAL_ALIASES
_MAP_RUNTIME_SUPERVISED_ALIASES = _MAP_RUNTIME_CRITICAL_ALIASES | {"MCPServerModule"}

_MAP_RUNTIME_START_ORDER = (
    "host.bus",
    "SlamAdapterModule",
    "camera",
    "nav.commands",
    "nav.inspection",
    "CameraJpegRelayModule",
    "maps.service",
    "MCPServerModule",
    "GatewayModule",
)

_MAP_RUNTIME_ROUTES = (
    ("camera", "color_image", "CameraJpegRelayModule", "color_image"),
    ("SlamAdapterModule", "map_cloud", "maps.service", "map_cloud"),
    ("SlamAdapterModule", "map_cloud", "GatewayModule", "map_cloud"),
    ("SlamAdapterModule", "lidar_scan", "GatewayModule", "lidar_scan"),
    ("SlamAdapterModule", "localization_status", "GatewayModule", "localization_status"),
    ("SlamAdapterModule", "localization_status", "maps.service", "localization_status"),
    ("SlamAdapterModule", "saved_map", "GatewayModule", "saved_map"),
    ("SlamAdapterModule", "localization_quality", "GatewayModule", "localization_quality"),
    ("SlamAdapterModule", "map_odom_tf", "GatewayModule", "map_odom_tf"),
    ("SlamAdapterModule", "gnss_fusion_health", "GatewayModule", "gnss_fusion_health"),
    ("SlamAdapterModule", "odometry", "MCPServerModule", "odometry"),
    ("SlamAdapterModule", "odometry", "GatewayModule", "odometry"),
    ("host.bus", "navigation_state", "GatewayModule", "navigation_state"),
    ("host.bus", "navigation_goal_status", "GatewayModule", "navigation_goal_status"),
    ("host.bus", "navigation_state", "MCPServerModule", "navigation_state"),
    ("host.bus", "navigation_goal_status", "MCPServerModule", "navigation_goal_status"),
    ("maps.service", "map_event", "GatewayModule", "map_event"),
    ("host.bus", "global_path", "GatewayModule", "global_path"),
    ("host.bus", "local_path", "GatewayModule", "local_path"),
    ("host.bus", "map_scene", "GatewayModule", "map_scene"),
)


@dataclass(frozen=True)
class RuntimeApplicationState:
    """Immutable lifecycle state for process-only Host monitoring."""

    phase: str
    fingerprint: str
    failures: tuple[str, ...] = ()


class _MapRuntimeStatusProvider:
    """Read-only diagnostics projection for Gateway and MCP adapters."""

    def __init__(self, application: _MapRuntimeApplication) -> None:
        self._application_ref = weakref.ref(application)

    def _application(self) -> _MapRuntimeApplication:
        application = self._application_ref()
        if application is None:
            raise RuntimeError("runtime application is no longer available")
        return application

    @property
    def startup_state(self) -> str:
        return self._application().state.phase

    @property
    def critical_modules(self) -> tuple[str, ...]:
        return tuple(
            alias
            for alias in _MAP_RUNTIME_START_ORDER
            if alias in _MAP_RUNTIME_CRITICAL_ALIASES
        )

    @property
    def failed_modules(self) -> Mapping[str, str]:
        failures: dict[str, str] = {}
        for failure in self._application().state.failures:
            alias, separator, reason = failure.partition(":")
            if not separator or alias not in _MAP_RUNTIME_ALIASES:
                alias, reason = "application", failure
            if alias in failures:
                failures[alias] = f"{failures[alias]}; {reason}"
            else:
                failures[alias] = reason
        return MappingProxyType(failures)

    @property
    def critical_failures(self) -> Mapping[str, str]:
        failures = {
            alias: reason
            for alias, reason in self.failed_modules.items()
            if alias == "application" or alias in _MAP_RUNTIME_CRITICAL_ALIASES
        }
        state = self._application().state
        if state.phase == "failed" and "application" not in failures:
            failures["application"] = "runtime application entered failed phase"
        return MappingProxyType(failures)

    @property
    def modules(self) -> Mapping[str, Any]:
        application = self._application()
        with application._lock:
            active = {
                alias: module
                for alias, module in application._modules.items()
                if alias not in application._disabled_aliases
            }
        return MappingProxyType(active)

    @property
    def connections(self) -> tuple[tuple[str, str, str, str], ...]:
        application = self._application()
        with application._lock:
            disabled = frozenset(application._disabled_aliases)
        return tuple(
            route
            for route in _MAP_RUNTIME_ROUTES
            if route[0] not in disabled and route[2] not in disabled
        )

    def health(self) -> Mapping[str, Any]:
        state = self._application().state
        components: dict[str, Any] = {}
        for alias, module in self.modules.items():
            health = getattr(module, "health", None)
            if not callable(health):
                components[alias] = {"running": bool(getattr(module, "running", False))}
                continue
            try:
                components[alias] = health()
            except Exception as exc:
                components[alias] = {"ok": False, "reason": f"health_failed:{exc}"}
        critical_failures = dict(self.critical_failures)
        return {
            "ok": state.phase == "ready" and not critical_failures,
            "phase": state.phase,
            "fingerprint": state.fingerprint,
            "failures": list(state.failures),
            "critical_modules": list(self.critical_modules),
            "failed_modules": dict(self.failed_modules),
            "critical_failures": critical_failures,
            "connections": [list(route) for route in self.connections],
            "components": components,
        }


def resolve_runtime(
    profile: str,
    *,
    runtime_endpoint: str | None = None,
    robot_preset: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> ResolvedRuntimeConfig:
    """Resolve a LingTu product profile into a runtime-ready configuration."""

    from runtime.profiles.resolver import resolve_runtime_config

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    return resolve_runtime_config(
        profile,
        runtime_endpoint_name=runtime_endpoint,
        robot_preset=robot_preset,
        overrides=merged_overrides,
        include_profile_metadata=include_profile_metadata,
    )


def build_system(
    profile: str,
    *,
    runtime_endpoint: str | None = None,
    robot_preset: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> Any:
    """Resolve *profile* and build the corresponding local Blueprint system."""

    from lingtu.assembly.profile_builder import compile_product

    resolved = resolve_runtime(
        profile,
        runtime_endpoint=runtime_endpoint,
        robot_preset=robot_preset,
        overrides=overrides,
        include_profile_metadata=include_profile_metadata,
        **inline_overrides,
    )
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    return product.build()


def build_runtime_application(
    manifest: Any,
    *,
    process: Any | None = None,
    _factory: Callable[[Mapping[str, Any]], Mapping[str, Any]] | None = None,
) -> Any:
    """Build the Host runtime application declared by a v5 map manifest."""

    if getattr(manifest, "schema_version", None) != "lingtu.product.v5":
        raise ValueError("runtime application only accepts lingtu.product.v5 manifests")
    if getattr(manifest, "profile", None) != "map" or getattr(manifest, "endpoint", None) != "thunder_field":
        raise ValueError("runtime application only accepts map/thunder_field manifests")
    if getattr(manifest, "process_control", None) != "launcher":
        raise ValueError("runtime application requires a launcher-owned manifest")
    if (
        getattr(manifest, "modules", ())
        or getattr(manifest, "critical_modules", ())
        or getattr(manifest, "host_config", {})
        or getattr(manifest, "module_transport", "")
    ):
        raise ValueError("runtime application requires a process-only manifest")
    manifest_dict = _verified_manifest_dict(manifest)
    try:
        signed_host_process = manifest.process("host")  # type: ignore[union-attr]
    except (AttributeError, KeyError) as exc:
        raise ValueError("runtime application manifest is missing host process") from exc
    if process is None:
        host_process = signed_host_process
    else:
        host_process = process
        if getattr(host_process, "name", None) != "host":
            raise ValueError("runtime application process must be the host process")
        supplied_dict = getattr(host_process, "as_dict", None)
        if not callable(supplied_dict) or supplied_dict() != signed_host_process.as_dict():
            raise ValueError("runtime application process must match the signed host process")
    if host_process.application != "map_control_plane":
        raise ValueError("runtime application host application must be map_control_plane")
    config = dict(host_process.config)
    if config.get("_endpoint_transport") != "dds":
        raise ValueError("map_control_plane host _endpoint_transport must be dds")
    if config.get("_endpoint_contract") != "thunder_field_dds_v1":
        raise ValueError("map_control_plane host _endpoint_contract must be thunder_field_dds_v1")
    for flag in ("enable_gateway", "enable_teleop", "enable_camera"):
        if config.get(flag) is not True:
            raise ValueError(f"map_control_plane host requires {flag}=true")
    if config.get("camera_backend") != "dds":
        raise ValueError("map_control_plane host requires camera_backend=dds")
    fingerprint = str(getattr(manifest, "fingerprint", "") or "").strip()
    if not fingerprint or manifest_dict.get("fingerprint") != fingerprint:
        raise ValueError("runtime application manifest is missing fingerprint")
    runtime_config = dict(config)
    runtime_config["product_fingerprint"] = fingerprint
    return _MapRuntimeApplication(
        fingerprint,
        runtime_config,
        factory=_factory or _production_map_runtime_factory,
    )


class _MapRuntimeApplication:
    def __init__(
        self,
        fingerprint: str,
        config: Mapping[str, Any],
        *,
        factory: Callable[[Mapping[str, Any]], Mapping[str, Any]],
    ) -> None:
        self._fingerprint = fingerprint
        self._config = dict(config)
        self._factory = factory
        self._status_provider: RuntimeStatusProvider = _MapRuntimeStatusProvider(self)
        self._constructed_modules: dict[str, Any] = {}
        self._modules: dict[str, Any] = {}
        self._disabled_aliases: set[str] = set()
        self._unsubscribe: list[Callable[[], None]] = []
        self._touched: list[str] = []
        self._state = RuntimeApplicationState("built", fingerprint)
        self._lock = threading.RLock()
        self._supervision_stop = threading.Event()
        self._supervision_thread: threading.Thread | None = None
        self._runtime_failure_grace_s = _positive_float(
            self._config.get("runtime_failure_grace_s"),
            default=1.0,
            label="runtime_failure_grace_s",
        )
        self._startup_timeout_s = _positive_float(
            self._config.get("startup_timeout_s"),
            default=5.0,
            label="startup_timeout_s",
        )
        self._readiness_poll_interval_s = _positive_float(
            self._config.get("readiness_poll_interval_s"),
            default=0.05,
            label="readiness_poll_interval_s",
        )
        self._stop_timeout_s = _positive_float(
            self._config.get("stop_timeout_s"),
            default=2.0,
            label="stop_timeout_s",
        )

    @property
    def state(self) -> RuntimeApplicationState:
        with self._lock:
            return self._state

    def start(self) -> None:
        with self._lock:
            phase = self._state.phase
            if phase == "ready":
                return
            if phase != "built":
                raise RuntimeError(f"runtime application cannot start from phase {phase}")
            self._set_state("starting")
        try:
            modules = self._materialize_modules()
            with self._lock:
                self._constructed_modules = dict(modules)
                self._modules = dict(modules)
            self._preflight_modules(modules)
            self._notify_modules(modules)
            for alias in _MAP_RUNTIME_START_ORDER:
                if alias in self._disabled_aliases:
                    continue
                module = modules[alias]
                self._start_one_module(alias, module)
            self._bind_routes(modules)
            self._wait_until_ready(modules)
            with self._lock:
                self._set_state("ready")
            self._start_supervision()
        except Exception as exc:
            self._record_failure(f"start_failed:{exc}")
            with self._lock:
                self._set_state("failed")
            self._unbind_routes()
            self._rollback_started()
            raise

    def stop(self) -> None:
        with self._lock:
            phase = self._state.phase
            if phase in {"built", "stopped"}:
                self._set_state("stopped")
                return
            self._set_state("stopping")
        self._supervision_stop.set()
        thread = self._supervision_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=self._stop_timeout_s)
        self._unbind_routes()
        self._rollback_started()
        with self._lock:
            self._modules = {}
            self._constructed_modules = {}
            self._set_state("stopped")

    def _materialize_modules(self) -> dict[str, Any]:
        modules = dict(self._factory(dict(self._config)))
        if tuple(modules) != _MAP_RUNTIME_ALIASES:
            raise ValueError(
                "map runtime factory must return exact aliases: "
                + ", ".join(_MAP_RUNTIME_ALIASES)
            )
        return modules

    def _preflight_modules(self, modules: Mapping[str, Any]) -> None:
        for alias in _MAP_RUNTIME_ALIASES:
            module = modules[alias]
            preflight = getattr(module, "preflight", None)
            if callable(preflight):
                try:
                    reason = preflight()
                except Exception as exc:
                    self._handle_module_start_failure(alias, module, f"preflight_failed:{exc}")
                    continue
                if reason:
                    self._handle_module_start_failure(alias, module, f"preflight:{reason}")

    def _bind_routes(self, modules: Mapping[str, Any]) -> None:
        unsubscribers: list[Callable[[], None]] = []
        try:
            for source_alias, out_name, dest_alias, in_name in _MAP_RUNTIME_ROUTES:
                if source_alias in self._disabled_aliases or dest_alias in self._disabled_aliases:
                    continue
                out_port = getattr(modules[source_alias], out_name)
                in_port = getattr(modules[dest_alias], in_name)
                unsubscribe = out_port.subscribe(in_port._deliver)
                unsubscribers.append(unsubscribe)
        except Exception:
            for unsubscribe in reversed(unsubscribers):
                unsubscribe()
            raise
        self._unsubscribe = unsubscribers

    def _notify_modules(self, modules: Mapping[str, Any]) -> None:
        snapshot = dict(modules)
        for alias in _MAP_RUNTIME_ALIASES:
            if alias in self._disabled_aliases:
                continue
            hook = getattr(modules[alias], "on_system_modules", None)
            if callable(hook):
                hook(dict(snapshot))
        for alias in ("GatewayModule", "MCPServerModule"):
            if alias in self._disabled_aliases:
                continue
            setter = getattr(modules[alias], "set_runtime_status_provider", None)
            if callable(setter):
                setter(self._status_provider)

    def _start_one_module(self, alias: str, module: Any) -> None:
        if alias not in self._touched:
            self._touched.append(alias)
        try:
            setup = getattr(module, "setup", None)
            if callable(setup):
                setup()
            start = getattr(module, "start", None)
            if callable(start):
                start()
        except Exception as exc:
            self._handle_module_start_failure(alias, module, f"start_failed:{exc}")

    def _handle_module_start_failure(self, alias: str, module: Any, failure: str) -> None:
        if alias in _MAP_RUNTIME_CRITICAL_ALIASES:
            raise RuntimeError(f"{alias}:{failure}")
        self._disabled_aliases.add(alias)
        self._record_failure(f"{alias}:{failure}")
        stop = getattr(module, "stop", None)
        if callable(stop):
            self._call_with_timeout(stop, alias)
        if alias in self._touched:
            self._touched.remove(alias)

    def _wait_until_ready(self, modules: Mapping[str, Any]) -> None:
        deadline = time.monotonic() + self._startup_timeout_s
        while True:
            failures = self._collect_failures(modules, include_optional=True)
            blocking = tuple(
                failure for failure in failures if not _is_optional_failure(failure)
            )
            if not blocking:
                if failures:
                    self._record_failure(*failures)
                return
            if time.monotonic() >= deadline:
                raise TimeoutError("; ".join(blocking))
            time.sleep(self._readiness_poll_interval_s)

    def _collect_failures(self, modules: Mapping[str, Any], *, include_optional: bool) -> tuple[str, ...]:
        failures: list[str] = []
        for alias in _MAP_RUNTIME_ALIASES:
            if alias in self._disabled_aliases:
                continue
            if alias in _MAP_RUNTIME_OPTIONAL_ALIASES and not include_optional:
                continue
            module = modules[alias]
            readiness = getattr(module, "startup_readiness", None)
            if callable(readiness):
                reason = readiness()
                if reason:
                    failures.append(f"{alias}:{reason}")
            health = getattr(module, "health", None)
            if callable(health):
                try:
                    status = health()
                except Exception as exc:
                    failures.append(f"{alias}:health_failed:{exc}")
                    continue
                if isinstance(status, Mapping):
                    mcp_failure = _mcp_health_failure(alias, status)
                    if mcp_failure:
                        failures.append(mcp_failure)
                    if alias == "camera" and status.get("ready") is False:
                        reason = status.get("error") or status.get("status") or "camera_not_ready"
                        failures.append(f"{alias}:{reason}")
                    ok = status.get("ok")
                    if ok is False:
                        reason = status.get("reason") or status.get("status") or "unhealthy"
                        failures.append(f"{alias}:{reason}")
        return tuple(dict.fromkeys(failures))

    def _start_supervision(self) -> None:
        self._supervision_stop.clear()
        self._supervision_thread = threading.Thread(
            target=self._supervise,
            name="map-runtime-supervisor",
            daemon=True,
        )
        self._supervision_thread.start()

    def _supervise(self) -> None:
        failing_since: float | None = None
        latest: tuple[str, ...] = ()
        while not self._supervision_stop.wait(self._readiness_poll_interval_s):
            with self._lock:
                if self._state.phase != "ready":
                    return
                modules = dict(self._modules)
            failures = self._collect_failures(modules, include_optional=True)
            post_ready_failures = tuple(
                failure
                for failure in failures
                if failure.partition(":")[0] in _MAP_RUNTIME_SUPERVISED_ALIASES
            )
            if not post_ready_failures:
                failing_since = None
                latest = ()
                continue
            if post_ready_failures != latest:
                failing_since = time.monotonic()
                latest = post_ready_failures
                continue
            if failing_since is not None and time.monotonic() - failing_since >= self._runtime_failure_grace_s:
                self._record_failure(*post_ready_failures)
                with self._lock:
                    self._set_state("failed")
                return

    def _rollback_started(self) -> None:
        touched = list(reversed(self._touched))
        self._touched = []
        for alias in touched:
            module = self._constructed_modules.get(alias) or self._modules.get(alias)
            stop = getattr(module, "stop", None)
            if callable(stop):
                self._call_with_timeout(stop, alias)

    def _unbind_routes(self) -> None:
        unsubscribers = list(reversed(self._unsubscribe))
        self._unsubscribe = []
        for unsubscribe in unsubscribers:
            unsubscribe()

    def _call_with_timeout(self, callback: Callable[[], None], alias: str) -> None:
        done = threading.Event()
        errors: list[BaseException] = []

        def runner() -> None:
            try:
                callback()
            except BaseException as exc:
                errors.append(exc)
            finally:
                done.set()

        thread = threading.Thread(target=runner, name=f"stop-{alias}", daemon=True)
        thread.start()
        if not done.wait(self._stop_timeout_s):
            self._record_failure(f"{alias}:stop_timeout")
        elif errors:
            self._record_failure(f"{alias}:stop_failed:{errors[0]}")

    def _record_failure(self, *failures: str) -> None:
        if not failures:
            return
        with self._lock:
            merged = tuple(dict.fromkeys((*self._state.failures, *failures)))
            self._state = RuntimeApplicationState(self._state.phase, self._fingerprint, merged)

    def _set_state(self, phase: str) -> None:
        self._state = RuntimeApplicationState(phase, self._fingerprint, self._state.failures)


def _is_optional_failure(failure: str) -> bool:
    alias = failure.split(":", 1)[0]
    return alias in _MAP_RUNTIME_OPTIONAL_ALIASES


def _mcp_health_failure(alias: str, status: Mapping[str, Any]) -> str | None:
    if alias != "MCPServerModule":
        return None
    mcp = status.get("mcp")
    if not isinstance(mcp, Mapping):
        return None
    last_error = str(mcp.get("last_error") or "").strip()
    if last_error:
        return f"{alias}:mcp_last_error:{last_error}"
    if mcp.get("server_started") is False:
        return f"{alias}:mcp_server_not_started"
    if mcp.get("thread_alive") is False:
        return f"{alias}:mcp_thread_not_alive"
    return None


def _verified_manifest_dict(manifest: Any) -> Mapping[str, Any]:
    as_dict = getattr(manifest, "as_dict", None)
    from_dict = getattr(type(manifest), "from_dict", None)
    if not callable(as_dict) or not callable(from_dict):
        raise ValueError("runtime application requires a verified ProductManifest")
    payload = as_dict()
    verified = from_dict(payload)
    if getattr(verified, "fingerprint", None) != getattr(manifest, "fingerprint", None):
        raise ValueError("runtime application manifest fingerprint mismatch")
    return payload


def _positive_float(value: Any, *, default: float, label: str) -> float:
    if value is None:
        return default
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a positive number") from exc
    if parsed <= 0:
        raise ValueError(f"{label} must be a positive number")
    return parsed


def _production_map_runtime_factory(config: Mapping[str, Any]) -> Mapping[str, Any]:
    from drivers.real.camera.dds_module import DdsCameraModule
    from drivers.real.camera_jpeg_relay import CameraJpegRelayModule
    from gateway.gateway_module import GatewayModule
    from gateway.mcp_server import MCPServerModule
    from lingtu.host_bus import HostBus
    from localization.adapters.status import CppSlamStatusAdapterModule
    from maps.modules.service import MapsModule
    from nav.commands.module import Commands
    from nav.inspection.service import Inspection

    gateway_port = int(config.get("gateway_port", 5050))
    mcp_port = int(config.get("mcp_port", 8090))
    endpoint_contract = str(config.get("_endpoint_contract") or "thunder_field_dds_v1")
    endpoint_transport = str(config.get("_endpoint_transport") or "dds")
    slam_profile = str(config.get("slam_profile") or config.get("localization_adapter") or "bridge")
    data_source = str(config.get("data_source") or config.get("_endpoint_data_source") or "thunder_field")
    source_profile = str(config.get("source_profile") or config.get("profile") or "map")
    endpoint_config = {
        "backend_profile": slam_profile,
        "slam_profile": slam_profile,
        "localization_adapter": config.get("localization_adapter"),
        "endpoint_contract": endpoint_contract,
        "endpoint_transport": endpoint_transport,
        "data_source": data_source,
        "source_profile": source_profile,
        "profile": source_profile,
    }
    maps_config = {
        key: config[key]
        for key in (
            "map_dir",
            "data_dir",
            "map_save_adapter",
            "map_save_timeout_sec",
            "map_artifact_converter_command",
            "octomap_converter_command",
            "octomap_build_mode",
            "octomap_resolution",
            "octomap_free_layers_above",
            "octomap_free_dilation_cells",
            "octomap_build_timeout_sec",
            "build_octomap_on_save",
            "map_prune_command",
            "dynamic_filter_command",
            "map_opt",
            "map_optimization",
            "map_opt_command",
            "map_optimization_command",
            "map_opt_timeout_sec",
            "map_opt_required",
            "semantic_taxonomy_path",
            "semantic_save_timeout_sec",
            "octomap_editor_command",
            "octomap_edit_timeout_sec",
        )
        if key in config
    }
    return {
        "SlamAdapterModule": CppSlamStatusAdapterModule(**endpoint_config),
        "camera": DdsCameraModule(),
        "maps.service": MapsModule(**endpoint_config, **maps_config),
        "host.bus": HostBus(require_map_scene=True),
        "nav.commands": Commands(),
        "nav.inspection": Inspection(),
        "GatewayModule": GatewayModule(
            port=gateway_port,
            command_output_mode=config.get("command_output_mode"),
            hardware_control_boundary=config.get("hardware_control_boundary"),
            manage_session_services=config.get("manage_session_services"),
            map_save_adapter=config.get("map_save_adapter"),
            product_profile="map",
            product_fingerprint=config.get("product_fingerprint"),
        ),
        "MCPServerModule": MCPServerModule(port=mcp_port),
        "CameraJpegRelayModule": CameraJpegRelayModule(
            jpeg_quality=int(config.get("camera_jpeg_quality", 60)),
            stream_fps=float(config.get("camera_fps", 10.0)),
        ),
    }
