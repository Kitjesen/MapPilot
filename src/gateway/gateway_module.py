"""Gateway Host module for REST, SSE, WebSocket, and MCP access.

Routes live under :mod:`gateway.routes`; generated API documentation is the
endpoint catalogue. Physical teleoperation uses ``/ws/teleop`` and native DDS.
"""

from __future__ import annotations

import logging
import os
import threading
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, Callable

from fastapi.responses import JSONResponse

from gateway.app_factory import build_gateway_app
from gateway.services.commands import (
    publish_command_ack,
    run_control_command,
)
from gateway.services.drift import (
    current_odom_divergence,
    odom_diverged,
    report_drift,
    watchdog_loop,
)
from gateway.services.environment_map_feedback import EnvironmentMapFeedback
from gateway.services.event_handlers import (
    handle_agent_message,
    handle_exploration_run_event,
    handle_exploration_supervisor,
    handle_gnss_fusion_health,
    handle_inspection_task_event,
    handle_navigation_goal_status,
    handle_navigation_state,
    handle_scene_graph,
    handle_tare_stats,
    handle_visual_servo_status,
    json_payload,
)
from gateway.services.exploration import (
    begin_exploration,
    coerce_explorer_result,
    end_exploration,
    exploration_start_readiness,
    exploration_status_payload,
    explorer_available,
    explorer_backend,
    explorer_stop_available,
    explorer_unavailable_detail,
    tare_status_payload,
)
from gateway.services.init_state import (
    init_cloud_and_frame_state,
    init_core_state,
    init_drift_watchdog_state,
    init_exploration_state,
    init_module_refs,
    init_recording_state,
    init_runtime_status_state,
    init_server_state,
)
from gateway.services.inspection_task_lifecycle import ensure_inspection_task_timeline
from gateway.services.lifecycle import start_background_threads, stop_background_threads
from gateway.services.localization_status import handle_localization_status
from gateway.services.mapd_transport import (
    active_map as maps_active_map,
)
from gateway.services.mapd_transport import (
    map_bundle as maps_map_bundle,
)
from gateway.services.module_refs import attach_module_refs
from gateway.services.native_control import teleop_active as native_teleop_active
from gateway.services.odometry import handle_odometry
from gateway.services.pose_recovery import (
    clear_localization_runtime_cache as reset_localization_runtime_cache,
)
from gateway.services.pose_recovery import (
    handle_map_odom_tf,
)
from gateway.services.server import run_server
from gateway.services.session_view import (
    detect_current_mode,
    reconcile_native_explore,
    refresh_session_projection,
    session_snapshot,
)
from gateway.services.slam_profile import (
    cached_slam_hz,
    current_slam_profile,
    slam_profile_from_status,
)
from gateway.services.sse import (
    push_event as sse_push_event,
)
from gateway.services.subscriptions import setup_subscriptions
from gateway.services.teleop import (
    claim as teleop_claim,
)
from gateway.services.teleop import (
    init_teleop_state,
    shutdown_teleop,
)
from gateway.services.teleop import (
    on_velocity as teleop_on_velocity,
)
from gateway.services.teleop import (
    on_velocity_with_request_id as teleop_on_velocity_with_request_id,
)
from gateway.services.teleop import (
    publish_remote_velocity_request as teleop_publish_remote_velocity_request,
)
from gateway.services.teleop import (
    release as teleop_release,
)
from gateway.services.teleop import (
    twist_from_velocity as teleop_twist_from_velocity,
)
from gateway.services.traffic import (
    DEFAULT_SSE_RASTER_MIN_INTERVAL_S,
)
from gateway.services.traffic import (
    snapshot as traffic_snapshot,
)
from gateway.services.viewer_events import (
    handle_global_path,
    handle_local_path,
    handle_native_traversability,
    xyz_point,
)
from runtime.module import Module

if TYPE_CHECKING:
    from runtime.status_provider import RuntimeStatusProvider
from runtime.msgs.geometry import Twist
from runtime.msgs.map import MapSceneFrame
from runtime.msgs.nav import (
    ExplorationRunEvent,
    InspectionTaskEvent,
    NavigationGoalStatus,
    NavigationState,
    Odometry,
    OperatorMotionReceipt,
    Path,
)
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


def _compiled_setting(value: Any, *environment_names: str) -> str:
    """Prefer an explicitly compiled runtime value over process environment."""

    if value is not None:
        return str(value).strip()
    for name in environment_names:
        raw = os.environ.get(name)
        if raw is not None:
            return raw.strip()
    return ""


def _run_plan_setting(
    plan_value: Any,
    explicit_value: Any,
    *,
    field: str,
) -> str:
    """Return one RunPlan-owned value and reject conflicting fragments."""

    plan_text = str(plan_value or "").strip()
    explicit_text = str(explicit_value or "").strip()
    if explicit_text and plan_text and explicit_text != plan_text:
        raise ValueError(
            f"Gateway {field} conflicts with RunPlan: "
            f"explicit={explicit_text!r} plan={plan_text!r}"
        )
    return plan_text or explicit_text


# ---------------------------------------------------------------------------
# GatewayModule
# ---------------------------------------------------------------------------


@register("gateway", "fastapi", description="FastAPI gateway: REST+SSE+WebSocket teleop")
class GatewayModule(Module, layer=6):
    """HTTP/WebSocket gateway with typed APIs and thread-safe telemetry.

    In:  odometry, scene_graph, navigation_state
    Out: instruction, servo_target, mode_cmd
    """

    _RUNTIME_CACHE_ATTRS = frozenset(
        {
            "_odom",
            "_last_invalid_odometry",
            "_odom_timestamps",
            "_T_map_odom",
            "_has_map_odom_tf",
        }
    )
    def __getattr__(self, name: str) -> Any:
        if name in self._RUNTIME_CACHE_ATTRS:
            runtime_cache = self.__dict__.get("_runtime_cache")
            if runtime_cache is not None:
                return getattr(runtime_cache, name)
        raise AttributeError(f"{type(self).__name__!s} object has no attribute {name!r}")

    def __setattr__(self, name: str, value: Any) -> None:
        if name in type(self)._RUNTIME_CACHE_ATTRS and "_runtime_cache" in self.__dict__:
            setattr(self.__dict__["_runtime_cache"], name, value)
            return
        super().__setattr__(name, value)

    # -- Inputs (module ->cache) --------------------------------------------
    odometry: In[Odometry]
    lidar_scan: In[PointCloud2]  # current raw LiDAR scan for /ws/scan
    map_scene: In[MapSceneFrame]  # canonical layered map product
    localization_quality: In[float]  # ICP fitness from SlamBridge -lower=better
    # Canonical T_map_from_odom; converts odom coordinates into map coordinates.
    map_odom_tf: In[dict]
    scene_graph: In[SceneGraph]
    visual_servo_status: In[dict]
    navigation_state: In[NavigationState]
    navigation_goal_status: In[NavigationGoalStatus]
    exploration_run_event: In[ExplorationRunEvent]
    inspection_task_event: In[InspectionTaskEvent]
    global_path: In[Path]  # from Navigation
    local_path: In[Path]  # from LocalPlanner -obstacle-free local path
    native_traversability: In[dict[str, Any]]  # native control-risk grid via HostBus
    agent_message: In[dict]  # from SemanticPlanner -chat-facing messages
    gnss_fusion_health: In[dict]  # from SLAM GNSS/alignment diagnostics
    localization_status: In[dict]  # from SLAM health (cov_trace, iter_num, ...)
    tare_stats: In[dict]  # from TAREExplorerModule -exploration diag
    supervisor_state: In[dict]  # from ExplorationSupervisorModule -watchdog

    # -- Outputs (client commands ->modules) --------------------------------
    instruction: Out[str]
    servo_target: Out[str]
    mode_cmd: Out[str]

    def __init__(
        self,
        port: int = 5050,
        host: str = "0.0.0.0",
        command_output_mode: str | None = None,
        hardware_control_boundary: str | None = None,
        product: str | None = None,
        run_plan: Any | None = None,
        product_session_id: str | None = None,
        frame_tree: Any | None = None,
        **kw,
    ):
        if kw:
            raise TypeError(
                "unsupported GatewayModule configuration field(s): "
                + ", ".join(sorted(kw))
            )
        super().__init__()
        self._port = port
        self._host = host
        self._compiled_run_plan = run_plan
        if run_plan is not None:
            plan_host_config = getattr(run_plan, "host_config", None)
            if not isinstance(plan_host_config, Mapping):
                raise ValueError("Gateway RunPlan is missing host_config")
            self._compiled_command_output_mode = _run_plan_setting(
                plan_host_config.get("command_output_mode"),
                command_output_mode,
                field="command_output_mode",
            )
            self._compiled_hardware_control_boundary = _run_plan_setting(
                plan_host_config.get("hardware_control_boundary"),
                hardware_control_boundary,
                field="hardware_control_boundary",
            )
            self._compiled_product = _run_plan_setting(
                getattr(run_plan, "product", None),
                product,
                field="product",
            )
            if not self._compiled_product:
                raise ValueError("Gateway RunPlan is missing identity")
            self._compiled_env = str(getattr(run_plan, "env", "") or "").strip()
            self._host_config = dict(plan_host_config)
        else:
            self._compiled_command_output_mode = _compiled_setting(
                command_output_mode,
                "LINGTU_COMMAND_OUTPUT_MODE",
            )
            self._compiled_hardware_control_boundary = _compiled_setting(
                hardware_control_boundary,
                "LINGTU_HARDWARE_CONTROL_BOUNDARY",
            )
            self._compiled_product = _compiled_setting(
                product,
                "LINGTU_PRODUCT",
            )
            self._compiled_env = _compiled_setting(None, "LINGTU_ENV") or "real"
            self._host_config = {}
        if self._compiled_env not in {"real", "sim"}:
            raise ValueError(
                f"Env must be 'real' or 'sim', received {self._compiled_env!r}"
            )
        self._compiled_product_session_id = _compiled_setting(
            product_session_id,
            "LINGTU_PRODUCT_SESSION_ID",
        )
        init_core_state(
            self,
            sse_raster_min_interval_s=max(
                0.0,
                _env_float(
                    "LINGTU_SSE_RASTER_MIN_INTERVAL_S",
                    DEFAULT_SSE_RASTER_MIN_INTERVAL_S,
                ),
            ),
        )
        init_teleop_state(
            self,
            max_speed=float(
                self._host_config.get(
                    "teleop_max_speed_mps",
                    _env_float("LINGTU_TELEOP_MAX_SPEED_MPS", 0.5),
                )
            ),
            max_yaw=float(
                self._host_config.get(
                    "teleop_max_yaw_rate_rad_s",
                    _env_float("LINGTU_TELEOP_MAX_YAW_RATE", 1.0),
                )
            ),
            release_timeout=_env_float("LINGTU_TELEOP_RELEASE_TIMEOUT_S", 0.5),
        )

        init_module_refs(self)
        refresh_session_projection(self)
        init_recording_state(self)
        init_cloud_and_frame_state(self, frame_tree=frame_tree)
        self._environment_map_feedback = EnvironmentMapFeedback()

        self._go2rtc_upstream: str = os.environ.get(
            "LINGTU_GO2RTC_URL",
            "http://127.0.0.1:1984",
        )

        init_runtime_status_state(
            self,
            brainstem_health_cache_ttl_s=max(
                0.0,
                _env_float("LINGTU_BRAINSTEM_HEALTH_CACHE_S", 2.0),
            ),
        )

        init_drift_watchdog_state(
            self,
            enabled=os.environ.get("LINGTU_DRIFT_WATCHDOG", "1") not in ("0", "false", "False"),
            interval_s=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_INTERVAL", "5")),
            xy_limit=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_XY_LIMIT", "50")),
            v_limit=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_V_LIMIT", "10")),
            cooldown_s=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_COOLDOWN", "300")),
        )

        # Crash-time black box. Records the gateway's view of the world (odom,
        # slam_diag, gnss_fusion, map_odom_tf) and dumps to disk before the
        # watchdog requests the Product-owned SLAM restart, so we can attribute
        # divergences offline.
        from runtime.utils.blackbox_recorder import BlackBoxRecorder

        self._blackbox = BlackBoxRecorder.from_env()

        init_exploration_state(self)
        init_server_state(self)

    # -- lifecycle ----------------------------------------------------------

    def set_system_handle(self, handle: Any) -> None:
        """Bind the assembled system so readiness can report critical failures."""

        self._system_handle = handle

    def set_runtime_status_provider(self, provider: RuntimeStatusProvider) -> None:
        """Bind read-only runtime status without giving Gateway orchestration control."""

        self._runtime_status_provider = provider

    def startup_readiness(self) -> str | None:
        """Require the externally visible HTTP server to be accepting traffic."""

        base_reason = super().startup_readiness()
        if base_reason is not None:
            return base_reason
        if self._defer_server:
            return None
        if self._server_error:
            return f"server_error:{self._server_error}"
        thread = self._server_thread
        if thread is None or not thread.is_alive():
            return "server_thread_not_running"
        server = self._server
        if server is None:
            return "server_not_created"
        if getattr(server, "started", False) is not True:
            return "server_not_started"
        return None

    def setup(self) -> None:
        refresh_session_projection(self)
        reconcile_native_explore(self)
        setup_subscriptions(self)

    def start(self) -> None:
        super().start()
        if self._stop_event.is_set():
            self._stop_event = threading.Event()
        start_background_threads(self)
        logger.info("GatewayModule started on %s:%d", self._host, self._port)

    def _drift_odom_diverged(
        self,
        odom: dict[str, Any],
    ) -> tuple[bool, float, float, float, float, bool]:
        return odom_diverged(self, odom)

    def _drift_current_odom_divergence(
        self,
    ) -> tuple[bool, float, float, float, float, bool]:
        return current_odom_divergence(self)

    def _drift_watchdog_loop(self, stop_event: threading.Event | None = None) -> None:
        watchdog_loop(self, stop_event=stop_event)

    def _drift_report_divergence(
        self,
        *,
        xy: float,
        y_abs: float,
        v: float,
        stop_event: threading.Event | None = None,
    ) -> bool:
        """Report SLAM divergence without taking ProductControl ownership."""
        return report_drift(self, xy=xy, y_abs=y_abs, v=v, stop_event=stop_event)

    def stop(self) -> None:
        stop_background_threads(self)
        shutdown_teleop(self)
        super().stop()

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        attach_module_refs(self, modules)

    def _active_map_from_mapd(self) -> str | None:
        return maps_active_map(self)

    def _map_bundle_from_mapd(
        self,
        map_name: str,
        capability: str,
    ) -> dict[str, Any] | None:
        return maps_map_bundle(self, map_name, capability)

    def _explorer_backend(self) -> str:
        return explorer_backend(self)

    def _explorer_available(self) -> bool:
        return explorer_available(self)

    def _explorer_stop_available(self) -> bool:
        return explorer_stop_available(self)

    def _explorer_unavailable_detail(self) -> dict[str, Any]:
        return explorer_unavailable_detail()

    def _coerce_explorer_result(self, result: Any) -> Any:
        return coerce_explorer_result(result)

    def _begin_exploration(self) -> Any:
        return begin_exploration(self)

    def _end_exploration(self) -> Any:
        return end_exploration(self)

    def _tare_status_payload(self) -> dict[str, Any]:
        return tare_status_payload(self)

    def _exploration_status_payload(self) -> dict[str, Any]:
        return exploration_status_payload(self)

    def _exploration_start_readiness(self) -> dict[str, Any]:
        return exploration_start_readiness(self)

    # -- native teleop helpers -----------------------------------------------

    @property
    def _teleop_active(self) -> bool:
        return native_teleop_active()

    def publish_remote_velocity_request(
        self,
        twist: Twist,
        *,
        request_id: str | None = None,
        source_id: str | None = None,
        source_epoch: int | None = None,
        sequence: int | None = None,
    ) -> bool:
        return teleop_publish_remote_velocity_request(
            self,
            twist,
            request_id=request_id,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
        )

    def _teleop_twist_from_velocity(
        self,
        vx_mps: float,
        vy_mps: float,
        yaw_rps: float,
    ) -> Twist:
        return teleop_twist_from_velocity(self, vx_mps, vy_mps, yaw_rps)

    def push_jpeg(self, jpeg_bytes: bytes) -> None:
        """Cache a frame produced by CameraJpegRelayModule."""
        with self._jpeg_lock:
            self._latest_jpeg = jpeg_bytes
            self._latest_jpeg_seq += 1

    # -- Module subscription callbacks -------------------------------------

    def _on_odometry(self, odom: Odometry) -> None:
        handle_odometry(self, odom)

    def _on_icp_quality(self, q: float) -> None:
        try:
            self._icp_quality = float(q)
        except Exception as e:
            logger.debug("_on_icp_quality: failed to convert %r: %s", q, e)

    def _on_map_odom_tf(self, tf: dict) -> None:
        handle_map_odom_tf(self, tf)

    # Session state helpers

    def _session_detect_current_mode(self) -> tuple[str, str | None]:
        return detect_current_mode(self)

    def _session_uses_external_slam_none(self) -> bool:
        """True when an active session intentionally uses external sim odometry."""
        profile = str(self._session_slam_profile or "").strip().lower()
        return (
            profile == "none"
            and self._session_mode in ("mapping", "navigating", "exploring")
            and self._explorer_available()
        )

    def _session_active_map_name(self) -> str | None:
        """Read the active map only through native mapd."""
        return self._active_map_from_mapd()

    def _session_snapshot(self) -> dict:
        snapshot = session_snapshot(self)
        if not snapshot.get("product") and self._compiled_product:
            snapshot["product"] = self._compiled_product
        return snapshot

    def _on_lidar_scan(self, cloud: PointCloud2) -> None:
        self._cloud_viewer.on_lidar_scan(cloud)

    def _on_map_scene(self, frame: MapSceneFrame | dict[str, Any]) -> None:
        self._environment_map_feedback.observe_scene(frame)
        self._cloud_viewer.on_map_scene(frame)

    @staticmethod
    def _slam_profile_from_status(status: dict | None) -> str:
        return slam_profile_from_status(status)

    def _get_slam_profile(self) -> str:
        return current_slam_profile(self)

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        handle_scene_graph(self, sg)

    def _on_visual_servo_status(self, status: dict) -> None:
        handle_visual_servo_status(self, status)

    def _on_navigation_state(self, state: NavigationState) -> None:
        handle_navigation_state(self, state)

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        handle_navigation_goal_status(self, status)

    def _on_exploration_run_event(self, event: ExplorationRunEvent) -> None:
        handle_exploration_run_event(self, event)

    def _on_inspection_task_event(self, event: InspectionTaskEvent) -> None:
        handle_inspection_task_event(self, event)

    def _on_gnss_fusion_health(self, state: dict) -> None:
        handle_gnss_fusion_health(self, state)

    def _on_localization_status(self, state: dict) -> None:
        handle_localization_status(self, state)

    def _on_tare_stats(self, stats: dict) -> None:
        handle_tare_stats(self, stats)

    def _on_exploration_supervisor(self, state: dict) -> None:
        handle_exploration_supervisor(self, state)

    def _json_payload(self, value: Any) -> Any:
        return json_payload(value)

    def _on_global_path(self, path: Path | list) -> None:
        handle_global_path(self, path)

    @staticmethod
    def _xyz_point(point: Any) -> tuple[float, float, float] | None:
        return xyz_point(point)

    def _on_local_path(self, path: Path) -> None:
        handle_local_path(self, path)

    def _on_native_traversability(self, data: dict[str, Any]) -> None:
        handle_native_traversability(self, data)

    def _on_agent_message(self, msg: dict) -> None:
        handle_agent_message(self, msg)

    def push_event(self, event: dict) -> None:
        sse_push_event(self, event)

    def _traffic_stats_snapshot(self) -> dict[str, Any]:
        return traffic_snapshot(self)

    def _command_stats_snapshot(self) -> dict[str, Any]:
        return self._command_journal.snapshot()

    def _publish_command_ack(
        self,
        payload: dict[str, Any],
        *,
        status_code: int | None = None,
    ) -> None:
        publish_command_ack(self, payload, status_code=status_code)

    def _run_control_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], Mapping[str, Any] | JSONResponse],
        *,
        success_status_code: int = 200,
    ) -> dict[str, Any] | JSONResponse:
        return run_control_command(
            self, command, body, action, success_status_code=success_status_code
        )

    # -- teleop WebSocket state -----------------------------------------------

    def _teleop_client_connected(self) -> int:
        with self._teleop_clients_lock:
            self._teleop_clients += 1
            return self._teleop_clients

    def _teleop_client_disconnected(self) -> int:
        with self._teleop_clients_lock:
            self._teleop_clients = max(0, self._teleop_clients - 1)
            return self._teleop_clients

    def _teleop_client_count(self) -> int:
        with self._teleop_clients_lock:
            return self._teleop_clients

    def _teleop_on_velocity(
        self,
        vx_mps: float,
        vy_mps: float,
        yaw_rps: float,
        *,
        request_id: str | None = None,
        source_id: str = "gateway:teleop",
        source_epoch: int = 1,
        sequence: int = 1,
        manual_mode: bool = False,
    ) -> bool:
        if (
            request_id is None
            and source_id == "gateway:teleop"
            and source_epoch == 1
            and sequence == 1
            and not manual_mode
        ):
            return teleop_on_velocity(self, vx_mps, vy_mps, yaw_rps)
        return teleop_on_velocity_with_request_id(
            self,
            vx_mps,
            vy_mps,
            yaw_rps,
            request_id=request_id,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
            manual_mode=manual_mode,
        )

    def _teleop_claim(
        self,
        *,
        source_id: str,
        source_epoch: int,
        sequence: int,
        lease_ttl_ms: int,
        request_id: str | None = None,
    ) -> OperatorMotionReceipt | bool | None:
        return teleop_claim(
            self,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
            lease_ttl_ms=lease_ttl_ms,
            request_id=request_id,
        )

    def _teleop_release(
        self,
        *,
        source_id: str = "gateway:teleop",
        source_epoch: int = 1,
        sequence: int = 1,
        release_sequence: int | None = None,
        request_id: str | None = None,
        reason: str = "operator_hold",
    ) -> OperatorMotionReceipt | bool | None:
        return teleop_release(
            self,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
            release_sequence=release_sequence,
            request_id=request_id,
            reason=reason,
        )

    # -- FastAPI app --------------------------------------------------------

    def _build_app(self):
        return build_gateway_app(self)

    def _run_server(self, stop_event: threading.Event | None = None) -> bool:
        return run_server(self, stop_event=stop_event)

    # -- health -------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        traffic = self._traffic_stats_snapshot()
        commands = self._command_stats_snapshot()
        n_sse = traffic["sse"]["clients"]
        info["gateway"] = {
            "port": self._port,
            "sse_clients": n_sse,
            "teleop_clients": self._teleop_client_count(),
            "teleop_active": self._teleop_active,
            "has_odom": self._odom is not None,
            "has_sg": self._sg_json != "{}",
            "map_points": self._cloud_viewer.cache_point_count(),
            "traffic": traffic,
            "cloud": self._cloud_viewer.debug_snapshot(),
            "commands": commands,
            "inspection_task_projection": ensure_inspection_task_timeline(self).health(),
        }
        return info

    def clear_map_cloud_cache(self, reason: str = "manual_reset") -> None:
        self._environment_map_feedback.clear()
        self._cloud_viewer.clear(reason=reason)

    def clear_localization_runtime_cache(self, reason: str = "localization_restart") -> None:
        reset_localization_runtime_cache(self, reason=reason)

    # -- SLAM Hz measurement --------------------------------------------------

    def _get_slam_hz_cached(self) -> float:
        return cached_slam_hz(self)
