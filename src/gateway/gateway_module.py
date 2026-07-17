"""GatewayModule -enterprise-grade FastAPI gateway.

Single uvicorn process, shared port 5050.  All external interfaces live here:
  REST API   -typed Pydantic v2 request/response models, validation errors ->422
  SSE        -thread-safe asyncio.Queue fan-out, one queue per connected client
  WebSocket  -teleop joystick + camera stream (replaces separate TeleopModule WS)
  MCP        -JSON-RPC 2.0 endpoint (served by MCPServerModule on port 8090)

Architecture
------------
Module threads write to _state (protected by RLock) and push events via
push_event() (thread-safe).  FastAPI coroutines read _state and dequeue
events -no shared mutable state between threads and coroutines except
through the explicit synchronisation primitives below.

Endpoints
---------
REST
  POST /api/v1/goal          {x,y,z?,instruction?}
  POST /api/v1/cmd_vel       {vx,vy?,wz}
  POST /api/v1/stop
  POST /api/v1/navigation/cancel {reason?, request_id?, client_id?}
  POST /api/v1/instruction   {text}
  POST /api/v1/mode          {mode: manual|autonomous|estop}
  POST /api/v1/lease         {action: acquire|release|renew, client_id, request_id?, ttl?}
  POST /api/v1/maps          {action: list|save|delete|rename|set_active|build_octomap, name?, new_name?}
  GET  /api/v1/state         full snapshot (odom, safety, mission, mode, lease)
  GET  /api/v1/scene_graph
  GET  /api/v1/health
Probes
  GET  /health               liveness probe (always 200 if alive)
  GET  /ready                readiness probe (200 if all modules ok, 503 if degraded)
SSE
  GET  /api/v1/events        event stream  (application/x-ndjson, chunked)
WebSocket
  WS   /ws/teleop            {type:joy, lx,ly,az} | {type:stop}
  WS   /ws/camera            binary JPEG frames
                             ->binary JPEG camera frames
  WS   /ws/cloud             ->binary point-cloud frames (quantized int16,
                                see runtime.utils.binary_codec)

Blueprint usage::

    bp.add(GatewayModule, port=5050)
"""

from __future__ import annotations

import asyncio
import logging
import os
import threading
from functools import lru_cache
from typing import Any, Callable

from gateway.app_factory import build_gateway_app
from gateway.services.commands import (
    publish_command_ack,
    run_control_command,
)
from gateway.services.drift import (
    current_odom_divergence,
    odom_diverged,
    restart_after_drift,
    watchdog_loop,
)
from gateway.services.driver_swap import handle_driver_swap
from gateway.services.event_handlers import (
    handle_agent_message,
    handle_dialogue,
    handle_eval,
    handle_exploration_supervisor,
    handle_frontier_candidate,
    handle_gnss_fusion_health,
    handle_map_event,
    handle_mission,
    handle_safety,
    handle_scene_graph,
    handle_tare_stats,
    handle_traversable_frontiers,
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
    explorer_unavailable_detail,
    tare_status_payload,
)
from gateway.services.http_prewarm import (
    prewarm_client_http_routes,
    start_client_http_prewarm,
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
from gateway.services.lifecycle import start_background_threads, stop_background_threads
from gateway.services.localization_status import handle_localization_status
from gateway.services.map_service import (
    active_map as maps_active_map,
)
from gateway.services.map_service import (
    artifact_path as maps_artifact_path,
)
from gateway.services.map_service import (
    map_bundle as maps_map_bundle,
)
from gateway.services.map_service import (
    maps_service,
)
from gateway.services.map_service import (
    saved_map_points as maps_saved_map_points,
)
from gateway.services.module_refs import (
    attach_module_refs,
    backend_reconfigure_targets,
)
from gateway.services.native_control import teleop_active as native_teleop_active
from gateway.services.odometry import handle_odometry
from gateway.services.pose_recovery import (
    LAST_POSE_MAX_AGE_S,
    LAST_POSE_PATH,
    handle_map_odom_tf,
    load_last_nav_pose,
    persist_last_nav_pose,
    spawn_auto_relocalize,
)
from gateway.services.pose_recovery import (
    clear_localization_runtime_cache as reset_localization_runtime_cache,
)
from gateway.services.saved_map_loader import load_saved_maps_loop
from gateway.services.server import run_server
from gateway.services.session_view import (
    detect_current_mode,
    recover_external_mapping_session,
    session_snapshot,
)
from gateway.services.slam_profile import (
    cached_slam_hz,
    current_slam_profile,
    slam_profile_from_status,
)
from gateway.services.sse import (
    call_queue_put_latest as sse_call_queue_put_latest,
)
from gateway.services.sse import (
    next_event_id as sse_next_event_id,
)
from gateway.services.sse import (
    push_event as sse_push_event,
)
from gateway.services.sse import (
    record_delivery as sse_record_delivery,
)
from gateway.services.sse import (
    running_loop_or_none as sse_running_loop_or_none,
)
from gateway.services.sse import (
    should_emit_raster as sse_should_emit_raster,
)
from gateway.services.sse import (
    subscribe as sse_subscribe,
)
from gateway.services.sse import (
    subscribe_with_event_id as sse_subscribe_with_event_id,
)
from gateway.services.sse import (
    unsubscribe as sse_unsubscribe,
)
from gateway.services.subscriptions import setup_subscriptions
from gateway.services.teleop import (
    configure_teleop as teleop_configure,
)
from gateway.services.teleop import (
    init_teleop_state,
    resolve_native_command_boundary,
    shutdown_teleop,
)
from gateway.services.teleop import (
    on_joy as teleop_on_joy,
)
from gateway.services.teleop import (
    parse_bridge_addr as parse_teleop_bridge_addr,
)
from gateway.services.teleop import (
    publish_remote_velocity_request as teleop_publish_remote_velocity_request,
)
from gateway.services.teleop import (
    release as teleop_release,
)
from gateway.services.teleop import (
    twist_from_joy as teleop_twist_from_joy,
)
from gateway.services.teleop import (
    write_bridge as teleop_write_bridge,
)
from gateway.services.traffic import (
    DEFAULT_SSE_RASTER_MIN_INTERVAL_S,
    DEFAULT_SSE_SLOPE_PAYLOAD_ENABLED,
    DROP_OLDEST_POLICY,
    RECOMMENDED_CLIENT_RATES_HZ,
)
from gateway.services.viewer_events import (
    handle_costmap,
    handle_global_path,
    handle_local_path,
    handle_slope_grid,
    xyz_point,
)
from maps.services.storage import (
    safe_map_name as _map_safe_map_name,
)
from runtime.module import Module
from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.map import MapSceneFrame
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import ExecutionEval, SafetyState, SceneGraph
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)
_MAP_VIEWER_TEMPLATE_PATH = os.path.join(os.path.dirname(__file__), "templates", "map_viewer.html")


@lru_cache(maxsize=1)
def _map_viewer_template_html() -> str:
    with open(_MAP_VIEWER_TEMPLATE_PATH, encoding="utf-8") as f:
        return f.read()


_MOTION_BACKEND_CATEGORIES = {
    "planner",
    "local_planner",
    "path_follower",
    "terrain",
    "slam",
}
_BACKEND_RECONFIGURE_TARGETS = backend_reconfigure_targets()


def _navigation_state(nav: Any) -> str:
    if nav is None:
        return ""
    health: dict[str, Any] = {}
    if hasattr(nav, "health"):
        try:
            raw_health = nav.health() or {}
            if isinstance(raw_health, dict):
                health = raw_health
        except Exception:
            return "UNKNOWN"
    state = health.get("state")
    nested = health.get("navigation")
    if state is None and isinstance(nested, dict):
        state = nested.get("state")
    if hasattr(state, "value"):
        state = state.value
    return str(state or "").upper()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _coerce_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


# Convenience alias - canonical implementation in maps.services.storage.
_safe_map_name = _map_safe_map_name


# ---------------------------------------------------------------------------
# GatewayModule
# ---------------------------------------------------------------------------


@register("gateway", "fastapi", description="FastAPI gateway: REST+SSE+WebSocket teleop")
class GatewayModule(Module, layer=6):
    """HTTP/WebSocket gateway with typed APIs and thread-safe telemetry.

    In:  odometry, scene_graph, safety_state, mission_status,
         execution_eval, dialogue_state, map_event
    Out: goal_pose, cmd_vel, stop_cmd, cancel, instruction, servo_target, mode_cmd
    """

    _run_in_main: bool = True
    _RUNTIME_CACHE_ATTRS = frozenset(
        {
            "_odom",
            "_last_invalid_odometry",
            "_odom_timestamps",
            "_T_map_odom",
            "_has_map_odom_tf",
        }
    )
    _SESSION_RUNTIME_ATTRS = frozenset(
        {
            "_session_mode",
            "_session_product_session",
            "_session_product_profile",
            "_session_map",
            "_session_slam_profile",
            "_session_since",
            "_session_error",
            "_session_pending",
            "_icp_quality",
            "_localization_status",
            "_cached_slam_profile",
            "_slam_profile_ts",
        }
    )

    def __getattr__(self, name: str) -> Any:
        if name in self._RUNTIME_CACHE_ATTRS:
            runtime_cache = self.__dict__.get("_runtime_cache")
            if runtime_cache is not None:
                return getattr(runtime_cache, name)
        if name in self._SESSION_RUNTIME_ATTRS:
            session_runtime = self.__dict__.get("_session_runtime")
            if session_runtime is not None:
                return getattr(session_runtime, name)
        raise AttributeError(f"{type(self).__name__!s} object has no attribute {name!r}")

    def __setattr__(self, name: str, value: Any) -> None:
        if name in type(self)._RUNTIME_CACHE_ATTRS and "_runtime_cache" in self.__dict__:
            setattr(self.__dict__["_runtime_cache"], name, value)
            return
        if name in type(self)._SESSION_RUNTIME_ATTRS and "_session_runtime" in self.__dict__:
            setattr(self.__dict__["_session_runtime"], name, value)
            return
        super().__setattr__(name, value)

    # -- Inputs (module ->cache) --------------------------------------------
    odometry: In[Odometry]
    lidar_scan: In[PointCloud2]  # current raw LiDAR scan for /ws/scan
    map_cloud: In[PointCloud2]
    map_scene: In[MapSceneFrame]  # canonical layered map product
    voxel_cloud: In[PointCloud2]  # cleaned live map layer from VoxelGridModule
    saved_map: In[PointCloud2]  # refined static map from localizer (map frame)
    localization_quality: In[float]  # ICP fitness from SlamBridge -lower=better
    map_odom_tf: In[dict]  # from SlamBridge -{tx,ty,tz,qx,qy,qz,qw,valid} for map->odom
    scene_graph: In[SceneGraph]
    safety_state: In[SafetyState]
    mission_status: In[dict]
    map_event: In[dict]
    execution_eval: In[ExecutionEval]
    dialogue_state: In[dict]
    global_path: In[Path]  # from Navigation
    local_path: In[Path]  # from LocalPlanner -obstacle-free local path
    costmap: In[dict]  # from TraversabilityCostModule -fused cost grid
    slope_grid: In[dict]  # from TraversabilityCostModule -slope in degrees
    agent_message: In[dict]  # from SemanticPlanner -chat-facing messages
    gnss_fusion_health: In[dict]  # from SLAM GNSS/alignment diagnostics
    localization_status: In[dict]  # from SLAM health (cov_trace, iter_num, ...)
    tare_stats: In[dict]  # from TAREExplorerModule -exploration diag
    supervisor_state: In[dict]  # from ExplorationSupervisorModule -watchdog

    # -- Outputs (client commands ->modules) --------------------------------
    traversable_frontiers: In[list]  # read-only TraversableFrontierModule candidates
    frontier_candidate: In[dict]  # read-only best traversable frontier

    goal_pose: Out[PoseStamped]
    cmd_vel: Out[Twist]
    stop_cmd: Out[int]  # 0=clear, 1=soft, 2=hard
    cancel: Out[str]
    instruction: Out[str]
    servo_target: Out[str]
    mode_cmd: Out[str]

    def __init__(self, port: int = 5050, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self._port = port
        self._host = host

        init_core_state(
            self,
            sse_raster_min_interval_s=max(
                0.0,
                _env_float(
                    "LINGTU_SSE_RASTER_MIN_INTERVAL_S",
                    DEFAULT_SSE_RASTER_MIN_INTERVAL_S,
                ),
            ),
            sse_slope_payload_enabled=_env_bool(
                "LINGTU_SSE_SLOPE_PAYLOAD",
                DEFAULT_SSE_SLOPE_PAYLOAD_ENABLED,
            ),
        )

        teleop_dds_env = os.environ.get("LINGTU_TELEOP_CMD_DDS")
        init_teleop_state(
            self,
            max_speed=_env_float("LINGTU_TELEOP_MAX_SPEED_MPS", 0.5),
            max_yaw=_env_float("LINGTU_TELEOP_MAX_YAW_RATE", 1.0),
            release_timeout=_env_float("LINGTU_TELEOP_RELEASE_TIMEOUT_S", 0.5),
            bridge_addr_raw=os.environ.get("LINGTU_TELEOP_BRIDGE_ADDR", ""),
            dds_enabled=resolve_native_command_boundary(
                command_output_mode=os.environ.get("LINGTU_COMMAND_OUTPUT_MODE", ""),
                legacy_dds_env=teleop_dds_env,
            ),
        )

        self._manage_session_services: bool = _coerce_bool(
            kw.get("manage_session_services"),
            _env_bool("LINGTU_MANAGE_SESSION_SERVICES", True),
        )
        init_module_refs(
            self,
            map_save_adapter=kw.get("map_save_adapter"),
            manage_session_services=self._manage_session_services,
        )
        init_recording_state(self)
        init_cloud_and_frame_state(self, frame_tree=kw.get("frame_tree"))

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
            enabled=(
                self._manage_session_services
                and os.environ.get("LINGTU_DRIFT_WATCHDOG", "1") not in ("0", "false", "False")
            ),
            interval_s=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_INTERVAL", "5")),
            xy_limit=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_XY_LIMIT", "50")),
            v_limit=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_V_LIMIT", "10")),
            cooldown_s=float(os.environ.get("LINGTU_DRIFT_WATCHDOG_COOLDOWN", "300")),
            restart_delay_s=float(os.environ.get("LINGTU_DRIFT_RESTART_DELAY_S", "2.0")),
        )

        # Crash-time black box. Records the gateway's view of the world (odom,
        # slam_diag, gnss_fusion, map_odom_tf) and dumps to disk before the
        # watchdog stops services, so we can attribute divergences offline.
        from runtime.utils.blackbox_recorder import BlackBoxRecorder

        self._blackbox = BlackBoxRecorder.from_env()

        init_exploration_state(self)
        init_server_state(self)

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        recover_external_mapping_session(self)
        setup_subscriptions(self)

    def start(self) -> None:
        super().start()
        if self._stop_event.is_set():
            self._stop_event = threading.Event()
        start_background_threads(self)
        logger.info("GatewayModule started on %s:%d", self._host, self._port)

    def _start_client_http_prewarm(
        self,
        stop_event: threading.Event | None = None,
        *,
        timeout_s: float = 15.0,
    ) -> bool:
        return start_client_http_prewarm(
            self,
            stop_event=stop_event,
            timeout_s=timeout_s,
        )

    def _prewarm_client_http_routes(
        self,
        stop_event: threading.Event | None = None,
        *,
        timeout_s: float = 4.0,
    ) -> bool:
        return prewarm_client_http_routes(
            self,
            stop_event=stop_event,
            timeout_s=timeout_s,
        )

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

    def _drift_restart_do_restart(
        self,
        *,
        xy: float,
        y_abs: float,
        v: float,
        stop_event: threading.Event | None = None,
    ) -> None:
        """Stop SLAM services, clear odom cache, ensure session-appropriate
        services back up. Pushes SSE event so Web shows a banner.
        """
        restart_after_drift(self, xy=xy, y_abs=y_abs, v=v, stop_event=stop_event)

    def _saved_map_loader_loop(self, stop_event: threading.Event | None = None) -> None:
        load_saved_maps_loop(self, stop_event=stop_event)

    def stop(self) -> None:
        stop_background_threads(self)
        shutdown_teleop(self)
        self._teleop_native_client = None
        super().stop()

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        attach_module_refs(self, modules)

    def _maps_service(self) -> Any | None:
        return maps_service(self)

    def _active_map_from_maps_service(self) -> str | None:
        return maps_active_map(self)

    def _saved_map_points_from_maps_service(
        self,
        map_name: str,
        *,
        max_points: int,
    ) -> np.ndarray | None:
        return maps_saved_map_points(self, map_name, max_points=max_points)

    def _map_bundle_from_maps_service(
        self,
        map_name: str,
        capability: str,
    ) -> dict[str, Any] | None:
        return maps_map_bundle(self, map_name, capability)

    def _map_artifact_path_from_maps_service(
        self,
        map_name: str,
        capability: str,
    ) -> Path | None:
        return maps_artifact_path(self, map_name, capability)

    def _set_swap_manager(self, swap_manager: Any) -> None:
        """Receive SwapManager reference from Blueprint.build().

        Called after on_system_modules() when the Blueprint's build() method
        creates and activates the SwapManager.  This two-phase setup is
        necessary because SwapManager is not a Module and therefore cannot
        be discovered during the on_system_modules() notification.
        """
        self._swap_manager = swap_manager
        logger.info(
            "GatewayModule: received swap_manager %s",
            swap_manager,
        )

    def _explorer_backend(self) -> str:
        return explorer_backend(self)

    def _explorer_available(self) -> bool:
        return explorer_available(self)

    def _explorer_unavailable_detail(self) -> dict[str, Any]:
        return explorer_unavailable_detail()

    def reconfigure_backend(
        self,
        category: str,
        backend: str,
        **config: Any,
    ) -> dict[str, Any]:
        if category in _MOTION_BACKEND_CATEGORIES:
            state = _navigation_state(self._navigation)
            if state != "IDLE":
                return {
                    "ok": False,
                    "category": category,
                    "requested_backend": backend,
                    "reason": "motion_backend_switch_requires_idle",
                    "navigation_state": state or "UNKNOWN",
                }

        for module_name in backend_reconfigure_targets().get(category, ()):
            module = self._backend_reconfigure_modules.get(module_name)
            reconfigure = getattr(module, "reconfigure_backend", None)
            if callable(reconfigure):
                return reconfigure(category, backend, **config)

        return super().reconfigure_backend(category, backend, **config)

    # ------------------------------------------------------------------
    # Driver swap -delegates to SwapManager when registered
    # ------------------------------------------------------------------

    def _on_driver_swap(self, payload: dict[str, Any]) -> dict[str, Any]:
        return handle_driver_swap(self, payload)

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

    # -- teleop helpers (delegate to TeleopModule) ---------------------------

    @property
    def _teleop_active(self) -> bool:
        """Proxy for teleop active state (lives in TeleopModule now)."""
        if bool(getattr(self, "_teleop_dds_enabled", False)):
            return native_teleop_active()
        tm = self._teleop_module
        return tm._active if tm is not None else False

    def configure_teleop(
        self,
        max_speed: float,
        max_yaw: float,
        release_timeout: float,
    ) -> None:
        """Called by TeleopModule during setup() -config stored for display."""
        teleop_configure(
            self,
            max_speed=max_speed,
            max_yaw=max_yaw,
            release_timeout=release_timeout,
        )

    @staticmethod
    def _parse_teleop_bridge_addr(raw: str) -> tuple[str, int] | None:
        return parse_teleop_bridge_addr(raw)

    def _write_teleop_bridge(self, twist: Twist) -> bool:
        return teleop_write_bridge(self, twist)

    def publish_remote_velocity_request(
        self,
        twist: Twist,
        *,
        publish_local_compat: bool = True,
        request_id: str | None = None,
    ) -> bool:
        return teleop_publish_remote_velocity_request(
            self,
            twist,
            publish_local_compat=publish_local_compat,
            request_id=request_id,
        )

    def _teleop_twist_from_joy(self, lx: float, ly: float, az: float) -> Twist:
        return teleop_twist_from_joy(self, lx, ly, az)

    def push_jpeg(self, jpeg_bytes: bytes) -> None:
        """Called by TeleopModule when a new camera frame is ready."""
        with self._jpeg_lock:
            self._latest_jpeg = jpeg_bytes
            self._latest_jpeg_seq += 1

    # -- Module subscription callbacks -------------------------------------

    def _on_odometry(self, odom: Odometry) -> None:
        handle_odometry(self, odom)

    def _on_icp_quality(self, q: float) -> None:
        try:
            self._session_runtime.set_icp_quality(q)
        except Exception as e:
            logger.debug("_on_icp_quality: failed to convert %r: %s", q, e)

    # Last-known-pose persistence (auto-relocalize on session/start)
    # Keeps a tiny JSON snapshot of the last *successful* relocalize call so
    # that a daemon restart doesn't force the operator to Shift+click again.
    # Only written when the user explicitly relocalized and the localizer
    # reported success; never overwritten by raw odometry (which can drift).
    _LAST_POSE_PATH = LAST_POSE_PATH
    _LAST_POSE_MAX_AGE_S = LAST_POSE_MAX_AGE_S

    def _persist_last_nav_pose(
        self,
        map_name: str,
        x: float,
        y: float,
        yaw: float,
        quality: float | None,
    ) -> None:
        persist_last_nav_pose(
            map_name,
            x,
            y,
            yaw,
            quality,
            path=self._LAST_POSE_PATH,
        )

    def _load_last_nav_pose(self, map_name: str) -> dict | None:
        return load_last_nav_pose(
            map_name,
            path=self._LAST_POSE_PATH,
            max_age_s=self._LAST_POSE_MAX_AGE_S,
        )

    def _spawn_auto_relocalize(self, map_name: str) -> None:
        spawn_auto_relocalize(self, map_name)

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
        """Read the active map only through the maps service contract."""
        return self._active_map_from_maps_service()

    def _session_snapshot(self) -> dict:
        return session_snapshot(self)

    def _on_saved_map(self, cloud: PointCloud2) -> None:
        """DDS-stream saved-map -intentionally ignored.

        Localizer publishes a refined saved_map every tick (~10 Hz). That
        previously got downsampled and re-pushed every 10 frames, but the
        frontend rebuilds the whole 80k-pt mesh on each event ->flicker.
        The MapsService snapshot pushed once by _saved_map_loader_loop is
        stable enough for visualization; the localizer's runtime refinement
        doesn't change the base map meaningfully for the operator view.
        """
        return

    def _viewer_scan_allowed_frames(self) -> set[str]:
        return self._cloud_viewer.viewer_scan_allowed_frames()

    def _is_viewer_scan_frame_compatible(self, cloud: PointCloud2) -> bool:
        return self._cloud_viewer.is_viewer_scan_frame_compatible(cloud)

    def _on_lidar_scan(self, cloud: PointCloud2) -> None:
        self._cloud_viewer.on_lidar_scan(cloud)

    def _on_map_cloud(self, cloud: PointCloud2) -> None:
        self._cloud_viewer.on_map_cloud(cloud)

    def _on_map_scene(self, frame: MapSceneFrame | dict[str, Any]) -> None:
        self._cloud_viewer.on_map_scene(frame)

    def _on_voxel_cloud(self, cloud: PointCloud2) -> None:
        self._cloud_viewer.on_voxel_cloud(cloud)

    def _handle_scan_cloud(
        self,
        cloud: PointCloud2,
        *,
        source: str,
        fallback: bool = False,
    ) -> None:
        self._cloud_viewer.handle_scan_cloud(cloud, source=source, fallback=fallback)

    def _handle_view_cloud(
        self,
        cloud: PointCloud2,
        *,
        source: str,
        authoritative: bool,
    ) -> None:
        self._cloud_viewer.handle_view_cloud(
            cloud,
            source=source,
            authoritative=authoritative,
        )

    def _should_publish_view_cloud(
        self,
        *,
        mode: str,
        cache_points: int,
        now: float,
    ) -> bool:
        return self._cloud_viewer.should_publish_view_cloud(
            mode=mode,
            cache_points=cache_points,
            now=now,
        )

    @staticmethod
    def _slam_profile_from_status(status: dict | None) -> str:
        return slam_profile_from_status(status)

    def _get_slam_profile(self) -> str:
        return current_slam_profile(self)

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        handle_scene_graph(self, sg)

    def _on_safety(self, state: SafetyState) -> None:
        handle_safety(self, state)

    def _on_mission(self, status: dict) -> None:
        handle_mission(self, status)

    def _on_map_event(self, event: dict) -> None:
        handle_map_event(self, event)

    def _on_eval(self, ev: ExecutionEval) -> None:
        handle_eval(self, ev)

    def _on_dialogue(self, state: dict) -> None:
        handle_dialogue(self, state)

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

    def _on_traversable_frontiers(self, candidates: list) -> None:
        handle_traversable_frontiers(self, candidates)

    def _on_frontier_candidate(self, candidate: dict) -> None:
        handle_frontier_candidate(self, candidate)

    def _on_global_path(self, path: Path | list) -> None:
        handle_global_path(self, path)

    @staticmethod
    def _xyz_point(point: Any) -> tuple[float, float, float] | None:
        return xyz_point(point)

    def _on_local_path(self, path: Path) -> None:
        handle_local_path(self, path)

    def _on_costmap(self, cm: dict) -> None:
        handle_costmap(self, cm)

    def _on_slope_grid(self, data: dict) -> None:
        handle_slope_grid(self, data)

    def _on_agent_message(self, msg: dict) -> None:
        handle_agent_message(self, msg)

    def _should_emit_sse_raster(self, event_type: str) -> bool:
        return sse_should_emit_raster(self, event_type)

    # -- SSE fan-out --------------------------------------------------------

    @staticmethod
    def _running_loop_or_none() -> asyncio.AbstractEventLoop | None:
        return sse_running_loop_or_none()

    @staticmethod
    def _call_queue_put_latest(
        q: asyncio.Queue,
        item: Any,
        loop: asyncio.AbstractEventLoop | None,
        record: Callable[[bool, int], None],
    ) -> None:
        sse_call_queue_put_latest(q, item, loop, record)

    def _record_sse_delivery(self, dropped: bool, depth: int) -> None:
        sse_record_delivery(self, dropped, depth)

    def _record_cloud_delivery(self, dropped: bool, depth: int) -> None:
        self._cloud_viewer.record_cloud_delivery(dropped, depth)

    def push_event(self, event: dict) -> None:
        sse_push_event(self, event)

    def _next_sse_event_id(self) -> int:
        return sse_next_event_id(self)

    def _sse_subscribe_with_event_id(self) -> tuple[asyncio.Queue, int]:
        return sse_subscribe_with_event_id(self)

    def _sse_subscribe(self) -> asyncio.Queue:
        return sse_subscribe(self)

    def _sse_unsubscribe(self, q: asyncio.Queue) -> None:
        sse_unsubscribe(self, q)

    # -- Binary cloud fan-out ----------------------------------------------

    def _publish_cloud_frame(
        self,
        buf: bytes,
        *,
        metadata: dict[str, Any] | None = None,
    ) -> int:
        return self._cloud_viewer.publish_cloud_frame(buf, metadata=metadata)

    def _cloud_subscribe(self) -> tuple[asyncio.Queue, bytes | None]:
        return self._cloud_viewer.cloud_subscribe()

    def _cloud_unsubscribe(self, q: asyncio.Queue) -> None:
        self._cloud_viewer.cloud_unsubscribe(q)

    def _record_scan_delivery(self, dropped: bool, depth: int) -> None:
        self._cloud_viewer.record_scan_delivery(dropped, depth)

    def _publish_scan_frame(
        self,
        buf: bytes,
        *,
        metadata: dict[str, Any] | None = None,
    ) -> int:
        return self._cloud_viewer.publish_scan_frame(buf, metadata=metadata)

    def _scan_subscribe(self) -> tuple[asyncio.Queue, bytes | None]:
        return self._cloud_viewer.scan_subscribe()

    def _scan_unsubscribe(self, q: asyncio.Queue) -> None:
        self._cloud_viewer.scan_unsubscribe(q)

    def _traffic_stats_snapshot(self) -> dict[str, Any]:
        with self._sse_lock:
            sse_depths = [q.qsize() for q in self._sse_queues]
            sse = {
                "clients": len(self._sse_queues),
                "queue_maxsize": self._sse_queue_maxsize,
                "queue_depths": sse_depths,
                "max_depth_seen": self._sse_max_depth_seen,
                "latest_event_id": self._sse_event_seq,
                "published_events": self._sse_published_events,
                "dropped_events": self._sse_dropped_events,
                "suppressed_events": dict(self._sse_suppressed_events),
                "raster_min_interval_s": self._sse_raster_min_interval_s,
                "slope_grid_inline": self._sse_slope_payload_enabled,
                "drop_policy": DROP_OLDEST_POLICY,
            }
        realtime = self._cloud_viewer.traffic_snapshot()
        return {
            "sse": sse,
            "cloud": realtime["cloud"],
            "scan": realtime["scan"],
            "recommended_client_rates_hz": dict(RECOMMENDED_CLIENT_RATES_HZ),
        }

    def _cloud_debug_snapshot(self) -> dict[str, Any]:
        return self._cloud_viewer.debug_snapshot()

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
        action: Callable[[], dict[str, Any]],
    ) -> dict[str, Any]:
        return run_control_command(self, command, body, action)

    # -- teleop internals (forwarded to TeleopModule) -------------------------

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

    def _teleop_on_joy(self, lx: float, ly: float, az: float) -> bool:
        return teleop_on_joy(self, lx, ly, az)

    def _teleop_release(self) -> bool:
        return teleop_release(self)

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
            "cloud": self._cloud_debug_snapshot(),
            "commands": commands,
        }
        return info

    def clear_map_cloud_cache(self, reason: str = "manual_reset") -> None:
        self._cloud_viewer.clear(reason=reason)

    def map_cloud_point_count(self) -> int:
        return self._cloud_viewer.cache_point_count()

    def map_cloud_frame_count(self) -> int:
        return self._cloud_viewer.cache_frames_seen()

    def map_cloud_summary(self) -> dict[str, Any]:
        return self._cloud_viewer.map_summary()

    def map_cloud_points_array(self) -> Any:
        return self._cloud_viewer.map_points_array()

    def replace_map_cloud_points(self, pts_xyz: Any) -> None:
        self._cloud_viewer.replace_map_points(np.asarray(pts_xyz, dtype=np.float32))

    def map_cloud_points_snapshot(self, *, max_points: int = 80000) -> dict[str, Any]:
        return self._cloud_viewer.map_points_snapshot(max_points=max_points)

    def configure_cloud_viewer(self, **kwargs: Any) -> None:
        self._cloud_viewer.configure(**kwargs)

    def cloud_viewer_config(self) -> dict[str, Any]:
        return self._cloud_viewer.viewer_config()

    def clean_map_layer_prefer_s(self) -> float:
        return self._cloud_viewer.clean_map_layer_prefer_s()

    def mark_clean_map_layer_recent(self, ts: float | None = None) -> None:
        self._cloud_viewer.mark_clean_map_layer_recent(ts)

    def adjust_cloud_viewer_last_publish_ts(self, delta_s: float) -> None:
        self._cloud_viewer.adjust_last_view_publish_ts(delta_s)

    def adjust_slam_map_scan_ts(self, delta_s: float) -> None:
        self._cloud_viewer.adjust_last_slam_map_scan_ts(delta_s)

    def cloud_queue_maxsize(self) -> int:
        return self._cloud_viewer.cloud_queue_maxsize()

    def scan_queue_maxsize(self) -> int:
        return self._cloud_viewer.scan_queue_maxsize()

    def cloud_published_frames(self) -> int:
        return self._cloud_viewer.cloud_published_frames()

    def scan_published_frames(self) -> int:
        return self._cloud_viewer.scan_published_frames()

    def latest_cloud_metadata(self) -> dict[str, Any]:
        return self._cloud_viewer.latest_cloud_metadata()

    def clear_localization_runtime_cache(self, reason: str = "localization_restart") -> None:
        reset_localization_runtime_cache(self, reason=reason)

    # -- SLAM Hz measurement --------------------------------------------------

    def _get_slam_hz_cached(self) -> float:
        return cached_slam_hz(self)

    # -- Map 3D viewer HTML generation ----------------------------------------

    def _generate_viewer_live(self) -> str:
        """Return the static viewer shell; live data arrives via SSE/WebSocket."""
        return self._generate_viewer_html()

    def _generate_viewer_from_pcd(self, map_name: str) -> str:
        """Return the static viewer shell for a saved-map query string."""
        _map_safe_map_name(map_name)
        return self._generate_viewer_html()

    def _generate_viewer_html(self, override_pts=None, robot_pos=None) -> str:
        """Return cached legacy viewer HTML.

        The HTML shell is static. Dynamic data arrives through SSE and
        ``/ws/cloud`` binary point-cloud frames, so this method must not parse
        PCD files or read the template on every request.
        """
        return _map_viewer_template_html()
