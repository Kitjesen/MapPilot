"""GatewayModule — enterprise-grade FastAPI gateway.

Single uvicorn process, shared port 5050.  All external interfaces live here:
  REST API   — typed Pydantic v2 request/response models, validation errors → 422
  SSE        — thread-safe asyncio.Queue fan-out, one queue per connected client
  WebSocket  — teleop joystick + camera stream (replaces separate TeleopModule WS)
  MCP        — JSON-RPC 2.0 endpoint (served by MCPServerModule on port 8090)

Architecture
------------
Module threads write to _state (protected by RLock) and push events via
push_event() (thread-safe).  FastAPI coroutines read _state and dequeue
events — no shared mutable state between threads and coroutines except
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
  POST /api/v1/maps          {action: list|save|delete|rename|set_active|build_tomogram, name?, new_name?}
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
                             ← binary JPEG camera frames
  WS   /ws/cloud             ← binary point-cloud frames (quantized int16,
                                see core.utils.binary_codec)

Blueprint usage::

    bp.add(GatewayModule, port=5050)
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path as FilePath
from typing import Any, Callable

import numpy as np

from core.module import Module
from core.msgs.geometry import PoseStamped, Twist, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.semantic import ExecutionEval, SafetyState, SceneGraph
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import In, Out
from gateway.schemas import (
    DriverSwapRequest,
    DriverSwapResponse,
    GatewayErrorResponse,
    MapLifecycleResponse,
    MapRequest,
)
from gateway.services.commands import CommandJournal
from gateway.services.map_paths import (
    active_map_name,
    map_dir_for,
)
from core.dynamic_filter import (
    apply_dynamic_filter_step1half as _map_apply_dynamic_filter_step1half,
)
from gateway.services.map_safety import (
    safe_map_name as _map_safe_map_name,
)
from gateway.services.runtime_status import (
    backend_capability_defaults,
    classify_pose_freshness,
    localizer_algorithm_healthy,
)
from gateway.services.safety_status import (
    SAFETY_STOP_BLOCKER,
    safety_clear_for_motion,
    safety_summary,
)
from gateway.services.traffic import (
    DEFAULT_CLOUD_QUEUE_MAXSIZE,
    DEFAULT_SSE_QUEUE_MAXSIZE,
    DEFAULT_SSE_RASTER_MIN_INTERVAL_S,
    DEFAULT_SSE_SLOPE_PAYLOAD_ENABLED,
    DROP_OLDEST_POLICY,
    RECOMMENDED_CLIENT_RATES_HZ,
    normalize_sse_event,
    put_latest,
)

logger = logging.getLogger(__name__)

_MOTION_BACKEND_CATEGORIES = {
    "planner",
    "local_planner",
    "path_follower",
    "terrain",
    "slam",
}

_BACKEND_RECONFIGURE_TARGETS = {
    "detector": ("PerceptionModule",),
    "encoder": ("PerceptionModule",),
    "llm": ("LLMModule",),
    "llm_client": ("LLMModule",),
    "planner": ("NavigationModule",),
    "local_planner": ("LocalPlannerModule",),
    "path_follower": ("PathFollowerModule",),
    "terrain": ("TerrainModule",),
    "slam": ("SlamBridgeModule", "SLAMModule", "SlamModule"),
}


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
    if state is None:
        state = getattr(nav, "_state", "")
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


# Convenience aliases — canonical implementations in gateway.services.map_safety
_safe_map_name = _map_safe_map_name
_apply_dynamic_filter_step1half = _map_apply_dynamic_filter_step1half


# ---------------------------------------------------------------------------
# Lease state
# ---------------------------------------------------------------------------

class _Lease:
    """Simple mutex-protected control lease."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._holder: str | None = None
        self._expiry: float = 0.0

    def acquire(self, client_id: str, ttl: float) -> bool:
        with self._lock:
            now = time.monotonic()
            if self._holder and self._holder != client_id and now < self._expiry:
                return False
            self._holder = client_id
            self._expiry = now + ttl
            return True

    def release(self, client_id: str) -> None:
        with self._lock:
            if self._holder == client_id:
                self._holder = None
                self._expiry = 0.0

    def renew(self, client_id: str, ttl: float) -> bool:
        with self._lock:
            if self._holder == client_id:
                self._expiry = time.monotonic() + ttl
                return True
            return False

    def check(self, client_id: str) -> bool:
        with self._lock:
            if self._holder is None:
                return True
            return self._holder == client_id and time.monotonic() < self._expiry

    def to_dict(self) -> dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            return {
                "holder":  self._holder,
                "active":  self._holder is not None and now < self._expiry,
                "expires_in": max(0.0, self._expiry - now) if self._holder else 0.0,
            }


# ---------------------------------------------------------------------------
# GatewayModule
# ---------------------------------------------------------------------------

@register("gateway", "fastapi", description="FastAPI gateway: REST+SSE+WebSocket teleop")
class GatewayModule(Module, layer=6):
    """HTTP/WebSocket gateway with typed APIs and thread-safe telemetry.

    In:  odometry, scene_graph, safety_state, mission_status,
         execution_eval, dialogue_state
    Out: goal_pose, cmd_vel, stop_cmd, cancel, instruction, mode_cmd
    """

    _run_in_main: bool = True

    # -- Inputs (module → cache) --------------------------------------------
    odometry:       In[Odometry]
    map_cloud:      In[PointCloud2]
    saved_map:      In[PointCloud2]  # refined static map from localizer (map frame)
    localization_quality: In[float]  # ICP fitness from SlamBridge — lower=better
    map_odom_tf:    In[dict]         # from SlamBridge — {tx,ty,tz,qx,qy,qz,qw,valid} for map→odom
    scene_graph:    In[SceneGraph]
    safety_state:   In[SafetyState]
    mission_status: In[dict]
    execution_eval: In[ExecutionEval]
    dialogue_state: In[dict]
    global_path:    In[list]  # from NavigationModule — list of np.ndarray [x,y,z]
    local_path:     In[Path]  # from LocalPlannerModule — obstacle-free local path
    costmap:        In[dict]  # from TraversabilityCostModule — fused cost grid
    slope_grid:     In[dict]  # from TraversabilityCostModule — slope in degrees
    agent_message:  In[dict]  # from SemanticPlanner — chat-facing messages
    gnss_fusion_health: In[dict]  # from SlamBridgeModule — GNSS/SLAM alignment diag
    localization_status: In[dict] # from SlamBridgeModule — full SLAM health (cov_trace, iter_num, ...)
    tare_stats:         In[dict]  # from TAREExplorerModule — exploration diag
    supervisor_state:   In[dict]  # from ExplorationSupervisorModule — watchdog

    # -- Outputs (client commands → modules) --------------------------------
    traversable_frontiers: In[list]  # read-only TraversableFrontierModule candidates
    frontier_candidate:    In[dict]  # read-only best traversable frontier

    goal_pose:   Out[PoseStamped]
    cmd_vel:     Out[Twist]
    stop_cmd:    Out[int]    # 0=clear, 1=soft, 2=hard
    cancel:      Out[str]
    instruction: Out[str]
    mode_cmd:    Out[str]

    def __init__(self, port: int = 5050, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self._port = port
        self._host = host

        # Cached state — written by Module callbacks, read by HTTP handlers.
        # Protected by RLock so callbacks and HTTP handler threads don't race.
        self._state_lock = threading.RLock()
        self._odom:     dict | None = None
        self._last_invalid_odometry: dict | None = None
        self._sg_json:  str = "{}"
        self._safety:   dict | None = None
        self._mission:  dict | None = None
        self._eval:     dict | None = None
        self._dialogue: dict | None = None
        self._mode: str = "manual"
        self._last_path: list[dict] = []

        self._lease = _Lease()
        self._command_journal = CommandJournal()

        # SSE fan-out — one asyncio.Queue per connected client.
        # Written from Module threads via push_event() (thread-safe).
        self._sse_lock:   threading.Lock = threading.Lock()
        self._sse_queues: list[asyncio.Queue] = []
        self._sse_queue_loops: dict[asyncio.Queue, asyncio.AbstractEventLoop | None] = {}
        self._sse_queue_maxsize: int = DEFAULT_SSE_QUEUE_MAXSIZE
        self._sse_event_seq: int = 0
        self._sse_published_events: int = 0
        self._sse_dropped_events: int = 0
        self._sse_max_depth_seen: int = 0
        self._sse_raster_min_interval_s: float = max(
            0.0,
            _env_float("LINGTU_SSE_RASTER_MIN_INTERVAL_S", DEFAULT_SSE_RASTER_MIN_INTERVAL_S),
        )
        self._sse_slope_payload_enabled: bool = _env_bool(
            "LINGTU_SSE_SLOPE_PAYLOAD",
            DEFAULT_SSE_SLOPE_PAYLOAD_ENABLED,
        )
        self._sse_raster_last_emit: dict[str, float] = {}
        self._sse_suppressed_events: dict[str, int] = {}

        # Teleop: delegate to TeleopModule (set by on_system_modules)
        self._teleop_module = None
        self._teleop_clients:   int  = 0
        self._teleop_clients_lock = threading.Lock()
        self._latest_jpeg:  bytes | None = None
        self._latest_jpeg_seq: int = 0
        self._jpeg_lock: threading.Lock = threading.Lock()

        # Reference to MapManagerModule (set by on_system_modules)
        self._map_mgr = None
        # All modules dict (set by on_system_modules)
        self._all_modules: dict[str, Any] = {}

        # rosbag recording state
        self._bag_proc: Any = None       # subprocess.Popen
        self._bag_path: str = ""
        self._bag_started_ts: float = 0.0
        self._bag_lock = threading.Lock()

        # Map cloud accumulator for /map/viewer
        self._map_points: np.ndarray | None = None
        self._map_cloud_lock = threading.Lock()
        self._map_cloud_count: int = 0
        self._map_voxel_size: float = 0.15
        self._inv_map_voxel_size: float = 1.0 / 0.15

        # Dynamic-obstacle removal — hit-count voting per voxel.
        # Phase 1 of docs/05-specialized/dynamic_obstacle_removal.md.
        # 同一 voxel 被多少帧观测到 = hit count。墙被扫 30-100 次,人走过每
        # 位置只 1-3 次。发送到 Web 时过滤 hit < min_hits 的格子,动态残影
        # 自然消失。只在 mapping / exploring 模式下生效。
        self._voxel_hits: dict[int, int] = {}
        self._voxel_min_hits: int = int(os.environ.get("LINGTU_MAP_MIN_HITS", "3"))
        self._voxel_key_offset: int = 1 << 19  # pad to keep packed keys non-negative

        # map→odom TF (from localizer via SlamBridge). Applied to odom-frame
        # map_cloud before SSE so the frontend sees it overlaid with saved_map
        # (which is already in map frame). Identity until localizer converges.
        self._T_map_odom: np.ndarray = np.eye(4, dtype=np.float64)
        self._has_map_odom_tf: bool = False

        # WebRTCStreamModule reference (set by on_system_modules).  Kept as
        # an attribute so the /api/v1/webrtc/offer route can check presence
        # without an AttributeError when aiortc is not installed.
        self._webrtc: Any = None
        self._go2rtc_upstream: str = os.environ.get(
            "LINGTU_GO2RTC_URL",
            "http://127.0.0.1:1984",
        )

        # Binary cloud channel — one frame buffer + per-client asyncio.Queue.
        # Points are quantized int16 (see core.utils.binary_codec) so a 60k
        # cloud is ~360 KB instead of ~1.4 MB JSON, and the browser decodes
        # it as a zero-copy Int16Array.  See /ws/cloud endpoint.
        self._cloud_lock = threading.Lock()
        self._latest_cloud_buf: bytes | None = None
        self._cloud_seq: int = 0
        self._cloud_subs: list[asyncio.Queue] = []
        self._cloud_sub_loops: dict[asyncio.Queue, asyncio.AbstractEventLoop | None] = {}
        self._cloud_queue_maxsize: int = DEFAULT_CLOUD_QUEUE_MAXSIZE
        self._cloud_published_frames: int = 0
        self._cloud_dropped_frames: int = 0
        self._cloud_max_depth_seen: int = 0

        # Odometry rate tracking (sliding window for SLAM Hz display)
        self._odom_timestamps: list = []  # last 20 timestamps

        # Lazy TemporalStore handle — shared file with TemporalMemoryModule
        self._temporal_store: Any = None

        # Costmap SSE throttle (publish at ~2Hz regardless of OccupancyGridModule rate)
        self._costmap_throttle: int = 0

        # SceneGraph SSE throttle (~2Hz)
        self._sg_throttle: int = 0

        # SLAM status SSE throttle (~1Hz at 10Hz odom)
        self._slam_status_throttle: int = 0

        # Cached SLAM profile (fastlio2 / localizer / super_lio / stopped)
        self._cached_slam_profile: str = "—"
        self._slam_profile_ts: float = 0.0
        self._brainstem_health_lock = threading.Lock()
        self._brainstem_health_cache: dict[str, Any] | None = None
        self._brainstem_health_cache_ts: float = 0.0
        self._brainstem_health_refreshing: bool = False
        self._brainstem_health_refresh_thread: threading.Thread | None = None
        self._brainstem_health_cache_ttl_s: float = max(
            0.0,
            _env_float("LINGTU_BRAINSTEM_HEALTH_CACHE_S", 2.0),
        )

        # SLAM drift watchdog — 兜底 Fast-LIO2 静置 IEKF 溢出 (xy 飞到万亿米级).
        # 每 interval 秒查 odom, 超阈值自动 stop+ensure slam.service 重置 IEKF.
        # 详见 docs/05-specialized/slam_drift_watchdog.md (TBD) + memory.
        self._drift_watchdog_enabled: bool = os.environ.get(
            "LINGTU_DRIFT_WATCHDOG", "1") not in ("0", "false", "False")
        self._drift_watchdog_interval: float = float(
            os.environ.get("LINGTU_DRIFT_WATCHDOG_INTERVAL", "5"))
        self._drift_watchdog_xy_limit: float = float(
            os.environ.get("LINGTU_DRIFT_WATCHDOG_XY_LIMIT", "50"))
        self._drift_watchdog_v_limit: float = float(
            os.environ.get("LINGTU_DRIFT_WATCHDOG_V_LIMIT", "10"))
        self._drift_watchdog_cooldown: float = float(
            os.environ.get("LINGTU_DRIFT_WATCHDOG_COOLDOWN", "300"))
        self._drift_restart_delay_s: float = float(
            os.environ.get("LINGTU_DRIFT_RESTART_DELAY_S", "2.0"))
        self._drift_last_restart_ts: float = 0.0
        self._drift_restart_count: int = 0
        self._drift_watchdog_thread: threading.Thread | None = None

        # Crash-time black box. Records the gateway's view of the world (odom,
        # slam_diag, gnss_fusion, map_odom_tf) and dumps to disk before the
        # watchdog stops services, so we can attribute divergences offline.
        from core.utils.blackbox_recorder import BlackBoxRecorder
        self._blackbox = BlackBoxRecorder.from_env()

        # ── Session state machine (single source of truth) ─────────────────
        # Every mode transition must go through /api/v1/session/start|end.
        # Frontend Topbar/Panel render from this state; no other code path
        # should invoke svc.ensure/stop directly (deprecated /slam/switch
        # forwards here).
        self._session_mode: str = "idle"        # idle | mapping | navigating
        self._session_map: str | None = None    # active map name for navigating
        self._session_slam_profile: str = "stopped"
        self._session_since: float = time.time()
        self._session_error: str = ""            # last error from start/end
        self._session_pending: bool = False      # True while transitioning
        self._icp_quality: float = 0.0           # from localization_quality
        self._localization_status: dict | None = None

        # Autonomous exploration state + explorer module refs
        self._exploring: bool = False
        self._frontier_explorer: Any = None
        self._tare_explorer: Any = None
        self._last_tare_stats: dict[str, Any] | None = None
        self._exploration_supervisor_state: dict[str, Any] | None = None
        self._last_traversable_frontiers: list[Any] | None = None
        self._last_frontier_candidate: dict[str, Any] | None = None

        # TaggedLocationsModule ref (set by on_system_modules)
        self._tagged_loc_module: Any = None

        # Driver SwapManager ref (set by on_system_modules when SwapManager is available)
        self._swap_manager: Any = None

        self._app   = None
        self._server: Any = None
        self._server_thread: threading.Thread | None = None
        self._client_http_prewarm_thread: threading.Thread | None = None
        self._saved_map_loader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._defer_server: bool = False  # True → main thread runs uvicorn

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odometry)
        self.map_cloud.subscribe(self._on_map_cloud)
        self.map_cloud.set_policy("latest")
        self.saved_map.subscribe(self._on_saved_map)
        self.saved_map.set_policy("latest")
        self.localization_quality.subscribe(self._on_icp_quality)
        self.localization_quality.set_policy("latest")
        self.map_odom_tf.subscribe(self._on_map_odom_tf)
        self.map_odom_tf.set_policy("latest")
        self.scene_graph.subscribe(self._on_scene_graph)
        self.safety_state.subscribe(self._on_safety)
        self.mission_status.subscribe(self._on_mission)
        self.execution_eval.subscribe(self._on_eval)
        self.dialogue_state.subscribe(self._on_dialogue)
        self.global_path.subscribe(self._on_global_path)
        self.local_path.subscribe(self._on_local_path)
        self.costmap.subscribe(self._on_costmap)
        self.costmap.set_policy("latest")
        self.slope_grid.subscribe(self._on_slope_grid)
        self.slope_grid.set_policy("latest")
        self.agent_message.subscribe(self._on_agent_message)
        self.gnss_fusion_health.subscribe(self._on_gnss_fusion_health)
        self.localization_status.subscribe(self._on_localization_status)
        self.localization_status.set_policy("latest")
        self.tare_stats.subscribe(self._on_tare_stats)
        self.supervisor_state.subscribe(self._on_exploration_supervisor)
        self.traversable_frontiers.subscribe(self._on_traversable_frontiers)
        self.traversable_frontiers.set_policy("latest")
        self.frontier_candidate.subscribe(self._on_frontier_candidate)
        self.frontier_candidate.set_policy("latest")
        self._app = self._build_app()

    def start(self) -> None:
        super().start()
        if self._stop_event.is_set():
            self._stop_event = threading.Event()
        stop_event = self._stop_event
        if (
            not self._defer_server
            and (
                self._server_thread is None
                or not self._server_thread.is_alive()
            )
        ):
            self._server_thread = threading.Thread(
                target=self._run_server,
                args=(stop_event,),
                daemon=True,
                name="gateway",
            )
            self._server_thread.start()
            self._start_client_http_prewarm(stop_event, timeout_s=15.0)
        # Background: load active map.pcd from disk and push as saved_map
        # event so the frontend底图 always has the stored map regardless of
        # whether localizer has converged. Re-pushes periodically so late-
        # connecting SSE clients also get it.
        if (
            self._saved_map_loader_thread is None
            or not self._saved_map_loader_thread.is_alive()
        ):
            self._saved_map_loader_thread = threading.Thread(
                target=self._saved_map_loader_loop,
                args=(stop_event,),
                daemon=True,
                name="saved_map_loader",
            )
            self._saved_map_loader_thread.start()

        # Drift watchdog — 兜底 Fast-LIO2 静置 IEKF 溢出.
        if (
            self._drift_watchdog_enabled
            and (
                self._drift_watchdog_thread is None
                or not self._drift_watchdog_thread.is_alive()
            )
        ):
            self._drift_watchdog_thread = threading.Thread(
                target=self._drift_watchdog_loop,
                args=(stop_event,),
                daemon=True,
                name="drift_watchdog",
            )
            self._drift_watchdog_thread.start()

        logger.info("GatewayModule started on %s:%d", self._host, self._port)

    def _start_client_http_prewarm(
        self,
        stop_event: threading.Event | None = None,
        *,
        timeout_s: float = 15.0,
    ) -> bool:
        stop_event = stop_event or self._stop_event
        if stop_event.is_set():
            return False
        if (
            self._client_http_prewarm_thread is not None
            and self._client_http_prewarm_thread.is_alive()
        ):
            return False
        self._client_http_prewarm_thread = threading.Thread(
            target=self._prewarm_client_http_routes,
            args=(stop_event,),
            kwargs={"timeout_s": timeout_s},
            daemon=True,
            name="gateway_client_prewarm",
        )
        self._client_http_prewarm_thread.start()
        return True

    def _prewarm_client_http_routes(
        self,
        stop_event: threading.Event | None = None,
        *,
        timeout_s: float = 4.0,
    ) -> bool:
        """Consume first-use FastAPI/Pydantic cost before App/Web clients arrive."""
        stop_event = stop_event or self._stop_event
        if stop_event.is_set():
            return False
        try:
            from urllib.error import URLError
            from urllib.request import Request, urlopen
        except Exception as e:
            logger.debug("GatewayModule: urllib import in prewarm failed: %s", e)
            return False

        headers: dict[str, str] = {}
        try:
            from gateway.auth import _get_configured_key

            api_key = _get_configured_key()
            if api_key:
                headers["X-API-Key"] = api_key
        except Exception as e:
            logger.debug("GatewayModule: failed to load API key for prewarm: %s", e)

        url = f"http://127.0.0.1:{self._port}/api/v1/app/capabilities"
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline and not stop_event.is_set():
            try:
                request_timeout = min(2.5, max(0.1, deadline - time.time()))
                with urlopen(
                    Request(url, headers=headers),
                    timeout=request_timeout,
                ) as response:
                    response.read()
                logger.info("GatewayModule: prewarmed App/Web capabilities route")
                return True
            except (OSError, URLError):
                stop_event.wait(0.1)
            except Exception:
                logger.debug(
                    "GatewayModule: App/Web HTTP prewarm failed",
                    exc_info=True,
                )
                return False
        return False

    def _drift_odom_diverged(
        self,
        odom: dict[str, Any],
    ) -> tuple[bool, float, float, float, float, bool]:
        def abs_finite(key: str) -> tuple[float, bool]:
            try:
                value = float(odom.get(key, 0.0))
            except (TypeError, ValueError):
                return float("inf"), False
            if not math.isfinite(value):
                return float("inf"), False
            return abs(value), True

        x, x_ok = abs_finite("x")
        y, y_ok = abs_finite("y")
        z, z_ok = abs_finite("z")
        vx, vx_ok = abs_finite("vx")
        vy, vy_ok = abs_finite("vy")
        vz, vz_ok = abs_finite("vz")
        v = max(vx, vy, vz)
        invalid = not all((x_ok, y_ok, z_ok, vx_ok, vy_ok, vz_ok))
        xy_bad = (
            x > self._drift_watchdog_xy_limit
            or y > self._drift_watchdog_xy_limit
            or z > self._drift_watchdog_xy_limit
        )
        v_bad = v > self._drift_watchdog_v_limit
        return invalid or xy_bad or v_bad, x, y, z, v, invalid

    def _drift_current_odom_divergence(
        self,
    ) -> tuple[bool, float, float, float, float, bool]:
        with self._state_lock:
            invalid_odom = (
                dict(self._last_invalid_odometry)
                if self._last_invalid_odometry
                else None
            )
            odom = dict(self._odom) if self._odom else {}
        if invalid_odom:
            return True, float("inf"), 0.0, 0.0, float("inf"), True
        if not odom:
            return False, 0.0, 0.0, 0.0, 0.0, False
        return self._drift_odom_diverged(odom)

    def _drift_watchdog_loop(
        self, stop_event: threading.Event | None = None
    ) -> None:
        """Periodic sanity check on odom; restart SLAM services if IEKF diverged.

        Fast-LIO2 IEKF co-variance grows unbounded under static poses (no new
        LiDAR observations to suppress it). After hours of idle, xy can blow
        out to trillions of metres. We detect via absolute bounds and restart
        the SLAM services (+ the current session's companion services) to reset
        the IEKF to zero.

        Cooldown: at most one restart per LINGTU_DRIFT_WATCHDOG_COOLDOWN
        seconds (default 5 min). If we're still flapping after that, most
        likely a real hardware/SLAM bug — keep triggering but log warning.
        """
        xy_lim = self._drift_watchdog_xy_limit
        v_lim  = self._drift_watchdog_v_limit
        interval = self._drift_watchdog_interval
        logger.info(
            "drift_watchdog: enabled, interval=%.0fs, |xy|<%.0fm, |v|<%.1fm/s",
            interval, xy_lim, v_lim,
        )
        stop_event = stop_event or self._stop_event
        while not stop_event.wait(interval):
            try:
                diverged, x, y, z, v, invalid = self._drift_current_odom_divergence()
                if not diverged:
                    continue
                # Divergence detected
                now = time.time()
                since = now - self._drift_last_restart_ts
                if since < self._drift_watchdog_cooldown:
                    logger.warning(
                        "drift_watchdog: still diverged (xy=%.0f,%.0f v=%.1f) but "
                        "cooldown (%.0fs) not elapsed — skipping restart",
                        x, y, v, self._drift_watchdog_cooldown - since,
                    )
                    continue
                logger.error(
                    "drift_watchdog: IEKF DIVERGED xy=(%.0f,%.0f) z=%.0f v=%.1f — "
                    "restarting SLAM services + session companions",
                    x, y, z, v,
                )
                if stop_event.is_set():
                    return
                self._drift_restart_do_restart(
                    xy=max(x, y, z), y_abs=y, v=v, stop_event=stop_event
                )
                self._drift_last_restart_ts = time.time()
                self._drift_restart_count += 1
            except Exception as e:
                logger.exception("drift_watchdog tick failed: %s", e)

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
        stop_event = stop_event or self._stop_event
        if stop_event.is_set():
            return
        try:
            from core.service_manager import get_service_manager
            svc = get_service_manager()
        except Exception as e:
            logger.error("drift_watchdog: service_manager unavailable: %s", e)
            return

        # Snapshot runtime state BEFORE stopping services. The nav profile keeps
        # SLAM/localizer alive even while the mission session is idle so App/Web
        # readiness and map state remain fresh.
        mode = self._session_mode
        service_names = (
            "slam",
            "slam_pgo",
            "localizer",
            "super_lio",
            "super_lio_relocation",
        )
        running_before = {name: False for name in service_names}
        for name in service_names:
            try:
                running_before[name] = bool(svc.is_running(name))
            except Exception as e:
                logger.warning(
                    "drift_watchdog: failed to read service state for %s: %s",
                    name,
                    e,
                )

        # Black-box dump must happen BEFORE svc.stop, otherwise the
        # localization_status / gnss / odom feeds dry up the moment slam exits
        # and we lose the very tail-end of the divergence we want to study.
        # Note: ``Path`` in this module imports the ROS message type, not
        # pathlib.Path; the recorder returns a pathlib.Path so we leave this
        # untyped to avoid the symbol clash.
        dump_path = None
        try:
            dump_path = self._blackbox.dump(
                reason="drift_watchdog",
                metadata={
                    "xy": float(xy),
                    "y_abs": float(y_abs),
                    "v": float(v),
                    "session_mode": mode,
                    "restart_count": self._drift_restart_count + 1,
                },
            )
        except Exception as e:
            logger.warning("drift_watchdog: blackbox dump failed (continuing): %s", e)

        try:
            svc.stop(
                "slam",
                "slam_pgo",
                "localizer",
                "super_lio",
                "super_lio_relocation",
            )
        except Exception as e:
            logger.warning("drift_watchdog: svc.stop failed (continuing): %s", e)

        # Clear stale odom — prevents downstream modules acting on trillion-meter
        # coordinates while slam re-initialises.
        with self._state_lock:
            self._odom = {}
            self._odom_timestamps.clear()

        evt: dict = {
            "type": "slam_drift",
            "level": "error",
            "xy": max(xy, y_abs),
            "v": v,
            "action": "slam_restart",
            "count": self._drift_restart_count + 1,
        }
        if dump_path is not None:
            evt["dump_path"] = str(dump_path)
        self.push_event(evt)

        # Re-ensure based on session mode. Idle restores any localization
        # services that were already running before this watchdog reset.
        if stop_event.wait(self._drift_restart_delay_s):
            return
        try:
            restart_services: list[str] = []
            if running_before.get("super_lio_relocation"):
                restart_services = ["lidar", "super_lio_relocation"]
            elif running_before.get("super_lio"):
                restart_services = ["lidar", "super_lio"]
            elif mode in ("mapping", "exploring"):
                restart_services = ["slam", "slam_pgo"]
            elif mode == "navigating":
                restart_services = ["slam", "localizer"]
            else:
                restart_services = [
                    name for name in service_names if running_before.get(name)
                ]
                if (
                    "localizer" in restart_services or "slam_pgo" in restart_services
                ) and "slam" not in restart_services:
                    restart_services.insert(0, "slam")

            if restart_services and not stop_event.is_set():
                svc.ensure(*restart_services)
                svc.wait_ready(*restart_services, timeout=10.0)
            logger.info(
                "drift_watchdog: restart complete (mode=%s, restored=%s)",
                mode,
                ",".join(restart_services) if restart_services else "none",
            )
        except Exception as e:
            logger.error("drift_watchdog: ensure failed: %s", e)

    def _saved_map_loader_loop(
        self, stop_event: threading.Event | None = None
    ) -> None:
        """Watch active/map.pcd symlink; push saved_map ONCE per change.

        Previous behaviour: push every 10 s for "new clients". But the
        frontend rebuilds the entire 80k-point Three.js mesh on every
        saved_map event → satellite flicker / GPU churn. Better: push only
        when the active map actually changes, and cache `_last_saved_map`
        so new SSE connections get it once from the snapshot.
        """
        MAX_SEND = 80_000
        last_target = None
        stop_event = stop_event or self._stop_event
        while not stop_event.is_set():
            try:
                target = active_map_name()
                if target != last_target:
                    last_target = target
                    pcd_path = map_dir_for(target) / "map.pcd" if target else None
                    if pcd_path and pcd_path.is_file():
                        pts = self._load_pcd_xyz(str(pcd_path))
                        if pts is not None and len(pts) > 0:
                            if len(pts) > MAX_SEND:
                                idx = np.random.choice(len(pts), MAX_SEND, replace=False)
                                pts = pts[idx]
                            flat = pts[:, :3].astype(np.float32).flatten().tolist()
                            count = len(pts)
                            self._last_saved_map_event = {
                                "type": "saved_map", "points": flat, "count": count,
                            }
                            self.push_event(self._last_saved_map_event)
                            logger.info(
                                "saved_map loader: loaded %s (%d pts) — pushed once",
                                target, count)
            except Exception as e:
                logger.debug("saved_map loader error: %s", e)
            stop_event.wait(5.0)

    @staticmethod
    def _load_pcd_xyz(path: str) -> np.ndarray | None:
        """Minimal PCD XYZ loader — supports ascii and binary little-endian."""
        try:
            with open(path, "rb") as f:
                head_bytes = b""
                while b"DATA" not in head_bytes.split(b"\n")[-1].upper() if b"DATA" in head_bytes else True:
                    line = f.readline()
                    if not line:
                        break
                    head_bytes += line
                    if line.upper().startswith(b"DATA"):
                        break
                header = head_bytes.decode("ascii", errors="ignore")
                fields = []
                sizes: list[int] = []
                count = 0
                fmt = "ascii"
                for ln in header.splitlines():
                    ln = ln.strip()
                    if ln.startswith("FIELDS"):
                        fields = ln.split()[1:]
                    elif ln.startswith("SIZE"):
                        sizes = [int(x) for x in ln.split()[1:]]
                    elif ln.startswith("POINTS"):
                        count = int(ln.split()[1])
                    elif ln.startswith("DATA"):
                        fmt = ln.split()[1].lower()
                if not fields or "x" not in fields or "y" not in fields:
                    return None
                xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z") if "z" in fields else -1
                if fmt == "ascii":
                    pts = []
                    for ln in f.read().decode(errors="ignore").splitlines():
                        parts = ln.split()
                        if len(parts) < len(fields):
                            continue
                        try:
                            x, y = float(parts[xi]), float(parts[yi])
                            z = float(parts[zi]) if zi >= 0 else 0.0
                            pts.append((x, y, z))
                        except ValueError:
                            pass
                    return np.array(pts, dtype=np.float32) if pts else None
                if fmt == "binary":
                    if not sizes:
                        sizes = [4] * len(fields)
                    stride = sum(sizes)
                    raw = np.frombuffer(f.read(), dtype=np.uint8)
                    n = count or (len(raw) // stride)
                    raw = raw[: n * stride].reshape(n, stride)
                    offsets = [sum(sizes[:i]) for i in range(len(sizes))]
                    out = np.zeros((n, 3), dtype=np.float32)
                    out[:, 0] = np.frombuffer(raw[:, offsets[xi]:offsets[xi]+4].tobytes(), dtype=np.float32)
                    out[:, 1] = np.frombuffer(raw[:, offsets[yi]:offsets[yi]+4].tobytes(), dtype=np.float32)
                    if zi >= 0:
                        out[:, 2] = np.frombuffer(raw[:, offsets[zi]:offsets[zi]+4].tobytes(), dtype=np.float32)
                    finite = np.isfinite(out).all(axis=1)
                    return out[finite]
        except Exception as e:
            logger.debug("PCD load failed %s: %s", path, e)
            return None
        return None

    def stop(self) -> None:
        self._stop_event.set()
        server = self._server
        if server is not None:
            try:
                server.should_exit = True
            except Exception:
                logger.debug(
                    "GatewayModule: failed to signal uvicorn shutdown",
                    exc_info=True,
                )

        current = threading.current_thread()
        for attr_name in (
            "_server_thread",
            "_client_http_prewarm_thread",
            "_saved_map_loader_thread",
            "_drift_watchdog_thread",
            "_brainstem_health_refresh_thread",
        ):
            thread = getattr(self, attr_name, None)
            if thread is None:
                continue
            if thread is current:
                setattr(self, attr_name, None)
                continue
            if thread.is_alive():
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(
                        "GatewayModule: %s did not stop within timeout",
                        thread.name,
                    )
                    continue
            setattr(self, attr_name, None)
        super().stop()

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        self._map_mgr = modules.get("MapManagerModule")
        self._all_modules = modules
        # WebRTCStreamModule handles its own ICE/SDP dance; we just forward
        # POST /api/v1/webrtc/offer to it.  See the route below.
        self._webrtc = modules.get("WebRTCStreamModule")
        self._frontier_explorer = next(
            (m for m in modules.values()
             if m.__class__.__name__ == "WavefrontFrontierExplorer"),
            None,
        )
        self._tare_explorer = next(
            (m for m in modules.values()
             if m.__class__.__name__ == "TAREExplorerModule"),
            None,
        )
        self._tagged_loc_module = next(
            (m for m in modules.values()
             if m.__class__.__name__ == "TaggedLocationsModule"),
            None,
        )

        # SwapManager is created after on_system_modules() returns, so it
        # cannot be discovered here.  It is injected later by Blueprint.build()
        # via _set_swap_manager().
        self._swap_manager = None

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
        if self._frontier_explorer is not None:
            return "frontier"
        if self._tare_explorer is not None:
            return "tare"
        return "none"

    def _explorer_available(self) -> bool:
        return self._explorer_backend() != "none"

    def _explorer_unavailable_detail(self) -> dict[str, Any]:
        return {
            "reason": "explorer_backend_not_running",
            "required_profile": "explore_or_tare_explore",
            "supported_profiles": ["explore", "tare_explore"],
            "action": (
                "restart LingTu with the explore or tare_explore profile before "
                "starting exploration"
            ),
        }

    def reconfigure_backend(
        self,
        category: str,
        backend: str,
        **config: Any,
    ) -> dict[str, Any]:
        if category in _MOTION_BACKEND_CATEGORIES:
            nav = self._all_modules.get("NavigationModule")
            state = _navigation_state(nav)
            if state != "IDLE":
                return {
                    "ok": False,
                    "category": category,
                    "requested_backend": backend,
                    "reason": "motion_backend_switch_requires_idle",
                    "navigation_state": state or "UNKNOWN",
                }

        for module_name in _BACKEND_RECONFIGURE_TARGETS.get(category, ()):
            module = self._all_modules.get(module_name)
            reconfigure = getattr(module, "reconfigure_backend", None)
            if callable(reconfigure):
                return reconfigure(category, backend, **config)

        return super().reconfigure_backend(category, backend, **config)

    # ------------------------------------------------------------------
    # Driver swap — delegates to SwapManager when registered
    # ------------------------------------------------------------------

    def _on_driver_swap(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Swap the active driver backend at runtime.

        Delegates to ``SwapManager.swap(driver_name, **config)`` if a
        SwapManager is registered in the system.  When no SwapManager is
        present the response reports ``success=False`` with a descriptive
        message so callers can react gracefully (e.g. show a ui banner).
        """
        swap = self._swap_manager
        if swap is None:
            return {
                "success": False,
                "message": "SwapManager not available — driver swap requires a running SwapManager",
                "swap_time_ms": 0.0,
            }

        driver_name = str(payload.get("driver", ""))
        if not driver_name:
            return {
                "success": False,
                "message": "Missing required field: driver",
                "swap_time_ms": 0.0,
            }

        config = payload.get("config") or {}
        if not isinstance(config, dict):
            return {
                "success": False,
                "message": "config must be a dict",
                "swap_time_ms": 0.0,
            }

        try:
            t0 = time.monotonic()
            result = swap.swap(driver_name, **config)
            elapsed = (time.monotonic() - t0) * 1000.0  # ms
            if isinstance(result, dict) and not result.get("success", True):
                return {
                    "success": False,
                    "message": result.get("message", "swap failed"),
                    "swap_time_ms": elapsed,
                    "detail": result,
                }
            return {
                "success": True,
                "message": f"Swapped to {driver_name}",
                "swap_time_ms": elapsed,
            }
        except Exception as e:
            logger.exception("Driver swap to %s failed", driver_name)
            return {
                "success": False,
                "message": str(e),
                "swap_time_ms": 0.0,
            }

    def _coerce_explorer_result(self, result: Any) -> Any:
        if isinstance(result, str):
            try:
                return json.loads(result)
            except Exception as e:
                logger.debug("_coerce_explorer_result JSON parse failed: %s", e)
                return result
        return result

    def _begin_exploration(self) -> Any:
        if self._frontier_explorer is not None:
            return self._coerce_explorer_result(
                self._frontier_explorer.begin_exploration()
            )
        if self._tare_explorer is not None:
            starter = getattr(
                self._tare_explorer,
                "start_tare_exploration",
                None,
            )
            if starter is None:
                starter = getattr(self._tare_explorer, "begin_exploration", None)
            if starter is None:
                raise RuntimeError("TARE explorer has no start method")
            return self._coerce_explorer_result(starter())
        raise RuntimeError("explorer_not_running")

    def _end_exploration(self) -> Any:
        if self._frontier_explorer is not None:
            return self._coerce_explorer_result(
                self._frontier_explorer.end_exploration()
            )
        if self._tare_explorer is not None:
            stopper = getattr(
                self._tare_explorer,
                "stop_tare_exploration",
                None,
            )
            if stopper is None:
                stopper = getattr(self._tare_explorer, "end_exploration", None)
            if stopper is None:
                raise RuntimeError("TARE explorer has no stop method")
            return self._coerce_explorer_result(stopper())
        raise RuntimeError("explorer_not_running")

    def _tare_status_payload(self) -> dict[str, Any]:
        if self._tare_explorer is None:
            return {}
        raw_status: Any = {}
        status_fn = getattr(self._tare_explorer, "get_tare_status", None)
        if status_fn is not None:
            try:
                raw_status = self._coerce_explorer_result(status_fn())
            except Exception as exc:
                raw_status = {"error": str(exc)}
        if not isinstance(raw_status, dict):
            raw_status = {"raw": raw_status}
        return {
            "status": raw_status,
            "stats": self._last_tare_stats or {},
        }

    def _exploration_status_payload(self) -> dict[str, Any]:
        backend = self._explorer_backend()
        readiness = self._exploration_start_readiness()
        if backend == "none":
            return {
                "available": False,
                "backend": "none",
                "exploring": False,
                "frontier_count": 0,
                **readiness,
                **self._explorer_unavailable_detail(),
            }
        if backend == "frontier":
            health = {}
            if hasattr(self._frontier_explorer, "health"):
                try:
                    health = self._frontier_explorer.health() or {}
                except Exception as exc:
                    health = {"error": str(exc)}
            frontier_count = 0
            if isinstance(health, dict):
                try:
                    frontier_count = int(health.get("frontier_count", 0) or 0)
                except (TypeError, ValueError):
                    frontier_count = 0
            return {
                "available": True,
                "backend": "frontier",
                "exploring": self._exploring,
                "frontier_count": frontier_count,
                **readiness,
            }

        tare = self._tare_status_payload()
        tare_status = tare.get("status") if isinstance(tare, dict) else {}
        tare_started = (
            bool(tare_status.get("started"))
            if isinstance(tare_status, dict)
            else False
        )
        return {
            "available": True,
            "backend": "tare",
            "exploring": bool(self._exploring or tare_started),
            "frontier_count": 0,
            "tare": tare,
            "supervisor": self._exploration_supervisor_state or {},
            **readiness,
        }

    def _exploration_start_readiness(self) -> dict[str, Any]:
        blockers: list[str] = []
        advisories: list[str] = []
        navigation: dict[str, Any] = {}
        exploration_ignored_nav_blockers = {"navigation_session_inactive"}

        if not self._explorer_available():
            blockers.append("explorer_backend_not_running")
        if self._session_pending:
            blockers.append("session_transition_pending")
        if self._exploring:
            blockers.append("exploration_already_active")
        if str(self._session_mode or "idle").lower() != "idle":
            blockers.append("session_not_idle")

        try:
            from gateway.services.runtime_status import build_navigation_status

            navigation = build_navigation_status(self)
            readiness = navigation.get("readiness") or {}
            nav_blockers = readiness.get("blockers") or []
            nav_advisories = readiness.get("advisories") or []
            if isinstance(nav_blockers, list):
                blockers.extend(
                    str(code)
                    for code in nav_blockers
                    if str(code) not in exploration_ignored_nav_blockers
                )
            if isinstance(nav_advisories, list):
                advisories.extend(str(code) for code in nav_advisories)
            if (
                not blockers
                and not bool(readiness.get("can_execute_autonomy", False))
                and not nav_blockers
            ):
                blockers.append("navigation_not_ready")
        except Exception as e:
            logger.debug("_exploration_start_readiness: build_navigation_status failed: %s", e)
            blockers.append("navigation_status_unavailable")

        blockers = list(dict.fromkeys(blockers))
        advisories = list(dict.fromkeys(advisories))
        return {
            "can_start": bool(self._explorer_available()) and not blockers,
            "blockers": blockers,
            "advisories": advisories,
            "navigation": {
                "state": navigation.get("state"),
                "can_accept_goal": navigation.get("can_accept_goal"),
                "readiness": navigation.get("readiness") or {},
            },
        }

    # -- teleop helpers (delegate to TeleopModule) ---------------------------

    @property
    def _teleop_active(self) -> bool:
        """Proxy for teleop active state (lives in TeleopModule now)."""
        tm = self._teleop_module
        return tm._active if tm is not None else False

    def configure_teleop(
        self,
        max_speed: float,
        max_yaw: float,
        release_timeout: float,
    ) -> None:
        """Called by TeleopModule during setup() — config stored for display."""
        self._teleop_max_speed = max_speed
        self._teleop_max_yaw   = max_yaw
        self._teleop_release_timeout = release_timeout

    def push_jpeg(self, jpeg_bytes: bytes) -> None:
        """Called by TeleopModule when a new camera frame is ready."""
        with self._jpeg_lock:
            self._latest_jpeg = jpeg_bytes
            self._latest_jpeg_seq += 1

    # -- Module subscription callbacks -------------------------------------

    def _on_odometry(self, odom: Odometry) -> None:
        # SlamBridge now applies map→odom TF on its publish side, so the
        # odom that reaches Gateway is already in map frame. Re-applying
        # the TF here would double-transform and push the frontend cursor
        # ~2m off from where it actually is (observed: 顶栏 -3.42,-0.46 vs
        # PCT start -1.27,-2.24). Pass through as-is.
        d = {
            "x":  float(odom.x),
            "y":  float(odom.y),
            "z":  float(getattr(odom, "z", 0.0)),
            "yaw": float(getattr(odom, "yaw", 0.0)),
            "vx": odom.twist.linear.x  if odom.twist else 0.0,
            "wz": odom.twist.angular.z if odom.twist else 0.0,
            "frame_id": str(getattr(odom, "frame_id", "") or "unknown"),
            "child_frame_id": str(getattr(odom, "child_frame_id", "") or "body"),
            "ts": odom.ts,
        }
        numeric_fields = ("x", "y", "z", "yaw", "vx", "wz", "ts")
        invalid_fields = [
            name for name in numeric_fields
            if not math.isfinite(float(d.get(name, 0.0)))
        ]
        if invalid_fields:
            invalid_odometry = {
                "reason": "non_finite_odometry",
                "fields": invalid_fields,
                "ts": time.time(),
            }
            with self._state_lock:
                self._last_invalid_odometry = invalid_odometry
            logger.warning(
                "GatewayModule: quarantined non-finite odometry fields=%s",
                invalid_fields,
            )
            return
        with self._state_lock:
            self._last_invalid_odometry = None
            self._odom = d
            # Track for Hz calculation (sliding window of 20)
            self._odom_timestamps.append(time.time())
            if len(self._odom_timestamps) > 20:
                self._odom_timestamps.pop(0)
        self._blackbox.record("odom", d)
        self.push_event({"type": "odometry", "data": d})

        # Push slam_status at ~1Hz (every 10 odometry frames)
        self._slam_status_throttle += 1
        if self._slam_status_throttle % 10 == 0:
            with self._map_cloud_lock:
                map_pts = len(self._map_points) if self._map_points is not None else 0
            self.push_event({
                "type": "slam_status",
                "mode": self._get_slam_profile(),
                "slam_hz": self._get_slam_hz_cached(),
                "map_points": map_pts,
                "degeneracy_count": getattr(self, "_degeneracy_count", 0),
            })

    @staticmethod
    def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
        """Voxel grid downsample: keep one point per cell (by first occurrence)."""
        if len(pts) == 0:
            return pts
        voxels = np.floor(pts[:, :3] / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxels, axis=0, return_index=True)
        return pts[np.sort(unique_idx)]

    def _on_icp_quality(self, q: float) -> None:
        try:
            self._icp_quality = float(q)
        except Exception as e:
            logger.debug("_on_icp_quality: failed to convert %r: %s", q, e)

    # ── Last-known-pose persistence (auto-relocalize on session/start) ────
    # Keeps a tiny JSON snapshot of the last *successful* relocalize call so
    # that a daemon restart doesn't force the operator to Shift+click again.
    # Only written when the user explicitly relocalized and the localizer
    # reported success; never overwritten by raw odometry (which can drift).
    _LAST_POSE_PATH = os.path.expanduser("~/.lingtu/last_nav_pose.json")
    _LAST_POSE_MAX_AGE_S = 7 * 24 * 3600  # 1 week

    def _persist_last_nav_pose(
        self, map_name: str, x: float, y: float, yaw: float,
        quality: float | None,
    ) -> None:
        """Atomically snapshot the last successful relocalize pose to disk."""
        try:
            path = self._LAST_POSE_PATH
            os.makedirs(os.path.dirname(path), exist_ok=True)
            payload = {
                "map_name": map_name,
                "x": float(x), "y": float(y), "yaw": float(yaw),
                "quality": float(quality) if quality is not None else None,
                "saved_at": time.time(),
            }
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(payload, f)
            os.replace(tmp, path)
            logger.info("gateway: persisted last_nav_pose map=%s (%.2f, %.2f, yaw=%.2f)",
                        map_name, x, y, yaw)
        except Exception as e:
            logger.warning("gateway: persist last_nav_pose failed: %s", e)

    def _load_last_nav_pose(self, map_name: str) -> dict | None:
        """Return persisted pose if map matches and snapshot is fresh, else None."""
        try:
            path = self._LAST_POSE_PATH
            if not os.path.isfile(path):
                return None
            with open(path, encoding="utf-8") as f:
                d = json.load(f)
            if d.get("map_name") != map_name:
                return None
            age = time.time() - float(d.get("saved_at", 0.0))
            if age > self._LAST_POSE_MAX_AGE_S:
                logger.info("gateway: last_nav_pose too old (%.1fh), ignoring", age / 3600)
                return None
            return d
        except Exception as e:
            logger.debug("gateway: load last_nav_pose failed: %s", e)
            return None

    def _spawn_auto_relocalize(self, map_name: str) -> None:
        """Fire-and-forget: if we have a persisted pose for this map, replay
        it as a /relocalize service call ~2s after localizer comes up.

        Runs in a daemon thread so session/start returns immediately — the
        operator sees mode=navigating right away, and the ICP converges in
        the background using the last known pose instead of (0,0,0).
        """
        d = self._load_last_nav_pose(map_name)
        if d is None:
            return
        x, y, yaw = d["x"], d["y"], d["yaw"]

        def _worker() -> None:
            time.sleep(2.5)  # give localizer time to finish loading static map
            try:
                pcd_path = str(map_dir_for(map_name) / "map.pcd")
                if not os.path.isfile(pcd_path):
                    logger.warning("auto-relocalize: map pcd missing: %s", pcd_path)
                    return
                env = (
                    "source /opt/ros/humble/setup.bash && "
                    "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
                    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                )
                cmd = (
                    env
                    + f"ros2 service call /nav/relocalize interface/srv/Relocalize "
                      f"\"{{pcd_path: '{pcd_path}', x: {x}, y: {y}, z: 0.0, "
                      f"yaw: {yaw}, pitch: 0.0, roll: 0.0}}\""
                )
                r = subprocess.run(
                    ["bash", "-c", cmd],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=20,
                )
                ok = "success=True" in r.stdout
                logger.info("auto-relocalize: map=%s pose=(%.2f,%.2f,yaw=%.2f) ok=%s",
                            map_name, x, y, yaw, ok)
            except Exception as e:
                logger.warning("auto-relocalize worker failed: %s", e)

        threading.Thread(
            target=_worker, daemon=True, name="gateway-auto-reloc",
        ).start()

    def _on_map_odom_tf(self, tf: dict) -> None:
        """Cache localizer's map→odom transform as a 4x4 matrix.

        Applied to Fast-LIO2's odom-frame map_cloud before SSE push so the
        frontend renders it in the same frame as saved_map (which is already
        in map frame).  Until the localizer converges (quality > 0), this
        stays identity so the cloud is still visible, just not yet aligned.
        """
        if not tf or not tf.get("valid", False):
            return
        self._blackbox.record("tf", tf)
        try:
            tx, ty, tz = float(tf["tx"]), float(tf["ty"]), float(tf["tz"])
            qx, qy, qz, qw = (float(tf["qx"]), float(tf["qy"]),
                              float(tf["qz"]), float(tf["qw"]))
            # Quaternion → 3x3 rotation (Hamilton convention, right-handed)
            xx, yy, zz = qx * qx, qy * qy, qz * qz
            xy, xz, yz = qx * qy, qx * qz, qy * qz
            wx, wy, wz = qw * qx, qw * qy, qw * qz
            R = np.array([
                [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
                [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
                [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
            ], dtype=np.float64)
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            self._T_map_odom = T
            self._has_map_odom_tf = True
        except Exception as e:
            logger.debug("gateway: _on_map_odom_tf parse failed: %s", e)

    # ── Session state helpers ─────────────────────────────────────────────

    def _session_detect_current_mode(self) -> tuple[str, str | None]:
        """Reflect what's actually running into session state (truth vs claim)."""
        try:
            from core.service_manager import get_service_manager
            svc = get_service_manager()
            status = svc.status(
                "slam",
                "slam_pgo",
                "localizer",
                "super_lio",
                "super_lio_relocation",
            )
            slam_active = status.get("slam") in ("running", "active")
            pgo_active = status.get("slam_pgo") in ("running", "active")
            loc_active = status.get("localizer") in ("running", "active")
            super_lio_active = status.get("super_lio") in ("running", "active")
            super_lio_relocation_active = status.get(
                "super_lio_relocation"
            ) in ("running", "active")
            if super_lio_relocation_active:
                return "navigating", self._session_active_map_name()
            if super_lio_active:
                if self._exploring:
                    return "exploring", None
                if self._session_mode in ("mapping", "navigating"):
                    return self._session_mode, (
                        self._session_active_map_name()
                        if self._session_mode == "navigating"
                        else None
                    )
                return "mapping", None
            if loc_active and slam_active:
                return "navigating", self._session_active_map_name()
            if pgo_active and slam_active:
                # Autonomous exploration is a mapping session with frontier
                # explorer actively driving goal poses.
                if self._exploring:
                    return "exploring", None
                return "mapping", None
            if self._exploring and self._session_uses_external_slam_none():
                return "exploring", None
            return "idle", None
        except Exception as e:
            logger.warning("_session_detect_current_mode: service_manager lookup failed: %s", e)
            if self._exploring and self._session_uses_external_slam_none():
                return "exploring", None
            return "idle", None

    def _session_uses_external_slam_none(self) -> bool:
        """True when an active session intentionally uses external sim odometry."""
        profile = str(self._session_slam_profile or "").strip().lower()
        return (
            profile == "none"
            and self._session_mode in ("mapping", "navigating", "exploring")
            and self._explorer_available()
        )

    def _session_active_map_name(self) -> str | None:
        """Read active map symlink target."""
        return active_map_name()

    def _session_snapshot(self) -> dict:
        """Full session state for GET /session + SSE."""
        active_map = self._session_active_map_name()
        has_tomogram = False
        has_pcd = False
        if active_map:
            base = map_dir_for(active_map)
            has_pcd = (base / "map.pcd").is_file()
            has_tomogram = (base / "tomogram.pickle").is_file()
        icp = self._icp_quality
        localization_status = self._localization_status or {}
        slam_profile = self._get_slam_profile()
        backend = str(
            localization_status.get("backend")
            or slam_profile
            or self._session_slam_profile
            or "stopped"
        ).lower()
        pose_fresh, pose_freshness = classify_pose_freshness(localization_status)
        algorithm_healthy = localizer_algorithm_healthy(localization_status, icp)
        confidence_ok = pose_fresh is not False
        loc_ready = (
            algorithm_healthy
            and confidence_ok
        )
        capability_defaults = backend_capability_defaults(backend)
        relocalization_supported = localization_status.get("relocalization_supported")
        if relocalization_supported is None:
            relocalization_supported = capability_defaults["relocalization_supported"]
        saved_map_relocalization_supported = localization_status.get(
            "saved_map_relocalization_supported"
        )
        if saved_map_relocalization_supported is None:
            saved_map_relocalization_supported = relocalization_supported
        restart_recovery_supported = localization_status.get(
            "restart_recovery_supported"
        )
        if restart_recovery_supported is None:
            restart_recovery_supported = capability_defaults[
                "restart_recovery_supported"
            ]
        recovery_method = localization_status.get("recovery_method")
        if not recovery_method:
            recovery_method = capability_defaults["recovery_method"]
        map_save_supported = localization_status.get("map_save_supported")
        if map_save_supported is None:
            map_save_supported = backend in {"super_lio", "fastlio2", "slam"}
        map_save_source = localization_status.get("map_save_source")
        if map_save_source is None and backend == "super_lio":
            map_save_source = "live_map_cloud_snapshot"
        if map_save_source is None and backend == "super_lio_relocation":
            map_save_source = "active_map"
        # Gate which transitions are allowed from current state.
        idle = self._session_mode == "idle" and not self._session_pending
        can_start_mapping = idle
        can_start_navigating = (
            idle
            and active_map is not None
            and has_pcd
            and has_tomogram
        )
        explorer_backend = self._explorer_backend()
        explorer_available = explorer_backend != "none"
        explorer_detail = (
            {} if explorer_available else self._explorer_unavailable_detail()
        )
        explorer_unavailable_reason = (
            None if explorer_available else explorer_detail.get("reason")
        )
        explorer_required_profile = (
            None if explorer_available else explorer_detail.get("required_profile")
        )
        safety = safety_summary(self._safety)
        safety_clear = safety_clear_for_motion(self._safety)
        exploration_blockers: list[str] = []
        if not explorer_available:
            exploration_blockers.append("explorer_backend_not_running")
        if self._session_pending:
            exploration_blockers.append("session_transition_pending")
        if self._exploring:
            exploration_blockers.append("exploration_already_active")
        if str(self._session_mode or "idle").lower() != "idle":
            exploration_blockers.append("session_not_idle")
        if self._mode == "estop":
            exploration_blockers.append("estop_active")
        if not safety_clear:
            exploration_blockers.append(SAFETY_STOP_BLOCKER)
        if self._odom is None:
            exploration_blockers.append("odometry_missing")
        localization_state = str(localization_status.get("state") or "").strip().lower()
        if localization_state in {"degraded", "lost", "relocalizing", "initializing"}:
            exploration_blockers.append(f"localization_{localization_state}")
        recovery_signal = str(
            localization_status.get("recovery_signal") or ""
        ).strip().upper()
        if recovery_signal not in {"", "NONE", "RECOVERED"}:
            exploration_blockers.append("localization_recovery_active")
        if pose_fresh is False and algorithm_healthy:
            exploration_blockers.append("pose_stale")
        exploration_blockers = list(dict.fromkeys(exploration_blockers))
        can_start_exploring = not exploration_blockers
        can_end = self._session_mode != "idle" and not self._session_pending
        return {
            "mode": self._session_mode,
            "slam_profile": slam_profile,
            "localization_backend": backend,
            "health_source": localization_status.get("health_source"),
            "active_map": active_map,
            "map_has_pcd": has_pcd,
            "map_has_tomogram": has_tomogram,
            "since": self._session_since,
            "pending": self._session_pending,
            "error": self._session_error,
            "icp_quality": icp,
            "localizer_ready": loc_ready,
            "localizer_algorithm_healthy": algorithm_healthy,
            "pose_fresh": pose_fresh,
            "pose_freshness": pose_freshness,
            "map_state": localization_status.get("map_state"),
            "map_save_supported": bool(map_save_supported),
            "map_save_source": map_save_source,
            "relocalization_supported": bool(relocalization_supported),
            "saved_map_relocalization_supported": bool(
                saved_map_relocalization_supported
            ),
            "restart_recovery_supported": bool(restart_recovery_supported),
            "recovery_method": recovery_method,
            "relocalization_state": localization_status.get("relocalization_state"),
            "recovery_signal": localization_status.get("recovery_signal"),
            "recovery_action": localization_status.get("recovery_action"),
            "can_start_mapping": can_start_mapping,
            "can_start_navigating": can_start_navigating,
            "can_start_exploring": can_start_exploring,
            "exploration_blockers": exploration_blockers,
            "safety_clear": safety_clear,
            "safety": safety,
            "can_end": can_end,
            "explorer_backend": explorer_backend,
            "explorer_available": explorer_available,
            "explorer_unavailable_reason": explorer_unavailable_reason,
            "explorer_required_profile": explorer_required_profile,
        }

    def _on_saved_map(self, cloud: PointCloud2) -> None:
        """DDS-stream saved-map — intentionally ignored.

        Localizer publishes a refined saved_map every tick (~10 Hz). That
        previously got downsampled and re-pushed every 10 frames, but the
        frontend rebuilds the whole 80k-pt mesh on each event → flicker.
        The on-disk PCD pushed once by _saved_map_loader_loop is stable
        enough for visualization — the localizer's runtime refinement
        doesn't change the底图 meaningfully for the operator view.
        """
        return

    def _on_map_cloud(self, cloud: PointCloud2) -> None:
        """Mode-aware cloud handling.

        mapping / exploring  → accumulate with voxel dedup (growing global map)
        navigating           → replace every frame, no accumulation
            Rationale: in navigating mode the map→odom TF keeps micro-adjusting
            while ICP converges. Accumulating odom-frame points then transforming
            on every send makes the *whole* accumulated cloud drift as TF shifts,
            producing the "flying" ghost effect. Replacing per frame makes the
            live scan track the robot cleanly, while saved_map carries the
            stable底图 layer.
        idle                 → keep latest, don't grow
        """
        self._map_cloud_count += 1
        pts = cloud.points  # (N, 3) float32
        if pts is None or len(pts) == 0:
            return
        pts = pts[:, :3].astype(np.float32)
        # Reject outliers: SLAM divergence can spit out points at 1e7+ meters,
        # and NaN/Inf can leak from uninitialized frames. Keep only plausible
        # world-frame points within a 500m box around origin.
        finite = np.isfinite(pts).all(axis=1)
        bounded = (np.abs(pts) < 500.0).all(axis=1)
        pts = pts[finite & bounded]
        if len(pts) == 0:
            return
        mode = self._session_mode
        with self._map_cloud_lock:
            if mode in ("navigating", "idle") or self._map_points is None:
                # Replace — latest scan only, no accumulation
                self._map_points = pts
            else:
                # mapping / exploring — grow the global map via voxel dedup
                combined = np.concatenate([self._map_points, pts], axis=0)
                combined = self._voxel_downsample(combined, self._map_voxel_size)
                self._map_points = combined
                # Increment per-voxel hit count using this frame's unique voxels
                # (pre-accumulation). Loop over ~500 unique voxels per frame
                # costs <1ms in Python — negligible vs numpy concat cost above.
                frame_keys = np.unique(self._pack_voxel_keys(pts))
                for k in frame_keys:
                    self._voxel_hits[int(k)] = self._voxel_hits.get(int(k), 0) + 1

                # GC: prevent unbounded dict growth on long mapping runs (4h+).
                # 当 dict 超 200k 条时, 淘汰 hit==1 的瞬态 voxel (保留 >=2 的真实观测)。
                # 典型 8k voxel/min × 60min = 480k, 建图 25+min 就会触发一次。
                if len(self._voxel_hits) > 200_000:
                    before = len(self._voxel_hits)
                    self._voxel_hits = {
                        k: c for k, c in self._voxel_hits.items() if c >= 2
                    }
                    logger.info(
                        "voxel_hits GC: %d → %d entries (dropped hit==1)",
                        before, len(self._voxel_hits),
                    )

        # Push to SSE at ~0.5Hz (every 2 frames) — often enough to look
        # "live accumulating" without flooding the browser.
        if self._map_cloud_count % 2 != 0:
            return

        with self._map_cloud_lock:
            pts_all = self._map_points
        if pts_all is None:
            return

        # Adaptive voxel: if still too many points after base voxel, grow
        # the voxel size so the send list stays in [30k, 60k] range and
        # stable across frames (same voxel grid → same point set).
        MAX_SEND = 60_000
        if len(pts_all) > MAX_SEND:
            scale = (len(pts_all) / MAX_SEND) ** (1 / 3)
            coarse_voxel = self._map_voxel_size * max(1.0, scale)
            pts_send = self._voxel_downsample(pts_all, coarse_voxel)
            # Final hard cap — deterministic stride, not random
            if len(pts_send) > MAX_SEND:
                stride = max(1, len(pts_send) // MAX_SEND)
                pts_send = pts_send[::stride][:MAX_SEND]
        else:
            pts_send = pts_all

        # Dynamic-obstacle filter — drop voxels hit fewer than min_hits times
        # in mapping / exploring modes. 动态物体 (行人) 只在几帧内命中一个
        # voxel,墙/家具长期命中数十帧。阈值一卡残影即散。
        if (
            mode in ("mapping", "exploring")
            and self._voxel_min_hits > 1
            and self._voxel_hits
            and len(pts_send) > 0
        ):
            stable_arr = np.fromiter(
                (k for k, c in self._voxel_hits.items() if c >= self._voxel_min_hits),
                dtype=np.int64,
            )
            if stable_arr.size > 0:
                all_keys = self._pack_voxel_keys(pts_send)
                mask = np.isin(all_keys, stable_arr)
                pts_send = pts_send[mask]

        # NOTE: SlamBridge now applies map→odom TF on the points before
        # publishing (commit d79c409). Re-transforming here would double-
        # shift the cloud and make it fly away from the saved_map底图 every
        # time the localizer refines TF. Points arrive already in map frame.

        # Binary path — encode once, fan out to /ws/cloud subscribers.
        # The old JSON SSE event would allocate ~180k Python floats per frame
        # (60k * 3 .tolist()) and serialize ~1.4 MB of text; the binary frame
        # is 6 bytes/point + 28 byte header.
        from core.utils.binary_codec import encode_pointcloud
        buf = encode_pointcloud(pts_send[:, :3])
        self._publish_cloud_frame(buf)
        # Tiny SSE meta event so legacy UI bits (status bar, point count)
        # still see "map updated" — payload stays under 100 B.
        self.push_event({
            "type": "map_cloud",
            "count": int(len(pts_send)),
            "seq": self._cloud_seq,
            "bytes": len(buf),
        })

    def _get_slam_profile(self) -> str:
        """Return current SLAM profile (cached 5s)."""
        now = time.time()
        if now - self._slam_profile_ts < 5.0:
            return self._cached_slam_profile
        self._slam_profile_ts = now
        try:
            from core.service_manager import get_service_manager
            svc = get_service_manager()
            services = svc.status(
                "super_lio_relocation",
                "super_lio",
                "slam_pgo",
                "localizer",
                "slam",
            )
            if services.get("super_lio_relocation") in ("running", "active"):
                profile = "super_lio_relocation"
            elif services.get("super_lio") in ("running", "active"):
                profile = "super_lio"
            elif services.get("slam_pgo") in ("running", "active"):
                profile = "fastlio2"
            elif services.get("localizer") in ("running", "active"):
                profile = "localizer"
            elif services.get("slam") in ("running", "active"):
                profile = "slam"
            elif self._session_uses_external_slam_none():
                profile = "none"
            else:
                profile = "stopped"
            self._cached_slam_profile = profile
        except Exception as e:
            logger.debug("_get_slam_profile: service_manager lookup failed: %s", e)
        return self._cached_slam_profile

    @staticmethod
    def _voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
        """Fast numpy voxel grid downsample."""
        keys = (pts[:, :3] / voxel).astype(np.int32)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx]

    def _pack_voxel_keys(self, pts_xyz: np.ndarray) -> np.ndarray:
        """Pack (x,y,z) voxel coords into int64 keys for dict lookup.

        Uses 20-bit axis coord (±524k after offset) packed into one int64
        so voxel_hits can be a fast int→int dict instead of a tuple-key dict.
        At voxel_size=0.15m this covers ±78km world range. Handles negative
        coords via self._voxel_key_offset.
        """
        k = (pts_xyz[:, :3] * self._inv_map_voxel_size).astype(np.int64)
        k = (k + self._voxel_key_offset) & 0xFFFFF
        return (k[:, 0] << 40) | (k[:, 1] << 20) | k[:, 2]

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        with self._state_lock:
            self._sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)
        # Push detected objects to SSE viewer at ~2Hz
        self._sg_throttle += 1
        if self._sg_throttle % 5 == 0:
            try:
                objects = [
                    {
                        "label": obj.label,
                        "x": round(float(obj.position.x), 2),
                        "y": round(float(obj.position.y), 2),
                        "conf": round(float(obj.confidence), 2),
                    }
                    for obj in sg.objects
                    if obj.label and float(obj.confidence) > 0.25
                ]
                if objects:
                    self.push_event({"type": "scene_graph", "objects": objects})
            except Exception as e:
                logger.debug("_on_scene_graph: failed to build objects list: %s", e)

    def _on_safety(self, state: SafetyState) -> None:
        d = {"level": getattr(state, "level", 0), "ts": time.time()}
        with self._state_lock:
            self._safety = d
        self.push_event({"type": "safety", "data": d})

    def _on_mission(self, status: dict) -> None:
        d = status if isinstance(status, dict) else {"raw": str(status)}
        with self._state_lock:
            self._mission = d
        self.push_event({"type": "mission", "data": d})
        try:
            from gateway.services.runtime_status import build_navigation_status

            self.push_event({
                "type": "navigation_status",
                "data": build_navigation_status(self),
            })
        except Exception as e:
            logger.debug("_on_mission: build_navigation_status failed: %s", e)

    def _on_eval(self, ev: ExecutionEval) -> None:
        d = ev.to_dict() if hasattr(ev, "to_dict") else {"raw": str(ev)}
        with self._state_lock:
            self._eval = d
        self.push_event({"type": "eval", "data": d})

    def _on_dialogue(self, state: dict) -> None:
        d = state if isinstance(state, dict) else {"raw": str(state)}
        with self._state_lock:
            self._dialogue = d
        self.push_event({"type": "dialogue", "data": d})

    def _on_gnss_fusion_health(self, state: dict) -> None:
        """Forward SlamBridge gnss_fusion_health to SSE (type=gnss_fusion)."""
        d = state if isinstance(state, dict) else {"raw": str(state)}
        self._blackbox.record("gnss", d)
        self.push_event({"type": "gnss_fusion", "data": d})

    def _on_localization_status(self, state: dict) -> None:
        """Forward SlamBridge localization_status to SSE (type=slam_diag).

        Surfaces IEKF internals (`pos_cov_trace`, `ieskf_iter_num`,
        `ieskf_converged`) plus the existing degeneracy fields so dashboards
        and the drift watchdog can react before pose itself blows up.
        """
        if not isinstance(state, dict):
            return
        d = dict(state)
        profile = self._get_slam_profile()
        if profile and profile != "—":
            d["backend"] = profile
        capability_defaults = backend_capability_defaults(profile)
        if profile in {"super_lio", "super_lio_relocation"}:
            d.setdefault("health_source", "odom_map_cloud")
            d["relocalization_supported"] = False
            d["saved_map_relocalization_supported"] = False
            d.setdefault("relocalization_state", "unsupported")
            d["map_save_supported"] = profile == "super_lio"
            d.setdefault(
                "map_save_source",
                (
                    "live_map_cloud_snapshot"
                    if profile == "super_lio"
                    else "active_map"
                ),
            )
            if "map_state" not in d:
                d["map_state"] = (
                    (
                        "live_map_cloud"
                        if profile == "super_lio"
                        else "relocation_map_cloud"
                    )
                    if d.get("map_cloud_fresh")
                    else "map_cloud_stale"
                )
            if "localizer_health" not in d:
                state_name = str(d.get("state", "") or "").upper()
                if state_name in {"LOST", "UNINIT", "UNINITIALIZED"}:
                    d["localizer_health"] = "LIO_LOST"
                elif state_name in {"DIVERGED"}:
                    d["localizer_health"] = "LIO_DIVERGED"
                elif state_name in {"DEGRADED", "FALLBACK_GNSS_ONLY"}:
                    d["localizer_health"] = "LIO_DEGRADED"
                else:
                    d["localizer_health"] = "LIO_TRACKING"
        elif profile == "localizer":
            d.setdefault("relocalization_supported", True)
            d.setdefault("saved_map_relocalization_supported", True)
        elif profile in {"fastlio2", "slam"}:
            d.setdefault("relocalization_supported", False)
            d.setdefault("saved_map_relocalization_supported", False)
        d.setdefault(
            "saved_map_relocalization_supported",
            d.get(
                "relocalization_supported",
                capability_defaults["saved_map_relocalization_supported"],
            ),
        )
        d.setdefault(
            "restart_recovery_supported",
            capability_defaults["restart_recovery_supported"],
        )
        d.setdefault("recovery_method", capability_defaults["recovery_method"])
        d["_gateway_received_ts"] = time.time()
        d["_gateway_received_mono"] = time.monotonic()
        with self._state_lock:
            self._localization_status = d
        self._blackbox.record("slam_diag", d)
        self.push_event({"type": "slam_diag", "data": d})

    def _on_tare_stats(self, stats: dict) -> None:
        """Forward TAREExplorerModule tare_stats to SSE (type=tare_stats)."""
        d = stats if isinstance(stats, dict) else {"raw": str(stats)}
        with self._state_lock:
            self._last_tare_stats = dict(d)
        self.push_event({"type": "tare_stats", "data": d})

    def _on_exploration_supervisor(self, state: dict) -> None:
        """Forward ExplorationSupervisorModule supervisor_state to SSE
        (type=exploration_supervisor). Dashboards show mode + reason."""
        d = state if isinstance(state, dict) else {"raw": str(state)}
        with self._state_lock:
            self._exploration_supervisor_state = dict(d)
        self.push_event({"type": "exploration_supervisor", "data": d})

    def _json_payload(self, value: Any) -> Any:
        try:
            return json.loads(json.dumps(value, ensure_ascii=False, default=str))
        except Exception as e:
            logger.debug("_json_payload round-trip failed: %s", e)
            return {"raw": str(value)}

    def _on_traversable_frontiers(self, candidates: list) -> None:
        """Forward read-only traversable frontier candidates to Gateway clients."""
        data = self._json_payload(candidates if isinstance(candidates, list) else [])
        with self._state_lock:
            self._last_traversable_frontiers = list(data) if isinstance(data, list) else []
        self.push_event({"type": "traversable_frontiers", "data": data})

    def _on_frontier_candidate(self, candidate: dict) -> None:
        """Forward the current best traversable frontier without issuing a command."""
        data = self._json_payload(candidate if isinstance(candidate, dict) else {"raw": str(candidate)})
        with self._state_lock:
            self._last_frontier_candidate = dict(data) if isinstance(data, dict) else {"raw": str(data)}
        self.push_event({"type": "frontier_candidate", "data": data})

    def _on_global_path(self, path: list) -> None:
        # path is list of np.ndarray [x, y, z] from NavigationModule
        points = [
            {"x": float(p[0]), "y": float(p[1]), "z": float(p[2]) if len(p) > 2 else 0.0}
            for p in path
        ]
        with self._state_lock:
            self._last_path = points
        self.push_event({"type": "global_path", "points": points})

    def _on_local_path(self, path: Path) -> None:
        """Push local planner path as SSE (updated at ~10Hz, throttle to ~2Hz)."""
        self._local_path_throttle = getattr(self, '_local_path_throttle', 0) + 1
        if self._local_path_throttle % 5 != 0:
            return
        try:
            points = [
                {"x": float(p.pose.position.x), "y": float(p.pose.position.y)}
                for p in path.poses
            ] if hasattr(path, 'poses') else []
            self.push_event({"type": "local_path", "points": points})
        except Exception as e:
            logger.debug("_on_local_path: failed to build points: %s", e)

    def _on_costmap(self, cm: dict) -> None:
        """Throttle OccupancyGridModule costmap to ~2 Hz and push as SSE.

        Costmap is generated in odom frame (OccupancyGridModule). When
        navigating, shift the grid origin into map frame so it overlays the
        saved map底图. Grid cells stay axis-aligned — if map→odom has
        significant yaw, cells will be slightly skewed; acceptable for
        short-term ICP tracking (yaw error typically < 5°).
        """
        self._costmap_throttle += 1
        if self._costmap_throttle % 5 != 0:
            return
        grid = cm.get("grid")
        if grid is None:
            return
        if not self._should_emit_sse_raster("costmap"):
            return
        try:
            g = np.clip(grid, 0, 100).astype(np.uint8)
            # grid[iy, ix]: shape[0]=rows(Y), shape[1]=cols(X)
            rows = int(g.shape[0])
            cols = int(g.shape[1]) if g.ndim >= 2 else rows
            origin = [float(v) for v in cm.get("origin", [0.0, 0.0])]
            # OccupancyGridModule now ingests map-frame odom (SlamBridge applies
            # TF before publish), so its grid.origin is already map-frame — no
            # Gateway re-transform. Re-applying TF here double-shifted and
            # caused costmap to orbit the robot each time the localizer
            # refined map→odom. Leave yaw=0 since no rotation left to apply.
            yaw = 0.0
            self.push_event({
                "type":       "costmap",
                "grid_b64":   base64.b64encode(g.tobytes()).decode(),
                "rows":       rows,
                "cols":       cols,
                "resolution": float(cm.get("resolution", 0.1)),
                "origin":     origin,
                "yaw":        float(yaw),
            })
        except Exception as exc:
            logger.debug("_on_costmap serialize failed: %s", exc)

    def _on_slope_grid(self, data: dict) -> None:
        """Push slope grid as SSE event for web visualization.

        Same origin-shift as _on_costmap so the slope overlay aligns with
        the map底图 in navigating mode.
        """
        grid = data.get("grid")
        if grid is None:
            return
        if not self._should_emit_sse_raster("slope_grid"):
            return
        try:
            # Shape metadata is cheap; inline raster payload is optional.
            arr = np.asarray(grid)
            rows = int(arr.shape[0])
            cols = int(arr.shape[1]) if arr.ndim >= 2 else rows
            origin = [float(v) for v in data.get("origin", [0.0, 0.0])]
            # Same reasoning as _on_costmap — slope grid origin is already in
            # map frame since SlamBridge upstream.
            yaw = 0.0
            event: dict[str, Any] = {
                "type":       "slope_grid",
                "available":  True,
                "rows":       rows,
                "cols":       cols,
                "resolution": float(data.get("resolution", 0.2)),
                "origin":     origin,
                "yaw":        float(yaw),
            }
            if self._sse_slope_payload_enabled:
                g = np.clip(arr * (255.0 / 90.0), 0, 255).astype(np.uint8)
                event.update({
                    "payload": "inline",
                    "encoding": "uint8_slope_degrees_0_90",
                    "grid_b64": base64.b64encode(g.tobytes()).decode(),
                })
            else:
                event.update({
                    "payload": "omitted",
                    "reason": "inline_payload_disabled",
                    "encoding": "uint8_slope_degrees_0_90",
                })
            self.push_event(event)
        except Exception as exc:
            logger.debug("_on_slope_grid serialize failed: %s", exc)

    def _on_agent_message(self, msg: dict) -> None:
        """Forward semantic planner chat messages as SSE for the Web ChatPanel."""
        try:
            self.push_event({
                "type":  "agent_message",
                "role":  str(msg.get("role", "assistant")),
                "text":  str(msg.get("text", "")),
                "phase": str(msg.get("phase", "")),
                "ts":    float(msg.get("ts", time.time())),
            })
        except Exception as exc:
            logger.debug("_on_agent_message serialize failed: %s", exc)

    def _should_emit_sse_raster(self, event_type: str) -> bool:
        """Gate expensive raster SSE payloads before serialization."""
        with self._sse_lock:
            if not self._sse_queues:
                return False
            interval = max(0.0, float(self._sse_raster_min_interval_s))
            now = time.monotonic()
            last = self._sse_raster_last_emit.get(event_type)
            if interval > 0 and last is not None and now - last < interval:
                self._sse_suppressed_events[event_type] = (
                    self._sse_suppressed_events.get(event_type, 0) + 1
                )
                return False
            self._sse_raster_last_emit[event_type] = now
            return True

    # -- SSE fan-out --------------------------------------------------------

    @staticmethod
    def _running_loop_or_none() -> asyncio.AbstractEventLoop | None:
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return None

    @staticmethod
    def _call_queue_put_latest(
        q: asyncio.Queue,
        item: Any,
        loop: asyncio.AbstractEventLoop | None,
        record: Callable[[bool, int], None],
    ) -> None:
        def _put_and_record() -> None:
            dropped = put_latest(q, item)
            record(dropped, q.qsize())

        if loop is not None and loop.is_running():
            try:
                loop.call_soon_threadsafe(_put_and_record)
                return
            except RuntimeError:
                pass
        _put_and_record()

    def _record_sse_delivery(self, dropped: bool, depth: int) -> None:
        with self._sse_lock:
            if dropped:
                self._sse_dropped_events += 1
            self._sse_max_depth_seen = max(self._sse_max_depth_seen, depth)

    def _record_cloud_delivery(self, dropped: bool, depth: int) -> None:
        with self._cloud_lock:
            if dropped:
                self._cloud_dropped_frames += 1
            self._cloud_max_depth_seen = max(self._cloud_max_depth_seen, depth)

    def push_event(self, event: dict) -> None:
        """Thread-safe: push an event to all connected SSE clients."""
        with self._sse_lock:
            self._sse_event_seq += 1
            event_id = self._sse_event_seq
            subscribers = [
                (q, self._sse_queue_loops.get(q))
                for q in self._sse_queues
            ]
            self._sse_published_events += 1
        payload = normalize_sse_event(event, event_id=event_id)
        for q, loop in subscribers:
            self._call_queue_put_latest(q, payload, loop, self._record_sse_delivery)

    def _next_sse_event_id(self) -> int:
        with self._sse_lock:
            self._sse_event_seq += 1
            return self._sse_event_seq

    def _sse_subscribe_with_event_id(self) -> tuple[asyncio.Queue, int]:
        """Subscribe and reserve the first snapshot event id atomically."""
        q: asyncio.Queue = asyncio.Queue(maxsize=self._sse_queue_maxsize)
        loop = self._running_loop_or_none()
        with self._sse_lock:
            self._sse_event_seq += 1
            event_id = self._sse_event_seq
            self._sse_queues.append(q)
            self._sse_queue_loops[q] = loop
        return q, event_id

    def _sse_subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=self._sse_queue_maxsize)
        loop = self._running_loop_or_none()
        with self._sse_lock:
            self._sse_queues.append(q)
            self._sse_queue_loops[q] = loop
        return q

    def _sse_unsubscribe(self, q: asyncio.Queue) -> None:
        with self._sse_lock:
            try:
                self._sse_queues.remove(q)
            except ValueError:
                pass
            self._sse_queue_loops.pop(q, None)

    # -- Binary cloud fan-out ----------------------------------------------

    def _publish_cloud_frame(self, buf: bytes) -> None:
        """Thread-safe: store latest binary cloud + wake /ws/cloud clients."""
        with self._cloud_lock:
            self._latest_cloud_buf = buf
            self._cloud_seq += 1
            subscribers = [
                (q, self._cloud_sub_loops.get(q))
                for q in self._cloud_subs
            ]
            self._cloud_published_frames += 1
        for q, loop in subscribers:
            self._call_queue_put_latest(q, buf, loop, self._record_cloud_delivery)

    def _cloud_subscribe(self) -> tuple[asyncio.Queue, bytes | None]:
        # maxsize=2 keeps memory bounded; cloud frames supersede each other,
        # so a backlog longer than that is wasted bandwidth.
        q: asyncio.Queue = asyncio.Queue(maxsize=self._cloud_queue_maxsize)
        loop = self._running_loop_or_none()
        with self._cloud_lock:
            self._cloud_subs.append(q)
            self._cloud_sub_loops[q] = loop
            latest = self._latest_cloud_buf
        return q, latest

    def _cloud_unsubscribe(self, q: asyncio.Queue) -> None:
        with self._cloud_lock:
            try:
                self._cloud_subs.remove(q)
            except ValueError:
                pass
            self._cloud_sub_loops.pop(q, None)

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
        with self._cloud_lock:
            cloud_depths = [q.qsize() for q in self._cloud_subs]
            cloud = {
                "clients": len(self._cloud_subs),
                "queue_maxsize": self._cloud_queue_maxsize,
                "queue_depths": cloud_depths,
                "max_depth_seen": self._cloud_max_depth_seen,
                "published_frames": self._cloud_published_frames,
                "dropped_frames": self._cloud_dropped_frames,
                "drop_policy": DROP_OLDEST_POLICY,
                "latest_seq": self._cloud_seq,
            }
        return {
            "sse": sse,
            "cloud": cloud,
            "recommended_client_rates_hz": dict(RECOMMENDED_CLIENT_RATES_HZ),
        }

    def _command_stats_snapshot(self) -> dict[str, Any]:
        return self._command_journal.snapshot()

    def _publish_command_ack(
        self,
        payload: dict[str, Any],
        *,
        status_code: int | None = None,
    ) -> None:
        """Publish a lightweight command acknowledgement for App/Web clients."""
        if not isinstance(payload, dict):
            return
        command = payload.get("command")
        if not isinstance(command, dict):
            return
        data = {
            "schema_version": 1,
            "ok": bool(payload.get("ok", False)),
            "status": payload.get("status"),
            "error": payload.get("error"),
            "message": payload.get("message"),
            "command": command,
            "detail": payload.get("detail"),
            "status_code": status_code,
            "ts": time.time(),
        }
        self.push_event({"type": "command_ack", "data": data})

    def _run_control_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], dict[str, Any]],
    ) -> dict[str, Any]:
        request_id = getattr(body, "request_id", None) if body is not None else None
        client_id = getattr(body, "client_id", None) if body is not None else None
        replay = self._command_journal.replay(command, request_id)
        if replay is not None:
            self._publish_command_ack(replay, status_code=200)
            return replay
        response = self._command_journal.accept(
            command,
            request_id,
            client_id,
            action(),
        )
        self._publish_command_ack(response, status_code=200)
        return response

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

    def _teleop_on_joy(self, lx: float, ly: float, az: float) -> None:
        """Forward raw joystick input to TeleopModule via joy_input port."""
        tm = self._teleop_module
        if tm is not None and hasattr(tm, "joy_input"):
            tm.joy_input.publish({"lx": lx, "ly": ly, "az": az})
        else:
            # Fallback: publish directly (no TeleopModule present)
            lx = max(-1.0, min(1.0, lx)) * getattr(self, "_teleop_max_speed", 0.5)
            ly = max(-1.0, min(1.0, ly)) * getattr(self, "_teleop_max_speed", 0.5)
            az = max(-1.0, min(1.0, az)) * getattr(self, "_teleop_max_yaw", 1.0)
            self.cmd_vel.publish(Twist(
                linear=Vector3(x=lx, y=ly, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=az),
            ))

    def _teleop_release(self) -> None:
        tm = self._teleop_module
        if tm is not None:
            tm.force_release()
        else:
            self.cmd_vel.publish(Twist())

    # -- FastAPI app --------------------------------------------------------

    def _build_app(self):
        try:
            from fastapi import FastAPI
            from fastapi.middleware.cors import CORSMiddleware
            from fastapi.responses import JSONResponse
        except ImportError:
            logger.error("FastAPI not installed — run: pip install fastapi uvicorn")
            return None

        cors_origins = os.environ.get(
            "LINGTU_CORS_ORIGINS",
            "http://localhost:5050,http://127.0.0.1:5050",
        ).split(",")
        app = FastAPI(
            title="LingTu Gateway",
            version="2.0",
            docs_url="/docs",
            redoc_url="/redoc",
        )
        app.add_middleware(
            CORSMiddleware,
            allow_origins=cors_origins,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # -- API Key auth (disabled if LINGTU_API_KEY not set) -------------
        from gateway.auth import APIKeyMiddleware
        from gateway.routes import (
            map_lifecycle_payload,
            mount_dashboard_assets,
            register_app_routes,
            register_auth_routes,
            register_camera_routes,
            register_command_routes,
            register_diagnostic_routes,
            register_map_routes,
            register_operation_routes,
            register_realtime_routes,
            register_session_routes,
            register_status_routes,
        )
        app.add_middleware(APIKeyMiddleware)

        gw = self

        @app.post("/api/v1/runtime/backend")
        async def post_runtime_backend(body: dict[str, Any]):
            category = str(body.get("category", ""))
            backend = str(body.get("backend", ""))
            config = body.get("config") or {}
            if not category or not backend:
                return {
                    "ok": False,
                    "reason": "missing_category_or_backend",
                    "category": category,
                    "requested_backend": backend,
                }
            if not isinstance(config, dict):
                return {
                    "ok": False,
                    "category": category,
                    "requested_backend": backend,
                    "reason": "invalid_config",
                }
            return gw.reconfigure_backend(category, backend, **config)

        @app.post(
            "/api/v1/driver/swap",
            summary="Swap the active driver backend at runtime",
            response_model=DriverSwapResponse,
            responses={
                503: {"model": GatewayErrorResponse},
            },
        )
        async def post_driver_swap(body: DriverSwapRequest):
            result = gw._on_driver_swap(body.model_dump())
            if not result.get("success"):
                return JSONResponse(
                    status_code=503,
                    content=GatewayErrorResponse(
                        error=result.get("message", "driver swap failed"),
                        message=result.get("message", "driver swap failed"),
                        detail=result,
                    ).model_dump(),
                )
            return DriverSwapResponse(
                success=True,
                message=result.get("message", f"Swapped to {body.driver}"),
                swap_time_ms=result.get("swap_time_ms", 0.0),
                driver=body.driver,
                detail=result.get("detail"),
            )

        register_operation_routes(app, gw)
        register_diagnostic_routes(app, gw)
        register_map_routes(app, gw)
        register_status_routes(app, gw)
        register_session_routes(app, gw)
        register_command_routes(app, gw)

        # ── Mode ───────────────────────────────────────────────────────────

        # ── Map management ─────────────────────────────────────────────────

        @app.post(
            "/api/v1/maps",
            summary="Map lifecycle management",
            response_model=MapLifecycleResponse,
            responses={
                400: {"model": GatewayErrorResponse},
                503: {"model": GatewayErrorResponse},
            },
        )
        async def post_maps(body: MapRequest):
            mgr = gw._map_mgr
            if mgr is None:
                message = "MapManagerModule not running"
                return JSONResponse(
                    status_code=503,
                    content=GatewayErrorResponse(
                        error=message,
                        message=message,
                    ).model_dump(),
                )
            # Deliver command via Module port (synchronous one-shot)
            result: list[dict] = []

            def _capture(resp: dict) -> None:
                result.append(resp)

            mgr.map_response._add_callback(_capture)
            try:
                action = {
                    "use": "set_active",
                    "build": "build_tomogram",
                }.get(body.action, body.action)
                cmd = {"action": action}
                if body.name:
                    cmd["name"] = body.name
                if body.new_name:
                    cmd["new_name"] = body.new_name
                mgr.map_command._deliver(json.dumps(cmd))
            finally:
                try:
                    mgr.map_response._callbacks.remove(_capture)
                except (ValueError, AttributeError):
                    pass

            resp = result[0] if result else {"success": False, "message": "no response"}
            if not resp.get("success"):
                message = str(resp.get("message") or "failed")
                return JSONResponse(
                    status_code=400,
                    content=GatewayErrorResponse(
                        error=message,
                        message=message,
                        detail=resp,
                    ).model_dump(),
                )
            legacy = dict(resp)
            legacy.pop("success", None)
            return map_lifecycle_payload(True, **legacy)

        register_auth_routes(app)
        register_app_routes(app, gw)
        register_camera_routes(app, gw)
        register_realtime_routes(app, gw)
        mount_dashboard_assets(app)
        try:
            from gateway.services.app_bootstrap import prewarm_app_capability_contracts

            prewarm_app_capability_contracts(gw)
        except Exception:
            logger.debug(
                "GatewayModule: App/Web capability contract prewarm failed",
                exc_info=True,
            )

        return app

    def _run_server(self, stop_event: threading.Event | None = None) -> bool:
        if self._app is None:
            logger.error("uvicorn server cannot start: FastAPI app is not configured")
            return False
        stop_event = stop_event or self._stop_event
        # Reduce GIL switch interval so uvicorn's event loop gets CPU time
        # even when LiDAR/DDS callbacks are active in other threads.
        # Default is 5ms; 1ms gives the event loop ~10x more scheduling chances.
        old_interval = sys.getswitchinterval()
        sys.setswitchinterval(0.001)
        uvicorn = None
        try:
            import uvicorn as _uv
            uvicorn = _uv
        except ImportError:
            logger.error("uvicorn not installed — run: pip install 'uvicorn[standard]'")
            return False
        server = None
        try:
            # uvloop only ships for Linux/macOS; "auto" picks uvloop where
            # available and falls back to asyncio on Windows.
            config = uvicorn.Config(
                self._app,
                host=self._host,
                port=self._port,
                log_level="warning",
                loop="auto",
                ws="auto",
                lifespan="off",
                timeout_keep_alive=30,
                ws_max_size=2 * 1024 * 1024,  # 2 MB — enough for 1080p JPEG
            )
            server = uvicorn.Server(config)
            self._server = server
            if stop_event.is_set():
                server.should_exit = True
            logger.info("uvicorn server.run() starting on %s:%d", self._host, self._port)
            server.run()
            if bool(getattr(server, "should_exit", False)) or bool(
                getattr(server, "force_exit", False)
            ):
                logger.info("uvicorn server stopped cleanly")
                return True
            logger.error("uvicorn server.run() returned without a shutdown signal")
            return False
        except Exception:
            logger.exception("uvicorn crashed")
            return False
        finally:
            if server is not None and self._server is server:
                self._server = None
            sys.setswitchinterval(old_interval)

    # -- health -------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        traffic = self._traffic_stats_snapshot()
        commands = self._command_stats_snapshot()
        n_sse = traffic["sse"]["clients"]
        with self._map_cloud_lock:
            map_pts = len(self._map_points) if self._map_points is not None else 0
        info["gateway"] = {
            "port":        self._port,
            "sse_clients": n_sse,
            "teleop_clients": self._teleop_client_count(),
            "teleop_active": self._teleop_active,
            "has_odom":    self._odom is not None,
            "has_sg":      self._sg_json != "{}",
            "map_points":  map_pts,
            "traffic":     traffic,
            "commands":    commands,
        }
        return info

    # -- SLAM Hz measurement --------------------------------------------------

    def _get_slam_hz_cached(self) -> float:
        """Estimate SLAM Hz from odometry already flowing through Gateway.

        This avoids spawning ``ros2 topic hz`` from the service process. Those
        diagnostic subprocesses are expensive on S100P and can become orphaned,
        stealing CPU from the SLAM/localizer stack.
        """
        now = time.time()
        with self._state_lock:
            timestamps = list(self._odom_timestamps)
        if len(timestamps) < 2:
            return 0.0
        if now - timestamps[-1] > 2.0:
            return 0.0
        span = timestamps[-1] - timestamps[0]
        if span <= 0.0:
            return 0.0
        return round((len(timestamps) - 1) / span, 1)

    # -- Map 3D viewer HTML generation ----------------------------------------

    def _generate_viewer_live(self) -> str:
        """Snapshot ikd-tree to temp PCD, then render. Shows exactly what save_map would produce."""
        import tempfile
        tmp = os.path.join(tempfile.gettempdir(), "lingtu_live_snapshot.pcd")
        try:
            subprocess.run(
                ["bash", "-c",
                 "source /opt/ros/humble/setup.bash && "
                 "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
                 "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                 f"ros2 service call /nav/save_map interface/srv/SaveMaps "
                 f"\"{{file_path: '{tmp}'}}\""],
                capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=15)
        except Exception as e:
            logger.debug("_generate_viewer_live: ros2 save_map failed: %s", e)

        if os.path.isfile(tmp) and os.path.getsize(tmp) > 200:
            # Parse temp PCD
            with open(tmp, "rb") as f:
                n_points = 0
                point_step = 16
                while True:
                    line = f.readline().decode("ascii", errors="ignore").strip()
                    if "POINTS" in line:
                        n_points = int(line.split()[-1])
                    if "SIZE" in line:
                        point_step = sum(int(s) for s in line.split()[1:])
                    if line.startswith("DATA"):
                        break
                data = f.read(n_points * point_step)
            pts = np.frombuffer(
                data[:n_points * point_step], dtype=np.float32,
            ).reshape(n_points, point_step // 4)[:, :3]
            valid = np.isfinite(pts).all(axis=1)
            pts = pts[valid]
            with self._map_cloud_lock:
                self._map_points = pts
            return self._generate_viewer_html(robot_pos=self._odom)
        else:
            return (
                "<html><body style='background:#0a0a0f;color:#fff;"
                "font-family:monospace;padding:40px'><h2>暂无地图数据</h2>"
                "<p>开始建图并移动机器人后刷新。</p></body></html>"
            )

    def _generate_viewer_from_pcd(self, map_name: str) -> str:
        """Load a saved PCD file and generate viewer HTML (does NOT touch live data)."""
        pcd_path = str(map_dir_for(map_name) / "map.pcd")
        if not os.path.isfile(pcd_path):
            return (
                f"<html><body style='background:#0a0a0f;color:#fff;"
                f"font-family:monospace;padding:40px'><h2>地图不存在: {map_name}</h2>"
                f"<p>找不到 {pcd_path}</p></body></html>"
            )

        # Parse PCD binary
        with open(pcd_path, "rb") as f:
            n_points = 0
            point_step = 16
            while True:
                line = f.readline().decode("ascii", errors="ignore").strip()
                if "POINTS" in line:
                    n_points = int(line.split()[-1])
                if "SIZE" in line:
                    point_step = sum(int(s) for s in line.split()[1:])
                if line.startswith("DATA"):
                    break
            data = f.read(n_points * point_step)

        pts = np.frombuffer(data[:n_points * point_step], dtype=np.float32).reshape(n_points, point_step // 4)[:, :3]
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) > 0:
            med = np.median(pts, axis=0)
            dist = np.abs(pts - med).max(axis=1)
            pts = pts[dist < 100]

        # Generate viewer with loaded data directly (no shared state modification)
        return self._generate_viewer_html(override_pts=pts)

    def _generate_viewer_html(self, override_pts=None, robot_pos=None) -> str:
        # NOTE: The 3D map viewer is served from a static template file.
        # Previously this method interpolated point-cloud data (n, coords, zmin,
        # zmax, cx, cy, rx, ry, ryaw, robot_visible) into the HTML via f-string.
        # Those dynamic values are now documented in the template header comment
        # at src/gateway/templates/map_viewer.html.
        template_path = FilePath(__file__).parent / "templates" / "map_viewer.html"
        return template_path.read_text(encoding="utf-8")
