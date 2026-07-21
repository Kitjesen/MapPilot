"""GatewayModule initialization helpers.

These helpers keep GatewayModule.__init__ readable. They only set process-local
state; anything that starts threads or talks to devices belongs in lifecycle or
runtime services.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any

from gateway.services.cloud_viewer import CloudViewerService
from gateway.services.commands import CommandJournal, ControlLease
from gateway.services.loc_cache import LocCache
from gateway.services.session_cache import SessionCache
from gateway.services.sse import call_queue_put_latest, running_loop_or_none
from gateway.services.traffic import (
    DEFAULT_CLOUD_QUEUE_MAXSIZE,
    DEFAULT_SSE_QUEUE_MAXSIZE,
)
from localization.service import Localization
from runtime.tf import FrameTree


def init_core_state(
    gw: Any,
    *,
    sse_raster_min_interval_s: float,
    sse_slope_payload_enabled: bool,
) -> None:
    gw._state_lock = threading.RLock()
    gw._runtime_cache = LocCache()
    gw._session_runtime = SessionCache()
    gw._sg_json = "{}"
    gw._safety = None
    gw._mission = None
    gw._eval = None
    gw._dialogue = None
    gw._mode = "manual"
    gw._last_path = []
    gw._last_local_path = []

    gw._lease = ControlLease()
    gw._command_journal = CommandJournal()

    gw._sse_lock = threading.Lock()
    gw._sse_queues: list[asyncio.Queue] = []
    gw._sse_queue_loops: dict[asyncio.Queue, asyncio.AbstractEventLoop | None] = {}
    gw._sse_queue_maxsize = DEFAULT_SSE_QUEUE_MAXSIZE
    gw._sse_event_seq = 0
    gw._sse_published_events = 0
    gw._sse_dropped_events = 0
    gw._sse_max_depth_seen = 0
    gw._sse_raster_min_interval_s = sse_raster_min_interval_s
    gw._sse_slope_payload_enabled = sse_slope_payload_enabled
    gw._sse_raster_last_emit = {}
    gw._sse_suppressed_events = {}


def init_module_refs(
    gw: Any,
    *,
    map_save_adapter: Any,
    manage_session_services: bool,
) -> None:
    gw._map_mgr = None
    gw._all_modules = {}
    gw._navigation = None
    gw._goals = None
    gw._nav_commands = None
    gw._inspection = None
    gw._cmd_vel_mux = None
    gw._backend_reconfigure_modules = {}
    gw.localization = Localization()
    gw._map_save_adapter = map_save_adapter
    gw._manage_session_services = manage_session_services


def init_recording_state(gw: Any) -> None:
    gw._bag_proc = None
    gw._bag_path = ""
    gw._bag_started_ts = 0.0
    gw._bag_lock = threading.Lock()


def init_cloud_and_frame_state(gw: Any, *, frame_tree: Any | None) -> None:
    gw._cloud_viewer = CloudViewerService(
        queue_put_latest=call_queue_put_latest,
        current_loop=running_loop_or_none,
        push_event=gw.push_event,
        session_mode=lambda: gw._session_mode,
        product_session=lambda: gw._session_product_session,
        active_session_map=lambda: gw._session_map,
        saved_active_map=gw._session_active_map_name,
        queue_maxsize=DEFAULT_CLOUD_QUEUE_MAXSIZE,
    )
    gw._frame_tree = frame_tree or FrameTree.from_robot_config()


def init_runtime_status_state(
    gw: Any,
    *,
    brainstem_health_cache_ttl_s: float,
) -> None:
    gw._temporal_store = None
    gw._costmap_throttle = 0
    gw._sg_throttle = 0
    gw._slam_status_throttle = 0

    gw._brainstem_health_lock = threading.Lock()
    gw._brainstem_health_cache = None
    gw._brainstem_health_cache_ts = 0.0
    gw._brainstem_health_refreshing = False
    gw._brainstem_health_refresh_thread = None
    gw._brainstem_health_cache_ttl_s = brainstem_health_cache_ttl_s


def init_drift_watchdog_state(
    gw: Any,
    *,
    enabled: bool,
    interval_s: float,
    xy_limit: float,
    v_limit: float,
    cooldown_s: float,
    restart_delay_s: float,
) -> None:
    gw._drift_watchdog_enabled = enabled
    gw._drift_watchdog_interval = interval_s
    gw._drift_watchdog_xy_limit = xy_limit
    gw._drift_watchdog_v_limit = v_limit
    gw._drift_watchdog_cooldown = cooldown_s
    gw._drift_restart_delay_s = restart_delay_s
    gw._drift_last_restart_ts = 0.0
    gw._drift_restart_count = 0
    gw._drift_watchdog_thread = None


def init_exploration_state(gw: Any) -> None:
    gw._exploring = False
    gw._frontier_explorer = None
    gw._tare_explorer = None
    gw._last_tare_stats = None
    gw._exploration_supervisor_state = None
    gw._last_traversable_frontiers = None
    gw._last_frontier_candidate = None
    gw._tagged_loc_module = None
    gw._swap_manager = None


def init_server_state(gw: Any) -> None:
    gw._app = None
    gw._server = None
    gw._server_thread = None
    gw._client_http_prewarm_thread = None
    gw._saved_map_loader_thread = None
    gw._stop_event = threading.Event()
    gw._defer_server = False
