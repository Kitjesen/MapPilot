"""GatewayModule initialization helpers.

These helpers keep GatewayModule.__init__ readable. They only set process-local
state; anything that starts threads or talks to devices belongs in lifecycle or
runtime services.
"""

from __future__ import annotations

import asyncio
import threading
import time
from collections import OrderedDict
from typing import Any

from gateway.services.cloud_viewer import CloudViewerService
from gateway.services.commands import CommandJournal, ControlLease
from gateway.services.inspection_task_lifecycle import create_inspection_task_timeline
from gateway.services.loc_cache import LocCache
from gateway.services.recording import NativeRecordingService
from gateway.services.sse import call_queue_put_latest, running_loop_or_none
from gateway.services.traffic import (
    DEFAULT_CLOUD_QUEUE_MAXSIZE,
    DEFAULT_SSE_QUEUE_MAXSIZE,
)
from gateway.services.ws_registry import WebSocketRegistry
from localization.service import Localization
from runtime.tf import FrameTree


def init_core_state(
    gw: Any,
    *,
    sse_raster_min_interval_s: float,
) -> None:
    gw._state_lock = threading.RLock()
    gw._runtime_cache = LocCache()
    gw._icp_quality = 0.0
    gw._localization_status = None
    gw._cached_slam_profile = ""
    gw._slam_profile_ts = 0.0
    gw._session_projection_key = None
    gw._session_mode = "idle"
    gw._session_product = gw._compiled_product or None
    gw._session_map = None
    gw._session_slam_profile = "stopped"
    gw._session_since = time.time()
    gw._sg_json = "{}"
    gw._navigation_state = None
    gw._navigation_goal_status_by_task = OrderedDict()
    gw._navigation_goal_status_by_request = OrderedDict()
    gw._navigation_goal_status_sequences = {}
    gw._latest_navigation_goal_status = None
    gw._inspection_task_timeline = create_inspection_task_timeline()
    gw._mode = "manual"
    gw._last_path = []
    gw._last_local_path = []
    gw._last_path_meta = None
    gw._last_local_path_meta = None

    gw._lease = ControlLease()
    gw._command_journal = CommandJournal()

    gw._sse_lock = threading.Lock()
    gw._sse_queues: list[asyncio.Queue] = []
    gw._sse_queue_loops: dict[asyncio.Queue, asyncio.AbstractEventLoop | None] = {}
    gw._sse_queue_event_types: dict[asyncio.Queue, set[str] | None] = {}
    gw._sse_queue_elevation_payload: dict[asyncio.Queue, bool] = {}
    gw._sse_queue_maxsize = DEFAULT_SSE_QUEUE_MAXSIZE
    gw._sse_event_seq = 0
    gw._sse_published_events = 0
    gw._sse_dropped_events = 0
    gw._sse_max_depth_seen = 0
    gw._sse_raster_min_interval_s = sse_raster_min_interval_s
    gw._sse_raster_last_emit = {}
    gw._sse_suppressed_events = {}
    gw._latest_elevation_event: dict[str, Any] | None = None


def init_module_refs(gw: Any) -> None:
    gw._all_modules = {}
    gw._goals = None
    gw._nav_commands = None
    gw._inspection = None
    gw.localization = Localization()


def init_recording_state(gw: Any) -> None:
    gw._recording = NativeRecordingService()


def init_cloud_and_frame_state(gw: Any, *, frame_tree: Any | None) -> None:
    gw._cloud_viewer = CloudViewerService(
        queue_put_latest=call_queue_put_latest,
        current_loop=running_loop_or_none,
        push_event=gw.push_event,
        session_mode=lambda: gw._session_mode,
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
) -> None:
    gw._drift_watchdog_enabled = enabled
    gw._drift_watchdog_interval = interval_s
    gw._drift_watchdog_xy_limit = xy_limit
    gw._drift_watchdog_v_limit = v_limit
    gw._drift_watchdog_cooldown = cooldown_s
    gw._drift_last_report_ts = 0.0
    gw._drift_incident_count = 0
    gw._drift_watchdog_thread = None


def init_exploration_state(gw: Any) -> None:
    gw._exploring = False
    gw._explore_runs = None
    gw._explore_projection_stop_requests = set()
    gw._last_tare_stats = None
    gw._exploration_supervisor_state = None
    gw._tagged_loc_module = None


def init_server_state(gw: Any) -> None:
    gw._app = None
    gw._server = None
    gw._server_error = None
    gw._server_thread = None
    gw._stop_event = threading.Event()
    gw._defer_server = False
    gw._runtime_status_provider = None
    gw._system_handle = None
    gw._ws_registry = WebSocketRegistry()
