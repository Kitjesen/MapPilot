"""Single native data boundary for the managed Python Host."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import OrderedDict
from typing import Any

from nav.adapters.native.abi import (
    NATIVE_COMMAND_CAP_INSPECTION_TASK_EVENTS,
    NATIVE_COMMAND_CAP_MAP_SCENE,
    NativeCommandSession,
    get_native_command_session,
)
from runtime.module import Module
from runtime.msgs import InspectionTaskEvent, NavigationGoalStatus, NavigationState, Path
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.map import MapSceneFrame
from runtime.msgs.numpy_compat import np, numpy_import_is_safe
from runtime.msgs.sensor import PointCloud2
from runtime.runtime_interface import TOPICS
from runtime.stream import Out

logger = logging.getLogger(__name__)


class HostBus(Module):
    """Bridge low-rate native product state into the in-process Host graph."""

    navigation_state: Out[NavigationState]
    navigation_goal_status: Out[NavigationGoalStatus]
    inspection_task_event: Out[InspectionTaskEvent]
    global_path: Out[Path]
    local_path: Out[Path]
    map_scene: Out[MapSceneFrame]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._poll_period_s = max(0.01, float(config.get("poll_period_s", 0.05)))
        self._goal_status_drain_limit = max(
            1,
            min(256, int(config.get("goal_status_drain_limit", 64))),
        )
        self._goal_status_retention = max(
            1,
            min(4096, int(config.get("goal_status_retention", 256))),
        )
        self._navigation_state_max_age_s = max(
            0.1,
            min(
                30.0,
                float(config.get("navigation_state_max_age_s", 2.0)),
            ),
        )
        self._session: NativeCommandSession | None = None
        self._require_map_scene = bool(config.get("require_map_scene", False))
        self._require_inspection_task_events = bool(
            config.get("require_inspection_task_events", False)
        )
        self._inspection_task_event_drain_limit = max(
            1,
            min(512, int(config.get("inspection_task_event_drain_limit", 128))),
        )
        self._inspection_task_events_enabled = False
        self._inspection_task_event_sequences: dict[str, int] = {}
        self._inspection_task_event_received = 0
        self._inspection_task_event_discarded = 0
        self._inspection_task_event_error = ""
        self._map_scene_max_age_s = max(
            0.1,
            min(30.0, float(config.get("map_scene_max_age_s", 2.0))),
        )
        self._map_scene_future_tolerance_s = max(
            0.0,
            min(
                5.0,
                float(config.get("map_scene_future_tolerance_s", 1.0)),
            ),
        )
        self._map_scene_enabled = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_cursor: tuple[str, int] | None = None
        self._nav_received_monotonic = 0.0
        self._nav_discarded = 0
        self._nav_error = ""
        self._goal_status_sequences: dict[str, int] = {}
        self._goal_status_by_request: OrderedDict[str, NavigationGoalStatus] = (
            OrderedDict()
        )
        self._goal_status_by_task: OrderedDict[str, NavigationGoalStatus] = OrderedDict()
        self._goal_status_lock = threading.RLock()
        self._received = False
        self._scene_cursor: tuple[str, int] | None = None
        self._scene_received_monotonic = 0.0
        self._scene_source_timestamp_s = 0.0
        self._scene_live = False
        self._scene_reset_epoch = 0
        self._scene_observation_sequence = 0
        self._scene_generation = 0
        self._scene_received = False
        self._scene_discarded = 0
        self._scene_error = ""
        self._map_health: dict[str, object] = {}
        self._map_state_received_monotonic = 0.0
        self._map_state_valid_samples = 0
        self._failure = ""

    def setup(self) -> None:
        session = get_native_command_session(required=True)
        if session is None:
            raise RuntimeError("native HostBus session is unavailable")
        session.ensure_host_state_abi()
        session.ensure_goal_status_abi()
        session.ensure_path_telemetry_abi()
        capabilities = int(getattr(session, "capabilities", 0))
        if self._require_map_scene or (
            capabilities & NATIVE_COMMAND_CAP_MAP_SCENE
        ):
            session.ensure_map_scene_abi()
            self._map_scene_enabled = True
        if self._require_inspection_task_events:
            if not (capabilities & NATIVE_COMMAND_CAP_INSPECTION_TASK_EVENTS):
                raise RuntimeError("native inspection task event ABI is unavailable")
            session.ensure_inspection_task_event_abi()
            self._inspection_task_events_enabled = True
        self._session = session

    def start(self) -> None:
        if self._session is None:
            raise RuntimeError("HostBus.setup() must complete before start")
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._poll,
            name="host-bus",
            daemon=True,
        )
        self._thread.start()

    def startup_readiness(self) -> str | None:
        if self._failure:
            return self._failure
        if not self._running:
            return "not_running"
        if not self._received:
            return "waiting_for_navd_navigation_state"
        if (
            self._nav_received_monotonic <= 0.0
            or time.monotonic() - self._nav_received_monotonic
            > self._navigation_state_max_age_s
        ):
            return "navd_navigation_state_stale"
        map_reason = self._map_readiness_reason()
        if map_reason is not None:
            return map_reason
        return None

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(1.0, self._poll_period_s * 4.0))
        self._thread = None
        super().stop()

    def _poll(self) -> None:
        assert self._session is not None
        while not self._stop_event.wait(self._poll_period_s):
            try:
                self._poll_navigation_state()
                self._drain_navigation_goal_status()
                if self._inspection_task_events_enabled:
                    self._drain_inspection_task_events()
                self._poll_paths()
                if self._map_scene_enabled:
                    self._poll_map_scene()
            except Exception as exc:
                self._failure = f"native_host_bus_failed:{exc}"
                self._running = False
                logger.exception("HostBus native data loop failed")
                return

    def _poll_navigation_state(self) -> None:
        assert self._session is not None
        payload = self._session.read_navigation_state()
        if payload is None:
            return
        cursor = (str(payload["boot_id"]), int(payload["sequence"]))
        if (
            self._last_cursor is not None
            and cursor[0] == self._last_cursor[0]
            and cursor[1] <= self._last_cursor[1]
        ):
            self._nav_discarded += 1
            return
        state = NavigationState(
            ts=float(payload["timestamp_s"]),
            frame_id=str(payload["frame_id"]),
            boot_id=cursor[0],
            sequence=cursor[1],
            control_mode=int(payload["control_mode"]),
            lifecycle_state=int(payload["lifecycle_state"]),
            active_task_id=str(payload["active_task_id"]),
            active_request_id=str(payload["active_request_id"]),
            goal_epoch=int(payload["goal_epoch"]),
            map_id=str(payload["map_id"]),
            map_version=int(payload["map_version"]),
            map_hash=str(payload["map_hash"]),
            planning_state=int(payload["planning_state"]),
            execution_state=int(payload["execution_state"]),
            recovery_state=int(payload["recovery_state"]),
            progress=float(payload["progress"]),
            authority=str(payload["authority"]),
            hold_reason=str(payload["hold_reason"]),
            failure_code=str(payload["failure_code"]),
        )
        self._last_cursor = cursor
        self._received = True
        self._nav_received_monotonic = time.monotonic()
        self._nav_error = ""
        self.navigation_state.publish(state)

    def _drain_navigation_goal_status(self) -> None:
        assert self._session is not None
        for _ in range(self._goal_status_drain_limit):
            payload = self._session.take_navigation_goal_status()
            if payload is None:
                return
            boot_id = str(payload["boot_id"])
            sequence = int(payload["sequence"])
            with self._goal_status_lock:
                if sequence <= self._goal_status_sequences.get(boot_id, 0):
                    continue
                self._goal_status_sequences[boot_id] = sequence
            status = NavigationGoalStatus(
                ts=float(payload["timestamp_s"]),
                frame_id=str(payload["frame_id"]),
                boot_id=boot_id,
                sequence=sequence,
                task_id=str(payload["task_id"]),
                request_id=str(payload["request_id"]),
                state=int(payload["state"]),
                goal_epoch=int(payload["goal_epoch"]),
                reason=str(payload["reason"]),
            )
            with self._goal_status_lock:
                self._goal_status_by_task.pop(status.task_id, None)
                self._goal_status_by_task[status.task_id] = status
                self._goal_status_by_request.pop(status.request_id, None)
                self._goal_status_by_request[status.request_id] = status
                while len(self._goal_status_by_task) > self._goal_status_retention:
                    self._goal_status_by_task.popitem(last=False)
                while (
                    len(self._goal_status_by_request)
                    > self._goal_status_retention
                ):
                    self._goal_status_by_request.popitem(last=False)
            self.navigation_goal_status.publish(status)

    def _drain_inspection_task_events(self) -> None:
        """Project ordered native inspection facts without deriving task state."""

        assert self._session is not None
        for _ in range(self._inspection_task_event_drain_limit):
            payload = self._session.take_inspection_task_event()
            if payload is None:
                return
            try:
                event = InspectionTaskEvent(
                    ts=float(payload["timestamp_s"]),
                    frame_id=str(payload["frame_id"]),
                    boot_id=str(payload["boot_id"]),
                    event_sequence=int(payload["event_sequence"]),
                    kind=int(payload["kind"]),
                    task_id=str(payload["task_id"]),
                    request_id=str(payload["request_id"]),
                    command_request_id=str(payload["command_request_id"]),
                    state=int(payload["state"]),
                    map_id=str(payload["map_id"]),
                    map_version=int(payload["map_version"]),
                    route_id=str(payload["route_id"]),
                    route_revision=int(payload["route_revision"]),
                    point_index=int(payload["point_index"]),
                    point_count=int(payload["point_count"]),
                    loop_index=int(payload["loop_index"]),
                    retry_count=int(payload["retry_count"]),
                    point_id=str(payload["point_id"]),
                    action=str(payload["action"]),
                    action_request_id=str(payload["action_request_id"]),
                    evidence_id=str(payload["evidence_id"]),
                    reason=str(payload["reason"]),
                )
            except (KeyError, TypeError, ValueError) as exc:
                self._inspection_task_event_discarded += 1
                self._inspection_task_event_error = f"invalid_native_inspection_task_event:{exc}"
                continue
            sequence = int(event.event_sequence)
            boot_id = event.boot_id
            if sequence <= self._inspection_task_event_sequences.get(boot_id, 0):
                self._inspection_task_event_discarded += 1
                continue
            self._inspection_task_event_sequences[boot_id] = sequence
            self._inspection_task_event_received += 1
            self._inspection_task_event_error = ""
            self.inspection_task_event.publish(event)

    def _poll_paths(self) -> None:
        assert self._session is not None
        self._publish_path(self._session.take_global_path(), self.global_path)
        self._publish_path(self._session.take_local_path(), self.local_path)

    def _poll_map_scene(self) -> None:
        assert self._session is not None
        payload = self._session.take_map_scene()
        health = self._session.read_map_scene_health()
        self._map_health = dict(health)
        valid_states = int(health.get("state_valid_samples", 0) or 0)
        if valid_states > self._map_state_valid_samples:
            self._map_state_valid_samples = valid_states
            self._map_state_received_monotonic = time.monotonic()
        if int(health.get("capacity_rejections", 0) or 0) > 0:
            self._scene_error = "native_map_scene_capacity_rejected"
        elif str(health.get("state_error") or ""):
            self._scene_error = (
                "native_map_state_error:"
                + str(health.get("state_error") or "")
            )
        else:
            self._scene_error = ""
        if payload is None:
            return

        producer_boot_id = str(payload.get("producer_boot_id") or "")
        receive_sequence = int(payload.get("receive_sequence", 0) or 0)
        cursor = (producer_boot_id, receive_sequence)
        if (
            not producer_boot_id
            or receive_sequence <= 0
            or (
                self._scene_cursor is not None
                and cursor[0] == self._scene_cursor[0]
                and cursor[1] <= self._scene_cursor[1]
            )
        ):
            self._scene_discarded += 1
            return
        frame = self._map_scene_frame(payload)
        self._scene_cursor = cursor
        self._scene_received = True
        self._scene_received_monotonic = time.monotonic()
        self._scene_source_timestamp_s = float(payload["timestamp_s"])
        self._scene_live = bool(payload["live"])
        self._scene_reset_epoch = int(payload["reset_epoch"])
        self._scene_observation_sequence = int(
            payload["observation_sequence"]
        )
        self._scene_generation = int(payload["generation"])
        self.map_scene.publish(frame)

    @staticmethod
    def _map_scene_frame(payload: dict[str, object]) -> MapSceneFrame:
        if not numpy_import_is_safe():
            raise RuntimeError("numpy is unavailable for native MapScene projection")
        timestamp_s = float(payload["timestamp_s"])
        frame_id = str(payload["frame_id"])
        generation = int(payload["generation"])
        reset_epoch = int(payload["reset_epoch"])
        observation_sequence = int(payload["observation_sequence"])
        receive_sequence = int(payload["receive_sequence"])
        common = {
            "reset_epoch": reset_epoch,
            "observation_sequence": observation_sequence,
            "generation": generation,
            "live": bool(payload["live"]),
        }
        raw_clouds = payload.get("clouds")
        raw_grids = payload.get("grids")
        if not isinstance(raw_clouds, dict) or not isinstance(raw_grids, dict):
            raise RuntimeError("native MapScene payload is missing layers")

        cloud_topics = {
            "live": TOPICS.maps_live_cloud,
            "voxel": TOPICS.maps_voxel_cloud,
            "accumulated": TOPICS.maps_accumulated_cloud,
        }
        layers: list[dict[str, Any]] = []
        for name, topic in cloud_topics.items():
            cloud = raw_clouds.get(name)
            if not isinstance(cloud, dict):
                raise RuntimeError(f"native MapScene cloud {name} is invalid")
            point_count = int(cloud.get("point_count", 0) or 0)
            raw_points = cloud.get("points_xyzi_f32")
            if not isinstance(raw_points, bytes) or len(raw_points) != point_count * 16:
                raise RuntimeError(
                    f"native MapScene cloud {name} byte length is invalid"
                )
            points = np.frombuffer(raw_points, dtype="<f4").reshape(
                point_count, 4
            )
            layers.append(
                {
                    "id": f"maps.{name}_cloud",
                    "type": "pointcloud",
                    "topic": topic,
                    "source": "mapd",
                    "semantic": (
                        "current_incremental_scan"
                        if name == "live"
                        else "live_voxel_map"
                        if name == "voxel"
                        else "accumulated_occupancy_points"
                    ),
                    "point_count": point_count,
                    "metadata": dict(common),
                    "payload": PointCloud2(
                        points=points,
                        ts=timestamp_s,
                        frame_id=frame_id,
                    ),
                }
            )

        grid_topics = {
            "occupancy": TOPICS.maps_occupancy,
            "elevation": TOPICS.maps_elevation,
            "esdf": TOPICS.maps_esdf,
            "traversability": TOPICS.maps_traversability,
        }
        for name, topic in grid_topics.items():
            grid = raw_grids.get(name)
            if not isinstance(grid, dict):
                raise RuntimeError(f"native MapScene grid {name} is invalid")
            width = int(grid.get("width", 0) or 0)
            height = int(grid.get("height", 0) or 0)
            cell_count = int(grid.get("cell_count", 0) or 0)
            raw_values = grid.get("values_f32")
            if (
                width * height != cell_count
                or not isinstance(raw_values, bytes)
                or len(raw_values) != cell_count * 4
            ):
                raise RuntimeError(
                    f"native MapScene grid {name} byte length is invalid"
                )
            values = np.frombuffer(raw_values, dtype="<f4").reshape(
                height, width
            )
            origin = grid.get("origin")
            if not isinstance(origin, dict):
                raise RuntimeError(
                    f"native MapScene grid {name} origin is invalid"
                )
            layers.append(
                {
                    "id": f"maps.{name}",
                    "type": "grid",
                    "topic": topic,
                    "source": "mapd",
                    "metadata": {
                        **common,
                        "width": width,
                        "height": height,
                        "resolution": float(grid["resolution"]),
                        "origin": dict(origin),
                    },
                    "grid": values,
                }
            )
        return MapSceneFrame(
            layers=layers,
            ts=timestamp_s,
            frame_id=frame_id,
            source="mapd",
            sequence=receive_sequence,
            metadata={
                **common,
                "producer_boot_id": str(payload["producer_boot_id"]),
                "receive_sequence": receive_sequence,
                "payload_bytes": int(payload["payload_bytes"]),
                "sensor_pose": dict(payload.get("sensor_pose") or {}),
            },
        )

    def _map_readiness_reason(self) -> str | None:
        if not self._require_map_scene:
            return None
        if not self._map_scene_enabled:
            return "map_scene_abi_unavailable"
        health = self._map_health
        if int(health.get("capacity_rejections", 0) or 0) > 0:
            return "map_scene_capacity_rejected"
        if int(health.get("state_invalid_samples", 0) or 0) > 0:
            return "map_state_invalid_sample"
        if str(health.get("state_error") or ""):
            return "map_state_error"
        if not bool(health.get("state_received")):
            return "waiting_for_mapd_state"
        now_monotonic = time.monotonic()
        now_wall = time.time()
        if self._map_state_received_monotonic <= 0.0:
            return "waiting_for_fresh_mapd_state"
        if (
            now_monotonic - self._map_state_received_monotonic
            > self._map_scene_max_age_s
        ):
            return "mapd_state_stale"
        state_timestamp_s = float(health.get("state_timestamp_s", 0.0) or 0.0)
        if (
            not math.isfinite(state_timestamp_s)
            or state_timestamp_s <= 0.0
            or now_wall - state_timestamp_s > self._map_scene_max_age_s
            or state_timestamp_s - now_wall
            > self._map_scene_future_tolerance_s
        ):
            return "mapd_state_timestamp_stale"
        if not bool(health.get("state_running")):
            return "mapd_not_running"
        if not bool(health.get("state_live")):
            return "mapd_not_live"
        if bool(health.get("state_capacity_limited")):
            return "mapd_capacity_limited"
        if not bool(health.get("state_required_publications_ready")):
            return "mapd_required_publications_pending"
        if not bool(health.get("state_current_generation_published")):
            return "mapd_current_generation_pending"
        if not self._scene_received or self._scene_cursor is None:
            return "waiting_for_mapd_scene"
        if self._scene_error:
            return self._scene_error
        if (
            now_monotonic - self._scene_received_monotonic
            > self._map_scene_max_age_s
        ):
            return "map_scene_stale"
        if (
            not math.isfinite(self._scene_source_timestamp_s)
            or self._scene_source_timestamp_s <= 0.0
            or now_wall - self._scene_source_timestamp_s
            > self._map_scene_max_age_s
            or self._scene_source_timestamp_s - now_wall
            > self._map_scene_future_tolerance_s
        ):
            return "map_scene_timestamp_stale"
        if not self._scene_live:
            return "map_scene_not_live"
        state_boot_id = str(health.get("state_producer_boot_id") or "")
        state_epoch = int(health.get("state_reset_epoch", 0) or 0)
        state_observation = int(
            health.get("state_observation_sequence", 0) or 0
        )
        state_generation = int(health.get("state_generation", 0) or 0)
        state_scene_generation = int(
            health.get("state_scene_published_generation", 0) or 0
        )
        if self._scene_cursor[0] != state_boot_id:
            return "map_scene_boot_mismatch"
        if self._scene_reset_epoch != state_epoch:
            return "map_scene_epoch_mismatch"
        if (
            self._scene_observation_sequence != state_observation
            or self._scene_generation != state_generation
            or self._scene_generation != state_scene_generation
        ):
            return "map_scene_generation_pending"
        return None

    def health(self) -> dict[str, object]:
        """Expose independent nav and map receive evidence for Gateway."""

        now = time.monotonic()
        readiness = self.startup_readiness()
        return {
            "ok": readiness is None,
            "running": bool(self._running),
            "readiness": readiness or "ready",
            "failure": self._failure,
            "navigation": {
                "received": self._received,
                "max_age_s": self._navigation_state_max_age_s,
                "cursor": list(self._last_cursor)
                if self._last_cursor is not None
                else None,
                "age_s": (
                    max(0.0, now - self._nav_received_monotonic)
                    if self._nav_received_monotonic > 0.0
                    else None
                ),
                "discarded": self._nav_discarded,
                "error": self._nav_error,
            },
            "map_scene": {
                "required": self._require_map_scene,
                "enabled": self._map_scene_enabled,
                "received": self._scene_received,
                "cursor": list(self._scene_cursor)
                if self._scene_cursor is not None
                else None,
                "age_s": (
                    max(0.0, now - self._scene_received_monotonic)
                    if self._scene_received_monotonic > 0.0
                    else None
                ),
                "live": self._scene_live,
                "reset_epoch": self._scene_reset_epoch,
                "observation_sequence": self._scene_observation_sequence,
                "generation": self._scene_generation,
                "discarded": self._scene_discarded,
                "error": self._scene_error,
                "native": dict(self._map_health),
            },
            "inspection_task_events": {
                "required": self._require_inspection_task_events,
                "enabled": self._inspection_task_events_enabled,
                "received": self._inspection_task_event_received,
                "cursors": dict(self._inspection_task_event_sequences),
                "discarded": self._inspection_task_event_discarded,
                "error": self._inspection_task_event_error,
            },
        }

    def map_readiness(self) -> str | None:
        """Return the field mapd state/scene blocker independently of navd."""

        return self._map_readiness_reason()

    @staticmethod
    def _publish_path(payload: dict[str, object] | None, output: Out[Path]) -> None:
        if payload is None:
            return
        timestamp_s = float(payload["timestamp_s"])
        frame_id = str(payload["frame_id"])
        raw_points = payload.get("points")
        if not isinstance(raw_points, list):
            raise RuntimeError("native path telemetry points must be a list")
        poses: list[PoseStamped] = []
        for point in raw_points:
            if not isinstance(point, dict):
                raise RuntimeError("native path telemetry point must be an object")
            poses.append(
                PoseStamped(
                    pose=Pose(
                        position=Vector3(
                            x=float(point["x"]),
                            y=float(point["y"]),
                            z=float(point["z"]),
                        )
                    ),
                    ts=timestamp_s,
                    frame_id=frame_id,
                )
            )
        output.publish(Path(poses=poses, ts=timestamp_s, frame_id=frame_id))

    def goal_status(self, request_id: str) -> NavigationGoalStatus | None:
        """Return the retained latest status for a request without consuming it."""

        with self._goal_status_lock:
            return self._goal_status_by_request.get(str(request_id or ""))

    def task_status(self, task_id: str) -> NavigationGoalStatus | None:
        """Return the retained latest status for a logical task without consuming it."""

        with self._goal_status_lock:
            return self._goal_status_by_task.get(str(task_id or ""))


__all__ = ["HostBus"]
