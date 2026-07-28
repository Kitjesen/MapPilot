from __future__ import annotations

import struct
import time

import pytest

import lingtu.host_bus as host_bus_module
from lingtu.host_bus import HostBus


def _grid(
    *,
    width: int = 0,
    height: int = 0,
    values: bytes = b"",
) -> dict[str, object]:
    return {
        "width": width,
        "height": height,
        "resolution": 0.1,
        "origin": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        },
        "cell_count": width * height,
        "values_f32": values,
    }


def _map_scene_payload(
    timestamp_s: float,
    *,
    generation: int = 5,
) -> dict[str, object]:
    return {
        "timestamp_s": timestamp_s,
        "frame_id": "map",
        "producer_boot_id": "mapd-boot",
        "receive_sequence": 7,
        "reset_epoch": 3,
        "observation_sequence": 11,
        "generation": generation,
        "live": True,
        "sensor_pose": {
            "x": 1.0,
            "y": 2.0,
            "z": 0.5,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        },
        "payload_bytes": 24,
        "clouds": {
            "live": {
                "point_count": 1,
                "points_xyzi_f32": struct.pack("<4f", 1.0, 2.0, 3.0, 4.0),
            },
            "voxel": {"point_count": 0, "points_xyzi_f32": b""},
            "accumulated": {"point_count": 0, "points_xyzi_f32": b""},
        },
        "grids": {
            "occupancy": _grid(
                width=2,
                height=1,
                values=struct.pack("<2f", 0.25, 0.75),
            ),
            "elevation": _grid(),
            "esdf": _grid(),
            "traversability": _grid(),
        },
    }


def _map_scene_health(
    timestamp_s: float,
    *,
    generation: int = 5,
    capacity_rejections: int = 0,
) -> dict[str, object]:
    return {
        "received_samples": 1,
        "valid_samples": 1,
        "stale_samples": 0,
        "invalid_samples": 0,
        "capacity_rejections": capacity_rejections,
        "replaced_samples": 0,
        "consumer_buffer_retries": 1,
        "last_receive_sequence": 7,
        "last_generation": generation,
        "last_sample_timestamp_s": timestamp_s,
        "pending": False,
        "last_error": "",
        "state_received_samples": 1,
        "state_valid_samples": 1,
        "state_stale_samples": 0,
        "state_invalid_samples": 0,
        "state_timestamp_s": timestamp_s,
        "state_producer_boot_id": "mapd-boot",
        "state_received": True,
        "state_running": True,
        "state_live": True,
        "state_required_publications_ready": True,
        "state_current_generation_published": True,
        "state_capacity_limited": False,
        "state_reset_epoch": 3,
        "state_observation_sequence": 11,
        "state_generation": generation,
        "state_scene_published_generation": generation,
        "state_error": "",
    }


def test_host_bus_projects_native_navigation_state() -> None:
    bus = HostBus(poll_period_s=0.01)
    observed = []
    bus.navigation_state._add_callback(observed.append)

    class Session:
        def read_navigation_state(self):
            bus._stop_event.set()
            return {
                "timestamp_s": 42.0,
                "frame_id": "map",
                "boot_id": "navd-boot",
                "sequence": 8,
                "control_mode": 1,
                "lifecycle_state": 2,
                "active_task_id": "navigation-task-7",
                "active_request_id": "goal-7",
                "goal_epoch": 7,
                "map_id": "site-a",
                "map_version": 3,
                "map_hash": "abc",
                "planning_state": 2,
                "execution_state": 1,
                "recovery_state": 0,
                "progress": 0.25,
                "authority": "autonomy",
                "hold_reason": "",
                "failure_code": "",
            }

        def take_navigation_goal_status(self):
            return None

        def take_global_path(self):
            return None

        def take_local_path(self):
            return None

    bus._session = Session()
    bus._poll()

    assert len(observed) == 1
    assert observed[0].boot_id == "navd-boot"
    assert observed[0].sequence == 8
    assert observed[0].active_task_id == "navigation-task-7"
    assert observed[0].to_dict()["lifecycle_state_name"] == "EXECUTING"
    assert bus.startup_readiness() == "not_running"


def test_host_bus_deduplicates_and_retains_goal_status_by_request() -> None:
    bus = HostBus(goal_status_retention=2)
    observed = []
    bus.navigation_goal_status._add_callback(observed.append)
    pending = [
        {
            "timestamp_s": 43.0,
            "frame_id": "map",
            "boot_id": "navd-boot",
            "sequence": 1,
            "task_id": "navigation-task-7",
            "request_id": "goal-7",
            "state": 1,
            "goal_epoch": 7,
            "reason": "planning",
        },
        {
            "timestamp_s": 43.1,
            "frame_id": "map",
            "boot_id": "navd-boot",
            "sequence": 1,
            "task_id": "navigation-task-7",
            "request_id": "goal-7",
            "state": 3,
            "goal_epoch": 7,
            "reason": "duplicate",
        },
        {
            "timestamp_s": 44.0,
            "frame_id": "map",
            "boot_id": "navd-boot",
            "sequence": 2,
            "task_id": "navigation-task-7",
            "request_id": "goal-7",
            "state": 4,
            "goal_epoch": 7,
            "reason": "goal_reached",
        },
    ]

    class Session:
        def take_navigation_goal_status(self):
            return pending.pop(0) if pending else None

    bus._session = Session()
    bus._drain_navigation_goal_status()

    assert [status.sequence for status in observed] == [1, 2]
    retained = bus.goal_status("goal-7")
    assert retained is not None
    assert retained.terminal is True
    assert retained.to_dict()["state_name"] == "REACHED"
    task_retained = bus.task_status("navigation-task-7")
    assert task_retained is retained


def test_host_bus_projects_ordered_native_inspection_task_events() -> None:
    bus = HostBus(require_inspection_task_events=True)
    observed = []
    bus.inspection_task_event._add_callback(observed.append)
    pending = [
        {
            "timestamp_s": 45.0,
            "frame_id": "map",
            "boot_id": "navd-boot",
            "event_sequence": 1,
            "kind": 1,
            "task_id": "inspection-task-7",
            "request_id": "inspection-start-7",
            "command_request_id": "inspection-start-7",
            "state": 2,
            "map_id": "field-map",
            "map_version": 3,
            "route_id": "route-a",
            "route_revision": 7,
            "point_index": 0,
            "point_count": 2,
            "loop_index": 0,
            "retry_count": 0,
            "point_id": "dock",
            "action": "capture:overview",
            "action_request_id": "",
            "evidence_id": "",
            "reason": "planning_route",
        },
        {
            "timestamp_s": 45.1,
            "frame_id": "map",
            "boot_id": "navd-boot",
            "event_sequence": 1,
            "kind": 2,
            "task_id": "inspection-task-7",
            "request_id": "inspection-start-7",
            "command_request_id": "inspection-start-7",
            "state": 9,
            "map_id": "field-map",
            "map_version": 3,
            "route_id": "route-a",
            "route_revision": 7,
            "point_index": 0,
            "point_count": 2,
            "loop_index": 0,
            "retry_count": 0,
            "point_id": "dock",
            "action": "capture:overview",
            "action_request_id": "",
            "evidence_id": "",
            "reason": "duplicate_must_not_replace_truth",
        },
    ]

    class Session:
        def take_inspection_task_event(self):
            return pending.pop(0) if pending else None

    bus._session = Session()
    bus._inspection_task_events_enabled = True
    bus._drain_inspection_task_events()

    assert len(observed) == 1
    assert observed[0].task_id == "inspection-task-7"
    assert observed[0].state_name == "PLANNING"
    assert observed[0].terminal is False


def test_host_bus_projects_native_global_and_local_paths() -> None:
    bus = HostBus()
    global_paths = []
    local_paths = []
    bus.global_path._add_callback(global_paths.append)
    bus.local_path._add_callback(local_paths.append)

    class Session:
        def take_global_path(self):
            return {
                "timestamp_s": 50.0,
                "frame_id": "map",
                "receive_sequence": 1,
                "points": [
                    {"x": 1.0, "y": 2.0, "z": 0.1},
                    {"x": 3.0, "y": 4.0, "z": 0.2},
                ],
            }

        def take_local_path(self):
            return {
                "timestamp_s": 50.1,
                "frame_id": "map",
                "receive_sequence": 2,
                "points": [{"x": 0.2, "y": 0.3, "z": 0.0}],
            }

    bus._session = Session()
    bus._poll_paths()

    assert len(global_paths) == 1
    assert len(global_paths[0].poses) == 2
    assert global_paths[0].poses[1].z == 0.2
    assert len(local_paths) == 1
    assert local_paths[0].poses[0].x == 0.2


def test_host_bus_projects_map_scene_only_after_aligned_mapd_state() -> None:
    timestamp_s = time.time()
    bus = HostBus(require_map_scene=True, map_scene_max_age_s=30.0)
    observed = []
    bus.map_scene._add_callback(observed.append)

    class Session:
        def take_map_scene(self):
            return _map_scene_payload(timestamp_s)

        def read_map_scene_health(self):
            return _map_scene_health(timestamp_s)

    bus._session = Session()
    bus._map_scene_enabled = True
    bus._poll_map_scene()

    assert bus.map_readiness() is None
    assert len(observed) == 1
    assert observed[0].source == "mapd"
    assert observed[0].sequence == 7
    assert len(observed[0].layers) == 7
    assert observed[0].layers[0]["point_count"] == 1
    occupancy = next(
        layer
        for layer in observed[0].layers
        if layer["id"] == "maps.occupancy"
    )
    assert occupancy["grid"].shape == (1, 2)
    health = bus.health()["map_scene"]
    assert health["cursor"] == ["mapd-boot", 7]
    assert health["generation"] == 5
    assert health["native"]["consumer_buffer_retries"] == 1


def test_host_bus_map_readiness_rejects_generation_mismatch_and_capacity() -> None:
    timestamp_s = time.time()
    bus = HostBus(require_map_scene=True, map_scene_max_age_s=2.0)

    class Session:
        def __init__(self):
            self.capacity_rejections = 0

        def take_map_scene(self):
            return _map_scene_payload(timestamp_s, generation=4)

        def read_map_scene_health(self):
            return _map_scene_health(
                timestamp_s,
                generation=5,
                capacity_rejections=self.capacity_rejections,
            )

    session = Session()
    bus._session = session
    bus._map_scene_enabled = True
    bus._poll_map_scene()

    assert bus.map_readiness() == "map_scene_generation_pending"

    session.capacity_rejections = 1
    bus._poll_map_scene()
    assert bus.map_readiness() == "map_scene_capacity_rejected"


def test_host_bus_navigation_state_staleness_blocks_readiness(monkeypatch) -> None:
    bus = HostBus(navigation_state_max_age_s=1.5)
    bus._running = True
    bus._received = True
    bus._nav_received_monotonic = 100.0
    now = [101.4]
    monkeypatch.setattr(host_bus_module.time, "monotonic", lambda: now[0])

    assert bus.startup_readiness() is None

    now[0] = 101.6
    assert bus.startup_readiness() == "navd_navigation_state_stale"
    health = bus.health()
    assert health["ok"] is False
    assert health["readiness"] == "navd_navigation_state_stale"
    assert health["navigation"]["max_age_s"] == 1.5
    assert health["navigation"]["age_s"] == pytest.approx(1.6)
