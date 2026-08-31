from __future__ import annotations

import asyncio
import base64
import json
import math

from gateway.services import cloud_viewer as cloud_viewer_module
from gateway.services.cloud_viewer import CloudViewerService
from gateway.services.sse import subscribe, subscribe_with_event_id, unsubscribe
from runtime.msgs.map import MapSceneFrame
from runtime.msgs.numpy_compat import np


def _queue_put_latest(q, buf, loop, record_delivery):
    try:
        q.put_nowait(buf)
        record_delivery(False, q.qsize())
    except asyncio.QueueFull:
        _ = q.get_nowait()
        q.put_nowait(buf)
        record_delivery(True, q.qsize())


def _service(events: list[dict]) -> CloudViewerService:
    return CloudViewerService(
        queue_put_latest=_queue_put_latest,
        current_loop=lambda: None,
        push_event=events.append,
        session_mode=lambda: "navigating",
        active_session_map=lambda: "field",
        saved_active_map=lambda: "field",
    )


def test_map_scene_elevation_emits_little_endian_float_grid_with_invalid_nan() -> None:
    events: list[dict] = []
    service = _service(events)
    source_stamp_s = 123.5
    grid = np.array([[-1.25, np.nan], [np.inf, 3.5]], dtype=np.float64)

    service.on_map_scene(
        MapSceneFrame(
            ts=source_stamp_s,
            frame_id="map",
            source="mapd",
            sequence=9,
            metadata={"producer_boot_id": "mapd-boot-a"},
            layers=[
                {
                    "id": "maps.elevation",
                    "type": "grid",
                    "source": "mapd",
                    "grid": grid,
                    "metadata": {
                        "generation": 7,
                        "reset_epoch": 3,
                        "observation_sequence": 11,
                        "width": 2,
                        "height": 2,
                        "resolution": 0.2,
                        "origin": {
                            "x": 1.0,
                            "y": -2.0,
                            "z": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": math.sin(math.pi / 4.0),
                            "qw": math.cos(math.pi / 4.0),
                        },
                    },
                }
            ],
        )
    )

    assert events[-1]["ts"] == source_stamp_s
    layer = events[-1]["layers"][0]
    decoded = np.frombuffer(base64.b64decode(layer["grid_b64"]), dtype="<f4").reshape(2, 2)

    assert layer["encoding"] == "float32_le"
    assert layer["rows"] == 2
    assert layer["cols"] == 2
    assert layer["resolution"] == 0.2
    assert layer["origin"] == [1.0, -2.0, 0.0]
    assert math.isclose(layer["yaw"], math.pi / 2.0)
    assert layer["producer_boot_id"] == "mapd-boot-a"
    assert layer["frame_id"] == "map"
    assert layer["stamp_s"] == source_stamp_s
    assert layer["source_stamp_s"] == source_stamp_s
    assert layer["age_s_at_emit"] >= 0.0
    assert layer["generation"] == 7
    assert layer["reset_epoch"] == 3
    assert layer["observation_sequence"] == 11
    assert layer["semantic"] == "min_observed_z_not_ground"
    assert layer["value_semantics"] == "min_observed_z_not_ground"
    assert layer["valid_count"] == 2
    assert layer["min_z"] == -1.25
    assert layer["max_z"] == 3.5
    assert layer["downsample_factor"] == 1
    assert layer["source_rows"] == 2
    assert layer["source_cols"] == 2
    assert decoded[0, 0] == np.float32(-1.25)
    assert np.isnan(decoded[0, 1])
    assert np.isnan(decoded[1, 0])
    assert decoded[1, 1] == np.float32(3.5)


def test_map_scene_elevation_payload_is_bounded_and_reports_downsampling() -> None:
    events: list[dict] = []
    service = _service(events)
    grid = np.full((513, 513), 5.0, dtype=np.float32)
    grid[0, 0] = 2.0
    grid[1, 1] = -7.0

    service.on_map_scene(
        MapSceneFrame(
            frame_id="map",
            source="mapd",
            metadata={"producer_boot_id": "mapd-boot-a"},
            layers=[
                {
                    "id": "maps.elevation",
                    "type": "grid",
                    "grid": grid,
                    "metadata": {
                        "generation": 8,
                        "width": 513,
                        "height": 513,
                        "resolution": 0.1,
                        "origin": {"x": 0.0, "y": 0.0, "qz": 0.0, "qw": 1.0},
                    },
                }
            ],
        )
    )

    layer = events[-1]["layers"][0]
    assert layer["source_rows"] == 513
    assert layer["source_cols"] == 513
    assert layer["downsample_factor"] == 2
    assert layer["rows"] == 257
    assert layer["cols"] == 257
    assert layer["rows"] * layer["cols"] <= 131_072
    assert layer["resolution"] == 0.2
    assert len(json.dumps(layer, separators=(",", ":")).encode("utf-8")) <= 1024 * 1024
    decoded = np.frombuffer(base64.b64decode(layer["grid_b64"]), dtype="<f4").reshape(257, 257)
    assert decoded[0, 0] == np.float32(-7.0)
    assert layer["min_z"] == -7.0


def test_map_scene_elevation_omits_unchanged_and_rate_limited_payloads(monkeypatch) -> None:
    now = [100.0]
    monkeypatch.setattr(cloud_viewer_module.time, "time", lambda: now[0])
    events: list[dict] = []
    service = _service(events)

    def emit(
        grid,
        *,
        generation: int,
        reset_epoch: int = 4,
        observation_sequence: int | None = None,
        source_stamp_s: float | None = None,
        producer_boot_id: str = "mapd-boot-a",
        resolution: float = 0.2,
    ) -> dict:
        stamp_s = now[0] if source_stamp_s is None else source_stamp_s
        service.on_map_scene(
            MapSceneFrame(
                ts=stamp_s,
                frame_id="map",
                source="mapd",
                metadata={"producer_boot_id": producer_boot_id},
                layers=[
                    {
                        "id": "maps.elevation",
                        "type": "grid",
                        "grid": grid,
                        "metadata": {
                            "generation": generation,
                            "reset_epoch": reset_epoch,
                            "observation_sequence": generation
                            if observation_sequence is None
                            else observation_sequence,
                            "width": 2,
                            "height": 2,
                            "resolution": resolution,
                            "origin": {"x": 0.0, "y": 0.0, "qz": 0.0, "qw": 1.0},
                        },
                    }
                ],
            )
        )
        return events[-1]["layers"][0]

    first_grid = np.array([[0.0, 1.0], [2.0, 3.0]], dtype=np.float32)
    first = emit(first_grid, generation=1)
    assert first["payload"] == "inline"
    assert first["grid_b64"]

    now[0] = 100.2
    unchanged = emit(
        first_grid.copy(),
        generation=1,
        observation_sequence=1,
        source_stamp_s=100.0,
    )
    assert "grid_b64" not in unchanged
    assert unchanged["payload"] == "omitted"
    assert unchanged["reason"] == "unchanged"
    assert unchanged["retain_previous"] is True
    assert unchanged["retention_scope"] == "same_elevation_cohort"

    now[0] = 100.3
    new_observation = emit(first_grid.copy(), generation=1, observation_sequence=2)
    assert new_observation["payload"] == "omitted"
    assert new_observation["reason"] == "rate_limited"

    changed_grid = first_grid.copy()
    changed_grid[0, 0] = -1.0
    now[0] = 100.4
    rate_limited = emit(changed_grid, generation=3)
    assert "grid_b64" not in rate_limited
    assert rate_limited["payload"] == "omitted"
    assert rate_limited["reason"] == "rate_limited"
    assert rate_limited["retain_previous"] is True

    now[0] = 100.6
    new_epoch = emit(changed_grid, generation=1, reset_epoch=5)
    assert new_epoch["payload"] == "omitted"
    assert new_epoch["reason"] == "rate_limited"
    assert new_epoch["retain_previous"] is False

    now[0] = 101.1
    refreshed = emit(changed_grid, generation=3)
    assert refreshed["payload"] == "inline"
    assert refreshed["grid_b64"]

    now[0] = 102.2
    unsafe = emit(np.array([0.0, 1.0], dtype=np.float32), generation=4)
    assert unsafe["payload"] == "omitted"
    assert unsafe["reason"] == "unsafe_elevation_payload"
    assert unsafe["retain_previous"] is False

    now[0] = 103.3
    recovered = emit(changed_grid, generation=4)
    assert recovered["payload"] == "inline"
    assert recovered["grid_b64"]


def test_elevation_cohort_uses_grid_shape_and_rejects_conflicting_declared_geometry(monkeypatch) -> None:
    now = [100.0]
    monkeypatch.setattr(cloud_viewer_module.time, "time", lambda: now[0])
    events: list[dict] = []
    service = _service(events)
    grid = np.arange(6, dtype=np.float32).reshape(2, 3)

    def emit(*, generation: int, declared_width: int | None = None) -> dict:
        metadata = {
            "generation": generation,
            "reset_epoch": 4,
            "observation_sequence": generation,
            "live": True,
            "resolution": 0.2,
            "origin": {"x": 0.0, "y": 0.0, "z": 0.0, "qw": 1.0},
        }
        if declared_width is not None:
            metadata.update({"width": declared_width, "height": 2})
        service.on_map_scene(
            MapSceneFrame(
                ts=now[0],
                frame_id="map",
                source="mapd",
                sequence=generation,
                metadata={"producer_boot_id": "mapd-boot-a"},
                layers=[
                    {
                        "id": "maps.elevation",
                        "type": "grid",
                        "grid": grid,
                        "metadata": metadata,
                    }
                ],
            )
        )
        return events[-1]["layers"][0]

    first = emit(generation=1)
    assert first["payload"] == "inline"
    assert (first["source_rows"], first["source_cols"]) == (2, 3)

    now[0] = 100.2
    retained = emit(generation=2)
    assert retained["payload"] == "omitted"
    assert retained["reason"] == "rate_limited"
    assert retained["retain_previous"] is True

    now[0] = 101.3
    conflicting = emit(generation=3, declared_width=99)
    assert conflicting["payload"] == "omitted"
    assert conflicting["reason"] == "unsafe_elevation_payload"
    assert conflicting["retain_previous"] is False


def test_map_scene_elevation_does_not_retain_across_producer_or_geometry_change(monkeypatch) -> None:
    now = [100.0]
    monkeypatch.setattr(cloud_viewer_module.time, "time", lambda: now[0])
    events: list[dict] = []
    service = _service(events)
    grid = np.array([[0.0, 1.0], [2.0, 3.0]], dtype=np.float32)

    def emit(*, producer_boot_id: str, resolution: float) -> dict:
        service.on_map_scene(
            MapSceneFrame(
                ts=now[0],
                frame_id="map",
                source="mapd",
                sequence=1,
                metadata={"producer_boot_id": producer_boot_id},
                layers=[
                    {
                        "id": "maps.elevation",
                        "type": "grid",
                        "grid": grid,
                        "metadata": {
                            "generation": 1,
                            "reset_epoch": 4,
                            "observation_sequence": 1,
                            "width": 2,
                            "height": 2,
                            "resolution": resolution,
                            "origin": {"x": 0.0, "y": 0.0, "z": 0.0, "qw": 1.0},
                        },
                    }
                ],
            )
        )
        return events[-1]["layers"][0]

    assert emit(producer_boot_id="mapd-boot-a", resolution=0.2)["payload"] == "inline"

    now[0] = 100.2
    restarted = emit(producer_boot_id="mapd-boot-b", resolution=0.2)
    assert restarted["payload"] == "omitted"
    assert restarted["reason"] == "rate_limited"
    assert restarted["retain_previous"] is False

    now[0] = 100.3
    resized = emit(producer_boot_id="mapd-boot-a", resolution=0.3)
    assert resized["payload"] == "omitted"
    assert resized["reason"] == "rate_limited"
    assert resized["retain_previous"] is False


def test_map_scene_does_not_inline_non_elevation_grids() -> None:
    events: list[dict] = []
    service = _service(events)

    service.on_map_scene(
        MapSceneFrame(
            frame_id="map",
            source="mapd",
            layers=[
                {
                    "id": "maps.occupancy",
                    "type": "grid",
                    "grid": np.ones((4, 4), dtype=np.float32),
                },
                {
                    "id": "maps.esdf",
                    "type": "grid",
                    "grid": np.ones((4, 4), dtype=np.float32),
                },
                {
                    "id": "custom.elevation",
                    "type": "elevation",
                    "grid": np.ones((4, 4), dtype=np.float32),
                },
            ],
        )
    )

    for layer in events[-1]["layers"]:
        assert "grid" not in layer
        assert "grid_b64" not in layer
        assert "payload" not in layer


def test_latest_elevation_is_replayed_to_new_and_reconnecting_sse_subscribers() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._on_map_scene(
        MapSceneFrame(
            ts=100.0,
            frame_id="map",
            source="mapd",
            sequence=7,
            metadata={
                "producer_boot_id": "mapd-boot-a",
                "generation": 3,
                "reset_epoch": 2,
                "observation_sequence": 9,
                "live": True,
            },
            layers=[
                {
                    "id": "maps.elevation",
                    "type": "grid",
                    "source": "mapd",
                    "grid": np.array([[0.0, 1.0], [2.0, 3.0]], dtype=np.float32),
                    "metadata": {
                        "generation": 3,
                        "reset_epoch": 2,
                        "observation_sequence": 9,
                        "live": True,
                        "width": 2,
                        "height": 2,
                        "resolution": 0.2,
                        "origin": {"x": 0.0, "y": 0.0, "z": 0.0, "qw": 1.0},
                    },
                },
                {
                    "id": "maps.occupancy",
                    "type": "grid",
                    "grid": np.ones((2, 2), dtype=np.float32),
                },
            ],
        )
    )

    first = subscribe(gateway, include_elevation_payload=True)
    first_replay = first.get_nowait()
    assert first_replay["type"] == "map_scene"
    assert [layer["id"] for layer in first_replay["layers"]] == ["maps.elevation"]
    assert first_replay["layers"][0]["payload"] == "inline"
    assert first_replay["layers"][0]["producer_boot_id"] == "mapd-boot-a"
    assert "event_id" not in first_replay
    first_grid = first_replay["layers"][0]["grid_b64"]
    unsubscribe(gateway, first)

    reconnect = subscribe(gateway, include_elevation_payload=True)
    reconnect_replay = reconnect.get_nowait()
    assert reconnect_replay["layers"][0]["grid_b64"] == first_grid
    assert "event_id" not in reconnect_replay
    unsubscribe(gateway, reconnect)

    routed, snapshot_event_id = subscribe_with_event_id(gateway, include_elevation_payload=True)
    routed_replay = routed.get_nowait()
    assert snapshot_event_id > 0
    assert "event_id" not in routed_replay
    assert routed_replay["layers"][0]["grid_b64"] == first_grid
    unsubscribe(gateway, routed)

    gateway.push_event(
        {
            "type": "map_scene",
            "frame_id": "map",
            "layers": [
                {
                    "id": "maps.elevation",
                    "type": "grid",
                    "payload": "omitted",
                    "reason": "scene_reset",
                    "retain_previous": False,
                }
            ],
        }
    )
    after_reset = subscribe(gateway)
    assert after_reset.empty()


def test_map_scene_is_declared_in_the_public_sse_event_catalog() -> None:
    from gateway.services.traffic import SSE_EVENT_TYPES

    assert "map_scene" in SSE_EVENT_TYPES


def test_cloud_viewer_scene_reset_clears_cached_elevation_before_next_subscriber() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._on_map_scene(
        MapSceneFrame(
            ts=100.0,
            frame_id="map",
            source="mapd",
            sequence=7,
            metadata={
                "producer_boot_id": "mapd-boot-a",
                "generation": 3,
                "reset_epoch": 2,
                "observation_sequence": 9,
                "live": True,
            },
            layers=[
                {
                    "id": "maps.elevation",
                    "type": "grid",
                    "grid": np.zeros((2, 2), dtype=np.float32),
                    "metadata": {
                        "generation": 3,
                        "reset_epoch": 2,
                        "observation_sequence": 9,
                        "live": True,
                        "width": 2,
                        "height": 2,
                        "resolution": 0.2,
                        "origin": {"x": 0.0, "y": 0.0, "z": 0.0, "qw": 1.0},
                    },
                }
            ],
        )
    )

    gateway._cloud_viewer.clear("test_scene_reset")

    subscriber = subscribe(gateway)
    assert subscriber.empty()


def test_private_elevation_replay_does_not_create_global_sse_sequence_gaps() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    observer = subscribe(gateway)
    gateway.push_event(
        {
            "type": "map_scene",
            "frame_id": "map",
            "metadata": {"producer_boot_id": "mapd-boot-a"},
            "layers": [
                {
                    "id": "maps.elevation",
                    "type": "grid",
                    "frame_id": "map",
                    "producer_boot_id": "mapd-boot-a",
                    "reset_epoch": 1,
                    "rows": 2,
                    "cols": 2,
                    "resolution": 0.2,
                    "origin": [0.0, 0.0, 0.0],
                    "yaw": 0.0,
                    "downsample_factor": 1,
                    "grid_b64": base64.b64encode(np.zeros((2, 2), dtype="<f4").tobytes()).decode("ascii"),
                }
            ],
        }
    )
    first_broadcast = observer.get_nowait()
    assert first_broadcast["event_id"] == 1

    newcomer = subscribe(gateway)
    replay = newcomer.get_nowait()
    assert "event_id" not in replay
    assert "grid_b64" not in replay["layers"][0]
    assert replay["layers"][0]["reason"] == "client_not_subscribed"
    assert gateway._traffic_stats_snapshot()["sse"]["latest_event_id"] == 1

    gateway.push_event({"type": "later"})
    assert observer.get_nowait()["event_id"] == 2
    assert newcomer.get_nowait()["event_id"] == 2


def test_sse_elevation_payload_is_per_subscriber_and_metadata_remains_visible() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    metadata_only = subscribe(gateway, include_elevation_payload=False)
    opted_in = subscribe(gateway, include_elevation_payload=True)
    grid_b64 = base64.b64encode(np.zeros((2, 2), dtype="<f4").tobytes()).decode("ascii")

    gateway.push_event(
        {
            "type": "map_scene",
            "frame_id": "map",
            "layers": [
                {
                    "id": "maps.elevation",
                    "type": "grid",
                    "frame_id": "map",
                    "rows": 2,
                    "cols": 2,
                    "resolution": 0.2,
                    "origin": [0.0, 0.0, 0.0],
                    "yaw": 0.0,
                    "grid_b64": grid_b64,
                }
            ],
        }
    )

    metadata_event = metadata_only.get_nowait()
    payload_event = opted_in.get_nowait()
    assert metadata_event["frame_id"] == "map"
    assert "grid_b64" not in metadata_event["layers"][0]
    assert metadata_event["layers"][0]["reason"] == "client_not_subscribed"
    assert payload_event["layers"][0]["grid_b64"] == grid_b64

    unsubscribe(gateway, metadata_only)
    unsubscribe(gateway, opted_in)


def test_native_traversability_is_separate_from_python_costmap() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    queue = subscribe(gateway)
    gateway._on_native_traversability(
        {
            "grid": np.array([[0, 50], [90, 100]], dtype=np.uint8),
            "frame_id": "map",
            "stamp_s": 123.5,
            "reset_epoch": 2,
            "sequence": 8,
            "resolution": 0.2,
            "origin": [1.0, -2.0, 0.0],
            "yaw": 0.0,
        }
    )
    event = queue.get_nowait()
    assert event["type"] == "native_traversability"
    assert event["source"] == "native_nav_client"
    assert event["control_authority"] is True
    assert event["value_semantics"] == "control_risk_0_100"
    assert np.frombuffer(base64.b64decode(event["grid_b64"]), dtype=np.uint8).tolist() == [0, 50, 90, 100]
    unsubscribe(gateway, queue)


def test_native_traversability_rejects_values_before_uint8_wrap() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    queue = subscribe(gateway, event_types={"native_traversability"})
    gateway._on_native_traversability(
        {
            "grid": np.array([[300.0]], dtype=np.float64),
            "frame_id": "map",
            "stamp_s": 123.5,
            "reset_epoch": 2,
            "sequence": 8,
            "resolution": 0.2,
            "origin": [0.0, 0.0, 0.0],
            "yaw": 0.0,
        }
    )
    assert queue.empty()
    unsubscribe(gateway, queue)


def test_sse_event_type_filter_does_not_consume_unrelated_events() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    path = subscribe(gateway, event_types={"local_path"}, include_elevation_payload=False)
    gateway.push_event({"type": "odometry", "x": 1.0, "y": 2.0})
    gateway.push_event({"type": "local_path", "points": [], "frame_id": "map"})
    event = path.get_nowait()
    assert event["type"] == "local_path"
    assert path.empty()
    unsubscribe(gateway, path)
