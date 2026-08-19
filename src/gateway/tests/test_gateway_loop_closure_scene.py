from __future__ import annotations

import asyncio

from gateway.services.cloud_viewer import CloudViewerService
from runtime.msgs.map import MapSceneFrame


def _queue_put_latest(queue, payload, loop, record_delivery) -> None:
    del loop
    try:
        queue.put_nowait(payload)
        record_delivery(False, queue.qsize())
    except asyncio.QueueFull:
        _ = queue.get_nowait()
        queue.put_nowait(payload)
        record_delivery(True, queue.qsize())


def test_map_scene_forwards_native_loop_validation_as_read_only_metadata() -> None:
    events: list[dict] = []
    service = CloudViewerService(
        queue_put_latest=_queue_put_latest,
        current_loop=lambda: None,
        push_event=events.append,
        session_mode=lambda: "mapping",
        product_session=lambda: "mapping",
        active_session_map=lambda: None,
        saved_active_map=lambda: None,
    )
    service.configure(
        map_voxel_size=0.05,
        cloud_viewer_min_interval_s=0.0,
        scan_viewer_min_interval_s=0.0,
    )
    identity = {
        "producer_boot_id": "slam-boot-a",
        "reset_epoch": 7,
        "observation_sequence": 42,
        "generation": 9,
    }
    constraints = [
        {
            "from_index": 1,
            "to_index": 12,
            "from": [1.0, 2.0, 0.3],
            "to": [4.0, -1.0, 0.5],
            "state": "accepted",
            "geometrically_verified": True,
            "rmse_m": 0.12,
        }
    ]

    service.on_map_scene(
        MapSceneFrame(
            frame_id="map",
            source="localization.loop_verifier",
            sequence=42,
            metadata=identity,
            layers=[
                {
                    "id": "localization.loop_constraints",
                    "type": "loop_constraints",
                    "frame_id": "map",
                    "stamp_s": 100.0,
                    **identity,
                    "online": True,
                    "identity_verified": True,
                    "constraint_semantics": "loop_closure_validation_v1",
                    "constraints": constraints,
                }
            ],
        )
    )

    assert events[-1]["type"] == "map_scene"
    assert events[-1]["frame_id"] == "map"
    layer = events[-1]["layers"][0]
    assert layer["id"] == "localization.loop_constraints"
    assert layer["constraints"] == constraints
    assert "payload" not in layer
