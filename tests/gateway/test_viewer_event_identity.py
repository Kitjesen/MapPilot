from __future__ import annotations

from threading import RLock
from types import SimpleNamespace

from gateway.services.event_handlers import handle_scene_graph
from gateway.services.viewer_events import handle_global_path, handle_local_path
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import Path


class FakeGateway:
    def __init__(self) -> None:
        self._state_lock = RLock()
        self._local_path_throttle = 0
        self._sg_throttle = 0
        self.events: list[dict] = []

    def push_event(self, event: dict) -> None:
        self.events.append(event)


def _path(*, frame_id: str = "map", ts: float = 12.5) -> Path:
    return Path(
        poses=[
            PoseStamped(
                pose=Pose(position=Vector3(x=1.0, y=2.0, z=0.25)),
                frame_id=frame_id,
                ts=ts,
            )
        ],
        frame_id=frame_id,
        ts=ts,
    )


def test_path_event_preserves_source_frame_and_stamp() -> None:
    gateway = FakeGateway()

    handle_global_path(gateway, _path(frame_id="odom"))

    assert gateway.events == [
        {
            "type": "global_path",
            "points": [{"x": 1.0, "y": 2.0, "z": 0.25, "frame_id": "odom", "ts": 12.5}],
            "frame_id": "odom",
            "stamp_s": 12.5,
            "receive_sequence": None,
        }
    ]


def test_empty_local_path_is_a_clear_event_even_when_throttled() -> None:
    gateway = FakeGateway()
    handle_local_path(gateway, Path(poses=[], frame_id="map", ts=20.0))

    assert gateway.events == [
        {
            "type": "local_path",
            "points": [],
            "frame_id": "map",
            "stamp_s": 20.0,
            "receive_sequence": None,
        }
    ]
    assert gateway._last_local_path == []


def test_scene_graph_empty_objects_clear_previous_markers() -> None:
    gateway = FakeGateway()
    object_graph = SimpleNamespace(
        frame_id="map",
        timestamp=30.0,
        objects=[
            SimpleNamespace(
                id="crate-1",
                label="crate",
                confidence=0.9,
                position_3d=SimpleNamespace(x=1.0, y=2.0, z=0.4),
            )
        ],
    )
    empty_graph = SimpleNamespace(frame_id="map", timestamp=31.0, objects=[])

    for _ in range(5):
        handle_scene_graph(gateway, object_graph)
    for _ in range(5):
        handle_scene_graph(gateway, empty_graph)

    assert gateway.events[0]["objects"] == [
        {
            "id": "crate-1",
            "label": "crate",
            "x": 1.0,
            "y": 2.0,
            "z": 0.4,
            "confidence": 0.9,
        }
    ]
    assert gateway.events[-1] == {
        "type": "scene_graph",
        "objects": [],
        "frame_id": "map",
        "stamp_s": 31.0,
    }


def test_scene_graph_preserves_runtime_scene_graph_ts_field() -> None:
    gateway = FakeGateway()
    handle_scene_graph(
        gateway,
        SimpleNamespace(frame_id="map", ts=42.5, objects=[]),
    )

    assert gateway.events[-1]["stamp_s"] == 42.5
