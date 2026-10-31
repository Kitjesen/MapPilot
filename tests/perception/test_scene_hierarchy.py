"""Contract tests for preserving the tracker hierarchy at the runtime boundary."""

from __future__ import annotations

import json
from types import SimpleNamespace

from perception.pipeline import _scene_graph_from_tracker


def _tracker_payload() -> dict:
    return {
        "graph_level": "hierarchical",
        "graph_version": "3.0",
        "frame_id": "map",
        "objects": [
            {"id": 10, "label": "door", "position": [0.0, 0.0, 1.0], "score": 0.9, "last_observed_time": 10.0},
            {"id": 11, "label": "desk", "position": [4.0, 0.0, 0.7], "score": 0.9, "last_observed_time": 10.0},
            {"id": 12, "label": "box", "position": [4.5, 0.2, 0.4], "score": 0.9, "last_observed_time": 10.0},
        ],
        "relations": [
            {"subject_id": 12, "relation": "near", "object_id": 11, "distance": 0.54},
        ],
        "floors": [{"floor_id": 0, "room_ids": [0, 1], "object_ids": [10, 11, 12]}],
        "rooms": [
            {"room_id": 0, "name": "corridor", "center": {"x": 0.0, "y": 0.0, "z": 0.0},
             "object_ids": [10], "group_ids": [0]},
            {"room_id": 1, "name": "office", "center": {"x": 4.0, "y": 0.1, "z": 0.0},
             "object_ids": [11, 12], "group_ids": [1]},
        ],
        "groups": [
            {"group_id": 0, "room_id": 0, "name": "structure", "object_ids": [10]},
            {"group_id": 1, "room_id": 1, "name": "furniture", "object_ids": [11, 12]},
        ],
        "views": [],
        "hierarchy_edges": [
            {"parent_type": "floor", "parent_id": 0, "child_type": "room", "child_id": 0, "relation": "contains"},
            {"parent_type": "floor", "parent_id": 0, "child_type": "room", "child_id": 1, "relation": "contains"},
            {"parent_type": "room", "parent_id": 0, "child_type": "group", "child_id": 0, "relation": "contains"},
            {"parent_type": "room", "parent_id": 1, "child_type": "group", "child_id": 1, "relation": "contains"},
            {"parent_type": "group", "parent_id": 0, "child_type": "object", "child_id": 10, "relation": "contains"},
            {"parent_type": "group", "parent_id": 1, "child_type": "object", "child_id": 11, "relation": "contains"},
            {"parent_type": "group", "parent_id": 1, "child_type": "object", "child_id": 12, "relation": "contains"},
        ],
        "topology_edges": [{"from_room": 0, "to_room": 1, "type": "door", "mediator": "door"}],
        "frontier_nodes": [],
        "subgraphs": [{"subgraph_id": "room_1", "level": "room", "room_id": 1, "object_ids": [11, 12]}],
        "room_type_posteriors": {},
        "belief_propagation": {"iterations": 1},
    }


def _adapt(payload: dict):
    tracker = SimpleNamespace(get_scene_graph_json=lambda: json.dumps(payload))
    return _scene_graph_from_tracker(
        tracker,
        latest_detections=(),
        observed_ids=set(),
        frame_id="map",
        source_ts=10.0,
    )


def test_tracker_hierarchy_is_preserved_in_public_scene_graph() -> None:
    scene = _adapt(_tracker_payload())

    assert scene.graph_level == "hierarchical"
    assert scene.graph_version == "3.0"
    assert [room.name for room in scene.regions] == ["corridor", "office"]
    assert scene.regions[1].center.x == 4.0
    assert len(scene.floors) == 1
    assert len(scene.rooms) == 2
    assert len(scene.groups) == 2
    assert len(scene.hierarchy_edges) == 7
    assert scene.topology_edges[0]["from_room"] == 0
    assert scene.validate_hierarchy() == ()


def test_hierarchy_roundtrip_keeps_ids_and_rejects_broken_edges() -> None:
    scene = _adapt(_tracker_payload())
    restored = scene.from_json(scene.to_json())
    assert restored.validate_hierarchy() == ()
    assert restored.rooms[1]["object_ids"] == [11, 12]

    broken = restored.to_dict()
    broken["hierarchy_edges"].append({
        "parent_type": "group", "parent_id": 1,
        "child_type": "object", "child_id": 999,
        "relation": "contains",
    })
    invalid = restored.from_dict(broken)
    assert "hierarchy_edge_unknown_child:object:999" in invalid.validate_hierarchy()

    cyclic = restored.to_dict()
    cyclic["hierarchy_edges"].extend([
        {"parent_type": "group", "parent_id": 1, "child_type": "room", "child_id": 1},
        {"parent_type": "room", "parent_id": 1, "child_type": "group", "child_id": 1},
    ])
    cycle_graph = restored.from_dict(cyclic)
    assert any(error.startswith("hierarchy_cycle:") for error in cycle_graph.validate_hierarchy())
