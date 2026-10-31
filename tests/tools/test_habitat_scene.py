"""Evidence export and relation-grounding tests for Habitat object scenes."""

from __future__ import annotations

import json
from types import SimpleNamespace

import numpy as np
from tools.datasets.habitat_scene import build_scene, query_scene

from perception.tracking.scene_graph_builder import SceneGraphBuilder
from perception.tracking.tracked_objects import TrackedObject


def _object(object_id: int, label: str, position: list[float], feature: list[float]) -> TrackedObject:
    return TrackedObject(
        object_id=object_id,
        label=label,
        position=np.asarray(position, dtype=float),
        best_score=0.9,
        detection_count=3,
        features=np.asarray(feature, dtype=float),
        extent=np.asarray([0.2, 0.2, 0.2]),
    )


def _tracker(
    *,
    sources: dict[int, str] | None = None,
    objects: dict[int, TrackedObject] | None = None,
) -> SimpleNamespace:
    sources = sources or {}
    objects = objects or {
        0: _object(0, "table", [0.0, 0.0, 0.7], [1.0, 0.0]),
        1: _object(1, "bottle", [0.8, 0.0, 0.8], [0.8, 0.6]),
        2: _object(2, "bottle", [5.0, 0.0, 0.8], [0.6, 0.8]),
    }
    base_scene = {
        "objects": [
            {
                "id": obj.object_id,
                "label": obj.label,
                "position": {"x": obj.position[0], "y": obj.position[1], "z": obj.position[2]},
                "score": obj.best_score,
                "detection_count": obj.detection_count,
                "source": sources.get(obj.object_id, "observed"),
            }
            for obj in objects.values()
        ]
    }
    return SimpleNamespace(
        _objects=objects,
        _scene_graph_builder=SceneGraphBuilder(objects),
        get_scene_graph_json=lambda: json.dumps(base_scene),
    )


def _view(step: int, rgb: str) -> dict:
    return {
        "step": step,
        "rgb": rgb,
        "bbox_xyxy": [10, 20, 30, 40],
        "camera_position": [1, 2, 3],
        "camera_rotation_xyzw": [0, 0, 0, 1],
        "camera_pose_frame": "habitat_y_up",
        "confidence": 0.9,
    }


def test_build_scene_exports_features_views_and_only_geometry_near_edges() -> None:
    scene = build_scene(
        _tracker(),
        {
            0: [_view(3, "rgb/003.png")],
            1: [_view(3, "rgb/003.png"), _view(5, "rgb/005.png")],
            2: [_view(7, "rgb/007.png")],
        },
    )

    assert scene["frame_id"] == "habitat_z_up"
    assert [obj["label"] for obj in scene["objects"]] == ["table", "bottle", "bottle"]
    near_bottle = scene["objects"][1]
    assert near_bottle["clip_feature"] == [0.8, 0.6]
    assert near_bottle["frame_id"] == "habitat_z_up"
    assert near_bottle["clip_feature_source"] == "fused_object_image_observations"
    assert near_bottle["description"] == {
        "text": "Detector label bottle; observed in 2 RGB view(s) at steps [3, 5]",
        "source": "detector_label_and_view_evidence",
        "vlm_caption": False,
    }
    assert near_bottle["view_evidence"][1] == {
        "step": 5,
        "rgb": "rgb/005.png",
        "bbox_xyxy": [10.0, 20.0, 30.0, 40.0],
        "camera_position": [1.0, 2.0, 3.0],
        "camera_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "camera_pose_frame": "habitat_y_up",
        "confidence": 0.9,
        "source": "rgb_detection_observation",
    }
    assert scene["relations"] == [
        {
            "subject_id": 0,
            "predicate": "near",
            "object_id": 1,
            "center_distance_m": 0.806,
            "bbox_gap_m": 0.4,
            "threshold_m": 1.5,
            "threshold_metric": "3d_bbox_gap",
            "source": "scene_graph_builder_geometry",
            "observed_steps": {"subject": [3], "object": [3, 5]},
            "co_observed_steps": [3],
        }
    ]


def test_build_scene_keeps_all_objects_and_exports_union_of_each_objects_two_nearest_edges() -> None:
    objects = {
        index: _object(index, f"object-{index}", [index * 0.6, 0.0, 0.5], [1.0, float(index)])
        for index in range(4)
    }

    scene = build_scene(_tracker(objects=objects), {})

    assert [obj["id"] for obj in scene["objects"]] == [0, 1, 2, 3]
    assert [(edge["subject_id"], edge["object_id"]) for edge in scene["relations"]] == [
        (0, 1),
        (0, 2),
        (1, 2),
        (1, 3),
        (2, 3),
    ]
    assert (0, 3) not in {
        (edge["subject_id"], edge["object_id"]) for edge in scene["relations"]
    }
    assert scene["relation_policy"] == {
        "kind": "sparse_nearest_near_neighbors",
        "max_selections_per_object": 2,
        "ranking": ["bbox_gap_m", "exact_center_distance_m"],
        "merge": "union_of_endpoint_selections",
        "order": "scene_graph_builder_original_order",
        "scope": "inspection_graph_not_exhaustive_all_pairs",
    }


def test_build_scene_includes_loaded_object_only_with_new_view_evidence() -> None:
    tracker = _tracker(sources={1: "loaded", 2: "loaded"})

    scene = build_scene(tracker, {1: [_view(8, "reobserved.png")]})

    assert [obj["id"] for obj in scene["objects"]] == [0, 1]
    assert scene["objects"][1]["source"] == "loaded"
    assert scene["objects"][1]["view_evidence"][0]["step"] == 8
    assert {edge["subject_id"] for edge in scene["relations"]} | {
        edge["object_id"] for edge in scene["relations"]
    } == {0, 1}


def test_build_scene_excludes_loaded_object_without_view_evidence() -> None:
    tracker = _tracker(sources={1: "loaded", 2: "loaded"})

    scene = build_scene(tracker, {})

    assert [obj["id"] for obj in scene["objects"]] == [0]
    assert scene["relations"] == []


def test_query_scene_selects_only_bottle_bound_to_table_near_edge() -> None:
    scene = build_scene(_tracker(), {0: [_view(3, "table.png")], 1: [_view(3, "near.png")]})

    result = query_scene(scene, "find bottle near table")
    chinese_result = query_scene(scene, "找到桌子旁边的水瓶")

    assert result["status"] == "grounded"
    assert result["relation_candidate_ids"] == [1]
    assert result["result"]["candidate_id"] == 1
    assert result["result"]["frame_id"] == "habitat_z_up"
    assert result["grounding_scope"] == "detector_candidate_graph"
    assert result["semantic_verification"] == "not_performed"
    assert result["motion_authorization"] is False
    assert chinese_result["status"] == "grounded"
    assert chinese_result["relation_candidate_ids"] == [1]
    assert chinese_result["result"]["candidate_id"] == 1
    json.dumps(chinese_result, ensure_ascii=False)


def test_query_scene_rejects_relation_request_without_actual_near_edge() -> None:
    scene = build_scene(_tracker(), {})
    scene["relations"] = []

    result = query_scene(scene, "find bottle near table")

    assert result == {
        "grounding_scope": "detector_candidate_graph",
        "semantic_verification": "not_performed",
        "motion_authorization": False,
        "status": "ungrounded",
        "reason": "near_relation_not_observed",
        "instruction": "find bottle near table",
        "target_labels": ["bottle"],
        "anchor_labels": ["table"],
        "relation_candidate_ids": [],
        "result": None,
    }


def test_query_scene_rejects_relation_request_when_anchor_is_absent() -> None:
    scene = build_scene(_tracker(), {})
    scene["objects"] = [obj for obj in scene["objects"] if obj["label"] == "bottle"]
    scene["relations"] = []

    result = query_scene(scene, "find bottle near table")

    assert result["status"] == "ungrounded"
    assert result["reason"] == "near_relation_not_observed"
    assert result["result"] is None


def test_query_scene_rejects_relation_request_when_target_is_absent() -> None:
    scene = build_scene(_tracker(), {})
    scene["objects"] = [obj for obj in scene["objects"] if obj["label"] == "table"]
    scene["relations"] = []

    result = query_scene(scene, "find bottle near table")

    assert result["status"] == "ungrounded"
    assert result["reason"] == "near_relation_not_observed"
    assert result["result"] is None
