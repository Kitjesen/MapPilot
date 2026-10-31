"""Export evidence-backed Habitat objects and resolve small scene queries."""

from __future__ import annotations

import json
import re
from dataclasses import asdict
from typing import Any

import numpy as np

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig
from perception.tracking.scene_graph_builder import SceneGraphBuilder
from runtime.msgs.scene import RELATION_NEAR_THRESHOLD


def _view_evidence(view: dict[str, Any]) -> dict[str, Any]:
    """Keep the camera evidence needed to inspect one object observation."""
    return {
        "step": int(view["step"]),
        "rgb": str(view["rgb"]),
        "bbox_xyxy": [float(value) for value in view["bbox_xyxy"]],
        "camera_position": [float(value) for value in view["camera_position"]],
        "camera_rotation_xyzw": [float(value) for value in view["camera_rotation_xyzw"]],
        "camera_pose_frame": str(view["camera_pose_frame"]),
        "confidence": float(view["confidence"]),
        "source": "rgb_detection_observation",
    }


def build_scene(tracker: Any, object_views: dict[int, list[dict]]) -> dict:
    """Build a compact object scene from tracker state and RGB evidence.

    Only observed objects and ``near`` relations computed by the existing
    :class:`SceneGraphBuilder` are exported.  Descriptions state the detector
    label provenance; they do not claim an unrun VLM caption.
    """
    tracker_scene = json.loads(tracker.get_scene_graph_json())
    tracked_objects = tracker._objects
    base_objects = []
    for item in tracker_scene.get("objects", []):
        object_id = int(item["id"])
        source = item.get("source", "observed")
        if source == "observed" or (source == "loaded" and object_views.get(object_id)):
            base_objects.append(item)
    observed_ids = {int(item["id"]) for item in base_objects}

    objects: list[dict[str, Any]] = []
    for item in base_objects:
        object_id = int(item["id"])
        tracked = tracked_objects[object_id]
        views = [_view_evidence(view) for view in object_views.get(object_id, [])]
        objects.append(
            {
                **item,
                "frame_id": "habitat_z_up",
                "clip_feature": np.asarray(tracked.features, dtype=float).tolist(),
                "clip_feature_source": "fused_object_image_observations",
                "view_evidence": views,
                "description": {
                    "text": (
                        f"Detector label {item['label']}; observed in {len(views)} RGB view(s) "
                        f"at steps {[view['step'] for view in views]}"
                    ),
                    "source": "detector_label_and_view_evidence",
                    "vlm_caption": False,
                },
            }
        )

    builder = SceneGraphBuilder(tracked_objects)
    relation_candidates: list[tuple[dict[str, Any], float, float]] = []
    for relation in builder.compute_spatial_relations():
        if relation.relation != "near":
            continue
        if relation.subject_id not in observed_ids or relation.object_id not in observed_ids:
            continue
        subject_steps = sorted({int(view["step"]) for view in object_views.get(relation.subject_id, [])})
        object_steps = sorted({int(view["step"]) for view in object_views.get(relation.object_id, [])})
        subject = tracked_objects[relation.subject_id]
        object_ = tracked_objects[relation.object_id]
        center_delta = np.abs(np.asarray(subject.position) - np.asarray(object_.position))
        bbox_gap = np.maximum(0.0, center_delta - np.asarray(subject.extent) - np.asarray(object_.extent))
        bbox_gap_m = float(np.linalg.norm(bbox_gap))
        center_distance_m = float(np.linalg.norm(center_delta))
        relation_candidates.append(
            (
                {
                    "subject_id": relation.subject_id,
                    "predicate": "near",
                    "object_id": relation.object_id,
                    "center_distance_m": round(center_distance_m, 3),
                    "bbox_gap_m": round(bbox_gap_m, 3),
                    "threshold_m": float(RELATION_NEAR_THRESHOLD),
                    "threshold_metric": "3d_bbox_gap",
                    "source": "scene_graph_builder_geometry",
                    "observed_steps": {
                        "subject": subject_steps,
                        "object": object_steps,
                    },
                    "co_observed_steps": sorted(set(subject_steps) & set(object_steps)),
                },
                bbox_gap_m,
                center_distance_m,
            )
        )

    selected_indices: set[int] = set()
    for object_id in sorted(observed_ids):
        incident = [
            (index, bbox_gap_m, center_distance_m)
            for index, (relation, bbox_gap_m, center_distance_m) in enumerate(relation_candidates)
            if object_id in (relation["subject_id"], relation["object_id"])
        ]
        incident.sort(key=lambda item: (item[1], item[2], item[0]))
        selected_indices.update(index for index, _, _ in incident[:2])
    relations = [
        relation
        for index, (relation, _, _) in enumerate(relation_candidates)
        if index in selected_indices
    ]

    return {
        "frame_id": "habitat_z_up",
        "object_count": len(objects),
        "objects": objects,
        "relations": relations,
        "relation_policy": {
            "kind": "sparse_nearest_near_neighbors",
            "max_selections_per_object": 2,
            "ranking": ["bbox_gap_m", "exact_center_distance_m"],
            "merge": "union_of_endpoint_selections",
            "order": "scene_graph_builder_original_order",
            "scope": "inspection_graph_not_exhaustive_all_pairs",
        },
        "description_policy": "detector labels and stored observations only; no VLM caption",
    }


def _relation_candidate_ids(
    scene: dict,
    subject_labels: list[str],
    modifier_labels: list[str],
) -> list[int]:
    labels = {int(obj["id"]): str(obj.get("label", "")).lower() for obj in scene.get("objects", [])}
    candidates: list[int] = []
    for relation in scene.get("relations", []):
        if relation.get("predicate") != "near":
            continue
        subject_id = int(relation["subject_id"])
        object_id = int(relation["object_id"])
        if labels.get(subject_id) in subject_labels and labels.get(object_id) in modifier_labels:
            candidates.append(subject_id)
        elif labels.get(object_id) in subject_labels and labels.get(subject_id) in modifier_labels:
            candidates.append(object_id)
    return list(dict.fromkeys(candidates))


def query_scene(scene: dict, instruction: str) -> dict:
    """Resolve an instruction without an LLM and require relation evidence."""
    grounding_limits = {
        "grounding_scope": "detector_candidate_graph",
        "semantic_verification": "not_performed",
        "motion_authorization": False,
    }
    resolver = GoalResolver(LLMConfig(backend="mock"), fast_path_threshold=0.5)
    labels = [str(obj.get("label", "")).lower() for obj in scene.get("objects", [])]
    keywords = resolver._extract_keywords(instruction)
    subject_labels, modifier_labels = resolver._parse_instruction_roles(
        instruction.lower(),
        keywords,
        labels,
    )
    relation_candidates = _relation_candidate_ids(scene, subject_labels, modifier_labels)
    relation_requested = bool(
        re.search(r"\b(?:near|beside|next\s+to|close\s+by)\b|旁边|附近", instruction.lower())
    )

    result = resolver.fast_resolve(instruction, json.dumps(scene, ensure_ascii=False))
    serialized = asdict(result) if result is not None else None
    if serialized is not None:
        serialized["frame_id"] = str(scene.get("frame_id", "habitat_z_up"))

    if relation_requested and (not subject_labels or not modifier_labels or not relation_candidates):
        return {
            **grounding_limits,
            "status": "ungrounded",
            "reason": "near_relation_not_observed",
            "instruction": instruction,
            "target_labels": subject_labels,
            "anchor_labels": modifier_labels,
            "relation_candidate_ids": [],
            "result": None,
        }

    if relation_requested and (
        serialized is None
        or not serialized.get("is_valid", False)
        or int(serialized.get("candidate_id", -1)) not in relation_candidates
    ):
        return {
            **grounding_limits,
            "status": "ungrounded",
            "reason": "resolver_candidate_not_bound_to_near_edge",
            "instruction": instruction,
            "target_labels": subject_labels,
            "anchor_labels": modifier_labels,
            "relation_candidate_ids": relation_candidates,
            "result": None,
        }

    return {
        **grounding_limits,
        "status": "grounded" if serialized is not None and serialized.get("is_valid", False) else "unresolved",
        "instruction": instruction,
        "target_labels": subject_labels,
        "anchor_labels": modifier_labels,
        "relation_candidate_ids": relation_candidates,
        "result": serialized,
    }


__all__ = ["build_scene", "query_scene"]
