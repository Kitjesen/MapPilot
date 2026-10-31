#!/usr/bin/env python3
"""Run the LingTu fast goal resolver against a converted semantic graph."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig
from decision.semantic_navigation.observation import ObjectTarget, observation_candidates
from decision.semantic_navigation.query import ObjectQuery, parse_object_query
from runtime.msgs.geometry import Pose, PoseStamped, Vector3


def _node_position(node: dict) -> dict[str, float] | None:
    """Return a bound node center, or ``None`` when geometry is unavailable."""
    position = node.get("position")
    if position is None:
        position = node.get("geometry_binding", {}).get("center")
    if isinstance(position, (list, tuple)) and len(position) >= 3:
        return {"x": float(position[0]), "y": float(position[1]), "z": float(position[2])}
    if isinstance(position, dict) and all(key in position for key in ("x", "y", "z")):
        return {key: float(position[key]) for key in ("x", "y", "z")}
    return None


def _approach_goal_candidates(
    nodes: list[dict],
    target_id: str | None,
    *,
    geometry_status: str,
    robot_position: dict[str, float] | None = None,
    standoff_m: float = 0.8,
) -> dict:
    """Build observation-goal hypotheses around a semantic object center.

    The returned poses are proposals for native read-only preview. They are
    deliberately offset from the object center and never authorize motion.
    """
    base = {
        "object_id": target_id,
        "frame_id": "map",
        "standoff_m": float(standoff_m),
        "motion_authorization": False,
        "native_planner_required": True,
        "candidates": [],
    }
    if not target_id:
        return {**base, "status": "no_target", "reason": "resolver_returned_no_candidate"}
    if geometry_status != "bound":
        return {**base, "status": "geometry_unavailable", "reason": "object_center_is_not_bound"}

    node = next((item for item in nodes if str(item.get("id")) == str(target_id)), None)
    center = _node_position(node) if node else None
    if center is None:
        return {**base, "status": "geometry_unavailable", "reason": "target_center_missing"}
    if not math.isfinite(standoff_m) or standoff_m <= 0.0:
        raise ValueError("standoff_m must be finite and positive")

    robot = robot_position or {"x": 0.0, "y": 0.0, "z": 0.3}
    robot_pose = PoseStamped(Pose(Vector3(robot["x"], robot["y"], robot["z"])), frame_id="map")
    target = ObjectTarget(
        object_id=str(target_id),
        label=str(node.get("label", "object")),
        position=(center["x"], center["y"], center["z"]),
    )
    poses = observation_candidates(target, robot_pose, standoff_m)
    return {
        **base,
        "status": "candidate_generated",
        "object_center": [center["x"], center["y"], center["z"]],
        "candidate_count": len(poses),
        "candidates": [
            {
                "x": pose.x,
                "y": pose.y,
                "z": pose.z,
                "yaw": pose.yaw,
                "frame_id": pose.frame_id,
                "source": "object_center_standoff",
                "requires_native_preview": True,
                "motion_authorization": False,
            }
            for pose in poses
        ],
    }


def _relation_grounded_candidates(
    nodes: list[dict],
    relations: list[dict],
    *,
    target_label: str,
    reference_label: str,
    predicate: str,
) -> list[str]:
    """Return target objects that have the requested relation to a reference.

    The fast resolver scores a relation hint, but this small adapter keeps the
    dataset smoke test honest by also computing the exact graph constraint.
    Geometry is still unavailable, so these IDs are semantic candidates only.
    """
    labels = {str(node.get("id")): str(node.get("label", "")).lower() for node in nodes}
    candidates: list[str] = []
    for relation in relations:
        if str(relation.get("predicate", "")).lower() != predicate.lower():
            continue
        subject = str(relation.get("subject_id", ""))
        target = str(relation.get("object_id", ""))
        if labels.get(subject) == target_label and labels.get(target) == reference_label:
            candidates.append(subject)
        elif labels.get(target) == target_label and labels.get(subject) == reference_label:
            candidates.append(target)
    return list(dict.fromkeys(candidates))


def _semantic_query_matches(nodes: list[dict], query: ObjectQuery) -> list[dict]:
    """Return geometry records whose stored label matches a symbolic query.

    This is deliberately label/evidence based. A query alias never turns a
    nearby unrelated object into a target; the instance must carry the label
    (or an explicit semantic evidence alias) and a geometry binding is checked
    later before approach candidates are emitted.
    """
    aliases = {str(alias).strip().lower() for alias in query.aliases}
    matches: list[dict] = []
    for node in nodes:
        labels = {str(node.get("label", "")).strip().lower()}
        evidence = node.get("semantic_evidence", node.get("semantic_evidence_refs", []))
        if isinstance(evidence, list):
            labels.update(
                str(item.get("label", item) if isinstance(item, dict) else item).strip().lower()
                for item in evidence
            )
        if labels & aliases:
            matches.append(node)
    return matches


def _offline_object_result(query: ObjectQuery, matches: list[dict], robot_position: dict[str, float] | None) -> dict | None:
    """Choose the closest matching persisted instance without inventing pose."""
    if not matches:
        return None
    robot = robot_position or {"x": 0.0, "y": 0.0, "z": 0.3}

    def distance(node: dict) -> float:
        position = _node_position(node) or {"x": 0.0, "y": 0.0, "z": 0.0}
        return math.sqrt((position["x"] - robot["x"]) ** 2 + (position["y"] - robot["y"]) ** 2)

    target = min(matches, key=distance)
    return {
        "action": query.action,
        "target_label": query.label,
        "candidate_id": target.get("id"),
        "confidence": 1.0,
        "path": "geometry_query",
        "reasoning": "stored semantic label/evidence matched a persisted instance",
    }


def resolve(
    graph_path: Path,
    instruction: str,
    *,
    robot_position: dict[str, float] | None = None,
    standoff_m: float = 0.8,
) -> dict:
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    semantic_query = parse_object_query(instruction)
    nodes = graph.get("nodes", [])
    objects = []
    for node in nodes:
        position = _node_position(node) or {"x": 0.0, "y": 0.0, "z": 0.0}
        objects.append(
            {
                "id": node["id"],
                "label": node["label"],
                "score": 1.0,
                "detection_count": 3,
                "position": position,
            }
        )
    relations = [
        {
            "subject_id": relation["subject"],
            "object_id": relation["object"],
            "predicate": "near" if relation["predicate"] == "close by" else relation["predicate"],
        }
        for relation in graph.get("relations", [])
    ]
    scene = {"objects": objects, "relations": relations, "rooms": []}
    matches = _semantic_query_matches(nodes, semantic_query) if semantic_query else []
    relational_request = any(token in instruction.lower() for token in ("旁边", "附近", "near", "next to", "close by"))
    direct_object_query = semantic_query is not None and not relational_request
    result = (
        _offline_object_result(semantic_query, matches, robot_position)
        if direct_object_query
        else GoalResolver(LLMConfig(backend="mock"), fast_path_threshold=0.5).fast_resolve(
            instruction,
            json.dumps(scene, ensure_ascii=False),
        )
    )
    relation_candidates = _relation_grounded_candidates(
        objects,
        relations,
        target_label="box",
        reference_label="table",
        predicate="near",
    )
    resolver_candidate_id = (
        result.get("candidate_id") if isinstance(result, dict) else getattr(result, "candidate_id", None)
    )
    geometry_status = graph.get("geometry_binding", {}).get("status", "unknown")
    approach = _approach_goal_candidates(
        nodes,
        resolver_candidate_id,
        geometry_status=geometry_status,
        robot_position=robot_position,
        standoff_m=standoff_m,
    )
    return {
        "query": instruction,
        "query_parse": semantic_query.__dict__ if direct_object_query and semantic_query else None,
        "source_scan": graph.get("source", {}).get("scan"),
        "input_object_count": len(objects),
        "input_relation_count": len(relations),
        "result": result if isinstance(result, dict) else (result.__dict__ if result else None),
        "semantic_match": {
            "candidate_ids": [str(node.get("id")) for node in matches],
            "matched_count": len(matches),
            "status": "candidate" if matches else ("target_unavailable" if direct_object_query else "not_applicable"),
        },
        "relation_grounding": (
            {"status": "not_applicable"}
            if direct_object_query
            else {
                "target_label": "box",
                "reference_label": "table",
                "predicate": "near",
                "candidate_ids": relation_candidates,
                "resolver_candidate_matches_relation": resolver_candidate_id in relation_candidates,
            }
        ),
        "geometry_status": geometry_status,
        "approach_goal_candidates": approach,
        "motion_authorization": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("graph", type=Path)
    parser.add_argument("--instruction", default="找到桌子旁边的箱子")
    parser.add_argument("--robot-x", type=float, default=0.0)
    parser.add_argument("--robot-y", type=float, default=0.0)
    parser.add_argument("--robot-z", type=float, default=0.3)
    parser.add_argument("--standoff-m", type=float, default=0.8)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = resolve(
        args.graph,
        args.instruction,
        robot_position={"x": args.robot_x, "y": args.robot_y, "z": args.robot_z},
        standoff_m=args.standoff_m,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
