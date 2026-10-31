from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "3dssg_probe.py"
SPEC = importlib.util.spec_from_file_location("three_dssg_probe", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_convert_scene_preserves_object_and_relation_ids(tmp_path: Path) -> None:
    dataset = tmp_path / "3dssg"
    dataset.mkdir()
    (dataset / "objects.json").write_text(
        json.dumps(
            {
                "scans": [
                    {
                        "scan": "scan-a",
                        "objects": [
                            {"id": "1", "label": "table", "attributes": {}},
                            {"id": "2", "label": "box", "attributes": {}},
                        ],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    (dataset / "relationships.json").write_text(
        json.dumps({"scans": [{"scan": "scan-a", "relationships": [[2, 1, 15, "standing on"]]}]}),
        encoding="utf-8",
    )

    result = MODULE.convert_scene(dataset)

    assert result["source"]["scan"] == "scan-a"
    assert result["stats"] == {"node_count": 2, "relation_count": 1, "skipped_relations": 0}
    assert result["relations"] == [
        {
            "subject": "object:scan-a:2",
            "predicate": "standing on",
            "predicate_id": 15,
            "object": "object:scan-a:1",
        }
    ]
    assert result["geometry_binding"]["status"] == "unavailable"


def test_relation_grounding_keeps_target_and_reference_roles() -> None:
    query_script = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "3dssg_goal_query.py"
    query_spec = importlib.util.spec_from_file_location("three_dssg_goal_query", query_script)
    assert query_spec is not None and query_spec.loader is not None
    query_module = importlib.util.module_from_spec(query_spec)
    query_spec.loader.exec_module(query_module)

    nodes = [
        {"id": "table-1", "label": "table"},
        {"id": "box-1", "label": "box"},
        {"id": "box-2", "label": "box"},
    ]
    relations = [
        {"subject_id": "table-1", "object_id": "box-1", "predicate": "near"},
        {"subject_id": "box-2", "object_id": "table-1", "predicate": "standing on"},
    ]

    assert query_module._relation_grounded_candidates(
        nodes,
        relations,
        target_label="box",
        reference_label="table",
        predicate="near",
    ) == ["box-1"]


def test_approach_candidates_stand_off_from_bound_object_center() -> None:
    query_script = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "3dssg_goal_query.py"
    query_spec = importlib.util.spec_from_file_location("three_dssg_goal_query_bound", query_script)
    assert query_spec is not None and query_spec.loader is not None
    query_module = importlib.util.module_from_spec(query_spec)
    query_spec.loader.exec_module(query_module)

    result = query_module._approach_goal_candidates(
        [
            {
                "id": "box-1",
                "label": "box",
                "geometry_binding": {"status": "bound", "center": [2.0, 1.0, 0.4]},
            }
        ],
        "box-1",
        geometry_status="bound",
        robot_position={"x": 0.0, "y": 1.0, "z": 0.3},
        standoff_m=0.5,
    )

    assert result["status"] == "candidate_generated"
    assert result["object_center"] == [2.0, 1.0, 0.4]
    assert result["candidate_count"] == 8
    assert result["motion_authorization"] is False
    assert result["native_planner_required"] is True
    for candidate in result["candidates"]:
        assert candidate["requires_native_preview"] is True
        assert candidate["motion_authorization"] is False
        assert (candidate["x"] - 2.0) ** 2 + (candidate["y"] - 1.0) ** 2 == pytest.approx(0.25)
        assert (candidate["x"], candidate["y"]) != (2.0, 1.0)


def test_approach_candidates_do_not_invent_pose_without_geometry() -> None:
    query_script = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "3dssg_goal_query.py"
    query_spec = importlib.util.spec_from_file_location("three_dssg_goal_query_unbound", query_script)
    assert query_spec is not None and query_spec.loader is not None
    query_module = importlib.util.module_from_spec(query_spec)
    query_spec.loader.exec_module(query_module)

    result = query_module._approach_goal_candidates(
        [{"id": "box-1", "label": "box"}],
        "box-1",
        geometry_status="unavailable",
    )

    assert result["status"] == "geometry_unavailable"
    assert result["candidates"] == []
    assert result["motion_authorization"] is False


def test_water_query_requires_a_matching_persisted_instance(tmp_path: Path) -> None:
    query_script = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "3dssg_goal_query.py"
    query_spec = importlib.util.spec_from_file_location("three_dssg_goal_query_water", query_script)
    assert query_spec is not None and query_spec.loader is not None
    query_module = importlib.util.module_from_spec(query_spec)
    query_spec.loader.exec_module(query_module)

    graph = {
        "schema_version": "lingtu.semantic_scene_graph.v1",
        "geometry_binding": {"status": "bound"},
        "nodes": [
            {
                "id": "cup-1",
                "label": "cup",
                "position": {"x": 1.0, "y": 0.0, "z": 0.8},
                "geometry_binding": {"status": "bound", "center": [1.0, 0.0, 0.8]},
            }
        ],
        "relations": [],
    }
    graph_path = tmp_path / "graph.json"
    graph_path.write_text(json.dumps(graph), encoding="utf-8")

    result = query_module.resolve(graph_path, "帮我找一瓶水")

    assert result["query_parse"]["label"] == "water_bottle"
    assert result["semantic_match"]["status"] == "target_unavailable"
    assert result["approach_goal_candidates"]["candidates"] == []
    assert result["motion_authorization"] is False


def test_water_query_grounds_only_an_explicit_water_instance(tmp_path: Path) -> None:
    query_script = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "3dssg_goal_query.py"
    query_spec = importlib.util.spec_from_file_location("three_dssg_goal_query_water_bound", query_script)
    assert query_spec is not None and query_spec.loader is not None
    query_module = importlib.util.module_from_spec(query_spec)
    query_spec.loader.exec_module(query_module)

    graph = {
        "schema_version": "lingtu.semantic_scene_graph.v1",
        "geometry_binding": {"status": "bound"},
        "nodes": [
            {
                "id": "water-1",
                "label": "bottle",
                "semantic_evidence": ["water bottle"],
                "position": {"x": 1.0, "y": 0.0, "z": 0.8},
                "geometry_binding": {"status": "bound", "center": [1.0, 0.0, 0.8]},
            }
        ],
        "relations": [],
    }
    graph_path = tmp_path / "graph.json"
    graph_path.write_text(json.dumps(graph), encoding="utf-8")

    result = query_module.resolve(graph_path, "帮我找一瓶水")

    assert result["result"]["candidate_id"] == "water-1"
    assert result["semantic_match"]["status"] == "candidate"
    assert result["approach_goal_candidates"]["status"] == "candidate_generated"
    assert result["approach_goal_candidates"]["candidate_count"] == 8
    assert result["motion_authorization"] is False
