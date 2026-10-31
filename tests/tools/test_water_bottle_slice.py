from __future__ import annotations

import importlib.util
import json
from pathlib import Path


def _module():
    path = Path(__file__).resolve().parents[2] / "tools" / "bench" / "water_bottle_slice.py"
    spec = importlib.util.spec_from_file_location("water_bottle_slice", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_slice_reports_missing_water_instance(tmp_path: Path) -> None:
    graph = tmp_path / "scene.json"
    graph.write_text(
        json.dumps(
            {
                "geometry_binding": {"status": "bound"},
                "nodes": [{"id": "cup-1", "label": "cup", "position": {"x": 1, "y": 0, "z": 1}}],
                "relations": [],
            }
        ),
        encoding="utf-8",
    )

    result = _module().run_slice(graph, "帮我找一瓶水")

    assert result["status"] == "target_unavailable"
    assert result["query"]["motion_authorization"] is False
    assert result["preview"] is None


def test_slice_stops_at_geometry_bound_candidate_without_native_map(tmp_path: Path) -> None:
    graph = tmp_path / "scene.json"
    graph.write_text(
        json.dumps(
            {
                "geometry_binding": {"status": "bound"},
                "nodes": [
                    {
                        "id": "water-1",
                        "label": "bottle",
                        "semantic_evidence": ["water bottle"],
                        "position": {"x": 1, "y": 0, "z": 1},
                        "geometry_binding": {"status": "bound", "center": [1, 0, 1]},
                    }
                ],
                "relations": [],
            }
        ),
        encoding="utf-8",
    )

    result = _module().run_slice(graph, "帮我找一瓶水")

    assert result["status"] == "candidate_generated"
    assert result["query"]["result"]["candidate_id"] == "water-1"
    assert result["query"]["approach_goal_candidates"]["candidate_count"] == 8
    assert result["motion_authorization"] is False
