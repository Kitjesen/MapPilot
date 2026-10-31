from __future__ import annotations

import json
from pathlib import Path

from tools.datasets.replicacad_persistence_check import check


def test_instance_records_survive_json_save_reload(tmp_path: Path) -> None:
    graph = tmp_path / "scene.json"
    graph.write_text(
        json.dumps(
            {
                "map_id": "replicacad:apt_0",
                "instance_records": [
                    {
                        "instance_id": "object:apt_0:1",
                        "map_identity": "replicacad:apt_0",
                        "geometry_ref": {"asset": "box.glb", "bbox_min": [0, 0, 0], "bbox_max": [1, 1, 1]},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    report = check(graph, tmp_path / "report.json")

    assert report["record_count"] == 1
    assert report["reloaded_count"] == 1
    assert report["stable_instance_ids"] is True
    assert report["geometry_refs_reloaded"] is True
    assert report["motion_authorization"] is False
