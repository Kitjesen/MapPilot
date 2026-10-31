from __future__ import annotations

from pathlib import Path

from tools.datasets.semantic_dataset_inventory import build_inventory


def test_current_repository_inventory_exposes_the_semantic_dataset_gap() -> None:
    report = build_inventory(Path(__file__).resolve().parents[2])

    assert report["pcd_geometry"]["status"] == "ready"
    assert len(report["pcd_geometry"]["files"]) == 2
    assert report["pcd_geometry"]["files"][0]["fields"] == ["x", "y", "z", "_"]
    assert report["lidar_imu_replay"]["camera_frames"] == 0
    assert report["rgbd_frames_in_evaluation_data"]["status"] == "missing"
    assert report["simulation_semantic_ground_truth"]["schema"] == "lingtu.sim.semantic-entities.v1"
