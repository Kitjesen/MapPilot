from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

from sim.scripts.mujoco.map_scene_roi_monitor import (
    count_points_in_roi,
    scene_roi_record,
)
from sim.scripts.mujoco.native_dds_sensors import LinearMocapMotion
from sim.scripts.mujoco.teleop_avoid_native_acceptance import (
    DYNAMIC_PERSON_CONTRACT,
    build_execution_plan,
    build_scene_variant,
    evaluate_dynamic_obstacle_residual,
)


def _point_payload(points: list[tuple[float, float, float, float]]) -> bytes:
    return b"".join(struct.pack("<ffff", *point) for point in points)


def _scene_sample(wall_s: float, generation: int, count: int) -> dict:
    return {
        "type": "scene",
        "wall_s": wall_s,
        "producer_boot_id": "mapd-boot",
        "reset_epoch": 7,
        "generation": generation,
        "roi_counts": {
            "live": count,
            "voxel": count,
            "accumulated": count,
        },
    }


def test_map_scene_roi_monitor_counts_only_finite_points_inside_bounds() -> None:
    payload = _point_payload(
        [
            (1.0, 2.0, 0.5, 0.0),
            (1.2, 2.2, 0.7, 1.0),
            (3.0, 2.0, 0.5, 2.0),
        ]
    )

    assert count_points_in_roi(
        payload,
        center=(1.0, 2.0, 0.5),
        half_extent=(0.25, 0.25, 0.25),
    ) == 2


def test_map_scene_roi_monitor_records_all_three_product_cloud_layers() -> None:
    inside = _point_payload([(1.0, 0.0, 0.5, 0.0)])
    outside = _point_payload([(4.0, 0.0, 0.5, 0.0)])
    record = scene_roi_record(
        {
            "timestamp_s": 5.0,
            "frame_id": "map",
            "producer_boot_id": "boot",
            "reset_epoch": 2,
            "observation_sequence": 9,
            "generation": 11,
            "live": True,
            "clouds": {
                "live": {"point_count": 1, "points_xyzi_f32": inside},
                "voxel": {"point_count": 1, "points_xyzi_f32": outside},
                "accumulated": {"point_count": 2, "points_xyzi_f32": inside + outside},
            },
        },
        center=(1.0, 0.0, 0.5),
        half_extent=(0.2, 0.2, 0.2),
        wall_s=10.0,
    )

    assert record["roi_counts"] == {"live": 1, "voxel": 0, "accumulated": 1}
    assert record["generation"] == 11


def test_linear_mocap_motion_is_deterministic_and_reports_completion() -> None:
    class Data:
        mocap_pos = [[0.0, 0.0, 0.0]]
        mocap_quat = [[0.0, 0.0, 0.0, 0.0]]

    motion = LinearMocapMotion(
        mocap_id=0,
        body_name="person",
        start_xyz=(1.0, -1.0, 0.0),
        end_xyz=(1.0, 1.0, 0.0),
        start_s=1.0,
        duration_s=2.0,
    )
    data = Data()

    motion.update(data, 0.5, wall_s=10.0)
    assert data.mocap_pos[0] == (1.0, -1.0, 0.0)
    motion.update(data, 2.0, wall_s=11.0)
    assert data.mocap_pos[0] == (1.0, 0.0, 0.0)
    motion.update(data, 3.0, wall_s=12.0)

    assert data.mocap_pos[0] == (1.0, 1.0, 0.0)
    assert motion.stats()["motion_start_wall_s"] == 11.0
    assert motion.stats()["motion_complete_wall_s"] == 12.0


def test_dynamic_scene_contains_one_lidar_visible_mocap_person(tmp_path: Path) -> None:
    base = tmp_path / "base.xml"
    base.write_text(
        '<mujoco><worldbody><geom name="floor" type="plane"/></worldbody></mujoco>',
        encoding="utf-8",
    )

    scene = build_scene_variant(base, tmp_path / "moving.xml", "moving_person_clear")
    root = ET.parse(scene).getroot()
    body = root.find("./worldbody/body[@name='acceptance_moving_person']")

    assert body is not None
    assert body.attrib["mocap"] == "true"
    assert body.attrib["pos"] == "1.6 -1.2 0.0"
    assert len(body.findall("geom")) == 4
    assert all(geom.attrib["contype"] == "0" for geom in body.findall("geom"))


def test_dynamic_residual_gate_accepts_detected_and_cleared_map_layers() -> None:
    samples = [
        _scene_sample(8.0, 1, 2),
        _scene_sample(9.0, 2, 2),
        _scene_sample(10.5, 3, 9),
        _scene_sample(12.0, 4, 12),
        _scene_sample(15.0, 5, 3),
        _scene_sample(16.0, 6, 2),
    ]

    result = evaluate_dynamic_obstacle_residual(
        samples,
        {
            "enabled": True,
            "motion_started": True,
            "motion_completed": True,
            "motion_start_wall_s": 10.0,
            "motion_complete_wall_s": 13.0,
        },
    )

    assert result["ok"] is True
    assert result["blockers"] == []
    assert result["layers"]["accumulated"]["peak_excess"] == 10.0


def test_dynamic_residual_gate_rejects_persistent_accumulated_person() -> None:
    samples = [
        _scene_sample(8.0, 1, 1),
        _scene_sample(9.0, 2, 1),
        _scene_sample(10.5, 3, 10),
        _scene_sample(12.0, 4, 12),
        _scene_sample(15.0, 5, 10),
        _scene_sample(16.0, 6, 10),
    ]
    for sample in samples[-2:]:
        sample["roi_counts"]["live"] = 1
        sample["roi_counts"]["voxel"] = 1

    result = evaluate_dynamic_obstacle_residual(
        samples,
        {
            "enabled": True,
            "motion_started": True,
            "motion_completed": True,
            "motion_start_wall_s": 10.0,
            "motion_complete_wall_s": 13.0,
        },
    )

    assert result["ok"] is False
    assert "dynamic_obstacle_accumulated_residual_excessive" in result["blockers"]


def test_dynamic_execution_plan_uses_mapd_scene_observer_and_mocap_motion(
    tmp_path: Path,
) -> None:
    binaries = {
        name: tmp_path / name
        for name in (
            "slam",
            "mapd",
            "traversability",
            "navigation",
            "navigation_control",
            "sensor_publisher",
            "cmd_vel_tap",
        )
    }
    paths = {
        "slam_config": tmp_path / "slam.yaml",
        "policy": tmp_path / "policy.onnx",
        "path_library": tmp_path / "paths",
        "sensor_runner": tmp_path / "native_dds_sensors.py",
        "world": tmp_path / "scene.xml",
    }

    plan = build_execution_plan(
        scenario="moving_person_clear",
        domain_id=240,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "case",
        duration_s=10.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={},
    )

    processes = {item["name"]: item for item in plan["processes"]}
    assert "map_scene_monitor" in processes
    sensor = processes["sensor"]["command"]
    assert sensor[sensor.index("--mocap-motion-body") + 1] == str(
        DYNAMIC_PERSON_CONTRACT["body_name"]
    )
    assert sensor[sensor.index("--mocap-motion-start-s") + 1] == "1.0"
    assert plan["functional_scope"]["dynamic_obstacle_residual_gate"] is True
    assert plan["functional_scope"]["mapd_data_contract"][
        "navigation_traversability_role"
    ] == "standalone_safety_authority"
