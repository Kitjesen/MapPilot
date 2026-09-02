from __future__ import annotations

import struct

from sim.scripts.mujoco.map_scene_roi_monitor import (
    count_points_in_roi,
    scene_roi_record,
)
from sim.scripts.mujoco.native_dds_sensors import LinearMocapMotion


def _point_payload(points: list[tuple[float, float, float, float]]) -> bytes:
    return b"".join(struct.pack("<ffff", *point) for point in points)


def test_map_scene_roi_monitor_counts_only_finite_points_inside_bounds() -> None:
    payload = _point_payload(
        [
            (1.0, 2.0, 0.5, 0.0),
            (1.2, 2.2, 0.7, 1.0),
            (3.0, 2.0, 0.5, 2.0),
        ]
    )

    assert (
        count_points_in_roi(
            payload,
            center=(1.0, 2.0, 0.5),
            half_extent=(0.25, 0.25, 0.25),
        )
        == 2
    )


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
