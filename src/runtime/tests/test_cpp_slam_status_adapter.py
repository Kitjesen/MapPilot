from __future__ import annotations

import json
import time

from runtime.adapters.native.localization_adapter import (
    CppSlamStatusAdapterModule,
    STATUS_SNAPSHOT_SCHEMA,
    STATUS_SNAPSHOT_HEALTH_SOURCE,
)
from runtime.blueprints.profile_builder import blueprint_for_resolved_profile
from runtime.blueprints.stacks.slam import slam
from runtime.msgs.sensor import PointCloud2
from runtime.profiles.resolver import resolve_profile_config


def test_cpp_slam_status_adapter_reads_cpp_status_snapshot(tmp_path) -> None:
    path = tmp_path / "slam_status.json"
    adapter = CppSlamStatusAdapterModule(
        status_snapshot_path=str(path),
        status_snapshot_interval_s=0.05,
        status_snapshot_stale_after_s=1.0,
    )
    odometry_seen = []
    status_seen = []
    tf_seen = []
    gnss_seen = []
    scene_seen = []
    adapter.odometry.subscribe(odometry_seen.append)
    adapter.localization_status.subscribe(status_seen.append)
    adapter.map_odom_tf.subscribe(tf_seen.append)
    adapter.gnss_fusion_health.subscribe(gnss_seen.append)
    adapter.scene_mode.subscribe(scene_seen.append)

    adapter.setup()
    try:
        path.write_text(json.dumps(_status_payload()), encoding="utf-8")
        deadline = time.time() + 1.0
        while time.time() < deadline and not odometry_seen:
            time.sleep(0.02)
    finally:
        adapter.stop()

    assert odometry_seen[-1].x == 1.0
    assert odometry_seen[-1].ts == 123.0
    assert status_seen[-1]["health_source"] == STATUS_SNAPSHOT_HEALTH_SOURCE
    assert status_seen[-1]["backend"] == "fastlio2"
    assert status_seen[-1]["mode"] == "mapping"
    assert status_seen[-1]["status_target_hz"] == 10.0
    assert status_seen[-1]["imu_input_hz"] == 196.0
    assert status_seen[-1]["lidar_input_hz"] == 10.0
    assert status_seen[-1]["slam_tick_hz"] == 50.0
    assert status_seen[-1]["processed_scan_hz"] == 9.8
    assert status_seen[-1]["imu_batch"] == 18
    assert status_seen[-1]["buffers"]["sync_wait_count"] == 2
    assert status_seen[-1]["map_loaded"] is True
    assert status_seen[-1]["scene_mode"] == "outdoor"
    assert status_seen[-1]["map_odom_tf"]["valid"] is True
    assert tf_seen[-1]["child_frame_id"] == "odom"
    assert tf_seen[-1]["tx"] == 0.1
    assert adapter._frame_tree.lookup("map", "odom", ts=123.0).translation.x == 0.1
    assert gnss_seen[-1]["last_fix_type"] == "RTK_FIXED"
    assert scene_seen[-1] == "outdoor"
    health = adapter.health()
    assert health["transport"] == "cpp_status_snapshot"
    assert health["snapshot_backend"] == "file"
    assert health["status_snapshot_contract"] == STATUS_SNAPSHOT_SCHEMA
    assert health["payload_contract"] == "status_and_cloud_snapshots"
    assert health["cloud_payloads"] == "binary_pointcloud2_snapshot"
    assert health["status_snapshot_stale"] is False


def test_cpp_slam_status_adapter_reads_cloud_snapshots(tmp_path) -> None:
    status_path = tmp_path / "slam_status.json"
    cloud_dir = tmp_path / "clouds"
    cloud_dir.mkdir()
    (cloud_dir / "map_cloud.bin").write_bytes(
        PointCloud2(
            points=[[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.7]],
            ts=124.0,
            frame_id="map",
        ).encode()
    )
    adapter = CppSlamStatusAdapterModule(
        status_snapshot_path=str(status_path),
        cloud_snapshot_dir=str(cloud_dir),
        status_snapshot_interval_s=0.05,
        status_snapshot_stale_after_s=1.0,
    )
    clouds_seen = []
    adapter.map_cloud.subscribe(clouds_seen.append)

    adapter.setup()
    try:
        deadline = time.time() + 1.0
        while time.time() < deadline and not clouds_seen:
            time.sleep(0.02)
    finally:
        adapter.stop()

    assert clouds_seen[-1].frame_id == "map"
    assert clouds_seen[-1].num_points == 2
    assert clouds_seen[-1].points[0, 0] == 1.0
    assert adapter.health()["message_counts"]["/slam/map_cloud"] >= 1


def test_cpp_slam_status_adapter_marks_stale_snapshot(tmp_path) -> None:
    path = tmp_path / "slam_status.json"
    adapter = CppSlamStatusAdapterModule(
        status_snapshot_path=str(path),
        status_snapshot_interval_s=0.05,
        status_snapshot_stale_after_s=0.1,
    )
    status_seen = []
    alive_seen = []
    adapter.localization_status.subscribe(status_seen.append)
    adapter.alive.subscribe(alive_seen.append)

    adapter.setup()
    try:
        path.write_text(json.dumps(_status_payload()), encoding="utf-8")
        deadline = time.time() + 1.0
        while time.time() < deadline and not status_seen:
            time.sleep(0.02)
        deadline = time.time() + 1.0
        while time.time() < deadline and status_seen[-1]["state"] != "STALE":
            time.sleep(0.02)
    finally:
        adapter.stop()

    assert status_seen[-1]["state"] == "STALE"
    assert status_seen[-1]["reason"] == "slam_runtime_status_stale"
    assert alive_seen[-1] is False
    assert adapter.health()["status_snapshot_stale"] is True


def test_slam_stack_can_select_cpp_slam_status_adapter() -> None:
    bp = slam("bridge", enable_visual_backup=False, localization_adapter="cpp_slam_status")

    assert bp._entries[0].name == "SlamAdapterModule"
    assert bp._entries[0].module_cls is CppSlamStatusAdapterModule


def test_thunder_field_product_blueprints_use_cpp_slam_status_adapter() -> None:
    for profile in ("map", "nav", "explore", "tare_explore"):
        config = resolve_profile_config(profile)
        bp = blueprint_for_resolved_profile(profile, config)
        slam_entry = next(entry for entry in bp._entries if entry.name == "SlamAdapterModule")

        assert config["_runtime_endpoint"] == "thunder_field"
        assert config["_endpoint_transport"] == "dds"
        assert config["localization_adapter"] == "cpp_slam_status"
        assert "nav_in_adapter" not in config
        assert "nav_out_adapter" not in config
        assert config["native_navigation_endpoint"] == "lingtu-nav-dds"
        assert slam_entry.module_cls is CppSlamStatusAdapterModule


def test_cpp_slam_status_adapter_adds_map_frame_jump_delta() -> None:
    adapter = CppSlamStatusAdapterModule()
    events = []
    adapter.map_frame_jump_event.subscribe(events.append)

    baseline = _status_payload()
    adapter._publish_status_snapshot(baseline)

    same_pose_jump = json.loads(json.dumps(baseline))
    same_pose_jump["map_frame_jump"] = True
    same_pose_jump["reason"] = "relocalized"
    adapter._publish_status_snapshot(same_pose_jump)

    assert events[-1]["dt_m"] == 0.0
    assert events[-1]["dyaw_deg"] == 0.0
    assert events[-1]["old_xyz"] == [0.1, 0.2, 0.0]
    assert events[-1]["new_xyz"] == [0.1, 0.2, 0.0]

    moved_jump = json.loads(json.dumps(baseline))
    moved_jump["stamp_s"] = 124.0
    moved_jump["map_frame_jump"] = True
    moved_jump["reason"] = "relocalized"
    moved_jump["map_odom_tf"]["tx"] = 1.3
    adapter._publish_status_snapshot(moved_jump)

    assert events[-1]["dt_m"] == 1.2
    assert events[-1]["dyaw_deg"] == 0.0
    assert events[-1]["old_xyz"] == [0.1, 0.2, 0.0]
    assert events[-1]["new_xyz"] == [1.3, 0.2, 0.0]


def _status_payload() -> dict:
    return {
        "schema_version": STATUS_SNAPSHOT_SCHEMA,
        "source": "cpp_cyclone_slam",
        "backend": "fastlio2",
        "mode": "mapping",
        "state": "TRACKING",
        "reason": "tracking",
        "alive": True,
        "has_odom": True,
        "stamp_s": 123.0,
        "confidence": 0.8,
        "localization_quality": 0.7,
        "status_target_hz": 10.0,
        "imu_input_hz": 196.0,
        "lidar_input_hz": 10.0,
        "slam_tick_hz": 50.0,
        "processed_scan_hz": 9.8,
        "registered_points": 10,
        "map_points": 20,
        "imu_buffer": 3,
        "lidar_buffer": 1,
        "imu_batch": 18,
        "scan_start_s": 122.9,
        "scan_end_s": 123.0,
        "last_imu_s": 123.01,
        "sync_wait_count": 2,
        "imu_rollback_count": 0,
        "lidar_rollback_count": 0,
        "dropped_lidar_frames": 0,
        "dropped_imu_frames": 0,
        "map_loaded": True,
        "map_frame_jump": False,
        "scene_mode": "outdoor",
        "gnss_fusion_health": {
            "enabled": True,
            "alignment_locked": True,
            "last_fix_type": "RTK_FIXED",
            "last_gnss_age_s": 0.2,
            "last_residual_m": 0.03,
            "relock_count": 1,
        },
        "map_odom_tf": {
            "valid": True,
            "frame_id": "map",
            "child_frame_id": "odom",
            "tx": 0.1,
            "ty": 0.2,
            "tz": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "ts": 123.0,
        },
        "odometry": {
            "frame_id": "odom",
            "child_frame_id": "body",
            "pose": {
                "x": 1.0,
                "y": 2.0,
                "z": 0.3,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        },
    }
