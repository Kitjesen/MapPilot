from __future__ import annotations

import json
import time

import pytest

from cli.runtime_extra import _uses_non_ros_localization_adapter
from lingtu.assembly.graph import graph_for_profile
from lingtu.assembly.products import resolve_product_host_config
from lingtu.assembly.profile_builder import blueprint_for_resolved_product
from lingtu.assembly.stacks.slam import slam
from localization.adapters.resolver import localization_adapter_module
from localization.adapters.status import (
    STATUS_SNAPSHOT_HEALTH_SOURCE,
    STATUS_SNAPSHOT_SCHEMA,
    CppSlamStatusAdapterModule,
)
from runtime.msgs.map import MapObservationFrame
from runtime.msgs.sensor import PointCloud2
from runtime.registry import list_plugins


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
    assert status_seen[-1]["runtime_instance_id"] == "slam-test-runtime"
    assert status_seen[-1]["source_epoch"] == 1
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
    assert status_seen[-1]["relocalization_refine_input_points"] == 485
    assert status_seen[-1]["relocalization_refine_evaluated_points"] == 298
    assert status_seen[-1]["relocalization_min_inliers"] == 30
    assert status_seen[-1]["relocalization_min_evaluated_points"] == 100
    assert status_seen[-1]["relocalization_refine_support_ratio"] == pytest.approx(
        298 / 485
    )
    assert status_seen[-1][
        "relocalization_refine_overlap_inlier_ratio"
    ] == pytest.approx(292 / 298)
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


def test_cpp_slam_status_adapter_reads_lidar_scan_snapshot(tmp_path) -> None:
    status_path = tmp_path / "slam_status.json"
    cloud_dir = tmp_path / "clouds"
    cloud_dir.mkdir()
    (cloud_dir / "lidar_scan.bin").write_bytes(
        PointCloud2(
            points=[[0.1, 0.2, 0.3, 10.0], [1.0, 2.0, 3.0, 20.0]],
            ts=125.0,
            frame_id="livox_frame",
        ).encode()
    )
    adapter = CppSlamStatusAdapterModule(
        status_snapshot_path=str(status_path),
        cloud_snapshot_dir=str(cloud_dir),
        status_snapshot_interval_s=0.05,
        status_snapshot_stale_after_s=1.0,
    )
    scans_seen = []
    adapter.lidar_scan.subscribe(scans_seen.append)

    adapter.setup()
    try:
        deadline = time.time() + 1.0
        while time.time() < deadline and not scans_seen:
            time.sleep(0.02)
    finally:
        adapter.stop()

    assert scans_seen[-1].frame_id == "livox_frame"
    assert scans_seen[-1].num_points == 2
    assert scans_seen[-1].points[0, 0] == pytest.approx(0.1)
    assert adapter.health()["message_counts"]["/lidar/raw_frame"] >= 1


def test_cpp_slam_status_adapter_pairs_registered_scan_with_exact_pose() -> None:
    adapter = CppSlamStatusAdapterModule()
    observations: list[MapObservationFrame] = []
    adapter.map_observation.subscribe(observations.append)
    status = _status_payload()
    status["observation_sequence"] = 9
    status["state_estimation_at_scan"] = {
        "stamp_s": 123.0,
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
    }

    adapter._publish_status_snapshot(status)
    adapter._latest_registered_cloud = PointCloud2(points=[[1.0, 0.0, 0.0]], ts=123.0, frame_id="body")
    adapter._publish_map_observation_if_ready()
    adapter._publish_map_observation_if_ready()

    assert len(observations) == 1
    frame = observations[0]
    assert frame.sequence == 9
    assert frame.sensor_origin.x == pytest.approx(1.1)
    assert frame.sensor_origin.y == pytest.approx(2.2)
    assert frame.sensor_origin.z == pytest.approx(0.3)
    assert frame.to_map_pointcloud2().points[0, :3].tolist() == pytest.approx([2.1, 2.2, 0.3])


def test_cpp_slam_status_adapter_rejects_unpaired_registered_scan() -> None:
    adapter = CppSlamStatusAdapterModule()
    observations: list[MapObservationFrame] = []
    adapter.map_observation.subscribe(observations.append)
    status = _status_payload()
    status["observation_sequence"] = 10
    status["state_estimation_at_scan"] = {
        "stamp_s": 123.0,
        "frame_id": "odom",
        "child_frame_id": "body",
        "pose": status["odometry"]["pose"],
    }
    adapter._publish_status_snapshot(status)
    adapter._latest_registered_cloud = PointCloud2(points=[[1.0, 0.0, 0.0]], ts=123.1, frame_id="body")

    adapter._publish_map_observation_if_ready()

    assert observations == []


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
    assert status_seen[-1]["runtime_instance_id"] == "slam-test-runtime"
    assert status_seen[-1]["observation_sequence"] == 0
    assert status_seen[-1]["map_frame_jump_sequence"] == 0
    assert alive_seen[-1] is False
    assert adapter.health()["status_snapshot_stale"] is True


def test_cpp_slam_status_adapter_marks_rewritten_but_stalled_source_stale(tmp_path) -> None:
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
    payload = _status_payload()
    payload["observation_sequence"] = 4
    payload["state_estimation_at_scan"] = {
        "stamp_s": 123.0,
        "frame_id": "odom",
        "child_frame_id": "body",
        "pose": payload["odometry"]["pose"],
    }

    adapter.setup()
    try:
        path.write_text(json.dumps(payload), encoding="utf-8")
        deadline = time.time() + 1.0
        while time.time() < deadline and not status_seen:
            time.sleep(0.02)

        deadline = time.time() + 1.0
        while time.time() < deadline and status_seen[-1]["state"] != "STALE":
            path.write_text(json.dumps(payload), encoding="utf-8")
            time.sleep(0.06)
        stalled_status = dict(status_seen[-1])

        previous_count = len(status_seen)
        path.write_text(json.dumps(payload), encoding="utf-8")
        deadline = time.time() + 1.0
        while time.time() < deadline and len(status_seen) == previous_count:
            time.sleep(0.02)
        repeated_status = dict(status_seen[-1])

        payload["runtime_instance_id"] = "slam-restarted-runtime"
        previous_count = len(status_seen)
        path.write_text(json.dumps(payload), encoding="utf-8")
        deadline = time.time() + 1.0
        while time.time() < deadline and len(status_seen) == previous_count:
            time.sleep(0.02)
        restarted_without_source_status = dict(status_seen[-1])

        payload["observation_sequence"] = 5
        payload["stamp_s"] = 124.0
        payload["state_estimation_at_scan"]["stamp_s"] = 124.0
        path.write_text(json.dumps(payload), encoding="utf-8")
        deadline = time.time() + 1.0
        while time.time() < deadline and status_seen[-1]["state"] != "TRACKING":
            time.sleep(0.02)
        recovered_status = dict(status_seen[-1])
    finally:
        adapter.stop()

    assert stalled_status["state"] == "STALE"
    assert stalled_status["reason"] == "slam_runtime_source_stale"
    assert stalled_status["alive"] is False
    assert stalled_status["imu_input_hz"] == 0.0
    assert stalled_status["lidar_input_hz"] == 0.0
    assert stalled_status["slam_tick_hz"] == 0.0
    assert stalled_status["processed_scan_hz"] == 0.0
    assert repeated_status["state"] == "STALE"
    assert restarted_without_source_status["state"] == "STALE"
    assert recovered_status["state"] == "TRACKING"
    assert recovered_status["observation_sequence"] == 5
    assert recovered_status["processed_scan_hz"] == 9.8
    assert alive_seen[-1] is True


def test_cpp_slam_status_adapter_is_the_only_status_adapter_name() -> None:
    assert localization_adapter_module("cpp_slam_status") is CppSlamStatusAdapterModule
    assert "native_slam_status" not in list_plugins("localization_adapter")
    assert _uses_non_ros_localization_adapter(
        {"localization_adapter": "cpp_slam_status"}
    )

    graph = graph_for_profile(
        "dev",
        slam_profile="bridge",
        localization_adapter="cpp_slam_status",
    )
    assert "SlamAdapterModule" in graph.modules
    assert "SlamModule" not in graph.modules


@pytest.mark.parametrize(
    "adapter_name",
    ("native_slam_status", "native", "native_slam", "slam"),
)
def test_removed_localization_adapter_aliases_fail_closed(adapter_name: str) -> None:
    with pytest.raises(ImportError, match="was removed"):
        localization_adapter_module(adapter_name)

    assert not _uses_non_ros_localization_adapter(
        {"localization_adapter": adapter_name}
    )
    assert not slam(
        "fastlio2",
        enable_visual_backup=False,
        localization_adapter=adapter_name,
    )._entries
    with pytest.raises(ImportError, match="was removed"):
        graph_for_profile(
            "dev",
            slam_profile="fastlio2",
            localization_adapter=adapter_name,
        )

def test_slam_stack_can_select_cpp_slam_status_adapter() -> None:
    bp = slam("bridge", enable_visual_backup=False, localization_adapter="cpp_slam_status")

    assert bp._entries[0].name == "SlamAdapterModule"
    assert bp._entries[0].module_cls is CppSlamStatusAdapterModule


@pytest.mark.parametrize(
    ("product", "product_variant"),
    (
        ("map", None),
        ("nav", None),
        ("explore", "live"),
        ("explore", "map"),
    ),
)
def test_real_product_blueprints_use_cpp_slam_status_adapter(
    product: str,
    product_variant: str | None,
) -> None:
    config = resolve_product_host_config(
        product,
        "real",
        product_variant=product_variant,
    )
    bp = blueprint_for_resolved_product(product, config)
    slam_entry = next(entry for entry in bp._entries if entry.name == "SlamAdapterModule")

    assert config["_selection_kind"] == "product"
    assert config["_env"] == "real"
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


def test_cpp_slam_status_adapter_resets_observation_cursor_for_new_runtime() -> None:
    adapter = CppSlamStatusAdapterModule()
    observations: list[MapObservationFrame] = []
    adapter.map_observation.subscribe(observations.append)

    first = _status_payload()
    first["observation_sequence"] = 10
    first["state_estimation_at_scan"] = {
        "stamp_s": 123.0,
        "frame_id": "odom",
        "child_frame_id": "body",
        "pose": first["odometry"]["pose"],
    }
    adapter._latest_registered_cloud = PointCloud2(points=[[1.0, 0.0, 0.0]], ts=123.0, frame_id="body")
    adapter._publish_status_snapshot(first)

    second = json.loads(json.dumps(first))
    second["runtime_instance_id"] = "slam-new-runtime"
    second["source_epoch"] = 2
    second["observation_sequence"] = 1
    second["stamp_s"] = 200.0
    second["state_estimation_at_scan"]["stamp_s"] = 200.0
    adapter._latest_registered_cloud = PointCloud2(points=[[2.0, 0.0, 0.0]], ts=200.0, frame_id="body")
    adapter._publish_status_snapshot(second)

    assert [frame.sequence for frame in observations] == [10, 1]
    assert [frame.reset_epoch for frame in observations] == [1, 2]
    assert adapter._last_observation_sequence == 1
    assert adapter._last_observation_runtime_id == "slam-new-runtime"


def test_cpp_slam_status_adapter_recovers_after_same_runtime_source_epoch_reset() -> None:
    adapter = CppSlamStatusAdapterModule(status_snapshot_stale_after_s=0.1)
    statuses: list[dict] = []
    observations: list[MapObservationFrame] = []
    adapter.localization_status.subscribe(statuses.append)
    adapter.map_observation.subscribe(observations.append)

    first = _status_payload()
    first["source_epoch"] = 7
    first["observation_sequence"] = 10
    first["state_estimation_at_scan"] = {
        "stamp_s": 123.0,
        "frame_id": "odom",
        "child_frame_id": "body",
        "pose": first["odometry"]["pose"],
    }
    adapter._latest_registered_cloud = PointCloud2(
        points=[[1.0, 0.0, 0.0]], ts=123.0, frame_id="body"
    )
    adapter._publish_status_snapshot(first)

    adapter._last_source_progress_monotonic -= 1.0
    adapter._publish_status_snapshot(first)
    assert statuses[-1]["state"] == "STALE"

    recovered = json.loads(json.dumps(first))
    recovered["source_epoch"] = 8
    recovered["observation_sequence"] = 1
    recovered["stamp_s"] = 10.0
    recovered["scan_end_s"] = 10.0
    recovered["state_estimation_at_scan"]["stamp_s"] = 10.0
    adapter._latest_registered_cloud = PointCloud2(
        points=[[2.0, 0.0, 0.0]], ts=10.0, frame_id="body"
    )
    adapter._publish_status_snapshot(recovered)

    assert statuses[-1]["state"] == "TRACKING"
    assert statuses[-1]["source_epoch"] == 8
    assert statuses[-1]["source_data_stale"] is False
    assert [frame.sequence for frame in observations] == [10, 1]


def test_cpp_slam_status_adapter_rejects_runaway_fastlio_velocity() -> None:
    adapter = CppSlamStatusAdapterModule()
    odometry_seen = []
    status_seen = []
    alive_seen = []
    adapter.odometry.subscribe(odometry_seen.append)
    adapter.localization_status.subscribe(status_seen.append)
    adapter.alive.subscribe(alive_seen.append)

    payload = _status_payload()
    payload["fastlio_velocity"] = {
        "x": -58.9,
        "y": 58.1,
        "z": -2804.8,
    }
    adapter._publish_status_snapshot(payload)

    assert odometry_seen == []
    assert status_seen[-1]["state"] == "DIVERGED"
    assert status_seen[-1]["reason"] == "fastlio_velocity_out_of_bounds"
    assert status_seen[-1]["fastlio_speed_mps"] > 2800.0
    assert status_seen[-1]["alive"] is False
    assert alive_seen[-1] is False


def _status_payload() -> dict:
    return {
        "schema_version": STATUS_SNAPSHOT_SCHEMA,
        "runtime_instance_id": "slam-test-runtime",
        "source_epoch": 1,
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
        "observation_sequence": 0,
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
        "map_frame_jump_sequence": 0,
        "scene_mode": "outdoor",
        "relocalization_refine_backend": "fixed_transform_seed_check",
        "relocalization_refine_iterations": 0,
        "relocalization_refine_inliers": 292,
        "relocalization_refine_input_points": 485,
        "relocalization_refine_evaluated_points": 298,
        "relocalization_min_inliers": 30,
        "relocalization_min_evaluated_points": 100,
        "relocalization_refine_support_ratio": 298 / 485,
        "relocalization_refine_overlap_inlier_ratio": 292 / 298,
        "relocalization_refine_converged": True,
        "relocalization_refine_pos_cov_trace": 0.01,
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
