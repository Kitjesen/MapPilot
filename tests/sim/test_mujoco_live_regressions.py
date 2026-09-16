import json
import threading
import time
from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest
from sim.scripts.mujoco import formal_feeder as feeder

from drivers.sim.mujoco.runtime import NavigationOverlay


@pytest.mark.parametrize("age,session,ready", [
    (0, "current", True), (60, "current", False), (0, "old", False),
])
def test_viewer_rejects_stale_and_foreign_ready_snapshots(tmp_path, age, session, ready):
    path = tmp_path / "nav.status.json"
    path.write_text(json.dumps({
        "stamp_s": time.time() - age,
        "native_product": {"product_session_id": session},
        "input_gate": {"ready": True}, "global_path": [[0, 0, 0], [1, 0, 0]],
    }))
    status = feeder._read_navigation_status(path, "current")
    assert status["input_gate"]["ready"] is ready
    if not ready:
        assert "global_path" not in status


def test_overlay_reuses_static_geometry_and_transforms_only_map_data(monkeypatch):
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size=".1"/></worldbody></mujoco>')
    viewer = SimpleNamespace(lock=nullcontext, user_scn=mujoco.MjvScene(model, maxgeom=20))
    status = {
        "global_path": [[10, 20, 0], [11, 20, 0]],
        "local_map": {"enabled": True, "frame_id": "map", "obstacle_points": [[12, 21, 0]]},
    }
    rotation = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    transform = (rotation, np.array([-20, 10, 0]))
    overlay = NavigationOverlay()
    calls = []
    init = mujoco.mjv_initGeom

    def counted(*args):
        calls.append(True)
        init(*args)

    monkeypatch.setattr(mujoco, "mjv_initGeom", counted)
    overlay.draw(viewer, status, [[3, 4, 0], [4, 4, 0]], transform)
    assert len(calls) == 3
    np.testing.assert_allclose(viewer.user_scn.geoms[0].pos, [0, -.5, .06])
    np.testing.assert_allclose(viewer.user_scn.geoms[1].pos, [1, -2, 0])
    np.testing.assert_allclose(viewer.user_scn.geoms[2].pos, [3.5, 4, .02])
    # An unrelated status timestamp and a marker do not invalidate native geometry.
    viewer.user_scn.ngeom += 1
    overlay.draw(viewer, {**status, "stamp_s": 42}, [[3, 4, 0], [4, 4, 0]], transform)
    assert len(calls) == 3
    assert viewer.user_scn.ngeom == 3
    overlay.draw(viewer, status, [[3, 4, 0], [5, 4, 0]], transform)
    assert len(calls) == 4
    overlay.draw(viewer, {}, [], transform)
    assert viewer.user_scn.ngeom == 0


def frame(sequence, *, captured_at_s=None):
    return feeder._LidarFrame(
        snapshot=None, state=None, frame_start_s=0, monotonic_s=0, wall_s=0,
        sequence=sequence, registered_sequence=None,
        captured_at_s=time.monotonic() if captured_at_s is None else captured_at_s,
    )


def test_lidar_origin_uses_scan_snapshot_and_rotated_mount():
    mujoco = pytest.importorskip("mujoco")
    from drivers.sim.mujoco.sensors import lidar_pose_world, world_xyzi_to_sensor_xyzi

    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base" pos="3 4 .5" euler="0 0 90">
        <site name="lidar_site" pos="-.3 0 .2"/>
      </body></worldbody></mujoco>''')
    snapshot = mujoco.MjData(model)
    mujoco.mj_forward(model, snapshot)
    engine = SimpleNamespace(_model=model, _data=mujoco.MjData(model))
    origin, _ = lidar_pose_world(engine, data=snapshot)
    np.testing.assert_allclose(origin, [3, 3.7, .7], atol=1e-6)
    cloud = np.array([[*origin, 11]], dtype=np.float32)
    local = world_xyzi_to_sensor_xyzi(engine, cloud, data=snapshot)
    np.testing.assert_allclose(local[0, :3], 0, atol=1e-6)


def test_lidar_overload_drops_obsolete_pending_work_and_keeps_exact_accounting(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    published = []
    stats = feeder._StreamStats(expected_hz=10)

    def publish(self, task):
        if task.sequence == 0:
            entered.set()
            assert release.wait(3)
        published.append(task.sequence)
        stats.published()

    monkeypatch.setattr(feeder._LidarPublisher, "_publish_frame", publish)
    publisher = feeder._LidarPublisher(
        client=SimpleNamespace(write=lambda _: None), engine=None, samples_per_frame=20,
        max_points=20, publish_registered_cloud_fixture=False, stats=stats,
    )
    try:
        stats.due(count=1, dropped=0, lateness_s=0)
        publisher.enqueue_frame(frame(0))
        assert entered.wait(1)
        for sequence in range(1, 21):
            stats.due(count=1, dropped=0, lateness_s=0)
            publisher.enqueue_frame(frame(sequence))
        assert publisher._tasks.qsize() == 1
    finally:
        release.set()
        publisher.close()
    assert published == [0, 20]
    assert (stats.scheduled_count, stats.published_count, stats.dropped_count) == (21, 2, 19)


def test_expired_lidar_frame_is_dropped_before_expensive_raycast():
    stats = feeder._StreamStats(expected_hz=10)
    stats.due(count=1, dropped=0, lateness_s=0)
    publisher = feeder._LidarPublisher(
        client=None, engine=None, samples_per_frame=20, max_points=20,
        publish_registered_cloud_fixture=False, stats=stats,
    )
    publisher.enqueue_frame(frame(0, captured_at_s=time.monotonic() - 2))
    publisher.close()
    assert stats.dropped_count == 1
    assert stats.published_count == 0
