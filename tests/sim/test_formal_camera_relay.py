"""UE-produced pixels join the Product's actual physics clock and endpoint."""

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from sim.adapters.shm.camera_ring import (
    CameraShmAllocation,
    CameraShmWriter,
    FrameMetadata,
    InMemoryMappingBackend,
    StreamKind,
)
from sim.scripts.mujoco.native_sensor_records import CAMERA_RECORD_HEADER, HEADER, RECORD_CAMERA


def test_ue_rgb_relay_uses_capture_truth_and_never_renders_mujoco(tmp_path):
    from sim.scripts.mujoco.formal_camera_relay import FormalCameraRelay

    from tests.sim.test_formal_feeder import FakeServices

    backend = InMemoryMappingBackend()
    allocation = SimpleNamespace(session_id="camera-session", shm={"robot.rgb": "camera-relay-rgb"})
    transport_ns = time.time_ns()
    mapping = backend.create(CameraShmAllocation("camera-relay-rgb", slot_capacity=6))
    writer = CameraShmWriter(mapping, CameraShmAllocation("camera-relay-rgb", slot_capacity=6),
                             now_ns=lambda: transport_ns, model_generation=3)
    sensor_plan = dict(schema="lingtu.sim.sensor-plan.v1", session_id="camera-session", streams={
        "rgb": [dict(sensor_id="robot.rgb", frame_id="robot/optical", owner="visual", source="unreal_camera",
                     transport="camera_shm", width=2, height=1, encoding="rgb8", rate_hz=30)], "depth": [],
    })
    services = FakeServices()
    delivered = threading.Event()
    client = SimpleNamespace(write=lambda wire: (services.sensor.records.append(wire), delivered.set()))
    relay = FormalCameraRelay(sensor_plan=sensor_plan, allocation=allocation, product_session_id="a" * 32,
        model_generation=3, reset_generation=7, client=client, services=services, session_root=tmp_path,
        stream_stats={}, mapping_backend=backend, slot_capacity=6)
    try:
        relay.observe_truth(dict(session_id="camera-session", model_generation=3, reset_generation=7,
                                sequence=17, sim_time_ns=500_000_000), epoch_s=1_800_000_000.0)
        writer.publish(FrameMetadata(
            StreamKind.COLOR, transport_ns, 2, 1, 6, "rgb8", "robot/optical",
            fx=1, fy=1, cx=0.5, cy=0, truth_sequence=17, reset_generation=7, sim_time_ns=500_000_000,
        ), bytes([10, 20, 30, 40, 50, 60]))
        relay.publish_due(monotonic_s=0, observed_s=0)
        assert delivered.wait(2)
    finally:
        relay.close()
    records = []
    for batch in services.sensor.records:
        offset = 0
        while offset < len(batch):
            size = HEADER.size + HEADER.unpack_from(batch, offset)[5]
            records.append(batch[offset:offset + size])
            offset += size
    images = [wire for wire in records if wire[4] == RECORD_CAMERA
              and CAMERA_RECORD_HEADER.unpack_from(wire, HEADER.size)[2] == 2]
    assert len(images) == 1
    assert HEADER.unpack_from(images[0])[2] == 1_800_000_000_000_000_000
    assert images[0][HEADER.size + CAMERA_RECORD_HEADER.size:] == bytes([10, 20, 30, 40, 50, 60])
    output = services.camera_writers["camera_color.shm"].published[0]
    assert output["payload"] == bytes([10, 20, 30, 40, 50, 60])
    assert output["frame_id"] == "robot/optical"
    assert "camera_depth.shm" not in services.camera_writers
    assert services.engine.camera_render_threads == []


def make_relay(tmp_path, *, kind="depth", fail_write=False):
    from sim.scripts.mujoco.formal_camera_relay import FormalCameraRelay
    from sim.scripts.mujoco.formal_feeder import _StreamStats

    from tests.sim.test_formal_feeder import FakeServices

    services, backend, delivered = FakeServices(), InMemoryMappingBackend(), threading.Event()
    allocation = SimpleNamespace(session_id="camera-session", shm={f"robot.{kind}": "camera-test"})
    ring = CameraShmAllocation("camera-test", slot_capacity=8)
    writer = CameraShmWriter(backend.create(ring), ring, model_generation=3)
    stats = _StreamStats(30)
    spec = dict(sensor_id=f"robot.{kind}", frame_id="robot/optical", owner="visual", source="unreal_camera",
                transport="camera_shm", width=4 if kind == "depth" else 2, height=1,
                encoding="16UC1" if kind == "depth" else "rgb8", rate_hz=30)

    def write(wire):
        if fail_write:
            delivered.set()
            raise OSError("endpoint disappeared")
        services.sensor.records.append(wire)
        delivered.set()

    relay = FormalCameraRelay(sensor_plan=dict(session_id=allocation.session_id, streams={kind: [spec]}),
        allocation=allocation, product_session_id="a" * 32, model_generation=3, reset_generation=7,
        client=SimpleNamespace(write=write), services=services, session_root=tmp_path,
        stream_stats={kind: stats}, mapping_backend=backend, slot_capacity=8)

    def capture(*, truth=17, reset=7, sim_time=500_000_000, epoch_offset=0.0, stamped=True):
        relay.observe_truth(dict(session_id="camera-session", model_generation=3, reset_generation=7,
                            sequence=17, sim_time_ns=500_000_000), epoch_s=services.wall_time() + epoch_offset)
        stamp = dict(truth_sequence=truth, reset_generation=reset, sim_time_ns=sim_time) if stamped else {}
        writer.publish(FrameMetadata(StreamKind.DEPTH if kind == "depth" else StreamKind.COLOR,
            time.time_ns(), spec["width"], 1, 8 if kind == "depth" else 6, spec["encoding"], spec["frame_id"],
            fx=1, fy=1, cx=0.5, cy=0, **stamp),
            np.array([0, 1, 1000, 65535], dtype="<u2").tobytes() if kind == "depth" else bytes(range(6)))

    return relay, capture, services, stats, delivered


def test_depth_relay_preserves_zero_and_full_millimetre_range(tmp_path):
    relay, capture, services, stats, delivered = make_relay(tmp_path)
    try:
        relay.begin(started_s=0, started_wall_s=services.wall_time())
        capture()
        relay.publish_due(monotonic_s=0)
        assert delivered.wait(2)
    finally:
        relay.close()
    assert set(services.camera_writers) == {"camera_depth.shm", "camera_info.shm"}
    out = services.camera_writers["camera_depth.shm"].published
    np.testing.assert_array_equal(np.frombuffer(out[0]["payload"], dtype="<u2"), [0, 1, 1000, 65535])
    assert (stats.scheduled_count, stats.published_count, stats.dropped_count) == (1, 1, 0)
    assert relay.evidence()["streams"]["depth"]["truth_sequence"] == 17


@pytest.mark.parametrize("change", [dict(reset=8), dict(truth=18), dict(sim_time=600_000_000), dict(stamped=False)])
def test_relay_rejects_cross_generation_unpublished_truth_or_missing_stamp(tmp_path, change):
    relay, capture, services, _, _ = make_relay(tmp_path)
    try:
        capture(**change)
        with pytest.raises(ValueError):
            relay.publish_due(monotonic_s=0)
        assert not services.sensor.records
        assert not relay.ready()
    finally:
        relay.close()


def test_fresh_transport_cannot_turn_old_simulation_capture_into_fresh_measurement(tmp_path):
    relay, capture, services, _, _ = make_relay(tmp_path)
    try:
        relay.begin(started_s=0, started_wall_s=services.wall_time())
        capture(epoch_offset=-2)
        relay.publish_due(monotonic_s=0)
    finally:
        relay.close()
    assert not services.sensor.records
    assert not relay.ready()


def test_relay_endpoint_failure_settles_all_scheduled_samples(tmp_path):
    relay, capture, services, stats, delivered = make_relay(tmp_path, fail_write=True)
    relay.begin(started_s=0, started_wall_s=services.wall_time())
    capture()
    relay.publish_due(monotonic_s=0)
    assert delivered.wait(2)
    with pytest.raises(RuntimeError, match="endpoint disappeared"):
        relay.close()
    assert (stats.scheduled_count, stats.published_count, stats.dropped_count) == (1, 0, 1)


def test_stalled_selected_camera_fails_instead_of_reusing_last_frame(tmp_path):
    relay, capture, services, stats, delivered = make_relay(tmp_path)
    try:
        relay.begin(started_s=0, started_wall_s=services.wall_time())
        capture()
        relay.publish_due(monotonic_s=0)
        assert delivered.wait(2)
        with pytest.raises(RuntimeError, match="stopped advancing"):
            relay.publish_due(monotonic_s=1.01)
    finally:
        relay.close()
    assert stats.scheduled_count == stats.published_count == 1


def test_low_rate_camera_watchdog_uses_selected_period_not_one_second(tmp_path):
    relay, _, services, _, _ = make_relay(tmp_path)
    try:
        relay._specs["depth"]["rate_hz"] = 0.5
        relay.begin(started_s=0, started_wall_s=services.wall_time())
        relay.publish_due(monotonic_s=1.5)
        with pytest.raises(RuntimeError, match="stopped advancing"):
            relay.publish_due(monotonic_s=6.01)
    finally:
        relay.close()


def test_declared_camera_slots_advance_even_when_source_produces_nothing(tmp_path):
    relay, _, services, _, _ = make_relay(tmp_path)
    try:
        relay.begin(started_s=0, started_wall_s=services.wall_time())
        relay.publish_due(monotonic_s=0.5)
        schedule = relay.evidence()["declared_schedule"]["depth"]
        assert schedule == dict(rate_hz=30, scheduled_count=15, published_count=0, unfilled_count=15)
    finally:
        relay.close()


def test_delayed_startup_stamp_remains_original_and_is_not_a_fresh_runtime_sample(tmp_path):
    relay, capture, services, stats, delivered = make_relay(tmp_path)
    try:
        capture(epoch_offset=-5)
        relay.publish_due(monotonic_s=5)
        assert delivered.wait(2)
        relay.begin(started_s=5, started_wall_s=services.wall_time())
        with pytest.raises(RuntimeError, match="stopped advancing"):
            relay.publish_due(monotonic_s=6.01)
        with pytest.raises(ValueError, match="restamped"):
            relay.observe_truth(dict(session_id="camera-session", model_generation=3, reset_generation=7,
                sequence=17, sim_time_ns=500_000_000), epoch_s=services.wall_time())
    finally:
        relay.close()
    first = services.camera_writers["camera_depth.shm"].published[0]
    assert first["timestamp_ns"] == int((services.wall_time() - 5) * 1e9)
    assert (stats.scheduled_count, stats.published_count, stats.dropped_count) == (0, 0, 0)
