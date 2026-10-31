"""Selected formal cameras publish only declared streams on the owned endpoint."""

import threading
from contextlib import contextmanager
from copy import deepcopy
from types import SimpleNamespace

import pytest
from sim.scripts.mujoco import formal_feeder as feeder
from sim.scripts.mujoco.evidence import SimFeederStatusError, load_feeder_status, publish_feeder_status
from sim.scripts.mujoco.native_sensor_records import HEADER, RECORD_CAMERA

from lingtu.run_plan import RunPlan
from tests.sim.test_formal_feeder import FakeServices, _run, _run_plan
from tests.sim.test_formal_measurement_feeder import _with_measurements


def selected_plan(*, rgb_hz=15.0, depth_hz=None):
    payload = _run_plan(camera=True, enable_camera=True).as_dict()
    streams = payload["launch"]["simulation"]["sensor_plan"]["streams"]
    for kind, rate in (("rgb", rgb_hz), ("depth", depth_hz)):
        if rate is None:
            streams[kind] = []
        else:
            streams[kind][0].update(
                rate_hz=rate,
                owner="sensor_runtime",
                source="mujoco_camera",
                transport="camera_shm",
                message_type="lingtu.dds.Image",
            )
    return RunPlan.from_dict(payload)


def camera_stamps(services, kind):
    return [
        HEADER.unpack_from(wire)[2]
        for wire in services.sensors["camera_publisher"].records
        if wire[4] == RECORD_CAMERA and wire[HEADER.size + 6] == kind
    ]


@pytest.mark.parametrize(
    "kind,wire_kind,writer_name", [("rgb", 2, "camera_color.shm"), ("depth", 3, "camera_depth.shm")]
)
def test_selected_single_stream_never_publishes_or_requires_disabled_peer(
    monkeypatch, tmp_path, kind, wire_kind, writer_name
):
    services = FakeServices()
    peer = "depth" if kind == "rgb" else "rgb"
    setattr(services.engine.camera_result, peer, None)

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=selected_plan(rgb_hz=15 if kind == "rgb" else None, depth_hz=15 if kind == "depth" else None),
    )

    assert rc == 0
    assert len(camera_stamps(services, wire_kind)) >= 1
    assert camera_stamps(services, 5 - wire_kind) == []
    assert set(services.camera_writers) == {writer_name, "camera_info.shm"}
    assert services.build_kwargs["camera_configs"][0].fps == 15.0
    assert services.build_kwargs["camera_configs"][0].render_depth is (kind == "depth")
    assert f"camera_{peer}" not in services.status_payloads[-1]["streams"]
    assert services.events.count("engine_close") == 1


@pytest.mark.parametrize(
    "owner,source,transport",
    [
        ("visual", "unreal_camera", "camera_shm"),
        ("sensor_runtime", "unreal_camera", "camera_shm"),
        ("visual", "mujoco_camera", "camera_shm"),
        ("sensor_runtime", "mujoco_camera", "typed_dds"),
    ],
)
def test_unsupported_declared_camera_route_fails_before_engine(monkeypatch, tmp_path, capsys, owner, source, transport):
    payload = selected_plan().as_dict()
    payload["launch"]["simulation"]["sensor_plan"]["streams"]["rgb"][0].update(
        owner=owner, source=source, transport=transport
    )
    services = FakeServices()
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=RunPlan.from_dict(payload))
    assert rc == 1
    assert services.build_kwargs is None
    assert services.connected_paths == []
    assert f"{owner}/{source}/{transport}" in capsys.readouterr().err


def test_exact_unreal_camera_route_is_accepted_only_in_explicit_unreal_mode():
    payload = selected_plan(rgb_hz=30, depth_hz=30).as_dict()
    simulation = payload["launch"]["simulation"]
    simulation["session"]["runtime"]["mode"] = "unreal"
    for kind in ("rgb", "depth"):
        simulation["sensor_plan"]["streams"][kind][0].update(owner="visual", source="unreal_camera")
    config = feeder._run_plan_runtime_config(feeder._parser().parse_args([]), RunPlan.from_dict(payload))
    assert config.camera_source == "unreal_camera"
    assert config.camera_stream_rates == {"rgb": 30, "depth": 30}
    assert config.visual_enabled is True


def make_pipeline(tmp_path, rates):
    services = FakeServices(stop_after_readiness=False)
    stats = {kind: feeder._StreamStats(rate) for kind, rate in rates.items()}
    pipeline = feeder._CameraPipeline(
        services=services,
        client=services.sensors["camera_publisher"],
        engine=services.engine,
        session_root=tmp_path,
        started_s=0,
        started_wall_s=1_800_000_000,
        rate_hz=max(rates.values()),
        stream_rates=rates,
        stream_stats=stats,
    )
    return services, stats, pipeline


def test_independent_nonmultiple_rates_publish_exact_due_frames_without_downsampling(tmp_path):
    services, stats, pipeline = make_pipeline(tmp_path, {"rgb": 15, "depth": 10})
    condition = threading.Condition()
    write = services.sensors["camera_publisher"].write

    def record(wire):
        with condition:
            write(wire)
            condition.notify_all()

    services.sensors["camera_publisher"].write = record
    try:
        pipeline.publish_initial()
        # Warmup/readiness must not shift or consume runtime sample slots.
        pipeline.begin(started_s=5, started_wall_s=1_800_000_005)
        for tick in range(1, 31):
            offset = tick / 30
            services.engine.sim_time = offset
            pipeline.publish_due(monotonic_s=5 + offset, observed_s=5 + offset)
            with condition:
                assert condition.wait_for(
                    lambda tick=tick: (
                        len(camera_stamps(services, 2)) == 1 + tick // 2
                        and len(camera_stamps(services, 3)) == 1 + tick // 3
                    ),
                    timeout=2,
                )
    finally:
        pipeline.close()
    for kind, wire_kind, rate in (("rgb", 2, 15), ("depth", 3, 10)):
        counts = stats[kind].payload(1)
        assert counts["scheduled_count"] == counts["published_count"] == rate
        assert counts["dropped_count"] == 0
        assert camera_stamps(services, wire_kind)[1:] == [
            int((1_800_000_005 + index / rate) * 1e9) for index in range(1, rate + 1)
        ]
    assert len(camera_stamps(services, 1)) == 1
    assert len(set(services.engine.camera_render_threads)) == 1
    assert services.engine.camera_close_threads == [services.engine.camera_render_threads[0]]
    assert services.engine.camera_render_threads[0] != threading.get_ident()


def test_independent_streams_expose_queue_replacement_and_late_slot_drops(tmp_path):
    services, stats, pipeline = make_pipeline(tmp_path, {"rgb": 10, "depth": 4})
    entered, release = threading.Event(), threading.Event()
    render = services.engine._cameras["front_camera"].render_snapshot
    try:
        pipeline.publish_initial()

        def blocked(snapshot):
            entered.set()
            assert release.wait(2)
            return render(snapshot)

        services.engine._cameras["front_camera"].render_snapshot = blocked
        services.engine.sim_time = 0.1
        pipeline.publish_due(monotonic_s=0.1)
        assert entered.wait(2)
        for offset in (0.25, 0.3, 0.71):
            services.engine.sim_time = offset
            pipeline.publish_due(monotonic_s=offset, observed_s=0.8 if offset == 0.71 else offset)
        release.set()
    finally:
        release.set()
        pipeline.close()
    assert camera_stamps(services, 2) == [int((1_800_000_000 + t) * 1e9) for t in (0, 0.1, 0.8)]
    assert camera_stamps(services, 3) == [int((1_800_000_000 + t) * 1e9) for t in (0, 0.8)]
    assert stats["rgb"].payload(1)["scheduled_count"] == 7
    assert stats["rgb"].payload(1)["published_count"] == 2
    assert stats["rgb"].payload(1)["dropped_count"] == 5
    assert stats["depth"].payload(1)["scheduled_count"] == 2
    assert stats["depth"].payload(1)["published_count"] == 1
    assert stats["depth"].payload(1)["dropped_count"] == 1
    assert [float(snapshot[0]) for snapshot in services.engine.camera_snapshots] == [0, 0.1, 0.71]


def test_partial_transport_failure_does_not_drop_already_published_peer(tmp_path):
    services, stats, pipeline = make_pipeline(tmp_path, {"rgb": 10, "depth": 10})
    write = services.sensors["camera_publisher"].write
    try:
        pipeline.publish_initial()

        def fail_depth(wire):
            if wire[HEADER.size + 6] == 3:
                raise OSError("synthetic depth endpoint write failure")
            write(wire)

        services.sensors["camera_publisher"].write = fail_depth
        pipeline.publish_due(monotonic_s=0.1)
    finally:
        with pytest.raises(feeder.FormalFeederError, match="synthetic depth"):
            pipeline.close()
    assert stats["rgb"].payload(1)["published_count"] == 1
    assert stats["rgb"].payload(1)["dropped_count"] == 0
    assert stats["depth"].payload(1)["published_count"] == 0
    assert stats["depth"].payload(1)["dropped_count"] == 1
    assert len(camera_stamps(services, 2)) == 2
    assert len(camera_stamps(services, 3)) == 1
    assert len(services.engine.camera_close_threads) == 1
    assert sum(event[0] == "camera_writer_close" for event in services.events) == 3


def test_selected_rate_above_physics_sampling_is_rejected_not_downsampled(monkeypatch, tmp_path):
    services = FakeServices()
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=selected_plan(rgb_hz=250))
    assert rc == 1
    assert services.build_kwargs is None
    assert services.connected_paths == []


@pytest.mark.parametrize("name", ["camera_rgbd", "camera_rgb", "camera_depth", "camera_unknown"])
def test_camera_evidence_accepts_known_streams_and_rejects_unknown(monkeypatch, tmp_path, name):
    services = FakeServices()
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=_run_plan(camera=True, enable_camera=True))
    assert rc == 0
    payload = deepcopy(services.status_payloads[-1])
    payload["streams"] = {name: payload["streams"]["camera_rgbd"]}

    def publish():
        return publish_feeder_status(
            session_root=tmp_path, payload=payload, environment={"LINGTU_PROCESS_LAUNCH_ID": "a" * 64}
        )

    if name == "camera_unknown":
        with pytest.raises(SimFeederStatusError, match="streams"):
            publish()
    else:
        publish()
        stored = load_feeder_status(
            session_root=tmp_path,
            product=payload["product"],
            product_session_id=payload["product_session_id"],
            process=payload["process"],
            launch_id="a" * 64,
        )
        assert stored["streams"] == payload["streams"]


def test_selected_rates_report_independent_schedules_and_wire_counts(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_PROCESS_LAUNCH_ID", "a" * 64)
    services = FakeServices(stop_after_readiness=False, stop_after_waits=210, publish_real_artifacts=True)
    rc, identity, _ = _run(monkeypatch, tmp_path, services, plan=selected_plan(rgb_hz=30, depth_hz=7))
    assert rc == 0
    payload = services.status_payloads[-1]
    assert "camera_rgbd" not in payload["streams"]
    for kind, wire_kind, rate in (("rgb", 2, 30), ("depth", 3, 7)):
        stats = payload["streams"][f"camera_{kind}"]
        assert stats["expected_hz"] == rate
        assert stats["scheduled_count"] == pytest.approx(payload["window_s"] * rate, abs=1)
        assert stats["published_count"] == len(camera_stamps(services, wire_kind)) - 1
        assert stats["scheduled_count"] == stats["published_count"] + stats["dropped_count"]
    stored = load_feeder_status(
        session_root=identity.session_root,
        product=payload["product"],
        product_session_id=identity.product_session_id,
        process="mujoco_feeder",
        launch_id="a" * 64,
    )
    assert stored["streams"] == payload["streams"]


@pytest.mark.parametrize("host_camera_enabled", [False, True])
@pytest.mark.parametrize("body_frame", ["base_link", "camera1_link"])
def test_measurement_only_uses_owned_camera_endpoint_without_legacy_images(
    monkeypatch, tmp_path, host_camera_enabled, body_frame
):
    mujoco = pytest.importorskip("mujoco")
    payload = _with_measurements().as_dict()
    payload["host"]["config"]["enable_camera"] = host_camera_enabled
    streams = payload["launch"]["simulation"]["sensor_plan"]["streams"]
    streams["rgb"] = []
    streams["depth"] = []
    payload["launch"]["simulation"]["physics_plan"]["robots"][0]["frames"].append(
        {"name": "camera1_link", "role": "sensor_mount", "parent_frame": "base_link"}
    )
    streams["thermal"][0]["measurement"]["body_frame_id"] = f"robot_01/{body_frame}"
    streams["thermal"][0]["frame_id"] = f"robot_01/{body_frame}"
    services = FakeServices(stop_after_readiness=False, stop_after_waits=25)
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
        <body name="base_link"><body name="camera1_link"/></body>
        <geom name="hot" type="box" pos="0 0 3" size="2 2 .1"/>
        </worldbody></mujoco>""")
    data = mujoco.MjData(model)

    @contextmanager
    def state():
        data.time = services.engine.sim_time
        yield model, data

    control = SimpleNamespace(take_request=lambda: None, close=lambda: services.events.append("control_close"))
    services.connect_measurement_control = lambda **binding: control
    services.engine.measurement_state = state
    rc, identity, _ = _run(monkeypatch, tmp_path, services, plan=RunPlan.from_dict(payload))

    assert rc == 0
    assert identity.session_root / "camera.ready.json" in services.connected_paths
    assert any(HEADER.unpack_from(wire)[1] == 9 for wire in services.sensors["camera_publisher"].records)
    assert camera_stamps(services, 2) == camera_stamps(services, 3) == []
    assert services.camera_writers == {}
    assert services.engine.camera_calls == 0
    assert ("retain_world_visuals", True) in services.events
    assert ("sensor_close", "camera_publisher") in services.events
    assert services.events.count("control_close") == 1
    assert services.events.count("engine_close") == 1


def test_both_cameras_unselected_do_not_get_reenabled_by_product_camera_owner(monkeypatch, tmp_path):
    services = FakeServices()
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=selected_plan(rgb_hz=None, depth_hz=None))
    assert rc == 0
    assert services.build_kwargs["camera_configs"] is None
    assert services.engine.camera_calls == 0
    assert services.camera_writers == {}
    assert "connect_sensor" in services.events  # Only the independently selected LiDAR/IMU.
    assert not any(path.name == "camera.ready.json" for path in services.connected_paths)
    assert not any(name.startswith("camera_") for name in services.status_payloads[-1]["streams"])


def test_measurement_unknown_body_is_still_rejected_before_engine(monkeypatch, tmp_path, capsys):
    payload = _with_measurements().as_dict()
    payload["launch"]["simulation"]["sensor_plan"]["streams"]["thermal"][0]["measurement"]["body_frame_id"] = (
        "robot_01/unknown_body"
    )
    services = FakeServices()
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=RunPlan.from_dict(payload))
    assert rc == 1
    assert services.build_kwargs is None
    assert services.connected_paths == []
    assert "measurement body is not a declared robot body" in capsys.readouterr().err


@pytest.mark.parametrize("endpoint", ["absent", "wrong_name"])
def test_measurement_only_requires_existing_native_camera_endpoint_before_engine(monkeypatch, tmp_path, endpoint):
    payload = _with_measurements().as_dict()
    payload["host"]["config"]["enable_camera"] = False
    streams = payload["launch"]["simulation"]["sensor_plan"]["streams"]
    streams["rgb"] = streams["depth"] = []
    selected = payload["launch"]["process_catalog"]["selected"]
    camera = next(process for process in selected if process["name"] == "camera_publisher")
    if endpoint == "absent":
        selected.remove(camera)
    else:
        camera["name"] = "other_camera"
        for process in payload["launch"]["process_catalog"]["available"]:
            if process["name"] == "camera_publisher":
                process["name"] = "other_camera"
    services = FakeServices()
    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=RunPlan.from_dict(payload))
    assert rc == 1
    assert services.build_kwargs is None
    assert services.connected_paths == []
