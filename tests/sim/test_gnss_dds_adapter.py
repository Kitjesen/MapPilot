from __future__ import annotations

import math
import struct
from dataclasses import replace
from fractions import Fraction

import pytest
from sim.runtime.sensors import GnssConfig, GnssExtractor, SensorRoute, SensorRuntime, SensorStreamPlan
from sim.runtime.sensors.gnss_dds import encode_gnss_sample

from runtime.msgs.gnss import GnssFixType

pytestmark = [pytest.mark.sim]


def _sample(*, available=True, reset=0, sequence=0):
    config = GnssConfig(45, 120, 100, "robot/base_link", fix_available=available)
    stream = SensorStreamPlan(
        "gnss",
        "robot",
        "robot.gnss",
        "robot/gnss",
        "lingtu.dds.GnssFix",
        Fraction(10),
        SensorRoute("physics", "mujoco_gnss", "typed_dds"),
        gnss=config,
    )
    scheduled = (
        SensorRuntime("gnss-native-test", (stream,))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=reset,
        )
        .samples[0]
    )
    scheduled = replace(scheduled, sequence=sequence, deadline_ns=sequence * 100_000_000)
    snapshot = {
        "session_id": scheduled.session_id,
        "model_generation": 1,
        "reset_generation": reset,
        "sim_time_ns": scheduled.deadline_ns,
        "bodies": [{"stable_id": "robot/base_link", "position_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]}],
    }
    return GnssExtractor(config)(scheduled, snapshot)


def test_gnss_native_record_preserves_identity_si_values_and_no_fix():
    for available in (True, False):
        sample = _sample(available=available)
        encoded = encode_gnss_sample(sample)
        assert len(encoded) == 176 + 124
        header = struct.unpack("<4sHHII64sQQQQ64s", encoded[:176])
        assert header[:5] == (b"LTGN", 1, 7, 176, 124)
        assert header[5].rstrip(b"\0") == b"gnss-native-test"
        assert header[6:10] == (1, 0, 0, 0)
        assert header[10].rstrip(b"\0") == b"robot/gnss"
        payload = struct.unpack("<iII14d", encoded[176:])
        assert payload[:3] == (int(GnssFixType.SIMULATION if available else GnssFixType.NO_FIX), 0, 0)
        if available:
            assert payload[3:6] == pytest.approx((45, 120, 100))
        else:
            assert all(math.isnan(value) for value in payload[3:15])
        assert all(math.isnan(value) for value in payload[-2:])


def test_gnss_factory_binds_a_native_writer_and_reports_missing_executable(tmp_path):
    from types import SimpleNamespace

    from sim.runtime.sensors.factory import GnssEndpointFactory

    sample = _sample()
    stream = SensorStreamPlan(
        "gnss",
        "robot",
        "robot.gnss",
        "robot/gnss",
        "lingtu.dds.GnssFix",
        Fraction(10),
        SensorRoute("physics", "mujoco_gnss", "typed_dds"),
        gnss=sample.configuration,
    )
    allocation = SimpleNamespace(
        session_id="gnss-native-test",
        run_dir=tmp_path,
        log_dir=tmp_path,
        dds_domain=133,
        child_environment=lambda: {},
    )
    endpoint = GnssEndpointFactory(executable=tmp_path / "missing.exe")(stream, allocation)
    assert endpoint is not None
    with pytest.raises(RuntimeError, match="GNSS publisher does not exist"):
        endpoint.sink.start()
    endpoint.sink.close()


@pytest.mark.parametrize(
    "field,value",
    [
        ("ts", 1.0),
        ("seq", 9),
        ("frame_id", "other/antenna"),
        ("num_sat", 12),
        ("fix_type", 4),
    ],
)
def test_gnss_record_rejects_a_payload_that_contradicts_its_source_stamp(field, value):
    sample = _sample()
    sample = replace(sample, fix=replace(sample.fix, **{field: value}))
    with pytest.raises(ValueError):
        encode_gnss_sample(sample)


def test_native_thunder_snapshot_to_gnss_dds_and_reset(tmp_path):
    """Use the real robot MJCF, C++ physics host, native writer and DDS readers."""
    import json
    import os
    import subprocess
    import tempfile
    import time
    from pathlib import Path

    from sim.catalog import CatalogResolver
    from sim.runtime.coordinator.coordinator import load_physics_plan
    from sim.runtime.coordinator.mujoco_process import MujocoProcess
    from sim.runtime.coordinator.run_allocation import RunAllocation
    from sim.runtime.process_owner import ProcessTreeOwner
    from sim.runtime.sensors import GnssEndpointFactory, SessionSensorRuntime

    root = Path(__file__).resolve().parents[2]
    native = root / "build/windows-native-dds-adapter/Release"
    publisher = native / "lingtu_gnss_publisher.exe"
    observer_exe = native / "test_gnss_publisher_dds_process.exe"
    physics_exe = root / "build/mujoco-motion-contact-win/Release/lingtu_mujoco_headless.exe"
    if not all(path.is_file() for path in (publisher, observer_exe, physics_exe)):
        pytest.skip("GNSS native integration requires the built DDS and MuJoCo test executables")
    evidence_root = Path(os.environ.get("LINGTU_GNSS_EVIDENCE_DIR", str(tmp_path)))
    evidence_root.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="gnss-native-", dir=evidence_root))
    resolved = CatalogResolver.from_repository(root).resolve(root / "sim/sessions/examples/thunderv4_gnss/session.yaml")
    resolved.write_bundle(run / "bundle")
    physics_plan = load_physics_plan(run / "bundle", root)
    sensor_plan = SensorRuntime.from_path(run / "bundle/sensor.plan.json")
    (stream,) = sensor_plan.streams
    allocation = RunAllocation(
        run_id="gnss-native-test",
        run_dir=run,
        artifact_root=run,
        log_dir=run,
        ports={},
        shm={},
        session_id=resolved.session_id,
        boot_id="gnss-native-test",
        dds_domain=int(os.environ.get("LINGTU_GNSS_NATIVE_TEST_DOMAIN", "133")),
    )
    physics = MujocoProcess(physics_exe)
    sensors = SessionSensorRuntime(
        plan=sensor_plan,
        allocation=allocation,
        endpoint_factory=GnssEndpointFactory(executable=publisher),
    )
    owner = ProcessTreeOwner()
    observer = None
    snapshots = []
    output, ready, matched = run / "observed.json", run / "reader.ready", run / "reader.matched"

    def wait_for(path):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if path.is_file():
                return
            if observer.poll() is not None:
                raise AssertionError(observer.communicate()[1])
            time.sleep(0.01)
        raise AssertionError(f"Timed out waiting for {path.name}")

    try:
        physics.prepare(physics_plan, allocation)
        physics.start()
        observer = subprocess.Popen(
            [
                str(observer_exe),
                "--observe-json",
                str(output),
                "--domain-id",
                str(allocation.dds_domain),
                "--count",
                "3",
                "--ready-file",
                str(ready),
                "--matched-file",
                str(matched),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            **owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0),
        )
        owner.attach(observer)
        wait_for(ready)
        sensors.prepare()
        wait_for(matched)
        sensors.set_gnss_fix_available(stream.sensor_id, True)
        snapshots.append(physics.snapshot())
        sensors.process_snapshot(snapshots[-1])
        # Sample at the configured 10 Hz cadence, including a scenario outage.
        time.sleep(0.1)
        sensors.set_gnss_fix_available(stream.sensor_id, False)
        snapshots.append(physics.advance(round(0.1 / physics_plan.global_policy.timestep_s)))
        sensors.process_snapshot(snapshots[-1])
        time.sleep(0.1)
        physics.pause()
        physics.reset()
        snapshots.append(physics.snapshot())
        sensors.set_gnss_fix_available(stream.sensor_id, True)
        sensors.process_snapshot(snapshots[-1])
        stdout, stderr = observer.communicate(timeout=12)
        assert observer.returncode == 0, (stdout, stderr)
        observed = json.loads(output.read_text(encoding="utf-8"))
        fixes, statuses = observed["fixes"], observed["statuses"]
        assert [sample["time_ns"] for sample in fixes] == [0, 100_000_000, 0]
        assert [sample["fix_type"] for sample in fixes] == [8, 0, 8]
        assert all(sample["frame_id"] == stream.frame_id for sample in fixes)
        assert fixes[1]["lat"] is fixes[1]["lon"] is fixes[1]["alt"] is None
        assert fixes[1]["cov"] == [None] * 9
        assert [sample["fix_type"] for sample in statuses] == [8, 0, 8]
        assert all(sample["link_ok"] and not sample["rtk"] for sample in statuses)
        assert statuses[1]["error"] == "simulation_no_fix"
        for fix, snapshot in zip((fixes[0], fixes[2]), (snapshots[0], snapshots[2])):
            body = next(body for body in snapshot["bodies"] if body["stable_id"] == stream.gnss.body_frame_id)
            # Exact pose-to-fix equality is covered by the geometric tests; here
            # prove the actual robot's antenna height reached the DDS reader.
            assert fix["alt"] == pytest.approx(body["position_m"][2] + 0.14, abs=1e-4)
        (run / "physics-and-sensors.json").write_text(
            json.dumps(
                {
                    "snapshots": snapshots,
                    "sensor_observations": sensors.evidence_observations(),
                },
                indent=2,
                allow_nan=False,
            ),
            encoding="utf-8",
        )
    finally:
        try:
            sensors.close()
        finally:
            try:
                physics.stop()
            finally:
                if observer is not None:
                    if observer.poll() is None:
                        owner.terminate(observer, timeout_s=2.0)
                    else:
                        owner.close_after_exit()
                    for pipe in (observer.stdout, observer.stderr):
                        if pipe is not None:
                            pipe.close()
                owner.close()
    assert physics.last_shutdown.exit_code == 0
    assert physics.last_shutdown.direct_child_running_after_close is False
    assert not list(run.glob("gnss-*.ready.json"))
    (run / "result.json").write_text(
        json.dumps(
            {
                "passed": True,
                "scope": "headless_thunder_gnss_dds",
                "fix_count": 3,
                "status_count": 3,
                "physics_exit_code": physics.last_shutdown.exit_code,
                "reader_exit_code": observer.returncode,
                "run_dir": str(run),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"GNSS_EVIDENCE_DIR={run}")
