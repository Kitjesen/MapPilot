"""Python typed arrays -> LTU1 -> actual native DDS, with owned cleanup.

Inputs are controlled transport fixtures, not a factory Product or robot run.
"""

import copy
import json
import os
import subprocess
import tempfile
import time
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest
from sim.runtime.process_owner import ProcessTreeOwner
from sim.runtime.sensors.acoustics import AcousticFrame
from sim.runtime.sensors.camera_geometry import PinholeCalibration
from sim.runtime.sensors.semantic_camera import SemanticCameraSample
from sim.scripts.mujoco.measurement_wire import encode_measurement

from tests.sim.test_measurement_payload import _acoustic, _semantic, _thermal


def _stamped(original, sequence):
    stamp = replace(original.stamp, sequence=sequence, sim_time_ns=1_000_000_000 + sequence * 10_000_000)
    if isinstance(original, SemanticCameraSample):
        return replace(original, stamp=stamp, source_stamp=stamp)
    metadata = copy.deepcopy(original.metadata)
    if isinstance(original, AcousticFrame):
        metadata["first_sample_index"] = stamp.sim_time_ns * original.sample_rate_hz // 1_000_000_000
        for source in metadata["sources"]:
            source["source_sim_time_ns"] = stamp.sim_time_ns
    else:
        metadata["source_stamp"] = asdict(stamp)
    return replace(original, stamp=stamp, metadata=metadata)


def test_python_measurements_reach_all_three_native_dds_types(tmp_path):
    root = Path(__file__).resolve().parents[2]
    binaries = root / "build/windows-native-dds-adapter/Release"
    publisher_exe = binaries / "lingtu_mujoco_sensor_publisher.exe"
    observer_exe = binaries / "test_measurement_publisher_dds_process.exe"
    if os.name != "nt" or not publisher_exe.exists() or not observer_exe.exists():
        pytest.skip("built Windows native measurement binaries required")
    base = Path(os.environ.get("LINGTU_MEASUREMENT_EVIDENCE_DIR", str(tmp_path)))
    base.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="measurement-wire-", dir=base))
    output, ready = run / "received.json", run / "reader.ready.json"
    frames = (_thermal(), _acoustic(), _semantic(PinholeCalibration(2, 2, 80, 80, 1, 1)))
    product = "measurement-interop-product"
    publisher_owner, reader_owner = ProcessTreeOwner(), ProcessTreeOwner()
    publisher = reader = None
    sent = {}
    result = {"passed": False, "scope": "controlled_arrays_python_ltu1_native_dds", "run_dir": str(run)}
    with (run / "publisher.stderr.log").open("w") as pub_log, (run / "reader.stderr.log").open("w") as reader_log:
        try:
            reader = subprocess.Popen(
                [
                    str(observer_exe),
                    "--observe-json",
                    str(output),
                    "--domain-id",
                    "6",
                    "--count",
                    "3",
                    "--require-all-modalities",
                    "1",
                    "--ready-file",
                    str(ready),
                ],
                stdout=subprocess.DEVNULL,
                stderr=reader_log,
                **reader_owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW),
            )
            reader_owner.attach(reader)
            deadline = time.monotonic() + 8
            while not ready.is_file():
                assert reader.poll() is None and time.monotonic() < deadline, "reader startup failed/timed out"
                time.sleep(0.02)
            publisher = subprocess.Popen(
                [
                    str(publisher_exe),
                    "--stdin-records",
                    "--dds",
                    "--stream",
                    "camera",
                    "--domain-id",
                    "6",
                    "--measurement-product-session-id",
                    product,
                    "--measurement-session-id",
                    "recording-test",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=pub_log,
                bufsize=0,
                **publisher_owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW),
            )
            publisher_owner.attach(publisher)
            for sequence in range(100):
                if reader.poll() is not None:
                    break
                assert publisher.poll() is None, "publisher rejected a measurement"
                for original in frames:
                    sample = _stamped(original, sequence)
                    wire = encode_measurement(sample, product_session_id=product).wire
                    assert publisher.stdin.write(wire) == len(wire)
                    sent[(sample.stamp.sensor_id, sequence)] = sample
                publisher.stdin.flush()
                time.sleep(0.03)
            reader.wait(timeout=8)
            publisher.stdin.close()
            publisher.wait(timeout=5)
            assert publisher.returncode == reader.returncode == 0
            records = json.loads(output.read_text(encoding="utf-8"))["measurements"]
            assert {record["modality"] for record in records} == {1, 2, 3}
            observed_keys = set()
            for record in records:
                key = (record["sensor_id"], record["sequence"])
                assert key not in observed_keys
                observed_keys.add(key)
                sample = sent[key]
                assert record["product_session_id"] == product
                for name in (
                    "session_id",
                    "instance_id",
                    "frame_id",
                    "model_generation",
                    "reset_generation",
                    "sim_time_ns",
                ):
                    assert record[name] == getattr(sample.stamp, name)
                metadata = json.loads(record["metadata_json"])
                assert metadata["stamp"] == asdict(sample.stamp)
                shape = (record["rows"], record["columns"])
                np.testing.assert_array_equal(np.asarray(record["valid_mask"]).reshape(shape), sample.valid_mask)
                if record["modality"] == 3:
                    np.testing.assert_array_equal(np.asarray(record["class_ids"]).reshape(shape), sample.class_ids)
                    np.testing.assert_array_equal(
                        np.asarray(record["instance_ids"]).reshape(shape), sample.instance_ids
                    )
                else:
                    expected = sample.temperature_k if record["modality"] == 1 else sample.pressure_pa
                    np.testing.assert_array_equal(
                        np.asarray(record["primary"], dtype=np.float32).reshape(shape), expected
                    )
                    if record["modality"] == 2:
                        assert record["sample_rate_hz"] == sample.sample_rate_hz
                        np.testing.assert_array_equal(
                            np.asarray(record["voltage_v"], dtype=np.float32).reshape(shape), sample.voltage_v
                        )
            result.update(
                passed=True,
                measurements_received=len(records),
                modalities=[1, 2, 3],
                first_packet_delivery_guaranteed=False,
            )
        finally:
            for process, owner in ((publisher, publisher_owner), (reader, reader_owner)):
                try:
                    if process is not None:
                        if process.stdin is not None and not process.stdin.closed:
                            process.stdin.close()
                        if process.poll() is None:
                            owner.terminate(process, timeout_s=2)
                        else:
                            owner.close_after_exit()
                finally:
                    owner.close()
            result.update(
                publisher_exit_code=publisher.returncode if publisher else None,
                observer_exit_code=reader.returncode if reader else None,
            )
            (run / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"MEASUREMENT_NATIVE_EVIDENCE={run}")


def test_native_dds_bridge_reaches_typed_python_host_messages(tmp_path):
    from drivers.sim.measurement.native_bridge import NativeMeasurementBridge
    from runtime.msgs.measurement import AcousticMeasurement, SemanticMeasurement, ThermalMeasurement

    root = Path(__file__).resolve().parents[2]
    binary_dir = root / "build/windows-native-dds-adapter/Release"
    executable = binary_dir / "lingtu_mujoco_sensor_publisher.exe"
    if os.name != "nt" or not executable.exists() or not (binary_dir / "lingtu_measurement_bridge.dll").exists():
        pytest.skip("native publisher and measurement bridge required")
    base = Path(os.environ.get("LINGTU_MEASUREMENT_EVIDENCE_DIR", str(tmp_path)))
    base.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="measurement-host-", dir=base))
    frames = (_thermal(), _acoustic(), _semantic(PinholeCalibration(2, 2, 80, 80, 1, 1)))
    owner = ProcessTreeOwner()
    process = None
    sent, received = {}, {}
    passed = False
    with (run / "publisher.stderr.log").open("w") as error_log:
        try:
            with NativeMeasurementBridge(
                domain_id=6, product_session_id="host-interop", session_id="recording-test"
            ) as reader:
                process = subprocess.Popen(
                    [
                        str(executable),
                        "--stdin-records",
                        "--dds",
                        "--stream",
                        "camera",
                        "--domain-id",
                        "6",
                        "--measurement-product-session-id",
                        "host-interop",
                        "--measurement-session-id",
                        "recording-test",
                    ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=error_log,
                    bufsize=0,
                    **owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW),
                )
                owner.attach(process)
                for sequence in range(100):
                    assert process.poll() is None, "publisher exited before Host received measurements"
                    for original in frames:
                        sample = _stamped(original, sequence)
                        wire = encode_measurement(sample, product_session_id="host-interop").wire
                        assert process.stdin.write(wire) == len(wire)
                        sent[(sample.stamp.sensor_id, sequence)] = sample
                    process.stdin.flush()
                    deadline = time.monotonic() + 0.03
                    while time.monotonic() < deadline:
                        frame = reader.take(received_at_s=time.time())
                        if frame is None:
                            time.sleep(0.002)
                            continue
                        original = sent[(frame.identity.sensor_id, frame.identity.sequence)]
                        assert frame.identity.product_session_id == "host-interop"
                        assert frame.identity.sim_time_ns == original.stamp.sim_time_ns
                        assert frame.captured_at_s is None, "receipt must not become source epoch time"
                        np.testing.assert_array_equal(frame.valid_mask, original.valid_mask)
                        if isinstance(frame, ThermalMeasurement):
                            np.testing.assert_array_equal(frame.temperature_k, original.temperature_k)
                        elif isinstance(frame, AcousticMeasurement):
                            np.testing.assert_array_equal(frame.pressure_pa, original.pressure_pa)
                            np.testing.assert_array_equal(frame.voltage_v, original.voltage_v)
                        elif isinstance(frame, SemanticMeasurement):
                            np.testing.assert_array_equal(frame.class_ids, original.class_ids)
                            np.testing.assert_array_equal(frame.instance_ids, original.instance_ids)
                        else:
                            pytest.fail("unexpected Host message type")
                        received[type(frame).__name__] = asdict(frame.identity)
                    if len(received) == 3:
                        break
                assert set(received) == {"ThermalMeasurement", "AcousticMeasurement", "SemanticMeasurement"}
                process.stdin.close()
                assert process.wait(timeout=5) == 0
                passed = True
        finally:
            try:
                if process is not None:
                    if process.stdin is not None and not process.stdin.closed:
                        process.stdin.close()
                    if process.poll() is None:
                        owner.terminate(process, timeout_s=2)
                    else:
                        owner.close_after_exit()
            finally:
                owner.close()
            (run / "result.json").write_text(
                json.dumps(
                    {
                        "passed": passed,
                        "received": received,
                        "scope": "controlled_arrays_python_to_native_dds_to_python_host",
                        "publisher_exit_code": process.returncode if process else None,
                        "bridge_closed": True,
                        "run_dir": str(run),
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
    print(f"MEASUREMENT_HOST_EVIDENCE={run}")
