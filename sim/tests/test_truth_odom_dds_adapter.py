# ruff: noqa: S101
from __future__ import annotations

import io
import json
import os
import struct
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest

from sim.runtime.coordinator.run_allocation import RunAllocation
from sim.runtime.sensors import SensorSampleStamp, TruthOdometrySample
from sim.runtime.sensors.dds_adapter import (
    TruthOdometryDdsAdapter,
    encode_truth_odometry_sample,
)
from sim.runtime.sensors.factory import TruthOdometryEndpointFactory
from sim.runtime.sensors.runtime import SensorRuntime

pytestmark = [pytest.mark.sim]

_HEADER = struct.Struct("<4sHHII64sQQQQ")
_PAYLOAD = struct.Struct("<85d")


def _sample(**stamp_overrides: object) -> TruthOdometrySample:
    stamp_values = {
        "session_id": "truth-session",
        "instance_id": "robot-1",
        "sensor_id": "robot-1.truth_odometry",
        "frame_id": "base_link",
        "model_generation": 7,
        "reset_generation": 3,
        "sequence": 0,
        "sim_time_ns": 123_456_789,
    }
    stamp_values.update(stamp_overrides)
    return TruthOdometrySample(
        stamp=SensorSampleStamp(**stamp_values),
        position_m=(1.0, 2.0, 3.0),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        linear_velocity_mps=(4.0, 5.0, 6.0),
        angular_velocity_rps=(0.1, 0.2, 0.3),
        pose_covariance=(0.01,) * 36,
        twist_covariance=(0.02,) * 36,
    )


def test_truth_odometry_sample_encodes_canonical_ltod_v1_record() -> None:
    encoded = encode_truth_odometry_sample(_sample())

    assert len(encoded) == _HEADER.size + _PAYLOAD.size == 792
    (
        magic,
        version,
        flags,
        header_bytes,
        payload_bytes,
        session_id,
        model_generation,
        reset_generation,
        sequence,
        timestamp_ns,
    ) = _HEADER.unpack_from(encoded)
    assert magic == b"LTOD"
    assert version == 1
    assert flags == 0b1111
    assert header_bytes == _HEADER.size
    assert payload_bytes == _PAYLOAD.size
    assert session_id.rstrip(b"\0") == b"truth-session"
    assert (model_generation, reset_generation, sequence, timestamp_ns) == (
        7,
        3,
        0,
        123_456_789,
    )

    values = _PAYLOAD.unpack_from(encoded, _HEADER.size)
    assert values[:13] == pytest.approx(
        (
            1.0,
            2.0,
            3.0,
            1.0,
            0.0,
            0.0,
            0.0,
            4.0,
            5.0,
            6.0,
            0.1,
            0.2,
            0.3,
        )
    )
    assert values[13:49] == pytest.approx((0.01,) * 36)
    assert values[49:] == pytest.approx((0.02,) * 36)


def test_truth_odometry_encoder_preserves_optional_field_availability() -> None:
    sample = replace(
        _sample(),
        linear_velocity_mps=None,
        angular_velocity_rps=None,
        pose_covariance=None,
        twist_covariance=None,
    )

    encoded = encode_truth_odometry_sample(sample)
    flags = _HEADER.unpack_from(encoded)[2]
    values = _PAYLOAD.unpack_from(encoded, _HEADER.size)

    assert flags == 0
    assert values[7:] == pytest.approx((0.0,) * 78)


@pytest.mark.parametrize(
    ("stamp_overrides", "message"),
    [
        ({"session_id": "bad session"}, "session_id"),
        ({"session_id": "x" * 64}, "session_id"),
        ({"session_id": "会话"}, "session_id"),
        ({"sim_time_ns": (1 << 31) * 1_000_000_000}, "IDL Time"),
    ],
)
def test_truth_odometry_encoder_rejects_unrepresentable_identity(
    stamp_overrides: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        encode_truth_odometry_sample(_sample(**stamp_overrides))


def _allocation(tmp_path: Path, *, domain: int = 77) -> RunAllocation:
    return RunAllocation(
        run_id="truth-odom-test",
        run_dir=tmp_path,
        artifact_root=tmp_path.resolve(),
        log_dir=tmp_path / "logs",
        ports={},
        shm={},
        session_id="truth-session",
        boot_id="test-boot",
        dds_domain=domain,
    )


def test_truth_odometry_dds_adapter_uses_run_allocation_and_waits_for_readiness(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    executable = tmp_path / "lingtu_truth_odom_publisher.exe"
    executable.write_bytes(b"placeholder")
    allocation = _allocation(tmp_path)
    launched: list[tuple[list[str], dict[str, object]]] = []

    class Input(io.BytesIO):
        def flush(self) -> None:
            return None

    class Process:
        pid = 4242

        def __init__(self) -> None:
            self.stdin = Input()
            self.returncode: int | None = None

        def poll(self) -> int | None:
            return self.returncode

        def wait(self, timeout: float) -> int:
            assert timeout == pytest.approx(5.0)
            self.returncode = 0
            return 0

    process = Process()
    readiness_reads = 0
    original_read_text = Path.read_text

    def read_text_with_one_transient_access_conflict(
        path: Path,
        *args: object,
        **kwargs: object,
    ) -> str:
        nonlocal readiness_reads
        if path.name == "truth-odom-publisher.ready.json":
            readiness_reads += 1
            if readiness_reads == 1:
                raise PermissionError(5, "transient readiness access conflict")
        return original_read_text(path, *args, **kwargs)

    def popen(command: list[str], **kwargs: object) -> Process:
        launched.append((command, kwargs))
        ready_file = Path(command[command.index("--ready-file") + 1])
        ready_file.write_text(
            json.dumps(
                {
                    "schema": "lingtu.sim.truth-odom-publisher.ready.v1",
                    "ready": True,
                    "dds_domain": allocation.dds_domain,
                    "session_id": allocation.session_id,
                    "topic": "rt/sim/truth/odom",
                }
            ),
            encoding="utf-8",
        )
        return process

    class ProcessOwner:
        @staticmethod
        def popen_options(*, creationflags: int = 0) -> dict[str, int]:
            return {"creationflags": creationflags}

        @staticmethod
        def attach(_process: Process) -> None:
            return None

        @staticmethod
        def close_after_exit() -> None:
            return None

    monkeypatch.setattr("sim.runtime.sensors.dds_adapter.subprocess.Popen", popen)
    monkeypatch.setattr(Path, "read_text", read_text_with_one_transient_access_conflict)
    monkeypatch.setattr(
        "sim.runtime.sensors.dds_adapter.ProcessTreeOwner", ProcessOwner
    )

    adapter = TruthOdometryDdsAdapter(
        executable,
        allocation=allocation,
        parent_frame="odom",
        child_frame="base_link",
    )
    readiness = adapter.start()
    adapter.publish(_sample())
    expected_record = encode_truth_odometry_sample(_sample())
    assert process.stdin.getvalue() == expected_record

    command, kwargs = launched[0]
    assert command == [
        str(executable.resolve()),
        "--dds-domain",
        "77",
        "--session-id",
        allocation.session_id,
        "--parent-frame",
        "odom",
        "--child-frame",
        "base_link",
        "--ready-file",
        str(tmp_path / "truth-odom-publisher.ready.json"),
    ]
    assert kwargs["stdin"] == -1
    assert kwargs["env"]["LINGTU_HOST_BOOT_ID"] == allocation.boot_id
    stderr = kwargs["stderr"]
    assert Path(stderr.name) == tmp_path / "logs" / "truth-odom-publisher.stderr.log"
    assert readiness["topic"] == "rt/sim/truth/odom"
    assert readiness_reads == 2

    adapter.close()
    assert process.stdin.closed
    assert stderr.closed


def test_truth_odometry_adapter_does_not_import_legacy_simulation_layers() -> None:
    source = Path("sim/runtime/sensors/dds_adapter.py").read_text(encoding="utf-8")

    assert "sim.engine" not in source
    assert "sim.scripts" not in source
    assert "drivers.sim" not in source
    assert "from .samples import TruthOdometrySample" in source


def test_truth_odometry_endpoint_factory_matches_only_the_canonical_stream(
    tmp_path: Path,
) -> None:
    plan = SensorRuntime.from_path(
        Path("build/session-bundles/thunderv4-unreal/sensor.plan.json")
    )
    allocation = RunAllocation(
        run_id="truth-endpoint",
        run_dir=tmp_path,
        artifact_root=tmp_path.resolve(),
        log_dir=tmp_path / "logs",
        ports={},
        shm={},
        session_id=plan.session_id,
        boot_id="boot-test",
        dds_domain=17,
    )
    factory = TruthOdometryEndpointFactory(
        executable=tmp_path / "lingtu_truth_odom_publisher.exe",
        parent_frame="world",
    )
    truth = next(stream for stream in plan.streams if stream.stream_kind == "truth_odom")
    imu = next(stream for stream in plan.streams if stream.stream_kind == "imu")

    endpoint = factory(truth, allocation)

    assert endpoint is not None
    assert endpoint.source_id == "truth-odom-dds"
    assert isinstance(endpoint.sink, TruthOdometryDdsAdapter)
    assert factory(imu, allocation) is None


def test_truth_odometry_cyclonedds_process_loopback() -> None:
    configured = os.environ.get("LINGTU_TRUTH_ODOM_PROCESS_TEST_BIN")
    candidates = [
        Path(configured) if configured else None,
        Path("build/windows-native-dds-adapter/Release") / "test_truth_odom_publisher_dds_process.exe",
        Path("build/mujoco_native_dds/test_truth_odom_publisher_dds_process"),
    ]
    test_binary = next(
        (candidate.resolve() for candidate in candidates if candidate and candidate.is_file()),
        None,
    )
    if test_binary is None:
        pytest.skip("CycloneDDS process-loopback binary is unavailable; build sim/adapters/dds")
    suffix = ".exe" if os.name == "nt" else ""
    publisher = test_binary.with_name(f"lingtu_truth_odom_publisher{suffix}")
    if not publisher.is_file():
        pytest.skip("truth odometry publisher binary is unavailable beside loopback test")

    completed = subprocess.run(  # noqa: S603 - locally built fixed test executable
        [str(test_binary), str(publisher)],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "topic=rt/sim/truth/odom" in completed.stdout
