# ruff: noqa: S101
from __future__ import annotations

import io
import json
import struct
from fractions import Fraction
from pathlib import Path

import pytest

from sim.runtime.coordinator.run_allocation import RunAllocation
from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.dds_adapter import ImuDdsAdapter, encode_imu_sample
from sim.runtime.sensors.factory import ImuEndpointFactory, SensorEndpointRouter
from sim.runtime.sensors.samples import ImuSample, SensorSampleStamp

pytestmark = [pytest.mark.sim]


def _sample(**overrides: object) -> ImuSample:
    stamp = {
        "session_id": "imu-session",
        "instance_id": "thunder_01",
        "sensor_id": "thunder_01.imu",
        "frame_id": "thunder_01/imu",
        "model_generation": 2,
        "reset_generation": 0,
        "sequence": 0,
        "sim_time_ns": 2_000_000,
    }
    stamp.update(overrides)
    return ImuSample(
        stamp=SensorSampleStamp(**stamp),
        orientation_wxyz=(1.0, 0.1, 0.2, 0.3),
        angular_velocity_rps=(0.01, 0.02, 0.03),
        linear_acceleration_mps2=(1.0, 2.0, 9.81),
    )


def test_imu_encoder_carries_version_generations_timestamp_frame_and_si_values() -> None:
    encoded = encode_imu_sample(_sample())
    assert encoded[:4] == b"LTIM"
    assert encoded[4:6] == (1).to_bytes(2, "little")
    assert b"imu-session" in encoded[:176]
    assert b"thunder_01/imu" in encoded[:176]
    assert struct.unpack_from("<10d", encoded, 176) == pytest.approx(
        (1.0, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 1.0, 2.0, 9.81)
    )


def test_imu_encoder_rejects_incomplete_or_oversized_frame() -> None:
    with pytest.raises(ValueError, match="require orientation"):
        encode_imu_sample(
            ImuSample(
                stamp=_sample().stamp,
                orientation_wxyz=None,
                angular_velocity_rps=(0.0, 0.0, 0.0),
                linear_acceleration_mps2=(0.0, 0.0, 9.81),
            )
        )
    with pytest.raises(ValueError, match="64-byte"):
        encode_imu_sample(_sample(frame_id="x" * 64))


def test_imu_adapter_uses_simulated_topic_and_run_allocation(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    executable = tmp_path / "lingtu_imu_publisher.exe"
    executable.write_bytes(b"placeholder")
    allocation = RunAllocation(
        run_id="imu-test",
        run_dir=tmp_path,
        artifact_root=tmp_path.resolve(),
        log_dir=tmp_path / "logs",
        ports={},
        shm={},
        session_id="imu-session",
        boot_id="boot",
        dds_domain=42,
    )
    launched: list[tuple[list[str], dict[str, object]]] = []

    class Process:
        pid = 1
        stdin = io.BytesIO()
        returncode = None
        def poll(self): return self.returncode
        def wait(self, timeout):
            self.returncode = 0
            return 0

    process = Process()

    def popen(command, **kwargs):
        launched.append((command, kwargs))
        Path(command[command.index("--ready-file") + 1]).write_text(
            json.dumps({
                "schema": "lingtu.sim.imu-publisher.ready.v1", "ready": True,
                "dds_domain": 42, "session_id": "imu-session", "topic": "rt/sim/imu",
            }), encoding="utf-8"
        )
        return process

    class ProcessOwner:
        @staticmethod
        def popen_options(*, creationflags: int = 0) -> dict[str, int]:
            return {"creationflags": creationflags}

        @staticmethod
        def attach(_process) -> None:
            return None

        @staticmethod
        def close_after_exit() -> None:
            return None

    monkeypatch.setattr("sim.runtime.sensors.dds_adapter.subprocess.Popen", popen)
    monkeypatch.setattr(
        "sim.runtime.sensors.dds_adapter.ProcessTreeOwner", ProcessOwner
    )
    adapter = ImuDdsAdapter(executable, allocation=allocation)
    assert adapter.start()["topic"] == "rt/sim/imu"
    adapter.publish(_sample())
    command, kwargs = launched[0]
    assert command[command.index("--dds-domain") + 1] == "42"
    assert command[command.index("--session-id") + 1] == "imu-session"
    assert kwargs["env"]["LINGTU_HOST_BOOT_ID"] == allocation.boot_id
    stderr = kwargs["stderr"]
    assert Path(stderr.name) == tmp_path / "logs" / "imu-publisher.stderr.log"
    adapter.close()
    assert stderr.closed


def test_imu_endpoint_factory_matches_only_the_new_physics_sensor_route(tmp_path: Path) -> None:
    stream = SensorStreamPlan(
        stream_kind="imu",
        instance_id="thunder_01",
        sensor_id="thunder_01.imu",
        frame_id="thunder_01/imu",
        message_type="lingtu.dds.Imu",
        rate_hz=Fraction(200, 1),
        route=SensorRoute("physics", "mujoco_sensor", "typed_dds"),
    )
    allocation = RunAllocation(
        run_id="imu-route-test",
        run_dir=tmp_path,
        artifact_root=tmp_path.resolve(),
        log_dir=tmp_path / "logs",
        ports={},
        shm={},
        session_id="imu-session",
        boot_id="boot",
        dds_domain=42,
    )
    factory = ImuEndpointFactory(tmp_path / "lingtu_imu_publisher.exe")
    endpoint = SensorEndpointRouter((factory,))(stream, allocation)
    assert endpoint is not None
    assert endpoint.source_id == "mujoco-imu-dds"
    assert factory(
        SensorStreamPlan(
            stream_kind="imu", instance_id=stream.instance_id, sensor_id=stream.sensor_id,
            frame_id=stream.frame_id, message_type=stream.message_type, rate_hz=stream.rate_hz,
            route=SensorRoute("physics", "mujoco_truth", "typed_dds"),
        ), allocation
    ) is None
