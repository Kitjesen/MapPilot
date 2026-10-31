"""Actual MuJoCo -> shared session controls -> raw timeline -> replay."""

from contextlib import nullcontext
from fractions import Fraction

import numpy as np
import pytest
from sim.runtime.recording.measurement_payload import decode_measurement_payload
from sim.runtime.recording.measurement_sink import MeasurementSink
from sim.runtime.recording.timeline import SimulationRecordingWriter
from sim.runtime.replay import SimulationReplay
from sim.runtime.sensors.acoustic_scene import AcousticScene, AcousticSceneSource
from sim.runtime.sensors.acoustics import MicrophoneConfig
from sim.runtime.sensors.camera_geometry import PinholeCalibration, calibrated_rays
from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.measurement_control import MeasurementNoise
from sim.runtime.sensors.measurement_extractors import AcousticExtractor, MountedCamera, ThermalCameraExtractor
from sim.runtime.sensors.measurement_scene import thermal_calibration
from sim.runtime.sensors.runtime import SensorRuntime
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.session import SensorEndpoint, SessionSensorRuntime
from sim.runtime.sensors.thermal import ThermalConfig, ThermalSurface


def test_real_physical_session_controls_recording_and_replay(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""<mujoco><option timestep=".01" gravity="0 0 0"/>
      <worldbody><body name="robot"><freejoint/><geom type="sphere" size=".01"/></body>
      <body name="pump" pos="0 0 3"><geom name="hot" type="box" size="2 2 .1"/></body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def read_state():
        return nullcontext((model, data))

    calibration = PinholeCalibration(8, 6, 6, 6, 4, 3)
    stamp = SensorSampleStamp("integrated", "robot", "robot.thermal", "camera", 0, 0, 0, 0)
    thermal = ThermalCameraExtractor(
        read_state,
        MountedCamera("robot", calibration),
        ThermalConfig(thermal_calibration(calibrated_rays(calibration, stamp)), 293),
        {"hot": ThermalSurface(360, 1)},
    )
    acoustic = AcousticExtractor(
        read_state,
        AcousticScene(
            MicrophoneConfig(8000, 80, ((0, 0, 0),)),
            receiver_body_name="robot",
            sources=(AcousticSceneSource("pump", "pump", np.ones(8000), 8000, 0, 1),),
        ),
    )
    streams = tuple(
        SensorStreamPlan(
            kind,
            "robot",
            f"robot.{kind}",
            "camera",
            "raw_measurement",
            Fraction(rate),
            SensorRoute("physics", "mujoco_measurement", "in_process"),
        )
        for kind, rate in (("thermal", 20), ("acoustic", 100))
    )
    pending = []
    sinks = {
        stream.sensor_id: MeasurementSink(stream.sensor_id, stream.stream_kind, on_record=pending.append)
        for stream in streams
    }
    extractors = {"thermal": thermal, "acoustic": acoustic}
    runtime = SessionSensorRuntime(
        plan=SensorRuntime("integrated", streams),
        allocation=None,
        endpoint_factory=lambda stream, allocation: SensorEndpoint(
            "mujoco-measurements", sinks[stream.sensor_id], extractors[stream.stream_kind]
        ),
    )
    runtime.prepare()
    runtime.set_recording("robot.thermal", True)
    runtime.set_recording("robot.acoustic", True)
    runtime.configure_noise("robot.thermal", MeasurementNoise("K", False, 0.2, 19))
    assert runtime.status("robot.thermal")["noise"]["seed"] == 19
    with SimulationRecordingWriter(
        tmp_path, run_id="measurement-physical", session_id="integrated", required_content={"sensor_payload"}
    ) as writer:
        for tick in range(16):
            if tick == 3:
                runtime.set_enabled("robot.thermal", False)
            if tick == 7:
                runtime.set_enabled("robot.thermal", True)
            snapshot = dict(
                event="snapshot",
                session_id="integrated",
                model_generation=0,
                reset_generation=0,
                sequence=tick,
                physics_step=tick,
                sim_time_ns=round(data.time * 1e9),
                bodies=[],
            )
            runtime.process_snapshot(snapshot)
            writer.append(snapshot, sensor_payloads=list(pending))
            pending.clear()
            if tick == 5:
                assert runtime.status("robot.thermal")["state"] == "DISABLED"
                assert (
                    next(item for item in runtime.evidence_observations() if item["sensor_id"] == "robot.thermal")[
                        "state"
                    ]
                    == "DISABLED"
                )
            mujoco.mj_step(model, data)
    runtime.close()
    replay = SimulationReplay.open(tmp_path)
    recovered = []
    for frame in replay.frames:
        for payload in frame.sensor_payloads:
            sample = decode_measurement_payload(replay.read_sensor_payload(payload), modality=payload["stream_kind"])
            recovered.append(sample)
    thermal_frames = [f for f in recovered if f.stamp.sensor_id == "robot.thermal"]
    audio_frames = [f for f in recovered if f.stamp.sensor_id == "robot.acoustic"]
    assert thermal_frames and len(audio_frames) == 16
    assert any(frame.stamp.sim_time_ns >= 70_000_000 for frame in thermal_frames)
    assert not any(30_000_000 <= frame.stamp.sim_time_ns < 70_000_000 for frame in thermal_frames)
    assert any(frame.valid_mask.any() for frame in thermal_frames)
    for frame in thermal_frames:
        np.testing.assert_allclose(frame.temperature_k[frame.valid_mask], 360)
    np.testing.assert_allclose(audio_frames[-1].pressure_pa, 1 / 3)
