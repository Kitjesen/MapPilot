"""Bounded physical measurement/recording check, NOT factory Product qualification.

Run with the repository and src/ on PYTHONPATH. Creates a new output directory,
three controlled cases, real MuJoCo RGB, raw NPZ measurements and inspection
evidence using the production sensor/session/store components. No motion owner,
native route success, UE integration or hardware fidelity is claimed.
"""

from __future__ import annotations

import argparse
import json
import time
from collections.abc import Mapping
from contextlib import nullcontext
from dataclasses import asdict, replace
from fractions import Fraction
from pathlib import Path

import cv2
import numpy as np
from sim.runtime.recording.measurement_payload import encode_measurement_payload
from sim.runtime.recording.measurement_sink import MeasurementSink
from sim.runtime.recording.timeline import SimulationRecordingWriter
from sim.runtime.sensors.acoustic_scene import AcousticScene, AcousticSceneSource
from sim.runtime.sensors.acoustics import MicrophoneConfig
from sim.runtime.sensors.camera_geometry import CameraPose, PinholeCalibration, calibrated_rays
from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.measurement_extractors import (
    AcousticExtractor,
    MountedCamera,
    SemanticCameraExtractor,
    ThermalCameraExtractor,
)
from sim.runtime.sensors.measurement_scene import thermal_calibration
from sim.runtime.sensors.runtime import SensorRuntime
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.session import SensorEndpoint, SessionSensorRuntime
from sim.runtime.sensors.thermal import ThermalConfig, ThermalSurface

import mujoco
from perception.inspection.measurement_checks import AcousticRule, ThermalRule, evaluate_acoustic, evaluate_thermal
from runtime.contracts.inspection_evidence import (
    InspectionEvidenceRequest,
    InspectionEvidenceStore,
    InspectionFinding,
    MeasurementArtifact,
)

SCENE = """<mujoco model="measurement-contract-scene">
  <option timestep=".01" gravity="0 0 -9.81"/>
  <visual><global offwidth="320" offheight="240"/></visual>
  <worldbody>
    <light pos="0 0 4" dir="1 0 -1" diffuse=".8 .8 .8"/>
    <geom name="floor" type="plane" size="8 8 .1" rgba=".45 .48 .5 1"/>
    <body name="robot" pos="0 0 .8"><geom type="sphere" size=".01"/>
      <camera name="evidence" quat=".5 .5 -.5 -.5" fovy="53.130102"/>
    </body>
    <body name="pump" pos="3 0 1"><geom name="pump_surface" type="box" size=".2 .8 .8" rgba=".15 .4 .55 1"/></body>
    <body name="cover" mocap="true" pos="1.5 5 1"><geom name="unknown_cover" type="box" size=".1 1 1.2" rgba=".5 .5 .5 1"/></body>
  </worldbody>
</mujoco>"""


def _plain(value):
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    return value


def run(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=False)
    (output / "scene.xml").write_text(SCENE, encoding="utf-8")
    session_id = output.name
    model = mujoco.MjModel.from_xml_string(SCENE)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def read_state():
        return nullcontext((model, data))  # Single owner; never concurrently stepped.

    calibration = PinholeCalibration(64, 48, 48, 48, 32, 24)
    camera = MountedCamera("robot", calibration, CameraPose((0, 0, 0), (0.5, -0.5, 0.5, -0.5)))
    stamp = SensorSampleStamp(session_id, "robot", "robot.thermal", "robot/optical", 0, 0, 0, 0)
    thermal = ThermalCameraExtractor(
        read_state,
        camera,
        ThermalConfig(
            thermal_calibration(calibrated_rays(calibration, stamp), "diagnostic-camera"),
            293.15,
            temperature_mode="surface_corrected",
        ),
        {"pump_surface": ThermalSurface(320, 0.8)},
    )
    source = AcousticSceneSource("pump", "pump", 0.3 * np.sin(2 * np.pi * 250 * np.arange(8000) / 8000), 8000, 0, 1)
    acoustic = AcousticExtractor(
        read_state,
        AcousticScene(
            MicrophoneConfig(
                8000,
                80,
                ((0, 0, 0),),
                sensitivity_volts_per_pa=0.02,
                calibration_metadata={"calibration_id": "diagnostic-mic", "kind": "declared_simulation_flat_response"},
            ),
            receiver_body_name="robot",
            sources=(source,),
        ),
    )
    semantic = SemanticCameraExtractor(read_state, camera, {"pump_surface": (1, 101), "floor": (2, 102)})
    extractors = {"thermal": thermal, "acoustic": acoustic, "semantic": semantic}
    streams = tuple(
        SensorStreamPlan(
            kind,
            "robot",
            f"robot.{kind}",
            "robot/optical",
            "raw_measurement",
            Fraction(100 if kind == "acoustic" else 20),
            SensorRoute("physics", "mujoco_measurement", "in_process"),
        )
        for kind in extractors
    )
    pending = []
    sinks = {
        stream.sensor_id: MeasurementSink(stream.sensor_id, stream.stream_kind, on_record=pending.append)
        for stream in streams
    }
    runtime = SessionSensorRuntime(
        plan=SensorRuntime(session_id, streams),
        allocation=None,
        endpoint_factory=lambda stream, _: SensorEndpoint(
            "mujoco-contract-scene", sinks[stream.sensor_id], extractors[stream.stream_kind]
        ),
    )
    store = InspectionEvidenceStore(output / "inspection")
    report = {
        "qualification": "CONTROLLED_PHYSICAL_DATA_NOT_FACTORY_PRODUCT",
        "session_id": session_id,
        "physics": f"MuJoCo {mujoco.__version__}",
        "cases": [],
        "status": "RUNNING",
    }
    renderer = None
    runtime.prepare()
    try:
        for sensor_id in runtime.active_sensor_ids:
            runtime.set_recording(sensor_id, True)
        renderer = mujoco.Renderer(model, height=240, width=320)
        with SimulationRecordingWriter(
            output / "recording", run_id=session_id, session_id=session_id, required_content={"sensor_payload"}
        ) as writer:
            for tick in range(31):
                case = {10: "normal", 20: "over_threshold", 30: "occluded"}.get(tick)
                if tick == 11:
                    thermal.surfaces = {"pump_surface": ThermalSurface(390, 0.8)}
                    acoustic.scene.sources = (replace(source, pressure_pa=source.pressure_pa * (1 / 0.3)),)
                if tick == 21:
                    data.mocap_pos[0, 1] = 0
                snapshot = dict(
                    event="snapshot",
                    session_id=session_id,
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
                if case:
                    renderer.update_scene(data, camera="evidence")
                    rgb = renderer.render()
                    ok, png = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    if not ok:
                        raise RuntimeError("RGB PNG encoding failed")
                    (output / f"{case}.png").write_bytes(png.tobytes())
                    case_result = {"case": case, "sim_time_ns": snapshot["sim_time_ns"], "sensors": {}}
                    for modality in ("thermal", "acoustic"):
                        frame = sinks[f"robot.{modality}"].latest
                        if frame is None or frame.stamp.sim_time_ns != snapshot["sim_time_ns"]:
                            raise RuntimeError("capture did not receive same-time measurement")
                        if modality == "thermal":
                            provenance = {
                                "model_id": "declared_broadband_greybody",
                                "model_version": "1",
                                "qualification": "EXPERIMENTAL",
                                "env": "sim",
                            }
                            assessment = evaluate_thermal(
                                frame.temperature_k,
                                frame.valid_mask,
                                ThermalRule(
                                    350,
                                    (24, 16, 40, 32),
                                    0.9,
                                    "corrected",
                                    "diagnostic-camera",
                                    provenance["model_id"],
                                    "1",
                                ),
                                unit=frame.metadata["temperature_unit"],
                                temperature_kind="corrected"
                                if frame.metadata["temperature_kind"] == "emissivity_corrected_surface"
                                else "apparent",
                                calibration=frame.metadata["camera_calibration"],
                                provenance=provenance,
                            )
                        else:
                            provenance = {
                                "model_id": "direct_path_pressure",
                                "model_version": "1",
                                "qualification": "EXPERIMENTAL",
                                "env": "sim",
                            }
                            assessment = evaluate_acoustic(
                                frame.pressure_pa,
                                frame.valid_mask,
                                AcousticRule(0.15, (0,), 0.01, 1, "diagnostic-mic", provenance["model_id"], "1"),
                                fs=frame.sample_rate_hz,
                                unit="Pa",
                                calibration=frame.metadata["calibration"],
                                provenance=provenance,
                            )
                        encoded = encode_measurement_payload(frame, modality=modality)
                        now = time.time()
                        request = InspectionEvidenceRequest(
                            task_id="diagnostic-not-native-route",
                            route_id="controlled-cases",
                            route_revision=1,
                            map_id="measurement-contract-scene",
                            map_content_epoch=1,
                            point_index=len(report["cases"]),
                            request_id=f"{case}-{modality}",
                            action=f"capture:{modality}",
                            requested_at_s=now,
                            deadline_s=now + 10,
                        )
                        finding = InspectionFinding(
                            assessment.verdict,
                            "explicit_threshold_rule",
                            assessment.reason,
                            details={"metrics": _plain(assessment.metrics), "rule": _plain(assessment.rule_details)},
                            modality=modality,
                            model_provenance=provenance,
                        )
                        result = store.persist(
                            request,
                            rgb_bytes=png.tobytes(),
                            media_type="image/png",
                            pose={
                                "source_stamp": asdict(frame.stamp),
                                "body": "robot",
                                "position_m": data.body("robot").xpos.tolist(),
                                "quaternion_wxyz": data.body("robot").xquat.tolist(),
                            },
                            finding=finding,
                            measurements=(
                                MeasurementArtifact(
                                    modality=modality,
                                    sensor_id=encoded.sensor_id,
                                    payload=encoded.payload,
                                    media_type=encoded.media_type,
                                    metadata=encoded.metadata,
                                ),
                            ),
                        )
                        case_result["sensors"][modality] = {
                            "verdict": result.analysis_verdict,
                            "reason": assessment.reason,
                            "metrics": _plain(assessment.metrics),
                            "evidence_id": request.request_id,
                        }
                    report["cases"].append(case_result)
                if tick < 30:
                    mujoco.mj_step(model, data)
        report["sensor_status"] = runtime.status()
        expected = ["normal", "anomaly", "inconclusive"]
        if any(
            [case["sensors"][modality]["verdict"] for case in report["cases"]] != expected
            for modality in ("thermal", "acoustic")
        ):
            raise RuntimeError("physical case verdicts do not match controlled inputs")
        report["status"] = "PASS"
    except Exception as exc:
        report["status"] = "FAIL"
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        runtime.close()
        if renderer is not None:
            renderer.close()
        report["cleanup"] = "closed_in_process_sensor_sinks_and_renderer"
        (output / "result.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8"
        )
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.output)
    print(
        json.dumps(
            {"status": result["status"], "output": str(args.output.resolve()), "cases": result["cases"]},
            ensure_ascii=False,
        )
    )
