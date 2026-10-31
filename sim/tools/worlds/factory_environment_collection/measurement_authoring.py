"""Offline measurement authoring for existing Fab collision geometry only.

Properties and anomaly thresholds are explicit author assumptions, not asset
metadata, field measurements, diagnosis, or qualification. No runtime is started.
"""

from __future__ import annotations

import argparse
import json
import shutil
from contextlib import nullcontext
from dataclasses import asdict
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import numpy as np
import yaml

from .bay_device_bindings import FRAME, RECIPE_SCHEMA, bind_devices

PREFIX = "/Game/Maps/Demonstration.Demonstration:PersistentLevel."
CONTROL = PREFIX + "SM_AssemblyLineControl22_81.StaticMeshComponent0"
ENGINE = PREFIX + "BP_SplineMovableV2_0.NODE_AddStaticMeshComponent-1"
WORLD_ID = "factory_environment_collection_measurements"
VERSION = "0.1.0"


def author_measurements(candidate_xml: str, candidate: dict) -> tuple[str, dict]:
    """Bind the reviewed Control22 and Engine source components, never proxies."""
    components = {c["source_id"]: c for c in candidate["components"]}
    for cid in (CONTROL, ENGINE):
        if cid not in components or not components[cid]["geometry_ids"]:
            raise ValueError(f"measurement authoring blocked: missing source component/geoms: {cid}")
    control = np.asarray(components[CONTROL]["source_world_transform"]["translation_cm"]) * [0.01, -0.01, 0.01]
    engine = next((g for g in candidate["geometries"] if g["component_source_id"] == ENGINE), None)
    if engine is None:
        raise ValueError(f"measurement authoring blocked: missing component geom: {ENGINE}")
    recipe: dict[str, Any] = {
        "schema": RECIPE_SCHEMA,
        "frame": FRAME,
        "simulation": True,
        "provenance": {
            "basis": "author_assumptions_not_field_measurements",
            "selection": "Reviewed Fab Demonstration Control22 console and Engine component source paths",
            "geometry": "Existing collision candidate only; no AABB, replacement, clipping or added geom",
            "waveform": "Finite 0.2 s Hann-windowed sinusoid at 16000 Hz, Pa at 0.1 m; no looping",
        },
        "devices": [],
    }
    assumptions: dict[str, Any] = {"basis": "author_assumptions_not_field_measurements", "devices": {}}
    for did, cid, point, temperature, emissivity, label, amplitude, frequency, threshold in (
        ("control22", CONTROL, control + np.array([0, 0, 1]), 330.0, 0.85, 101, 0.02, 440.0, 345.0),
        ("engine", ENGINE, engine["mesh_origin_m"], 365.0, 0.9, 102, 0.1, 180.0, 360.0),
    ):
        clip = amplitude * np.sin(2 * np.pi * frequency * np.arange(3200) / 16000) * np.hanning(3200)
        recipe["devices"].append(
            {
                "device_id": did,
                "source_component_id": cid,
                "measurement_origin_m": np.asarray(point).tolist(),
                "thermal": {"temperature_k": temperature, "emissivity": emissivity},
                "semantic": {"class_id": label, "instance_id": 2000 + label},
                "acoustic": {
                    "pressure_pa": clip.tolist(),
                    "sample_rate_hz": 16000,
                    "start_sim_time_ns": 0,
                    "reference_distance_m": 0.1,
                },
            }
        )
        assumptions["devices"][did] = {
            "anomaly_temperature_k": threshold,
            "temperature_mode": "surface_corrected",
            "expected_design_state": "above_threshold" if temperature > threshold else "below_threshold",
            "class_meaning": "assembly_line_control_console" if cid == CONTROL else "engine_component",
            "acoustic_peak_pa_at_reference": amplitude,
            "acoustic_frequency_hz": frequency,
            "limitation": "No measured thermal/emissivity/acoustic data or validated fault threshold",
        }
    output, report = bind_devices(candidate_xml, candidate, recipe)
    report["assumptions"] = assumptions
    return output, report


def write_measurement_candidate(repo_root: Path, candidate_dir: Path, output_dir: Path) -> Path:
    """Write a NEW offline catalog candidate, never promote or qualify a World.

    Uses WorldImporter's public static-MJCF candidate intake. Its ImportDraft
    stays unqualified, with original source/provenance retained and no fabricated
    visual alignment or approved spawn. An explicit extra catalog root allows
    offline SensorPlan compilation without promotion into sim/packages.
    """
    from sim.catalog.importers.world import WorldImporter

    repo_root, candidate_dir, output_dir = map(Path, (repo_root, candidate_dir, output_dir))
    if output_dir.exists():
        raise FileExistsError(f"measurement candidate directory must not exist: {output_dir}")
    candidate = json.loads((candidate_dir / "manifest.json").read_text(encoding="utf-8"))
    xml, report = author_measurements(
        (candidate_dir / "collision-candidate.xml").read_text(encoding="utf-8"), candidate
    )
    # The source candidate has no authored runtime policy. Make the experimental
    # policy explicit without changing any geometry, contact flags or transforms.
    root = ET.fromstring(xml)
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.attrib.update(
        timestep="0.001", integrator="implicitfast", solver="Newton", iterations="100", gravity="0 0 -9.81"
    )
    source_root = output_dir / "authoring-source"
    source_root.mkdir(parents=True, exist_ok=False)
    mjcf = source_root / "world.xml"
    mjcf.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")
    report["remaining_blockers"] = [
        "Fab visual migration/alignment pending",
        "Navigation/spawn and world qualification pending",
    ]
    report["assumptions"]["physics_policy"] = (
        "1 ms implicitfast/Newton/100 iterations, gravity -9.81 m/s2; author assumption"
    )
    (source_root / "source-bindings.json").write_text(
        json.dumps(
            [
                {key: geom[key] for key in ("source_id", "component_source_id", "mjcf_geom")}
                for geom in candidate["geometries"]
            ],
            indent=2,
        ),
        encoding="utf-8",
    )
    (source_root / "authoring-report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    # This is the existing local acquisition/provenance record, not an invented
    # replacement for the license text or a new asset redistribution grant.
    shutil.copyfile(repo_root / "sim/library/SOURCES.md", source_root / "source-notice.md")
    request = {
        "package": {
            "id": WORLD_ID,
            "version": VERSION,
            "description": "UNQUALIFIED offline Fab measurement candidate; no visual migration or route",
        },
        "source": {
            "path": str(source_root.resolve()),
            "provenance": {
                "owner": "Denys Rutkovskyi / Factory Environment Collection",
                "license": "Fab Standard License; acquisition recorded in source-notice.md; restricted raw redistribution",
                "license_file": "source-notice.md",
                "source_uri": "https://www.fab.com/listings/2ee66462-8c2b-4303-892c-83f7fc0d9b3e",
            },
        },
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "mjcf": "world.xml",
        "source_bindings": "source-bindings.json",
        "visual": {"binding": "WorldVisual:FactoryEnvironmentCollectionCandidate", "level": "/Game/Maps/Demonstration"},
        "measurements": report["scene_measurements"],
    }
    draft = WorldImporter(repo_root).import_mjcf_candidate(
        request, draft_root=output_dir / "packages" / WORLD_ID / VERSION
    )
    report["importer_status"] = "candidate_imported_unqualified"
    report["draft"] = draft.to_dict()
    (output_dir / "authoring-report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    session = {
        "schema": "lingtu.sim.session.v1",
        "session_id": WORLD_ID,
        "mujoco_version": "3.10.0",
        "seed": 20261023,
        "world": f"{WORLD_ID}@{VERSION}",
        "robots": [
            {
                "instance_id": "thunder_01",
                "package": "thunderv4@1.0.3",
                "sensor_rig": "thunderv4_headless_multimodal@1.0.0",
                "controller": None,
                "spawn": {"position_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
            }
        ],
        "runtime": {"backend": "mujoco", "mode": "headless", "required_bindings": ["physics", "sensors"]},
    }
    session_path = output_dir / "session.yaml"
    session_path.write_text(
        "# UNQUALIFIED compile-only candidate; placeholder spawn is NOT navigable/approved.\n"
        + yaml.safe_dump(session, sort_keys=False),
        encoding="utf-8",
    )
    return session_path


def sample_candidate(repo_root: Path, resolved) -> tuple[dict, dict]:
    """Sample compiled configuration on static, geometry-free test receivers.

    Receiver poses are synthetic observation fixtures, not navigable waypoints,
    measured robot calibrations, or an inspection Product run. No stepping or
    model composition with a robot occurs. The compiled scene stays unfiltered.
    """
    import mujoco
    from scipy.spatial.transform import Rotation
    from sim.runtime.sensors.camera_geometry import CameraPose, PinholeCalibration, calibrated_rays
    from sim.runtime.sensors.contracts import DeadlinePolicy, ScheduledSensorSample
    from sim.runtime.sensors.measurement_configuration import make_measurement_extractor
    from sim.runtime.sensors.runtime import SensorRuntime

    world_path = Path(repo_root) / resolved.physics_plan["world"]["mjcf"]
    report = json.loads((world_path.parent / "authoring-report.json").read_text(encoding="utf-8"))
    plan = resolved.sensor_plan
    runtime = SensorRuntime.from_plan(plan)
    streams = {s.stream_kind: s for s in runtime.streams if s.stream_kind in {"thermal", "semantic", "acoustic"}}
    configs = {kind: plan["streams"][kind][0]["measurement"] for kind in streams}
    body_names = {
        kind: config["body_frame_id"].removeprefix(streams[kind].instance_id + "/") for kind, config in configs.items()
    }
    root = ET.parse(world_path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("candidate sampling requires an existing worldbody")
    for name in set(body_names.values()):
        if any(body.get("name") == name for body in root.iter("body")):
            raise ValueError(f"fixture receiver collides with source body name: {name}")
        ET.SubElement(worldbody, "body", name=name)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    recipe = {d["device_id"]: d for d in report["recipe"]["devices"]}
    points = {name: np.asarray(d["measurement_origin_m"]) for name, d in recipe.items()}
    control_origin = points["control22"] - [0, 0, 1]
    rigs = {
        "control22": control_origin + np.array([-0.75, 0, 0.9012481689453125]),
        "engine": points["engine"] + [0, -0.75, 0.56],
        "wall": np.array([points["engine"][0], 12.0, 1.70125]),
    }
    arrays, captures = {}, {}

    def rotation(wxyz):
        return Rotation.from_quat(np.asarray(wxyz)[[1, 2, 3, 0]])

    def capture(name, kind, receiver_position, desired_rotation, time_ns=0):
        stream, config = streams[kind], configs[kind]
        mount = config["mount_pose"]
        body_rotation = desired_rotation * rotation(mount["orientation_wxyz"]).inv()
        body_id = model.body(body_names[kind]).id
        model.body_pos[body_id] = receiver_position - body_rotation.apply(mount["position_m"])
        xyzw = body_rotation.as_quat()
        model.body_quat[body_id] = xyzw[[3, 0, 1, 2]]
        data.time = time_ns / 1e9
        request = ScheduledSensorSample(
            resolved.session["session_id"], stream, 0, 0, 0, time_ns, DeadlinePolicy.DROP_INTERMEDIATE
        )
        extractor = make_measurement_extractor(
            stream, config, plan["scene_measurements"], lambda: nullcontext((model, data))
        )
        frame = extractor(
            request,
            {
                "session_id": request.session_id,
                "model_generation": 0,
                "reset_generation": 0,
                "sim_time_ns": time_ns,
            },
        )
        captures[f"{name}_{kind}"] = {"stamp": asdict(frame.stamp), "configuration": config}
        if kind == "thermal":
            arrays[f"{name}_temperature_k"] = frame.temperature_k
            arrays[f"{name}_thermal_valid"] = frame.valid_mask
            captures[f"{name}_{kind}"]["metadata"] = frame.metadata
            rays = calibrated_rays(
                PinholeCalibration(**{k: v for k, v in config["camera"].items() if k != "model"}),
                frame.stamp,
                base_pose=CameraPose(tuple(data.xpos[body_id]), tuple(data.xquat[body_id])),
                mount_pose=CameraPose(tuple(mount["position_m"]), tuple(mount["orientation_wxyz"])),
            )
            nearest = np.full(frame.valid_mask.shape, -1, dtype=np.int32)
            for pixel in np.ndindex(nearest.shape):
                gid = np.array([-1], dtype=np.int32)
                distance = mujoco.mj_ray(
                    model, data, rays.origins_m[pixel], rays.directions_world[pixel], None, 1, body_id, gid
                )
                if 0.001 <= distance <= config.get("max_distance_m", 100):
                    nearest[pixel] = gid[0]
            known_ids = [model.geom(geom).id for geom in plan["scene_measurements"]["thermal_surfaces"]]
            if not np.array_equal(frame.valid_mask, np.isin(nearest, known_ids)):
                raise ValueError("thermal validity differs from independent nearest-geom scalar rays")
            arrays[f"{name}_nearest_geom_ids"] = nearest
        elif kind == "semantic":
            arrays[f"{name}_class_ids"] = frame.class_ids
            arrays[f"{name}_instance_ids"] = frame.instance_ids
            arrays[f"{name}_semantic_valid"] = frame.valid_mask
        else:
            arrays[f"{name}_pressure_pa"] = frame.pressure_pa
            arrays[f"{name}_acoustic_valid"] = frame.valid_mask
            captures[f"{name}_{kind}"]["metadata"] = frame.metadata
        return frame

    for name, rig in rigs.items():
        target = points["control22"] if name == "control22" else points["engine"]
        optical = rig + np.array([0, 0, 0.04])
        forward = target - optical
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0, 0, 1])
        right /= np.linalg.norm(right)
        view_rotation = Rotation.from_matrix(np.column_stack((right, np.cross(forward, right), forward)))
        for kind in ("thermal", "semantic"):
            capture(name, kind, optical, view_rotation)
        capture(name, "acoustic", rig + np.array([0, 0.08, 0]), Rotation.identity())
    capture("mixed", "acoustic", rigs["control22"] + [0, 0.08, 0], Rotation.identity(), 50_000_000)
    capture("expired", "acoustic", rigs["control22"] + [0, 0.08, 0], Rotation.identity(), 1_000_000_000)
    evidence = {
        "status": "UNQUALIFIED_STATIC_MEASUREMENT_FIXTURE",
        "fixture_only": True,
        "qualified_world": False,
        "product_run": False,
        "inspection_success": False,
        "ngeom": int(model.ngeom),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "initial_sim_time_ns": 0,
        "final_fixture_sim_time_ns": round(data.time * 1e9),
        "physics_steps": 0,
        "expired_sample_time_ns": 1_000_000_000,
        "independent_scalar_rays": int(sum(arrays[f"{name}_thermal_valid"].size for name in rigs)),
        "acoustic_sample_rate_hz": configs["acoustic"]["microphone"]["sample_rate_hz"],
        "scene_source": "compiled_sensor_plan_unfiltered",
        "geom_bindings": [
            {
                "compiled_geom_id": int(model.geom(g["mjcf_geom"]).id),
                "mjcf_geom": g["mjcf_geom"],
                "source_geometry_id": g["source_id"],
                "source_component_id": g["component_source_id"],
            }
            for g in report["source_candidate"]["geometries"]
        ],
        "camera_valid_pixels": {name: int(arrays[f"{name}_thermal_valid"].sum()) for name in rigs},
        "acoustic_stats": {
            name: {
                "valid_samples": int(arrays[f"{name}_acoustic_valid"].sum()),
                "peak_pa": float(np.max(np.abs(arrays[f"{name}_pressure_pa"][arrays[f"{name}_acoustic_valid"]])))
                if arrays[f"{name}_acoustic_valid"].any()
                else None,
            }
            for name in (*rigs, "mixed", "expired")
        },
        "captures": captures,
        "rig_positions_m": {name: point.tolist() for name, point in rigs.items()},
        "units": {"temperature": "K apparent", "pressure": "Pa", "class_instance": "author_assumed_labels"},
        "limitations": [
            *report["remaining_blockers"],
            "No UE/RGB, robot trajectory, native Product or field qualification",
        ],
    }
    return arrays, evidence


def main():
    from sim.catalog import CatalogResolver

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--sample", action="store_true", help="Acquire static MuJoCo fixture measurements, no motion")
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[4]
    session = write_measurement_candidate(repo_root, args.candidate, args.output)
    resolved = CatalogResolver(repo_root, [repo_root / "sim/packages", session.parent / "packages"]).resolve(session)
    resolved.write_bundle(session.parent / "bundle")
    if args.sample:
        arrays, evidence = sample_candidate(repo_root, resolved)
        np.savez_compressed(session.parent / "measurements.npz", **arrays)
        (session.parent / "sampling-evidence.json").write_text(
            json.dumps(evidence, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
    print(json.dumps({"status": "UNQUALIFIED_CANDIDATE", "session": str(session), "candidate_intake_complete": True}))


if __name__ == "__main__":
    main()
