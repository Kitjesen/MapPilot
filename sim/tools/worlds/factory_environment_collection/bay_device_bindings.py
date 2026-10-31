"""Pure, draft-only device bindings for the existing factory collision candidate.

``bind_devices(xml, candidate, recipe)`` returns new MJCF text and a binding
report; it does not read/write files, compile, step, publish, or launch anything.
Pass report["scene_measurements"] directly to make_measurement_extractor's scene
argument. The report itself is NOT a WorldPackage or a sensor plan.

Recipe positions are canonical world metres. Bodies have identity orientation:
an explicit body_origin_m, or the source component's converted world translation.
measurement_origin_m is an independently chosen point, not a visual bounding-box
centre. Source rotation/scale is already baked into the candidate geometry; it
must NOT be applied again. Stable source IDs and MJCF names survive, but compiled
numeric geom indices can change and must be resolved by name on each model.
Thermal, semantic and finite acoustic declarations are synthetic simulation inputs,
not properties measured from assets, inferred faults, or actual factory evidence.
"""

from __future__ import annotations

import copy
import re
from xml.etree import ElementTree as ET

import numpy as np
from sim.runtime.sensors.acoustic_scene import AcousticSceneSource
from sim.runtime.sensors.camera_geometry import PinholeCalibration
from sim.runtime.sensors.semantic_camera import SemanticCameraConfig, SemanticLabel
from sim.runtime.sensors.thermal import ThermalSurface

FRAME = "right_handed_Xforward_Yleft_Zup_m"
RECIPE_SCHEMA = "lingtu.factory-device-binding-recipe.v1"
REPORT_SCHEMA = "lingtu.factory-device-bindings.v1"


def _vector(value, name):
    raw = np.asarray(value)
    if raw.shape != (3,) or raw.dtype.kind not in "iuf" or not np.isfinite(raw).all():
        raise ValueError(f"{name} must contain three finite numeric metre coordinates")
    return raw.astype(float)


def _name(value, name):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
        raise ValueError(f"{name} must be a nonempty simple identifier")
    return value


def _index(rows, key, name):
    result = {}
    for row in rows:
        value = row.get(key)
        if not isinstance(value, str) or not value or value in result:
            raise ValueError(f"missing/duplicate {name}: {value!r}")
        result[value] = row
    return result


def _fields(value, required, optional, name):
    if not isinstance(value, dict) or required - value.keys() or value.keys() - required - optional:
        raise ValueError(f"{name} requires {sorted(required)}; optional {sorted(optional)}")


def _numbers(values):
    return " ".join(format(float(value), ".17g") for value in values)


def bind_devices(candidate_xml: str, candidate: dict, recipe: dict) -> tuple[str, dict]:
    """Group only declared components' existing geoms into named static bodies.

    Each recipe device has device_id, source_component_id, measurement_origin_m,
    thermal {temperature_k, emissivity}, semantic {class_id, instance_id}, and
    acoustic {pressure_pa, sample_rate_hz, start_sim_time_ns, reference_distance_m}.
    body_origin_m is optional (default: converted component world translation).
    The explicit list pressure_pa is finite, mono and non-looping. One component
    per device, all that component's existing geoms, no partial geom selection.
    Missing/duplicate components or geoms, overlapping mappings, non-world input
    geoms and undeclared recipe fields are errors, not fallback opportunities.
    Input dictionaries and strings are never mutated, including on rejection.
    """
    _fields(recipe, {"schema", "frame", "simulation", "provenance", "devices"}, set(), "recipe")
    if recipe["schema"] != RECIPE_SCHEMA or recipe["frame"] != FRAME or recipe["simulation"] is not True:
        raise ValueError("expected a canonical, explicitly simulation-only binding recipe")
    if not isinstance(recipe["provenance"], dict) or not recipe["provenance"]:
        raise ValueError("recipe provenance must explicitly describe the synthetic declarations")
    if not isinstance(recipe["devices"], list) or not recipe["devices"]:
        raise ValueError("devices must be a nonempty explicit list")
    if candidate.get("schema") != "lingtu.factory-collision-candidate.v1" or candidate.get("frame") != FRAME:
        raise ValueError("expected canonical factory collision candidate v1")

    components = _index(candidate["components"], "source_id", "source component")
    geometries = _index(candidate["geometries"], "source_id", "source geometry")
    named_geoms = _index(candidate["geometries"], "mjcf_geom", "MJCF geom name")
    root = ET.fromstring(candidate_xml)
    world = root.find("worldbody")
    if root.tag != "mujoco" or world is None or world.findall("body"):
        raise ValueError("expected existing static world geoms, not pre-grouped bodies")
    compiler = root.find("compiler")
    if compiler is not None and (
        compiler.get("coordinate", "local") != "local" or compiler.get("fusestatic", "false") != "false"
    ):
        raise ValueError("compiler must keep local coordinates and named static bodies")
    xml_geoms = _index([dict(g.attrib) for g in world.findall("geom")], "name", "XML geom")
    if xml_geoms.keys() != named_geoms.keys():
        raise ValueError("missing/extra XML geom relative to candidate manifest")
    claimed = set()
    for cid, component in components.items():
        ids = component["geometry_ids"]
        if len(ids) != len(set(ids)):
            raise ValueError(f"duplicate component geom: {cid}")
        for gid in ids:
            if gid not in geometries:
                raise ValueError(f"missing component geom: {gid}")
            if gid in claimed or geometries[gid]["component_source_id"] != cid:
                raise ValueError(f"overlapping component geom mapping: {gid}")
            claimed.add(gid)
    if claimed != geometries.keys():
        raise ValueError("missing component ownership for candidate geom")

    scene = {"thermal_surfaces": {}, "semantic_labels": {}, "acoustic_sources": []}
    bindings = []
    used_components, used_devices, used_instances = set(), set(), set()
    nodes = {g.get("name"): g for g in world.findall("geom")}
    for device in recipe["devices"]:
        _fields(
            device,
            {"device_id", "source_component_id", "measurement_origin_m", "thermal", "semantic", "acoustic"},
            {"body_origin_m"},
            "device",
        )
        did = _name(device["device_id"], "device_id")
        cid = device["source_component_id"]
        if not isinstance(cid, str) or cid not in components:
            raise ValueError(f"missing source component: {cid!r}")
        if did in used_devices or cid in used_components:
            raise ValueError(f"duplicate device or overlapping device component mapping: {did}")
        used_devices.add(did)
        used_components.add(cid)
        component = components[cid]
        if not component["geometry_ids"]:
            raise ValueError(f"source component has no existing geoms: {cid}")
        origin = (
            _vector(device["body_origin_m"], "body_origin_m")
            if "body_origin_m" in device
            else _vector(component["source_world_transform"]["translation_cm"], "source translation_cm")
            * [0.01, -0.01, 0.01]
        )
        measurement = _vector(device["measurement_origin_m"], "measurement_origin_m")
        _fields(device["thermal"], {"temperature_k", "emissivity"}, set(), "thermal")
        ThermalSurface(**device["thermal"])
        _fields(device["semantic"], {"class_id", "instance_id"}, set(), "semantic")
        label = SemanticLabel(0, **device["semantic"])
        SemanticCameraConfig(PinholeCalibration(1, 1, 1, 1, 0.5, 0.5), (label,))
        if label.instance_id in used_instances:
            raise ValueError("semantic instance_id must uniquely identify a device")
        used_instances.add(label.instance_id)
        _fields(
            device["acoustic"],
            {"pressure_pa", "sample_rate_hz", "start_sim_time_ns", "reference_distance_m"},
            set(),
            "acoustic",
        )
        if not isinstance(device["acoustic"]["pressure_pa"], list):
            raise ValueError("pressure_pa must be a finite nonempty mono waveform list")
        body_name = f"factory_device/{did}"
        acoustic = {
            **copy.deepcopy(device["acoustic"]),
            "source_id": did,
            "body_name": body_name,
            "local_offset_m": (measurement - origin).tolist(),
        }
        AcousticSceneSource(**acoustic)
        scene["acoustic_sources"].append(acoustic)
        body = ET.SubElement(world, "body", name=body_name, pos=_numbers(origin))
        geom_bindings = []
        for gid in component["geometry_ids"]:
            geometry = geometries[gid]
            name = geometry["mjcf_geom"]
            geom = nodes[name]
            if "fromto" in geom.attrib:
                raise ValueError("fromto geoms are not supported by this position-only candidate converter")
            position = _vector([float(v) for v in geom.get("pos", "0 0 0").split()], "geom pos")
            local = position - origin
            geom.set("pos", _numbers(local))
            world.remove(geom)
            body.append(geom)
            scene["thermal_surfaces"][name] = copy.deepcopy(device["thermal"])
            scene["semantic_labels"][name] = copy.deepcopy(device["semantic"])
            geom_bindings.append(
                {
                    "geometry_id": gid,
                    "mjcf_geom": name,
                    "world_pos_m": position.tolist(),
                    "body_local_pos_m": local.tolist(),
                }
            )
        bindings.append(
            {
                "device_id": did,
                "source_component_id": cid,
                "body_name": body_name,
                "body_pose": {"position_m": origin.tolist(), "orientation_wxyz": [1, 0, 0, 0]},
                "body_origin_policy": "recipe"
                if "body_origin_m" in device
                else "converted_source_component_world_translation",
                "measurement_pose": {"position_m": measurement.tolist(), "orientation_wxyz": [1, 0, 0, 0]},
                "measurement_local_offset_m": (measurement - origin).tolist(),
                "geometries": geom_bindings,
            }
        )
    report = {
        "schema": REPORT_SCHEMA,
        "status": "draft_synthetic_SIMULATION_unqualified_full_world_blocked",
        "simulation": True,
        "qualified_world": False,
        "qualified_collision": False,
        "full_world_blocked": True,
        "published_world": False,
        "physics_authority": "mujoco",
        "visual_aabb_used": False,
        "frame": FRAME,
        "recipe": copy.deepcopy(recipe),
        "source_candidate": copy.deepcopy(candidate),
        "bindings": bindings,
        "scene_measurements": scene,
        "unbound_geometry_ids": [
            g["source_id"] for g in candidate["geometries"] if g["component_source_id"] not in used_components
        ],
        "counts": {
            "devices": len(bindings),
            "bound_geoms": len(scene["thermal_surfaces"]),
            "total_geoms": len(geometries),
            "added_geoms": 0,
            "added_joints": 0,
        },
        "limitations": [
            "Synthetic simulation declarations only; asset exports contain no measured temperature, emissivity or pressure",
            "Source candidate provenance, unknowns and world blockers retained unchanged under source_candidate",
            "Stable source geometry IDs and MJCF names preserved; compiled numeric geom indices may change",
            "Only declared devices have measurement properties; all other surfaces remain undeclared/unknown",
            "No heat dynamics, room acoustics, sensor/robot/field qualification or actual factory inspection",
            "Not a WorldPackage; no package, default, lifecycle, physics policy or UE authority changes",
        ],
    }
    ET.indent(root)
    return ET.tostring(root, encoding="unicode") + "\n", report


def synthetic_factory_recipe() -> dict:
    """Two inspected source components, not a factory inspection configuration.

    Exported Switchboard06_358 translation is (6833.525879, -288.958252,
    1384.406006) cm, at ~13.84 m height, not a floor-mounted cabinet.
    EngineModifyPivot component -1 is at (5233.265259, 513.102314,
    143.150406) cm. No reachable inspection waypoint is implied by either point.
    """
    prefix = "/Game/Maps/Demonstration.Demonstration:PersistentLevel."
    devices = []
    for did, component, point, temperature, class_id, instance_id, pressure in (
        (
            "synthetic_switchboard",
            "SM_ElectricalSupply_Switchboard06_358.StaticMeshComponent0",
            [68.3352587890625, 2.88958251953125, 13.84406005859375],
            330.0,
            101,
            1001,
            [0.02, 0.0, -0.02, 0.0],
        ),
        (
            "synthetic_engine",
            "BP_SplineMovableV2_0.NODE_AddStaticMeshComponent-1",
            [52.33265259066842, -5.131023138907344, 1.4315040588378907],
            365.0,
            102,
            1002,
            [0.1, 0.0, -0.1, 0.0],
        ),
    ):
        devices.append(
            {
                "device_id": did,
                "source_component_id": prefix + component,
                "measurement_origin_m": point,
                "thermal": {"temperature_k": temperature, "emissivity": 0.9},
                "semantic": {"class_id": class_id, "instance_id": instance_id},
                "acoustic": {
                    "pressure_pa": pressure,
                    "sample_rate_hz": 1000,
                    "start_sim_time_ns": 0,
                    "reference_distance_m": 1.0,
                },
            }
        )
    return {
        "schema": RECIPE_SCHEMA,
        "frame": FRAME,
        "simulation": True,
        "provenance": {
            "declarations": "SYNTHETIC SIMULATION: invented Kelvin/emissivity, class/instance IDs and finite Pa clips; no actual measurements",
            "source_export": "build/unreal-tests/factory-bay-export-20261022-01/payload/bay-export.json",
            "supplement": "build/unreal-tests/factory-bay-collision-export-20261022-02/payload/collision-mesh-export.json",
            "candidate": "build/inspection-tests/20261022/factory-collision-candidate-02/manifest.json",
            "selection": "Explicit exported Switchboard and Engine components; names and world translations inspected, no AABB selection",
            "world_status": "DRAFT / UNQUALIFIED / FULL WORLD BLOCKED; not a route, spawn or field inspection",
        },
        "devices": devices,
    }
