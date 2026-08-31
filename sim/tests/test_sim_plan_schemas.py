"""Gate 0 tests for the simulation plan JSON Schema contracts.

The repository intentionally does not add a JSON Schema dependency. This test
contains a small validator for the draft-2020-12 vocabulary used by the local
contracts, so positive and negative cases use only the standard library.
"""

# ruff: noqa: S101

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, cast

import pytest
import yaml

from sim.catalog import CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_ROOT = REPO_ROOT / "schemas" / "simulation"
RELATIVE_PATH_SCHEMAS = (
    "entity-visual.v1.json",
    "qualification.v1.json",
    "robot-import.v1.json",
    "robot.v1.json",
    "robot-visual.v1.json",
    "world-import.v1.json",
    "world.v1.json",
    "world-visual.v1.json",
    "visual-plan.v1.json",
)
RELATIVE_PATH_DEF_NAMES = {"visual-plan.v1.json": "relativePath"}
SCHEMAS = {
    "physics": SCHEMA_ROOT / "physics-plan.v1.json",
    "visual": SCHEMA_ROOT / "visual-plan.v1.json",
    "sensor": SCHEMA_ROOT / "sensor-plan.v1.json",
    "control": SCHEMA_ROOT / "control-plan.v1.json",
    "scenario": SCHEMA_ROOT / "scenario-plan.v1.json",
    "transport": SCHEMA_ROOT / "transport-intent.v1.json",
}
PACKAGE_SCHEMAS = {
    "controller": SCHEMA_ROOT / "controller.v1.json",
}
SESSION_ID = "contract_session"
THUNDER_UNREAL_SESSION = REPO_ROOT / "sim" / "scenarios" / "catalog" / "thunderv4_unreal" / "session.yaml"
THUNDER_PEDESTRIAN_CROSSING_SESSION = (
    REPO_ROOT / "sim" / "scenarios" / "catalog" / "open_field_pedestrian_crossing" / "session.yaml"
)
THUNDER_SENSOR_RIG_PACKAGE = (
    REPO_ROOT / "sim" / "sensor_rigs" / "doso" / "thunder_v4" / "navigation" / "sensor-rig.package.yaml"
)
THUNDER_ROBOT_PACKAGE = REPO_ROOT / "sim" / "robots" / "doso" / "thunder_v4" / "robot.package.yaml"


class SchemaError(ValueError):
    """Raised when an instance does not satisfy a local schema."""


@pytest.mark.parametrize("filename", RELATIVE_PATH_SCHEMAS)
def test_all_filesystem_relative_path_schemas_reject_nested_colons(filename: str) -> None:
    document = json.loads((SCHEMA_ROOT / filename).read_text(encoding="utf-8"))
    definition = RELATIVE_PATH_DEF_NAMES.get(filename, "relative_path")
    pattern = document["$defs"][definition]["pattern"]

    assert re.search(pattern, "nested/ordinary/file.bin")
    assert re.search(pattern, "nested:file/file.bin") is None
    assert re.search(pattern, "nested/file:ads") is None


def test_world_package_path_schema_rejects_nested_colons() -> None:
    document = json.loads((SCHEMA_ROOT / "world.v1.json").read_text(encoding="utf-8"))
    pattern = document["$defs"]["package_path"]["pattern"]

    assert re.search(pattern, "world.xml")
    assert re.search(pattern, "nested/world.xml:ads") is None


def _resolve(schema: dict[str, Any] | bool, root: dict[str, Any]) -> dict[str, Any] | bool:
    if isinstance(schema, bool):
        return schema
    ref = schema.get("$ref")
    if not ref:
        return schema
    if not ref.startswith("#/"):
        raise SchemaError(f"unsupported reference: {ref}")
    current: Any = root
    for part in ref[2:].split("/"):
        current = current[part.replace("~1", "/").replace("~0", "~")]
    return cast(dict[str, Any] | bool, current)


def _is_valid(instance: Any, schema: dict[str, Any] | bool, root: dict[str, Any]) -> bool:
    try:
        _validate(instance, schema, root)
    except SchemaError:
        return False
    return True


def _validate(instance: Any, schema: dict[str, Any] | bool, root: dict[str, Any], path: str = "$") -> None:
    schema = _resolve(schema, root)
    if schema is False:
        raise SchemaError(f"{path}: false schema rejected value")
    if schema is True:
        return

    if "allOf" in schema:
        for subschema in schema["allOf"]:
            _validate(instance, subschema, root, path)
    if "if" in schema:
        branch = schema.get("then") if _is_valid(instance, schema["if"], root) else schema.get("else")
        if branch is not None:
            _validate(instance, branch, root, path)
    if "not" in schema and _is_valid(instance, schema["not"], root):
        raise SchemaError(f"{path}: not condition matched")
    if "anyOf" in schema and not any(_is_valid(instance, candidate, root) for candidate in schema["anyOf"]):
        raise SchemaError(f"{path}: expected at least one anyOf branch")
    if "oneOf" in schema:
        matches = sum(_is_valid(instance, candidate, root) for candidate in schema["oneOf"])
        if matches != 1:
            raise SchemaError(f"{path}: expected exactly one oneOf branch, got {matches}")

    expected_type = schema.get("type")
    if expected_type is not None:
        types = expected_type if isinstance(expected_type, list) else [expected_type]
        if not any(_type_matches(instance, kind) for kind in types):
            raise SchemaError(f"{path}: expected {expected_type}")
    if "const" in schema and instance != schema["const"]:
        raise SchemaError(f"{path}: expected const {schema['const']!r}")
    if "enum" in schema and instance not in schema["enum"]:
        raise SchemaError(f"{path}: expected enum value")

    if isinstance(instance, dict):
        for name in schema.get("required", []):
            if name not in instance:
                raise SchemaError(f"{path}: missing required property {name}")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            unknown = set(instance) - set(properties)
            if unknown:
                raise SchemaError(f"{path}: unknown properties {sorted(unknown)}")
        for name, value in instance.items():
            if name in properties:
                _validate(value, properties[name], root, f"{path}.{name}")

    if isinstance(instance, list):
        if len(instance) < schema.get("minItems", 0):
            raise SchemaError(f"{path}: too few items")
        if "maxItems" in schema and len(instance) > schema["maxItems"]:
            raise SchemaError(f"{path}: too many items")
        if schema.get("uniqueItems"):
            encoded = [json.dumps(item, sort_keys=True, separators=(",", ":")) for item in instance]
            if len(set(encoded)) != len(encoded):
                raise SchemaError(f"{path}: items are not unique")
        prefix_items = schema.get("prefixItems", [])
        for index, subschema in enumerate(prefix_items):
            if index >= len(instance):
                break
            _validate(instance[index], subschema, root, f"{path}[{index}]")
        if "items" in schema:
            for index in range(len(prefix_items), len(instance)):
                _validate(instance[index], schema["items"], root, f"{path}[{index}]")
        if "contains" in schema:
            count = sum(_is_valid(value, schema["contains"], root) for value in instance)
            if count < schema.get("minContains", 1):
                raise SchemaError(f"{path}: contains condition not met")

    if isinstance(instance, str):
        if "minLength" in schema and len(instance) < schema["minLength"]:
            raise SchemaError(f"{path}: value is too short")
        if "pattern" in schema and re.search(schema["pattern"], instance) is None:
            raise SchemaError(f"{path}: value does not match pattern")
    if isinstance(instance, (int, float)) and not isinstance(instance, bool):
        if "minimum" in schema and instance < schema["minimum"]:
            raise SchemaError(f"{path}: below minimum")
        if "exclusiveMinimum" in schema and instance <= schema["exclusiveMinimum"]:
            raise SchemaError(f"{path}: below exclusive minimum")
        if "maximum" in schema and instance > schema["maximum"]:
            raise SchemaError(f"{path}: above maximum")


def _type_matches(value: Any, kind: str) -> bool:
    if kind == "object":
        return isinstance(value, dict)
    if kind == "array":
        return isinstance(value, list)
    if kind == "string":
        return isinstance(value, str)
    if kind == "boolean":
        return isinstance(value, bool)
    if kind == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if kind == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if kind == "null":
        return value is None
    raise SchemaError(f"unsupported type: {kind}")


def _load_schema(name: str) -> dict[str, Any]:
    with SCHEMAS[name].open(encoding="utf-8-sig") as stream:
        schema = json.load(stream)
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    expected_id = "lingtu.sim.transport-intent.v1" if name == "transport" else f"lingtu.sim.{name}-plan.v1"
    assert schema["$id"] == expected_id
    return cast(dict[str, Any], schema)


def _load_package_schema(name: str) -> dict[str, Any]:
    with PACKAGE_SCHEMAS[name].open(encoding="utf-8-sig") as stream:
        schema = json.load(stream)
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["$id"] == f"lingtu.sim.{name}-package.v1"
    return cast(dict[str, Any], schema)


def _minimal_controller_package() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.controller-package.v1",
        "id": "legacy_controller",
        "version": "1.0.0",
        "kind": "controller",
        "adapter": {"plugin": "legacy", "abi": "legacy.v1"},
        "policy": {"runtime": "local", "artifact": "policy/model.bin", "manifest": "policy/manifest.json"},
        "timing": {"inference_hz": 50, "low_level_hz": 200},
        "robot_interface": {
            "requires_state": ["legacy_state"],
            "produces_command": {"type": "actuator_command"},
        },
        "actuators": {"channels": ["hip"]},
    }


def test_controller_package_schema_keeps_accepts_command_and_message_type_backward_compatible() -> None:
    schema = _load_package_schema("controller")
    payload = _minimal_controller_package()

    _validate(payload, schema, schema)

    payload["robot_interface"]["accepts_command"] = {"type": "base_velocity"}
    _validate(payload, schema, schema)


def test_controller_package_schema_rejects_non_auditable_command_calibration() -> None:
    schema = _load_package_schema("controller")
    payload = _minimal_controller_package()
    payload["command_calibration"] = {
        "schema": "lingtu.sim.controller-command-calibration.v1",
        "scope": "quadruped_him_observation_only",
        "provenance": "factory_park_turn_truth_qa_20260809",
        "external_yaw_cap_radps": 0.35,
        "policy_yaw_observation_gain": 1.2857142857142858,
        "policy_yaw_observation_limit_radps": 0.45,
        "leaves_base_twist_unchanged": True,
        "affected_axes": ["angular_z"],
    }

    with pytest.raises(SchemaError, match="expected object"):
        _validate(payload, schema, schema)


@pytest.mark.parametrize("axes", [["angular_z", "angular_z"], ["linear_x"], []])
def test_controller_package_schema_requires_exact_yaw_calibration_axis(axes: list[str]) -> None:
    schema = _load_package_schema("controller")
    payload = _minimal_controller_package()
    payload["command_calibration"] = {
        "schema": "lingtu.sim.controller-command-calibration.v1",
        "scope": "quadruped_him_observation_only",
        "provenance": {
            "source_id": "factory_park_turn_truth_qa_20260809",
            "audit_note": "Calibration source reference only; not qualification proof.",
            "qualification_claim": False,
        },
        "external_yaw_cap_radps": 0.35,
        "policy_yaw_observation_gain": 1.2857142857142858,
        "policy_yaw_observation_limit_radps": 0.45,
        "leaves_base_twist_unchanged": True,
        "affected_axes": axes,
    }

    with pytest.raises(SchemaError):
        _validate(payload, schema, schema)


def test_checked_in_sensor_rig_package_accepts_200_hz_imu() -> None:
    schema = json.loads((SCHEMA_ROOT / "sensor-rig.v1.json").read_text(encoding="utf-8"))
    package = yaml.safe_load(THUNDER_SENSOR_RIG_PACKAGE.read_text(encoding="utf-8"))

    imu = next(sensor for sensor in package["sensors"] if sensor["package"] == "imu@1.0.0")
    assert imu["frequency_hz"] == 200
    _validate(package, schema, schema)


def test_checked_in_robot_package_accepts_only_a_safe_source_mesh_root() -> None:
    schema = json.loads((SCHEMA_ROOT / "robot.v1.json").read_text(encoding="utf-8"))
    package = yaml.safe_load(THUNDER_ROBOT_PACKAGE.read_text(encoding="utf-8"))

    assert package["visual"]["source_mesh_root"] == "visual/meshes"
    _validate(package, schema, schema)

    package["visual"]["source_mesh_root"] = "../outside"
    with pytest.raises(SchemaError):
        _validate(package, schema, schema)


def _common(name: str) -> dict[str, Any]:
    schema_name = "transport-intent" if name == "transport" else f"{name}-plan"
    document: dict[str, Any] = {
        "schema": f"lingtu.sim.{schema_name}.v1",
        "session_id": SESSION_ID,
        "env": "sim",
    }
    if name == "scenario":
        document.update({"backend": "ue5", "model_generation": 1, "reset_generation": 2})
    else:
        document["backends"] = {"physics": "mujoco", "visual": "unreal"}
    return document


def _package_ref(package_id: str, kind: str, manifest: str) -> dict[str, str]:
    return {"id": package_id, "version": "1.0.0", "kind": kind, "manifest": manifest}


def _physics_plan() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.physics-plan.v1",
        "session_id": SESSION_ID,
        "composition": {
            "model_kind": "single_mjmodel",
            "composer": "mjs_attach_v1",
            "namespace_separator": "__",
            "state_authority": "mujoco",
        },
        "global_policy": {
            "owner": "world",
            "timestep_s": 0.002,
            "integrator": "rk4",
            "solver": "newton",
            "iterations": 100,
            "gravity_mps2": [0.0, 0.0, -9.81],
        },
        "world": {
            "package": _package_ref("open_field", "world", "sim/packages/worlds/open_field/world.package.yaml"),
            "mjcf": "sim/worlds/mujoco/open_field.xml",
        },
        "robots": [
            {
                "instance_id": "thunder_01",
                "namespace": "thunder_01",
                "package": _package_ref("thunderv4", "robot", "sim/robots/doso/thunder_v4/robot.package.yaml"),
                "controller": _package_ref(
                    "thunderv4_locomotion",
                    "controller",
                    "sim/controllers/doso/thunder_v4/locomotion/controller.package.yaml",
                ),
                "sensor_rig": _package_ref(
                    "thunderv4_navigation",
                    "sensor_rig",
                    "sim/sensor_rigs/doso/thunder_v4/navigation/sensor-rig.package.yaml",
                ),
                "semantic": {"class": "quadruped"},
                "spawn": {"position_m": [0.0, 0.0, 0.0], "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]},
                "model": {
                    "mjcf": "sim/robots/doso/thunder_v4/mjcf/thunderv4.xml",
                    "attach_root": "base_link",
                    "root_joint": "floating_base_joint",
                    "initial_keyframe": "v4_nominal_stand",
                },
                "frames": [{"name": "base_link", "role": "body"}],
            }
        ],
    }


def _visual_plan() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.visual-plan.v1",
        "session_id": SESSION_ID,
        "backends": {"physics": "mujoco", "visual": "unreal"},
        "coordinate_system": {
            "source": "mujoco_rh_z_up_m",
            "target": "unreal_lh_z_up_cm",
            "position_scale": 100.0,
            "axis_mapping": ["x", "-y", "z"],
            "quaternion_order": "wxyz",
        },
        "binding_policy": {"missing_asset": "fail", "data_asset_is_projection": True},
        "world": {
            "package": _package_ref("open_field", "world", "sim/packages/worlds/open_field/world.package.yaml"),
            "binding": "WorldVisual:OpenField",
            "level": "/Game/RobotSim/Maps/ThunderV4_RuntimePreview",
        },
        "robots": [
            {
                "instance_id": "thunder_01",
                "namespace": "thunder_01",
                "package": _package_ref("thunderv4", "robot", "sim/robots/doso/thunder_v4/robot.package.yaml"),
                "binding": "RobotVisual:ThunderV4",
                "projection": {
                    "schema": "lingtu.sim.robot-visual-projection.v1",
                    "path": "sim/robots/doso/thunder_v4/visual/robot.visual-projection.json",
                },
                "spawn": {"position_m": [0.0, 0.0, 0.0], "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]},
            }
        ],
    }


def _sensor_plan() -> dict[str, Any]:
    document = _common("sensor")
    document.update(
        {
            "streams": {
                "rgb": [
                    {
                        "instance_id": "thunder_01",
                        "sensor_id": "thunder_01.front_rgb",
                        "owner": "visual",
                        "source": "unreal_camera",
                        "frame_id": "thunder_01/front_camera",
                        "rate_hz": 30,
                        "transport": "camera_shm",
                        "message_type": "lingtu.dds.Image",
                        "width": 1280,
                        "height": 720,
                        "encoding": "rgb8",
                    }
                ],
                "depth": [
                    {
                        "instance_id": "thunder_01",
                        "sensor_id": "thunder_01.front_depth",
                        "owner": "visual",
                        "source": "unreal_camera",
                        "frame_id": "thunder_01/front_camera",
                        "rate_hz": 30,
                        "transport": "camera_shm",
                        "message_type": "lingtu.dds.Image",
                        "width": 640,
                        "height": 480,
                        "encoding": "16UC1",
                        "requested_encoding": "32FC1",
                        "depth_scale": 0.001,
                        "unit": "m",
                    }
                ],
                "imu": [
                    {
                        "instance_id": "thunder_01",
                        "sensor_id": "thunder_01.imu",
                        "owner": "physics",
                        "source": "mujoco_sensor",
                        "frame_id": "thunder_01/imu",
                        "rate_hz": 200,
                        "transport": "typed_dds",
                        "message_type": "lingtu.dds.Imu",
                        "fields": ["orientation", "angular_velocity", "linear_acceleration"],
                    }
                ],
                "mid360": [
                    {
                        "instance_id": "thunder_01",
                        "sensor_id": "thunder_01.mid360",
                        "owner": "physics",
                        "source": "mujoco_livox_model",
                        "frame_id": "thunder_01/lidar1_link",
                        "raycast_frame_stable_id": "thunder_01/lidar1_link_site",
                        "rate_hz": 10,
                        "transport": "typed_dds",
                        "message_type": "lingtu.dds.LivoxFrame",
                        "point_fields": ["x", "y", "z", "offset_time_ns", "reflectivity", "tag", "line"],
                        "offset_time_unit": "ns",
                        "line_semantics": "livox_channel",
                    }
                ],
                "truth_odom": [
                    {
                        "instance_id": "thunder_01",
                        "sensor_id": "thunder_01.truth_odom",
                        "owner": "physics",
                        "source": "mujoco_truth",
                        "frame_id": "thunder_01/base_link",
                        "rate_hz": 100,
                        "transport": "typed_dds",
                        "message_type": "lingtu.dds.Odometry",
                        "estimator_input": False,
                    }
                ],
            },
        }
    )
    return document


def _control_plan() -> dict[str, Any]:
    document = _common("control")
    document.update(
        {
            "controllers": [
                {
                    "instance_id": "thunder_01",
                    "controller_id": "thunder_01.thunderv4_locomotion",
                    "package": {
                        "id": "thunderv4_locomotion",
                        "version": "1.0.0",
                        "kind": "controller",
                        "manifest": "sim/controllers/doso/thunder_v4/locomotion/controller.package.yaml",
                    },
                    "adapter": {"plugin": "quadruped_him", "abi": "lingtu.sim.controller-adapter.v1"},
                    "policy": {
                        "runtime": "torchscript",
                        "artifact": "sim/controllers/doso/thunder_v4/locomotion/policy/policy.pt",
                        "manifest": "sim/controllers/doso/thunder_v4/locomotion/policy/policy_manifest.json",
                    },
                    "timing": {"inference_hz": 50, "low_level_hz": 500},
                    "state_channels": [
                        "joint_position",
                        "joint_velocity",
                        "base_angular_velocity",
                        "projected_gravity",
                    ],
                    "command_channels": ["thunder_01.control.base_twist", "thunder_01.control.joint_torque"],
                    "actuator_channels": ["FR_hip_joint"],
                }
            ],
            "command_channels": [
                {
                    "channel_id": "thunder_01.control.base_twist",
                    "direction": "subscribe",
                    "owner": "simulation",
                    "source": "dds_base_twist/thunder_01",
                    "transport": "typed_dds",
                    "message_type": "lingtu.dds.FinalVelocityCommand",
                    "command_type": "base_twist",
                    "target": "base",
                },
                {
                    "channel_id": "thunder_01.control.joint_torque",
                    "direction": "publish",
                    "owner": "physics",
                    "source": "thunder_01.thunderv4_locomotion",
                    "transport": "in_process",
                    "message_type": "lingtu.sim.joint-torque.v1",
                    "command_type": "joint_torque",
                    "target": "actuators",
                },
            ],
            "stale_stop_authority": {
                "owner": "simulation",
                "hardware_forwarding": False,
                "safe_stop_on_stale": True,
                "stale_timeout_ms": 100,
            },
        }
    )
    return document


def _scenario_plan() -> dict[str, Any]:
    document = _common("scenario")
    document.update(
        {
            "package": _package_ref(
                "open_field_pedestrian_crossing",
                "scenario",
                "sim/packages/scenarios/open_field_pedestrian_crossing/scenario.package.yaml",
            ),
            "seed": 7,
            "clock": {"unit": "ns", "source": "mujoco_sim_time", "sim_time_ns": 0},
            "authority_policy": {
                "robot_physics_owner": "mujoco",
                "dynamic_behavior_owner": "scenario",
                "visual_animation_owner": "ue_animation",
            },
            "entities": [
                {
                    "entity_id": "thunder_01",
                    "entity_type": "robot",
                    "authority": "mujoco",
                    "source_epoch": 1,
                    "initial_transform": {"position_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
                    "physics_proxy": "mujoco",
                    "semantic_class": "quadruped",
                },
                {
                    "entity_id": "pedestrian_01",
                    "entity_type": "pedestrian",
                    "authority": "scenario",
                    "source_epoch": 1,
                    "initial_transform": {"position_m": [4, 1, 0], "quaternion_wxyz": [1, 0, 0, 0]},
                    "physics_proxy": "kinematic",
                    "behavior": {
                        "profile": "walk_crossing",
                        "seed": 9,
                        "parameters": {"speed_mps": 1.2, "duration_s": 5.0},
                    },
                    "semantic_class": "pedestrian",
                },
            ],
        }
    )
    return document


def _transport_intent() -> dict[str, Any]:
    document = _common("transport")
    document.update(
        {
            "channels": [
                {
                    "channel_id": "runtime.truth_snapshot",
                    "direction": "publish",
                    "owner": "physics",
                    "source": "mujoco_runtime",
                    "transport": "udp_loopback_json",
                    "delivery": "latest_wins",
                    "message_type": "lingtu.sim.truth-snapshot.v1",
                    "payload_role": "truth_snapshot",
                    "contract": "lingtu.sim.truth-snapshot.v1",
                    "frame_policy": "sim_time",
                },
                {
                    "channel_id": "camera.rgb",
                    "direction": "publish",
                    "owner": "visual",
                    "source": "unreal_camera",
                    "transport": "camera_shm",
                    "delivery": "latest_wins",
                    "message_type": "lingtu.dds.Image",
                    "payload_role": "rgb",
                    "contract": "camera_frame_descriptor.v1",
                    "frame_policy": "source_time",
                },
                {
                    "channel_id": "lidar.raw_frame",
                    "direction": "publish",
                    "owner": "physics",
                    "source": "mujoco_livox_model",
                    "transport": "typed_dds",
                    "delivery": "ordered_no_drop",
                    "message_type": "lingtu.dds.LivoxFrame",
                    "payload_role": "mid360_livox",
                    "contract": "field_dds_v1",
                    "frame_policy": "source_time",
                },
                {
                    "channel_id": "control.command",
                    "direction": "subscribe",
                    "owner": "simulation",
                    "source": "navd",
                    "transport": "typed_dds",
                    "delivery": "reliable",
                    "message_type": "lingtu.dds.FinalVelocityCommand",
                    "payload_role": "control",
                    "contract": "field_dds_v1",
                    "frame_policy": "source_time",
                },
            ],
            "allocation_boundary": {
                "owner": "RunAllocation",
                "runtime_values_external": True,
            },
        }
    )
    return document


POSITIVES = {
    "physics": _physics_plan,
    "visual": _visual_plan,
    "sensor": _sensor_plan,
    "control": _control_plan,
    "scenario": _scenario_plan,
    "transport": _transport_intent,
}
DETERMINISTIC_PLAN_FACTORIES = {
    "physics": _physics_plan,
    "visual": _visual_plan,
    "sensor": _sensor_plan,
    "control": _control_plan,
    "transport": _transport_intent,
}
BACKEND_ROLE_PLAN_FACTORIES = {
    "visual": _visual_plan,
    "sensor": _sensor_plan,
    "control": _control_plan,
    "transport": _transport_intent,
}
RUNTIME_MUTABLE_ROOT_FIELDS = {
    "model_generation": 1,
    "reset_generation": 2,
    "sequence": 3,
    "physics_step": 4,
    "sim_time": 4.0,
    "sim_time_ns": 4_000_000,
    "clock": {"unit": "ns", "source": "mujoco_sim_time", "sim_time_ns": 4_000_000},
    "commands": [],
    "command_payload": {"linear": {"x": 0.2, "y": 0.0, "z": 0.0}},
    "apply_time_ns": 4_000_000,
}


def test_all_gate0_schemas_load_and_positive_examples_validate() -> None:
    for name, factory in POSITIVES.items():
        schema = _load_schema(name)
        _validate(factory(), schema, schema)


@pytest.mark.parametrize(
    "level",
    [
        "",
        "   ",
        "/Engine/Maps/Forbidden",
        "/Game/",
        "/Game/   ",
        " /Game/Maps/OpenField",
        "/Game/Maps/OpenField ",
        "/Game/Maps\\OpenField",
        "/Game/Maps//OpenField",
        "/Game/./Map",
        "/Game/../Map",
        "/Game/Maps/./OpenField",
        "/Game/Maps/../OpenField",
        "/Game/Maps/OpenField/",
    ],
)
def test_visual_plan_rejects_invalid_world_level(level: str) -> None:
    schema = _load_schema("visual")
    document = _visual_plan()
    document["world"]["level"] = level

    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


@pytest.mark.parametrize(
    "level",
    ["/Game/A", "/Game/Maps/OpenField", "/Game/RobotSim/Maps/ThunderV4_RuntimePreview"],
)
def test_visual_plan_accepts_strict_world_level(level: str) -> None:
    schema = _load_schema("visual")
    document = _visual_plan()
    document["world"]["level"] = level

    _validate(document, schema, schema)


def test_visual_plan_accepts_a_world_projection_reference() -> None:
    schema = _load_schema("visual")
    document = _visual_plan()
    document["world"]["projection"] = {
        "schema": "lingtu.sim.world-visual-projection.v1",
        "path": "sim/packages/worlds/field/1.0.0/visual/world.visual-projection.json",
    }

    _validate(document, schema, schema)


def test_visual_plan_accepts_a_generic_scenario_entity_projection() -> None:
    schema = _load_schema("visual")
    document = _visual_plan()
    document["scenario_entities"] = [
        {
            "entity_id": "pedestrian_01",
            "namespace": "pedestrian_01",
            "package": _package_ref(
                "open_field_pedestrian_crossing",
                "scenario",
                "sim/packages/scenarios/open_field_pedestrian_crossing/scenario.package.yaml",
            ),
            "binding": "EntityVisual:PedestrianCapsule",
            "projection": {
                "schema": "lingtu.sim.entity-visual-projection.v1",
                "path": (
                    "sim/packages/scenarios/open_field_pedestrian_crossing/"
                    "visual/entity.visual-projection.json"
                ),
            },
            "spawn": {
                "position_m": [4.0, -6.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
        }
    ]

    _validate(document, schema, schema)


def test_catalog_resolver_physics_and_visual_outputs_validate() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(THUNDER_UNREAL_SESSION)
    plans = {"physics": resolved.physics_plan, "visual": resolved.visual_plan}

    assert "backends" not in resolved.physics_plan
    assert "model_generation" not in resolved.physics_plan
    assert "reset_generation" not in resolved.physics_plan
    assert resolved.visual_plan["backends"] == {"physics": "mujoco", "visual": "unreal"}
    for name, document in plans.items():
        schema = _load_schema(name)
        _validate(document, schema, schema)


def test_catalog_resolver_depth_plan_preserves_requested_encoding_and_compiles_wire_format() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(THUNDER_UNREAL_SESSION)
    depth_stream = resolved.sensor_plan["streams"]["depth"][0]

    assert depth_stream["encoding"] == "16UC1"
    assert depth_stream["requested_encoding"] == "32FC1"
    assert depth_stream["depth_scale"] == 0.001
    schema = _load_schema("sensor")
    _validate(resolved.sensor_plan, schema, schema)


def test_catalog_resolver_selected_scenario_output_validates() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(THUNDER_PEDESTRIAN_CROSSING_SESSION)

    assert resolved.scenario_plan is not None
    schema = _load_schema("scenario")
    _validate(resolved.scenario_plan, schema, schema)


def test_deterministic_plans_use_explicit_backend_roles() -> None:
    for name, factory in BACKEND_ROLE_PLAN_FACTORIES.items():
        schema = _load_schema(name)
        document = factory()
        assert document["backends"] == {"physics": "mujoco", "visual": "unreal"}
        _validate(document, schema, schema)


def test_physics_plan_does_not_duplicate_backend_selection() -> None:
    schema = _load_schema("physics")
    document = _physics_plan()
    assert "backends" not in document
    _validate(document, schema, schema)


def test_sensor_plan_allows_empty_stream_classes_for_headless_or_sparse_rigs() -> None:
    schema = _load_schema("sensor")
    document = _sensor_plan()
    document["backends"]["visual"] = None
    for streams in document["streams"].values():
        streams.clear()
    _validate(document, schema, schema)


def test_transport_intent_accepts_runtime_truth_snapshot_channel() -> None:
    schema = _load_schema("transport")
    document = _transport_intent()
    assert document["channels"][0] == {
        "channel_id": "runtime.truth_snapshot",
        "direction": "publish",
        "owner": "physics",
        "source": "mujoco_runtime",
        "transport": "udp_loopback_json",
        "delivery": "latest_wins",
        "message_type": "lingtu.sim.truth-snapshot.v1",
        "payload_role": "truth_snapshot",
        "contract": "lingtu.sim.truth-snapshot.v1",
        "frame_policy": "sim_time",
    }
    _validate(document, schema, schema)


@pytest.mark.parametrize("name", sorted(SCHEMAS))
def test_all_gate0_schemas_are_strict_at_the_root(name: str) -> None:
    schema = _load_schema(name)
    document = POSITIVES[name]()
    document["unknown_field"] = True
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


@pytest.mark.parametrize("name", sorted(DETERMINISTIC_PLAN_FACTORIES))
@pytest.mark.parametrize("field,value", sorted(RUNTIME_MUTABLE_ROOT_FIELDS.items()))
def test_deterministic_plans_reject_runtime_mutable_fields_at_the_root(name: str, field: str, value: Any) -> None:
    schema = _load_schema(name)
    document = DETERMINISTIC_PLAN_FACTORIES[name]()
    document[field] = value
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


@pytest.mark.parametrize("name", sorted(DETERMINISTIC_PLAN_FACTORIES))
def test_deterministic_plans_reject_legacy_mutually_exclusive_backend_field(name: str) -> None:
    schema = _load_schema(name)
    document = DETERMINISTIC_PLAN_FACTORIES[name]()
    document["backend"] = "ue5"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


@pytest.mark.parametrize("name", sorted(BACKEND_ROLE_PLAN_FACTORIES))
def test_deterministic_plans_reject_unreal_as_physics_backend(name: str) -> None:
    schema = _load_schema(name)
    document = DETERMINISTIC_PLAN_FACTORIES[name]()
    document["backends"]["physics"] = "unreal"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_sensor_plan_rejects_invalid_authoritative_frequencies() -> None:
    schema = _load_schema("sensor")
    document = _sensor_plan()
    document["streams"]["imu"][0]["rate_hz"] = 500
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_sensor_plan_rejects_runtime_capture_clock_on_streams() -> None:
    schema = _load_schema("sensor")
    document = _sensor_plan()
    document["streams"]["rgb"][0]["capture_clock"] = "sim_time"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_sensor_plan_rejects_incomplete_livox_fields() -> None:
    schema = _load_schema("sensor")
    document = _sensor_plan()
    document["streams"]["mid360"][0]["point_fields"] = ["x", "y", "z", "offset_time_ns"]
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_sensor_plan_rejects_nonboolean_navigation_fixture_raw_overlay() -> None:
    schema = _load_schema("sensor")
    document = _sensor_plan()
    document["streams"]["mid360"][0]["navigation_fixture_raw_overlay"] = "false"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_scenario_plan_rejects_unknown_entity_authority() -> None:
    schema = _load_schema("scenario")
    document = _scenario_plan()
    document["entities"][0]["authority"] = "chaos"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_control_plan_rejects_actual_command_payloads_on_channels() -> None:
    schema = _load_schema("control")
    document = _control_plan()
    document["command_channels"][0]["payload"] = {"linear": {"x": 0.2, "y": 0, "z": 0}}
    document["command_channels"][0]["command_sequence"] = 3
    document["command_channels"][0]["apply_time_ns"] = 4_000_000
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_transport_intent_rejects_runtime_port_or_shm_allocation() -> None:
    schema = _load_schema("transport")
    document = _transport_intent()
    document["ports"] = {"dds": 7400}
    document["shm_name"] = "run-001-camera"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def test_ue5_is_only_valid_inside_sim_env_for_unchanged_scenario_contract() -> None:
    schema = _load_schema("scenario")
    document = _scenario_plan()
    document["env"] = "real"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


@pytest.mark.parametrize("name", sorted(DETERMINISTIC_PLAN_FACTORIES))
def test_deterministic_plans_are_sim_env_contracts(name: str) -> None:
    schema = _load_schema(name)
    document = DETERMINISTIC_PLAN_FACTORIES[name]()
    document["env"] = "real"
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)
