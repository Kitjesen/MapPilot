"""Opt-in measurement catalog compilation, without engines or default assets."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import pytest
import yaml
from sim.catalog import CatalogError, CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
LEGACY_STREAMS = {"rgb", "depth", "imu", "mid360", "truth_odom"}
MESSAGES = {"thermal": "ThermalImage", "acoustic": "AcousticBlock", "semantic": "SemanticImage"}
pytestmark = pytest.mark.sim


@pytest.fixture
def catalog(tmp_path):
    packages = tmp_path / "packages"
    packages.mkdir()
    robot = {
        "schema": "lingtu.sim.robot-package.v1", "id": "fixture_robot", "version": "1.0.0", "kind": "robot",
        "physics": {"mjcf": "robot.xml", "attach_root": "base_link", "root_joint": "root_joint",
                    "global_options": "inherit_session"},
        "visual": {"binding": "RobotVisual:Fixture", "projection": "robot.visual.json"},
        "semantic": {"class": "quadruped"},
        "frames": [{"name": "base_link", "role": "body"},
                   {"name": "payload_top", "role": "sensor_mount", "parent_frame": "base_link",
                    "extrinsic": {"position_m": [0, 0, 0.14], "quaternion_wxyz": [1, 0, 0, 0]}},
                   {"name": "arm", "role": "body", "parent_frame": "base_link",
                    "extrinsic": {"position_m": [1, 2, 3], "quaternion_wxyz": [1, 0, 0, 0]}},
                   {"name": "tip", "role": "sensor_origin", "parent_frame": "arm",
                    "extrinsic": {"position_m": [0, 0, 0.2], "quaternion_wxyz": [1, 0, 0, 0]}}],
        "interfaces": {"state": ["lingtu.sim.base-state.v1"], "command": ["lingtu.sim.base-velocity.v1"]},
        "defaults": {"controller": None, "sensor_rig": None}, "declared_capabilities": {},
    }
    (packages / "robot.xml").write_text(
        '<mujoco><worldbody><body name="base_link"><joint name="root_joint"/><geom name="robot_geom"/>'
        '<site name="payload_top"/><body name="arm"><site name="tip"/></body></body></worldbody></mujoco>',
        encoding="utf-8",
    )
    (packages / "robot.package.yaml").write_text(yaml.safe_dump(robot), encoding="utf-8")
    (packages / "robot.visual.json").write_text(json.dumps({
        "schema": "lingtu.sim.robot-visual-projection.v1", "binding": "RobotVisual:Fixture",
        "package": {"id": "fixture_robot", "version": "1.0.0"}, "mjcf": {}, "components": [{}],
    }), encoding="utf-8")
    scene = {
        "thermal_surfaces": {"target": {"temperature_k": 310.0, "emissivity": 0.9}},
        "semantic_labels": {"target": {"class_id": 2, "instance_id": 7}},
        "acoustic_sources": [{
            "source_id": "motor", "body_name": "machine", "pressure_pa": [0.0, 0.1, -0.1],
            "sample_rate_hz": 1000, "start_sim_time_ns": 0, "reference_distance_m": 1.0,
            "local_offset_m": [0, 0, 0.1],
        }],
    }
    world = {
        "schema": "lingtu.sim.world-package.v1", "id": "measurement_fixture", "version": "1.0.0",
        "kind": "world", "physics": {"mjcf": "world.xml", "global_policy": {
            "timestep_s": 0.002, "integrator": "rk4", "solver": "newton", "iterations": 100,
            "gravity_mps2": [0, 0, -9.81],
        }}, "visual": {"binding": "WorldVisual:Fixture", "level": "/Game/Fixture"}, "entities": [],
    }
    (packages / "world.xml").write_text(
        '<mujoco><option timestep="0.002" integrator="RK4" solver="Newton" iterations="100" '
        'gravity="0 0 -9.81"/><worldbody><body name="machine"><geom name="target"/></body>'
        '</worldbody></mujoco>', encoding="utf-8",
    )
    (packages / "world.package.yaml").write_text(yaml.safe_dump(world), encoding="utf-8")
    sensors = []
    for kind, message in MESSAGES.items():
        measurement = {"body_frame_id": "base_link"}
        if kind == "acoustic":
            measurement["microphone"] = {
                "sample_rate_hz": 1000, "block_samples": 100, "channel_offsets_m": [[0, 0, 0]],
            }
        else:
            measurement["camera"] = {
                "model": "pinhole", "width": 2, "height": 1,
                "fx_px": 2, "fy_px": 2, "cx_px": 1, "cy_px": 0.5,
            }
        if kind == "thermal":
            measurement["thermal"] = {"calibration_id": "fixture", "background_temperature_k": 293.15}
        package = {
            "schema": "lingtu.sim.sensor-package.v1", "id": f"fixture_{kind}", "version": "1.0.0",
            "kind": "sensor", "sensor_type": kind, "outputs": [kind], "timing": {"rate_hz": 10},
            "interface": {"transport": "typed_dds", "message_type": f"lingtu.dds.{message}"},
            "configuration": {"measurement": measurement},
        }
        (packages / f"{kind}.package.yaml").write_text(yaml.safe_dump(package), encoding="utf-8")
        sensors.append({
            "id": kind, "package": f"fixture_{kind}@1.0.0", "parent_frame": "payload_top",
            "runtime": {"owner": "physics", "source": "mujoco_measurement"},
        })
    rig = {"schema": "lingtu.sim.sensor-rig-package.v1", "id": "fixture_rig", "version": "1.0.0",
           "kind": "sensor_rig", "sensors": sensors}
    (packages / "rig.package.yaml").write_text(yaml.safe_dump(rig), encoding="utf-8")
    session = yaml.safe_load((REPO_ROOT / "sim/sessions/examples/thunderv4_gnss/session.yaml").read_text())
    session["world"] = "measurement_fixture@1.0.0"
    session["robots"][0]["package"] = "fixture_robot@1.0.0"
    session["robots"][0]["sensor_rig"] = "fixture_rig@1.0.0"
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(session), encoding="utf-8")
    resolver = CatalogResolver(tmp_path, (packages,))
    resolver.find_package("measurement_fixture@1.0.0").data["measurements"] = scene
    return resolver, path


def test_opt_in_packages_compile_measurement_streams_and_transport(catalog):
    resolver, path = catalog
    resolved = resolver.resolve(path)
    assert set(resolved.sensor_plan["streams"]) == LEGACY_STREAMS | MESSAGES.keys()
    assert resolved.sensor_plan["scene_measurements"] == resolver.find_package("measurement_fixture@1.0.0").data["measurements"]
    for kind, message in MESSAGES.items():
        stream, = resolved.sensor_plan["streams"][kind]
        assert stream["message_type"] == f"lingtu.dds.{message}"
        assert stream["measurement"]["body_frame_id"] == "thunder_01/base_link"
        assert stream["measurement"]["mount_pose"] == {
            "position_m": [0, 0, 0.14], "orientation_wxyz": [1, 0, 0, 0],
        }
        channel = next(c for c in resolved.transport_intent["channels"] if c["payload_role"] == kind)
        assert channel["source"] == "mujoco_measurement"
        assert channel["transport"] == "typed_dds"
        assert channel["message_type"] == stream["message_type"]


def test_world_manifest_owns_measurement_declarations(catalog):
    resolver, path = catalog
    world = resolver.find_package("measurement_fixture@1.0.0")
    world.manifest_path.write_text(yaml.safe_dump(world.data), encoding="utf-8")
    reloaded = CatalogResolver(resolver.repo_root, resolver.catalog_roots)
    assert reloaded.resolve(path).sensor_plan["scene_measurements"] == world.data["measurements"]


def test_looping_world_source_survives_catalog_reload_and_plan_compilation(catalog):
    resolver, path = catalog
    world = resolver.find_package("measurement_fixture@1.0.0")
    world.data["measurements"]["acoustic_sources"][0]["loop"] = True
    world.manifest_path.write_text(yaml.safe_dump(world.data), encoding="utf-8")
    reloaded = CatalogResolver(resolver.repo_root, resolver.catalog_roots)
    assert reloaded.resolve(path).sensor_plan["scene_measurements"]["acoustic_sources"][0]["loop"] is True


@pytest.mark.parametrize("scene", [
    {"typo": {}},
    {"thermal_surfaces": {"missing": {"temperature_k": 300, "emissivity": 1}}},
    {"thermal_surfaces": {"robot_geom": {"temperature_k": 300, "emissivity": 1}}},
    {"thermal_surfaces": {" target": {"temperature_k": 300, "emissivity": 1}}},
    {"thermal_surfaces": {"target": {"temperature_k": 0, "emissivity": 1}}},
    {"thermal_surfaces": {"target": {"temperature_k": float("nan"), "emissivity": 1}}},
    {"thermal_surfaces": {"target": {"temperature_k": 300, "emissivity": True}}},
    {"thermal_surfaces": {"target": {"temperature_k": 300, "emissivity": 1.1}}},
    {"thermal_surfaces": {"target": {"temperature_k": 300, "emissivity": 1, "typo": 0}}},
    {"semantic_labels": {"target": {"class_id": 0, "instance_id": 1}}},
    {"semantic_labels": {"target": {"class_id": 1, "instance_id": 2**31}}},
    {"semantic_labels": {"target": {"class_id": True, "instance_id": 1}}},
    {"semantic_labels": {"machine": {"class_id": 1, "instance_id": 1}}},
    {"acoustic_sources": {}},
    {"acoustic_sources": [{"source_id": "motor"}]},
])
def test_invalid_world_measurements_are_rejected_at_load(catalog, scene):
    resolver, _ = catalog
    world = resolver.find_package("measurement_fixture@1.0.0")
    world.data["measurements"] = scene
    world.manifest_path.write_text(yaml.safe_dump(world.data), encoding="utf-8")
    with pytest.raises(CatalogError, match=r"measurement|thermal|semantic|acoustic"):
        CatalogResolver(resolver.repo_root, resolver.catalog_roots)


@pytest.mark.parametrize(("kind", "field", "value"), [
    ("thermal", "camera", {"model": "pinhole", "width": 1, "height": 1, "fx_px": -1,
                           "fy_px": 1, "cx_px": 0, "cy_px": 0}),
    ("thermal", "camera", {"model": "unknown"}),
    ("thermal", "thermal", {"background_temperature_k": 300}),
    ("thermal", "thermal", {"calibration_id": "fixture", "background_temperature_k": -1}),
    ("thermal", "noise", {"unit": "Pa"}),
    ("thermal", "noise", {"unit": "K", "seed": 2**64}),
    ("thermal", "noise", {"unit": "K", "enabled": 1}),
    ("thermal", "gimbal", {"pan_deg": 181}),
    ("thermal", "max_distance_m", 0.001),
    ("thermal", "typo", 0),
    ("semantic", "thermal", {}),
    ("semantic", "noise", {"unit": "label", "enabled": True}),
    ("acoustic", "microphone", {"sample_rate_hz": 1000, "block_samples": 99,
                               "channel_offsets_m": [[0, 0, 0]]}),
    ("acoustic", "microphone", {"sample_rate_hz": True, "block_samples": 100,
                               "channel_offsets_m": [[0, 0, 0]]}),
    ("acoustic", "camera", {}),
])
def test_catalog_rejects_invalid_factory_configuration(catalog, kind, field, value):
    resolver, path = catalog
    resolver.find_package(f"fixture_{kind}@1.0.0").data["configuration"]["measurement"][field] = value
    with pytest.raises(CatalogError, match=r"measurement|camera|thermal|noise|gimbal|microphone|distance"):
        resolver.resolve(path)


def _validate(document, name):
    jsonschema = pytest.importorskip("jsonschema")
    referencing = pytest.importorskip("referencing")
    schemas = [json.loads(path.read_text(encoding="utf-8"))
               for path in (REPO_ROOT / "sim/contracts/schemas").glob("*.json")]
    registry = referencing.Registry().with_resources(
        (schema["$id"], referencing.Resource.from_contents(schema)) for schema in schemas
    )
    schema = next(schema for schema in schemas if schema["$id"] == {
        "sensor-plan": "lingtu.sim.sensor-plan.v1", "world": "lingtu.sim.world-package.v1",
        "measurement-scene": "lingtu.sim.measurement-scene.v1", "transport-intent": "lingtu.sim.transport-intent.v1",
    }[name])
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema, registry=registry).validate(document)


def test_measurement_plan_and_world_pass_strict_schemas(catalog):
    resolver, path = catalog
    resolved = resolver.resolve(path)
    _validate(resolved.sensor_plan, "sensor-plan")
    _validate(resolver.find_package("measurement_fixture@1.0.0").data, "world")


def test_measurement_transport_intent_schema_owner_integration(catalog):
    resolver, path = catalog
    _validate(resolver.resolve(path).transport_intent, "transport-intent")


@pytest.mark.parametrize(("field", "value"), [
    ("owner", "visual"), ("source", "mujoco_truth"),
    ("transport", "in_process"), ("message_type", "lingtu.dds.Image"),
])
@pytest.mark.parametrize("kind", MESSAGES)
def test_measurement_route_is_exact(catalog, kind, field, value):
    resolver, path = catalog
    if field in {"owner", "source"}:
        rig = resolver.find_package("fixture_rig@1.0.0").data
        next(sensor for sensor in rig["sensors"] if sensor["id"] == kind)["runtime"][field] = value
    else:
        resolver.find_package(f"fixture_{kind}@1.0.0").data["interface"][field] = value
    with pytest.raises(CatalogError, match="requires physics/mujoco_measurement/typed_dds"):
        resolver.resolve(path)


@pytest.mark.parametrize("body", [None, "payload_top", "missing", "other/base_link", " base_link", "arm"])
def test_measurement_body_must_be_an_actual_ancestor(catalog, body):
    resolver, path = catalog
    config = resolver.find_package("fixture_thermal@1.0.0").data["configuration"]["measurement"]
    if body is None:
        del config["body_frame_id"]
    else:
        config["body_frame_id"] = body
    with pytest.raises(CatalogError, match="body_frame_id"):
        resolver.resolve(path)


@pytest.mark.parametrize("body", ["base_link", "thunder_01/base_link", "arm", "thunder_01/arm"])
@pytest.mark.parametrize("override", [False, True])
def test_measurement_mount_composes_ancestry_once(catalog, body, override):
    resolver, path = catalog
    robot = resolver.find_package("fixture_robot@1.0.0").data
    arm = next(frame for frame in robot["frames"] if frame["name"] == "arm")
    arm["extrinsic"]["quaternion_wxyz"] = [math.sqrt(0.5), 0, math.sqrt(0.5), 0]
    rig = resolver.find_package("fixture_rig@1.0.0").data["sensors"][0]
    rig["parent_frame"] = "tip"
    if override:
        rig["extrinsic"] = {"position_m": [0, 0, 0.4], "quaternion_wxyz": [1, 0, 0, 0]}
    config = resolver.find_package("fixture_thermal@1.0.0").data["configuration"]["measurement"]
    config["body_frame_id"] = body
    measurement = resolver.resolve(path).sensor_plan["streams"]["thermal"][0]["measurement"]
    expected = [1.4 if override else 1.2, 2, 3] if body.endswith("base_link") else [0, 0, 0.4 if override else 0.2]
    assert measurement["mount_pose"]["position_m"] == pytest.approx(expected)
    assert measurement["body_frame_id"] == f"thunder_01/{body.removeprefix('thunder_01/')}"


def test_actual_body_mount_does_not_apply_body_transform_twice(catalog):
    resolver, path = catalog
    resolver.find_package("fixture_rig@1.0.0").data["sensors"][0]["parent_frame"] = "arm"
    resolver.find_package("fixture_thermal@1.0.0").data["configuration"]["measurement"]["body_frame_id"] = "arm"
    measurement = resolver.resolve(path).sensor_plan["streams"]["thermal"][0]["measurement"]
    assert measurement["mount_pose"] == {"position_m": [0, 0, 0], "orientation_wxyz": [1, 0, 0, 0]}


@pytest.mark.parametrize("where", ["package", "rig"])
def test_measurement_mount_pose_is_catalog_owned(catalog, where):
    resolver, path = catalog
    package_config = resolver.find_package("fixture_thermal@1.0.0").data["configuration"]
    config = package_config
    if where == "rig":
        config = copy.deepcopy(package_config)
        resolver.find_package("fixture_rig@1.0.0").data["sensors"][0]["configuration"] = config
    config["measurement"]["mount_pose"] = {"position_m": [1, 2, 3]}
    with pytest.raises(CatalogError, match=r"mount_pose.*catalog-owned"):
        resolver.resolve(path)


def test_measurement_world_is_explicit_but_may_be_empty(catalog):
    resolver, path = catalog
    world = resolver.find_package("measurement_fixture@1.0.0").data
    del world["measurements"]
    with pytest.raises(CatalogError, match="explicit world measurements"):
        resolver.resolve(path)
    world["measurements"] = {}
    resolved = resolver.resolve(path)
    assert resolved.sensor_plan["scene_measurements"] == {}
    _validate(resolved.sensor_plan, "sensor-plan")


def test_disabled_streams_emit_no_measurement_keys(catalog):
    resolver, path = catalog
    session = yaml.safe_load(path.read_text())
    session["runtime"]["required_bindings"] = ["physics"]
    path.write_text(yaml.safe_dump(session), encoding="utf-8")
    del resolver.find_package("measurement_fixture@1.0.0").data["measurements"]
    resolved = resolver.resolve(path)
    assert set(resolved.sensor_plan["streams"]) == LEGACY_STREAMS
    assert "scene_measurements" not in resolved.sensor_plan
    assert resolved.transport_intent["channels"] == []


def test_rig_can_supply_body_and_factory_options_without_losing_package_calibration(catalog):
    resolver, path = catalog
    rig = resolver.find_package("fixture_rig@1.0.0").data["sensors"][0]
    rig["configuration"] = {"measurement": {"body_frame_id": "thunder_01/base_link",
                                            "noise": {"unit": "K", "enabled": True, "stddev": 0.1}}}
    resolved = resolver.resolve(path)
    config = resolved.sensor_plan["streams"]["thermal"][0]["measurement"]
    assert config["camera"]["fx_px"] == 2
    assert config["thermal"]["calibration_id"] == "fixture"
    assert config["noise"]["stddev"] == 0.1
    _validate(resolved.sensor_plan, "sensor-plan")


@pytest.mark.parametrize("camera", [
    {"model": "equidistant_fisheye", "width": 4, "height": 3, "focal_px": 2,
     "cx_px": 2, "cy_px": 1.5, "fov_deg": 180, "valid_radius_px": 1.5},
    {"model": "equirectangular", "width": 4, "height": 2},
    {"model": "equirectangular", "width": 4, "height": 2, "longitude_min_deg": -100,
     "longitude_max_deg": 100, "latitude_min_deg": -45, "latitude_max_deg": 45},
])
def test_factory_projection_and_optional_calibration_fields_compile(catalog, camera):
    resolver, path = catalog
    for kind in ("thermal", "semantic"):
        config = resolver.find_package(f"fixture_{kind}@1.0.0").data["configuration"]["measurement"]
        config.update(camera=camera, gimbal={"pan_deg": 5, "tilt_deg": -2,
                                           "pan_limits_deg": [-10, 10], "tilt_limits_deg": [-5, 5]},
                      max_distance_m=42)
    thermal = resolver.find_package("fixture_thermal@1.0.0").data["configuration"]["measurement"]
    thermal["thermal"].update(transmittance=0.8, atmospheric_temperature_k=280,
                              temperature_mode="surface_corrected", min_temperature_k=100, max_temperature_k=400)
    thermal["noise"] = {"unit": "K", "enabled": True, "stddev": 0.2, "seed": 2**64 - 1}
    acoustic = resolver.find_package("fixture_acoustic@1.0.0").data["configuration"]["measurement"]
    acoustic["microphone"].update(speed_of_sound_mps=340, sensitivity_volts_per_pa=0.01,
                                  calibration_metadata={"calibration_id": "fixture", "gain": 1, "extra": None})
    acoustic["noise"] = {"unit": "Pa", "enabled": True, "stddev": 0.01}
    resolved = resolver.resolve(path)
    _validate(resolved.sensor_plan, "sensor-plan")
    assert resolved.sensor_plan["streams"]["thermal"][0]["measurement"]["camera"] == camera
    assert resolved.sensor_plan["streams"]["acoustic"][0]["measurement"]["microphone"]["sensitivity_volts_per_pa"] == 0.01


def test_null_optional_microphone_calibration_is_accepted_from_rig_manifest(catalog):
    resolver, path = catalog
    rig = resolver.find_package("fixture_rig@1.0.0")
    acoustic = next(sensor for sensor in rig.data["sensors"] if sensor["id"] == "acoustic")
    config = copy.deepcopy(resolver.find_package("fixture_acoustic@1.0.0").data["configuration"])
    config["measurement"]["microphone"].update(sensitivity_volts_per_pa=None, calibration_metadata=None)
    acoustic["configuration"] = config
    rig.manifest_path.write_text(yaml.safe_dump(rig.data), encoding="utf-8")
    world = resolver.find_package("measurement_fixture@1.0.0")
    world.manifest_path.write_text(yaml.safe_dump(world.data), encoding="utf-8")
    reloaded = CatalogResolver(resolver.repo_root, resolver.catalog_roots)
    _validate(reloaded.resolve(path).sensor_plan, "sensor-plan")


@pytest.mark.parametrize(("field", "value"), [
    ("source_id", " motor"), ("body_name", "target"), ("body_name", "thunder_01/base_link"),
    ("pressure_pa", []), ("pressure_pa", [[0.1]]), ("pressure_pa", [float("inf")]),
    ("pressure_pa", [True]), ("pressure_pa", ["0.1"]), ("sample_rate_hz", 0),
    ("start_sim_time_ns", -1), ("reference_distance_m", 0), ("local_offset_m", [0, 0]),
    ("local_offset_m", [0, float("nan"), 0]), ("typo", 0),
])
def test_invalid_world_acoustic_sources_are_rejected(catalog, field, value):
    resolver, path = catalog
    resolver.find_package("measurement_fixture@1.0.0").data["measurements"]["acoustic_sources"][0][field] = value
    with pytest.raises(CatalogError, match="acoustic_sources"):
        resolver.resolve(path)


def test_scene_rejects_duplicate_sources_and_conflicting_instance_labels(catalog):
    resolver, path = catalog
    world = resolver.find_package("measurement_fixture@1.0.0").data["measurements"]
    world["acoustic_sources"] *= 2
    with pytest.raises(CatalogError, match="source_id must be unique"):
        resolver.resolve(path)
    world["acoustic_sources"] = []
    mjcf = resolver.find_package("measurement_fixture@1.0.0").manifest_path.parent / "world.xml"
    mjcf.write_text(mjcf.read_text().replace('<geom name="target"/>', '<geom name="target"/><geom name="other"/>'))
    world["semantic_labels"]["other"] = {"class_id": 3, "instance_id": 7}
    with pytest.raises(CatalogError, match="one class per instance"):
        resolver.resolve(path)


@pytest.mark.parametrize(("kind", "field", "value"), [
    ("thermal", "body_frame_id", "base_link"), ("thermal", "body_frame_id", " robot/base_link"),
    ("thermal", "mount_pose", {"position_m": [0, 0, 0], "orientation_wxyz": [0, 0, 0, 0]}),
    ("thermal", "noise", {"unit": "Pa"}), ("thermal", "noise", {"unit": "K", "seed": 2**64}),
    ("thermal", "noise", {"unit": "K", "stddev": -1}), ("thermal", "max_distance_m", 0),
    ("thermal", "typo", True), ("thermal", "thermal", {"calibration_id": "f", "background_temperature_k": 0}),
    ("thermal", "thermal", {"calibration_id": "f", "background_temperature_k": 300, "transmittance": 1.1}),
    ("thermal", "camera", {"model": "pinhole", "width": 0, "height": 1,
                           "fx_px": 1, "fy_px": 1, "cx_px": 0, "cy_px": 0}),
    ("semantic", "noise", {"unit": "label", "enabled": True}),
    ("semantic", "thermal", {}), ("acoustic", "camera", {}),
    ("acoustic", "microphone", {"sample_rate_hz": 1000, "block_samples": 100,
                               "channel_offsets_m": [[0, 0, 0]], "sensitivity_volts_per_pa": 0.01}),
    ("acoustic", "microphone", {"sample_rate_hz": 1000, "block_samples": 100,
                               "channel_offsets_m": [[0, 0, 0]], "sensitivity_volts_per_pa": 0.01,
                               "calibration_metadata": {"nested": [{"gain": float("inf")}]}}),
])
def test_sensor_plan_schema_rejects_invalid_measurement_fields(catalog, kind, field, value):
    jsonschema = pytest.importorskip("jsonschema")
    resolver, path = catalog
    document = resolver.resolve(path).sensor_plan
    document["streams"][kind][0]["measurement"][field] = value
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, "sensor-plan")


@pytest.mark.parametrize("name", ["sensor-plan", "world"])
def test_scene_schema_is_strict_in_both_owners(catalog, name):
    jsonschema = pytest.importorskip("jsonschema")
    resolver, path = catalog
    document = (resolver.resolve(path).sensor_plan if name == "sensor-plan"
                else copy.deepcopy(resolver.find_package("measurement_fixture@1.0.0").data))
    scene = document["scene_measurements" if name == "sensor-plan" else "measurements"]
    scene["thermal_surfaces"]["target"]["typo"] = 1
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, name)


def test_sensor_plan_schema_requires_scene_exactly_when_measurement_streams_exist(catalog):
    jsonschema = pytest.importorskip("jsonschema")
    resolver, path = catalog
    document = resolver.resolve(path).sensor_plan
    del document["scene_measurements"]
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, "sensor-plan")
    for kind in MESSAGES:
        del document["streams"][kind]
    _validate(document, "sensor-plan")
    document["scene_measurements"] = {}
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, "sensor-plan")


@pytest.mark.parametrize("kind", MESSAGES)
def test_sensor_plan_schema_rejects_measurement_route_and_message_mismatches(catalog, kind):
    jsonschema = pytest.importorskip("jsonschema")
    resolver, path = catalog
    document = resolver.resolve(path).sensor_plan
    stream = document["streams"][kind][0]
    stream["message_type"] = "lingtu.dds.Odometry"
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, "sensor-plan")
    stream["message_type"] = f"lingtu.dds.{MESSAGES[kind]}"
    stream["transport"] = "in_process"
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, "sensor-plan")


@pytest.mark.parametrize("relative", [
    "sim/sessions/products/doso/thunder_v4/default.yaml",
    "sim/sessions/examples/thunder_omni_contract/session.yaml",
    "sim/sessions/examples/thunderv4_controlled_headless/session.yaml",
])
def test_existing_session_bytes_ignore_unconsumed_world_measurements(relative):
    resolver = CatalogResolver.from_repository(REPO_ROOT)
    path = REPO_ROOT / relative
    before = resolver.resolve(path)
    session = yaml.safe_load(path.read_text(encoding="utf-8"))
    resolver.find_package(session["world"]).data["measurements"] = {}
    after = resolver.resolve(path)
    assert after.sensor_json.encode("utf-8") == before.sensor_json.encode("utf-8")
    assert after.transport_json.encode("utf-8") == before.transport_json.encode("utf-8")
    assert set(after.sensor_plan["streams"]) == LEGACY_STREAMS
    assert "scene_measurements" not in after.sensor_plan
