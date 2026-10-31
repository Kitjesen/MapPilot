"""Opt-in SensorPlan measurement binding at the Product compiler boundary."""

import json
from copy import deepcopy
from types import SimpleNamespace

import pytest
from sim.catalog import CatalogResolver

from lingtu.assembly.compile_plan import compile_run_plan, host_builder_from_run_plan
from lingtu.assembly.graph import ProcessArtifact, load_runtime_graph


def measurement_stream(kind, sensor_id=None, *, instance_id="thunder_01", frame_id=None):
    return {
        "sensor_id": sensor_id or f"{instance_id}.{kind}",
        "instance_id": instance_id,
        "frame_id": frame_id or f"{instance_id}/{kind}_link",
        "owner": "physics",
        "source": "mujoco_measurement",
        "transport": "typed_dds",
        "message_type": f"{kind.capitalize()}Measurement",
        "rate_hz": 10,
    }


@pytest.fixture(scope="module")
def baseline_plan():
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(ProcessArtifact, "from_repository_path", classmethod(lambda cls, root, path: cls(str(path))))
        return compile_run_plan("tracking", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})


@pytest.fixture
def compilation(monkeypatch, allow_unbuilt_process_artifacts, baseline_plan):
    baseline = baseline_plan
    snapshot = baseline.simulation
    # Supply the external catalog's compiled plans; exercise the real Product compiler.
    monkeypatch.setattr(CatalogResolver, "resolve", lambda self, path: SimpleNamespace(**deepcopy(snapshot)))

    def compile_plan(**kwargs):
        return compile_run_plan("tracking", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"}, **kwargs)

    return baseline, snapshot, compile_plan


def test_measurements_bind_exact_session_and_sensor_ids_without_product_identity(compilation):
    baseline, snapshot, compile_plan = compilation
    ids = ["thunder_01.thermal", "thunder_01.microphone", "thunder_01.labels"]
    for kind, sensor_id in zip(("thermal", "acoustic", "semantic"), ids):
        snapshot["sensor_plan"]["streams"][kind] = [measurement_stream(kind, sensor_id)]

    plan = compile_plan()
    camera = next(process for process in baseline.processes if "camera" in process.provides)
    expected_argv = (*camera.command.argv, "--measurement-session-id", snapshot["sensor_plan"]["session_id"])
    selected = next(process for process in plan.processes if process.name == camera.name)
    available = next(process for process in plan.available_processes if process.name == camera.name)
    assert selected.command.argv == available.command.argv == expected_argv
    expected_process = camera.as_dict()
    expected_process["command"]["argv"] = list(expected_argv)
    assert selected.as_dict() == available.as_dict() == expected_process
    assert plan.host_config == {
        **baseline.host_config,
        "enable_measurements": True,
        "measurement_session_id": snapshot["sensor_plan"]["session_id"],
        "measurement_sensor_ids": ids,
        "measurement_sensors": {
            sensor_id: {
                "modality": kind,
                "session_id": snapshot["sensor_plan"]["session_id"],
                "instance_id": "thunder_01",
                "frame_id": f"thunder_01/{kind}_link",
                "model_generation": snapshot["scenario_plan"]["model_generation"],
                "reset_generation": snapshot["scenario_plan"]["reset_generation"],
            }
            for kind, sensor_id in zip(("thermal", "acoustic", "semantic"), ids)
        },
    }
    assert plan.process_environment == baseline.process_environment
    assert [p for p in plan.processes if p.name != camera.name] == [
        p for p in baseline.processes if p.name != camera.name
    ]
    assert "--measurement-product-session-id" not in selected.command.argv
    assert plan.critical_modules == (*baseline.critical_modules, "measurements", "sensor.controls")


def test_no_measurement_declarations_leave_default_run_plan_byte_identical(compilation):
    baseline, _, compile_plan = compilation
    assert json.dumps(compile_plan().as_dict(), sort_keys=True) == json.dumps(baseline.as_dict(), sort_keys=True)
    assert "measurement_sensors" not in baseline.host_config


def test_inspection_rebuild_connects_reader_capture_and_gateway_without_starting(compilation, monkeypatch):
    _, snapshot, _ = compilation
    for kind in ("thermal", "acoustic", "semantic"):
        snapshot["sensor_plan"]["streams"][kind] = [measurement_stream(kind)]
    monkeypatch.delenv("LINGTU_PRODUCT_SESSION_ID", raising=False)
    plan = compile_run_plan("inspection", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})
    assert "measurements" in plan.critical_modules
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "active-inspection-product")
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "6")
    builder = host_builder_from_run_plan(plan)
    entries = {item.name: item for item in builder._entries}
    assert entries["measurements"].config["product_session_id"] == "active-inspection-product"
    assert entries["InspectionEvidenceModule"].config["measurement_sensors"] == plan.host_config["measurement_sensors"]
    assert entries["InspectionEvidenceModule"].config["product_session_id"] == "active-inspection-product"
    assert entries["GatewayModule"].config["measurement_sensor_ids"] == plan.host_config["measurement_sensor_ids"]
    wires = builder.export_graph().explicit_wires
    assert {(wire.out_port, wire.in_module, wire.in_port) for wire in wires if wire.out_module == "measurements"} == {
        (kind, consumer, kind) for kind in ("thermal", "acoustic", "semantic")
        for consumer in ("InspectionEvidenceModule", "GatewayModule")}


def test_measurement_source_identity_and_scenario_generations_are_copied_exactly(compilation):
    _, snapshot, compile_plan = compilation
    for field in ("session", "physics_plan", "visual_plan", "sensor_plan", "control_plan", "transport_intent", "scenario_plan"):
        snapshot[field]["session_id"] = "measurement-source-session"
    snapshot["scenario_plan"].update(model_generation=7, reset_generation=12)
    streams = [
        measurement_stream("thermal", "payload.hot", instance_id="payload_02", frame_id="payload_02/thermal_optical"),
        measurement_stream("thermal", "robot.hot", instance_id="robot_03", frame_id="robot_03/camera/optical"),
    ]
    for stream in streams:
        # Only the scenario owns generations, not individual stream declarations.
        stream.update(model_generation=91, reset_generation=92)
    snapshot["sensor_plan"]["streams"]["thermal"] = streams

    plan = compile_plan()
    assert plan.host_config["measurement_sensor_ids"] == ["payload.hot", "robot.hot"]
    assert plan.host_config["measurement_sensors"] == {
        stream["sensor_id"]: {
            "modality": "thermal",
            "session_id": "measurement-source-session",
            "instance_id": stream["instance_id"],
            "frame_id": stream["frame_id"],
            "model_generation": 7,
            "reset_generation": 12,
        }
        for stream in streams
    }
    assert "product_session_id" not in plan.host_config


def test_measurement_generations_default_to_zero_only_without_scenario(compilation):
    _, snapshot, compile_plan = compilation
    snapshot["scenario_plan"] = None
    snapshot["session"].pop("scenario", None)
    snapshot["sensor_plan"]["streams"]["thermal"] = [measurement_stream("thermal")]
    binding = compile_plan().host_config["measurement_sensors"]["thunder_01.thermal"]
    assert binding["model_generation"] == binding["reset_generation"] == 0


@pytest.mark.parametrize("field", ["instance_id", "frame_id"])
@pytest.mark.parametrize("value", [None, "", "   ", " padded", "padded ", "bad\0identity", 12, False])
def test_measurement_source_identity_must_be_usable(compilation, field, value):
    _, snapshot, compile_plan = compilation
    stream = measurement_stream("thermal")
    stream[field] = value
    snapshot["sensor_plan"]["streams"]["thermal"] = [stream]
    with pytest.raises(ValueError, match=rf"measurement.*{field}"):
        compile_plan()


@pytest.mark.parametrize("field", ["instance_id", "frame_id"])
def test_measurement_source_identity_must_be_declared(compilation, field):
    _, snapshot, compile_plan = compilation
    stream = measurement_stream("thermal")
    del stream[field]
    snapshot["sensor_plan"]["streams"]["thermal"] = [stream]
    with pytest.raises(ValueError, match=rf"measurement.*{field}"):
        compile_plan()


@pytest.mark.parametrize("field", ["model_generation", "reset_generation"])
@pytest.mark.parametrize("value", [None, -1, True, False, 1.0, "2"])
def test_present_scenario_requires_valid_measurement_generations(compilation, field, value):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["thermal"] = [measurement_stream("thermal")]
    snapshot["scenario_plan"][field] = value
    with pytest.raises(ValueError, match=r"measurement.*generation"):
        compile_plan()


@pytest.mark.parametrize("scenario", [{}, {"model_generation": 1}, {"reset_generation": 2}, [], "invalid"])
def test_incomplete_scenario_does_not_default_measurement_generations(compilation, scenario):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["thermal"] = [measurement_stream("thermal")]
    snapshot["scenario_plan"] = scenario
    with pytest.raises(ValueError, match=r"measurement.*scenario"):
        compile_plan()


@pytest.mark.parametrize(
    ("remove", "append"),
    [
        (("--dds",), ()),
        (("--local-endpoint",), ()),
        (("--stream", "camera"), ()),
        (("camera",), ("--stream", "imu")),
        ((), ("--dds",)),
        ((), ("--local-endpoint",)),
        ((), ("--stream", "camera")),
        ((), ("--domain-id", "17")),
        (("--domain-id",), ("--domain-id", "999")),
        (("--domain-id",), ("--domain-id", "other")),
        (("--domain-id",), ()),
        ((), ("--measurement-session-id", "wrong-session")),
        ((), ("--measurement-session-id",)),
        ((), ("--measurement-session-id=wrong-session",)),
        (
            (),
            (
                "--measurement-session-id",
                "thunderv4_tracking_headless",
                "--measurement-session-id",
                "thunderv4_tracking_headless",
            ),
        ),
        ((), ("--measurement-product-session-id", "not-authenticated")),
        ((), ("--measurement-product-session-id=not-authenticated",)),
        ((), ("--stdin-records",)),
        ((), ("--validate-records",)),
        ((), ("--ready-file", "camera.ready.json")),
    ],
)
def test_measurements_require_complete_native_camera_endpoint(compilation, remove, append):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["thermal"] = [measurement_stream("thermal")]
    graph = deepcopy(load_runtime_graph())
    camera = graph.envs["sim"]["backends"]["mujoco"]["processes"]["camera_publisher"]
    for command in camera["platforms"].values():
        for arg in remove:
            command["argv"].remove(arg)
        command["argv"].extend(append)
    with pytest.raises(ValueError, match="measurement"):
        compile_plan(graph=graph)


@pytest.mark.parametrize(
    "declaration",
    [
        [],
        None,
        {},
        [{}],
        [{"sensor_id": ""}],
        [{"sensor_id": " invalid"}],
        [{"sensor_id": "sensor-only"}],
        [{"sensor_id": "same"}, {"sensor_id": "same"}],
    ],
)
def test_partial_measurement_declarations_are_rejected(compilation, declaration):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["thermal"] = declaration
    with pytest.raises(ValueError, match="measurement"):
        compile_plan()


@pytest.mark.parametrize("kind", ["thermal", "acoustic", "semantic"])
def test_each_measurement_kind_can_opt_in_independently(compilation, kind):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"][kind] = [measurement_stream(kind)]
    config = compile_plan().host_config
    assert config["measurement_sensor_ids"] == [f"thunder_01.{kind}"]
    assert list(config["measurement_sensors"]) == [f"thunder_01.{kind}"]
    assert config["measurement_sensors"][f"thunder_01.{kind}"]["modality"] == kind


@pytest.mark.parametrize("session_id", [None, "", " padded", "invalid/session"])
def test_measurements_never_invent_or_normalize_session_identity(compilation, session_id):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["thermal"] = [measurement_stream("thermal")]
    snapshot["sensor_plan"]["session_id"] = session_id
    with pytest.raises(ValueError, match=r"measurement.*session_id"):
        compile_plan()


def test_duplicate_sensor_identity_across_measurement_kinds_is_rejected(compilation):
    _, snapshot, compile_plan = compilation
    for kind in ("thermal", "semantic"):
        snapshot["sensor_plan"]["streams"][kind] = [measurement_stream(kind, "thunder_01.same")]
    with pytest.raises(ValueError, match=r"measurement.*unique sensor"):
        compile_plan()


def test_measurements_do_not_invent_a_camera_process(compilation):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["acoustic"] = [measurement_stream("acoustic", "thunder_01.microphone")]
    graph = deepcopy(load_runtime_graph())
    graph.products["tracking"]["processes"].remove("camera")
    with pytest.raises(ValueError, match=r"measurement.*selected native camera"):
        compile_plan(graph=graph)


def test_matching_explicit_session_argument_is_not_duplicated(compilation):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["semantic"] = [measurement_stream("semantic")]
    graph = deepcopy(load_runtime_graph())
    camera = graph.envs["sim"]["backends"]["mujoco"]["processes"]["camera_publisher"]
    for command in camera["platforms"].values():
        command["argv"].extend(("--measurement-session-id", snapshot["sensor_plan"]["session_id"]))
    plan = compile_plan(graph=graph)
    assert next(p for p in plan.processes if "camera" in p.provides).command.argv.count("--measurement-session-id") == 1


def test_non_native_camera_executable_cannot_claim_measurement_transport(compilation):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["thermal"] = [measurement_stream("thermal")]
    graph = deepcopy(load_runtime_graph())
    camera = graph.envs["sim"]["backends"]["mujoco"]["processes"]["camera_publisher"]
    for command in camera["platforms"].values():
        command["argv"][0] = command["artifact"]["path"] = "build/other_camera"
    with pytest.raises(ValueError, match=r"measurement.*native sensor publisher"):
        compile_plan(graph=graph)


def test_duplicate_camera_arguments_are_rejected(compilation):
    _, snapshot, compile_plan = compilation
    snapshot["sensor_plan"]["streams"]["thermal"] = [measurement_stream("thermal")]
    graph = deepcopy(load_runtime_graph())
    camera = graph.envs["sim"]["backends"]["mujoco"]["processes"]["camera_publisher"]
    for command in camera["platforms"].values():
        command["argv"].extend(("--scan-window", "0"))
    with pytest.raises(ValueError, match=r"measurement.*duplicate"):
        compile_plan(graph=graph)
