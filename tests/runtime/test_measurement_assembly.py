"""Measurements join the Host graph only for an explicit compiled declaration."""

from lingtu.assembly.host.stacks import perception as perception_stack
from lingtu.assembly.host.stacks.measurements import measurements
from lingtu.assembly.host.wiring.connect import apply_full_stack_wires
from perception.inspection.bridge_module import InspectionEvidenceModule
from runtime.config import RobotConfig
from runtime.module import Module


def test_no_measurements_does_not_resolve_native_endpoint(monkeypatch):
    def unexpected(*args, **kwargs):
        raise AssertionError("disabled measurements resolved an endpoint")

    monkeypatch.setattr("lingtu.assembly.host.stacks.measurements.stack_module", unexpected)
    assert measurements().module_names == ()


def test_bound_reader_has_exact_sensor_identity_and_inspection_ports(monkeypatch):
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "active-product")
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "6")
    builder = measurements(enable_measurements=True, measurement_session_id="compiled-source",
                           measurement_sensor_ids=["robot.thermal", "robot.acoustic"],
                           measurement_sensors={key: {"model_generation": 2, "reset_generation": 1}
                               for key in ["robot.thermal", "robot.acoustic"]})
    entry = builder._entries[0]
    assert entry.name == "measurements"
    assert entry.config == dict(product_session_id="active-product", session_id="compiled-source",
                               domain_id=6, required_sensor_ids=["robot.thermal", "robot.acoustic"])
    builder.add(InspectionEvidenceModule, alias="InspectionEvidenceModule")
    apply_full_stack_wires(builder, driver_module="missing", slam_profile="none")
    wires = builder.export_graph().explicit_wires
    assert {(wire.out_port, wire.in_port) for wire in wires} == {
        ("thermal", "thermal"), ("acoustic", "acoustic"), ("semantic", "semantic"), ("state", "control_state"),
        ("state", "sensor_control_state")}
    controller = next(item for item in builder._entries if item.name == "sensor.controls")
    assert controller.config["model_generation"] == 2 and controller.config["reset_generation"] == 1


def test_compile_graph_has_no_native_side_effects_without_active_product(monkeypatch):
    monkeypatch.delenv("LINGTU_PRODUCT_SESSION_ID", raising=False)
    builder = measurements(enable_measurements=True, measurement_session_id="compiled-source",
                           measurement_sensor_ids=["robot.thermal"],
                           measurement_sensors={"robot.thermal": {"model_generation": 0, "reset_generation": 0}})
    assert builder.module_names == ("measurements", "sensor.controls")
    assert builder._entries[0].config["product_session_id"] == ""


def test_inspection_receives_compiled_bindings_and_explicit_rules(monkeypatch):
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "owned-product")
    monkeypatch.setattr(perception_stack, "get_config", RobotConfig)
    monkeypatch.setattr(perception_stack, "stack_module", lambda *a, **kw: Module)
    monkeypatch.setattr(perception_stack, "optional_stack_module", lambda *a, **kw: InspectionEvidenceModule)
    bindings = {"thermal": dict(modality="thermal", session_id="source", instance_id="robot", frame_id="robot/camera",
                                model_generation=2, reset_generation=1)}
    rules = {"thermal": dict(max_kelvin=350, roi=[0, 0, 2, 2], min_valid_coverage=1,
                            temperature_kind="apparent", calibration_id="declared",
                            model_id="declared_surface_broadband_greybody", model_version="1")}
    builder = perception_stack.perception(enable_camera=False, enable_inspection_evidence=True,
        enable_measurements=True, measurement_sensors=bindings, measurement_rules=rules,
        capture_sensor_ids={"capture:thermal": ["thermal"]})
    entry = next(item for item in builder._entries if item.name == "InspectionEvidenceModule")
    assert entry.config["measurement_sensors"] == bindings
    assert entry.config["measurement_rules"] == rules
    assert entry.config["product_session_id"] == "owned-product"
    assert entry.config["capture_sensor_ids"] == {"capture:thermal": ["thermal"]}
