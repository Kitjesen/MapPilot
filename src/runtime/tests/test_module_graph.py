from __future__ import annotations

import json

from runtime.blueprint import Blueprint
from lingtu.assembly.graph import WireEdge, blueprint_for_profile, graph_for_profile
from lingtu.assembly.wires.types import WireSpec
from runtime.module import Module
from runtime.stream import In, Out


class SourceModule(Module):
    pass


class SinkModule(Module):
    pass


class PortSourceModule(Module):
    frames: Out[dict]


class PortSinkModule(Module):
    frames_in: In[dict]


class ExplodingModule(Module):
    def __init__(self, **_config):
        raise AssertionError("export_graph must not instantiate modules")


class ObjectTransport:
    backend_name = "object_transport"


def test_blueprint_exports_portable_module_graph_without_building_modules():
    bp = (
        Blueprint()
        .add(SourceModule, alias="Source", rate_hz=20, tags=("lidar", "portable"))
        .add(SinkModule)
        .wire(
            "Source",
            "frames",
            "SinkModule",
            "frames_in",
            transport="dds",
            topic="/source/frames",
        )
        .auto_wire()
        .global_config(n_workers=2, runtime="portable")
    )

    graph = bp.export_graph(profile="unit")
    manifest = graph.to_manifest()

    assert graph.profile == "unit"
    assert graph.module_names == ("Source", "SinkModule")
    assert graph.dangling_wires() == ()
    assert graph.explicit_wires[0].as_snapshot() == (
        "Source.frames->SinkModule.frames_in[dds]@/source/frames"
    )
    assert manifest["schema_version"] == "lingtu.module_graph.v1"
    assert manifest["modules"][0]["config"] == {
        "rate_hz": 20,
        "tags": ["lidar", "portable"],
    }
    assert manifest["auto_wire"] is True
    assert manifest["global_config"] == {"n_workers": 2, "runtime": "portable"}
    assert manifest["explicit_wires"][0]["topic"] == "/source/frames"
    assert manifest["explicit_wires"][0]["delivery"] == "dds"
    assert manifest["explicit_wires"][0]["transport"] == "dds"


def test_module_graph_export_does_not_instantiate_modules():
    graph = Blueprint().add(ExplodingModule, answer=42).export_graph()

    assert graph.module_names == ("ExplodingModule",)
    assert graph.modules[0].config == {"answer": 42}


def test_module_graph_marks_preinstantiated_modules():
    instance = SourceModule()

    graph = Blueprint().add(instance, alias="ReadySource").export_graph()

    assert graph.module_names == ("ReadySource",)
    assert graph.modules[0].preinstantiated is True
    assert graph.modules[0].config == {}


def test_module_graph_exposes_declared_ports_from_module_annotations():
    graph = Blueprint().add(PortSourceModule).add(PortSinkModule).export_graph()

    assert graph.module_spec("PortSourceModule").declares_port("frames")
    assert graph.module_spec("PortSinkModule").declares_port("frames_in")
    assert not graph.module_spec("PortSinkModule").declares_port("frames")


def test_module_graph_manifest_is_json_ready_for_object_transport():
    bp = Blueprint().add(SourceModule, alias="Source").add(SinkModule)
    bp.wire("Source", "frames", "SinkModule", "frames_in", transport=ObjectTransport())

    manifest = bp.export_graph().to_manifest()

    assert manifest["explicit_wires"][0]["transport"] == "object_transport"
    json.dumps(manifest)


def test_blueprint_wire_accepts_delivery_alias_for_transport():
    bp = Blueprint().add(SourceModule, alias="Source").add(SinkModule)
    bp.wire(
        "Source",
        "frames",
        "SinkModule",
        "frames_in",
        delivery="local",
        topic="/source/frames",
    )

    manifest = bp.export_graph().to_manifest()

    assert manifest["explicit_wires"][0]["delivery"] == "local"
    assert manifest["explicit_wires"][0]["transport"] == "local"


def test_blueprint_wire_rejects_conflicting_delivery_and_transport():
    bp = Blueprint().add(SourceModule, alias="Source").add(SinkModule)

    try:
        bp.wire(
            "Source",
            "frames",
            "SinkModule",
            "frames_in",
            transport="dds",
            delivery="shm",
        )
    except ValueError as exc:
        assert "conflicting delivery and transport" in str(exc)
    else:
        raise AssertionError("conflicting delivery/transport should fail")


def test_profile_wire_edge_preserves_string_transport_name():
    spec = WireSpec(
        "Source",
        "frames",
        "SinkModule",
        "frames_in",
        transport="local",
        topic="/source/frames",
    )

    edge = WireEdge.from_blueprint_spec(spec)

    assert edge.as_snapshot() == "Source.frames->SinkModule.frames_in[local]@/source/frames"


def test_module_graph_reflects_namespace_and_merge():
    first = Blueprint().add(SourceModule, alias="Source")
    first.wire("Source", "frames", "SinkModule", "frames_in")
    second = Blueprint().add(SinkModule)

    bp = first.namespace("robot_0").merge(second.namespace("robot_0"))
    graph = bp.export_graph()

    assert graph.module_names == ("robot_0/Source", "robot_0/SinkModule")
    assert graph.explicit_wires[0].as_snapshot() == (
        "robot_0/Source.frames->robot_0/SinkModule.frames_in"
    )


def test_module_graph_reports_dangling_wires_before_runtime_build():
    bp = Blueprint().add(SourceModule, alias="Source")
    bp.wire("Source", "frames", "MissingModule", "frames_in")

    graph = bp.export_graph()

    assert [wire.in_module for wire in graph.dangling_wires()] == ["MissingModule"]


def test_profile_runtime_graph_uses_blueprint_export_graph():
    bp = blueprint_for_profile("stub")
    exported = bp.export_graph(profile="stub")
    runtime_graph = graph_for_profile("stub", mode="runtime")

    assert runtime_graph.modules == exported.module_names
    assert runtime_graph.as_snapshot() == exported.as_snapshot()
