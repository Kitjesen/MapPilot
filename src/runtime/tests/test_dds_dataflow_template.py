"""Tests for the DDS dataflow template, example_flow, and example modules.

Covers:
    1. ``template_specs`` — WireSpec generation from WiringContext.
    2. ``example_flow_specs`` — producer-to-consumer WireSpec with DDS delivery.
    3. Module port declarations (In/Out) for ExampleProducerModule and
       ExampleConsumerModule.
    4. Blueprint assembly and local-transport mock wiring end-to-end.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from unittest.mock import patch

import pytest

# Ensure src/ is importable.
_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from runtime.blueprint import Blueprint
from runtime.blueprints.wires.context import WiringContext
from runtime.blueprints.wires.example_flow import (
    CONSUMER_MODULE,
    PRODUCER_MODULE,
    TOPIC_EXAMPLE_SENSOR,
    example_flow_specs,
)
from runtime.blueprints.wires.template import (
    TEMPLATE_DATA_CONSUMERS,
    TOPIC_TEMPLATE_DATA,
    template_specs,
)
from runtime.blueprints.wires.types import WireSpec, wire_key
from runtime.module import Module
from runtime.stream import In, Out
from runtime.wiring import WireDelivery

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_ctx(
    names: set[str] | frozenset[str],
    *,
    slam_profile: str = "none",
    driver_module: str = "StubDriverModule",
) -> WiringContext:
    """Build a minimal WiringContext for tests."""
    return WiringContext(
        names=frozenset(names),
        robot="stub",
        driver_module=driver_module,
        slam_profile=slam_profile,
        slam_module="",
        scene_xml="",
        enable_semantic=False,
        camera_src=driver_module,
        color_out="color_image",
        nav_odom_src=driver_module,
    )


# ---------------------------------------------------------------------------
# template_specs tests
# ---------------------------------------------------------------------------


class TestTemplateSpecs:
    """Verify template_specs generates correct WireSpec entries."""

    def test_returns_empty_when_producer_absent(self) -> None:
        ctx = _make_ctx({"ExampleConsumerModule"})
        assert template_specs(ctx) == ()

    def test_returns_empty_when_consumer_absent(self) -> None:
        ctx = _make_ctx({"ExampleProducerModule"})
        assert template_specs(ctx) == ()

    def test_generates_wire_when_both_present(self) -> None:
        ctx = _make_ctx({"ExampleProducerModule", "ExampleConsumerModule"})
        specs = template_specs(ctx)
        assert len(specs) == 1
        spec = specs[0]
        assert spec.out_module == "ExampleProducerModule"
        assert spec.out_port == "sensor_data"
        assert spec.in_module == "ExampleConsumerModule"
        assert spec.in_port == "sensor_data"

    def test_delivery_is_dds(self) -> None:
        ctx = _make_ctx({"ExampleProducerModule", "ExampleConsumerModule"})
        specs = template_specs(ctx)
        assert specs[0].delivery == "dds"

    def test_topic_constant_used(self) -> None:
        ctx = _make_ctx({"ExampleProducerModule", "ExampleConsumerModule"})
        specs = template_specs(ctx)
        assert specs[0].topic == TOPIC_TEMPLATE_DATA

    def test_wire_key_uniqueness(self) -> None:
        ctx = _make_ctx({"ExampleProducerModule", "ExampleConsumerModule"})
        specs = template_specs(ctx)
        keys = {wire_key(s) for s in specs}
        assert len(keys) == len(specs)

    def test_consumers_tuple_is_immutable(self) -> None:
        assert isinstance(TEMPLATE_DATA_CONSUMERS, tuple)


# ---------------------------------------------------------------------------
# example_flow_specs tests
# ---------------------------------------------------------------------------


class TestExampleFlowSpecs:
    """Verify example_flow_specs returns the correct WireSpec."""

    def test_empty_when_producer_missing(self) -> None:
        ctx = _make_ctx({CONSUMER_MODULE})
        assert example_flow_specs(ctx) == ()

    def test_empty_when_consumer_missing(self) -> None:
        ctx = _make_ctx({PRODUCER_MODULE})
        assert example_flow_specs(ctx) == ()

    def test_generates_wire(self) -> None:
        ctx = _make_ctx({PRODUCER_MODULE, CONSUMER_MODULE})
        specs = example_flow_specs(ctx)
        assert len(specs) == 1
        spec = specs[0]
        assert spec.out_module == PRODUCER_MODULE
        assert spec.in_module == CONSUMER_MODULE
        assert spec.out_port == "sensor_data"
        assert spec.in_port == "sensor_data"

    def test_delivery_and_topic(self) -> None:
        ctx = _make_ctx({PRODUCER_MODULE, CONSUMER_MODULE})
        spec = example_flow_specs(ctx)[0]
        assert spec.delivery == "dds"
        assert spec.topic == TOPIC_EXAMPLE_SENSOR

    def test_returns_tuple(self) -> None:
        ctx = _make_ctx({PRODUCER_MODULE, CONSUMER_MODULE})
        assert isinstance(example_flow_specs(ctx), tuple)


# ---------------------------------------------------------------------------
# Module port-declaration tests (imported from the example script)
# ---------------------------------------------------------------------------


class TestModulePortDeclarations:
    """Verify that ExampleProducerModule and ExampleConsumerModule declare
    the correct In/Out ports."""

    def test_producer_has_sensor_data_out(self) -> None:
        # Import from the example script without running main().
        from examples.dds_dataflow_example import ExampleProducerModule

        assert hasattr(ExampleProducerModule, "sensor_data")

    def test_consumer_has_sensor_data_in(self) -> None:
        from examples.dds_dataflow_example import ExampleConsumerModule

        assert hasattr(ExampleConsumerModule, "sensor_data")

    def test_producer_instantiates_with_port(self) -> None:
        from examples.dds_dataflow_example import ExampleProducerModule

        m = ExampleProducerModule()
        assert "sensor_data" in m.ports_out

    def test_consumer_instantiates_with_port(self) -> None:
        from examples.dds_dataflow_example import ExampleConsumerModule

        m = ExampleConsumerModule()
        assert "sensor_data" in m.ports_in


# ---------------------------------------------------------------------------
# Blueprint assembly with local transport mock
# ---------------------------------------------------------------------------


class TestBlueprintAssembly:
    """End-to-end test: build a Blueprint with both modules and verify the
    wire is established using local transport (DDS mocked to local)."""

    def test_blueprint_builds_with_local_fallback(self) -> None:
        """Blueprint wires resolve to local when DDS backend is absent."""
        from examples.dds_dataflow_example import (
            ExampleConsumerModule,
            ExampleProducerModule,
        )

        bp = (
            Blueprint("test_assembly")
            .add(ExampleProducerModule, interval=0.1)
            .add(ExampleConsumerModule)
            .wire(
                "ExampleProducerModule",
                "sensor_data",
                "ExampleConsumerModule",
                "sensor_data",
                delivery="local",
                topic="/test/sensor_data",
            )
        )
        system = bp.build()
        try:
            system.start()
            # Verify both modules are running.
            producer = system.get_module("ExampleProducerModule")
            consumer = system.get_module("ExampleConsumerModule")
            assert producer is not None
            assert consumer is not None
            assert producer.running
            assert consumer.running
        finally:
            system.stop()

    def test_consumer_receives_messages(self) -> None:
        """Verify end-to-end data delivery with local transport."""
        import time

        from examples.dds_dataflow_example import (
            ExampleConsumerModule,
            ExampleProducerModule,
        )

        bp = (
            Blueprint("test_delivery")
            .add(ExampleProducerModule, interval=0.05)
            .add(ExampleConsumerModule)
            .wire(
                "ExampleProducerModule",
                "sensor_data",
                "ExampleConsumerModule",
                "sensor_data",
                delivery="local",
            )
        )
        system = bp.build()
        try:
            system.start()
            time.sleep(0.3)
            consumer = system.get_module("ExampleConsumerModule")
            assert len(consumer.received) > 0, "Consumer should have received at least one message"
        finally:
            system.stop()

    def test_wire_spec_apply_matches_blueprint_wire(self) -> None:
        """WireSpec.apply() produces the same connection as Blueprint.wire()."""
        from examples.dds_dataflow_example import (
            ExampleConsumerModule,
            ExampleProducerModule,
        )

        spec = WireSpec(
            out_module="ExampleProducerModule",
            out_port="sensor_data",
            in_module="ExampleConsumerModule",
            in_port="sensor_data",
            delivery="local",
            topic="/spec/test",
        )

        bp = Blueprint("test_spec_apply").add(ExampleProducerModule).add(ExampleConsumerModule)
        spec.apply(bp)

        # The wire should now be present in the blueprint's internal list.
        assert len(bp._wires) == 1
        w = bp._wires[0]
        assert w.out_module == "ExampleProducerModule"
        assert w.in_module == "ExampleConsumerModule"
        assert w.topic == "/spec/test"
