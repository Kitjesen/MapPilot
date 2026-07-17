"""Producer -> Consumer WireSpec definitions for the DDS dataflow example.

This module registers the wire that connects ``ExampleProducerModule`` to
``ExampleConsumerModule`` using DDS delivery with an explicit topic name.

The specs function is designed to be called from
``full_stack_wiring.py`` (or any custom assembly script) with a
``WiringContext`` that describes the current module set.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from runtime.registry import register

from .types import WireSpec

if TYPE_CHECKING:
    from .context import WiringContext

# ---------------------------------------------------------------------------
# Topic contract
# ---------------------------------------------------------------------------

# Stable DDS topic name shared between producer and consumer.
TOPIC_EXAMPLE_SENSOR = "/example/sensor_data"

# ---------------------------------------------------------------------------
# Module names
# ---------------------------------------------------------------------------

PRODUCER_MODULE = "ExampleProducerModule"
CONSUMER_MODULE = "ExampleConsumerModule"

# ---------------------------------------------------------------------------
# WireSpec factory
# ---------------------------------------------------------------------------


def example_flow_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Return WireSpec entries for the example producer-to-consumer flow.

    Only emits a wire when *both* producer and consumer modules are present
    in ``ctx.names``. Delivery defaults to ``"dds"`` with the topic constant
    above so the DDS backend can route the data correctly.

    On profiles where DDS is unavailable (e.g. ``stub``, ``dev``), the
    blueprint assembly will transparently downgrade to local transport.

    Args:
        ctx: Current wiring context.

    Returns:
        Tuple of zero or one WireSpec.
    """
    if PRODUCER_MODULE not in ctx.names or CONSUMER_MODULE not in ctx.names:
        return ()

    return (
        WireSpec(
            out_module=PRODUCER_MODULE,
            out_port="sensor_data",
            in_module=CONSUMER_MODULE,
            in_port="sensor_data",
            delivery="dds",
            topic=TOPIC_EXAMPLE_SENSOR,
        ),
    )
