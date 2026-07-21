"""WireSpec template for new data-flow integration via DDS.

This module demonstrates how to declare WireSpec entries that connect a
producer module's Out port to a consumer module's In port using DDS delivery
with a QoS profile configuration.

Usage checklist for integrating a new data flow into the LingTu blueprint:

    1. Define a producer Module (Out port) and a consumer Module (In port).
       Both must be registered via @register(category, name).

    2. Copy ``template_specs`` below, rename it (e.g. ``my_sensor_specs``),
       and adjust module names, port names, topic, and QoS profile to match
       your data flow.

    3. Import and call the new specs function from
       ``src/lingtu/assembly/full_stack_wiring.py``:

           from lingtu.assembly.wires.my_flow import my_sensor_specs
           ...
           specs.extend(my_sensor_specs(ctx))

    4. If the flow must cross machine boundaries, register the topic in
       ``src/runtime/route_contract/routes.py`` under ``_ROBOT_DDS_QOS`` so
       the DDS backend picks the correct QoS profile.

    5. Write a unit test in ``src/runtime/tests/`` following the pattern in
       ``test_dds_dataflow_template.py``.

All configurable WireSpec fields
---------------------------------

``out_module`` (str)
    Name of the source module (class name or alias used in the Blueprint).

``out_port`` (str)
    Name of the source module's Out[T] port attribute.

``in_module`` (str)
    Name of the destination module.

``in_port`` (str)
    Name of the destination module's In[T] port attribute.

``delivery`` (str | None)
    Per-wire delivery mode. Supported values:
      - None or ""      → in-process callback (default, zero-copy)
      - "local"         → explicit LocalTransport (same process, decoupled)
      - "dds"           → DDS transport (cross-process / cross-machine)
      - "shm"           → shared-memory transport (same host, large payloads)

    When ``delivery`` and ``transport`` are both set they must agree.
    Prefer ``delivery`` in new code; ``transport`` is a backward-compat alias.

``topic`` (str | None)
    Stable topic name used by the transport layer. When ``None`` the system
    auto-generates ``/<out_module>/<out_port>``. Explicit topics are required
    for DDS routes that must match a robot-side subscriber topic.

``transport`` (str | None)
    Backward-compatible alias for ``delivery``. Do not mix with ``delivery``
    unless both resolve to the same value.

QoS profile integration
-----------------------
The DDS transport reads a named QoS profile from
``config/qos_profiles.yaml`` via ``TopicConfig.qos_profile``.  Setting a
profile name here (e.g. ``"sensor_stream"``) overrides the default
``qos_depth`` / ``reliable`` fields.

To wire a QoS profile into a WireSpec, pass it via the ``topic`` field
combined with a route-contract entry.  The template below shows the
recommended pattern.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .types import WireSpec

if TYPE_CHECKING:
    from .context import WiringContext

# ---------------------------------------------------------------------------
# Topic constants — use runtime_interface.TOPICS when available; define local
# constants here for topics that are not yet in the central registry.
# ---------------------------------------------------------------------------

# Example topic for the template data flow.
TOPIC_TEMPLATE_DATA = "/template/sensor_data"

# ---------------------------------------------------------------------------
# Consumer list — modules that subscribe to the producer output.
# Keep this as a tuple for immutability; mirror the pattern in slam.py.
# ---------------------------------------------------------------------------

TEMPLATE_DATA_CONSUMERS: tuple[str, ...] = (
    "ExampleConsumerModule",
    # Add more consumer module names here as needed.
)

# ---------------------------------------------------------------------------
# WireSpec factory
# ---------------------------------------------------------------------------


def template_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Generate WireSpec entries for the template data flow.

    The function follows the same contract as other ``*_specs`` functions in
    this package: it receives a ``WiringContext``, inspects which modules are
    present in ``ctx.names``, and returns only the wires whose endpoints both
    exist in the current blueprint.

    Delivery is set to ``"dds"`` to demonstrate cross-process transport.
    On single-machine setups the blueprint will automatically fall back to
    ``"local"`` when the DDS backend is unavailable.

    Args:
        ctx: Current wiring context describing the module set and
             configuration of the system being assembled.

    Returns:
        A tuple of WireSpec instances ready to be applied to a Blueprint.
    """
    # The producer module must be present in the blueprint.
    producer = "ExampleProducerModule"
    if producer not in ctx.names:
        return ()

    specs: list[WireSpec] = []

    for consumer in TEMPLATE_DATA_CONSUMERS:
        if consumer not in ctx.names:
            continue
        specs.append(
            WireSpec(
                out_module=producer,
                out_port="sensor_data",  # Out[T] port on producer
                in_module=consumer,
                in_port="sensor_data",  # In[T] port on consumer
                delivery="dds",  # DDS transport for cross-machine
                topic=TOPIC_TEMPLATE_DATA,  # stable topic for DDS routing
            )
        )

    return tuple(specs)
