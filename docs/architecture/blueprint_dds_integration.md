# Blueprint and DDS

Status: current architecture note

Blueprint wires Python Modules inside one Host with direct callbacks.

DDS sits outside Blueprint:

```text
Python Host Modules -- local callbacks
Native lidar/slam/maps/nav/driver -- CycloneDDS + IDL-generated C types
```

`ProductControl` resolves and starts the native processes. The Host receives
native status through its explicit adapters and sends navigation/operator
commands through the native client library. It does not instantiate a generic
CycloneDDS-Python transport.

Sources of truth:

- logical topics: `runtime.runtime_interface.TOPICS`
- Host-readable native metadata: `message/topics.py`
- native topic/QoS catalogue: `message/cpp/topics.hpp`, `message/cpp/qos.hpp`
- wire schema: `message/idl/messages.idl`
- Product endpoint contract: `runtime/endpoints/dds/contracts.py`

This deliberately leaves no Python DDS payload mirror, endpoint runner,
Python worker router, or fallback DDS localization adapter.
