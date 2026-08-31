# Message contracts

This package describes LingTu's native DDS wire contract:

- `idl/messages.idl` is the native DDS schema used by C++ field processes.
- `topics.py` exposes metadata to Product resolution and diagnostics; it does
  not define Python DDS payload classes.
- `cpp/topics.hpp` and `cpp/qos.hpp` are the native topic and QoS catalogue.
- the remaining small C++ headers contain command/status protocol values.

The IDL covers sensors, localization, maps, navigation, teleop, inspection,
and driver status.

In-process Python messages belong in `runtime.msgs`. Native robot and
simulation processes use `idlc`-generated C types with the CycloneDDS C API.
There is no CycloneDDS-Python message mirror or generic Python DDS transport.
