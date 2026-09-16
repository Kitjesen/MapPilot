# Message contracts

This package describes LingTu's native DDS wire contract:

- `idl/messages.idl` is the umbrella schema; the domain IDL files beside it
  define messages in the same style as ROS message packages.
- `topics.py` and `generated/topics.hpp` are generated from
  `src/message/topics/*.yaml`.
- `idl/constants.idl` owns wire integer values. `generated/enums.py` and
  `generated/enums.hpp` expose the same values in Python and C++.
- `generated/schema.py` describes IDL fields for diagnostics; it is not a
  second payload implementation. `protocol/*.hpp` contains C++ semantic helpers.
- `transport/dds/qos.hpp` implements the named DDS QoS profiles.

The IDL covers sensors, localization, maps, navigation, teleop, inspection,
and driver status.

## Add or change a message

1. Put the struct in the owning domain IDL, for example
   `idl/navigation.idl`; shared geometry and stamped primitives belong in
   `idl/common.idl`.
2. Use `PascalCase` for message names, `snake_case` for fields, and encode units
   in ambiguous numeric names (`timeout_s`, `speed_mps`, `stamp_ns`).
3. Bind the logical Topic, DDS wire name, message type, and QoS once in the
   matching `src/message/topics/<domain>.yaml` file.
4. Run `python tools/generate_topic_contracts.py` and commit its generated
   Python, C++, and umbrella IDL outputs with the source change.

For a new command or state, declare its integer in `idl/constants.idl` and
document the corresponding field in its domain IDL. Do not renumber existing
values. The fields stay `long`: generating language enums does not change DDS
layout or introduce a new wire enum type.

Run `python tools/generate_topic_contracts.py --check` to detect stale generated
files. `tests/runtime/test_message_enum_generation.py` freezes the existing
wire values and compiles the C++ protocol consumers when a compiler is present.

Do not add a second language-owned Topic table. Product and Env files may
reference Topic names to declare wiring, but they do not redefine type or QoS.

The robot route reads DDS transport and single-writer declarations directly
from the topic catalogue. Product-required topics come from RunPlan; a global
DDS endpoint list does not decide which topics a particular Product needs.
Deployment checks use the generated `message.topics.topic_spec` view.

## Where to look

| Question | Owner |
| --- | --- |
| Which topics exist, and who publishes/subscribes? | `topics/<domain>.yaml` |
| What fields and numeric states cross a process boundary? | `idl/<domain>.idl`, `idl/constants.idl` |
| Which topics does this Product need? | `config/runtime_graph/products/*.yaml` |
| Which process implements a role in real/sim? | `config/runtime_graph/envs/*.yaml` |
| Which coordinate frame or calibration applies? | `runtime/tf/frames.py`, `config/robots/*/*/robot.yaml` |

`diagnostics/runtime_contract.py` only projects diagnostics. Explicit legacy
adapter mappings live in `runtime/adapters/topics.py`. Do not add message fields, topic bindings, Product modes, or
physical mounting numbers there.

`message/catalog.py` is the only YAML reader used by the generator, runtime
routes, and Product graph. It validates topic declarations without importing
Host runtime or Product assembly.

In the diagnostics manifest, canonical topic formats now use fully qualified
IDL names (for example `lingtu.dds.FinalVelocityCommand`). `required_fields`
lists actual IDL fields, not adapter-flattened dictionary keys; topic frame
rules remain in the separate frame projection. Legacy ROS formats appear only
on explicitly declared adapter aliases.

In-process Python messages belong in `runtime.msgs`. Native robot and
simulation processes use `idlc`-generated C types with the CycloneDDS C API.
There is no CycloneDDS-Python message mirror or generic Python DDS transport.

## Python boundary

The message path has three explicit layers:

1. `src/message/idl/<domain>.idl` owns the DDS wire schema.
2. `runtime/endpoints/dds/adapters.py` converts native DDS payloads at the Host seam.
3. `runtime.msgs` owns Python objects used by `Module` ports.

Do not decode DDS dictionaries inside a Module loop. Add the domain conversion
to the DDS adapter and keep transport ordering, deduplication, and publication
in the consuming endpoint Module.
