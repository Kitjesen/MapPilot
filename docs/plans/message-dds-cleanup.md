# Message/DDS cleanup

Status: active cleanup plan

Goal: keep one production DDS implementation: CycloneDDS C API with IDL-generated C types.

- Keep the native topic catalogue, QoS builder, IDL, and small command enums.
- Rename native headers to `topics.hpp` and `qos.hpp`; QoS uses topic constants instead of repeating wire names.
- Replace Python DDS type loading with metadata-only topic descriptions.
- Delete `message/dds_types`, the Python DDS transport/endpoint/localization path, and their dedicated tests.
- Keep Product endpoint contracts as metadata; real/sim processes continue to use native DDS.
- Verify metadata imports plus representative native builds. No Rust, Dart, new dependency, compatibility shim, or checksum is added.
