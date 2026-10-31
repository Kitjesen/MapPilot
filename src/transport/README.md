# transport/ - Native DDS transport

`dds/profile.hpp` declares the named QoS profiles and `dds/qos.hpp` applies
them to CycloneDDS endpoints. Native process adapters share these headers;
topic metadata and IDL remain owned by [`message/`](../message/README.md).

This directory belongs to the adapter layer declared in
[`architecture_layers.yaml`](../../config/architecture_layers.yaml). It does
not own Product policy, process lifecycle, or navigation algorithms.
