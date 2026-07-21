# FAR Native Endpoint aarch64 Validation

- Date: 2026-07-20
- Target: `worm@192.168.66.9`
- Architecture: Ubuntu GNU/Linux, ARM aarch64
- Scope: build and non-motion tests only

## Build

The source snapshot was built with the canonical entry point:

```bash
LINGTU_NAV_ENDPOINT_BUILD_DIR=/home/worm/lingtu-test/20260720_far_product/build/nav-endpoint \
LINGTU_BUILD_JOBS=4 \
LINGTU_NAV_ENDPOINT_RUN_TESTS=1 \
bash scripts/build/build_nav_endpoint.sh
```

Result: `navd` and all required native helper binaries built successfully.
The output is an ARM aarch64 ELF executable.

## Tests

CTest result: **36/36 passed** in 14.19 seconds.

The mandatory FAR coverage passed:

- `test_nav_endpoint_config`
- `test_active_occupancy_gate`
- `test_far_planner`
- `test_far_c_api`

The same run also passed NavLoop, path follower, local planner, teleop safety,
active OctoMap, inspection, DDS message, and OctoPlanner3D smoke tests.

## Binary Boundary

`navd` links system OctoMap, OpenMP, CycloneDDS, standard C/C++ libraries, and
the optional iceoryx transport. It does not link ROS 2, RMW, tf2, PCL, VTK,
OpenNI, or checked-in architecture-specific libraries. PCL remains limited to
the standalone offline PCD converter target.

No `navd`, exploration, or traversability process remained running after the
test. No systemd service was installed or started, and no motion topic was
published.

## Release Package Gate

After adding the product install manifest, the same source tree was
reconfigured and rebuilt on the target. The second CTest run passed **36/36**
in 14.17 seconds. `bash -n scripts/deploy/cut_release.sh` also passed.

An isolated `cmake --install` to
`.install/nav-endpoint-20260720-2` produced the runtime layout consumed by the
systemd units:

- `navd`
- `lingtu_traversability_dds`
- `lingtu_explore_dds`
- `lingtu_nav_control`
- `lingtu_motion_mock_dds`
- `liblingtu_nav_client.so`
- `liblingtu_inspection_evidence_bridge.so`
- `inspection/liblingtu_inspection.so`
- optional FAR and OctoPlanner3D libraries/tools

The installed `navd` dependency scan again found no ROS, RMW, tf2, PCL, or VTK
runtime link. This was an isolated package validation only; no release symlink,
systemd unit, DDS writer, or actuator command was touched.

## Native Exploration Lifecycle Gate

The native tree was rebuilt after adding the typed exploration lifecycle
boundary. The field endpoint now starts idle and accepts `START`, `PAUSE`,
`RESUME`, and `STOP` through `ExplorationCommandRequest`; every request returns
an `ExplorationCommandAck`. Planning remains gated until a valid `START` has
been acknowledged.

Focused CTest passed **4/4**:

- `test_nav_client`
- `test_explore_control`
- `test_explore_input`
- `test_tare_policy`

The complete native suite then passed **39/39** in 14.23 seconds. This included
FAR, TARE, endpoint control authority, navigation loop, local planning,
inspection, OctoPlanner3D smoke tests, and the DDS request/ack client test.

`ldd` was checked for `lingtu_explore_dds`,
`lingtu_traversability_dds`, and `test_nav_client`. The runtime boundary contains
CycloneDDS, standard C/C++ libraries, OpenMP where required, and optional
iceoryx libraries. It contains no ROS 2, rclcpp, rcutils, PCL, or Open3D link.

The matching local product-contract suite passed **287 tests** with one
intentional skip. It covered topic/QoS contracts, profile and runtime graph
resolution, Gateway native command behavior, stale-status failsafe stop, and
the Python-to-C-ABI command adapter. No endpoint service or robot motion process
was started during this validation.