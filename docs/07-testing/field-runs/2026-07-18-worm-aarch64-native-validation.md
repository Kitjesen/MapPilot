# Worm AArch64 Native Validation - 2026-07-18

Status: **PASS for isolated aarch64 build, native tests, and no-motion DDS
dataflow. Not field readiness.**

## Run identity

| Field | Value |
| --- | --- |
| Host | `worm@192.168.66.9` (lab LAN compute host) |
| OS | Ubuntu 22.04.5 LTS, Linux 5.15.185-tegra |
| Architecture | aarch64, Cortex-A78AE |
| Test root | `/home/worm/lingtu-test/20260718_contract` |
| Product profile | None; native components were invoked directly |
| Endpoint mode | `teleop` for the startup-only check |
| Product session | None |
| Active map | None |
| Motion output | Disabled with `--publish-cmd-vel false` |
| Persistent services | None; no systemd units were installed or started |

The reduced source bundle used for the run had SHA-256
`99772ac918354a252c301e7fb0cfa9827595590b02bd415ae2ad8f2ee92889c2`.
The separately transferred local-planner path library archive had SHA-256
`c8b843f1a0470625443715f4157b87fee1db81b7e3344ec6a76dedaaa557d066`.

## Host preparation

The native validation required CycloneDDS and OctoMap development packages.
The host already contained the compiler toolchain, Eigen, yaml-cpp, PCL, and a
ROS 2 Humble installation. No LingTu build used ROS CMake packages or ROS
runtime libraries.

Installed for this isolated run:

- `cyclonedds-dev`
- `cyclonedds-tools`
- `libddsc0`
- `liboctomap-dev`
- their required iceoryx runtime packages

## Commands and gates

The canonical native build entry point was executed against the isolated
build directory:

```bash
LINGTU_NAV_ENDPOINT_BUILD_DIR=/home/worm/lingtu-test/20260718_contract/build/nav-endpoint \
LINGTU_BUILD_JOBS=4 \
LINGTU_NAV_ENDPOINT_RUN_TESTS=1 \
bash scripts/build/build_nav_endpoint.sh
```

The DDS dataflow check used an isolated CycloneDDS domain and only mock or
non-actuating processes:

```text
lingtu_motion_mock_dds
  -> /slam/odometry + /tf
lingtu_nav_control cloud
  -> /slam/registered_cloud
lingtu_traversability_dds
  -> traversability + exploration grid
lingtu_explore_dds
  -> consumes the exploration grid
```

The endpoint startup check used another isolated DDS domain, disabled final
command publication, and terminated through a bounded timeout.

## Results

| Gate | Result | Evidence |
| --- | --- | --- |
| Clean aarch64 native endpoint build | PASS | `lingtu_nav_native_endpoint` produced successfully |
| Unified native build script | PASS | Required binaries and shared libraries all present |
| Native endpoint CTest | PASS | 33/33 tests |
| Maps C++ CTest | PASS | 23/23 tests |
| Runtime graph/product-mode focused tests | PASS | 30 tests |
| Typed DDS point-cloud dataflow | PASS | 3 received, 3 fast publishes, 3 slow publishes |
| DDS frame contract | PASS | 0 TF, odometry-frame, or cloud-frame rejections |
| DDS processing health | PASS | 0 reported errors |
| Exploration grid boundary | PASS | 3 grids received; no false `robot_not_in_free_space` |
| Native endpoint startup | PASS | Status snapshots written; no startup failure |
| Fail-closed motion ownership | PASS | `driver_control_missing`; no command publication |
| Online endpoint ROS linkage | PASS | No ROS, RMW, tf2, or ROS message libraries in `ldd` |
| Online endpoint PCL linkage | PASS | No PCL, VTK, or OpenNI libraries in `ldd` |
| Offline PCD converter | PASS | PCL remains linked to `octoplanner3d_pcd_to_octomap` only |

Final online endpoint size was 901,808 bytes after separating offline PCD
conversion from the online planning runtime.

Relevant remote evidence remains under:

```text
/home/worm/lingtu-test/20260718_contract/run/
```

The table preserves the executable name used by that recorded run. The current
source target is `navd`. A follow-up isolated aarch64 Release build from the
current source completed under:

```text
/home/worm/lingtu-test/20260718_navd_runtimeplan/
```

That follow-up produced an ARM64 `navd`, passed `test_nav_endpoint_config`, and
its online `ldd` scan contained no ROS, RMW, tf2, PCL, VTK, or OpenNI library.
Binary SHA-256:

```text
e1f6dc7bd101653dc0441a4885c7f343e44494c88f2cf6c85c0e4db9c67fcdd6
```

The follow-up did not install the binary, start a service, or publish a motion
command. Historical endpoint-startup and motion-safety evidence therefore
remains attached to the original recorded run until `navd` is deployed through
the normal release process and a fresh non-motion startup gate is collected.

No LingTu validation process remained running after the test.

## Defects found and fixed

1. Maps subprocess execution ignored a failed working-directory change.
   `RunShellCommand` now rejects an unavailable directory before launch and the
   child exits if a race still makes `chdir()` fail.
2. A DDS float32 grid resolution could move a mathematically exact world
   coordinate into the preceding cell. TARE world-to-cell conversion now snaps
   values that are within a small numerical tolerance of an integer boundary.
3. Inspection tests were registered with CTest but hidden from the default
   build by `EXCLUDE_FROM_ALL`. They now build with the endpoint test tree.
4. The release build did not require `lingtu_explore_dds`. The native build
   gate now checks that artifact explicitly.
5. Online OctoPlanner runtime linked the offline PCL converter and therefore
   inherited PCL, VTK, and OpenNI. Product endpoint builds now force direct PCD
   input off; the standalone converter keeps the feature.

## Not validated

This run does **not** prove:

- real MID-360 packet timing, LiDAR/IMU synchronization, or packet loss;
- live `lingtu-driver` control leases or Brainstem command forwarding;
- a systemd release install, restart, watchdog, or Gateway readiness path;
- planning against a field map with real localization;
- physical obstacle avoidance, gait stability, braking distance, or motion;
- installation on a clean Ubuntu image with no ROS packages present.

The host has ROS 2 installed. The result proves that the produced LingTu
endpoint has no ROS linkage; a clean-image deployment test is still needed to
prove package-level installation independence.

## Next field gate

Use the real robot release with a non-motion startup first. Require fresh
DriverControl, odometry, TF, registered cloud, and traversability evidence in
Gateway readiness before enabling `teleop_avoid` or autonomy motion.
