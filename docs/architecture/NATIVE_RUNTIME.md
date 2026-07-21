# Native Runtime

Status: current product boundary contract
Audience: native runtime, deployment, Gateway operations, and field-readiness maintainers
Replaced by: not replaced

This document separates three things that were being mixed together:

- C++ real-time product components;
- product-native map optimization commands;
- Gateway/Web API routes that only trigger or observe runtime actions.

`src/gateway/routes/operations.py` belongs to the third category. It exposes
HTTP routes such as SLAM status, restart, runtime switch, relocalization,
visual-servo operations, and WebRTC/memory/diagnostic operations. It must not
own LiDAR ingestion, SLAM, PGO, HBA, or navigation compute.

## Canonical Folders

| Folder | Owns |
| --- | --- |
| `src/drivers/real/lidar/sdk2_stream/` | Livox SDK2 C++ ingestion and optional CycloneDDS publication. |
| `src/localization/slam/cpp/` | Native SLAM contract, Fast-LIO/Point-LIO wrapper, DDS runtime, SLAM control, native relocalization support. |
| `src/nav/cpp/endpoint/` | Native navigation endpoint, traversability DDS, nav control, and robot command publication gate. |
| `src/localization/pgo/` | Legacy ROS2 PGO node wrapper and historical algorithm code. |
| `src/localization/hba/` | Legacy ROS2 HBA node wrapper and historical algorithm code. |
| `src/kernels/slam/pose_graph_opt/` | Portable pose-graph optimization kernel with C ABI used by PGO/HBA code. |
| `src/native/runtime/` | C++ manifest of product-native runtime component ownership. This is a boundary registry, not an algorithm home. |
| `src/localization/opt/` | Product-facing non-ROS map optimization entrypoints. |

## Current Implementation Status

| Component | Current status | Evidence |
| --- | --- | --- |
| LiDAR DDS | Product-native C++ implemented. | `livox_sdk2_stream` in `src/drivers/real/lidar/sdk2_stream/`; deployed as `lingtu-livox-dds.service`. |
| SLAM DDS | Product-native C++ implemented. | `lingtu_slam_cyclone_runtime` and `lingtu_slam_control` in `src/localization/slam/cpp/`; deployed as `lingtu-slam-dds.service`. |
| Relocalization | Product-native support exists in the SLAM C++ contract, with optional BBS3D/small_gicp build support. | `native_relocalizer.cpp` in `src/localization/slam/cpp/`. |
| Navigation DDS | Product-native C++ implemented. OctoPlanner3D is default; FAR is an explicit validated-occupancy option. | `navd`, `lingtu_nav_control`, `lingtu_traversability_dds` in `src/nav/cpp/endpoint/`; deployed as `lingtu-nav-dds.service` and `lingtu-traversability-dds.service`. |
| Thunder driver | Product-native C++ implemented. | `lingtu_driver` in `src/drivers/real/thunder/native/`; deployed as `lingtu-driver.service`, conflicts with the old Python endpoint service, consumes `rt/nav/cmd_vel`, and calls remote Brainstem `WalkChecked`. |
| Saved-map loop verification | `wip`: deterministic native shadow verifier exists; no map/pose mutation and no PGO feed yet. | `lt_loop_verify` emits versioned constraints/diagnostics, validates exact pose-patch provenance, and rejects changing inputs. |
| PGO | `exp`: binary and solver exist; product save defaults to `off`. | `lt_pgo` refuses to rebuild a map when the graph has no independent geometric constraints and reports `skipped_no_independent_constraints`. |
| HBA | `exp`: binary and solver exist; product save defaults to `off`. | `lt_hba` currently shares the same constraint gap; it is not a supported high-quality product action yet. |

## States

| State | Meaning |
| --- | --- |
| `ready` | Can be used in the current product path. |
| `wip` | Code exists, but there is no supported product command/UI path yet. |
| `legacy` | Old compatibility path only. |
| `exp` | Experimental, not product. |

## PGO/HBA Product Boundary

PGO/HBA are no longer hidden behind ROS2 services. Their experimental boundary
is save-time and file-based:

1. read saved-map poses/cloud patches without ROS topics;
2. keep optimization disabled by default;
3. when explicitly invoked, require independently verified geometric
   constraints; an odometry-only graph is reported as skipped and must not
   rewrite `poses.txt` or `map.pcd`;
4. only a genuinely performed optimization may write optimized artifacts and
   `map_optimization.json`;
5. expose `performed`, `status`, and reason fields to Gateway/Web.

## Save-Map Optimization Policy

PGO/HBA should be used through the save-map pipeline first:

```text
Save Map
  -> validate map.pcd / poses.txt / patches
  -> optimization disabled by default
  -> optional PGO only with independent geometric constraints
  -> HBA remains experimental and is not a product save action
  -> rebuild planner artifacts
```

Fast-LIO remains the realtime front end. A future verified PGO backend could run
as a separate low-rate mode, not as a hidden side effect in the 10 Hz SLAM
loop. HBA must remain outside the product path until it has an independent
constraint source and field-quality evidence.

## Native Optimization Runner Files

These files are the non-ROS experimental entry surfaces for PGO/HBA work. The
solver calls exist, but the current saved-map inputs do not provide independent
geometric constraints. The product save path therefore keeps them disabled by
default and fails closed when optimization is required:

| File or target | Purpose |
| --- | --- |
| `src/localization/opt/map.hpp` | Shared non-ROS artifact contract for `map.pcd`, `poses.txt`, and `patches/*.pcd`. |
| `src/localization/opt/map.cpp` | Shared artifact validation implementation. |
| `src/localization/opt/graph.hpp` | Shared save-time pose graph and PCD patch API. |
| `src/localization/opt/graph.cpp` | Calls the portable pose graph optimizer, rebuilds map artifacts, and writes reports. |
| `src/localization/opt/loop_constraints.hpp` | Shadow loop options, diagnostics, constraint, and report contract. |
| `src/localization/opt/loop_constraints.cpp` | Gravity-aligned descriptor, deterministic 4DoF verification, point-to-plane observability/false-loop gates, bounded candidate discovery, and immutable-input audit. |
| `lt_loop_verify` | Read-only saved-map loop verifier; report path must be outside the map directory. |
| `src/localization/opt/pgo.cpp` | Experimental opt-in PGO runner. |
| `src/localization/opt/pgo.hpp` | PGO runner interface. |
| `src/localization/opt/hba.cpp` | Experimental HBA runner; no product quality claim. |
| `src/localization/opt/hba.hpp` | HBA runner interface. |
| `lt_pgo` | Experimental save-time PGO binary, disabled by default. |
| `lt_hba` | Experimental HBA binary, outside the product save path. |

Do not place those algorithms in `gateway/`, `runtime/service_catalogs/`, or
Web code. Gateway may call them through explicit operation routes after the
native binaries exist.

## How To Use

Start a complete product through the Runtime Graph control plane:

```bash
bash scripts/lingtu nav start <map>
```

`scripts/lingtu` resolves the selected Product and Endpoint into a typed
`RuntimePlan`, then starts the declared processes in dependency order. Direct
`systemctl` calls are reserved for service-level diagnosis; they are not a
second product startup contract.

`thunder_field` is an `endpoint_only + driver` product endpoint:
`config/runtime_graph/endpoints/thunder_field.yaml` sets
`localization_adapter=cpp_slam_status`,
`command_output_mode=endpoint_only`, and
`hardware_control_boundary=driver`. The deployed `lingtu.service` also sets
`LINGTU_ENABLE_ROBOT_DRIVER=0`, so the Python process does not open a second
robot hardware writer.

`lingtu-thunder-dds-endpoint.service` remains in the repository as a
compatibility Python endpoint, but `lingtu-driver.service` declares a systemd
conflict against it. It is not part of the default field product chain.

To inspect native component state from a local build:

```bash
lt_native
```

PGO/HBA are wired as opt-in save-map experiments. The product default is
`LINGTU_MAP_OPT=off`. The current frontends do not yet generate verified loop
or other independent geometric factors, so invoking `lt_pgo`/`lt_hba` on only
saved odometry poses safely returns a skipped result. They become product-ready
only after place recognition plus geometric validation feeds real constraints
and the corrected pose/map artifacts are committed atomically.

The first independent-constraint milestone is available only in shadow mode:

```bash
lt_loop_verify \
  --map /home/sunrise/data/nova/maps/<map_name> \
  --report /tmp/<map_name>-loop-$(date +%s).json
```

Exit `0` means the audit ran successfully, not that a loop was necessarily
accepted. Read `code`, `candidate_count`, `accepted_constraint_count`,
`consensus_support_count`, candidate rejection reasons, exact `options`, and
the input fingerprints. `shadow_verified_loops` is still not authorization to
run `lt_pgo`: v3 reports point-to-plane eigenvalues and the weakest mode, but
the emitted graph information diagonal is deliberately all-zero and marked
`not_graph_compatible`. The missing full 6x6 tangent-frame conversion and map
transaction path have not passed moving MID-360 field acceptance.

Build note for robots: `lt_pgo`/`lt_hba` link `pose_graph_opt` as a static
library. The CMake target can build that library with `cargo` on development
machines, but field robots should receive a prebuilt aarch64
`liblingtu_pose_graph_opt.a` and configure with:

```bash
cmake -S src/native/runtime -B build/native-runtime \
  -DLINGTU_POSE_GRAPH_OPT_LIB=/path/to/liblingtu_pose_graph_opt.a
```

If neither the prebuilt library nor `cargo` is available, configure fails
explicitly instead of producing a half-wired optimizer.
