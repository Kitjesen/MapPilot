# Map Optimization

## Module and document boundary

This directory contains the map-optimization source and its native build
targets. Its Markdown files are documentation, not runtime inputs:

- `README.md` is the short, current module contract: ownership, public targets,
  and the commands that remain supported.
- [迁移记录](../../../docs/research/localization-opt-migration.md) is a historical migration and acceptance record. It preserves
  past decisions, measurements, warnings, and field evidence; changing it does
  not change the running system.
- [September handoff](../../../docs/operations/localization-opt-handoff-20260917.md) records the 2026-09-17 migration context.
  Current delivery priorities belong to the [Roadmap](../../../docs/roadmap.md#current-delivery-cycle).

When these documents disagree with CMake or code, use the code and the current
tests as the source of runtime behavior. Keep product policy and deployment
configuration in `config/` and `docs/`; do not make an executable import a
Markdown file.

`src/localization/map_optimization` is the product-facing C++ entry surface for map
optimization. Online mapping now uses the pinned native LIO-SAM backend in
[`../online_mapping`](../online_mapping/upstream/UPSTREAM.md). Field release `.71` uses this backend; ARM integration checks pass, while
new field loop-closure accuracy remains to be measured.

The 2026-09-25 candidate splits the online target into `online/CMakeLists.txt`.
`slamd` links `lingtu_online_mapping` without Cargo or the legacy Rust optimizer.
The separate `map_optimization` build still owns `lt_pgo` and its existing offline contracts.
`poses.hpp` and `cloud_io.cpp` hold solver-independent saved-data types/readers.
New loop verification, evidence and performance changes are documented in
[`../online_mapping/upstream/UPSTREAM.md`](../online_mapping/upstream/UPSTREAM.md); they are not a claim
that this candidate has been deployed.

Short names are intentional:

| File | Role |
| --- | --- |
| `map.*` | Resolve and check saved-map artifacts: `map.pcd`, `poses.txt`, `patches/*.pcd`. |
| `graph.*` | Legacy offline pose graph and bundle writer used by PGO. |
| `poses.hpp`, `cloud_io.cpp` | Solver-independent pose types and saved pose/PCD readers. |
| `gravity_graph.*` | GTSAM batch LM over XYZ and yaw with the original LIO gravity held fixed. |
| `io/` | Independent shared build target for the pose/PCD readers. |
| `../online_mapping/worker.*` | Bounded online queue, loop evidence and map preview reconstruction. |
| `pose_math.hpp` | Shared, explicit `T_parent_child` pose algebra used by loop verification. |
| `cloud.hpp` | Portable PCD input contract shared by map optimization and loop verification. |
| `loop_constraints.*` | Deterministic saved-map loop candidate, 4DoF verification, and audit report. |
| `assembly.*` | Build a complete measured graph from adjacent patches plus verified loops. |
| `loop_cli.cpp` | Read-only `lt_loop_verify` verification command. |
| `pgo.*` | ROS-free save-time pose graph optimization runner. |
| `constraints.*` | Strict dependency-free parser for independent full-information constraints. |

Runtime rule:

- the former ROS2 PGO and HBA packages remain deleted;
- `lt_pgo` is a short-lived native SaveMap helper, not a resident service;
- LTLIO freezes `map.pcd`, `poses.txt`, body-local `patches/*.pcd`, and
  `patch_bundle.manifest`; it does not publish `pose_graph.constraints`;
- `lt_pgo --auto-constraints` measures every adjacent single-patch edge, retains
  accepted edges after a rejection, checks up to three recent disconnected
  neighbors for a measured bridge, and runs loop verification once. Optimization
  requires all saved nodes to be connected by measured edges and at least one
  verified loop; a loop may bridge a missing adjacent edge;
- `lt_loop_verify` reports verified constraints but never calls PGO
  and never rewrites `map.pcd`, `poses.txt`, or patches;
- a disconnected measured graph or absence of verified loops is a structured
  successful skip: no optimization output or partial constraint file is left;
- `performed` distinguishes an optimized bundle from that successful skip.
  `sequential_chain_incomplete` means the graph remains disconnected after loop
  verification, not that the first adjacent rejection stopped the assembly;
- `pose_graph.constraints` is an atomically written, strictly re-read private
  optimizer input; automatic mode deletes it before publishing any output;
- Gateway/Web do not implement optimization logic or expose an optimizer switch.

## Online backend migration

`online_mapping::Worker` retains the bounded queue, epoch reset, background-worker and
snapshot contracts. Algorithm work now calls the native LIO-SAM extraction:
continuous raw LIO Pose3 between factors, GTSAM iSAM2, temporal/spatial loop
candidates, PCL ICP against a historical submap, and updates of all key poses.
It no longer runs LingTu's four-DoF adjacent registration or descriptors online.
Sparse clouds do not disconnect a valid continuous odometry chain. A backend
exception stops that worker generation and prevents a corrected save.

Raw LIO deltas are measurements from the frontend; their fixed noise parameters
are explicitly inherited upstream defaults, not statistically calibrated
relative covariances. The candidate uses the upstream six-DoF model rather than
combining it with the previous custom fixed-gravity factors. Preserve and
validate this distinction in replay and field evidence.

Snapshots retain the existing continuous-odometry publication frame: optimized
history is rigidly expressed at the latest odometry anchor; live odometry is not
mutated. Preview geometry is invalidated when iSAM2 revises old poses, including
relinearization without a newly accepted loop. Saving captures the same poses
and full-resolution body patches. It preserves `poses.raw.txt`,
`trajectory.raw.txt`, and `keyframes.timestamps.txt` for independent replay.
A complete `lio_sam_isam2` snapshot is already optimized: mapd preserves its report
and does not run the legacy save-time PGO on it. Explicit offline constraints
conflicting with that snapshot are rejected instead of mixing two models.

The explicit offline `lt_pgo` and old-map `--auto-constraints` paths use the
saved-map graph utilities and Rust kernel. The unused `OnlinePoseGraph` interface
and its standalone test were removed. Online SLAM uses `online_mapping::Worker`
and `Backend`; the offline tools do not link that worker or its PCL dependency.
Offline gravity optimization still requires GTSAM. Shared point-cloud and pose
reading is provided by `lingtu_mapping_io`.

`lt_mapping_replay` requires original LIO poses and recorded keyframe timestamps.
Previously optimized `poses.txt` is not a fallback. Old 903room_v2 lacks those
inputs and cannot establish end-to-end equivalence for this new algorithm.

Native commands:

```bash
lt_pgo --map MAP_DIR --out NEW_BUNDLE_DIR --constraints CONSTRAINTS.txt
lt_pgo --map MAP_DIR --out NEW_BUNDLE_DIR --auto-constraints
lt_loop_verify --map MAP_DIR --report /tmp/loop-$(date +%s).json
```

### Windows simulation build

The saved-map helper is a separate process. Build it against a private static
GTSAM SDK to avoid the shared Windows SDK's exported STL template symbols
colliding with the optimizer's own instantiations. This does not change the
shared SDK or the binaries used by `slamd`, and does not suppress linker errors.
Both SDKs use the version in `scripts/build/locks/gtsam.json`.

From the repository root, with the existing Visual Studio, CMake, Rust cache,
GTSAM source tree and Windows Eigen/PCL dependencies available:

```powershell
./scripts/build/prepare_gtsam_windows_static.ps1
./scripts/build/build_map_opt_windows.ps1
```

These commands do not download dependencies. The SDK goes to
`build/deps/gtsam-<version>-windows-static-sdk`; the current binaries go to
`build/map_optimization/Release`. The build script stages their actual PE DLL
dependencies beside the executables, then runs `pgo_core`, `loop_information`,
`constraint_assembly`. Logs and JUnit output are in the
optimizer build directory. Windows simulation declares this optimizer in its
map process environment and artifact dependencies; it does not inherit a
shell-provided optimizer or search path. Linux/field deployment is unchanged.

For the native map transaction integration test, configure `src/maps` with
OctoMap, build `lingtu_maps_real_pgo_save_reload_test`, then run:

```powershell
./build/maps-pgo-save-windows/Release/lingtu_maps_real_pgo_save_reload_test.exe `
  ./build/map_optimization/Release/lt_pgo.exe
```

An optional second argument selects a new evidence directory. Setting
`LINGTU_MAPS_TEST_PGO_EXECUTABLE` to the absolute current optimizer path when
configuring `src/maps` also registers this test with CTest. The test uses a
synthetic closed trajectory, the real optimizer, native SaveMapEngine and
OctoMap builder; it reopens the map store, validates/activates the saved payload
in that isolated test store, and checks that a failed replacement preserves
the previous map. Static fixture cleanup is disabled explicitly. It is not a
Product session, live SLAM capture, navigation run, or factory qualification.

`lt_pgo` reads `poses.txt`, `patches/*.pcd`, and an explicit independent
constraint file containing the complete factor graph (including adjacent
odometry and loop factors), calls the portable `src/kernels/slam/pose_graph_opt` C ABI,
and writes a new self-contained bundle containing `map.pcd`, `poses.txt`,
`patches/`, `patch_bundle.manifest`, and `map_optimization.json`. In-place output
and any output below the source map directory are rejected. The executable is
included in the native release; no systemd unit or DDS endpoint is created for it.

`poses.txt` provides initial estimates only. PGO fixes pose 0 as the sole gauge
anchor and never turns pose differences into graph weights. In automatic mode,
each adjacent factor is instead measured from its two local patches with the
same bidirectional trimmed 4DoF registration and point-to-plane information
model used by loop verification. Its information comes from the measured
`J^T J / sigma^2` and is converted to the body-right tangent. Each input factor
carries `T_from_to` and a full packed
upper-21 information matrix in the right tangent order
`[omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z]`.

`lt_loop_verify` is deliberately separate. It requires one basename-only,
unique `patch_name` per `poses.txt` row, exact pose/patch cardinality, finite
unit quaternions, and a report path outside the source map directory. Reports
are written through a same-directory temporary file and published by rename;
an existing report is never overwritten.

The verifier uses:

1. planar XY candidate radius plus independent Z and traveled-path gates;
2. local multi-patch submaps transformed into gravity-aligned anchor frames;
3. directional descriptors with distinct yaw-peak and historical-place
   ambiguity margins;
4. deterministic trimmed 4DoF ICP in gravity coordinates, converted back to
   body-frame `T_from_to`;
5. forward/reverse independent registration; every yaw hypothesis must pass
   overlap, residual, span, and observability gates before coverage-first
   ranking, so a low-overlap repeated fragment cannot win on RMSE alone;
6. target-normal point-to-plane Fisher/Hessian observability gates that reject
   single-wall and open-corridor sliding modes; accepted measurements carry a
   full packed 6x6 right-tangent information matrix;
7. consensus requiring different history and current anchors that advance in
   the same direction; one current scan matching two adjacent history scans
   is not independent evidence;
8. bounded spatial-history sampling and hard physical parameter limits before
   descriptor/ICP work, plus pre/post fingerprints so a changing map is
   rejected.

The implementation is independent and informed by public loop-closure
literature; it does not copy STD/LTA-OM code or claim their closed-source
Fast-LIO2-BA implementation. Automatic assembly now produces a complete graph
only from measured adjacent factors and accepted loop factors; it never uses a
fixed information matrix or a keyframe covariance substitute. Field replay,
false-loop, trajectory-quality, and target-platform performance acceptance remain
outstanding.
