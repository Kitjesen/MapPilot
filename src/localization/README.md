# Localization

`src/localization` is the localization domain. It owns pose sources and localization health, not navigation decisions.

The current Product dataflow, control commands, complete DDS topic inventory,
relocalization/loop-closure status, and SaveMap boundary are documented in
[`docs/runtime.md`](../../docs/runtime.md).
The current mapping/localization data owners and their remaining limits are
explained in [mapping responsibilities](../../docs/research/mapping-responsibilities-20261102.md).

## Responsibilities

- Native `slamd` contract: pose, registered cloud, map observation/cloud,
  saved map, localization health, optional GNSS capability diagnostics,
  map->odom, and scene mode.
- LTLIO LiDAR-inertial odometry and saved-map relocalization adapters.
- Relocalization adapters and saved-map alignment helpers.
- Native GNSS/localization fusion contracts.
- Runtime outputs used by navigation: odometry, map cloud, map->odom transform, localization quality, localization status.

## Boundaries

- Navigation decisions stay in `src/nav`.
- Sensor device ownership stays in `src/drivers`.
- Localization has no ROS2 message/service package. Product process boundaries
  use native typed DDS.
- SLAM does not publish navigation paths or velocity commands. Historical pose
  tracks may be saved as `poses.txt`, but `global_path`, `local_path`,
  `waypoint`, and `cmd_vel` belong to `src/nav`.
- No-ROS localization implementations attach behind `localization/slam/cpp`
  through the `ISlamBackend` contract.

## How to read this directory

The localization tree is split by ownership and lifetime, not by estimator
brand. There is one production LIO core; the other directories provide the
service seam around it or separate map and relocalization jobs.

| Path | Kind | Owns | Does not own |
| --- | --- | --- | --- |
| `ltlio/` | algorithm library | LTLIO state estimation, scan/IMU update, elevator transport mode, and frontend configuration | DDS, Product lifecycle, saved-map activation, navigation decisions |
| `leg_odometry/` | local measurement diagnostics | externally supplied leg-chain FK/Jacobian and contact-velocity residuals with explicit point, time and contact assumptions | contact detection, measurement covariance, production filter updates, DDS or another pose estimator |
| `slam/cpp/` | field service adapter | `slamd`, DDS input/output, backend lifecycle, status, map/odom publication, save/load and relocalization wiring | A second filter or an independent pose solver |
| `online_mapping/` | online map backend | online queue, GTSAM/iSAM2 loop backend and corrected preview | sensor ownership, live LTLIO odometry, navigation paths |
| `map_optimization/` | map optimization and shared file I/O | save-time PGO, loop verification, map-bundle tools and point-cloud/pose readers | online scheduling, DDS topics, field motion |
| `relocalization/` | relocalization library | seeded/global registration and ICP refinement used by the service adapter | continuous odometry or map persistence |
| `adapters/` | Python API adapter | normalized localization status and native endpoint calls | estimation, map geometry, or transport policy |

The runtime path is therefore:

```text
drivers -> slam/cpp -> ltlio -> localization status -> nav/maps
                         \-> online_mapping (optional online map correction)
saved map -> relocalization -> slam/cpp map->odom commit
saved map -> map_optimization (short-lived save-time job)
```

The online scheduler and backend live together in `online_mapping/`. The SLAM
service keeps the selected, deskewed body-frame keyframes for saving; these are
pre-graph-correction scans, not unfiltered LiDAR packets. The scheduler holds pending
work and the backend retains scans needed for loop detection. Replay and the
offline optimizer share the point-cloud and pose file reader without making the
offline optimizer depend on the online worker.

Saved scan patches require a 0.20 m XYZ translation or 5 degree full-rotation
threshold, together with `patch_min_interval_s` (default `1.0` s). The interval
limits recording/save memory but can omit observations needed for saved-ray
coverage. Explicit `0` enables spatial-only sampling for development; complete
long-map saves at that density remain unqualified because of save-time memory.
Stationary noise below the spatial thresholds does not
produce additional patches. The default 3,000-keyframe capacity stops adding
mapping keyframes while continuous odometry continues; saving preserves the
recorded prefix rather than claiming later scans are in the saved map. Saved patches, raw poses,
timestamps, graph identities and calibrated scan origins keep the same path.
These sampling bounds improve retained observations; they do not guarantee that
every surface in the fused PCD has a saved ray or that a route is navigable.

`slam/cpp` is the field SLAM process boundary. `ltlio` is the production LIO
frontend; the online graph and relocalization have their own numerical algorithms.
`online_mapping`, `map_optimization`, and `relocalization` are bounded collaborators with
different lifetimes; merging them into `ltlio` would make the core own DDS,
disk, or loop-closure policy and would recreate the coupling this layout is
meant to remove.

For changes, first decide which question the code answers:

1. How is the pose estimated from a scan and IMU? `ltlio/`.
2. How does the field process receive and publish typed data? `slam/cpp/`.
3. How are keyframes and loops optimized while mapping? `online_mapping/`.
4. How is a saved bundle optimized or audited after capture? `map_optimization/`.
5. How is an existing map aligned during relocalization? `relocalization/`.
6. Does a supplied leg model and aligned contact sample imply a consistent body velocity? `leg_odometry/`, currently local validation only; implementation and remaining sensor prerequisites are recorded in [D3](../../docs/design/slam-scene-adaptation.md#d3-分阶段接入与-d2-的依赖).

If a change answers more than one of these questions, keep the algorithm in
its owner and connect it through the existing contract instead of moving files
into a new shared or generic directory.

## Naming policy

The product vocabulary uses responsibility names:

| Product name | Current technical path | Rename rule |
| --- | --- | --- |
| LTLIO frontend | `ltlio/` | Keep the product algorithm name. |
| SLAM service | `slam/cpp/` | Keep the native service boundary. |
| Online mapping | `online_mapping/` | Pinned LIO-SAM-derived graph backend and its online queue/snapshot worker. |
| Map optimization job | `map_optimization/` | Save-time PGO, loop-verification executables and shared file I/O. |
| Relocalization library | `relocalization/` | Owns request-scoped BBS3D/ICP alignment and `relocalization_cli`. |

New files should use these responsibility names in prose and include the
matching physical path when they link to one of these libraries.

## Main Files

| Path | Category | Role |
| --- | --- | --- |
| `service.py` | public API | Backend-neutral relocalization result, protocol, and stable `Localization` capability. |
| `adapters/relocalization.py` | native adapter | Calls the C++ CycloneDDS SLAM control tool; no ROS dependency. |
| `adapters/status.py` | native adapter | Converts the native SLAM endpoint status stream into localization-domain messages. |
| `adapters/resolver.py` | composition adapter | Resolves the explicit localization adapter selected by Product/env resolution; it never guesses a transport. |
| `slam/cpp/` | realtime | ROS-free C++ `ISlamBackend`, DDS runtime, SLAM control, native relocalization hooks. |
| `ltlio/` | algorithm asset | LTLIO supplies the native Product backend. |
| `relocalization/` | relocalization | Single ROS-free ICP/BBS3D library and offline `relocalization_cli`; `slamd` owns the live DDS runtime. |

Product Hosts identify the external runtime as `native_dds`. The algorithm
backend is `ltlio`; `mapping` versus `localization` comes only from the
Product's `slam_mode`. Python does not own an in-process SLAM runner or native
binding. Removed legacy backend names are rejected instead of falling back to a
contract-only backend. GNSS device access belongs to the native service under
`src/drivers/real/gnss`; localization consumes its typed DDS output only when a
Product explicitly connects GNSS fusion.

## LTLIO naming

LTLIO is LingTu's native LiDAR-inertial frontend, derived from the Fast-LIO2
reference. The public backend is `ltlio`; the algorithm and its configuration
live under `src/localization/ltlio/`. Original upstream notices and licenses
keep their source names. This rename does not integrate a different estimator.

The active product vocabulary is `ltlio` everywhere in source configuration,
service names, and runtime status. A remaining `fastlio2` spelling is either an
upstream attribution, a dated evidence record, or a retired deployment name;
it must not be used to select a second backend. Keep those historical names in
their original records so old evidence remains searchable, but do not copy
them into new source files or current product configuration.

For locally maintained launch commands or robot catalogs, replace backend and
`env_config.localization` value `fastlio2` with `ltlio`, model key
`sensors.mid360.fastlio2_config` with `sensors.mid360.ltlio_config`, and sensor
filename `mid360_fastlio2.yaml` with `mid360_ltlio.yaml`. Custom sensor YAMLs
use `ltlio_max_pos_cov_trace` and `odom_prior_bypass_ltlio` for the renamed
frontend settings. Build with environment variable `LINGTU_SLAM_LTLIO` or
CMake option `LINGTU_SLAM_LTLIO_BACKEND`; removed build options report the new
name rather than selecting another backend. Re-resolve the Product after
updating these inputs. Native runtime and Host must be updated together because
their frontend diagnostic fields and reason codes now use the `ltlio_` prefix.

When reusing a CMake build directory, remove the retired cached option with
`-U LINGTU_SLAM_FASTLIO2_BACKEND` and set `-DLINGTU_SLAM_LTLIO_BACKEND=ON`.
The core implementation and Elevator-LIO comparison are documented in
[`ltlio/README.md`](ltlio/README.md).

## Semantic Occupancy Localization

SOCC-ICP is a research input, not a vendored runtime. LingTu owns semantic
occupancy and free-space evidence in `src/maps`; localization owns scan-to-map
registration, pose estimates, and health.

The current ROS-free path is:

```text
semantic_map.bin
  -> SemanticMapClient (direct C++ persistence reader)
  -> MapIcp
  -> NativeRelocalizer
  -> LTLIO map->odom commit gate
```

`NativeRelocalizer` prefers a sibling `semantic_map.bin` for seeded ICP. If the
artifact exists but is invalid, loading fails closed;
it does not silently fall back to PCD. If no semantic artifact exists, the
saved `map.pcd` remains the explicit geometry source. BBS3D supplies global
coarse search when built, then the same MapIcp refines the result.

The removed Python `SemanticMapModule` is not part of Product assembly. A sibling
artifact therefore comes only from an explicit offline workflow, not from the
field Product's online map stack.

Initial relocalization may establish a large map alignment. Periodic drift
correction is stricter: it requires inlier/covariance diagnostics and rejects
fitness failures, map-bound violations, covariance degeneracy, and corrections
larger than the selected runtime profile permits. The generic backend defaults
are 1 m and 15 degrees; the selected field profile may tighten these to
0.15 m translation, 2 degrees yaw, and 5 degrees map-to-odom tilt. Rejected ICP
never changes the active map or `map->odom`.

The current MapIcp uses semantic-map voxel geometry as its target. Semantic
class/confidence weighting and mixed point-to-plane residuals remain a measured
algorithm upgrade, not a claimed property of this baseline.

The adoption decision and product gates are documented in
`docs/roadmap.md`.

## Loop and PGO Status

The online LTLIO frontend does not change poses or maps through loop closure.
During capture it freezes `map.pcd`, `poses.txt`, body-local patches, and their
manifest. SaveMap can then run the ROS-free `lt_pgo --auto-constraints` helper on
its private working copy. The helper measures a complete adjacent chain from the
patches, merges verified loop factors, and optimizes only when the chain is
complete and at least one loop is trusted. A non-ready graph leaves the raw
source unchanged and records its precise skip code in `map_optimization.json`.

`pose_graph.constraints` is an optimizer-private temporary input, not a
LTLIO output or Product artifact, and is not published in the saved map.
The former ROS2 PGO/HBA runtime remains physically removed. This connected
save-time path has local contract coverage, but field loop quality, trajectory
quality, and target-platform performance evidence are still outstanding.
