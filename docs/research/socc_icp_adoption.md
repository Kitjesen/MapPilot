# SOCC-ICP Adoption Decision

Status: native geometry baseline implemented on 2026-07-10; S100P field
validation and semantic-weighted ICP remain gated.
Audience: mapping/localization research and product adoption maintainers
Replaced by: `MAP_SERVICE_CONTRACT.md` and `NATIVE_RUNTIME.md` for current product boundaries

## Decision

LingTu will adapt the useful SOCC-ICP ideas, but it will not vendor or run the
upstream ROS2/Python system as a product dependency.

The upstream project explicitly describes itself as a proof of concept. Its
scan registration is Python, its semantic occupancy map is provided by a
separate ROS2 Radix node, and the maintainers identify Python registration and
ROS2 transport as current performance bottlenecks. Those constraints conflict
with LingTu's ROS-free C++ product path.

Primary references:

- https://github.com/josch14/socc_icp
- https://arxiv.org/abs/2605.15074
- https://github.com/ProjectVERUM/radix_ros2_pkg

The SOCC-ICP repository is MIT licensed. Radix repositories used by it are
MPL-2.0. LingTu therefore implements its own map representation and algorithms
instead of copying the Radix implementation.

## What LingTu Reuses

The reusable design is one shared spatial representation with:

- probabilistic hit and miss evidence per 3D voxel;
- ray-based free-space updates that remove stale dynamic obstacles;
- point mean and covariance per occupied voxel;
- semantic evidence and a dominant class per voxel;
- local map queries suitable for scan-to-map registration;
- occupancy and semantic confidence as ICP correspondence weights;
- point-to-plane residuals for planar cells and point-to-point residuals for
  non-planar cells.

This is a design adoption, not a source-tree import.

## Ownership

| Capability | Owner | Reason |
| --- | --- | --- |
| Semantic occupancy storage | `src/maps` | It is a reusable spatial product. |
| Hit/miss and ray free-space update | `src/maps` | It changes map evidence, not robot pose. |
| Voxel geometry and semantic statistics | `src/maps` | Planning, visualization, and localization can share them. |
| Local occupied-map query | `src/maps` | Callers must not inspect map storage internals. |
| Scan-to-map ICP and pose estimation | `src/localization` | It estimates robot motion and localization health. |
| Point/plane residual selection and ICP weighting | `src/localization` | They are registration policy. |
| Semantic label production | `src/perception` | Maps consumes labels but does not run perception models. |
| Transport and process placement | `src/runtime` adapters | Domain code remains transport independent. |

`src/maps` must never call localization. Localization may consume the public
maps C++ interface, which keeps the dependency one-way.

## Implemented Map Interface

The native map product is implemented in:

- `src/maps/include/lingtu/maps/layers/semantic_occupancy.hpp`
- `src/maps/cpp/layers/semantic_occupancy.cpp`
- `tests/maps/cpp/semantic_occupancy_test.cpp`
- `src/maps/include/lingtu/maps/layers/semantic_occupancy.hpp`
- `src/maps/include/lingtu/maps/semantic_map_persistence.hpp`

`SemanticOccupancyLayerCore` currently provides:

- optional per-point `uint16` semantic labels;
- enforced label timestamp, frame, taxonomy, and taxonomy-version matching;
- bounded log-odds occupancy evidence;
- 3D voxel traversal for free-space ray updates;
- per-frame hit/miss deduplication so LiDAR density does not change evidence;
- Welford mean/covariance accumulation;
- fixed-memory dominant-semantic tracking;
- clearing of stale geometry when miss evidence makes a cell free;
- `full_map` replacement and incremental update semantics;
- deterministic local radius queries with hard work/result limits;
- a hard voxel-count limit with deterministic eviction;
- returned update statistics, including explicit partial-ray/truncation status;
- immutable generation-tagged SoA chunks for concurrent readers;
- a stable count-then-fill C ABI for update, query, snapshot, metadata, save,
  validate, open, and load;
- atomic, versioned `semantic_map.bin` persistence with strict bounds and
  checksum validation;
- no ROS, PCL, Python, Radix, Eigen, or new third-party dependency.

The fixed-size semantic summary is intentional for aarch64 cache locality. It
tracks the dominant classes needed by registration without allocating a hash
table inside every voxel.

`full_map` frames replace occupied state but do not perform ray clearing. A
single accumulated cloud does not carry the per-ray sensor origins required for
valid free-space evidence; ray updates are accepted only from incremental
frames.

## Implemented Data Flow

```text
Fast-LIO accepted incremental scan + exact state-at-scan pose
  -> MapObservationFrame
  -> optional exact sequence/time/frame uint16 labels
  -> SemanticMapModule runtime ports
  -> lingtu_maps semantic C ABI
  -> semantic occupancy / MapSceneFrame / semantic_map.bin

saved semantic_map.bin
  -> stable lingtu_maps C ABI
  -> localization SemanticMapClient
  -> MapIcp seeded refinement
  -> NativeRelocalizer / optional BBS3D coarse search
  -> Fast-LIO map->odom commit gate
```

Realtime map layers no longer raycast accumulated `map_cloud`; they consume only
accepted incremental observations with their exact origin. Geometry-only mode
uses label `0` and follows the same persistence and localization path.

MapIcp currently uses occupied semantic-map voxel geometry as a GICP target.
The label and confidence fields are preserved through the ABI and Gateway, but
they are not yet registration weights.

## Remaining Algorithm Work

1. Replace the transitional global voxel hash with persistent sparse blocks
   only after profiling proves it improves long-run memory/local-query cost.
2. Add measured point-to-plane/point-to-point residual selection and robust
   occupancy/semantic weighting behind MapIcp. Do not make it default from the
   paper alone.
3. Export p50/p95 registration timing and correction-gate counters in field
   telemetry.

## Product Gate

Automated gates now cover no-ROS source boundaries, deterministic LTU1 replay
validation, stale-obstacle ray clearing, geometry-only operation, generation
guards, corridor-degeneracy diagnostics, transform jump rejection, and a C++
semantic update throughput threshold. The aarch64 build automatically uses a
stricter default threshold, overrideable with `LINGTU_MAPS_SEMANTIC_MAX_MS`.

The localization extension must not become the field default until the
remaining hardware gates pass on the S100P with a real MID-360 recording:

- bounded memory under a long MID-360 recording;
- p50/p95 scan latency and CPU utilization within the field budget;
- lower or equal trajectory RTE/ATE than the current native localizer;
- measured reduction of dynamic-obstacle ghost lifetime;
- successful recovery and global relocalization in repeated corridor and
  kidnapped-robot trials.

Until those measurements are archived, semantic weighting remains experimental;
the shipped baseline is the guarded geometry MapIcp path using the production
maps interface, not a second map system.
