# MLS and tomography backend direction

This note records the backend boundary chosen on 2026-11-15. It compares the
public DimOS MLS planner, PCT Planner, and the current LingTu implementation;
it is not a claim that either external project is already a drop-in field
component.

## Public references

- DimOS MLS planner, current `main` checked at commit `5e0ebfd422`:
  [`surfaces.rs`](https://github.com/dimensionalOS/dimos/blob/5e0ebfd422/dimos/navigation/global_planner/mls_planner/rust/src/surfaces.rs),
  [`nodes.rs`](https://github.com/dimensionalOS/dimos/blob/5e0ebfd422/dimos/navigation/global_planner/mls_planner/rust/src/nodes.rs),
  [`edges.rs`](https://github.com/dimensionalOS/dimos/blob/5e0ebfd422/dimos/navigation/global_planner/mls_planner/rust/src/edges.rs),
  [`mls_planner.rs`](https://github.com/dimensionalOS/dimos/blob/5e0ebfd422/dimos/navigation/global_planner/mls_planner/rust/src/mls_planner.rs).
- DimOS release `v0.0.15` (2026-10-27):
  <https://github.com/dimensionalOS/dimos/releases/tag/v0.0.15>.
- PCT Planner, current `main`: <https://github.com/byangw/PCT_planner>.
  The repository identifies the method as *Efficient Global Navigational
  Planning in 3-D Structures Based on Point Cloud Tomography*,
  IEEE/ASME T-Mech 2024, DOI landing page:
  <https://ieeexplore.ieee.org/document/10531813>.

PCT is GPLv2 and its public build requires ROS, CUDA, CuPy, and Open3D. DimOS
MLS is Apache-2.0, but its planner is part of the DimOS Rust/runtime and message
graph. These licenses and runtime boundaries matter before copying code into
the native product.

## What the algorithms represent

| Layer | PCT tomography | DimOS MLS | LingTu now |
| --- | --- | --- | --- |
| Input | Point cloud converted to a tomogram | Ray/voxel map | Saved OctoMap snapshot |
| Surface | Tomographic multi-layer structure | Standable cells per Z layer | `SurfaceIndex`, observed and bounded inferred cells |
| Sparse route representation | Planner search structure from tomogram | `Nodes` plus Voronoi/corridor `Edges` | `SurfaceGraph` nodes and corridor chains |
| Query | Global 3-D route | Snap poses, search graph, string-pull path | Attach endpoints, search graph, smooth, then XYZ fallback |
| Runtime updates | Primarily scene/tomogram rebuild | Regional voxel edits repair surfaces/nodes/edges | Whole artifact rebuild when the map snapshot changes |
| Local execution | Separate from the global route | Separate SE(2) planner/follower | SCAN and Follower remain separate |

The useful PCT idea is the problem boundary: first construct a stable,
multi-layer representation, then search across floors, stairs, platforms and
overhangs as one 3-D route. The useful DimOS idea is the ownership and
lifecycle: surface cells, node placement, corridor edges, and a planner asset
are separate stages, and a regional map edit repairs only the affected area.

## What changed in LingTu

`OctoPlanner3D` now owns one `PlannerArtifact` containing the prepared
`SurfaceIndex` and `SurfaceGraph`. The route query consumes that asset; it no
longer treats the two derived objects as independent planner state. Changes to
`PlannerConfig::preparation()` rebuild the map-derived asset. Query-only values
(`max_iterations`, snap radius, terminal tolerances, and same-floor preference)
call `setQuery()` and reuse the existing graph.

This is an ownership and invalidation improvement. It does not claim that
LingTu already has DimOS regional repair. The current field map path gives the
planner a new immutable OctoMap snapshot, not a stream of changed voxel
columns. Until mapd exposes a supported cell-delta or region-snapshot input,
rebuilding the artifact on a new map is the truthful behavior.

## Why we are not linking the external Rust planner directly

Direct linking would import more than a search routine: Rust ABI/build rules,
DimOS voxel and message types, worker/runtime ownership, coordinate and
artifact formats, and a fast-moving upstream module path. PCT would additionally
bring a GPLv2 component and a CUDA/ROS/Open3D build contract. That is not a
small replacement of global search.

The stable product seam is smaller:

```text
Map snapshot
  -> SurfaceProvider      SurfaceArtifact
  -> RouteProvider        GlobalRoute
  -> SCAN                 local trajectory
  -> Follower             desired velocity
  -> Driver               platform execution
```

The first provider is the current native C++ implementation. A future DimOS
MLS or PCT adapter can produce the same `SurfaceArtifact`/`GlobalRoute`
boundary for offline comparison before it is allowed to drive field motion.

## Map update contract

Static changes rebuild the asset:

- map content or frame identity;
- surface closing/repair settings;
- robot envelope and support band;
- step, slope, and edge-cost preparation settings.

Per-goal changes reuse the asset:

- start and goal;
- terminal tolerance;
- snap radius and iteration budget;
- same-floor query preference;
- request cancellation and temporary route overlays.

Realtime local obstacles stay with SCAN. They do not rebuild the global MLS
artifact and do not become a second global map authority.

## Next implementation slice

The next justified slice is not a new planner framework. It is to add a mapd
region/snapshot update operation, then port the DimOS repair order in small
steps: re-extract affected surface cells, repair nearby nodes, repair corridor
edges, and publish one new immutable artifact. Each stage must be compared with
a full rebuild on the same map edit before it is used by field navigation.

## Full comparison before switching

The latest public repository metadata checked on 2026-11-15 is:

| Project | Latest public state checked | License | Main strength |
| --- | --- | --- | --- |
| DimOS | `main` at `5e0ebfd422a6`, pushed 2026-11-14 | Apache-2.0 in the repository and MLS source headers | Online MLS graph lifecycle and regional repair |
| PCT Planner | `main` at `35cd73fd82bc`, repository last pushed 2025-02-17 | GPL-2.0 | Tomographic global planning for multi-layer structures |
| LingTu | Current workspace, native C++/DDS | Product code plus existing upstream notices | Executable 3-D body/support contract and field integration |
| OctoPlanner3D | LingTu pinned `9a9cc431ea90` | MIT | Small baseline 3-D grid search |

DimOS's Rust MLS crate is not a small standalone search function. Its current
`Cargo.toml` pulls in the DimOS module runtime, generated message crate,
Tokio, Rayon, AHash, validator, tracing, and SmallVec. Copying the four Rust
files alone would leave out the runtime and message contracts they expect.
Those files are still useful as the reference for the surface, node, edge, and
regional repair algorithms.

PCT's public package is a ROS/CUDA/CuPy/Open3D system with a GTSAM-based
trajectory optimizer. Its README describes a two-stage workflow in which
tomography constructs the scene representation and the planner searches and
optimizes the route. It is a strong algorithm reference, but GPLv2 prevents
copying its implementation into this product without a separate licensing
decision.

### Representation and search

| Question | PCT | DimOS MLS | LingTu current | Better choice |
| --- | --- | --- | --- | --- |
| Keep multiple floors at the same XY | Yes, through tomography | Yes, explicitly per Z surface | Yes, `SurfaceIndex` | Tie |
| Separate map representation from route search | Yes, tomogram then planner | Yes, surface, nodes, edges, artifact | Partly, now unified under `PlannerArtifact` | DimOS structure |
| Stairs and overhangs | First-class global 3-D cases | First-class multi-level surfaces | Supported by surfaces plus 3-D checks | PCT concept, LingTu contract |
| Sparse route graph | Tomogram search structure | Nodes and Voronoi corridor edges | `SurfaceGraph` nodes and corridor chains | DimOS organization |
| Endpoint attachment | Planner-specific | Explicit surface snap candidates | Existing graph and XYZ attachment | DimOS is clearer |
| Route smoothing | GTSAM spline and trajectory optimization | String pulling and smoother | C++ route smoothing with geometry rechecks | Depends on execution contract |

PCT has the clearest answer to the global question. It turns a point cloud
into a stable multi-layer planning representation before searching. DimOS has
the clearest software ownership. It keeps surface extraction, node placement,
corridor edges, updates, and query state in separate modules. LingTu now has
the same top-level asset boundary, but its implementation still rebuilds the
whole asset for a changed map.

### Update and runtime behavior

| Area | PCT | DimOS MLS | LingTu current |
| --- | --- | --- | --- |
| Full map build | Tomogram construction before planning | Voxelization, surfaces, nodes, edges | OctoMap, `SurfaceIndex`, `SurfaceGraph` |
| Regional update | No equivalent online repair in the public planner workflow | `update_region` re-extracts affected surfaces and repairs nodes/edges | No supported cell-delta input from mapd, so whole artifact replacement |
| Cached artifact | Tomogram files and planner data | In-memory graph plus published artifacts | `.nav/profile-N.bin` persists surface and graph |
| Live local obstacle handling | Separate from global planner | Separate local planner input | SCAN owns live local occupancy |
| Runtime coupling | ROS/CUDA/GTSAM | DimOS module/DDS/runtime | Native C++/CycloneDDS and existing process contracts |

DimOS is the strongest reference for incremental updates. LingTu should copy
the update order only after mapd exposes a supported changed-region snapshot or
cell-delta input. Without that input, a regional planner method would be an
API with no trustworthy source data.

### Same-input measurements already available

The 2026-11-14 matched replay used the same public OctoPlanner3D point clouds,
the same target requests, 5 cm map resolution, and a fixed support/body
contract. It is the most useful evidence because it keeps the geometry inputs
close, even though the two planners still use different internal predicates.

| Case | LingTu cold preparation | LingTu warm query | LingTu result | DimOS matched preparation | DimOS warm query | DimOS result |
| --- | ---: | ---: | --- | ---: | ---: | --- |
| Building cross-area | 32.9 s | 318.7 ms | Route returned and passed LingTu 3-D audit | 4.02 s | 14.10 ms | No route, disconnected components |
| Building stairs | 21.9 s | 97.1 ms | Route returned and passed LingTu 3-D audit | Shared building graph | 9.45 ms | No route, disconnected components |
| Plaza | 98.2 s | 74.3 ms | Route returned | 19.84 s | 16.4 ms | Route returned, failed LingTu support audit at point 3 |

This evidence gives three different answers. DimOS is faster after the graph
exists. LingTu returned all three matched routes and its own body/support audit
accepted them. DimOS's larger wall-clearance predicate disconnected the two
building cases, and its plaza route did not satisfy LingTu's support contract.
Neither result justifies replacing the execution geometry with the other
planner's route without an explicit conversion and audit.

### Decision by goal

| Goal | Best source to reuse |
| --- | --- |
| Stable multi-floor representation | PCT's tomography boundary |
| Surface extraction and bounded closing | DimOS `surfaces.rs` |
| Sparse node placement and corridor edges | DimOS `nodes.rs` and `edges.rs` |
| Incremental graph repair | DimOS `mls_planner.rs` and regional repair helpers |
| Body clearance, support, step, slope, and swept route checks | LingTu's native C++ geometry |
| Local dynamic obstacle handling | LingTu SCAN |
| Route execution and platform command delivery | LingTu Follower and Driver |

The overall choice is therefore a hybrid. DimOS is better as the global graph
implementation reference. PCT is better as the representation and paper
reference. LingTu remains the authority for whether a route can be executed by
the selected robot.

## What should be copied and what should be adapted

The first code import should contain only the Apache-licensed MLS algorithm
pieces and their attribution. The useful order is:

```text
DimOS voxel columns
  -> surface extraction and bounded closing
  -> sparse node placement
  -> corridor edge construction
  -> immutable planner artifact
  -> LingTu body/support and route checks
```

The DimOS module wrapper, generated message crate, Tokio worker lifecycle, and
DimOS-specific visualization messages do not belong inside `navd`. PCT's
tomography and GTSAM optimizer stay as an offline comparison backend until a
license and runtime decision exists.

The current `PlannerArtifact` change is the first small step toward this
layout. The actual Rust import should start only after the full comparison is
accepted as the baseline. It should be measured against the current C++
artifact on surface count, graph connectivity, route length, body audit,
regional update time, and cold/warm planning time.
