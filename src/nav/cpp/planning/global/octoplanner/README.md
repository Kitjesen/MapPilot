# OctoPlanner3D Backend

This directory is the complete product-owned integration boundary for the
default 3D global planner.

```text
navd
  -> GlobalPlanRequest / GlobalPlanResult
  -> octoplanner3d_runtime
  -> vendor/planner
  -> system liboctomap
```

`vendor/` contains only the four OctoPlanner3D files used by the product:
the constrained planner header/source and the optional offline PCD converter
header/source. The ROS2/RViz wrapper, duplicate runtime, sample assets, build
directories, and precompiled libraries are not product inputs.

The online `navd` path reads an immutable, Maps-validated `octomap.ot` snapshot
and does not link PCL. PCL is limited to the standalone offline
`octoplanner3d_pcd_to_octomap` tool. OctoMap must come from the target system
(`liboctomap-dev`); checked-in `.so` files are intentionally rejected so an
x86_64 binary cannot leak into an aarch64 release.

The vendored code derives from
[JackJu-HIT/OctoPlanner3D](https://github.com/JackJu-HIT/OctoPlanner3D), commit
`9a9cc431ea905a5878975cc6fbbce6c9618b31a4`, under the MIT license. LingTu's
copy includes robot-envelope, ground-support, floor-continuity, step/slope,
terminal-tolerance, and cancellation changes. See `LICENSE`.

FAR is a separate optional 2D global planner under `../far`; it is not part of
this vendor tree and does not replace OctoPlanner3D as the product default.

## Runtime ownership

OctoPlanner3D generates XYZ reference routes from the activated immutable
OctoMap. Initial goals, preview and replanning use the same saved map and
configured 3D cylindrical corridor. The request carries no local observation
callbacks, compound Body or raw terrain points. The local planner and final
control own current collision evidence and the selected execution geometry.
Terminal yaw remains an execution objective kept by the task layer.

## XYZ search and prepared surfaces

The pinned OctoPlanner3D search uses position states. LingTu adds cached
multi-layer support surfaces, sparse corridors, continuous 3D connections
and validated route shortening and smoothing. Coarse, fine and fallback
search key states by position. Connections check static occupancy, support,
step and slope limits at map resolution. The default XY search spacing is
0.10 m. Queries reuse the topology prepared for the map and its configuration.

The default 0.20 m surface closing is applied independently at each height.
Inferred support needs nearby occupied evidence and headroom. It remains in
the support index, separate from the persisted OctoMap. Strict direct support
disables inference. Surface preparation and smoothing adapt the workflow of
DimOS MLS at `26144f7fd8424bdc33ab97dcd58e9497f2d413ee`.

## Saved point-cloud conversion

`octoplanner3d_pcd_to_octomap` consumes saved, voxel-sampled point clouds.
Save-time pruning owns persistent-map cleanup. The converter preserves each
finite, in-bounds occupied voxel; point multiplicity after voxel sampling is
not an observation count. Requiring several points per voxel or applying a
second small-cluster filter can erase both ground and thin obstacles.

PCL supplies the ASCII/binary reader when available; the no-PCL ASCII reader
feeds the same occupancy, support dilation, and free-envelope builder. Free
envelopes are generated only above detected support surfaces. A cloud with no
usable occupied voxels fails without writing an empty OctoMap. The optional
direct-PCD backend preserves one occupied voxel per sampled point location,
without a second point-count/cluster filter or invented support/free padding.
Like the PCD import in
[jie_3d_nav](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/src/pcd_to_octomap_node.cpp),
this is geometry conversion, not ray evidence. It cannot itself establish
navigation readiness; normal Product navigation consumes the saved OctoMap.
The PCL-enabled sampled-PCD regression covers both converter entry points.

Support seeds must be exposed (no occupied voxel immediately above) and have
exposed neighbors in at least three of the four directly adjacent XY cells,
allowing one voxel of height difference for treads. Do not probe across several
empty cells: nearby parallel walls can otherwise satisfy the horizontal-support
test together and acquire incorrect padding and free envelopes. Sparse points
that fail this support test remain occupied; they are not deleted. The generated
free envelope is a configured surface-based inference, not measured ray clearing.
Support padding also checks the original target column: if an occupied surface
already exists more than one voxel below, it does not invent an upper surface
there. This prevents raised object tops from growing into aisles above observed
floor. Existing upper-floor and obstacle points remain occupied.

`octoplanner3d_sampled_pcd_smoke` checks one-point-per-voxel ground, an isolated
thin obstacle, support-only free space, and empty-result rejection. It exercises
support/free dilation of both zero and one cell, preserves obstacles inside the
free envelope, rejects isolated/parallel vertical walls as support, and retains
one-voxel stair transitions. When PCL
is available it checks both ASCII and binary PCD, while
`octoplanner3d_sampled_ascii_pcd_smoke` checks the no-PCL reader as well.

Footprint support samples use the cells containing the physical radius edges.
For a 0.43 m radius at 0.2 m resolution, the offset is two cells (0.4 m);
rounding up to three queried a cell entirely outside the footprint. The grid
query regression checks both a supported narrow floor and a genuinely missing
edge support. Collision-envelope sampling is unchanged.

## Search neighborhoods

Search retains upstream commit `9a9cc431ea905a5878975cc6fbbce6c9618b31a4`'s
single A* frontier and 26 adjacent offsets. Candidate vertices use the existing
3D occupancy and ground-support predicate. There is no large stair neighborhood
or separate stage budget.
In body-centre mode (`support_height_m > 0`, `lowest_traversable_only=false`),
the planning height step is `map_resolution / ceil(1 / max_slope)` when a
positive slope limit is configured; XY and the measured OctoMap resolution stay
unchanged. At 0.10 m and a 0.57735 slope bound, the planning height step is
0.05 m. An isotropic adjacent grid has no rising edge below slope 0.707, so
checking slope on that old grid would disconnect every allowed ascent.
Every body-mode candidate edge now checks metric step/slope limits and the
continuous 3D route sweep before entering the frontier. All fine-height
vertices still query the original observed support, unknown-space and body
collision evidence. Clear endpoints alone do not establish clearance through
a diagonal corner. Existing cell/support-layer modes retain their own vertex
semantics. The prepared surface graph can connect the observed 20 cm riser
fixture when its configured step/slope limits and the existing continuous
motion query permit the transition. The previous adjacent body-height lattice
could not represent it. Drop-offs, unknown gaps and body obstacles remain
negative cases; a planned riser is not proof of physical stair climbing.

LingTu retains map/query caches, cancellation and endpoint tolerances.
`searchInfo().used_surface_graph` and the surface-route log identify the fast
path. `searchInfo().iterations` describes the fallback XYZ search; its budget
exhaustion returns `search_iteration_limit`. Metric motion queries and the
verifier use the same physical height step.
A returned global route is guidance for SCAN, not proof of executable motion.

## Support surfaces and collision envelopes

Global vertices and continuous edges check the configured three-dimensional
cylindrical corridor against saved occupancy. The global query also checks
ground support, slope and step limits.

SCAN checks the selected compound Body at its queried heading using live
trajectory and dynamic-obstacle evidence. Final control checks actual-pose
reaction and braking sweeps after velocity shaping. The global route provides
static-map guidance for local execution.

Endpoint resolution first validates the actual requested start, then requires
an allowed connection from that start to the first output waypoint. Collision,
ground support and unknown-space checks use the configured 3D query; the entry
also obeys metric step and slope limits and forbids a nonzero purely vertical
move. Body-centre mode sweeps the actual metric connection, including sub-voxel
coordinates. Legacy surface mode retains its voxel-rooted support and upper-body
semantics while sampling the connection. The actual start checks map bounds;
SCAN owns its current-body collision check. Departure samples
and the first supported candidate still require ground evidence. If the nearest
quantized candidate or connection is invalid, the search
continues to other reachable candidates within the configured snap radius.
Body-centre XYZ goal resolution first tries a fully checked connection from
a grid node to the exact requested goal. This uses the existing metric-body
mode (positive support height or explicit upper/lower clearance); the legacy
voxel-rooted surface mode retains its original endpoint handling. The node
search is bounded by half the planning
cell's 3D diagonal; this bound compensates grid quantization and does not relax
arrival accuracy. A successful route includes the original goal as its final
point, so runtime terminal tolerances still apply to that actual endpoint.
The existing metric step/slope, 3D collision and support checks validate both
the requested pose and its connecting sweep. If the exact goal or connection
is invalid, the original snapped-goal search and all configured goal tolerances
remain in force. On a 20cm grid a supported node can lie15.56cm from a valid
requested pose even with15cm arrival tolerance; rejecting that connection
without trying the exact pose was a LingTu quantization error. Upstream commit
`9a9cc431ea905a5878975cc6fbbce6c9618b31a4` returns grid centres and has no
LingTu terminal-tolerance bounds or this continuous goal connection.

The planner exempts saved occupied voxels that overlap the request's initial
cylindrical envelope so the robot can depart from a start containing its own
recorded occupancy. The exemption is confined to that envelope and that
request. The saved tree, support evidence, and explicit blocked regions retain
their own semantics. The remaining route checks saved obstacles.

The current SCAN grid does not recheck an optimized local curve against this
saved support surface. It checks its live collision geometry, which alone does
not prove a local detour has continuous support. The final arrival contract
follows the original requested goal rather than adding the selected endpoint's
displacement to the execution tolerance. See the
[implementation and evidence](../../../../../../build/navigation-repair-20261031/README.md).

The production route uses OctoPlanner3D's neighbouring occupied-support query
so a single missing floor voxel in a saved scan does not block an otherwise
supported stance. Strict direct support and the optional four-footprint
diagnostic remain available for map-quality tests. The neighbouring-support
fallback does not bypass the metric limits on body-mode graph edges.

Therefore a headless result with `reached_goal=true` is a discrete graph
result until the continuous verifier also passes. Semantic navigation keeps
that result read-only otherwise. `octoplanner3d_verify_path` accepts
`--planning-anchor x y z` so a snapped first waypoint does not change the
planning lattice used for support queries. It also checks the anchor-to-first
waypoint connection, reporting `start_connection_failed` if that entry fails.
The official benchmark passes the original request start as this anchor. A
standalone path file without an anchor verifies only the returned route and
cannot establish validity at the original request start.

For an uneven supported stance, the surface-rooted envelope distinguishes a
nearby tread from a taller obstacle. It never ignores occupancy in the route
node's own XY column. Flat-ground clearance and explicit body-centre cylinder
clearance retain their occupied-volume checks. These tests establish geometric
route feasibility, not a foothold sequence or the locomotion policy's ability
to climb the resulting staircase.

## Files

The [official PCD regression](../../../../../../tools/bench/README.md) retains
pinned upstream clouds, fixed routes and negative cases. It runs this runtime
and rechecks the returned graph route through `octoplanner3d_verify_path`.
The tool reports `route_valid` for vertex validity and 26-neighbor connectivity,
`sweep_valid` for the full route's continuous 3D sweep, and
`motion_diagnostic_valid` for metric step/slope checks on the planning edges.
It checks every segment even after a metric diagnostic fails. Failure arrays
record segment endpoints, the failed sweep sample (when available), rise,
horizontal distance and slope; configured step/slope limits are also returned.
The conservative aggregate `valid`/`reason` remains false when either check
fails. The official regression requires all three results to pass. The finer
body-height lattice makes the metric checks part of search as well as validation.
None of these checks runs SCAN, supplies a foothold sequence, simulates robot
dynamics or certifies field execution. An anchor-verified entry establishes the
configured geometric and metric constraints, not successful physical motion.

| File | Responsibility |
| --- | --- |
| `octoplanner3d_core.hpp/.cpp` | Adapts `GlobalPlanRequest/Result` to OctoPlanner3D, loads immutable OctoMaps, caches a map-bound `PlannerSession`, and applies terminal constraints. Explicit temporary overlays remain an offline/request-level capability; the field local-obstacle loop does not provide them. |
| `surface.hpp/.cpp` | Immutable occupied columns and inferred multi-layer support surfaces; does not rewrite the saved map. |
| `surface_graph.hpp/.cpp` | Prepared three-dimensional surface adjacency and sparse corridors, with bounded endpoint connections and request-local edge checks. |
| `route_smoothing.hpp/.cpp` | Validated equal-height shortcuts and relaxation, preserving height-transition anchors and measured endpoints. |
| `surface_graph_smoke.cpp` | Sparse corridors, separated floors, walls, explicit overlay rerouting, request reset, stairs, bounded endpoint connection and build/query cancellation. |
| `route_smoothing_smoke.cpp` | Walls, stacked floors, stair resampling, long-platform lookahead, measured flat height, endpoint retention, length and cancellation. |
| `xyz_smoke.cpp` | Continuous XYZ guidance, explicit cylinder corridor, obstacles, support and cancellation. |
| `octoplanner3d_headless.cpp` | Standalone JSON stdin/stdout frontend used by diagnostics and compatibility tooling. |
| `verify_path.cpp` | Offline graph-route check plus separate continuous sweep/motion diagnostics; includes a CTest self-test. |
| `pcd_to_octomap.cpp` | Offline PCD-to-`.ot`/`.bt` conversion CLI with support/free-envelope options; not part of online `navd`. |
| `edit_octomap.cpp` | Offline editor for occupied, free, preblocked, traversable, and cleared regions. |
| `dump_octomap.cpp` | Exports bounded occupied voxel centers for inspection and debugging. |
| `make_test_octomap.cpp` | Generates deterministic two-floor and spiral-stair OctoMaps for tests. |
| `no_air_climb_smoke.cpp` | Checks ground support, body clearance, floor continuity, overlays, and no-air-climb behavior. |
| `neighborhood_smoke.cpp` | Checks 26-neighbor routing, quantized floor changes, stairs, disconnected risers, walls, budgets, cancellation, and runtime failure reasons. |
| `grid_smoke.cpp` | Compares cached grid occupancy and leaf-center queries with OctoMap, including map and overlay invalidation. |
| `queue_node_compare_smoke.cpp` | Checks the vendored planner priority-queue ordering. |
| `edit_octomap_smoke.cmake` | Drives the binary OctoMap edit/read/write smoke test. |
| `vendor/planner/include/global_planner.h` | Vendored constrained search types and `OctoPlanner3D` interface. |
| `vendor/planner/src/global_planner.cpp` | Vendored 3D grid/A* search implementation with LingTu constraints. |
| `vendor/octomap/include/pcd2octomap_converter.h` | Optional vendored PCD converter interface. |
| `vendor/octomap/src/pcd2octomap_converter.cpp` | Optional vendored PCD converter implementation. |
| `CMakeLists.txt` | Wires system OctoMap, optional PCL, runtime library, CLIs, and smoke tests. |
| `LICENSE` | MIT license and provenance for the vendored subset. |
