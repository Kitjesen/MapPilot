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

OctoPlanner3D consumes the activated OctoMap together with request-scoped
queries over Mapd's real-time measured occupied/free evidence. It returns one
global route without changing the saved map. Current hits also block a route
through space that was free in the saved map; fresh free evidence can resolve
obsolete saved occupancy under the start-admission rules below. Last-known
hits remain effective when a snapshot ages, while stale free evidence cannot
clear obstacles.

SCAN owns local trajectory optimization, live collision checks and recovery
while the route is executed. A local recovery failure requests a global replan
through the existing goal runtime. The old temporary preblocked overlay remains
available to offline tests and explicit callers; the field loop does not build
that overlay. The occupied/free snapshot queries above are the current field
integration, including initial goals, replans and previews.

## XYZ global route and local execution

The body-centre route now tries a prepared multi-layer surface graph first.
Map preparation enumerates exposed and repaired surfaces, validates static
poses and local 3D connections, computes a soft boundary-distance preference,
then stores roughly metre-spaced nodes and the actual fine-cell corridors
between them. Queries reuse these corridors, check them against current
observations and request overlays, and disable blocked edges only for that
request. Within the selected corridor regions, a bounded search over cached
fine edges can bypass intermediate region centres. It uses the original edge
costs, the coarse route's cost as an upper bound, and the remaining iteration
budget. Only a shorter or equal-length valid candidate replaces the checked
coarse route. Endpoints first try direct connections to nearby surface candidates.
Only when those fail, a bounded local XYZ connection reuses the existing snap
radius to join the graph around an intervening corner. This does not add a
second global search budget or move the original requested endpoint. The original
XYZ search remains available when the prepared graph
cannot connect the current endpoints or live free evidence opens a passage
that the saved graph did not contain. There is no additional public mode.

Successful graph routes are shortened and relaxed with fixed endpoints. A
shortcut across several edges at different heights must also admit standing
poses along its interpolated height within the configured support tolerance.
This can remove small nominal-height detours while preserving a platform
before a stair where a long rising chord would lose standing support. An
original short stair edge keeps its step rule. A valid constant-height chord
preserves measured endpoint heights. Elastic moves stay on the same layer,
and only changed neighbourhoods are evaluated again. Resampling projects
new points onto the corresponding layer and keeps the original edge when its
subedges are invalid. Every accepted connection uses the existing 3D support,
cylinder and metric-motion query. This adapts
DimOS's Voronoi corridors and postprocessing workflow, not its exact geometry
or cost model: the soft distance comes from graph boundaries and the elastic
step does not include DimOS's wall-gradient force. The same map/configuration
reuses the prepared graph; changed map/configuration rebuilds it. Temporary
overlays and live occupied/free evidence only refresh request checks. Existing
navd startup preparation builds it before the first goal. Unprepared calls
still pay the build cost. Graph construction checks cancellation and discards
an interrupted build without publishing a partial graph.

The [2026-10-31 comparison](../../../../../../docs/research/surface-graph-20261031.md)
records the five fixed real-map requests, repeated query results, cold preparation
cost, path-length differences and independent original-OctoMap collision audit.
The [follow-up cost and route study](../../../../../../docs/research/surface-graph-costs-20261031.md)
separates OctoMap input costs from graph construction and measures the bounded refinement.

The global planner now searches positions only. The XYZ+yaw lattice, heading
assignment along a guide, and global per-part body/turn-sweep admission were
removed on 2026-10-30 at the user's request. Returned route points have no yaw;
GoalPlanController activates the XYZ route without requiring intermediate pose
headings. A requested final yaw remains an Executor terminal objective.

The global XYZ search retains its existing configured radius/vertical envelope,
3D obstacle, ground, step/slope, endpoint tolerance and cancellation semantics.
The change does not remove SCAN's live collision queries or final motion arbitration.
A request's immutable body snapshot is forwarded to local execution; it does
not select a second global search or authorize global body clearance.

`body_query` remains an offline geometry diagnostic used by test/benchmark tools.
It is not called by global route generation. Historical body/yaw experiments do
not describe current production planning. `xyz_smoke.cpp` checks body/yaw-independent
planning, solid obstacles, ground gaps and cancellation; endpoint tests check
XYZ activation and retention of terminal yaw/local body data.

The fixed-map before/after evidence and surface repair investigation are in
[`build/xyz-global-20261030/README.md`](../../../../../../build/xyz-global-20261030/README.md).
The 2026-10-30 surface update adds a separate native support index; the persisted
OctoMap and saved point cloud remain unchanged. The runtime default closes
surface holes over 0.20 m (`surface_closing_radius_m`), independently at each
height, using four-neighbour dilation followed by erosion. Original occupied
columns are retained. An inferred cell must have headroom and nearby original
occupied evidence; it never becomes a new occupied obstacle. Closing can infer
small real gaps as well as scan gaps: these cells are inferred support, not new
sensor observations. `direct_support_only` deliberately disables this inference.

This follows the per-height surface construction in pinned DimOS MLS
`26144f7fd8424bdc33ab97dcd58e9497f2d413ee`, `src/surfaces.rs`. ETH
`elevation_mapping_cupy` at `20a8a26b67a995b43eb44c23568854d1fed82a52` uses
inpainting then `MORPH_CLOSE` for its smooth base-reference layer; we do not
flatten the 3D map into that single elevation representation or add OpenCV/ROS.

Body-centre search uses a 0.10 m XY lattice (`search_resolution_m`), never finer
than the map voxel size. The Z spacing is subdivided for the configured slope.
Map occupancy, collision sampling and support queries keep the original map
resolution. Snap radius remains measured in **map cells**. The start anchors
the lattice; straight/diagonal edges still sample the fine map between vertices.
The original 26-neighbour A* priority is retained; weighted-A* candidates were
measured offline but are not the production implementation.

The index stores sorted occupied heights per XY and a coarse occupied-height
summary only to skip empty regions. A positive summary always falls through to
fine collision checks. It is prepared when a map/configuration changes, not for
every goal. Explicit `setOctomap` calls rebuild it even when the caller reuses
and mutates an OcTree pointer; `PlannerSession` already avoids redundant loads.

Product keys are `octoplanner3d_surface_closing_radius_m` and
`octoplanner3d_search_resolution_m`; navd accepts the corresponding
`LINGTU_NAV_OCTO_SURFACE_CLOSING_RADIUS_M` / `LINGTU_NAV_OCTO_SEARCH_RESOLUTION_M`
and `--octo-surface-closing-radius-m` / `--octo-search-resolution-m`.
Headless request/result JSON exposes the unprefixed option names. Vendor
`PlannerConfig` defaults remain zero (no closing, map-resolution search) for
upstream/legacy comparisons. Fixed-map evidence is in
[`build/surface-native-20261030`](../../../../../../build/surface-native-20261030/).


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

Global vertices and continuous edges check a 3D corridor around the route
centreline. Surface-route shortcuts are accepted only after the same continuous
3D query; XYZ fallback retains its adjacent cells. In the SCAN composition, the corridor radius is at least the local
single-cylinder radius. The longitudinal cylinder offset is not added to that
radius: doing so demands room for every body heading even along a straight
passage. Explicitly larger global radii remain available. Global clearance above
and below the body is at least the corresponding local clearance, rounded up
to the local collision map's voxel resolution. Ground support, unknown-space
evidence, slope and step limits remain part of the unchanged 3D query.

This is reference-route geometry that has passed the global checks; it is not a
guarantee for every body orientation or corner. SCAN retains the full pair of
offset cylinders, actual body heading,
trajectory collision and dynamic-obstacle checks. The final command boundary
checks actual-pose reaction/braking sweeps for both translation and rotation
after velocity shaping; a reference route cannot override that check. No
heading search, oriented Route payload or additional motion-state machine is
required for this division of responsibility. A difficult corner can still
require local replanning/recovery; global XYZ search does not prove that every
reference route can be executed. Standalone callers choose their intended
route-query radius themselves.

Endpoint resolution first validates the actual requested start, then requires
an allowed connection from that start to the first output waypoint. Collision,
ground support and unknown-space checks use the configured 3D query; the entry
also obeys metric step and slope limits and forbids a nonzero purely vertical
move. Body-centre mode sweeps the actual metric connection, including sub-voxel
coordinates. Legacy surface mode retains its voxel-rooted support and upper-body
semantics while sampling the connection. An occupied or unsupported start reports
its specific cause; nearby-node selection cannot repair it. If the actual start is
valid but its nearest quantized candidate or connection is invalid, the search
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

The 2026-10-31 runtime accepts the actual start as already occupied by the robot.
Saved occupied cells overlapping that request's initial body envelope do not
prevent departure, including offline planning without new free observations.
The exemption operates on whole intersecting voxels, not clipped voxel pieces.
The remaining route still checks saved obstacles. Measured occupied cells take
priority even inside the initial envelope; observed free clears other saved
body obstacles only when the complete cell is currently observed free. Saved
ground evidence and explicit blocked regions remain separate. Both body-centre
and legacy surface-node queries use this request-scoped binding for initial
planning, replanning and preview. It does not mutate the tree or persist across
requests. Actual-start failures report
`start_occupied`, `start_unsupported` or `start_temporary_obstacle`; only unresolved
candidate/entry searches retain `start_snap_exhausted`, and bounds keep their
existing public reason.

The same request snapshot also supplies measured occupied cells. They override
saved free space and live clearing in body-centre, legacy and lowest-surface
candidate queries. Raw hits are tested in the configured body height range;
support-only queries do not treat the observed floor as a body obstacle. The
lowest-surface prebuilt graph still does not restore candidates removed by old
static obstacles; the initial-envelope exemption also does not rebuild those
candidates. Live clearing is verified for the ordinary runtime modes.

Successful body-centre routes also carry an immutable support query from this
same SurfaceIndex to SCAN. The reference route selects the terrain layer for
optimized-curve checks; current base heave is not another nominal-height gate.
Body clearance is checked separately, so querying ground does not reintroduce
the saved obstacle that a live-free observation just cleared. The final arrival
contract follows the original requested goal rather than adding the selected
endpoint's displacement to the execution tolerance. See the
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
| `body_query.hpp/.cpp` | Offline diagnostic for saved-map body geometry; not global route admission. |
| `surface.hpp/.cpp` | Immutable occupied columns and inferred multi-layer support surfaces; does not rewrite the saved map. |
| `surface_graph.hpp/.cpp` | Prepared three-dimensional surface adjacency and sparse corridors, with bounded endpoint connections and request-local edge checks. |
| `route_smoothing.hpp/.cpp` | Validated equal-height shortcuts and relaxation, preserving height-transition anchors and measured endpoints. |
| `surface_graph_smoke.cpp` | Sparse corridors, separated floors, walls, live rerouting, request reset, stairs, bounded endpoint connection and build/query cancellation. |
| `route_smoothing_smoke.cpp` | Walls, stacked floors, stair resampling, long-platform lookahead, measured flat height, endpoint retention, length and cancellation. |
| `xyz_smoke.cpp` | XYZ planning and activation contract without global heading stages. |
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
