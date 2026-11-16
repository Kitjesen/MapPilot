# OctoPlanner3D Backend

This directory is the complete product-owned integration boundary for the
default 3D global planner.

```text
navd
  -> GlobalPlanRequest / GlobalPlanResult
  -> octoplanner3d_runtime
  -> PlannerArtifact (Rust surface + sparse graph)
  -> vendor/planner
  -> system liboctomap
```

Rust owns MLS surfaces, graph search, endpoint connections and the geometry
checks used by the normal MLS route. C++ connects OctoMap and the native process,
and retains the older grid search and geometry implementation for fallback and comparison.

```text
OctoMap snapshot
  -> Rust MLS            observed and inferred multi-level cells
  -> Rust graph          sparse nodes and corridor edges
  -> PlannerArtifact     one map-bound asset reused by route queries
  -> Rust metric query  endpoint attachment, route search and geometry
  -> route smoothing    shortcuts use the same Rust geometry check
```

`PlannerArtifact` owns the Rust prepared surface and graph together. A different
map or preparation geometry rebuilds that asset. A committed revision of the
same map compares occupied/free/unknown evidence and updates the changed region
inside the existing PlannerSession lock. Query-only changes such as
goal tolerance, snap radius, iteration budget, or same-floor preference update
the existing graph query. This avoids rebuilding static geometry for every
goal and leaves a small boundary for a future MLS or tomography backend to
replace only the global route source.

The Rust graph backend is built by this CMake target. The dependency-free
`lingtu_mls_core` crate exposes multi-level surface extraction and a native
sparse graph through a small C ABI. Its graph core follows the DimOS shape with
four-neighbor surface edges, wall-distance node placement, a multi-source
Voronoi partition, and predecessor corridors. On the fixed arrival-box map,
with DimOS-compatible floor step conversion, the Rust native graph reports
29 nodes, 45 edges, and 2 components. DimOS reports 29 nodes, 47 edges, and 2
components. The former C++ graph reported 36 nodes, 43 edges, and 9
components. The native comparison tests exercise matching Rust and C++ body,
support, step, slope, continuous sweep and no-go semantics.

The production `PlannerArtifact` holds a Rust MLS graph handle. `startPlan()`
uses its metric query. Rust connects start and goal through the prepared
surface, searches sparse corridors, and searches surface cells for a detour
when the sparse corridors are blocked. Connections, full candidate routes and
smoothing use the Rust metric, body, support, sweep and no-go checker. A rejected
turn excludes its directed incoming connection for that request, then search
continues. This conservative exclusion can also remove the edge for a different
arrival heading; the graph does not store a full heading state. Endpoints,
detours and retries share the request budget,
and cancellation returns without publishing a route. Exhausted searches can
fall back to the 3-D grid. The C++ `SurfaceGraph` remains only for explicit legacy
callers. New `.8` prepared artifacts restore the Rust graph directly and attach
raw occupied/free evidence from the same OctoMap without rebuilding the graph.
The map-bound geometry index is reused by edge checks and invalidated by updates.

The sparse graph retains the four-neighbor topology of the pinned DimOS
reference in [SURFACE_NOTICE](SURFACE_NOTICE), commit
`26144f7fd8424bdc33ab97dcd58e9497f2d413ee`. LingTu's checked full-surface detour
locally extends its candidate connections by enumerating independent X and Y
offsets within the existing `horizontal_hop_cells` range. This supplies the
`(1, 2, 1)` stair-turn connection missed in the spiral fixture. Every candidate
passes the geometry check and the configured step, slope and wall-distance
rules. The change adds no step or slope threshold.

For taller treads, the checked detour also considers the horizontal run required
by that height change and the existing slope limit. A two-cell rise at a 0.577
slope needs a four-cell run; a hop derived from a one-cell rise cannot connect it.
This required run is a minimum, not a maximum connection radius. Checked
cross-height candidates cover the per-level hop over the full height change,
including diagonal offsets. Flat gaps and unchecked adjacency retain their
previous range. This is a LingTu extension;
the pinned DimOS adjacency connects immediate XY neighbors by a step threshold.
Route simplification retains intermediate terrain heights when a proposed
shortcut departs from the sampled height profile, while flat-floor offsets and
straight ramps can still simplify. Fixed-map and paired-door/stair evidence is
recorded in `build/mls-stairs-fix-20261116/`.

SCAN and the global planner derive their dual-cylinder body from the same radius,
offset and vertical configuration. Straight segments sweep both cylinders;
vertices and explicit endpoint headings check their circular turning sweep.
Start occupancy exemption applies only to the initial measured footprint, never
to no-go regions. An unsupported actual start does not exempt the rest of its
entry segment from ground support checks.

Surface Z denotes the occupied voxel's **top**, as in DimOS. Body candidates add
the configured support height to that top; using voxel centres lowers the body
by half a map cell. Height fitting uses the same surface-top convention.
Smoothing preserves the existing wall-distance soft cost. It can prefer more
room without adding a hard clearance or changing the body size.

`PreparedArtifactHandle` holds a Rust-side `PlannerState` through the C ABI.
The migration smoke test covers artifact creation, surface and graph export,
graph-only query, limited endpoint query, regional update, stable revision, and
destruction. Production `PlannerSession` owns the loaded OctoMap and updates its
Rust artifact and query caches under the same mutex. The
legacy C++ graph remains only for explicit compatibility callers; new prepared
caches contain only the Rust artifact.

Rust exposes metric route checks for production queries and same-input geometry
comparison. Inputs include body-reference poses in metres, dual-cylinder radius
and offset, vertical extent, support calibration, step/slope limits, no-go,
initial-footprint exemption and external preblocked cells.
`octoplanner3d_route_check_smoke` compares its results and first failing segment
with C++ on flat ground, stairs, a thin wall, missing support and no-go regions.
It requires raw occupied evidence. Region extraction reads surrounding context
and complete height columns, then replaces only affected surfaces; a clipped
high wall must not create a false surface inside the wall. This update path
consumes committed saved-map revisions, not rolling live collision windows.

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

The planner resolves the requested terminal once at map XY spacing. A valid
physical target is retained; otherwise the nearest valid candidate must satisfy
all configured XYZ, XY and Z tolerances. Both the surface graph and grid search
connect to that same resolved terminal. Graph attachment radius controls the
connector search, not another choice of final goal. Lattice-indexed body and
snap caches are cleared when the query lattice changes; validated grid vertices
reuse their collision result while continuous edges are still checked.

Prepared graph vertices use the nearest collision-free stance within the
existing support-height band, including voxel extent, rather than forcing
every vertex to exactly ground plus nominal body height. Static body clearance,
support and metric edge limits are checked at construction. Query-time checks
still apply the request start exemption, no-go policy and temporary overlays.
This changes the derived graph, not the saved OctoMap or the completion limits.

## Persistent preparation and query controls

`PlannerSession` saves the completed Rust MLS artifact (observed/inferred
surface cells, nodes, edges and corridors) in
`<map-directory>/.nav/profile-N.bin` after the
first successful preparation. Later processes restore that artifact instead
of fitting the same holes and constructing the same graph again. Preparation
is lazy at navigation startup; saving a new raw map alone does not generate
the graph. The original OctoMap and point cloud remain measured evidence;
inferred support is stored separately inside the prepared artifact.

`navd` supplies the original map directory explicitly because its planner reads
a private immutable snapshot. That snapshot retains the original modification
time, so a new temporary filename does not invalidate preparation. Cache files
are replaceable derived data: an unsupported/truncated artifact is rebuilt,
and an unwritable directory leaves planning usable but prevents reuse after
restart. The binary format has an explicit version and endianness marker;
it is not a public interchange format. Copy the map directory, including `.nav`,
to retain prepared profiles; downloading only a PCD does not include them.

| Change | Effect |
| --- | --- |
| Map identity/version or source size/modification time | Reject stale disk preparation |
| Robot envelope, support-height band, support/completion rules, search spacing, step/slope or graph costs | Select a matching profile, otherwise rebuild and save |
| Goal XYZ/XY/Z tolerances, attachment radius, iteration limit, same-floor query preference | Update the existing planner and graph query without rebuilding |
| Start/goal, request cancellation deadline, no-go policy, temporary request overlay | Apply to this request; do not persist as static geometry |

`PlannerConfig::preparation()` declares preparation inputs; `setQuery()` updates
the live query consumers. Identical map/preparation inputs reuse static edge
validation during graph search. New endpoint connectors and smoothing chords
still receive full 3D checks, as does the assembled route before smoothing.
Requests carrying no-go regions or a temporary overlay retain full per-edge
checks. This optimization adds no real-time occupancy input to global planning.
The current field map pipeline still replaces the artifact for a changed
OctoMap; it does not yet provide map-cell deltas to this planner. The boundary
is ready for a regional update API, but claiming incremental updates before
that input exists would be false.

The matched 903 replay, graph visualization, cache roundtrip and native snapshot
test are recorded in `artifacts/nav/903-persistent-20261110/README.md`. These are
offline planning and native contract results, not field-motion acceptance.

## XYZ search and prepared surfaces

LingTu's independent `no_go.json` policy is a local extension to the pinned
OctoPlanner3D commit `9a9cc431ea905a5878975cc6fbbce6c9618b31a4` above. It is
not an upstream default. User regions remain finite 3D boxes, separate from
sensor occupancy. Global endpoint checks, traversable nodes and continuous
route segments reject intersection with those regions using the configured
cylindrical body clearance; no new obstacle margin is added. The swept check
uses circle/rectangle distance in XY and bounded vertical overlap, so a box
corner is not expanded into a square and a thin region cannot fall between
path samples. The saved OctoMap is unchanged.

The active-map gate captures the policy alongside its private OctoMap snapshot
and supplies that same immutable policy to planning. Standalone `runPlan`,
cached sessions and offline projections also read the sidecar when no explicit
validated request policy is provided. Malformed sidecars fail map preparation.
Removing a restriction only changes policy; measured occupied nodes remain.

The pinned OctoPlanner3D search uses position states. LingTu adds cached
multi-layer support surfaces, sparse corridors, continuous 3D connections
and validated route shortening and smoothing. Coarse, fine and fallback
search key states by position. Connections check static occupancy, support,
step and slope limits at map resolution. The nominal XY search spacing is
0.10 m. If that lattice exhausts its open set, the query retries once with XY
spacing at the finer OctoMap resolution, the same geometry checks and remaining total
iteration budget. Cancellation and iteration-limit failures do not retry.
Queries reuse the topology prepared for the map and its configuration; the
retry does not rebuild it, and the next request starts at nominal spacing.
This coarse-first search is a LingTu extension; the pinned upstream searches
the OctoMap grid directly.

The Rust MLS surface uses the DimOS-compatible
surface rules. Each occupied column keeps cells with the configured robot
headroom, each height sheet receives an L1 closing, and newly closed cells must
remain standable and have nearby occupied support. Closing is performed per Z
sheet, so it does not join separate stair surfaces. Raw occupied and known-free
evidence remains available to the body and motion safety layers. Surface
generation no longer applies a second plane fit, residual threshold, hole-area
filter, or known-free inference rejection. The old `support_radius_cells` /
`--octo-support-radius-cells` / environment setting remains removed from
current configs, launch adapters and tools. Direct-only mode disables closing.
Map or surface-option changes rebuild the Rust artifact and graph; this is a complete
rebuild, not per-frame regional updating. The original DimOS workflow at
`26144f7fd8424bdc33ab97dcd58e9497f2d413ee` and bounded-hole selection from
elevation_mapping_cupy `d9e1c9fe42ea3d463519333a443da99f680adde8` inform this
local implementation. See `src/maps/SURFACE_NOTICE` and the 2026-11-08 report.

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
