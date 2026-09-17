# LingTu Explore C++ Core

This directory owns field exploration algorithms that must not depend on
Python, ROS, or the LingTu Module runtime.

## Layout

| Path | Owns |
| --- | --- |
| `bindings/` | Nanobind exposure for Python development and simulation. |
| `endpoint/` | `lingtu_explore_dds` process entry and exploration lifecycle support. |
| other sources | Transport-free exploration contracts and policies. |

## Current core

`explore_contract.hpp` defines the interface all exploration algorithms must
implement:

```cpp
class IExplorePlanner {
 public:
  virtual const char* name() const = 0;
  virtual ExploreDecision plan(const ExploreInput& input) const = 0;
};
```

`TarePolicy` is one implementation. It selects the next exploration viewpoint
from:

- `Grid2D`: `0` free, `100` occupied, `-1` unknown;
- `Pose2D`: current robot pose in map frame;
- visited goals, used to suppress repeated viewpoints.

It returns:

- `TareDecision.has_goal`;
- selected goal in map frame;
- scored candidates;
- a machine-readable reason.

The policy performs:

```text
reachable free-space flood fill
-> frontier detection next to unknown cells
-> frontier clustering
-> reachable candidate viewpoint search
-> score by frontier coverage, distance, heading momentum, novelty
```

## Python bindings (nanobind)

### Semantic search proposals

`semantic_views.hpp` provides the transport-free `ProposeSemanticViews` query.
It reuses `TarePolicy` for reachability and viewpoint ranking, without changing
an executing exploration policy. It takes a saved-map snapshot, per-cell robot
reference heights, and camera views bound to that map and reset epoch. The
camera view footprint uses range, horizontal field of view and map occlusion;
it is modeled geometric coverage, not proof of successful visual recognition.

The result contains candidates with heights and `geometry_exhausted`, not a
motion command or semantic task success. Unsupported heights remain unknown.
Queries can be repeated at the same map generation as camera observations are
added. Query cancellation and failed planning return no candidates. The query
does not infer traversability across height discontinuities; every candidate
still requires native 3D path admission.

The native navigation endpoint now serves a read-only query on
`/nav/semantic/views/request` and `/nav/semantic/views/result`. Its worker uses
the active saved OctoMap and the global planner's ground-support and clearance
predicate. Candidate heights come from the tested, snapped reference layer;
unsupported cells stay unknown. This is a single-height query, not a
multi-floor visibility model. It does not use the display projection sidecar.

Requests carry the native boot identity, map identity, localization epoch,
camera range/FOV, and camera-view history. Missing camera parameters are rejected
instead of assuming a 360-degree LiDAR. History cannot cross a height-layer or
localization change. The worker is cancellable and does not send motion.
Camera history excludes only observed directions, not entire visited positions;
an in-place turn can cover new geometry in a small reachable region.

`visible_cells` is a bounded geometric gain estimate, not an object-detection
confidence. Saved-map reachability and ray visibility do not prove a live
collision-free turn or a visually observable object. Every candidate still
needs native motion admission and new camera evidence.

The persistent native client, Host registry and SemanticPlanner search loop are
not yet connected to this request. Do not claim field semantic search is ready.
`/nav/exploration_snapshot` still lacks per-cell support/reference height; do not
replace the native query with a zero-filled or guessed height array.

The test target `test_semantic_views` checks camera coverage versus map coverage,
opposite views at one map generation, occlusion, unsupported connectivity,
map-bound histories, exhaustion, cancellation and successive in-place turns.
`test_tare_policy` covers the existing exploration policy separately.
`test_semantic_view_query` exercises the worker using a real OctoMap fixture and
typed DDS requests/results. These local tests do not prove field behavior.

The C++ core is exposed to Python through the `lingtu_explore_kernel` nanobind
extension. The bindings live in `bindings/`:

- `bindings.cpp` — `NB_MODULE(lingtu_explore_kernel, m)` entrypoint.
- `bind_types.cpp` — `Grid2D`, `Pose2D`, `ExploreInput`, `ExploreCandidate`, `ExploreDecision`.
- `bind_tare.cpp` — `TarePolicyConfig` and `TarePolicy`.

Build with:

```bash
scripts/build/build_explore_py.sh
```

This produces the extension under `src/explore/cpp/build_nb` (Linux) or
`build_nb_win` (Windows). The Python loader in `src/explore/kernel/` discovers
it at runtime; `explore.explore_kernel_available()` reports availability.

## Runtime target

The robot-side runtime should use `lingtu_explore_dds`:

```text
/nav/exploration_grid + /slam/odometry
  -> C++ TarePolicy
  -> NavigationCommandClient
  -> /nav/command/request + /nav/command/ack
  -> C++ nav endpoint
  -> OctoPlanner3D / LocalPlanner / PathFollower
```

Python wrappers may remain for compatibility tests and old profiles, but field
behavior should be implemented here.

`endpoint/` is built by `src/nav/cpp/endpoint/CMakeLists.txt`; this keeps the
existing navigation linkage and public `lingtu_explore_dds` target while the
source remains owned by the exploration domain.
