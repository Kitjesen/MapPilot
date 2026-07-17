# explore

`src/explore` is the LingTu exploration package. It owns where-to-look /
where-to-go-next decisions for unknown or partially mapped space.

Exploration does not control the robot directly. It publishes exploration goals
or candidate goal sequences. Navigation still owns:

- OctoPlanner3D global planning;
- local planning and obstacle avoidance;
- path following;
- safety checks;
- `cmd_vel` output.

## Package layout

| Path | Role |
| --- | --- |
| `base.py` | Standardized `ExploreModule` base class (shared exploration port contract). |
| `cpp/` | C++ exploration algorithm core. This is the primary place for field exploration algorithms. |
| `kernel/` | Python loader for the `lingtu_explore_kernel` nanobind extension (mirrors `nav/kernel`). |
| `frontier.py` | Wavefront frontier exploration over `exploration_grid`. Used by the `explore` profile. |
| `traversable_frontier.py` | Frontier preview scored with traversability, slope, ESDF, and semantic evidence. |
| `tare/backend.py` | `create_nanobind_explore_backend` — wraps the native `TarePolicy` for Python callers. |
| `tare/policy.py` | Thin Python data contract (`TAREPolicyConfig`/`TAREDecision`) + `PortableTAREPolicy` delegating to C++. |
| `tare/module.py` | `TAREExplorerModule(ExploreModule)` — standardized exploration module. |
| `tare/supervisor.py` | Health and timeout supervisor for TARE exploration. |
| `tare/topics.py` | External CMU/TARE topic remap contract. |

## C++ first rule

Exploration algorithms that affect field behavior must live in `cpp/`. Python
holds **no algorithm implementation** — `PortableTAREPolicy.select()` delegates
entirely to the native `TarePolicy` exposed through `lingtu_explore_kernel`.
Python keeps only the stable data contract (`TAREPolicyConfig`/`TAREDecision`)
and the Module port orchestration.

The kernel is optional at runtime: `explore.explore_kernel_available()` reports
whether the extension is importable. When it is not, the module stays in
``configured`` state and tests/DDS paths remain usable. Build it with
`scripts/build/build_explore_kernel.sh`.

Current C++ core:

```text
Grid2D + robot Pose2D + visited goals
  -> lingtu::explore::TarePolicy  (via lingtu_explore_kernel nanobind)
  -> ExploreDecision(has_goal, goal, candidates, reason)
```

Python orchestration:

```text
exploration_grid + odometry
  -> TAREExplorerModule (ExploreModule, layer=5)
    -> PortableTAREPolicy.select(...)
      -> NanobindExploreBackend.plan(...)
        -> C++ TarePolicy
    -> exploration_goal / exploration_path / tare_stats
```

## CMU TARE boundary

This package is not the CMU original TARE runtime. The original project is a
C++/ROS runtime with its own launch files, topics, and simulator assumptions.
LingTu supports it only as an endpoint-owned external runtime through
`backend="tare_external"` / `cmu_unity`.

The target `tare_explore` product path is:

```text
OccupancyGridModule.exploration_grid
  -> C++ TarePolicy / lingtu_explore_dds
  -> exploration goal
  -> C++ NavigationCommandClient
  -> /nav/command/request + /nav/command/ack
  -> OctoPlanner3D / LocalPlanner / PathFollower / Safety
```

The existing Python Module path remains only until `lingtu_explore_dds` is wired
into the field profile.

Compatibility wrappers remain under `nav.exploration.*` for old imports. New
code should import from `explore.*`.
