# explore

`src/explore` is the LingTu exploration package. It owns where-to-look /
where-to-go-next decisions for unknown or partially mapped space.

Exploration does not control the robot directly. It publishes exploration goals
or candidate goal sequences. Navigation still owns:

- OctoPlanner3D global planning;
- local planning and obstacle avoidance;
- path following;
- safety checks;
- final logical `/nav/cmd_vel` output. On `thunder_field` this is encoded as DDS
  wire topic `rt/nav/cmd_vel` and consumed only by `lingtu-driver`.

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

The `tare_explore` field product uses the standalone native endpoint:

```text
Gateway / operator
  -> /nav/exploration/command (START, PAUSE, RESUME, STOP)
  <- /nav/exploration/ack (matching request_id, business acceptance)

/slam/odometry + /tf + /nav/exploration_snapshot
  -> lingtu_explore_dds (ExploreControl + TarePolicy)
  -> /nav/command/request (goal)
  <- /nav/command/ack
  -> OctoPlanner3D / LocalPlanner / PathFollower / Safety
```

`lingtu_explore_dds` starts idle. It cannot select or submit a goal until a
fresh, map-frame `START` request is accepted and odometry/snapshot inputs pass
the freshness gate. Duplicate request IDs replay the cached ACK without
repeating the transition. Pause and stop clear queued work; stop remains
callable even when the telemetry file is stale.

`/dev/shm/lingtu/explore_status.json` is read-only telemetry with a bounded age.
It never grants command authority. Gateway commands use
`liblingtu_nav_client.so` and complete only after the matching DDS ACK.

The Python `TAREExplorerModule` and nanobind path remain for development and
compatibility profiles; they are not the `thunder_field` product control plane.
Assembly resolves that field profile with `owner="native"`, so Blueprint omits
both `TAREExplorerModule` and `ExplorationSupervisorModule`; `nav.commands` is
the only Gateway control adapter for the native endpoint.
Compatibility wrappers remain under `nav.exploration.*` for old imports. New
code should import from `explore.*`.
