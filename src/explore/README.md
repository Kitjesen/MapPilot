# explore

`src/explore` is the LingTu exploration package. It owns where-to-look /
where-to-go-next decisions for unknown or partially mapped space.

Exploration does not control the robot directly. It selects the next useful
frontier or viewpoint. Navigation executes that decision through one short
route contract:

- `Map`: saved-map GlobalPlanner, then local planning;
- `Live`: identity-bound rolling segment, then local planning;
- local planning and obstacle avoidance;
- path following;
- safety checks;
- final logical `/nav/cmd_vel` output. In `env=real` this is encoded as DDS
  wire topic `rt/nav/cmd_vel` and consumed only by `lingtu-driver`.

## Package layout

| Path | Role |
| --- | --- |
| `base.py` | Standardized `ExploreModule` base class (shared exploration port contract). |
| `cpp/` | C++ exploration algorithm core. This is the primary place for field exploration algorithms. |
| `kernel/` | Python loader for the `lingtu_explore_kernel` nanobind extension (mirrors `nav/kernel`). |
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
Local development Hosts may consume its external topic boundary through
`backend="tare_external"`. This does not select CMU Unity as an `env=sim`
backend or create a Product RunPlan.

The `explore` field Product uses the standalone native endpoint. TARE is the
internal policy implementation, not a second Product:

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

## Field routes

The route names are intentionally short:

| Operator command | Route | Required map |
| --- | --- | --- |
| `lingtu explore start` | `Live` | no saved map; mapping builds the current rolling exploration grid |
| `lingtu explore start --map MAP` | `Map` | exact validated map identity; localization precedes coverage |

`ProductControl` derives the route from the presence of the map argument and
the resolved Product SLAM contract, then passes
`LINGTU_EXPLORE_ROUTE=live|map` to the native endpoint. `Live` submits
`/nav/exploration_segment/request` directly. It never sends a generic Goal to
GlobalPlanner first. Both routes require a fresh map-frame robot pose and a
fresh identity-versioned exploration snapshot.

### What a Live segment is

A Live segment is one short target request, not a persistent map route. The
request carries `request_id`, `session_id`, `reset_epoch`,
`minimum_generation`, and a map-frame target. `navd` matches it against the
same-generation execution snapshot, extracts only the currently observed-free
prefix, and publishes that prefix to its embedded LocalPlanner and
PathFollower. A reset, stale generation, unsafe prefix, cancellation, or
missing input terminates the segment and produces zero motion until a newer
snapshot permits replanning.

The native status snapshot exposes `pending_segment` and the segment request,
ACK, status, and terminal counters. This is the evidence used to distinguish a
real Live run from a generic saved-map goal.

Strict MuJoCo preflight and execution:

```bash
python sim/scripts/mujoco/explore_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_explore_native_acceptance.json \
  --artifact-dir artifacts/mujoco-explore \
  --preflight-only --strict

python sim/scripts/mujoco/explore_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_explore_native_acceptance.json \
  --artifact-dir artifacts/mujoco-explore \
  --strict
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
simulation; they are not the `env=real` Product control plane.
Assembly resolves that field Product with `owner="native"`, so Blueprint omits
both `TAREExplorerModule` and `ExplorationSupervisorModule`; `nav.commands` is
the only Gateway control adapter for the native endpoint.
All Python exploration code uses the single `explore.*` import surface.
