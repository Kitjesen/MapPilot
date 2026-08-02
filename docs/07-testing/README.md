# LingTu Validation Gates

Status: current validation index
Updated: 2026-07-28

This directory contains reusable gates. Dated results belong in
[`field-runs/`](field-runs/README.md). A passing lower-level gate never implies
a higher-level Product or field claim.

## Evidence Levels

| Level | Proves | Does not prove |
| --- | --- | --- |
| Local contract | Schema, algorithm, fail-closed behavior, and process ownership in focused tests. | Cross-process timing or robot behavior. |
| Native MuJoCo | The named Product chain with native processes and typed DDS in simulation. | MID-360 artifacts, Brainstem execution, or physical safety. |
| Field no-motion | Release provenance, topic writers, readiness, route preview, and stop barriers on target. | Safe commanded locomotion. |
| Supervised field motion | The named bounded motion scenario on the physical robot. | Any untested environment or capability. |

The capability-level evidence ledger is
[`NAVIGATION_CAPABILITY_MATRIX.md`](../architecture/NAVIGATION_CAPABILITY_MATRIX.md).

## Current Gate Index

| Document | Purpose |
| --- | --- |
| [`ALGORITHM_VALIDATION_FLOW.md`](ALGORITHM_VALIDATION_FLOW.md) | Separates algorithm tests, simulation evidence, and field claims. |
| [`FIELD_MAPPING_ACCEPTANCE.md`](FIELD_MAPPING_ACCEPTANCE.md) | Product map/save/restore acceptance. |
| [`MUJOCO_NAVIGATION_ACCEPTANCE.md`](MUJOCO_NAVIGATION_ACCEPTANCE.md) | Native-DDS navigation acceptance. |
| [`MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md`](MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md) | `autonomy`, `teleop`, and `teleop_avoid` promotion rules. |
| [`MUJOCO_MID360_FIDELITY.md`](MUJOCO_MID360_FIDELITY.md) | Optical and scan-time fidelity, separate from navigation success. |
| [`SEMANTIC_MEMORY_FIELD_CHECKLIST.md`](SEMANTIC_MEMORY_FIELD_CHECKLIST.md) | Reusable semantic-memory field checklist. |
| [`thunderv4_mujoco_lidar_recording_requirements.md`](thunderv4_mujoco_lidar_recording_requirements.md) | Required sensor recording evidence for ThunderV4 MuJoCo work. |
| [`mujoco_scene_design_guidelines.md`](mujoco_scene_design_guidelines.md) | Scene construction constraints for reproducible tests. |
| [`COMMIT_PUSH_POLICY.md`](COMMIT_PUSH_POLICY.md) | Local commit/push checks. |

## Native Product Acceptance

Every accepted native Product report must include:

- the exact Product declaration, env, RunPlan fingerprint, and artifact source;
- selected executable/library paths, hashes, and source/IDL freshness;
- writer ownership for critical DDS topics;
- input freshness and frame/epoch/sequence evidence;
- cleanup, zero-command barrier, and process exit results;
- explicit blockers, including tests that did not start.

Strict preflight is fail-closed. A stale or missing native artifact is a release
blocker, not a failed scenario and not permission to use a Python fallback.

For `teleop_avoid`, the authoritative Product contract is
`config/runtime_graph/products/teleop_avoid.yaml`: SLAM runs in `mapping`, a
saved map is not required, standalone native traversability owns
`/nav/traversability`, and mapd provides scene/map state without becoming the
control-risk writer.

Dynamic residual acceptance uses the `moving_person_clear` scenario in
`sim/scripts/mujoco/teleop_avoid_native_acceptance.py`. It must prove that the
person was observed, vacated voxel/accumulated occupancy returned near baseline,
generations stayed monotonic, resource limits held, and no stale obstacle epoch
survived reset. Code for column carving and decay alone is not this evidence.

## Focused Local Gates

Start with the narrowest affected suite. Common baselines are:

```bash
python -m pytest src/runtime/tests/ -q
python -m pytest tests/contracts/ -q

cmake -S src/nav/cpp -B build/nav-cpp \
  -DCMAKE_BUILD_TYPE=Release \
  -DLINGTU_NAV_CPP_BUILD_TESTS=ON \
  -DLINGTU_NAV_CPP_BUILD_ENDPOINT=OFF \
  -DLINGTU_NAV_CPP_BUILD_PYTHON=OFF
cmake --build build/nav-cpp -j
ctest --test-dir build/nav-cpp --output-on-failure
```

Run subsystem-specific build scripts and tests for maps, SLAM, drivers, or web
changes. Do not replace a failed native build with a mock-only test.

## Hardware P0 Scripts

These scripts are operator-driven gates, not general development commands:

| Script | Scope |
| --- | --- |
| `p0_cold_boot.sh` | Cold-start process and readiness check. |
| `p0_mapping.sh` | Mapping, save, validation, and activation. |
| `p0_route_safety.sh` | No-motion route preview and command-source check. |
| `p0_goto.sh` | Supervised point-goal motion after preview. |
| `p0_estop.sh` | Stop latency under commanded motion. |
| `p0_explore.sh` | Exploration start/stop in a prepared area. |
| `p0_all.sh` | Ordered wrapper; motion and exploration remain explicit. |

Each run writes a new date-prefixed note in `field-runs/`. Never edit an old
record to make a current build look successful.

## Legacy Aggregate

`l25_fresh_closure.sh` remains a compatibility aggregator for older simulation,
replay, and benchmark reports. It is not the acceptance authority for the
current typed-DDS `mapd/navd` Product chain. New native claims must use the
dedicated MuJoCo/Product gates above.

## Rules

- Never describe MuJoCo evidence as field evidence.
- Never use unit-test success to claim a Product capability is complete.
- Never bypass strict provenance, topic ownership, or freshness checks.
- Never publish a motion command merely to diagnose readiness.
- Record blocked gates as blocked; do not manufacture maps, samples, or mtimes.
- Update `docs/plans/current-roadmap.md` only when priority or completion state
  changes; keep run details in `field-runs/`.
