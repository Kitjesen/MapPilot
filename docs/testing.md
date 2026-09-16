# Testing and Evidence

**Status:** Current validation and claim policy
**Audience:** Developers, release engineers, simulation users, and field operators
**Runs on:** Local development, native simulation, target compute, and physical robots

Tests are selected by the claim being made. The repository does not promote a
lower evidence level into a higher one.

## Evidence levels

| Level | Proves | Does not prove |
| --- | --- | --- |
| Local contract | The named schema, algorithm, ownership rule, or fail-closed behavior | Cross-process timing, simulator physics, or robot behavior |
| Native simulation | The named component or Product on the selected simulator and platform | Field calibration, networking, or physical safety |
| Field no-motion | Release identity, process ownership, freshness, readiness, and stop barriers on the target | Permission or evidence for locomotion |
| Supervised field motion | Only the recorded bounded scenario on the named physical robot | Other maps, speeds, environments, releases, or targets |

A running process is not a ready Product. A component pass is not a Product
pass. Windows and Linux/WSL must be qualified independently.

## Local checks

Run the narrowest check that can expose the failure under review:

```bash
python tools/validate/validate_docs.py
python -m pytest tests/docs/test_documentation_navigation.py -q
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
python -m pytest tests/runtime/ -q
```

Subsystems also have package-local tests and native CMake targets. A mock-only
result cannot replace a failed native build when native behavior is the claim.

## Product acceptance

A Product-level pass requires one exact ProductControl transaction and one
identity chain:

```text
Product + robot + env
  -> resolved RunPlan
  -> product_session_id
  -> declared processes and streams
  -> readiness
  -> scenario result
  -> terminal zero
  -> cleanup and rollback evidence
```

Manual process assembly, a dry run, a Catalog compile, or separate component
runs cannot satisfy this gate.

## Simulation gates

Simulation evidence must name the platform, package/session identity, Product
or component scope, scenario, clock/generation, and output artifacts.

Name the actual localization provider as well as the selected robot. The
default `teleop_avoid` component manifest uses simulated rolling LiDAR and IMU
through native Fast-LIO2, Mapd, SCAN, final motion arbitration, and the MuJoCo
driver. An explicitly selected `mujoco_navigation_fixture` supplies truth pose
and ground coverage; it isolates navigation and cannot validate Fast-LIO2 or
raw sensor-to-map behavior. Neither Thunder simulation nor an offline replay
validates Go2 sensor calibration or Unitree SDK2 motion.

An attached avoidance scenario must observe published final commands and
odometry progress, not just a nonzero planning-layer output or a drawn path.
The complete Product gate also checks MuJoCo physical motion and terminal zero.
RobotSimUE evidence adds presentation or render-sensor claims; Editor evidence
does not prove a Shipping package.

MID-360 fidelity checks should cover scan pattern, timestamps, metadata,
self-filtering, range behavior, and deterministic replay. Only real MID-360
evidence can justify a new field terrain algorithm or hardware-specific claim.

See [Simulation](./simulation.md) for the platform gates and current
qualification boundary.

## Field no-motion order

1. Resolve the exact Product and RunPlan.
2. Verify target, network, release, binary, map, and calibration identity.
3. Start only through ProductControl.
4. Check unique process and topic owners.
5. Check sensor, localization, map, navigation, and driver freshness.
6. Prove stop barriers and zero output.
7. Record the result before considering motion.

Common entrypoints:

```bash
bash scripts/lingtu status
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
PYTHONPATH=src python -m diagnostics.field.soak \
  --duration 120 --interval 2 --json --strict
```

Executable P0 procedures live under `scripts/gates/field/`. They include cold
boot, mapping, route safety, goto, emergency stop, explore, and the aggregate
gate. The scripts are the executable source; this page owns the evidence rules.

## Supervised motion

Motion requires a separately approved bounded scenario after no-motion gates
pass. The operator must know the map, route, speed bounds, stop mechanism,
clearance, and expected terminal condition.

The record must include the final zero-output state and driver acknowledgement.
An API ACK proves command admission, not actuator behavior.

## Evidence record

Store generated logs, JSON, MCAP, images, and reports under `artifacts/` or the
designated external evidence store. Link the immutable bundle from a commit,
pull request, release record, or issue.

Record:

- repository revision and release/artifact identity;
- Product, robot, env, RunPlan, and `product_session_id`;
- platform, scenario, map, and relevant configuration;
- exact commands and observed inputs/outputs;
- PASS, FAIL, or BLOCKED;
- the narrow claim proved and the remaining blocker.

Old Markdown run reports were removed from the live docs tree. Git history
retains them; they do not define current behavior.

## Current claim boundary

The repository supports local contract checks and selected native simulation
gates. No current document alone claims autonomous S100P readiness.

Field readiness requires fresh target, no-motion, fault-injection, and
supervised motion evidence for the selected release.

Open evidence gaps are tracked in [Roadmap](./roadmap.md).
