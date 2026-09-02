# LingTu Test And Acceptance Map

Status: current test/acceptance map as of 2026-07-18.

This directory contains legacy integration and planning tests. The current
product acceptance path lives mostly under `tests/runtime`, `sim/scripts`, and
Gateway/CLI gates. Use this file as an operator-facing map, not as the only CI
entry point.

## Product Direction

LingTu is a Product-defined outdoor inspection navigation stack. ROS 2 topics can
exist at endpoint adapters, but product acceptance should not require an
operator to inspect `ros2 topic` output. The primary field-facing evidence is:

- Gateway readiness and runtime status
- Gateway runtime dataflow over Module `In/Out` ports
- Non-motion route preview and diagnostic pack
- Saved-map metadata and required artifact formats
- Real S100P runtime evidence before claiming real robot readiness

## Fast Checks

```bash
python -m pytest tests/runtime/ -q
python -m pytest tests/runtime/test_gateway_runtime_acceptance.py -q
python -m pytest tests/runtime/test_runtime_graph_contract.py -q
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
```

## Gateway Runtime Inspection

Use ProductControl `status`, the field doctor, and the Gateway diagnostic
routes. They expose the runtime without requiring ROS tooling:

```bash
export LINGTU_GATEWAY_URL=http://ROBOT_IP_OR_HOSTNAME:5050
PYTHONPATH=src python -m diagnostics.field.doctor --json --strict
```

For one stream, use the Gateway dataflow detail route instead of `ros2 topic`:

```bash
curl "$LINGTU_GATEWAY_URL/api/v1/runtime/dataflow/topic?topic=odometry"
curl "$LINGTU_GATEWAY_URL/api/v1/runtime/dataflow/topic?topic=cmd_vel"
```

The detail route reports whether the stream has a fresh ModulePort sample,
where payloads can be observed through Gateway REST/SSE/WebSocket, and whether
any whitelisted Gateway command can write to that stream. It never enables
arbitrary publish into ModulePorts.

`field-check` is the operator-facing one-screen readiness view. It consumes the
same lower-level gates rather than replacing them. `non_motion` mode checks that
Gateway exposes the product runtime boundary, runtime dataflow, readiness,
localization, navigation status, and command whitelist. `simulation` mode uses
the same Gateway and ModulePorts surface but requires a live simulation
endpoint, live Module samples, motion readiness, goal acceptance, and routecheck
evidence; it does not require real-runtime-evidence. `field` mode requires the
real S100P runtime boundary plus fresh passing real-runtime-evidence through
`/api/v1/diagnostics/real-runtime-evidence/latest`.

`inspection-check` is the customer-facing patrol preflight. It runs
`field-check`, reads saved locations through Gateway, then calls
`/api/v1/navigation/goal_candidate` with `preview=true` for each named point.
It is a no-motion acceptance pack: it does not publish goals, `cmd_vel`, or
arbitrary ModulePort/ROS messages.

The same verdict is available to Web/App clients through
`POST /api/v1/inspection/acceptance`. That endpoint is still read-only: it
builds Gateway snapshots in-process, resolves saved locations, previews plans,
and returns the canonical inspection acceptance schema without writing to
`goal_pose`, `cmd_vel`, `stop_cmd`, or arbitrary ROS/ModulePort channels.

## Contract And Field Gates

```bash
python -m pytest tests/runtime/test_runtime_graph_contract.py -q
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py

PYTHONPATH=src python -m diagnostics.field.map_artifacts <map-dir> \
  --require-octomap \
  --require-occupancy \
  --json-out artifacts/saved_map_artifacts/report.json

PYTHONPATH=src python -m diagnostics.field.runtime_evidence \
  --duration-sec 20 \
  --json-out artifacts/real_runtime/report.json

```

The validators and saved-map gate are non-motion. Runtime evidence collection is also a
read-only collector, but it must run during an operator-controlled real S100P
session to prove real runtime readiness.

## Simulation Closure

Server-side simulation closure is tracked by:

```bash
python -m sim.diagnostics --required-only --strict
python -m pytest sim/tests/test_sim_diagnostics.py -q
```

Simulation closure is useful algorithm evidence. It is not a substitute for
real S100P runtime evidence.

## Script Tests And Manual Smokes

Script-related checks are split by how they should be run:

- `tests/scripts/test_*.py` are pytest-collected script utility tests.
- `tests/scripts/smoke/*.py` are manual S100P/ROS/Gateway smoke scripts migrated
  from the old `scripts/test_*.py` paths. They are not pytest tests.
- `tests/scripts/*.sh` are legacy/manual shell integration helpers; read each
  script before running it on hardware.
