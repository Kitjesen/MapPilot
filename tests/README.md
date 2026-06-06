# LingTu Test And Acceptance Map

This directory contains legacy integration and planning tests. The current
product acceptance path lives mostly under `src/core/tests`, `sim/scripts`, and
Gateway/CLI gates. Use this file as an operator-facing map, not as the only CI
entry point.

## Product Direction

LingTu is a Module-first outdoor inspection navigation stack. ROS 2 topics can
exist at endpoint adapters, but product acceptance should not require an
operator to inspect `ros2 topic` output. The primary field-facing evidence is:

- Gateway readiness and runtime status
- Gateway runtime dataflow over Module `In/Out` ports
- Non-motion route preview and diagnostic pack
- Saved-map artifact provenance when a tomogram or occupancy artifact is used
- Real S100P runtime evidence before claiming real robot readiness

## Fast Checks

```bash
python -m pytest src/core/tests/ -q
python -m pytest src/core/tests/test_gateway_runtime_acceptance.py -q
python lingtu.py runtime-audit
```

## Gateway-Only Runtime Acceptance

Use this when the Gateway is running and the question is whether a customer or
field engineer can understand the runtime without ROS tooling:

```bash
python lingtu.py field-check \
  --gateway-url http://192.168.66.190:5050 \
  maps/active \
  --require-tomogram \
  --require-occupancy

python lingtu.py inspection-check \
  --gateway-url http://192.168.66.190:5050 \
  maps/active \
  --point pump_room \
  --point dock \
  --require-tomogram \
  --require-occupancy

python lingtu.py gateway-runtime-acceptance
python lingtu.py gateway-runtime-acceptance --acceptance-mode simulation
python lingtu.py gateway-runtime-acceptance --acceptance-mode field
python lingtu.py gateway-runtime-acceptance --gateway-url http://192.168.66.190:5050 --json
```

For one stream, use the Gateway dataflow detail route instead of `ros2 topic`:

```bash
curl "http://192.168.66.190:5050/api/v1/runtime/dataflow/topic?topic=odometry"
curl "http://192.168.66.190:5050/api/v1/runtime/dataflow/topic?topic=cmd_vel"
```

The detail route reports whether the stream has a fresh ModulePort sample,
where payloads can be observed through Gateway REST/SSE/WebSocket, and whether
any whitelisted Gateway command can write to that stream. It never enables
arbitrary publish into ModulePorts.

`field-check` is the operator-facing one-screen readiness view. It consumes the
same lower-level gates rather than replacing them. `non_motion` mode checks that
Gateway exposes the product runtime boundary, Module-first dataflow, readiness,
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

## Runtime Contract Gates

```bash
python lingtu.py runtime-audit \
  --json-out artifacts/runtime_contract_audit.json

python lingtu.py saved-map-artifact-gate <map-dir> \
  --require-tomogram \
  --require-occupancy \
  --json-out artifacts/saved_map_artifacts/report.json

python lingtu.py real-runtime-evidence \
  --duration-sec 20 \
  --json-out artifacts/real_s100p_runtime/report.json

python lingtu.py gateway-runtime-acceptance \
  --acceptance-mode field \
  --gateway-url http://192.168.66.190:5050
```

The first two commands are non-motion. `real-runtime-evidence` is also a
read-only collector, but it must run during an operator-controlled real S100P
session to prove real runtime readiness. The final command consumes that
evidence through Gateway; it should not require the operator to inspect
`ros2 topic` output directly.

## Simulation Closure

Server-side simulation closure is tracked by:

```bash
python sim/scripts/server_sim_closure.py --required-only --strict
python -m pytest sim/tests/test_server_sim_closure.py -q
```

Simulation closure is useful algorithm evidence. It is not a substitute for
real S100P runtime evidence.

## Legacy Integration Tests

The scripts under `tests/integration` and `tests/planning` may still be useful
for ROS-node or legacy deployment checks, but they are no longer the canonical
product acceptance path. Prefer the Gateway/runtime gates above for
Module-first field validation.
