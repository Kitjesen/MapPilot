# LingTu Product Control

A Product is one complete robot working mode. `ProductControl` fixes one Robot
and one Env (`real` or `sim`), then exposes only `switch`, `status`, and `stop`.
Process selection, Host assembly, readiness, rollback, and the internal plan
file stay behind that interface.

Remote control of an already deployed robot belongs to `lingtu.sdk`, not this
package.

## Quick Start

```python
from lingtu.control import ProductControl

control = ProductControl(robot="doso/thunder_v4", env="sim")
control.switch("teleop")
control.status()
control.stop()
```

## Public API

| Entry | Purpose |
|-------|---------|
| `ProductControl` | Local `switch`, `status`, and `stop` |
| `lingtu.sdk` | Remote client for an already running robot |

`assembly`, real/sim switching, process runners, and plan files are internal
implementation. Host assembly manages only Modules inside the Python Host;
ProductControl owns the complete Product lifecycle.

`switch()` returns the committed `product_session_id` and typed readiness
evidence, but never exposes the RunPlan path. `stop()` accepts optional
`expected_product` and `expected_product_session_id` guards so in-repo
coordinators can stop only the exact Product run they started.

`status()` includes the current `product_session_id`. A caller can pass that
value as `switch(expected_product_session_id=...)` to reject a stale request
under the existing mutation lock, before resolving or stopping a Product.

`python -m lingtu.control serve --robot unitree/go2 --env real` exposes a
loopback transport on port 5051. It runs outside Host, so an accepted switch
continues when Host restarts. Gateway forwards requests; it does not own the
transaction. Request receipts survive transport restarts in the selected state
directory; unfinished requests are reported as interrupted and never replayed.
The Web offers mapping and saved-map navigation only. See
[operations](../../docs/operations.md#web-mapping-and-navigation-switches)
for deployment and the fixed robot/environment configuration.

Standalone camera, LiDAR, SLAM, navigator, and detector wrappers were removed.
Those capabilities live in domain Module packages and are selected by
`lingtu.assembly`; the internal Host module assembly mechanism remains in
`runtime.host`.

Host is still part of the formal Product runtime. The real entry
[`real/host.py`](real/host.py) runs under `lt-host.service`; the simulation entry
[`sim/host.py`](sim/host.py) runs as a direct child. Both load the RunPlan
published by ProductControl, build and start the selected Module graph, and
stop it on exit. Host carries Gateway, semantic/Agent tasks, and native-state
adapters; native C++ services own field LiDAR, SLAM, map state,
terrain/traversability, navigation, and driver hot paths.

Host module assembly is local to the Python Host. It does not start those native
services. See [Assembly](assembly/README.md) for the graph, configuration
precedence, and internal path changes.

## Remote Python SDK

`LingTuClient` configures one Gateway address, API key and request timeout. Its
domain clients share that configuration and own their request construction and
result handling. This is shared connection configuration, not a shared TCP
session: constructing a client makes no network request, and the stdlib HTTP
implementation opens each request through `urllib.request.urlopen`. The SDK
does not acquire robot control or start a Product when it is constructed.
`AsyncLingTuClient` exposes the same domains and method names;
await its methods. Both clients use the Python standard library.

| Property | Client | Responsibility |
| --- | --- | --- |
| `navigation` | `NavigationClient` | Goals, navigation status, paths, exploration, position and relocalization |
| `map` | `MapClient` | Saved maps, durable save operations, downloads, point clouds and tagged places |
| `tasks` | `TaskClient` | Text instructions, Agent runs, Follow executions, inspection routes, task control and reports |
| `observations` | `ObservationClient` | Scene graphs, camera snapshots, retained observations and native recordings |
| `system` | `SystemClient` | State, health, session, capabilities, readiness, diagnostics, software emergency stop and control leases |

These five entry points group caller workflows. Localization prepares and
observes live navigation; maps own durable spatial assets. Task methods keep
their execution kind explicit, such as `get_agent_run`, `get_follow_task` and
`inspection_task`, because those owners use different task identities.
Observation methods cover reading and capturing evidence. System methods cover
robot administration. Grouping clients does not combine backend task owners or
change their execution and cancellation contracts.

```python
from lingtu.sdk import LingTuClient, Pose2D

robot = LingTuClient("robot-host")
print(robot.system.health())
print(robot.map.list())
print(robot.navigation.position())
```

Use the selected Product's declared capabilities. Interface availability does
not mean that a model, calibration or physical motion has been qualified.
Navigation remains safety-mediated by the native navigation owner:

```python
receipt = robot.navigation.go(2.0, 1.0, yaw=0.0)
status = robot.navigation.status()
if receipt.accepted and receipt.task_id:
    task = robot.navigation.task(receipt.task_id)

saved = robot.map.save_and_wait("mapping_capture")
```

`map.save_and_wait()` follows the durable operation identity until its actual
result; saving does not activate that map. After ProductControl selects the
navigation map, relocalize on the current active map with a known pose seed:

```python
session = robot.system.session()
if session.active_map:
    seeded = robot.navigation.relocalize(
        session.active_map, initial_pose=Pose2D(x=known_x, y=known_y, yaw=known_yaw)
    )
```

A navigation or instruction receipt acknowledges admission, not arrival
or physical motion. Keep Agent run identities when reading or cancelling a run:

```python
receipt = robot.tasks.send_instruction("现在在干什么", control_only=True)
if receipt.run_id:
    run = robot.tasks.get_agent_run(receipt.run_id)
    # To cancel this exact run, use robot.tasks.cancel_agent_run(receipt.run_id).
```

`robot.navigation.cancel()` requests cancellation of navigation;
`robot.system.emergency_stop()` invokes the software emergency stop. They have
different semantics. Public SDK clients never send raw sport commands to the
robot driver.
Product switching and process lifecycle remain under `ProductControl`.

Native recording belongs to the observation workflow:

```python
recording = robot.observations.start_recording(prefix="mapping_capture", duration=600)
recording_status = robot.observations.recording_status()
# To stop the active recording, use robot.observations.stop_recording().
```

```python
from lingtu.sdk import AsyncLingTuClient

async with AsyncLingTuClient("robot-host") as robot:
    health = await robot.system.health()
    maps = await robot.map.list()
```

Close the root client when finished, or use it as a context manager. Domain
classes can also be constructed with an existing root client, for example
`NavigationClient(robot)`. Existing flat methods such as `robot.go()`,
`robot.save_map()` and `robot.start_follow()` remain supported delegates. The
historical `robot.localization` entry also delegates its relocalization and map
tracking methods to `navigation`; use the five domains above for new integrations.
Audio has not been added to the public SDK.
The complete external contract is documented in [API](../../docs/api.md).

## Layout

| Path | Responsibility |
| --- | --- |
| `control.py` | Public `switch`, `status`, `stop` and the CLI |
| `control_server.py` | Loopback request receipts and forwarding into ProductControl, outside Host |
| `run_plan.py` | Internal resolved Product data and JSON load/save |
| `real/` | Real-robot switch transaction and systemd ownership |
| `sim/` | Simulation switch transaction and direct child ownership |
| `assembly/` | Product compilation and internal Host module assembly |
| `assembly/compile_plan.py` | Compile one immutable RunPlan and rebuild its declared Host graph |
| `assembly/config/` | Resolve Product/Env inputs, endpoint bindings, native parameters, and simulation data |
| `assembly/graph/` | Product/Env declarations, variant resolution, process contracts, and validation |
| `assembly/host/` | Build selected Modules, connect them, and supply the concrete native-state bridge and plugin catalog |
| `assembly/build_scripts.py` | Select native build scripts from declared process roles |
| `sdk/` | Remote Gateway client |

Product behavior is declared in `config/system/products/*.yaml`.
Assembly translates `host.capabilities` into Modules and contributes the
Product's final topics, capabilities, and navigation values to the private
execution snapshot. ProductControl applies `--set` during compilation;
runners execute that snapshot. `RunPlan` is its current internal file format,
not a second public lifecycle API.

`load_runtime_graph()` returns raw Product declarations, including variants.
The compiler resolves the selected variant and passes Product identity, required
topics/capabilities, and the internal execution snapshot explicitly to Host
assembly. `host.config`
contains Module configuration; it does not carry hidden `_product` or `_run_plan`
metadata.

Variant selection happens once. `product_requirements()` only validates the
already selected declaration; there is no separate `ProductRuntime` object.
The generic runtime does not load this graph. Topic loading is owned by
`message/catalog.py`, shared by the graph, runtime routes, and the generator.

The internal snapshot stores navigation settings in `launch.native_nav` and other process
environment values in `launch.process_environment`. Its
`native_process_environment` property renders the final environment for process
launch. Navigation settings are never decoded back from environment strings.
The internal RunPlan v11 records each process's activation requirement. Regenerate
older plans through ProductControl; callers should not load or edit them directly.
The `map/camera` and `nav/camera` variants explicitly own a nonblocking `video`
process. A preview failure is reported under `degraded`; core process readiness,
rollback and stop requirements remain mandatory. Product status queries inspect
the optional process again, so a later failure or source recovery is visible.

Runtime status and message conversion have separate owners:

| Path | Responsibility |
| --- | --- |
| `gateway/services/runtime_facts.py` | Capture one lock-consistent set of Gateway runtime facts |
| `gateway/navigation/status.py` | Evaluate navigation admission and publish the shared public navigation projection |
| `gateway/services/runtime_status.py` | Project localization and runtime identity |
| `runtime/endpoints/dds/adapters.py` | Convert DDS payloads into Python Module messages |
| `assembly/host/native_state_bridge.py` | Receive, deduplicate, and publish those messages inside Host |

## Testing

```bash
python -m pytest \
  tests/lingtu \
  tests/lingtu/real \
  tests/lingtu/sim \
  tests/lingtu/assembly \
  tests/lingtu/sdk
```
