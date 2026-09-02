# Operations

This page is the operating playbook for a running LingTu system: observe first,
separate a no-motion diagnosis from a state change, recover through the Product
lifecycle, and re-run the gate that justified the next action. It is
not a replacement for the robot's local emergency procedure or supervision
rules.

> **Status:** Current navigation entry point<br>
> **Audience:** Robot operators, field integrators, and on-call maintainers<br>
> **Runs on:** Supported field robots; selected observation commands also apply to local and simulated runs

## Operating principles

- Use `bash scripts/lingtu` only for Product `switch`, `status`, and
  `stop`. Use Gateway routes for application operations and native
  `systemctl`/`journalctl` for service state and logs.
- Start every investigation with observation. A process being active does not
  prove that localization, map artifacts, planner inputs, or safety are ready.
- A no-motion check can still change state. For example, a comparative route
  check may switch and roll back localization, while map validation can rebuild
  an artifact. Run those checks only while the robot is stationary and clients
  do not hold control.
- Treat a Product switch as recovery, not proof that navigation is safe.
  Repeat readiness, localization, map, and route gates afterward.
- The normal field product path is native DDS. Do not start ROS 2 compatibility
  services beside it unless an explicit compatibility test requires that
  ownership change.

The [deployment guide](../04-deployment/README.md) is the authority for
service layout and installation; the [robot operations CLI reference](../04-deployment/lingtu_cli.md)
is the authority for every subcommand and its arguments.

## Command effect key

| Class | What it may do | Examples |
| --- | --- | --- |
| **Observe** | Reads state only; does not intentionally publish motion or change a session. | `status`, `health`, `log`, `dataflow`, `doctor --non-motion`, `soak`, and saved-map artifact checks. |
| **No physical motion, stateful** | Does not publish a goal or velocity command, but can change Product/session state or create artifacts. | `system-acceptance` without `--allow-motion` and relocalization. |
| **State-changing recovery** | Changes a Product, map/session, backend, or lease. | `switch`, `stop`, session start/end, and backend switching. |
| **Can move hardware** | Can submit a velocity/goal or activate autonomous behavior. | Teleop, `nav goal`, semantic instruction, visual-servo find/follow, and exploration start. |

For an emergency, follow the site emergency procedure first. Software
stop/cancel interfaces are part of the control path, but do not substitute for
physical separation, a safety observer, or a hardware emergency mechanism when
those are required by the site.

## Normal operating loop

### 1. Establish a read-only baseline

Run these from the authorized robot-side shell before changing any session or
service:

```bash
bash scripts/lingtu status
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
PYTHONPATH=src python -m diagnostics.field.soak --duration 120 --interval 2 --json --strict
```

`status` is a one-screen snapshot of session, SLAM, robot pose, mission,
control, map, and recent logs. `health` is the detailed Gateway health view.
`doctor --non-motion` checks the service/Gateway/dataflow path without a motion
request. `soak` samples freshness, map-cloud stability, stationary odometry,
and command-source idleness over an interval. Keep its JSON result with the
run record when it is a readiness gate.

At minimum, establish all of the following before authorizing a navigation or
exploration action:

| Surface | What to look for | Do not proceed when… |
| --- | --- | --- |
| Service inventory | Required native services are active for the current RunPlan. | A required service is failed or an unexpected legacy/experimental owner is active. |
| Sensor and dataflow | Current LiDAR/IMU, odometry, map cloud, and localization input are fresh. | A required stream is missing, stale, or has an unexplained duplicate producer. |
| Localization | The reported state and map-frame transform are valid for the active map. | Localization is lost/degraded, the pose is implausible, or `map_odom_tf` is invalid. |
| Map | The active package and selected planner artifacts are compatible with the current frame/source. | The map is stale, incomplete, or not the map used for localization. |
| Control | Safety allows operation and no unexpected teleop/servo/recovery source is active. | Safety is STOP/ESTOP or control ownership is uncertain. |

### 2. Run the smallest no-motion gate that answers the question

Use the appropriate artifact or field gate before an action that can move hardware:

```bash
# Native mapd artifact check: no goal or cmd_vel.
PYTHONPATH=src python -m diagnostics.field.map_artifacts <map-id> --require-occupancy

# Full native/Gateway acceptance for one map and target: no motion unless
# --allow-motion is explicitly added.
PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" \
  --map <map-name> \
  --goal <x> <y> <yaw> \
  --with-relocalization
```

Use [Task Guides](../05-guides/README.md) to select a map, route, semantic, or
exploration workflow. `system-acceptance` deliberately records several stages
of evidence; it is not merely a successful planner call. Do not add
`--allow-motion` unless a controlled field test has separately been approved.

### 3. Operate under supervision

During an approved mission, monitor the mission state, localization, command
source, safety state, and path rather than assuming a submitted request will
complete. The Gateway exposes read-only navigation, path, state, readiness,
and dataflow views; [Reference](../08-reference/README.md) maps those surfaces.

If a route blocks or a command source changes unexpectedly, stop sending new
goals. Move to the decision path below, preserve the evidence, and use the
narrowest recovery action.

## Decision path

```text
Unexpected behavior or failed gate
  |
  +-- Immediate physical hazard? --> Follow the local emergency procedure,
  |                                  then preserve status/log evidence.
  |
  +-- No immediate hazard
       |
       +-- Safety STOP/ESTOP or command owner unexpected?
       |     -> Hold motion; inspect control/safety state; do not restart blindly.
       |
       +-- Required service or Gateway unavailable?
       |     -> Capture status + logs; switch the Product again; re-run doctor/soak.
       |
       +-- Sensor or localization stale/lost?
       |     -> Inspect dataflow and map-frame state; repair/relocalize; re-run map/route gate.
       |
       +-- Map/artifact/route validation failed?
       |     -> Keep motion disabled; repair through MapsService; preview again.
       |
       +-- Mission rejected or repeatedly recovering?
             -> Inspect readiness blockers and path safety; cancel/stop the mission;
                do not retry with a different goal until the cause is understood.
```

## Diagnose and recover by symptom

### Gateway or service is unavailable

**Observe first:**

```bash
bash scripts/lingtu status
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
journalctl -u 'lt-*' -p err -n 100 --no-pager
journalctl -u 'lt-*' -f
```

After preserving status and logs, recover through the Product lifecycle:

```bash
bash scripts/lingtu --robot <vendor/model> --env real switch <product> [--map <name>]
```

ProductControl resolves the Product, owns process ordering, and waits for
readiness. After switching, repeat `status`, `doctor --non-motion`, and a short
`soak`; then repeat the map or route gate that the interrupted operation
required.

### LiDAR, IMU, odometry, or map-cloud data is missing

**Observe:**

```bash
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/lidar_scan"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/imu"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/map_cloud"
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
```

Correlate dataflow with the native service logs before changing the Product. The
product chain has one owner for a physical input. Do not start an external
LiDAR/SLAM service merely to make data appear; parallel owners can fight for
the same hardware and produce ambiguous data.

When recovery is justified, capture the preceding logs, switch the Product,
then verify freshness again. If the same condition recurs, stop the recovery
loop and escalate with the collected diagnostics.

### Localization is degraded, lost, or not aligned to the map

**Observe:**

```bash
bash scripts/lingtu status
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/map_cloud"
```

For saved-map navigation, `TRACKING` alone is insufficient. The runtime must
report a valid map-to-odometry transform and a credible pose inside the active
map frame. A low-quality relocalization or an identity map transform is a failed
localization gate even if the SLAM service remains alive.

The recovery order is:

1. Hold/cancel the current mission and confirm there is no active command
   source.
2. Inspect the active map, frame, and localization status.
3. Switch the Product again only after preserving the failure evidence:

   ```bash
   bash scripts/lingtu --robot <vendor/model> --env real switch <product> [--map <name>]
   ```

4. If the service is healthy but the pose needs recovery, use the documented
   saved-map relocalization flow (`nav relocalize` or global relocalization) in
   a stationary, supervised condition.
5. Re-run the no-motion map/route gate before accepting another goal.

Switching the Product and relocalizing are different claims: the first shows
the Product can become ready again; the second shows the current pose can be
aligned to the selected saved map.

### Map package or route preview fails

Keep motion disabled. Start with the map and route evidence rather than
modifying files in the map directory:

```bash
PYTHONPATH=src python -m diagnostics.field.map_artifacts <map-id> --require-occupancy
PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" --map <map-name> --goal <x> <y> <yaw>
```

The map service owns map versions, rollback, artifact building, and integrity.
A source PCD edit invalidates derived planning artifacts; rebuilding must go
through the map operation so it is published atomically. ProductControl owns
active-map changes. See the [map service contract](../architecture/MAP_SERVICE_CONTRACT.md)
for the underlying invariants.

### Navigation is rejected, blocked, or repeatedly recovers

Read the navigation status and path-safety details before sending another goal:

```bash
bash scripts/lingtu status
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
```

Then query the Gateway's read-only navigation/status/path surfaces or run the
route preview for the exact target. Common blockers are a missing map artifact,
invalid frame, localization loss, active manual takeover, or a safety/path
rejection. Do not try to solve a safety rejection by bypassing the velocity
mux, resubmitting faster, or issuing direct velocity commands.

Use a graceful navigation cancel or session end for a controlled mission stop.
For immediate hazards, use the emergency procedure and the documented emergency
stop endpoint. The generated [Gateway reference](../api/gateway_rest.md)
distinguishes `stop`, `navigation/cancel`, and session operations.

### Safety STOP/ESTOP or unexpected teleop ownership

Treat this as a control-boundary incident, not a navigation-planning failure.
Keep motion disabled, identify the active command source/lease, and preserve
the status and logs. Do not clear a stop by switching Products or restarting the
entire stack unless the local procedure directs that recovery. A stop/cancel
action changes control state but does not prove that the underlying sensor,
localization, or physical hazard has cleared.

## Product lifecycle

| Need | Command | Follow-up evidence |
| --- | --- | --- |
| Observe | `status` | Current Product, processes, map, and readiness. |
| Activate or recover | `switch <product> [--map <name>]` | All readiness gates declared by the resolved Product. |
| Stop | `stop` | Product processes and current lifecycle state are cleared. |

ProductControl owns the complete switch and stop transactions. The shell does
not select units or reproduce process ordering.

## Preserve useful evidence

Capture evidence before it scrolls away, especially before a Product switch:

```bash
bash scripts/lingtu status
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
journalctl -u 'lt-*' -p err -n 100 --no-pager
journalctl -u 'lt-*' -n 200 --no-pager | grep -i drift
journalctl -u 'lt-*' -f
PYTHONPATH=src python -m diagnostics.field.runtime_evidence --duration-sec 20 --json-out <report.json>
```

`evidence` is a read-only runtime dataflow/frame collection command. Store the
report together with the active Product and Product session ID, map version/name, intended goal frame, time
window, and whether any physical motion occurred. For a field validation
campaign, record `PASS`, `FAIL`, or `BLOCKED` and the reason in the relevant
[field-run record](../07-testing/field-runs/README.md). A log excerpt without
the corresponding Product, RunPlan, map, and readiness result is usually not enough to
reproduce a field issue.

## Escalate with a bounded handoff

Escalate rather than repeatedly restarting when any of these is true:

- a safety stop/physical hazard remains unexplained;
- the same service or input fault returns after one targeted recovery;
- map and localization frames disagree or an artifact gate cannot establish a
  compatible map package;
- the system reports a duplicate hardware owner or an unintended compatibility
  service;
- a no-motion gate fails and the reason is not clear from its saved evidence.

Include the command output, timestamp, Product session ID, RunPlan path, active map,
Gateway health, relevant dataflow, and the exact recovery action already taken.
This lets the next maintainer reproduce the state without guessing or exposing
the robot to another uncontrolled retry.

## Up next

Use [Task Guides](../05-guides/README.md) when you need the workflow for a
specific outcome. Use [Reference](../08-reference/README.md) for exact CLI,
REST, MCP, configuration, and contract lookup.
