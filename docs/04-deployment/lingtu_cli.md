# `lingtu` Operations CLI

Status: current robot-side operations CLI contract as of 2026-07-18.

`scripts/lingtu` is the robot-side operations CLI. It is designed for SSH use
on the S100P/sunrise board and replaces ad-hoc `curl`, `systemctl`, and
`journalctl` commands during field work.

Deployment path on the robot:

```text
/opt/lingtu/current/scripts/lingtu
```

The Python application entry is still `lingtu.py`. The shell CLI starts,
stops, inspects, and coordinates the native field services around it.

## Local Alias

```bash
export LINGTU_ROBOT_HOST=ROBOT_IP_OR_HOSTNAME
alias lingtu='ssh sunrise@"$LINGTU_ROBOT_HOST" "bash /opt/lingtu/current/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@"$LINGTU_ROBOT_HOST" "bash /opt/lingtu/current/scripts/lingtu watch"'
```

Then:

```bash
lingtu status
lingwatch
```

Common read-only diagnostics:

```bash
lingtu doctor                   # read-only service/Gateway/dataflow diagnostics
lingtu doctor --ros2            # explicit ROS2 compatibility diagnostics
```

The default doctor path checks Gateway readiness, health, localization, navigation, state, and camera snapshot without requiring ROS topic inspection.

## Product Runtime

The normal field runtime is native DDS:

```text
lingtu-livox-dds
  -> lingtu-slam-dds
  -> LingTu Modules / Gateway / MCP
  -> lingtu-nav-dds
  -> DDS rt/nav/cmd_vel
  -> lingtu-driver
  -> remote Brainstem gRPC WalkChecked
```

Do not source ROS 2 or a colcon overlay for normal field navigation. ROS 2 is
only for explicit compatibility checks or legacy replay gates.

`lingtu-driver` is the unique speed exit in the current RunPlan. It
reads `/opt/lingtu/config/brainstem.env`, requires a remote Brainstem endpoint,
publishes `/dev/shm/lingtu/driver_status.json`, and fails closed if the lease,
ACK, DDS freshness, or gRPC connection is not ready.

## Status

```bash
lingtu status
lingtu watch [interval_seconds]
```

The status board is an 8-section snapshot:

```text
=== Lingtu @ 17:26:52 ===
[1] Session   mode=idle   product=idle   map=corrected_20260406_224020
[2] SLAM      hz=10.0Hz   live_pts=0   loc=GOOD
[3] Robot     xy=(1.2, -0.4)   z=0.05   yaw=12.3 deg   v=0.0 m/s
[4] Mission   state=IDLE   wp=0/0   replan=0   deg=NONE
[5] Path      (no active plan)
[6] Ctrl      teleop=False(0)   safety=OK
[7] Map       active   map.pcd=2.6M   opt=ok   patches=105
[8] Log       (recent drift / map-save / error)
```

| Section | Healthy | Bad |
| --- | --- | --- |
| Session | `idle`, `mapping`, `navigating`, `exploring` plus correct `product_session` | wrong product mode or missing active map |
| SLAM | nonzero Hz and `GOOD`/`OK` localization | 0 Hz, `DEGRADED`, `LOST` |
| Robot | bounded pose and plausible velocity | divergent odometry or impossible speed |
| Mission | `IDLE` or `EXECUTING` with bounded replans | `FAILED`, repeated recovery, stale goal |
| Ctrl | safety `OK`/`WARN` as expected | safety `STOP`/`ESTOP` without operator intent |
| Map | `map.pcd`, optimization metadata, and planner artifacts present | missing `map.pcd`, `octomap.ot`, or metadata |

## Mapping

```bash
lingtu map start
lingtu map save lab_0423
lingtu map end
lingtu map list
lingtu map restore lab_0423
```

`map start` is a thin CLI entry into `ProductControl`: it switches to the
`map` Product inside the fixed runtime `env`, resolves and stages one
fingerprinted `RunPlan`, commits that plan as current, and waits for readiness.
It is not a Bash shortcut that directly mutates a SLAM DDS mode.

`map save` submits a durable map-save operation through the Gateway map API.
The CLI waits for the operation to reach a terminal state and reports the
navigation-ready map result; callers should treat HTTP `202` as admission only,
not as a completed save. The resulting map directory is a package, not just one
PCD file.

Expected map bundle:

| Artifact | Purpose |
| --- | --- |
| `map.pcd` | optimized navigation map |
| `map.raw.pcd` | raw SLAM/builder output before save-time optimization |
| `patches/*.pcd` | keyframe or scan patches |
| `poses.txt` | patch poses for optimization and occupancy raycasting |
| `map_optimization.json` | loop/refine status, point counts, schema metadata |
| `metadata.json` | map package and planner artifact metadata |
| `occupancy.npz` | 2D occupancy/cost artifact |
| `octomap.ot` | OctoPlanner3D 3D map artifact |
| `tomogram.pickle` | optional legacy/PCT artifact |

Save admission example:

```json
{
  "ok": true,
  "success": null,
  "accepted": true,
  "status": "running",
  "reason_code": "map_save_in_progress",
  "request_id": "01K1M9S4FX27T8XMY6QJNBAV3W",
  "operation_id": "01K1MA6Q9J7R5C8D2N4P0V1X3Y",
  "operation": {
    "state": "RUNNING"
  }
}
```

`request_id` is the caller-provided idempotency key for retrying the admission
request. `operation_id` is the server-issued handle used to poll, cancel, or
retry the save operation; do not assume the two values are equal. The operation
is terminal only when `operation.state` is `SUCCEEDED`, `FAILED`, or
`CANCELLED`.

`dynamic_filter` is optional cleanup evidence. The product readiness gate is
the saved-map bundle plus `map_optimization.json`, `metadata.json`, and
`octomap.ot` when OctoPlanner3D navigation is required.

`map restore <name>` promotes the preserved raw/backup map back to `map.pcd`
when save-time cleanup removed static structure. The replaced file is kept as
`map.pcd.replaced-<timestamp>` and downstream artifacts are rebuilt.

The field CLI does not expose an ad-hoc artifact rebuild command. A successful
`map save` must produce its required artifacts as one operation; missing
`octomap.ot` or `occupancy.npz` is a failed/incomplete map result, not a reason
to bypass the operation through raw `curl`. Validate the completed package
with `lingtu saved-map-artifact-gate <map-directory> --require-occupancy`.
External map-management clients must use the authenticated API contract rather
than copying robot-local paths or unauthenticated rebuild requests.

## Navigation

```bash
lingtu nav start corrected_20260406_224020 --initial-pose 0 0 0
lingtu nav goal 3.5 2.1 0.0
lingtu nav stop
```

`nav start <map>` delegates to `ProductControl`. ProductControl stages the
selected saved-map package, resolves and executes the `nav` Product `RunPlan`
inside its fixed `env`, owns readiness checks and rollback, and commits the
current plan only after a successful switch. The CLI does not bypass
ProductControl or select localization profiles directly.

When `--initial-pose` or `--relocalize` is provided, the navigation startup
also requests saved-map relocalization as part of the controlled product
transaction.

Before sending a field goal, check:

- `relocalization_state=completed` when relocalization is requested.
- `map_odom_tf.valid=true`.

## That-nav Parity Gate

That-nav parity gate is the no-motion system acceptance flow for saved-map
navigation. It validates the native/Gateway service state, dataflow evidence,
active map artifacts, OctoPlanner3D preview, relocalization, and native nav
endpoint status before any controlled motion check is allowed.
It does not send motion commands by default.
- active map has `map.pcd`, `metadata.json`, and `octomap.ot`.
- safety state is not STOP/ESTOP.

The optional goal yaw is in radians. If omitted, heading defaults to `0.0`.

## Product Mode Switching

```bash
lingtu mode switch teleop
lingtu mode switch teleop_avoid
lingtu mode switch map
lingtu mode switch tracking --map lab_0423
lingtu mode switch nav --map lab_0423
lingtu mode switch inspection --map lab_0423
lingtu explore start
lingtu explore start --map lab_0423
lingtu explore task start
```

`explore start` activates the live-mapping variant; `explore start --map MAP`
activates the saved-map-localization variant. Both commands leave the
exploration task idle. Run `explore task start` only after Product readiness is
green and the motion area is supervised.

On the robot, `lingtu explore task start` and `lingtu explore status` read the
active Product session from `/run/lingtu/session.env` and send
`X-LingTu-Product-Session` only to the exact loopback exploration routes.
Remote Gateway clients still require `LINGTU_API_KEY`; the Product-session
header is not a general API credential.

For an isolated native exploration lifecycle check, use the installed control
binary. These commands wait for the typed endpoint ACK and return non-zero on
rejection or timeout:

```bash
/opt/lingtu/current/build/nav_endpoint/lingtu_nav_control \
  explore start field-session --request-id field-start --domain-id 0
/opt/lingtu/current/build/nav_endpoint/lingtu_nav_control \
  explore pause operator_pause --request-id field-pause --domain-id 0
/opt/lingtu/current/build/nav_endpoint/lingtu_nav_control \
  explore resume operator_resume --request-id field-resume --domain-id 0
/opt/lingtu/current/build/nav_endpoint/lingtu_nav_control \
  explore stop operator_stop --request-id field-stop --domain-id 0
```

Use a non-production DDS domain for no-motion integration tests. Running
`explore start` against the production domain can dispatch navigation goals
when fresh map and localization inputs are present.

The operator selects a Product such as `teleop`, `teleop_avoid`, `map`,
`tracking`, `nav`, `inspection`, or `explore`. Gateway `mode` and
`product_session` are derived observations of the active Product lifecycle;
they are not independent field-mode selectors.

Product switching may restart processes when the resolved RunPlan changes.
Visual-servo target/mode switching is the explicit hot-switch entry inside
a Host whose RunPlan already loads `VisualServoModule`.

### Operator-assisted local avoidance

`teleop_avoid` enables the native assisted LocalPlanner. The autonomous Product
modes `tracking`, `nav`, `inspection`, and the saved-map variant of `explore`
enable the same branch when a non-zero teleop command latches operator
takeover. This is a hot control
handoff inside the existing C++ endpoint; it is not a product-mode restart.

ProductControl stages these values in the boot-scoped
`/run/lingtu/session.env` consumed by the current RunPlan:

```bash
LINGTU_TELEOP_LOCAL_PLANNER=1
LINGTU_TELEOP_PLANNER_HORIZON_M=2.0
LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG=55.0
```

The joystick direction and magnitude are treated as operator intent. The native
LocalPlanner searches a short local detour, PathFollower generates the command,
and the final curved-path safety gate may still stop it. A blocked scene always
produces zero; assisted teleop does not autonomously reverse or run recovery
rotation. Pure `teleop` intentionally leaves this planner disabled.

Before live motion, run the endpoint with `LINGTU_NAV_PUBLISH_CMD_VEL=0` and
verify `teleop_local_planner: true`, `/nav/local_path`, and
`teleop_assist_detour` in the native status. Then restore publishing and begin
with a low operator speed. Releasing the WebSocket deadman sends zero and keeps
manual hold; use `POST /api/v1/navigation/resume` and submit a fresh goal to
return to autonomy.

The Web dashboard exposes the same contract in the `Runtime` tab:

- Product cards call read-only `POST /api/v1/runtime/switch-plan`.
- The response includes the exact `python -m lingtu.control switch ...`
  command for the resolved Product and map.
- The dashboard copies that command for an authorized operator; Gateway and
  Web never apply a Product switch or invoke systemd.
- Visual Servo `Find`, `Follow`, and `Stop` call `POST /api/v1/visual_servo`.

## Services

```bash
lingtu svc status
lingtu svc restart slam
lingtu svc restart nav
lingtu svc restart host
lingtu svc reapply
```

`svc restart` accepts exactly one RunPlan logical process:

```text
lidar | slam | maps | traversability | nav | driver | camera | explore | host
```

The corresponding real-env systemd targets are:

- `lidar` -> `lingtu-livox-dds.service`
- `slam` -> `lingtu-slam-dds.service`
- `maps` -> `mapd.service`
- `traversability` -> `lingtu-traversability-dds.service`
- `nav` -> `lingtu-nav-dds.service`
- `driver` -> `lingtu-driver.service`
- `camera` -> `lingtu-camera-dds.service`
- `explore` -> `lingtu-explore-dds.service`
- `host` -> `lingtu.service`

Useful native status files:

- `/dev/shm/lingtu/nav_endpoint_status.json`
- `/dev/shm/lingtu/driver_status.json`

`svc status` is read-only and reports all nine logical process labels. A
`svc restart <logical-process>` request sends that label unchanged to
ProductControl, which resolves exactly one process from the committed RunPlan.
It does not select a backend, accept a systemd unit name, or restart a backend
chain. `svc reapply` reapplies the exact committed Product; `svc restart all`
is the equivalent explicit `all` spelling. The shell does not choose systemd
ordering or run its own readiness loop.

## Logs And Health

```bash
lingtu log tail
lingtu log drift
lingtu log error
lingtu log all
lingtu health
```

`health` reads Gateway health and is the best single JSON source for module
state, runtime contracts, active map, mission state, and safety state.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `ros2: command not found` | Normal for product runtime; only compatibility checks need ROS 2. |
| `No active map` | Run `lingtu map list`, then `lingtu nav start <map>`. |
| `saved-map-artifact-gate` fails | Check `octomap.ot`, `occupancy.npz`, `metadata.json`, and the saved-map path. |
| localization lost | Restart SLAM or relocalize before sending goals. |
| map looks smeared | Inspect `map_optimization.json`, raw map, patch poses, and calibration. |
| teleop does not move | Check mode, teleop lease, safety level, native `rt/nav/cmd_vel`, `nav_endpoint_status.json`, `driver_status.json`, and remote Brainstem readiness. |
