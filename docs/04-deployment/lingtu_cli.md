# `lingtu` Operations CLI

A single SSH-friendly entry point that replaces ad-hoc `curl`, `systemctl`, and
`journalctl` invocations. The script lives at `scripts/lingtu` and is deployed on the
robot at `/home/sunrise/data/SLAM/navigation/scripts/lingtu`.

This is the operations CLI run **on the robot**. The Python entry `lingtu.py` (with
profiles like `s100p`, `sim`, `dev`, `map`, `explore`, `stub`) is the application
itself, started by `lingtu.service`.

---

## Local alias (one-time setup)

```bash
# ~/.bashrc / ~/.zshrc
alias lingtu='ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
```

After `source ~/.bashrc`:

```bash
lingtu status     # one-screen snapshot
lingwatch         # secondary screen, live refresh
```

---

## Subcommands

### `status` —8-section snapshot

```
=== Lingtu @ 17:26:52 ===
[1] Session   mode=idle   map=corrected_20260406_224020   can_map=True
[2] SLAM      hz=10.0Hz   live_pts=0   loc=GOOD
[3] Robot     xy=(1.2, -0.4)   z=0.05   yaw=12.3 deg   v=0.0 m/s   w=0.0 rad/s
[4] Mission   state=IDLE   wp=0/0   replan=0   speed=1.0   deg=NONE
[5] Path      (no active plan)
[6] Ctrl      teleop=False(0)   safety=OK
[7] Map       active   @02:57:26   map.pcd=2.6M   predufo=-   patches=105
[8] Log       (recent drift / dufomap / error)
```

| Section       | Field                  | Healthy                            | Bad                            |
|---------------|------------------------|------------------------------------|--------------------------------|
| [1] Session   | `mode`                 | `idle` / `mapping` / `navigating`  | -                              |
| [2] SLAM      | `hz`                   | 20-47 Hz (mapping), 10 Hz (nav)    | 0 Hz means service is down     |
| [2] SLAM      | `loc`                  | `GOOD` / `OK`                      | `DEGRADED` / `LOST`            |
| [3] Robot     | `xy`                   | within ~+-50 m indoors             | `ODOM DIVERGED` => IEKF blew   |
| [4] Mission   | `state`                | `IDLE` / `EXECUTING`               | `FAILED`                       |
| [4] Mission   | `deg`                  | `NONE`                             | `CRITICAL`                     |
| [5] Path      | `pts/len/next/goal`    | populated when navigating          | -                              |
| [6] Ctrl      | `safety`               | `OK (0)` / `WARN (1)`              | `STOP (2/3)`                   |
| [7] Map       | `dufo -N pts (X%)`     | dynamic-removal stats present      | -                              |
| [8] Log       | drift / dufomap / err  | quiet                              | recent entries imply trouble   |

### `watch [interval]` —continuous monitor

```bash
lingtu watch       # 1 second
lingtu watch 2     # 2 seconds
```

Internally `watch -c -n <interval> bash <script> status`. Keep it on a side screen
during mapping / navigation runs.

### `map` —mapping session

```bash
lingtu map start              # enter mapping (starts robot-fastlio2 + slam_pgo)
lingtu map save lab_0423      # PGO save + DUFOMap clean + tomogram + occupancy rebuild
lingtu map end                # back to idle (stops SLAM)
lingtu map list               # list saved maps
lingtu map restore lab_0423   # restore predufo backup -> map.pcd (DUFOMap was too aggressive)
```

`save` takes 15-60 s. Toast / JSON return contains DUFOMap stats:

```json
{
  "success": true,
  "name": "lab_0423",
  "dynamic_filter": {
    "success": true,
    "orig_count": 170086,
    "clean_count": 169512,
    "dropped": 574,
    "elapsed_s": 0.8
  }
}
```

`restore` use case: DUFOMap removed static structure by mistake. `map.pcd.predufo` is
restored to `map.pcd`; the discarded `map.pcd` is preserved as
`map.pcd.replaced-<ts>` (double safety). The `lingtu map restore <name>` command
then rebuilds the tomogram and occupancy grid automatically. To run those rebuild
steps manually:

```bash
curl -X POST -H 'Content-Type: application/json' \
    -d '{"action":"build_tomogram","name":"lab_0423"}' \
    http://localhost:5050/api/v1/maps
curl -X POST -H 'Content-Type: application/json' \
    -d '{"action":"build_occupancy","name":"lab_0423"}' \
    http://localhost:5050/api/v1/maps
```

### `nav` —navigation session

```bash
lingtu nav start corrected_20260406_224020    # navigating + load map (starts robot-fastlio2 + robot-localizer)
lingtu nav goal 3.5 2.1 0.0                   # send map-frame goal (x, y, yaw)
lingtu nav stop                               # same as map end
```

The optional `yaw` is in radians. It is forwarded to Gateway `/api/v1/goal` and
published as the map-frame pose orientation; when omitted, the heading defaults
to `0.0`.

### `plan-preview` - offline tomogram planner gate

```bash
lingtu plan-preview --internal-only --strict
lingtu plan-preview --start -9.974 -8.141 0 --goal 2.826 -6.741 0 --strict
```

This command is the safest planner check. It does not start `lingtu.service`,
does not call Gateway, does not send `/api/v1/goal`, and does not publish
`cmd_vel`. It loads `<map-root>/active/tomogram.pickle`, injects synthetic
odometry into `NavigationModule`, calls `preview_plan()`, prints JSON evidence,
and exits.

Use it before any real navigation session when validating that the active
tomogram and planner runtime can produce a feasible global path. The output
reports tomogram bounds, `last_pose.txt` bounds, PCT runtime library status,
planner backend class/load errors, start/goal bounds, path point count,
distance, per-segment spacing, and plan latency.

### Exploration profiles and Gateway contract

`python lingtu.py explore` runs the Wavefront frontier backend, while
`python lingtu.py tare_explore` now runs the ROS-free LingTu-native frontier
stack with traversable frontier candidates and OctoPlanner3D global planning.
Gateway `/api/v1/explore/status` reports `backend=frontier|tare|none`;
`/api/v1/explore/start` and `/api/v1/explore/stop` dispatch to the active
backend. In the normal `nav` profile, `backend=none` and start returns `503`;
this is expected because exploration is not part of the default saved-map
navigation path. External TARE benchmark endpoints still use the TARE bridge
when selected explicitly.

### `svc` —systemd service control

```bash
lingtu svc status               # 6 core services: enabled / active
lingtu doctor                   # read-only service/Gateway/dataflow diagnostics
lingtu doctor --ros2            # optional legacy ROS topic/node diagnostics
lingtu svc restart slam         # restart Fast-LIO2 (i.e. robot-fastlio2.service)
lingtu svc restart localization # restart Fast-LIO2 + localizer, then wait for /ready
lingtu svc restart lidar        # restart Livox driver (i.e. robot-lidar.service)
lingtu svc restart localizer    # restart ICP localizer
lingtu svc restart super_lio    # restart experimental Super-LIO backend
lingtu svc restart super_lio_relocation
lingtu svc restart lingtu       # restart algorithm layer (clears odom cache)
lingtu svc restart all          # restart LiDAR + SLAM + localizer + lingtu
lingtu svc stop slam            # stop a single service
```

The aliases `lidar` / `slam` / `localization` / `localizer` / `lingtu` / `camera` / `brainstem` map
to the corresponding production systemd unit. There is no `localization.service` anymore;
`slam` aliases to `robot-fastlio2.service`. `svc status` warns if legacy
`lidar.service`, `localization.service`, or `localizer.service` is still active because those
services can duplicate ROS nodes and starve `/nav/lidar_scan` / `/nav/imu`.
Use `svc restart localization` when systemd still shows Fast-LIO2 active but
`/nav/odometry` has no publisher; it restarts only Fast-LIO2 and the ICP
localizer, preserving the LiDAR driver and LingTu process.
The LingTu localization watchdog now performs the same ordered restart
automatically for the `localizer` backend when fresh `LOCKED`/`RECOVERED`
localizer health contradicts a sustained `/nav/odometry` publisher loss. Tune it
with `LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY`,
`LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY_S`, and
`LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY_COOLDOWN_S` on `lingtu.service`.

The Super-LIO aliases are experimental. They map to `robot-super-lio.service`
and `robot-super-lio-relocation.service` when those field-evaluation units are
installed. They must not replace the default `nav`/`localizer` path until the
route-level gate in `super_lio_backend.md` passes.

### `slamcheck` - Super-LIO non-motion smoke

```bash
lingtu slamcheck super_lio --duration 60 --rollback previous
lingtu slamcheck super_lio_relocation \
  --map corrected_20260406_224020 \
  --initial-pose 0 0 0 \
  --duration 60 \
  --rollback previous
```

This command switches the SLAM backend through Gateway, samples the localization
contract, and rolls back. It is intended for non-motion field checks. The smoke
requires:

- `backend=super_lio` or `backend=super_lio_relocation`
- `health_source=odom_map_cloud`
- `map_save_source=live_map_cloud_snapshot` for normal Super-LIO
- `map_save_source=active_map` for relocation
- `saved_map_relocalization_supported=false`
- `restart_recovery_supported=true`
- `recovery_method=restart_super_lio*`
- `recovery_signal=NONE` or `RECOVERED`
- `recovery_action=none` or the matching restart action
- no unit restart-count increase during the window

For relocation, `--initial-pose X Y YAW` writes a fixed-decimal
`SUPER_LIO_RELOCATION_INIT_POSE=[x,y,0.0,0.0,0.0,yaw]` into
`/run/lingtu/super_lio_relocation.env`, and `--map NAME` points the active map
symlink at `$HOME/data/nova/maps/NAME`.

The relocation map must be a direct child of `$HOME/data/nova/maps`. `slamcheck`
rejects nested paths, path traversal, absolute paths outside the map root, and
unsafe active-map symlink targets before it writes the runtime env file or
switches services.

Passing `slamcheck` is not promotion evidence by itself. Save the command
output and then run the route-level validation gate in
`docs/04-deployment/super_lio_backend.md` before considering
`super_lio_relocation` for navigation defaults.

### `slamcompare` - localizer vs Super-LIO static gate

```bash
lingtu slamcompare \
  --map corrected_20260406_224020 \
  --duration 60 \
  --warmup 20 \
  --initial-pose current
```

This command is the repeatable version of the pre-motion A/B check. It switches
to `localizer`, samples a stationary baseline, uses the current localizer pose as
the Super-LIO relocation initial pose, runs a `slamcheck` warm-up for
`super_lio_relocation`, samples the candidate, and always rolls back to
`localizer`. It does not send goals or velocity commands.

Default gates are intentionally conservative:

- localizer stationary XY drift <= 0.20 m
- Super-LIO relocation stationary XY drift <= 0.30 m
- localizer-to-candidate first-pose jump <= 0.50 m
- Super-LIO relocation stationary yaw drift <= 0.25 rad
- localizer-to-candidate yaw jump <= 0.35 rad
- candidate stays ready with `health_source=odom_map_cloud`
- candidate reports `map_save_source=active_map`
- recovery signal stays `NONE` or `RECOVERED`
- active command source stays `none`

Tune the thresholds with `--max-localizer-drift`, `--max-candidate-drift`,
`--max-pose-jump`, `--max-yaw-drift`, and `--max-yaw-jump` only when the route
artifact explains the reason. A failed `slamcompare` blocks robot-motion
validation for Super-LIO and keeps the default navigation path on `localizer`.

### `doctor` - localization chain diagnostics

```bash
lingtu doctor
```

This is read-only. It checks:

- production `robot-*` service state
- active legacy service conflicts
- Gateway readiness, health, localization, navigation, state, and camera snapshot
- runtime dataflow for canonical streams such as `/nav/lidar_scan`, `/nav/imu`,
  `/nav/odometry`, and `/nav/map_cloud`
- Gateway `/api/v1/health` SLAM summary

Use `lingtu doctor --ros2` only when you intentionally need legacy ROS
compatibility graph details such as duplicate node names or endpoint counts.

### `soak` - non-motion readiness soak

```bash
lingtu soak --duration 120 --interval 2 --json --strict
```

This is read-only. It samples `/ready`, `/api/v1/health`,
`/api/v1/localization/status`, `/api/v1/navigation/status`, `/api/v1/state`, and
`/api/v1/path`. It does not start sessions, send goals, publish velocity
commands, or restart services.

Use it after boot, sensor reconnects, or localization changes to produce a
repeatable evidence window. The report records localization state/confidence,
odometry and map-cloud freshness, Gateway latency, live map-point stability,
navigation command source, and stationary pose drift inferred from odometry
samples. That drift is windowed no-motion displacement, not absolute drift
against a survey-grade map.

In `--strict` mode, stale localization, non-idle command sources, readiness
failures, or excessive stationary drift return a non-zero exit code.

### `log` —journalctl filters

```bash
lingtu log drift      # drift_watchdog firings
lingtu log dufomap    # DUFOMap save stats (last hour)
lingtu log error      # last 30 min ERROR-level (filters VectorMemory / WebRTC noise)
lingtu log super_lio  # Super-LIO normal backend journals
lingtu log relocation # Super-LIO relocation backend journals
lingtu log tail       # live tail (Ctrl+C to exit)
lingtu log all        # last 10 min, everything
```

### `health` —REST passthrough

```bash
lingtu health         # GET /api/v1/health | python3 -m json.tool
```

Full module roster, per-sensor status, fine-grained Hz/FPS metrics.

---

## Workflows

### A. Mapping with dynamic-object validation

```bash
lingtu watch                                  # secondary screen
# then on the primary screen:
lingtu status                                 # confirm idle + can_map=True
lingtu map start                              # enter mapping
# wait 5-10 s; lingwatch should show mode=mapping, hz>20, robot xy=(0,0)
# drive the robot through the area; let a person walk through for ~15 s
lingtu map save lab_0423_test                 # ~30 s
# lingwatch [7] Map should show "dufo -N pts (X%)"
lingtu log dufomap | tail -5                  # confirm in the log
```

### B. Load a map and navigate

```bash
lingtu map list
lingtu nav start lab_0423_test                # navigating
# lingwatch shows mode=navigating, loc=GOOD
lingtu nav goal 3.5 2.1
# [5] Path populates, [4] Mission state=EXECUTING, wp counts up
# on success: state=SUCCEEDED
```

### C. Diagnostic playbook

```bash
# Case 1: ODOM DIVERGED on the Robot row
#   wait up to 60 s for the watchdog to auto-recover, or:
lingtu svc restart slam

# Case 2: map save failed
lingtu log error | tail -20
lingtu log dufomap

# Case 3: SLAM won't start
lingtu svc status
# if slam=inactive while mode=mapping:
lingtu map end && sleep 2 && lingtu map start
```

---

## Exit codes

| Code | Meaning                                  |
|------|------------------------------------------|
| 0    | success                                  |
| 1    | argument / usage error                   |
| 2    | runtime error (curl / ssh / systemctl)   |

---

## Related

- `docs/04-deployment/super_lio_backend.md` - experimental Super-LIO build,
  smoke, rollback, failure table, and route-validation gate

- `docs/04-deployment/README.md` —deployment overview, service inventory
- `docs/archive/05-specialized/dynamic_obstacle_removal.md` —DUFOMap Phase 1 + 2
- `docs/archive/05-specialized/slam_drift_watchdog.md` —IEKF divergence watchdog
