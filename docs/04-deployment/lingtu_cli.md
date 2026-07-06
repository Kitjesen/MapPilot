# `lingtu` Operations CLI

`scripts/lingtu` is the robot-side operations CLI. It is designed for SSH use
on the S100P/sunrise board and replaces ad-hoc `curl`, `systemctl`, and
`journalctl` commands during field work.

Deployment path on the robot:

```text
/home/sunrise/data/SLAM/navigation/scripts/lingtu
```

The Python application entry is still `lingtu.py`. The shell CLI starts,
stops, inspects, and coordinates the native field services around it.

## Local Alias

```bash
alias lingtu='ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t -p 12346 sunrise@fe91fae6a6756695.natapp.cc "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
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
  -> DDS /nav/cmd_vel
```

Do not source ROS 2 or a colcon overlay for normal field navigation. ROS 2 is
only for explicit compatibility checks or legacy replay gates.

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

`map start` switches the native SLAM DDS service to mapping mode. `map save`
uses the canonical SLAM map-save adapter and writes a map directory, not just
one PCD file.

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

Save result example:

```json
{
  "success": true,
  "name": "lab_0423",
  "map_optimization": {
    "present": true,
    "status": "optimized_loop_closed",
    "refine_applied": true,
    "loop_count": 1
  },
  "dynamic_filter": {
    "success": true,
    "orig_count": 170086,
    "clean_count": 169512,
    "dropped": 574,
    "elapsed_s": 0.8
  }
}
```

`dynamic_filter` is optional cleanup evidence. The product readiness gate is
the saved-map bundle plus `map_optimization.json`, `metadata.json`, and
`octomap.ot` when OctoPlanner3D navigation is required.

`map restore <name>` promotes the preserved raw/backup map back to `map.pcd`
when save-time cleanup removed static structure. The replaced file is kept as
`map.pcd.replaced-<timestamp>` and downstream artifacts are rebuilt.

Manual artifact rebuilds:

```bash
curl -X POST -H 'Content-Type: application/json' \
  -d '{"action":"build_tomogram","name":"lab_0423"}' \
  http://localhost:5050/api/v1/maps

curl -X POST -H 'Content-Type: application/json' \
  -d '{"action":"build_occupancy","name":"lab_0423"}' \
  http://localhost:5050/api/v1/maps
```

## Navigation

```bash
lingtu nav start corrected_20260406_224020 --initial-pose 0 0 0
lingtu nav goal 3.5 2.1 0.0
lingtu nav stop
```

`nav start` switches native SLAM to localization mode, loads
`<map>/map.pcd`, starts the Gateway navigation session, and optionally runs
saved-map relocalization when `--initial-pose` or `--relocalize` is provided.

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
lingtu mode switch teleop_avoid --map lab_0423
lingtu mode switch map
lingtu mode switch tracking --map lab_0423
lingtu mode switch nav --map lab_0423
lingtu mode switch inspection --map lab_0423
lingtu mode switch tare_explore
```

`mode` is the low-level Gateway session (`mapping`, `navigating`, or
`exploring`). `product_session` is the operator-facing mode:
`teleop`, `teleop_avoid`, `mapping`, `tracking`, `navigation`, `inspection`,
or `exploration`.

Profile-level switching may still require restart depending on the endpoint.
Visual-servo target/mode switching is the explicit hot-switch entry inside
profiles that already load `VisualServoModule`.

The Web dashboard exposes the same contract in the `Runtime` tab:

- Product mode cards call `POST /api/v1/runtime/switch`.
- `Preflight` sends `execute=false`; it is read-only and should be run before
  any switch.
- `Execute Switch` sends `execute=true`; cold-restart modes are disabled until
  the operator explicitly enables service restart in the UI.
- Visual Servo `Find`, `Follow`, and `Stop` call `POST /api/v1/visual_servo`.

## Plan Preview

```bash
lingtu plan-preview --internal-only --strict
lingtu plan-preview --start -9.974 -8.141 0 --goal 2.826 -6.741 0 --strict
```

This is the safest planner check. It does not publish a real goal, stop, or
`cmd_vel`. It validates the active map package and OctoPlanner3D artifact gate,
injects synthetic odometry into `NavigationModule`, calls `preview_plan()`,
prints JSON evidence, and exits.

Run this before a real navigation session when validating a saved map.
PCT/tomogram fields are legacy diagnostics unless the selected profile
explicitly uses PCT.

## Services

```bash
lingtu svc status
lingtu svc restart slam
lingtu svc restart lingtu
lingtu svc restart all
```

Useful native services:

- `lingtu-livox-dds`
- `lingtu-slam-dds`
- `lingtu-nav-dds`
- `lingtu`

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
| `No active map` | Run `lingtu map list`, then `lingtu nav start <map>` or `map use <name>`. |
| plan preview fails | Check `octomap.ot`, `metadata.json`, and the active map path. |
| localization lost | Restart SLAM or relocalize before sending goals. |
| map looks smeared | Inspect `map_optimization.json`, raw map, patch poses, and calibration. |
| teleop does not move | Check mode, teleop lease, safety level, and native `/nav/cmd_vel` ownership. |
