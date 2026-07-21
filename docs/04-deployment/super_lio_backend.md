# Super-LIO Backend Status

Super-LIO is currently integrated as an external SLAM/localization backend behind
`SlamBridgeModule`, not as a normal in-process LingTu Module. It is selected with
`slam_profile="super_lio"` and consumed through the same bridge contract used by
the rest of the navigation stack.

The upstream project checked for this integration is the Super-LIO ROS 2 branch
at commit `a9220861e59194e3f10019192c6e5d61427a5b58`:

- Repository/README: <https://github.com/Liansheng-Wang/Super-LIO>
- Normal ROS 2 launch: `ros2 launch super_lio Livox_mid360.py`
- Upstream normal outputs: `/lio/odom` and `/lio/cloud_world`
- LingTu service remaps: `/lio/odom -> /nav/odometry`,
  `/lio/cloud_world -> /nav/map_cloud`

## Build And Install Runbook

Super-LIO is an experimental external ROS 2 workspace. Keep it outside the
normal LingTu Python module graph and expose it only through systemd plus
`SlamBridgeModule`.

Expected S100P paths:

| Path | Purpose |
| --- | --- |
| `/home/sunrise/data/inovxio/super-lio` | Super-LIO ROS 2 workspace |
| `/home/sunrise/data/inovxio/super-lio/src/super_lio` | Upstream package root used by the relocation map path logic |
| `/home/sunrise/data/inovxio/lingtu` | LingTu checkout used by developer-shell services |
| `/home/sunrise/data/SLAM/navigation` | Robot operations symlink used by `scripts/lingtu` |
| `/home/sunrise/data/nova/maps/active` | Active saved-map symlink or directory |
| `/run/lingtu/super_lio_relocation.env` | Runtime map and initial-pose override written by `slamcheck` |
| `/etc/lingtu/super_lio.env` | Optional persistent Super-LIO environment overrides |

Build or refresh Super-LIO on the robot:

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
ssh sunrise@"$LINGTU_HOST"
mkdir -p /home/sunrise/data/inovxio
cd /home/sunrise/data/inovxio/super-lio
git fetch --all --tags
git checkout a9220861e59194e3f10019192c6e5d61427a5b58
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
ros2 pkg prefix super_lio
```

If `/home/sunrise/data/inovxio/super-lio` does not exist, clone the upstream
repository first and keep the checked commit recorded in this document before
field testing:

```bash
cd /home/sunrise/data/inovxio
git clone https://github.com/Liansheng-Wang/Super-LIO.git super-lio
cd super-lio
git checkout a9220861e59194e3f10019192c6e5d61427a5b58
```

Install the experimental service templates from the LingTu checkout:

```bash
cd /home/sunrise/data/inovxio/lingtu
scp scripts/deploy/s100p/super_lio.service \
    scripts/deploy/s100p/super_lio_relocation.service \
    scripts/deploy/s100p/install_services.sh \
    sunrise@"$LINGTU_HOST":/tmp/
ssh sunrise@"$LINGTU_HOST" 'bash /tmp/install_services.sh /tmp'
```

There are two installer lanes:

| Installer | Scope | Super-LIO behavior |
| --- | --- | --- |
| `docs/04-deployment/services/install.sh` | legacy ROS2 service set, guarded by `LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1` | does not install Super-LIO |
| `scripts/deploy/s100p/install_services.sh` | developer and field-evaluation SLAM services | installs `super_lio` and `super_lio_relocation`, plus `robot-super-lio*` aliases |

Do not move Super-LIO into the production installer or `lingtu.target` until the
route-level gate below passes.

Post-install checks:

```bash
systemctl status robot-super-lio.service robot-super-lio-relocation.service --no-pager
systemctl show -p FragmentPath -p EnvironmentFiles -p NRestarts robot-super-lio.service robot-super-lio-relocation.service
bash /home/sunrise/data/SLAM/navigation/scripts/lingtu svc status
bash /home/sunrise/data/SLAM/navigation/scripts/lingtu slamcheck super_lio --duration 10 --rollback previous
```

These checks should prove that the service files were installed from the
developer lane, the environment files are readable, aliases resolve to the
`robot-super-lio*` units, and rollback returns to the previous profile.

## Environment Overrides

Persistent overrides belong in `/etc/lingtu/super_lio.env`. Per-smoke
relocation overrides are written to `/run/lingtu/super_lio_relocation.env` by
`scripts/lingtu slamcheck`.

Override precedence:

1. Systemd unit defaults in `scripts/deploy/s100p/super_lio*.service`.
2. Persistent field overrides in `/etc/lingtu/super_lio.env`.
3. Runtime relocation overrides in `/run/lingtu/super_lio_relocation.env`.

Use `/etc/lingtu/super_lio.env` for robot-specific paths and topic names that
should survive reboot. Use `/run/lingtu/super_lio_relocation.env` only for a
single relocation smoke or route validation run. `slamcheck` overwrites the
runtime relocation file, so stale map or pose values should not survive the next
run.

Keep `ROS_DOMAIN_ID` in the normal ROS 2 service environment shared by LingTu
and the robot drivers. Do not set a different domain only in
`super_lio.env`; a mismatched domain can make Super-LIO look healthy at the
process level while subscribing to no Livox or IMU data.

| Variable | Default | Used by | Purpose |
| --- | --- | --- | --- |
| `SUPER_LIO_WS` | `/home/sunrise/data/inovxio/super-lio` | both services | Super-LIO workspace path |
| `SUPER_LIO_CONFIG` | package `livox_360.yaml` | normal service | normal LIO config |
| `SUPER_LIO_LIDAR_TOPIC` | `/nav/lidar_scan` | both services | LingTu Livox point cloud topic |
| `SUPER_LIO_IMU_TOPIC` | `/nav/imu` | both services | LingTu IMU topic |
| `SUPER_LIO_RELOCATION_CONFIG` | package `relocation.yaml` | relocation service | relocation config override |
| `SUPER_LIO_RELOCATION_MAP_DIR` | `/home/sunrise/data/nova/maps/active` | relocation service | active map directory or symlink |
| `SUPER_LIO_RELOCATION_MAP_NAME` | `map.pcd` | relocation service | PCD filename loaded by upstream |
| `SUPER_LIO_RELOCATION_UPDATE_MAP` | `false` | relocation service | keep evaluation runs read-only against saved maps |
| `SUPER_LIO_RELOCATION_INIT_POSE` | `[0.0,0.0,0.0,0.0,0.0,0.0]` | relocation service | initial pose `[x,y,z,roll,pitch,yaw]` |

Example persistent override file:

```bash
sudo install -d -m 0755 /etc/lingtu
sudo tee /etc/lingtu/super_lio.env >/dev/null <<'EOF'
SUPER_LIO_WS=/home/sunrise/data/inovxio/super-lio
SUPER_LIO_LIDAR_TOPIC=/nav/lidar_scan
SUPER_LIO_IMU_TOPIC=/nav/imu
EOF
sudo systemctl daemon-reload
```

## Current Role

In the current LingTu stack, Super-LIO provides:

- odometry for pose freshness and localization health inference
- live map cloud for map-cloud freshness and Gateway visualization
- localization health source derived from `odom` plus `map_cloud`
- live map cloud snapshot source for map save when normal SLAM save services do
  not produce `map.pcd`

The bridge contract for `super_lio` is:

| Field | Value |
| --- | --- |
| `backend` | `super_lio` |
| `health_source` | `odom_map_cloud` |
| `map_save_source` | `live_map_cloud_snapshot` |
| `relocalization_supported` | `false` (legacy alias for saved-map relocalization) |
| `saved_map_relocalization_supported` | `false` |
| `restart_recovery_supported` | `true` |
| `recovery_method` | `restart_super_lio` |
| `relocalization_state` | `unsupported` unless a stale in-flight state is reported |
| `recovery_signal` | `NONE`, `LOC_DIVERGED`, or `RECOVERED` from bridge health logic |
| `recovery_action` | `restart_super_lio` when recovery is needed; `none` after recovery |

Operationally, this means Super-LIO can support live mapping/evaluation and
Gateway health reporting, but it is not yet a saved-map localization backend.

## Upstream Relocalization vs LingTu Online Relocalize

Upstream Super-LIO does include a relocation mode. It is started with
`ros2 launch super_lio relocation.py`, which runs `relocation_node` and loads a
previously saved PCD map from `lio.map.save_map_dir` plus
`lio.map.map_name`.

That is different from LingTu's current Gateway relocalization contract:

- LingTu `/api/v1/slam/relocalize` and `/api/v1/slam/auto_relocalize` expect an
  online service-style relocalization backend.
- The current `robot-super-lio.service` runs upstream `super_lio_node`, not
  `relocation_node`.
- Therefore `saved_map_relocalization_supported=false` is correct for the
  current LingTu `super_lio` bridge, even though upstream has a separate
  relocation executable.

To promote Super-LIO for saved-map navigation, add a separate LingTu relocation
profile/service that owns map path selection, initial pose injection
(`/initialpose`), and rollback into normal navigation mode.

## Experimental `super_lio_relocation` Profile

LingTu now has a guarded scaffold for that separate relocation path:

- CLI profiles: `python lingtu.py super_lio` and
  `python lingtu.py super_lio_relocation`
- REPL hot-switch commands: `slam super_lio` and
  `slam super_lio_relocation`
- Gateway profile/service name: `super_lio_relocation`
- Systemd unit: `robot-super-lio-relocation.service`
- Deploy source: `scripts/deploy/s100p/super_lio_relocation.service`
- Upstream executable: `ros2 run super_lio relocation_node`
- Map source: `/home/sunrise/data/nova/maps/active/map.pcd` by default
- Output remaps: `/lio/odom -> /nav/odometry`,
  `/lio/cloud_world -> /nav/map_cloud`

Super-LIO's `/lio/odom` is already the relocation node's optimized
world/map-frame pose after ICP. LingTu's `SlamBridgeModule` must therefore
consume Super-LIO odometry and cloud output directly. The bridge applies the
cached `map->odom` TF only for the `localizer` backend; applying a stale
localizer TF to Super-LIO output double-transforms the pose and creates a
stationary jump even when the upstream ICP result is correct.

The upstream ROS wrapper prepends Super-LIO's compile-time `ROOT` directory to
`lio.map.save_map_dir`. The S100P systemd unit therefore converts an absolute
LingTu active-map path into a stable source-tree symlink before starting
`relocation_node`:

```text
/home/sunrise/data/inovxio/super-lio/src/super_lio/map/lingtu_active
  -> /home/sunrise/data/nova/maps/active
```

The value passed to Super-LIO is the relative directory
`map/lingtu_active`, so the upstream `ROOT + save_map_dir` behavior resolves to
the intended `active/map.pcd` file. The unit also checks that the final
`map.pcd` is non-empty and logs the resolved path plus byte size at startup.

This is a candidate backend for saved-map navigation startup, not the current
production relocalizer. Its LingTu contract is intentionally conservative:

| Field | Value |
| --- | --- |
| `backend` | `super_lio_relocation` |
| `health_source` | `odom_map_cloud` |
| `map_save_supported` | `false` |
| `map_save_source` | `active_map` |
| `relocalization_supported` | `false` |
| `saved_map_relocalization_supported` | `false` |
| `restart_recovery_supported` | `true` |
| `recovery_method` | `restart_super_lio_relocation` |

Use it only for field evaluation until we verify upstream relocation quality,
initial-pose behavior, drift, restart recovery, and route completion against the
current `localizer` path.

## Difference From `localizer`

`localizer` remains the saved-map relocalization path.

| Capability | `super_lio` | `localizer` |
| --- | --- | --- |
| Primary source | live LIO odometry and map cloud | ICP localizer against active saved map |
| Health source | `odom_map_cloud` | `localizer_health_topic` when fresh |
| Map save source | `live_map_cloud_snapshot` | existing active/saved map path |
| Saved-map relocalization | not supported | supported |
| Restart recovery | supported | supported as fallback |
| Primary recovery method | restart Super-LIO service | call relocalize service |

Do not treat Super-LIO as a drop-in replacement for `localizer` in saved-map
navigation. A `nav` session that must recover against an existing `map.pcd`
still needs the `localizer` backend.

## Gateway/API Fields

The current status fields are exposed through Gateway localization and health
responses. The most direct endpoint is:

```bash
export LINGTU_GATEWAY=http://ROBOT_IP_OR_HOSTNAME:5050
curl -s "$LINGTU_GATEWAY/api/v1/localization/status" | python3 -m json.tool
```

Check these fields in the response:

| Field | Expected Super-LIO value |
| --- | --- |
| `backend` | `super_lio` |
| `health_source` | `odom_map_cloud` |
| `map_save_source` | `live_map_cloud_snapshot` |
| `relocalization_supported` | `false` |
| `saved_map_relocalization_supported` | `false` |
| `restart_recovery_supported` | `true` |
| `recovery_method` | `restart_super_lio` |
| `relocalization_state` | usually `unsupported` |
| `recovery_signal` | `NONE`, `LOC_DIVERGED`, or `RECOVERED` |
| `recovery_action` | `restart_super_lio` during recovery; otherwise `none` |

Related endpoints:

```bash
curl -s "$LINGTU_GATEWAY/api/v1/health" | python3 -m json.tool
curl -s "$LINGTU_GATEWAY/api/v1/slam/status" | python3 -m json.tool
```

Use `/api/v1/localization/status` for the field contract above. Use
`/api/v1/health` for whole-system health and `/api/v1/slam/status` for active
SLAM service mode.

## Field Verification

Run this smoke check on the robot from the LingTu checkout:

```bash
cd ~/data/SLAM/navigation
bash scripts/lingtu slamcheck super_lio --duration 30 --rollback previous
```

The command switches to Super-LIO, verifies the Gateway/session/localization
contract, and switches back to the previous SLAM profile. It checks:

- `backend=super_lio`
- `health_source=odom_map_cloud`
- `map_save_source=live_map_cloud_snapshot`
- `saved_map_relocalization_supported=false`
- `restart_recovery_supported=true`
- `recovery_method=restart_super_lio`
- `robot-super-lio.service` active through `/api/v1/slam/status`

For an explicit map snapshot check, add `--save-map`:

```bash
bash scripts/lingtu slamcheck super_lio --duration 30 \
  --save-map super_lio_field_check --rollback previous
```

To try the guarded relocation backend against a saved map:

```bash
bash scripts/lingtu slamcheck super_lio_relocation \
  --map corrected_20260406_224020 \
  --initial-pose 0 0 0 \
  --duration 45 \
  --rollback previous
```

This writes `/run/lingtu/super_lio_relocation.env`, switches to
`robot-super-lio-relocation.service`, checks the Gateway contract, then rolls
back. It does not save a new map and should still report
`saved_map_relocalization_supported=false`.

Do not treat Gateway readiness alone as relocation success. After a relocation
smoke, inspect the service journal and require evidence that the upstream node
loaded the saved map and used a non-empty ICP target:

```bash
journalctl -q -u robot-super-lio-relocation.service --since '5 min ago' \
  --no-pager \
  | grep -E 'Super-LIO relocation map|Load map|Map size|target size|Global ICP'
```

Expected signs:

- `Super-LIO relocation map: .../map/lingtu_active/map.pcd bytes=<nonzero>`
- `Load map success`
- `Map size: <nonzero>`
- `INIT start... target size: <same nonzero map size>`
- Ideally `Global ICP Converged Succeed`

On the 2026-05-04 S100P smoke with map `corrected_20260406_224020`, the unit
resolved `map/lingtu_active/map.pcd`, loaded `Map size: 170086`, and reached
`Global ICP Converged Succeed`. That proves the wrapper can load the saved map;
route-level drift/recovery validation is still required before promotion.

## 2026-05-04 Non-Motion Gate Evidence

Historical robot used for this evaluation: a private lab `sunrise@<robot-ip>`
target.

Active map:

```text
/home/sunrise/data/nova/maps/active
  -> /home/sunrise/data/nova/maps/corrected_20260406_224020
```

The non-motion gate was repeated twice for each experimental backend, with
rollback to `localizer` after every run.

| Backend | Command shape | Result | Key contract evidence |
| --- | --- | --- | --- |
| `super_lio` | `slamcheck super_lio --duration 60 --rollback previous` | PASS, 16 samples | `health_source=odom_map_cloud`, `map_save_source=live_map_cloud_snapshot`, `restart_recovery_supported=true`, `recovery_method=restart_super_lio`, rollback to `localizer` |
| `super_lio` | repeat 60 s smoke | PASS, 15 samples | same contract, cloud rate recovered to about 9-10 Hz after transition |
| `super_lio_relocation` | `slamcheck super_lio_relocation --map corrected_20260406_224020 --initial-pose 0 0 0 --duration 60 --rollback previous` | PASS, 16 samples | `health_source=odom_map_cloud`, `map_save_source=active_map`, `restart_recovery_supported=true`, `recovery_method=restart_super_lio_relocation`, rollback to `localizer` |
| `super_lio_relocation` | repeat 60 s smoke | PASS, 16 samples | same contract, `recovery_signal` moved from `NONE` to `RECOVERED`, cloud rate recovered to about 9-10 Hz after ICP |

Relocation journal evidence during the repeated smoke:

- `Super-LIO relocation map: .../map/lingtu_active/map.pcd`
- `Load map success`
- `Map size: 170086`
- `Global ICP Converged Succeed`
- Example `FitnessScore` values: `0.117472`, `0.076873`
- Average runtime after startup: observe about `3.3-3.5 ms`, undistort about
  `2.0-2.4 ms`, map update about `0.1 ms`

Post-rollback state:

- `scripts/lingtu status`: `backend=localizer`, localization `TRACKING`,
  safety OK, navigation IDLE
- `robot-fastlio2.service`: active, `NRestarts=0`, memory about 118 MB
- `robot-localizer.service`: active, `NRestarts=0`, memory about 37 MB
- `robot-super-lio.service`: inactive, `NRestarts=0`
- `robot-super-lio-relocation.service`: inactive, `NRestarts=0`
- `/run/lingtu/super_lio_relocation.env` used the active map symlink and wrote
  `SUPER_LIO_RELOCATION_INIT_POSE=[0.000000,0.000000,0.0,0.0,0.0,0.000000]`

Additional stationary A/B evidence on 2026-05-04 used the repeatable
`slamcompare` gate and the same saved map with no robot-motion command:

- `localizer` baseline over about 60 s: XY drift `0.008 m`, yaw drift
  `0.00078 rad`, confidence average `0.962`, map-cloud age about `234 ms`
- A pre-bridge-fix `slamcompare --map corrected_20260406_224020 --duration 60
  --warmup 20` run showed localizer baseline XY drift `0.0209 m`, yaw drift
  `0.00443 rad`, confidence average `0.974`; Super-LIO relocation candidate
  internal XY drift `0.0011 m`, yaw drift `0.00009 rad`, confidence average
  `0.986`, but the candidate first pose jumped by `1.7074 m` and
  `1.03527 rad`
- Root cause: the bridge was applying cached localizer `map->odom` TF to
  Super-LIO relocation odom that was already in the optimized world/map frame
- After redeploying the bridge TF fix and the `slamcompare` ready/pose wait,
  the same command passed: localizer baseline XY drift `0.0442 m`, yaw drift
  `0.00065 rad`, confidence average `0.9669`; Super-LIO relocation candidate
  XY drift `0.0051 m`, yaw drift `0.00008 rad`, pose jump `0.0168 m`, yaw jump
  `0.00363 rad`, confidence average `0.9585`
- Post-run rollback returned to `localizer`; `/api/v1/localization/status`
  reported `ready=true`, `reported_state=TRACKING`, `health_source=localizer_health_topic`,
  `recovery_signal=RECOVERED`, and all relevant SLAM/LingTu units had
  `NRestarts=0`
- Route plan preflight was added and deployed to both robot script entrypoints
  (`/opt/lingtu/current/scripts/lingtu` and
  `~/data/SLAM/navigation/scripts/lingtu`). A first non-motion routecheck with
  goal `(0, 0, 0)` correctly blocked promotion at plan preview:
  `feasible=false`, `planner=pct`, `reasons=["planning_failed"]`; artifact:
  `~/data/SLAM/navigation/artifacts/super_lio_route_preflight_20260504_223205`.
- A second non-motion routecheck used the same saved map with a nearby feasible
  short goal `(-1.071, -1.619, 1.050)` and passed for both backends; artifact:
  `~/data/SLAM/navigation/artifacts/super_lio_route_preflight_20260504_223333`.
  Baseline `localizer` plan: `count=2`, distance `0.548 m`,
  `plan_ms=5.49`; candidate `super_lio_relocation` plan: `count=2`, distance
  `0.548 m`, `plan_ms=4.98`.
- During the successful routecheck candidate phase,
  `/api/v1/localization/status` reported `backend=super_lio_relocation`,
  `ready=true`, `health_source=odom_map_cloud`, `map_save_source=active_map`,
  `confidence=0.96`, `recovery_signal=RECOVERED`, `recovery_action=none`.
  Rollback returned to `localizer`, mission stayed `IDLE`, active command source
  stayed `none`, safety stayed OK, and all relevant SLAM/LingTu units had
  `NRestarts=0`.
- `routecompare` was deployed with a machine-readable
  `routecompare_summary.json` writer and re-checked on sunrise without
  `--allow-motion`; it exited before backend switch, session start, goal post,
  or artifact creation. The robot remained `localizer`, mission `IDLE`, no
  active plan, and safety `OK`.

This satisfies the stationary non-motion and static pre-motion comparison gates
for this saved map, plus the non-motion route planning preflight. The route-level
saved-map motion comparison has not been executed in this evidence set, so
Super-LIO remains experimental and must not become the default `nav` backend.

## Rollback

Use rollback on every smoke until Super-LIO is promoted:

```bash
bash scripts/lingtu slamcheck super_lio --duration 60 --rollback previous
bash scripts/lingtu slamcheck super_lio_relocation \
  --map corrected_20260406_224020 \
  --initial-pose 0 0 0 \
  --duration 60 \
  --rollback previous
```

Rollback decision tree:

| Failure stage | Immediate action | Evidence to capture |
| --- | --- | --- |
| switch rejected before service start | keep the previous profile; do not retry with motion | Gateway switch response, `scripts/lingtu status`, `systemctl show -p NRestarts robot-super-lio*` |
| relocation env or active-map validation fails | fix the map name or active symlink; rerun `slamcheck` only | `/run/lingtu/super_lio_relocation.env`, `readlink -f ~/data/nova/maps/active`, relocation unit journal |
| smoke contract fails | rollback to `previous` or `localizer`, then run `lingtu doctor --non-motion --strict` | failed `slamcheck` JSON/text, `/api/v1/localization/status`, `/api/v1/slam/status`, unit restart counts |
| route candidate fails before motion | stop the mission, rollback to `localizer`, keep the map unchanged | `/api/v1/navigation/status`, `/api/v1/localization/status`, route artifact directory |
| route candidate fails during motion | teleop/stop through the normal safety channel, rollback after the robot is still | safety state, mission state, driver command source, active service journals |

Manual fallback commands:

```bash
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"profile":"localizer"}' \
  "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/slam/switch" | python3 -m json.tool

sudo systemctl stop robot-super-lio robot-super-lio-relocation \
  super_lio super_lio_relocation
sudo systemctl start robot-lidar robot-fastlio2 robot-localizer
```

Post-rollback evidence must show:

- `/api/v1/localization/status`: `backend=localizer`, `ready=true`
- `/api/v1/navigation/status`: `state=IDLE`, no blockers before any motion test
- `scripts/lingtu status`: `loc=TRACKING` or equivalent ready state
- `systemctl show -p NRestarts robot-fastlio2.service robot-localizer.service
  robot-super-lio.service robot-super-lio-relocation.service`: no unexpected
  restart-count increase from the failed candidate run

## Failure Modes

| Symptom | Likely cause | Check | Action |
| --- | --- | --- | --- |
| `Super-LIO install setup not found` | workspace path mismatch | `ros2 pkg prefix super_lio` after sourcing the workspace | set `SUPER_LIO_WS` or rebuild |
| service starts but no Livox/IMU data | topic name mismatch or driver bridge inactive | `ros2 topic hz /nav/lidar_scan`, `ros2 topic hz /nav/imu`, service journal subscriptions | fix topic overrides or restart `robot-lidar` before re-running smoke |
| service active but topics stay silent | `ROS_DOMAIN_ID` mismatch | compare `systemctl show -p Environment robot-lidar robot-super-lio robot-lingtu` and shell `env` | align ROS domain in the shared service environment, then restart the affected services |
| launch exits with config error | config path not present in the installed workspace | `journalctl -u robot-super-lio*` for missing YAML/config path | rebuild Super-LIO or set `SUPER_LIO_CONFIG` / `SUPER_LIO_RELOCATION_CONFIG` |
| relocation exits with missing `map.pcd` | active map symlink or map name wrong | service journal line `Super-LIO relocation map not found` | run `lingtu map list`, pass `--map NAME`, verify `<map>/map.pcd` |
| `slamcheck` rejects map before switch | unsafe explicit map name or active symlink target | stderr line `unsafe relocation map name or active map target` | use a direct child under `$HOME/data/nova/maps`; do not use nested, absolute outside, or path-traversal names |
| relocation starts with empty ICP target | saved map file is empty or invalid | `stat <map>/map.pcd`, relocation journal `Map size` and `target size` | rebuild or restore the saved map before testing relocation |
| ROS 2 rejects `init_pose` | integer sequence mixed with doubles | `/run/lingtu/super_lio_relocation.env` | use `scripts/lingtu slamcheck --initial-pose`, which writes fixed decimal floats |
| `slamcheck` rejects initial pose | non-numeric or incomplete `--initial-pose X Y YAW` | command stderr | rerun with numeric X, Y, and YAW values |
| Gateway ready but no real relocation | node published odom/map without loading saved map | relocation journal map and ICP lines | require `Load map success`, nonzero `Map size`, and ICP target evidence |
| ICP never converges | bad initial pose, stale map, or insufficient overlap | relocation journal `Global ICP`, drift in `/api/v1/localization/status` | retry from a surveyed initial pose; fail the candidate if convergence is not repeatable |
| `recovery_signal=LOC_DIVERGED` during smoke | bridge still sees diverged odom or stale cloud | `scripts/lingtu status`, `/api/v1/localization/status` | fail the smoke, rollback, inspect drift logs |
| restart count increased during smoke | service crashed and recovered | `systemctl show -p NRestarts robot-super-lio*` | fail the smoke and inspect journal |
| CPU or memory starves Gateway/nav | Super-LIO resource envelope too high for S100P | `ps -o pid,comm,%cpu,%mem,rss,vsz,args -C super_lio_node -C relocation_node -C python3`, Gateway latency | keep experimental, capture resource trace, reduce load before route tests |
| duplicate Super-LIO or localizer nodes publish to `/nav/odometry` | old ROS nodes or legacy services still running | `ros2 node list`, `ros2 topic info /nav/odometry -v`, `scripts/lingtu svc status` | stop duplicate units and rerun smoke from a clean service state |
| stale odom or map cloud while service is active | upstream node stuck or subscriptions broken | `/api/v1/localization/status` odom/map-cloud age and rates | rollback, restart the service, and require a clean non-motion smoke before route validation |
| `slamcompare` shows a large pose/yaw jump but raw `/nav/odometry` matches the Super-LIO ICP final transform | bridge is applying a stale localizer `map->odom` TF to Super-LIO world/map odom, or the conditional TF fix regressed | compare `ros2 topic echo --once --field pose.pose /nav/odometry` with `/api/v1/state` while `super_lio_relocation` is active | verify `SlamBridgeModule` only applies `map->odom` for `localizer`, redeploy LingTu, rerun `slamcompare` |

## Route-Level Validation Gate

Run this only after the non-motion smoke is stable, `scripts/lingtu status`
does not show safety `STOP`, and the static A/B gate passes. Keep teleop ready
and start with a short known route.

Required pre-motion gate:

```bash
bash scripts/lingtu slamcompare \
  --map corrected_20260406_224020 \
  --duration 60 \
  --warmup 20 \
  --initial-pose current
```

If this fails on stationary drift, localizer-to-candidate pose jump, readiness,
recovery, or command-source checks, do not send a navigation goal. Capture the
output, keep `super_lio_relocation` experimental, and investigate frame/initial
pose alignment before route testing.

Then run the route plan preflight. This still sends no motion command: it
captures evidence, switches `localizer` and `super_lio_relocation` in sequence,
previews `/api/v1/navigation/plan` for each backend, and rolls back to
`localizer`.

```bash
bash scripts/lingtu routecheck \
  --map corrected_20260406_224020 \
  --goal <x> <y> <yaw> \
  --candidate-warmup 20
```

`routecheck` must pass before the first real route goal. It writes a timestamped
artifact directory under `~/data/SLAM/navigation/artifacts/` by default:

| Path | Purpose |
| --- | --- |
| `before/` | starting status, localization, navigation, health, services, resources, and journals |
| `baseline/` | `localizer` switch response, readiness snapshots, and baseline `plan.json` |
| `candidate/` | initial pose, Super-LIO relocation smoke output, readiness snapshots, and candidate `plan.json` |
| `after_rollback/` | final `localizer` rollback snapshots after a successful preflight |
| `failed_rollback/` | rollback snapshots when readiness or plan preview blocks the preflight |
| `slamcompare.txt` | optional static A/B output when `--with-slamcompare` is used |

If `routecheck` reports blockers such as `odometry_missing`, `safety_stop`, or
an infeasible plan, keep the robot still, keep Super-LIO experimental, and fix
the underlying readiness issue before any route motion.

Only after `slamcompare` and `routecheck` both pass should you run a motion
comparison. The first motion-level harness is `routecompare`. It is locked
behind an explicit motion gate: without `--allow-motion`, it must fail before
starting a navigation session or posting `/api/v1/goal`.

Safe dry-run shape:

```bash
bash scripts/lingtu routecompare \
  --map corrected_20260406_224020 \
  --goal <x> <y> <yaw>
```

This dry run must not move the robot and must not call
`/api/v1/session/start`, `/api/v1/goal`, or `/api/v1/session/end`.

Motion-enabled A/B shape:

```bash
bash scripts/lingtu routecompare \
  --map corrected_20260406_224020 \
  --goal <x> <y> <yaw> \
  --candidate-warmup 20 \
  --timeout 120 \
  --allow-motion
```

With `--allow-motion`, the harness records the same route twice:

1. Baseline: `localizer`
2. Candidate: `super_lio_relocation`

Each run starts a navigation session, sends exactly one route goal, waits for a
mission terminal state, captures route evidence, and stops the session before
switching backend or returning. The harness must rollback to `localizer` at the
end of both success and failure paths.

`routecompare` writes one artifact directory per route pair under
`~/data/SLAM/navigation/artifacts/` by default:

| Path | Purpose |
| --- | --- |
| `before/` | starting localization, navigation, health, service, resource, and journal snapshots |
| `baseline/` | localizer switch, plan preview, session, goal, terminal result, and after-stop snapshots |
| `candidate/` | Super-LIO relocation initial pose, warmup output, plan preview, session, goal, terminal result, and after-stop snapshots |
| `after_rollback/` | final localizer state after a successful A/B pair |
| `failed_rollback/` | stop/session-end/rollback snapshots after any blocked or failed motion run |
| `routecompare_summary.json` | machine-readable baseline/candidate route result, drift, recovery, restart, resource, and rollback summary |

If baseline or candidate motion fails, the harness must stop the active
navigation session, rollback to `localizer`, capture `failed_rollback/`, and
exit non-zero. A failed candidate must never promote Super-LIO, leave an active
session running, or continue sending additional goals.

Manual fallback shape, if the harness itself needs diagnosis, is still to create
one artifact directory per route pair:

```bash
cd ~/data/SLAM/navigation
ART_ROOT=~/data/SLAM/navigation/artifacts/super_lio_route_$(date +%Y%m%d_%H%M%S)
mkdir -p "$ART_ROOT"/{baseline,candidate,journals}

bash scripts/lingtu status | tee "$ART_ROOT/status.before.txt"
curl -s "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/localization/status" \
  | tee "$ART_ROOT/localization.before.json" | python3 -m json.tool >/dev/null
curl -s "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/navigation/status" \
  | tee "$ART_ROOT/navigation.before.json" | python3 -m json.tool >/dev/null
```

Baseline run shape:

```bash
MAP=corrected_20260406_224020
GOAL_X=0
GOAL_Y=0
GOAL_YAW=0

curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"profile":"localizer"}' \
  "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/slam/switch" | tee "$ART_ROOT/baseline/switch.json"
bash scripts/lingtu nav start "$MAP" | tee "$ART_ROOT/baseline/nav_start.txt"
bash scripts/lingtu nav goal "$GOAL_X" "$GOAL_Y" "$GOAL_YAW" \
  | tee "$ART_ROOT/baseline/nav_goal.txt"
```

Candidate run shape:

```bash
bash scripts/lingtu slamcheck super_lio_relocation \
  --map "$MAP" \
  --initial-pose 0 0 0 \
  --duration 60 \
  --rollback none | tee "$ART_ROOT/candidate/slamcheck.txt"
bash scripts/lingtu nav start "$MAP" | tee "$ART_ROOT/candidate/nav_start.txt"
bash scripts/lingtu nav goal "$GOAL_X" "$GOAL_Y" "$GOAL_YAW" \
  | tee "$ART_ROOT/candidate/nav_goal.txt"
```

For each run, capture:

- map name and initial pose
- route waypoints and completion state
- start/end odometry pose and route-closure drift
- localization state, confidence, `recovery_signal`, `recovery_action`
- restart count for active SLAM services
- CPU and memory for Super-LIO/localizer and `lingtu`
- Gateway `/api/v1/navigation/status` blockers and mission result

Capture the same API and service snapshots after each run:

```bash
PHASE=baseline  # or candidate
curl -s "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/localization/status" \
  | tee "$ART_ROOT/$PHASE/localization.after.json" | python3 -m json.tool >/dev/null
curl -s "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/navigation/status" \
  | tee "$ART_ROOT/$PHASE/navigation.after.json" | python3 -m json.tool >/dev/null
curl -s "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/health" \
  | tee "$ART_ROOT/$PHASE/health.after.json" | python3 -m json.tool >/dev/null
systemctl show -p ActiveState -p SubState -p NRestarts \
  robot-fastlio2.service robot-localizer.service \
  robot-super-lio.service robot-super-lio-relocation.service \
  | tee "$ART_ROOT/$PHASE/services.after.txt"
ps -o pid,comm,%cpu,%mem,rss,vsz,args -C super_lio_node -C relocation_node -C python3 \
  | tee "$ART_ROOT/$PHASE/resources.after.txt"
journalctl -u robot-super-lio.service -u robot-super-lio-relocation.service \
  -u robot-localizer.service -u robot-fastlio2.service -u lingtu.service \
  --since '30 min ago' --no-pager > "$ART_ROOT/journals/$PHASE.txt"
```

Abort the candidate run and rollback before continuing if any of these occur:

- safety state enters `STOP` unexpectedly
- `/api/v1/navigation/status` reports `can_accept_goal=false` for a non-transient
  reason before sending the route goal
- `recovery_signal=LOC_DIVERGED` remains unresolved or maps to the wrong
  `recovery_action`
- odometry or map-cloud age exceeds the configured readiness window
- `NRestarts` increases for the candidate service during the route
- relocation journal lacks non-empty map-load and ICP-target evidence
- CPU or memory use starves Gateway, navigation, or safety update rates

Route result template:

| Run | Backend | Map | Initial pose | Route result | Max drift | Recovery signal/action | Restarts | CPU/mem note | Decision |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| baseline | `localizer` |  |  |  |  |  |  |  |  |
| candidate | `super_lio_relocation` |  |  |  |  |  |  |  |  |

Promotion threshold:

| Gate | Required result |
| --- | --- |
| startup | two consecutive 60 s non-motion smokes pass for normal and relocation modes |
| route completion | candidate completes the same saved-map route as `localizer` |
| drift | no worse than the current localizer tolerance for the route |
| recovery | no unresolved `LOC_DIVERGED`; restart recovery returns to ready if induced |
| resources | no sustained CPU or memory envelope regression that starves Gateway/nav |
| rollback | every failed candidate run returns to `localizer` without reboot |

Until every gate passes, keep `super_lio` and `super_lio_relocation` as
experimental profiles and do not change the default `nav` profile.

For routine status inspection:

```bash
cd ~/data/SLAM/navigation
bash scripts/lingtu status
bash scripts/lingtu svc status-legacy
```

Expected service-level signs:

- `super_lio` appears in `scripts/lingtu svc status-legacy`.
- Super-LIO mode should not require `robot-localizer` to be the active
  localization source.
- `scripts/lingtu status` should show live SLAM/localization data instead of
  zero-Hz odometry or stale map cloud.

For API-level verification, check the compact field set:

```bash
curl -s "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/localization/status" \
  | python3 -c 'import json,sys; d=json.load(sys.stdin); keys=("backend","health_source","map_save_source","relocalization_supported","saved_map_relocalization_supported","restart_recovery_supported","recovery_method","relocalization_state","recovery_signal","recovery_action"); print({k:d.get(k) for k in keys})'
```

For map-save behavior in Super-LIO mode:

```bash
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"action":"save","name":"super_lio_field_check"}' \
  "${LINGTU_GATEWAY:-http://127.0.0.1:5050}/api/v1/maps" | python3 -m json.tool
```

The response should report `slam_profile` or source metadata indicating
Super-LIO mode, `map_save_source` or `source` as `live_map_cloud_snapshot`, and
`saved_map_relocalization_supported` as `false`.

## Known Limits

- Super-LIO does not currently support LingTu online saved-map relocalization.
- Saved maps produced from the live map cloud snapshot are useful for inspection
  and evaluation, but they do not make Super-LIO a relocalizing backend.
- Recovery is service restart based. It does not perform global relocalization.
- Health is inferred from odometry/map-cloud freshness and bridge diagnostics,
  not from a Super-LIO-specific localization quality service.
- Upstream relocation mode is separate and still needs a LingTu-owned
  profile/service wrapper validation before use in saved-map navigation.

## Next Steps

- Validate `super_lio_relocation` on S100P against a known saved route before
  promoting it beyond experimental use.
- Add a real online global relocalization path before treating Super-LIO as the
  replacement for `localizer`.
- Decide whether Super-LIO should emit a backend-specific health/quality topic
  instead of relying only on `odom_map_cloud` inference.
- Compare static drift, route closure drift, CPU, memory, and restart recovery
  against the current Fast-LIO2/localizer stack.
- Promote the backend only after map save, recovery, and long-run field tests
  match the localizer-backed navigation requirements.
