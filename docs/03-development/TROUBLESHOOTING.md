# Troubleshooting Guide

Diagnose build, runtime, localization, planning, communication, and OTA failures on the
S100P deployment. The production entry is `python lingtu.py` orchestrated by
`lingtu.service`; lower-level ROS2 nodes (`livox`, `fastlio2`, `localizer`) run as
`robot-lidar.service` / `robot-fastlio2.service` / `robot-localizer.service`.

If your symptom isn't here, capture a 60s window with `lingtu log all` and check
`docs/04-deployment/lingtu_cli.md` for richer status commands.

---

## Contents

- [Build errors](#build-errors)
- [Runtime startup](#runtime-startup)
- [Localization](#localization)
- [Planning](#planning)
- [Communication](#communication)
- [OTA](#ota)

---

## Build errors

### `tf2_ros/buffer_interface.hpp` not found

```
fatal error: tf2_ros/buffer_interface.hpp: No such file or directory
```

`tf2_ros` is missing.

```bash
sudo apt install ros-humble-tf2-ros ros-humble-tf2 ros-humble-tf2-geometry-msgs
```

### `tf2_geometry_msgs/tf2_geometry_msgs.hpp` not found

CMake dependency missing. In the offending `CMakeLists.txt`:

```cmake
find_package(tf2_geometry_msgs REQUIRED)
# add to ament_target_dependencies(... tf2_geometry_msgs)
```

### GTSAM not found / `libgtsam.so.4: cannot open`

GTSAM is vendored under
`src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/`. Either
rebuild it:

```bash
cd src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DGTSAM_BUILD_TESTS=OFF -DGTSAM_WITH_TBB=OFF
make -j$(nproc) && make install
```

or extend `LD_LIBRARY_PATH` (the `lingtu.service` unit already does this via
`source install/setup.bash`):

```bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(pwd)/src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib"
```

### `_nav_core` import fails after build

```bash
bash scripts/build/build_nav_core.sh        # rebuild nanobind extension
ls src/_nav_core*.so                   # symlink should exist
python3 -c "import sys; sys.path.insert(0, 'src'); import _nav_core; print(_nav_core.LocalPlannerCore)"
```

### OR-Tools missing for TARE planner

```bash
bash scripts/build/fetch_ortools.sh    # arch-aware fetch (~130 MB)
bash scripts/build/build_tare.sh
```

The script handles both `x86_64` and `aarch64` (S100P uses the AlmaLinux-8.10 aarch64
release; glibc-compatible with Ubuntu 22.04).

### Generic dependency install

```bash
sudo apt install ros-humble-desktop-full libusb-dev python3-colcon-common-extensions \
    python-is-python3 python3-pip
pip install transforms3d pyyaml numpy scipy scikit-learn nanobind
rosdep install --from-paths src --ignore-src -r -y
```

---

## Runtime startup

### `lingtu.service` fails to start

```bash
sudo systemctl status lingtu
journalctl -u lingtu -n 80 --no-pager
```

Common causes:

| Cause                             | Fix                                                           |
|-----------------------------------|---------------------------------------------------------------|
| `robot-fastlio2` not yet ready    | `lingtu` waits up to 60 s; check `journalctl -u robot-fastlio2` |
| Missing API key for LLM module    | `eval $(grep MOONSHOT_API_KEY ~/.bashrc)` and re-`systemctl restart lingtu` |
| `_nav_core.so` missing            | `bash scripts/build/build_nav_core.sh` then restart                  |
| Port 5050 / 8090 already bound    | `ss -tnlp | grep -E '5050|8090'` 鈥?kill stragglers             |

### TF chain `map -> odom -> body` broken

```bash
ros2 run tf2_tools view_frames                   # writes frames.pdf
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom body
```

| Cause                              | Fix                                                  |
|------------------------------------|------------------------------------------------------|
| `robot-localizer` not running      | `sudo systemctl start robot-localizer`               |
| No active map loaded               | `lingtu nav start <map_name>`                        |
| Fast-LIO2 hasn't published         | `ros2 topic hz /Odometry` 鈥?expect ~100 Hz           |

### Topic frame_ids wrong

```bash
ros2 topic echo /cloud_map      --field header.frame_id --once   # expect: odom
ros2 topic echo /terrain_map    --field header.frame_id --once   # expect: odom
ros2 topic echo /path           --field header.frame_id --once   # expect: body
```

### Topic rates wrong

```bash
ros2 topic hz /Odometry       # expect ~100 Hz
ros2 topic hz /cloud_map      # expect ~10 Hz
ros2 topic hz /terrain_map    # expect ~5 Hz
ros2 topic hz /cmd_vel        # expect ~20 Hz when moving
```

If everything is slow, check CPU with `htop` and inspect the SLAM voxel size in
`config/robot_config.yaml` under `local_planner.laser_voxel_size`.

---

## Localization

### Position jumps / drift

Common causes:
1. Fast-LIO2 degeneracy in long corridors / open fields.
2. Localizer ICP didn't converge (initial pose too far from truth).
3. Wrong PCD loaded.

```bash
ros2 topic echo /localization_quality            # < 0.1 good, >= 0.3 poor
ros2 topic echo /Odometry --field pose.covariance
ros2 service call /relocalize interface/srv/Relocalize \
    "{pcd_path: '/home/sunrise/data/nova/maps/active/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

### IEKF blew up (xy ~ 1e12)

The Fast-LIO2 IEKF can diverge after long static periods. The Gateway watchdog
(`_drift_watchdog_loop` in `src/gateway/gateway_module.py`) auto-recovers every 60 s
when xy or velocity exceeds limits:

1. Stop SLAM services.
2. Clear cached odom.
3. Push `slam_drift` SSE event.
4. Re-`ensure(...)` services for the current session mode.
5. 300 s cooldown.

To reset manually:

```bash
lingtu svc restart slam
```

For non-motion evidence before restarting anything, run:

```bash
lingtu soak --duration 120 --interval 2 --json --strict
```

This reports Gateway readiness, localization freshness, live map-cloud stability,
command-source idleness, and odometry displacement during the sample window. The
drift value is stationary odometry displacement over that window; it is not a
ground-truth map error measurement.

Tunables (env vars on `lingtu.service`):
`LINGTU_DRIFT_WATCHDOG=0` disables. `_INTERVAL` / `_XY_LIMIT` / `_V_LIMIT` / `_COOLDOWN`
override defaults.

### Localizer is locked but odometry vanished

On the S100P navigation profile, Fast-LIO2 publishes `/nav/odometry` and the ICP
localizer publishes `/nav/localization_health`. A field failure can leave
`robot-fastlio2.service` active while `/nav/odometry` has no publisher; in that
case the localizer may keep reporting `LOCKED` briefly because its own health
heartbeat is still alive.

`SlamBridgeModule` treats this contradiction as an odometry-publisher loss. If
the active backend is `localizer`, localizer health is fresh `LOCKED` or
`RECOVERED`, and odometry stays absent/stale for the configured threshold, it
restarts the localization chain in the same order as the field CLI:

1. Stop `robot-localizer.service`.
2. Stop `robot-fastlio2.service`.
3. Start Fast-LIO2 and wait for `/nav/odometry` publishers.
4. Start the localizer and wait for `/nav/localization_health` publishers.

This recovery path does not send navigation goals, teleop commands, or
`cmd_vel`. It only restarts localization services.

Tunables (env vars on `lingtu.service`):

```bash
LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY=1          # set 0 to disable
LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY_S=20       # stale/absent odom duration
LINGTU_LOCALIZER_ODOM_LOSS_RECOVERY_COOLDOWN_S=300
```

Manual equivalent:

```bash
lingtu svc restart localization
```

### Relocalization fails

```bash
ros2 service list | grep relocalize
ros2 node info /localizer
ls -la /home/sunrise/data/nova/maps/active/map.pcd
```

---

## Planning

### Local planner publishes nothing

Checklist:

1. `/terrain_map` flowing 鈥?`ros2 topic hz /terrain_map`.
2. Goal set 鈥?`lingtu status` `[5] Path` should show waypoints.
3. All candidate paths blocked 鈥?open `http://<robot>:5050/api/v1/free_paths` (debug
   endpoint exposes the same data RViz used to render `/free_paths`).

### `cmd_vel` arbitration confusion

`CmdVelMux` (L0, `src/nav/cmd_vel_mux_module.py`) arbitrates by priority:

| Source             | Priority | Timeout |
|--------------------|----------|---------|
| Teleop joystick    | 100      | 0.5 s   |
| VisualServo        | 80       | 0.5 s   |
| Recovery (stuck)   | 60       | 0.5 s   |
| PathFollower       | 40       | 0.5 s   |

Inspect live arbitration:

```bash
curl http://<robot>:5050/api/v1/cmd_vel_mux
```

### Global planner returns empty path

1. Verify map is loaded: `lingtu status` `[1] Session map=`.
2. Goal must be inside the tomogram bounds.
3. Drop `w_traversability` in `src/global_planning/pct_planner/config/params.yaml`.
4. Rebuild tomogram via the Gateway:
   ```bash
   curl -X POST -H 'Content-Type: application/json' \
       -d '{"action":"build_tomogram","name":"<map_name>"}' \
       http://<robot>:5050/api/v1/maps
   ```

### Exploration start returns 503

Check the active backend first:

```bash
curl http://<robot>:5050/api/v1/explore/status
```

`available=false` with `backend=none` usually means LingTu is running a normal
navigation or mapping profile. Start `python lingtu.py explore` for Wavefront
frontiers, or `python lingtu.py tare_explore` after building the TARE binary for
hierarchical exploration. If `backend=tare` is available but progress stalls,
inspect the latest `tare_stats` and `exploration_supervisor` events in Gateway
SSE before changing navigation or SLAM services.

---

## Communication

### REPL / Gateway not reachable on port 5050

```bash
ss -tnlp | grep -E "5050|8090|13145"
sudo systemctl status lingtu
```

S100P firewall: `iptables -L -n | grep 5050` 鈥?the `ROBOT_REMOTE` chain may be blocking.
See `memory/feedback_orbbec_color_fps_trap.md` for the recurring port reset on reboot.

### MCP tools fail from external client

```bash
curl http://<robot>:8090/mcp/tools
claude mcp add --transport http lingtu http://<robot>:8090/mcp
```

If MCP is up but tools error, the framework module behind the tool may be down 鈥?see
`lingtu status` `[1] Session` for module health.

### brainstem (`robot-brainstem`) unresponsive on port 13145

```bash
journalctl -u robot-brainstem -n 30
sudo systemctl restart robot-brainstem
```

The Gateway and ThunderDriver both depend on this gRPC server. Restarting it forces
a new control lease.

### Teleop joystick has no effect

`TeleopModule` runs inside `lingtu.service`. Symptoms and fixes:

1. WS connection dropped 鈥?refresh the joystick UI at `http://<robot>:5050/teleop`.
2. 3 s idle auto-release fired 鈥?re-engage the joystick.
3. Mux outranked 鈥?VisualServo or Recovery is publishing higher priority. Check
   `/api/v1/cmd_vel_mux`.

---

## OTA

OTA is implemented by `ota-agent.service` (`/opt/lingtu/nav/ota/`). It polls a
registered server URL, downloads signed releases, and atomically swaps the
`/opt/lingtu/current` symlink. The legacy in-process gRPC OTA service (`grpc_gateway`,
`data_service.cpp`) was removed.

### `ota-agent` can't reach server

```bash
journalctl -u ota-agent -n 50 --no-pager
curl -I https://github.com
df -h /tmp
```

### SHA256 mismatch

```bash
sha256sum /tmp/ota_staging/<artifact>
# compare with manifest.json from release
```

Release pipeline issue 鈥?the artifact and manifest got out of sync. Re-publish the
release.

### Rollback

```bash
ls -la /opt/lingtu/releases/
sudo ln -sfn /opt/lingtu/releases/<previous_version> /opt/lingtu/current
sudo systemctl restart lingtu
```

The `ota-agent` records each install in `/opt/lingtu/nav/ota/installed_manifest.json`;
there is no in-process rollback RPC anymore.

### Stale transaction log after power loss

```bash
ls /opt/lingtu/nav/ota/backup/txn_*.json   # if present, last install was interrupted
sudo rm /opt/lingtu/nav/ota/backup/txn_*.json
```

The agent self-recovers on next boot but if `lingtu.service` won't start after an
interrupted install, manually roll back the symlink (above).

---

## Quick diagnostics

```bash
# One-screen status from the laptop
lingtu status

# Service summary
sudo systemctl list-units 'robot-*' lingtu.service ota-agent.service

# Critical topic rates (run on robot)
for t in /Odometry /cloud_map /terrain_map /cmd_vel; do
    echo -n "$t  "; timeout 2 ros2 topic hz $t 2>/dev/null | tail -1
done

# Frame ID sanity
for t in /cloud_map /terrain_map /path; do
    echo -n "$t: "
    ros2 topic echo $t --field header.frame_id --once 2>/dev/null
done

# Active SLAM drift watchdog state
curl http://<robot>:5050/api/v1/health | jq '.slam'
```

---

## Related

- `docs/03-development/PARAMETER_TUNING.md` 鈥?tuning defaults and effects
- `docs/04-deployment/README.md` 鈥?service layout, deployment basics
- `docs/04-deployment/lingtu_cli.md` 鈥?operations CLI reference
- `docs/archive/05-specialized/slam_drift_watchdog.md` 鈥?drift watchdog internals
