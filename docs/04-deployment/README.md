# LingTu Real-Robot Deployment

Status: current real-environment deployment guide as of 2026-08-23.

Current LingTu-authored Go2 EDU + external MID-360 assisted-teleoperation
deployment manual:
[`go2_edu_mid360_teleop_avoid.md`](go2_edu_mid360_teleop_avoid.md).

How LingTu is laid out on the field robot, how to install, update, roll back,
and diagnose the running stack. The product runtime is native DDS plus
Product-managed native services and scoped Host Modules; ROS 2 compatibility services are optional
fallbacks and must be enabled explicitly.

Public Product lifecycle commands are `switch / status / stop`.

## One-Liner Status

```bash
export LINGTU_TARGET_HOST=ROBOT_IP_OR_HOSTNAME
export LINGTU_TARGET_USER=ROBOT_SSH_USER
ssh "${LINGTU_TARGET_USER}@${LINGTU_TARGET_HOST}" 'systemctl status lt-lidar lt-slam lt-maps lt-terrain lt-nav lt-driver lt-host'
ssh "${LINGTU_TARGET_USER}@${LINGTU_TARGET_HOST}" 'bash /opt/lingtu/current/scripts/lingtu status'
```

The `lingtu` CLI reports the active Product, RunPlan, process state, and
readiness; see `lingtu_cli.md`.

## Robot Runtime

```text
Sensor DDS     lt-lidar              Livox SDK2 -> typed DDS       [systemd]
SLAM DDS       lt-slam               C++ SLAM runtime + snapshots  [systemd]
Maps           lt-maps               Native layers/state/scene     [systemd]
Risk Grid      lt-terrain            Native control-risk grid      [systemd]
Nav Endpoint   lt-nav                Native DDS planner/control    [systemd]
Driver         lt-driver             DDS cmd_vel -> robot adapter  [systemd]

Control        Go2 or Brainstem     vendor motion boundary        [external]
Camera         lt-camera             Native Orbbec -> typed DDS    [systemd]

Host           lt-host               RunPlan -> Blueprint          [systemd]
                                     Gateway HTTP :5050 / MCP :8090
```

Product mode changes run through
`scripts/lingtu -> lingtu.control.ProductControl -> systemd`.
`python -m lingtu.real.host` builds the published RunPlan inside `lt-host`; it is
not a second field-process controller.

`robot-camera.service`, `robot-fastlio2.service`,
`robot-localizer.service`, and `robot-lidar.service` are retired ROS2 unit-name
tombstones. The repository ships neither their templates nor an installer, and
ProductControl never starts them. Doctor, the service catalog, and the real-env
conflict list retain the names only to detect externally installed conflicts.

Runtime plugin registration is a separate opt-in boundary: ROS2 adapter groups
such as `ros2_slam_bridge` and `ros2_map_output` are not part of the product
plugin catalog unless `LINGTU_ENABLE_ROS2_COMPAT=1` is set before startup.

There is no standalone `lingtu-gateway.service`: Gateway runs inside
`lt-host.service` and exposes HTTP `:5050` plus MCP `:8090` from the same
process.

In the current field path, `lt-host.service` runs with
`LINGTU_COMMAND_OUTPUT_MODE=endpoint_only` and
`LINGTU_HARDWARE_CONTROL_BOUNDARY=driver`. The native navigation endpoint
publishes the only product velocity stream (`rt/nav/cmd_vel`), and
`lt-driver.service` is the unique speed exit to the adapter selected by
`RobotConfig.driver.backend` (`go2` or `doso`). The Python application must
not enable a second robot driver in this Product.

`lt-camera.service` owns the current native Orbbec capture path.
`robot-camera.service`, `camera.service`, and `orbbec-camera.service` are
retired unit-name tombstones kept only for conflict detection. If any is
externally installed, disable or remove it before starting
`lt-camera.service`. The `/camera/camera` and
`/camera/camera_container` nodes belong only to that historical graph.

## Service Inventory

| Service | Role | Selected when |
| --- | --- | --- |
| `lt-lidar` | Livox MID-360 SDK2 publisher into typed DDS | Product declares LiDAR/IMU ingress |
| `lt-slam` | C++ SLAM/localization runtime and status snapshots | Product declares SLAM |
| `lt-maps` | Native realtime map ingestion, layers, state, and scene | Product declares maps |
| `lt-terrain` | Native control-risk grid and unique `/nav/traversability` writer | Product requires obstacle/traversability checks |
| `lt-nav` | Native DDS navigation endpoint and final logical command owner | Product declares navigation control |
| `lt-driver` | Native motion driver: typed DDS `rt/nav/cmd_vel` to the selected robot adapter | Real Product declares robot motion |
| `lt-host` | Python Host for Gateway, Agent, MCP, and low-rate adapters | Current real Product declares Host |
| Go2 SDK2 or remote Brainstem | Selected vendor motion boundary | `lt-driver` selects the corresponding adapter |
| `lt-camera` | Native Orbbec capture and typed DDS publisher | Product declares camera |
| `lt-explore` | Native exploration policy process | Product is `explore` |
| `robot-camera` / `robot-fastlio2` / `robot-localizer` / `robot-lidar` | retired ROS2 unit-name tombstones; external conflicts only | never |

## Filesystem Layout

```text
/opt/lingtu/
  current -> releases/vX.Y.Z/
  releases/
  config/
    thunder-runtime-env.sh
    brainstem.env             # Thunder only
  nav/
  logs/

/etc/systemd/system/
  lt-lidar.service
  lt-slam.service
  lt-maps.service
  lt-terrain.service
  lt-nav.service
  lt-camera.service
  lt-explore.service
  lt-driver.service
  lt-host.service

/var/lib/lingtu/
  maps/
```

Developer checkout on the robot usually lives at
`~/data/SLAM/navigation` or `~/data/inovxio/lingtu`.

## Installing On A Robot

```bash
# 1. Build native product modules
cd ~/data/SLAM/navigation
export LINGTU_ROBOT=unitree/go2
export LINGTU_DRIVER_BACKEND=go2
bash scripts/build/build_livox_sdk2_stream.sh
LINGTU_SLAM_BUILD_DDS_RUNTIME=ON bash scripts/build/build_slam_core.sh
bash scripts/build/build_mapd.sh
bash scripts/build/build_dds_probe.sh
bash scripts/build/build_nav_endpoint.sh
LINGTU_DRIVER_BACKEND="${LINGTU_DRIVER_BACKEND}" bash scripts/build/build_driver.sh
bash scripts/build/build_native_recording.sh
bash scripts/build/build_octoplanner3d.sh --require-pcl

# 2. Install native field services for the robot selected by RobotConfig.
LINGTU_DRIVER_BACKEND="${LINGTU_DRIVER_BACKEND}" \
  bash scripts/deploy/thunder/install_services.sh field-cpp

# 3. Start the Product through ProductControl in env=real
bash scripts/lingtu --robot "${LINGTU_ROBOT}" --env real switch nav --map MAP_NAME
```

The Product command validates the map and RunPlan contract, removes stale
mode-owned services, starts the declared processes in dependency order, and
waits for role-specific readiness. Direct multi-service `systemctl start`
commands are diagnostic procedures, not a supported product startup path.

ProductControl publishes the selected backend and its network configuration in
`/run/lingtu/session.env`. Go2 uses `driver.network_interface` and does not use a
target IP. Thunder uses `driver.target` plus mTLS files; its remote Brainstem
must support the lease-and-ack v1 RPCs documented in
`src/drivers/real/motion/README.md`.

For Thunder only, explicitly allow the LingTu host's source IP on the Brainstem computer with
`HAN_DOG_LINGTU_ALLOWED_IPS=LINGTU_HOST_IP`. The default is empty and rejects
every remote motion caller. This allowlist only permits lease-aware motion;
motor enable, posture, zeroing, and fault-clear RPCs remain loopback-only.

The Brainstem hardware service must also configure
`HAN_DOG_GRPC_TLS_CERT_FILE`, `HAN_DOG_GRPC_TLS_KEY_FILE`, and
`HAN_DOG_GRPC_TLS_CLIENT_CA_FILE`. A remote motion peer without a mutually
authenticated TLS session is rejected even when its IP is allowlisted.

## Release Flow

```bash
cd ~/data/SLAM/navigation
bash scripts/deploy/deploy_robot.sh nav
bash scripts/deploy/package_native_release.sh vX.Y.Z dist
```

Deployment and artifact creation remain two explicit steps. `deploy_robot.sh`
owns Product-specific build and activation; `package_native_release.sh` creates
the reviewed release artifact.

## Common Operations

| Need | Command |
| --- | --- |
| One-screen status | `bash scripts/lingtu status` |
| Watch status | `watch -n 2 bash scripts/lingtu status` |
| Switch Product | `bash scripts/lingtu --robot "${LINGTU_ROBOT:?}" --env real switch "${LINGTU_PRODUCT:?}"` |
| Stop Product | `bash scripts/lingtu --robot "${LINGTU_ROBOT:?}" --env real stop` |
| Record native DDS | `/opt/lingtu/current/bin/lingtu_recorder record --output-dir SESSION_DIR --dds on --camera off` |
| Record native DDS and camera | `/opt/lingtu/current/bin/lingtu_recorder record --output-dir SESSION_DIR --dds on --camera on` |
| Verify a recording | `/opt/lingtu/current/bin/lingtu_dds_player SESSION_DIR/dds/sensors.mcap --dry-run` |
| Tail app logs | `journalctl -u lt-host -f` |
| Tail native SLAM logs | `journalctl -u lt-slam -f` |
| Tail native Livox logs | `journalctl -u lt-lidar -f` |
| Tail native driver logs | `journalctl -u lt-driver -f` |
| Verify ports | `ss -tnlp \| grep -E '13145\|5050\|8090'` |

## Diagnostics

### Native Runtime Not Ready

```bash
bash scripts/lingtu status
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/map_cloud"
jq . /dev/shm/lingtu/nav_endpoint_status.json
jq . /dev/shm/lingtu/driver_status.json
journalctl -u lt-slam.service -n 80 --no-pager
```

If native binaries are missing, the service logs print the exact build command
for the missing component.

### LiDAR Or IMU Missing

```bash
journalctl -u lt-lidar.service -n 80 --no-pager
PYTHONPATH=src python -m diagnostics.field.doctor
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/lidar_scan"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/imu"
ip -br addr | grep 192.168.1
```

### Drift Or Lost Localization

```bash
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/map_cloud"
PYTHONPATH=src python -m diagnostics.field.soak --duration 120 --interval 2 --json --strict
bash scripts/lingtu --robot "${LINGTU_ROBOT:?}" --env real switch "${LINGTU_PRODUCT:?}"
```

`python -m diagnostics.field.soak` is the preferred non-motion evidence command after boot or sensor
reconnects. It samples Gateway readiness, localization freshness, map-cloud
stability, command-source idleness, and stationary odometry displacement.

Saved-map relocalization is part of the ProductControl switch. It is not an
in-place DDS command:

| Operation | Command | What It Proves |
| --- | --- | --- |
| Switch Product | `bash scripts/lingtu --robot "${LINGTU_ROBOT:?}" --env real switch "${LINGTU_PRODUCT:?}"` | the Product can start and pass its declared readiness |
| Seeded nav switch + relocalization | `bash scripts/lingtu --env real switch nav --map "${LINGTU_MAP:?}" --initial-pose X Y YAW` | the nav Product cold-starts on the exact map and accepts the operator seed |
| Global nav switch + relocalization | `bash scripts/lingtu --env real switch nav --map "${LINGTU_MAP:?}" --relocalize` | the nav Product cold-starts on the exact map and performs global relocalization |
| Stationary RunPlan gate | `PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --strict` | declared processes and live no-motion readiness agree |
| Integrated saved-map/plan gate | `PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" --map "${LINGTU_MAP:?}" --goal X Y YAW` | map, localization, and requested plan evidence pass without motion |

Do not treat `TRACKING` alone as navigation-ready. For saved-map navigation,
`map_odom_tf` must be valid and the odometry must be inside the active map
frame. A relocalization result with low quality or identity `map->odom` is a
failed localization gate even if the SLAM process is alive.

Gateway's `/api/v1/slam/*relocalize` endpoints are a separate in-place retry
surface for the already running Product and its already loaded map. They never
start `slamd` or switch maps. See
[`LOCALIZATION_RUNTIME.md`](../architecture/LOCALIZATION_RUNTIME.md).

## References

- [`go2_edu_mid360_teleop_avoid.md`](go2_edu_mid360_teleop_avoid.md) -
  LingTu-authored Go2 EDU external MID-360 deployment, calibration, no-motion,
  bounded-motion, and obstacle-avoidance gates
- `lingtu_cli.md` - operations CLI subcommands
- `native_recording.md` - native C++ recording, replay, safety, and recovery boundary
- `OTA_GUIDE.md` - native release packaging, external OTA distribution boundary, installation, and rollback
