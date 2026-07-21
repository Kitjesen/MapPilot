# LingTu Thunder Deployment

Status: current Thunder deployment guide as of 2026-07-18.

How LingTu is laid out on the field robot, how to install, update, roll back,
and diagnose the running stack. The product runtime is native DDS plus
Module-first Python/C++ services; ROS 2 compatibility services are optional
fallbacks and must be enabled explicitly.

## One-Liner Status

```bash
export LINGTU_ROBOT_HOST=ROBOT_IP_OR_HOSTNAME
ssh sunrise@"$LINGTU_ROBOT_HOST" 'systemctl status lingtu-livox-dds lingtu-slam-dds lingtu-nav-dds lingtu-driver lingtu'
ssh sunrise@"$LINGTU_ROBOT_HOST" 'bash /opt/lingtu/current/scripts/lingtu status'
```

The `lingtu` CLI gives an 8-section snapshot; see `lingtu_cli.md`.

## Robot Runtime

```text
Sensor DDS     lingtu-livox-dds     Livox SDK2 -> typed DDS       [systemd]
SLAM DDS       lingtu-slam-dds      C++ SLAM runtime + snapshots  [systemd]
Nav Endpoint   lingtu-nav-dds       Native DDS planner/control    [systemd]
Driver         lingtu-driver        DDS cmd_vel -> Brainstem gRPC [systemd]

Control        remote Brainstem     Dart gRPC :13145              [external]
Camera         robot-camera         Orbbec camera service         [systemd]

Application    lingtu               python lingtu.py nav          [systemd]
                                     HTTP :5050  MCP :8090
```

`robot-fastlio2.service`, `robot-localizer.service`, and `robot-lidar.service`
are legacy ROS2 compatibility units. They are useful for fallback or comparison
work, but they are not the product default and should not run beside the native
DDS chain unless a specific compatibility test requires it.

Runtime plugin registration follows the same rule: ROS2 adapter groups such as
`ros2_slam_bridge` and `ros2_map_output` are not part of the product plugin
catalog unless `LINGTU_ENABLE_ROS2_COMPAT=1` or
`LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1` is set before startup.

There is no standalone `lingtu-gateway.service`: Gateway runs inside
`lingtu.service` and exposes HTTP `:5050` plus MCP `:8090` from the same
process.

In the current field path, `lingtu.service` runs with
`LINGTU_COMMAND_OUTPUT_MODE=endpoint_only` and
`LINGTU_HARDWARE_CONTROL_BOUNDARY=driver`. The native navigation endpoint
publishes the only product velocity stream (`rt/nav/cmd_vel`), and
`lingtu-driver.service` is the unique speed exit to the remote Brainstem gRPC
server. The Python application must not enable a second robot driver in this
profile.

Only `robot-camera.service` should own the Orbbec ROS driver. Legacy camera
units such as `camera.service` or `orbbec-camera.service` must stay stopped or
masked; if they run together with `robot-camera.service`, duplicate Orbbec node
instances or image publishers can appear. Seeing one `/camera/camera` node and
one `/camera/camera_container` node is normal for the component launch.

## Service Inventory

| Service | Role | Default |
| --- | --- | --- |
| `lingtu-livox-dds` | Livox MID-360 SDK2 publisher into typed DDS | yes |
| `lingtu-slam-dds` | C++ SLAM/localization runtime and status snapshots | yes |
| `lingtu-nav-dds` | Native DDS navigation endpoint | yes |
| `lingtu-driver` | Native Thunder driver: typed DDS `rt/nav/cmd_vel` to remote Brainstem gRPC | yes |
| `lingtu` | Gateway, session control, navigation module graph | yes |
| remote Brainstem | Quadruped leg-control gRPC on a separate computer | yes |
| `robot-camera` | Camera service | yes |
| `robot-fastlio2` / `robot-localizer` / `robot-lidar` | ROS2 compatibility fallback | no |
| `robot-genz-icp` / `robot-hba` / `robot-super-lio*` | experimental evaluation backends | no |

## Filesystem Layout

```text
/opt/lingtu/
  current -> releases/vX.Y.Z/
  releases/
  config/
    thunder-runtime-env.sh
    brainstem.env
  nav/
  logs/

/etc/systemd/system/
  lingtu-livox-dds.service
  lingtu-slam-dds.service
  lingtu-nav-dds.service
  lingtu-driver.service
  lingtu.service

/home/sunrise/data/
  nova/maps/
```

Developer checkout on the robot usually lives at
`~/data/SLAM/navigation` or `~/data/inovxio/lingtu`.

## Installing On A Robot

```bash
# 1. Build native product modules
cd ~/data/SLAM/navigation
bash scripts/build/build_livox_sdk2_stream.sh
LINGTU_SLAM_BUILD_DDS_RUNTIME=ON LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF bash scripts/build/build_slam_core.sh
bash scripts/build/build_nav_endpoint.sh
bash scripts/build/build_driver.sh
bash scripts/build/build_nav_kernel.sh --clean
bash scripts/build/build_octoplanner3d.sh --require-pcl

# 2. Optional ROS2 compatibility workspace, only when explicitly needed
bash scripts/build/build_ros_workspace.sh

# 3. Install native field services. Brainstem is on a separate computer;
#    loopback or a missing endpoint is rejected.
LINGTU_BRAINSTEM_HOST=REMOTE_BRAINSTEM_IP \
LINGTU_BRAINSTEM_PORT=13145 \
LINGTU_BRAINSTEM_TLS_CA_FILE=/opt/lingtu/config/tls/brainstem-ca.crt \
LINGTU_BRAINSTEM_TLS_CERT_FILE=/opt/lingtu/config/tls/lingtu-driver.crt \
LINGTU_BRAINSTEM_TLS_KEY_FILE=/opt/lingtu/config/tls/lingtu-driver.key \
  bash scripts/deploy/thunder/install_services.sh field-cpp

# 4. Start the selected product through its resolved RuntimePlan
bash scripts/lingtu mode switch nav --map MAP_NAME --endpoint thunder_field
```

The product command validates the map and endpoint contract, removes stale
mode-owned services, starts the declared processes in dependency order, and
waits for role-specific readiness. Direct multi-service `systemctl start`
commands are diagnostic procedures, not a supported product startup path.

The legacy installer under `docs/04-deployment/services/install.sh` is guarded by
`LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1` and should not be used for new installs.

The driver installer persists the remote endpoint to
`/opt/lingtu/config/brainstem.env`. The remote Brainstem must support the
lease-and-ack v1 RPCs documented in
`src/drivers/real/thunder/native/README.md`; a legacy `Walk`-only server remains
fail-closed and cannot make `lingtu-driver` ready.

On the Brainstem computer, explicitly allow the LingTu host's source IP with
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
bash scripts/deploy/cut_release.sh vX.Y.Z
```

`cut_release.sh` is native-first: it builds/tests and installs the complete
`navd` endpoint package, writes an immutable selected-planner contract, then
requires a fresh endpoint status after restart. OctoPlanner3D and its converter
are required for the default `octoplanner3d` release. FAR is an explicit extra
backend:

```bash
LINGTU_RELEASE_GLOBAL_PLANNER=far \
  bash scripts/deploy/cut_release.sh vX.Y.Z
```

That release requires the active, validated `occupancy.npz`; it does not
silently fall back to OctoPlanner3D. The nanobind Python kernel is optional via
`LINGTU_RELEASE_REQUIRE_PYTHON_NAV_KERNEL=1`. ROS2 compatibility package checks
run only when `LINGTU_RELEASE_REQUIRE_ROS2_COMPAT=1`.

## Common Operations

| Need | Command |
| --- | --- |
| One-screen status | `bash scripts/lingtu status` |
| Watch status | `bash scripts/lingtu watch` |
| Restart native SLAM | `bash scripts/lingtu svc restart slam` |
| Restart localization chain | `bash scripts/lingtu svc restart localization` |
| Restart full native chain | `bash scripts/lingtu svc restart all` |
| Tail app logs | `journalctl -u lingtu -f` |
| Tail native SLAM logs | `journalctl -u lingtu-slam-dds -f` |
| Tail native Livox logs | `journalctl -u lingtu-livox-dds -f` |
| Tail native driver logs | `journalctl -u lingtu-driver -f` |
| Verify ports | `ss -tnlp \| grep -E '13145\|5050\|8090'` |

## Diagnostics

### Native Runtime Not Ready

```bash
bash scripts/lingtu svc status
bash scripts/lingtu health
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/map_cloud
jq . /dev/shm/lingtu/nav_endpoint_status.json
jq . /dev/shm/lingtu/driver_status.json
journalctl -u lingtu-slam-dds.service -n 80 --no-pager
```

If native binaries are missing, the service logs print the exact build command
for the missing component.

### LiDAR Or IMU Missing

```bash
journalctl -u lingtu-livox-dds.service -n 80 --no-pager
bash scripts/lingtu doctor
bash scripts/lingtu dataflow /nav/lidar_scan
bash scripts/lingtu dataflow /nav/imu
ip -br addr | grep 192.168.1
```

Use `lingtu doctor --ros2` only for explicit compatibility graph inspection.

### Drift Or Lost Localization

```bash
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/map_cloud
bash scripts/lingtu soak --duration 120 --interval 2 --json --strict
bash scripts/lingtu svc restart localization
```

`lingtu soak` is the preferred non-motion evidence command after boot or sensor
reconnects. It samples Gateway readiness, localization freshness, map-cloud
stability, command-source idleness, and stationary odometry displacement.

`svc restart localization` is the narrow restart path for native localization:
it stops conflicting legacy/experimental localization units, keeps or starts
`lingtu-livox-dds.service`, restarts `lingtu-slam-dds.service`, then waits for
the SLAM status snapshot and Gateway readiness. Use `svc restart all` only when
Livox, SLAM, nav DDS, and Gateway all need a cold restart.

Restart and relocalization are different gates:

| Operation | Command | What It Proves |
| --- | --- | --- |
| Restart localization process | `bash scripts/lingtu svc restart localization` | native SLAM DDS can restart and produce status again |
| Seeded saved-map relocalization | `bash scripts/lingtu nav relocalize <map> X Y YAW` | saved map accepts an operator-provided initial pose |
| Global saved-map relocalization | `bash scripts/lingtu nav global-relocalize <map>` | backend can find a saved-map alignment without a seed |
| Static drift gate | `bash scripts/lingtu slamcompare --map <map>` | stationary pose stays within configured drift limits |

Do not treat `TRACKING` alone as navigation-ready. For saved-map navigation,
`map_odom_tf` must be valid and the odometry must be inside the active map
frame. A relocalization result with low quality or identity `map->odom` is a
failed localization gate even if the SLAM process is alive.

### Legacy ROS2 Compatibility

The compatibility path remains available for field comparison:

```bash
LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1 bash scripts/deploy/thunder/install_services.sh ros-compat
bash scripts/lingtu svc restart legacy_lidar
bash scripts/lingtu svc restart legacy_fastlio2
bash scripts/lingtu svc restart legacy_localizer
bash scripts/lingtu svc status-legacy
bash scripts/lingtu doctor --ros2
```

Do not leave legacy LiDAR/SLAM services active beside the native DDS chain unless
the test plan explicitly requires it; duplicate drivers can fight for the same
sensor ports.

## References

- `lingtu_cli.md` - operations CLI subcommands
- `super_lio_backend.md` - experimental Super-LIO evaluation path
- `OTA_GUIDE.md` - historical OTA design
