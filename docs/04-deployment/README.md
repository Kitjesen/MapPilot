# LingTu Thunder Deployment

Status: current Thunder deployment guide as of 2026-07-25.

How LingTu is laid out on the field robot, how to install, update, roll back,
and diagnose the running stack. The product runtime is native DDS plus
Product-managed native services and scoped Host Modules; ROS 2 compatibility services are optional
fallbacks and must be enabled explicitly.

## One-Liner Status

```bash
export LINGTU_ROBOT_HOST=ROBOT_IP_OR_HOSTNAME
ssh sunrise@"$LINGTU_ROBOT_HOST" 'systemctl status lingtu-livox-dds lingtu-slam-dds mapd lingtu-traversability-dds lingtu-nav-dds lingtu-driver lingtu'
ssh sunrise@"$LINGTU_ROBOT_HOST" 'bash /opt/lingtu/current/scripts/lingtu status'
```

The `lingtu` CLI gives an 8-section snapshot; see `lingtu_cli.md`.

## Robot Runtime

```text
Sensor DDS     lingtu-livox-dds     Livox SDK2 -> typed DDS       [systemd]
SLAM DDS       lingtu-slam-dds      C++ SLAM runtime + snapshots  [systemd]
Maps           mapd                 Native layers/state/scene     [systemd]
Risk Grid      traversability       Native control-risk grid      [systemd]
Nav Endpoint   lingtu-nav-dds       Native DDS planner/control    [systemd]
Driver         lingtu-driver        DDS cmd_vel -> Brainstem gRPC [systemd]

Control        remote Brainstem     Dart gRPC :13145              [external]
Camera         lingtu-camera-dds     Native Orbbec -> typed DDS    [systemd]

Host           lingtu               RunPlan -> Blueprint          [systemd]
                                     Gateway HTTP :5050 / MCP :8090
```

Product mode changes run through
`scripts/lingtu -> lingtu.control.ProductControl -> systemd`.
`lingtu.py` is the Host process entry after it verifies the published RunPlan;
it is not a second field-process controller.

`robot-camera.service`, `robot-fastlio2.service`,
`robot-localizer.service`, and `robot-lidar.service` are retired ROS2 unit-name
tombstones. The repository ships neither their templates nor an installer, and
ProductControl never starts them. Doctor, the service catalog, and the real-env
conflict list retain the names only to detect externally installed conflicts.

Runtime plugin registration is a separate opt-in boundary: ROS2 adapter groups
such as `ros2_slam_bridge` and `ros2_map_output` are not part of the product
plugin catalog unless `LINGTU_ENABLE_ROS2_COMPAT=1` is set before startup.

There is no standalone `lingtu-gateway.service`: Gateway runs inside
`lingtu.service` and exposes HTTP `:5050` plus MCP `:8090` from the same
process.

In the current field path, `lingtu.service` runs with
`LINGTU_COMMAND_OUTPUT_MODE=endpoint_only` and
`LINGTU_HARDWARE_CONTROL_BOUNDARY=driver`. The native navigation endpoint
publishes the only product velocity stream (`rt/nav/cmd_vel`), and
`lingtu-driver.service` is the unique speed exit to the remote Brainstem gRPC
server. The Python application must not enable a second robot driver in this
Product.

`lingtu-camera-dds.service` owns the current native Orbbec capture path.
`robot-camera.service`, `camera.service`, and `orbbec-camera.service` are
retired unit-name tombstones kept only for conflict detection. If any is
externally installed, disable or remove it before starting
`lingtu-camera-dds.service`. The `/camera/camera` and
`/camera/camera_container` nodes belong only to that historical graph.

## Service Inventory

| Service | Role | Default |
| --- | --- | --- |
| `lingtu-livox-dds` | Livox MID-360 SDK2 publisher into typed DDS | yes |
| `lingtu-slam-dds` | C++ SLAM/localization runtime and status snapshots | yes |
| `mapd` | Native realtime map ingestion, layers, state, and scene | yes where declared by Product |
| `lingtu-traversability-dds` | Native control-risk grid and unique `/nav/traversability` writer | yes for obstacle-aware Products |
| `lingtu-nav-dds` | Native DDS navigation endpoint and final logical command owner | yes |
| `lingtu-driver` | Native Thunder driver: typed DDS `rt/nav/cmd_vel` to remote Brainstem gRPC | yes |
| `lingtu` | Python Host for Gateway, Agent, MCP, and low-rate adapters | yes |
| remote Brainstem | Quadruped leg-control gRPC on a separate computer | yes |
| `lingtu-camera-dds` | Native Orbbec capture and typed DDS publisher | yes where declared by Product |
| `lingtu-explore-dds` | Native exploration policy process | yes where declared by Product |
| `robot-camera` / `robot-fastlio2` / `robot-localizer` / `robot-lidar` | retired ROS2 unit-name tombstones; external conflicts only | no |

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
  mapd.service
  lingtu-traversability-dds.service
  lingtu-nav-dds.service
  lingtu-camera-dds.service
  lingtu-explore-dds.service
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
bash scripts/build/build_mapd.sh
bash scripts/build/build_nav_endpoint.sh
bash scripts/build/build_driver.sh
bash scripts/build/build_native_recording.sh
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

# 4. Start the Product through ProductControl in env=real
bash scripts/lingtu --env real mode switch nav --map MAP_NAME
```

The Product command validates the map and RunPlan contract, removes stale
mode-owned services, starts the declared processes in dependency order, and
waits for role-specific readiness. Direct multi-service `systemctl start`
commands are diagnostic procedures, not a supported product startup path.

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
`LINGTU_RELEASE_REQUIRE_PYTHON_NAV_KERNEL=1`.

Before any build or activation, `cut_release.sh` loads the canonical
ProductControl `current.json` and its referenced RunPlan. A missing record,
invalid fingerprint, or Product/Env mismatch stops the release. Product identity,
native control mode, and the driver/mapd process targets come from that exact
RunPlan; ambient `LINGTU_PRODUCT` values and hand-written unit lists are not
release inputs. ROS2 compatibility packages and services are outside this native
release path and remain isolated under `scripts/compat/ros2/`.

## Common Operations

| Need | Command |
| --- | --- |
| One-screen status | `bash scripts/lingtu status` |
| Watch status | `bash scripts/lingtu watch` |
| Restart the logical SLAM process | `bash scripts/lingtu svc restart slam` |
| Restart the logical Host process | `bash scripts/lingtu svc restart host` |
| Reapply the committed Product | `bash scripts/lingtu svc reapply` |
| Record native DDS | `bash scripts/lingtu record` |
| Record native DDS and camera | `bash scripts/lingtu record --camera` |
| Verify a recording | `bash scripts/lingtu record verify SESSION_DIR` |
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
bash scripts/lingtu svc restart slam
```

`lingtu soak` is the preferred non-motion evidence command after boot or sensor
reconnects. It samples Gateway readiness, localization freshness, map-cloud
stability, command-source idleness, and stationary odometry displacement.

`svc restart slam` is a thin operator entry. It sends the logical `slam` label
unchanged to ProductControl, which resolves and restarts exactly that process
from the committed RunPlan and owns its readiness and failure handling. Bash
does not select a backend, sequence a service chain, or poll readiness itself.
Use `svc reapply` (equivalently, `svc restart all`) only when the exact committed
Product must be reapplied.

Restart and relocalization are different gates:

| Operation | Command | What It Proves |
| --- | --- | --- |
| Restart SLAM process | `bash scripts/lingtu svc restart slam` | the single native SLAM DDS process can restart and produce status again |
| Seeded saved-map relocalization | `bash scripts/lingtu nav relocalize <map> X Y YAW` | saved map accepts an operator-provided initial pose |
| Global saved-map relocalization | `bash scripts/lingtu nav global-relocalize <map>` | backend can find a saved-map alignment without a seed |
| Stationary RunPlan gate | `bash scripts/lingtu doctor --non-motion --strict` | declared processes and live no-motion readiness agree |
| Integrated saved-map/plan gate | `bash scripts/lingtu system-acceptance --map <map> --goal X Y YAW` | map, localization, and requested plan evidence pass without motion |

Do not treat `TRACKING` alone as navigation-ready. For saved-map navigation,
`map_odom_tf` must be valid and the odometry must be inside the active map
frame. A relocalization result with low quality or identity `map->odom` is a
failed localization gate even if the SLAM process is alive.

### Legacy ROS2 Detection

The repository no longer ships the S100P ROS2 installer or its systemd unit
templates. Field doctor still detects a legacy ROS graph that is already present
on a robot so operators can collect comparison or removal evidence:

```bash
bash scripts/lingtu doctor --ros2
```

Do not leave externally installed legacy LiDAR/SLAM services active beside the
native DDS chain; duplicate drivers can fight for the same sensor ports.

## References

- `lingtu_cli.md` - operations CLI subcommands
- `native_recording.md` - native C++ recording, replay, safety, and recovery boundary
- `OTA_GUIDE.md` - native release packaging, external OTA distribution boundary, installation, and rollback
