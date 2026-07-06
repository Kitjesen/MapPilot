# LingTu Thunder Deployment

How LingTu is laid out on the field robot, how to install, update, roll back,
and diagnose the running stack. The product runtime is native DDS plus
Module-first Python/C++ services; ROS 2 compatibility services are optional
fallbacks and must be enabled explicitly.

## One-Liner Status

```bash
ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc 'systemctl status lingtu-livox-dds lingtu-slam-dds lingtu-nav-dds lingtu'
ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc 'bash ~/data/SLAM/navigation/scripts/lingtu status'
```

The `lingtu` CLI gives an 8-section snapshot; see `lingtu_cli.md`.

## Robot Runtime

```text
Sensor DDS     lingtu-livox-dds     Livox SDK2 -> typed DDS       [systemd]
SLAM DDS       lingtu-slam-dds      C++ SLAM runtime + snapshots  [systemd]
Nav DDS        lingtu-nav-dds       Gateway/DDS nav endpoint      [systemd]

Control        robot-brainstem      Dart gRPC :13145              [systemd]
Camera         robot-camera         Orbbec camera service         [systemd]

Application    lingtu               python lingtu.py nav          [systemd]
                                     HTTP :5050  MCP :8090
```

`robot-fastlio2.service`, `robot-localizer.service`, and `robot-lidar.service`
are legacy ROS2 compatibility units. They are useful for fallback or comparison
work, but they are not the product default and should not run beside the native
DDS chain unless a specific compatibility test requires it.

There is no standalone `lingtu-gateway.service`: Gateway runs inside
`lingtu.service` and exposes HTTP `:5050` plus MCP `:8090` from the same
process.

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
| `lingtu` | Gateway, session control, navigation module graph | yes |
| `robot-brainstem` | Quadruped leg-control gRPC | yes |
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
  nav/
  logs/

/etc/systemd/system/
  lingtu-livox-dds.service
  lingtu-slam-dds.service
  lingtu-nav-dds.service
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
bash scripts/build/build_nav_kernel.sh --clean
bash scripts/build/build_octoplanner3d.sh --require-pcl

# 2. Optional ROS2 compatibility workspace, only when explicitly needed
bash scripts/build/build_ros_workspace.sh

# 3. Install native field services
bash scripts/deploy/thunder/install_services.sh field-cpp

# 4. Start native DDS + LingTu
sudo systemctl start lingtu-livox-dds.service lingtu-slam-dds.service lingtu-nav-dds.service lingtu.service
```

The legacy installer under `docs/04-deployment/services/install.sh` is guarded by
`LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1` and should not be used for new installs.

## Release Flow

```bash
cd ~/data/SLAM/navigation
bash scripts/deploy/cut_release.sh vX.Y.Z
```

`cut_release.sh` is native-first: it gates on `nav_kernel`, OctoPlanner3D, and
the OctoMap PCD converter. ROS2 compatibility package checks run only when
`LINGTU_RELEASE_REQUIRE_ROS2_COMPAT=1`.

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
| Verify ports | `ss -tnlp \| grep -E '13145\|5050\|8090'` |

## Diagnostics

### Native Runtime Not Ready

```bash
bash scripts/lingtu svc status
bash scripts/lingtu health
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/map_cloud
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
LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1 bash scripts/deploy/s100p/install_services.sh
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
