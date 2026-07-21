# Sunrise S100P Native Validation

- Date: 2026-07-20
- Target: `sunrise@192.168.66.13`
- Architecture: Ubuntu GNU/Linux, ARM aarch64, real-time kernel
- Scope: read-only product preflight plus isolated, non-motion native validation

## Result Summary

| Gate | Result | Evidence |
| --- | --- | --- |
| Host and CycloneDDS preflight | PASS | aarch64 host reachable; CycloneDDS runtime and `idlc` installed |
| Isolated native build | PASS | latest source built under the test directory |
| Native CTest | PASS | 39/39 tests passed |
| Runtime dependency boundary | PASS | no ROS 2, RMW, tf2, PCL, Open3D, or VTK runtime link |
| Isolated DDS map input chain | PASS | mock odometry/TF and synthetic cloud produced a ready exploration snapshot |
| Exploration lifecycle command E2E | NOT PASSED | endpoint received zero control messages in the attempted run |
| Real MID-360 LiDAR/IMU chain | BLOCKED | `eth1` reported `NO-CARRIER`; device subnet was unreachable |
| Robot motion | NOT RUN | no driver was started and no motion command was published |

## Installed Product Preflight

`/opt/lingtu/current` resolves to `/home/sunrise/data/inovxio/lingtu`.
The following installed services were active when inspected:

- `lingtu-livox-dds`
- `lingtu-slam-dds`
- `lingtu-traversability-dds`
- `lingtu-nav-dds`
- `lingtu.service`

`lingtu-explore-dds` and `lingtu-driver` were inactive. The active navigation
service was configured with command publication, obstacle checking, and
traversability-cost use disabled. No service was installed, restarted, or
reconfigured during this run.

The installed SLAM status remained `INITIALIZING` with reason
`waiting_for_imu`. LiDAR, IMU, odometry, registered-cloud, and map-cloud
counters were all zero.

## MID-360 Blocker

The Livox configuration expects the host at `192.168.1.5` and the MID-360 at
`192.168.1.178` on `eth1`. The interface was administratively enabled, but it
remained `NO-CARRIER` with carrier value `0`; the LiDAR address had 100 percent
packet loss.

This is a physical Ethernet/power/switch-path blocker. The software run cannot
validate packet reception, LiDAR/IMU synchronization, registered clouds, or
real dynamic-obstacle clearing until carrier is restored.

## Isolated Native Build

The current source snapshot was built outside the installed release at:

```text
/home/sunrise/lingtu-test/20260720_native_explore_v2
```

After the complete navigation path resources were included, the full native
CTest suite passed **39/39**. The produced executables and shared library are
ARM aarch64 ELF files.

Dependency scans covered:

- `navd`
- `lingtu_explore_dds`
- `lingtu_traversability_dds`
- `liblingtu_nav_client.so`

They link CycloneDDS, standard C/C++ libraries, and optional iceoryx support.
No ROS 2, rclcpp, rcutils, RMW, tf2, PCL, Open3D, or VTK runtime dependency was
found.

## Isolated DDS Data Chain

The non-motion test used CycloneDDS domain `77` and did not start `navd` or a
robot driver:

```text
lingtu_motion_mock_dds
  -> /slam/odometry + /tf
  -> lingtu_traversability_dds
  <- synthetic registered cloud
  -> /nav/exploration_snapshot
  -> lingtu_explore_dds
```

Observed final evidence included:

- 189 odometry and TF samples at the exploration endpoint
- three accepted exploration snapshots
- `ready: true`
- `map` geometry frame with a live map identity and increasing generation
- no rejected odometry or snapshots
- zero received `cmd_vel` samples in the motion mock

The traversability endpoint accepted the synthetic cloud, preserved the
`body -> map` frame contract, and reported zero frame-contract errors.

## Exploration Control Gap

The attempted C ABI lifecycle run did **not** prove the typed control path.
The endpoint status ended with:

```text
control_messages: 0
control_accepted: 0
control_duplicates: 0
control_rejected: 0
```

Therefore `START -> duplicate START -> PAUSE -> RESUME -> STOP` must be rerun
and asserted from endpoint counters and ACKs. A successful rerun requires:

- five received control samples
- four accepted unique commands
- one idempotent duplicate
- zero rejected commands
- final `active: false` and `paused: false`

During the rerun attempt the host became unreachable (`ping` and TCP port 22
both failed), so this gate remains open rather than being reported as a pass.

The repeatable isolated-domain command sequence is:

```bash
lingtu_nav_control explore start e2e-session --request-id e2e-start --domain-id 77
lingtu_nav_control explore start e2e-session --request-id e2e-start --domain-id 77
lingtu_nav_control explore pause field_validation --request-id e2e-pause --domain-id 77
lingtu_nav_control explore resume field_validation --request-id e2e-resume --domain-id 77
lingtu_nav_control explore stop field_validation --request-id e2e-stop --domain-id 77
```

## Safety Boundary

- `/opt/lingtu/current` was not modified.
- No release symlink was changed.
- No systemd unit was installed, enabled, restarted, or stopped.
- No robot driver was started.
- No physical motion command was sent.
- Isolated test processes used domain `77` and had bounded timeouts.

## Next Field Actions

1. Restore power/link on the MID-360 Ethernet path and verify `eth1` carrier.
2. Confirm live Livox packet, IMU, odometry, registered-cloud, and map-cloud counters.
3. Rerun the isolated typed exploration lifecycle gate and retain its ACK/status report.
4. Only after those gates pass, run a supervised zero-speed readiness check before any motion test.
