# Unitree Go2

`model.yaml` describes the Go2 and its MID-360 configuration at
`sensors/mid360_fastlio2.yaml`. Robot selection does not choose `real` or
`sim`; the adjacent `robot.yaml` supplies the physical configuration when the
selected Env is `real`.

| Area | Owner |
| --- | --- | --- |
| Motion | `src/drivers/real/motion/robots/unitree/go2/` |
| Physical configuration | `config/robots/unitree/go2/robot.yaml` |
| MID-360 / Fast-LIO2 | `config/robots/unitree/go2/sensors/mid360_fastlio2.yaml` |
| Deployment | `scripts/deploy/` |
| Products | `config/runtime_graph/products/` |

`unitree/go2 + real` has runnable configuration. The simulation environment
does not yet bind a Go2 session and robot assets, so `unitree/go2 + sim`
currently fails for that missing input rather than because the model forbids
`sim`.

## Navigation collision geometry

The real Go2 RobotConfig keeps each horizontal safety quantity under one owner:

- Physical envelope: `0.76 x 0.31 m`, using Unitree's larger published crouching length.
- CMU and final safety: the physical rectangle plus one `0.10 m` hard margin per side.
- SCAN: the Go2-tuned heading-aligned double cylinder, `radius=0.25 m` and
  `offset=0.18 m`, with no additional footprint padding.
- `live_obstacle_inflation_radius_m` belongs to active-path blockage overlay
  diagnostics. It is not added to the robot hard footprint.

The vertical clearances remain separate because they are measured from LingTu's
`body` origin and include the legs and the installed MID-360 bracket. Upstream
SCAN obstacle Z inflation is not a replacement for those measurements.

The deployment host address is site state and is intentionally not part of the
model. SDK2 motion discovery uses `driver.network_interface` from
`robot.yaml`, not a configured Go2 target IP.

## Navigation deployment gates

Before starting the `nav` Product on the expansion Linux computer:

1. The host must have a reachable interface on the Go2 wired subnet; set
   `driver.network_interface` to that host's actual interface name.
2. Unitree SDK2 must be installed and the Go2 motion implementation must build on the
   target architecture.
3. Validate the nominal body-to-LiDAR composite derived from the official Go2
   URDF plus the archived MID-360 interface mirror. Update both
   `robot.yaml` and `sensors/mid360_fastlio2.yaml` if the installed
   bracket differs, and change `calibration.status` only after the calibration
   check passes.
4. Verify the MID-360 host/device addresses and obtain a fresh map with the
   point-cloud, occupancy, and OctoMap artifacts required by `nav`.
5. Install the `lt-*` units, run a no-motion readiness check, and only then run
   a supervised bounded-motion acceptance.

## Go2 EDU + external MID-360 field target

The current first field target is:

- Sunrise is a temporary development bridge only.
- The Go2 expansion Linux/NX computer is the production runtime host.
- `teleop_avoid` uses the external MID-360 point cloud. Fast-LIO also consumes
  the MID-360 package IMU; the camera is not an input to this first acceptance.
- Operator input is physical velocity (`m/s`, `rad/s`) over `/ws/teleop`, then
  typed DDS operator motion, native local planning/final safety, and finally the
  Go2 SDK2 motion adapter.

The Go2 configuration uses the nominal Unitree companion MID-360 bracket pose
from an archived interface-document mirror, composed with the public Go2
base-to-IMU transform. It is intentionally separate from both the Go2 built-in
L1 radar and Thunder V4's front-nose mount. The adjacent `robot.yaml` records
the verified MID-360 result and keeps the unmeasured camera separate.

There is also no official public default SSH username/password for a Go2 EDU
expansion Jetson. NVIDIA Jetson images create or pre-provision the account; use
the credentials delivered with this particular NX image. Do not add guessed
credentials to the repository.

Official references:

- [Unitree Go2 specifications](https://www.unitree.com/go2/)
- [Unitree Go2 accessories: D435i and MID-360](https://www.unitree.com/go2/charger/)
- [Unitree Go2 body/front-camera xacro](https://github.com/unitreerobotics/unitree_ros/blob/daadf41ee9afce8f90fdc09a98506012691fa122/robots/go2_description/xacro/robot.xacro#L95-L102)
- [Unitree MID-360 SLAM selector example](https://github.com/unitreerobotics/unitree_slam/blob/1e49bfa4dca2c992a566ee7d1d8480d4102149b8/unitree_slam_example/demo_mid360.cpp#L158-L188)
- [NVIDIA Jetson account provisioning](https://docs.nvidia.com/jetson/l4t/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html)

The executable field sequence is documented in
[`docs/04-deployment/go2_edu_mid360_teleop_avoid.md`](../../../../docs/04-deployment/go2_edu_mid360_teleop_avoid.md).
