# MuJoCo MID-360 Fidelity Contract

Status: 2026-07-15

MuJoCo provides an official MID-360 scan-angle sequence over ideal geometric
ray casts. It is suitable for navigation-function regression, but it is not a
digital twin of the optical sensor and cannot replace a real MID-360 gate.

## Runtime Boundary

The simulation-only sensor adapter owns MuJoCo ray casting and sensor-message
construction; it owns no localization, planning, or control. In
`Port -> Wire -> Transport` terms, its LiDAR/IMU output role feeds typed sensor
payload wires over native DDS to the same native consumers used by the
navigation or SLAM gate. The backend profile below is an implementation choice
inside that adapter boundary, not a product Module API.

## Official Baseline

The nominal simulator profile is grounded in the following Livox sources:

- [MID-360 specifications](https://www.livoxtech.com/mid-360/specs)
- [MID-360 User Manual v1.2](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)
- [MID-360 Ethernet protocol](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)
- [Livox SDK2 point structures](https://github.com/Livox-SDK/Livox-SDK2/blob/master/include/livox_lidar_def.h)
- [Official pinned MID-360 scan pattern](https://github.com/Livox-SDK/livox_laser_simulation/blob/1cce1073633a062b92e30243a4c2920e45551bb5/scan_mode/mid360.csv)

Relevant official limits are:

| Property | Official value used by this contract |
| --- | --- |
| Horizontal / vertical field of view | `360 deg`, `-7 deg .. 52 deg` |
| Point rate / typical frame rate | `200,000 points/s`, `10 Hz` |
| Range under 100 klx | `40 m @ 10%`, `70 m @ 80%` reflectivity |
| Blind zone | `0.1 m`; accuracy is not guaranteed from `0.1 m` to `0.2 m` |
| Range random error, 1 sigma | at most `3 cm @ 0.2 m`, `2 cm @ 10 m` under the documented test conditions |
| Angular random error, 1 sigma | below `0.15 deg` |
| Return protocol | single return; 96 points per Ethernet data packet |
| Point time | packet timestamp is the first point; points are evenly spaced over `time_interval` |
| Normal confidence tag | `0x00`; `0x10` means moderate-confidence `other` |

## Nominal Simulation Profile

Within the simulation sensor-adapter boundary, the nominal profile uses:

```text
backend                         mujoco_lidar
require_product_backend          true
scan pattern                     official pinned mid360.csv conversion
samples_per_frame                20,000
frame rate                       10 Hz
range_min                        0.1 m
range_max                        40 m
range noise                      radial Gaussian, 3 cm @ 0.2 m -> 2 cm @ 10 m
angular noise                    0.10 deg sigma
nominal random dropout           disabled
nominal distance dropout         disabled
unknown material reflectivity    15, a 10% Lambertian proxy
normal point tag                 0x00
unknown physical channel/line    0
```

`40 m` is deliberate. MuJoCo geometry has no calibrated material reflectivity,
so granting the `70 m @ 80%` result to every surface would be systematically
optimistic. Non-zero dropout and intensity noise are fault-injection controls,
not nominal MID-360 parameters.

All backends use the same return-model seam for range noise, dropout, and the
documented intensity proxy. `mujoco_lidar` and official-pattern fallback apply
angular perturbation before ray casting. The plugin backend cannot accept the
angle table, so it applies a post-return angular approximation and reports that
limitation in `backend_report()`.

## Scan-Time Profiles

| Profile | Geometry and point times | Permitted evidence |
| --- | --- | --- |
| `physical_rolling/subscan` | Samples the official pattern at 200 Hz simulation states, then assembles a 100 ms frame with actual subscan offsets | MuJoCo LiDAR/IMU timing and SLAM sensor regression |
| `instantaneous` | One full ray cast at one MuJoCo state; all offsets are zero | Navigation/control functional regression only |
| `synthetic_rolling` | One instantaneous geometry snapshot with synthetic offsets | Diagnostic compatibility only; not sensor acceptance |

The 200 Hz physical profile is still an approximation: points inside each 5 ms
subscan share one pose. It does not reproduce the firmware's 96-point packet
schedule or per-point motion continuously.

The general `MujocoDriver` publishes instantaneous clouds with zero point
offsets. The dedicated native DDS sensor bridge defaults to
`physical_rolling/subscan`. A navigation acceptance manifest may explicitly
use a smaller instantaneous frame to protect control-loop performance, but it
must declare `sensor_accuracy=evidence_only` and keep SLAM accuracy in a
separate gate.

The 2026-07-16 industrial-park 60 m function gate uses exactly that isolation
profile: 10,000 rays in one instantaneous 10 Hz frame, 100 Hz navigation IMU,
and MuJoCo truth navigation state. New recordings store at most 640
presentation points per motion frame. Sampling first removes non-finite and
roof/overhead returns, then reserves 75% of the budget for points within 4 m of
the robot and retains the remainder as wider scene context. This is a video-log
sampling policy only; native DDS and planner inputs still receive their
configured full frames.

The local inset separates the evidence layers:

- cyan points: sampled raw LiDAR returns;
- dark red points: obstacles admitted to the planner;
- red/orange cells: traversability risk;
- gray/red/orange fans: feasible, blocked, and terrain-penalized candidates;
- bright green: the selected local path.

The presentation renderer scales its headlight and all scene lights by `0.65`
before rasterization. A video is rejected when any first/middle/last keyframe
has more than 10% luminance-clipped pixels. This fixes information loss at the
rendering source; H.264 or post-process darkening is not used to hide clipped
white materials.

The white fan is sampled candidate paths, not LiDAR rays. Therefore the video
is a planner/control visualization, not a complete raw-cloud or optical
fidelity proof. Artifacts made before the 640-point/local-priority change still
contain only their old 240-point samples and cannot be made denser by
re-rendering.

## What MuJoCo Does and Does Not Validate

| Function | MuJoCo status | Required real-world follow-up |
| --- | --- | --- |
| Scan direction coverage and ordering | Official pinned scan pattern | Firmware/version cross-check when upgraded |
| Self-occlusion and scene geometry | Robot geoms excluded as a group; environment ray-cast is exercised | Verify physical mount, cables, body reflections |
| Native DDS framing and cadence | Exercised at 10 Hz LiDAR and configured IMU rate | Packet loss, clock sync and long-run network soak |
| Range/angular error envelope | Explicit nominal approximation | Calibrated bag comparison against known targets |
| Reflectivity and tag semantics | Conservative proxy; normal tag is correct | Material, incidence-angle and retroreflector validation |
| Rolling motion distortion | 200 Hz subscan approximation | Real 96-point packet timing and deskew validation |
| Weather and optical failure modes | Not modeled | Rain, fog, dust, sunlight, dirty window and temperature gates |
| Multipath, mixed pixels, thin/black/wet surfaces | Not modeled | Real target suite |
| Navigation planning and control | Valid when its own manifest passes | Hardware locomotion and final safety acceptance |

## Acceptance Boundary

Navigation acceptance and sensor acceptance are intentionally independent:

```text
Navigation function gate
  existing map -> global planner -> local planner -> PathFollower
  -> final safety -> native DDS cmd_vel -> ThunderV4 -> goal arrival

Sensor / SLAM gate
  physical_rolling MID-360 + 200 Hz IMU -> native DDS -> Fast-LIO2
  -> odometry / registered cloud / map quality

Real sensor gate
  physical MID-360 packet capture -> timing / materials / weather / deskew
  -> field SLAM and navigation
```

A successful navigation-function run proves the control dataflow and obstacle
response. It does not prove real MID-360 optics, Fast-LIO scale stability, or
long-term map quality.
