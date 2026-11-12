# DOSO Thunder V4

`model.yaml` describes the Thunder V4 and keeps its MID-360 configuration
independent of Go2. Robot selection does not choose `real` or `sim`.

| Area | Owner |
| --- | --- | --- |
| Motion | `src/drivers/real/motion/robots/doso/` |
| MID-360 / LTLIO | `config/robots/doso/thunder_v4/sensors/mid360_ltlio.yaml` |
| Simulation | `sim/packages/robots/doso/thunder_v4/` |
| Deployment | `scripts/deploy/thunder/` |
| Products | `config/system/products/` |

The same robot ID now resolves in both environments:

- `real` reads the adjacent `robot.yaml` and native DOSO/Brainstem settings.
- `sim` selects a preset from `sim/sessions/products/doso/thunder_v4/`.

This proves configuration and compilation only; field motion still requires the
normal no-motion and supervised-motion acceptance on the target robot.

## Navigation support height

The real `robot.yaml` now declares `geometry.support_height: 0.435` m and
`geometry.support_height_tolerance: 0.10` m. These are **provisional model-based
values, not field calibration**. Their source is the existing Thunder V4
[simulation package](../../../../sim/packages/robots/doso/thunder_v4/robot.package.yaml)
and its [standing-height explanation](../../../../sim/packages/robots/doso/thunder_v4/README.md#navigation-geometry).
The height is measured from the supporting surface to the `base_link` origin,
not to the top of the robot or the LiDAR. The nominal matching band is
0.335–0.535 m; the planner also accounts for occupied voxel extent. This is not
a maximum pit depth or a qualified step-climbing capability.

Real Product assembly passes these two fields through RunPlan to
`LINGTU_NAV_OCTO_SUPPORT_HEIGHT_M` and
`LINGTU_NAV_OCTO_SUPPORT_HEIGHT_TOLERANCE_M`. Previously both resolved to zero.
OctoPlanner uses the height to find support and lift ground routes to body-origin
routes; SCAN consumes those routes through the existing height conversion.
This configuration does not command the robot's standing posture. A newly
compiled RunPlan is required; an already-running process retains its old values.

Pinned upstream SCAN
[`348e8a590a50a5a6bbab8d8c6dcfd171f009be26`](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/launch/advanced_param.xml)
uses an example `grid_map/body_height=0.4` m. The Thunder height and support
tolerance above belong to LingTu's platform adaptation, not that upstream default.

Configuration evidence (2026-11-06): the same `test_real_support_height_reaches_run_plan`
cases failed for Thunder `map` and `nav` before this change (height resolved to
zero), then all four Thunder/Go2 cases passed with RunPlan serialization included.
The existing Thunder real/sim collision-geometry and Go2 standing-height tests
also passed: six checks total. These checks neither started hardware nor proved
field support matching.

Before field motion, measure floor-to-`base_link` height in the actual loaded
standing mode and check its variation in the intended posture range. Replace
the estimate and assess the tolerance using those observations and map resolution.
Brainstem owns standing control; the local driver only requests `standUp()`.
Fixed sensor extrinsics do not change merely because the robot stands taller.

## Navigation collision model

`Body` is the C++ navigation geometry container and `BODY2` is its internal
serialization format. See the
[maintained model design](../../../../docs/design/navigation-body-model-and-motion-posture.md).

The real RobotConfig and formal simulation package select an eight-part compound
navigation model. It has four chassis boxes, preserving the narrow nose, tail
and raised sensor, plus four leg-clearance boxes. Each leg box preserves the
analytic XY projection of that leg's hip, thigh, calf and wheel at
`v4_nominal_stand`. Its vertical range follows the hip and thigh. The real asset is
[`body/thunderv4_navigation.body2`](body/thunderv4_navigation.body2). The exporter
writes the identical simulation copy and records source parts, projection
bounds and all 16 joint positions in the adjacent JSON.

The raw outer bounds are 0.835000 × 0.598331 × 0.411549 m, with body Z from
-0.186777 to 0.224772 m. `collision_hard_margin=0.08 m` is applied once in
world-horizontal axes, giving axis-aligned outer bounds of
0.995000 × 0.758331 m at zero yaw. Primitive cylinder projections preserve the
physical nominal footprint without the extra corners of their enclosing boxes.
The four leg regions reserve horizontal space above the moving lower legs and
wheels. Low obstacles below their vertical range and moving leg/wheel clearance
require locomotion qualification. The source MJCF continues to own physical
wheel-ground contacts. Global three-dimensional support checks remain enabled.

The source MJCF and nominal keyframe determine this envelope. Installation and
walking-envelope qualification are recorded as pending in its metadata.
Regenerate both real and sim assets using the existing development MuJoCo environment:

```bash
python tools/robot/export_navigation_bodies.py
```

Native field navigation does not load MuJoCo. The articulated `body_physics`
benchmark can update joint geometry separately; that is a distinct evidence
level from the default nominal model. See the
[model/configuration audit and correction](../../../../docs/operations/mujoco-navigation-20261026.md#模型来源与配置混用的修正).
