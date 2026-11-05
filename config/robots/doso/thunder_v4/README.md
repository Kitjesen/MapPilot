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
