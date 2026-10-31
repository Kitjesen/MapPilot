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

The documented multi-shape representation is the **组合碰撞模型（Compound Collision
Model）**: several collision primitives linked by body-relative transforms.
URDF supplies geometry; `Body` is the C++ container and `BODY2` is a legacy
internal serialization magic. See the
[maintained model design](../../../../docs/design/navigation-body-model-and-motion-posture.md).

The real RobotConfig and formal simulation package select the same nominal
compound geometry: 38 raw collision primitives from the checked-in
`thunderv4.xml`, evaluated at its actual `v4_nominal_stand` keyframe. The real
asset is [`body/thunderv4_nominal.body2`](body/thunderv4_nominal.body2); the
simulation copy is generated from that same export, not maintained separately.
The metadata beside each asset records all 16 joint positions and source qpos.
No Go2 stance angles are applied to Thunder.

The foot cylinders may contact the support surface; they do not require all
four fixed-pose feet to touch the floor on every query. Global ground-support
checks remain enabled. `collision_hard_margin=0.08 m` is applied once in
world-horizontal axes at runtime and is not baked into the model. The previous
single OBB is no longer selected by this configuration.

This is a fixed nominal model, not live joint FK or a measured walking envelope.
The source URDF is not present locally; the checked-in MJCF is the geometry
source. `installation_qualified=false` records that physical installation and
field-motion qualification are still outstanding. Regenerate both real and
sim assets using the existing development MuJoCo environment:

```bash
python tools/robot/export_navigation_bodies.py
```

Native field navigation does not load MuJoCo. The articulated `body_physics`
benchmark can update joint geometry separately; that is a distinct evidence
level from the default nominal model. See the
[model/configuration audit and correction](../../../../docs/operations/mujoco-navigation-20261026.md#模型来源与配置混用的修正).
