# Thunder V4 MuJoCo Asset

This directory targets the real Thunder V4 small-wheel hardware asset.

Source of truth:

- Upstream repo: `Kitjesen/thunder_assets`
- Default local source URDF: `thirdpart/thunder_assets/thunder_v4/urdf/thunder_v4.urdf`
- Default local source meshes: `thirdpart/thunder_assets/thunder_v4/meshes`
- Local MJCF: `mjcf/thunderv4.xml`

Regenerate:

```powershell
python D:\inovxio\brain\lingtu\sim\packages\robots\doso\thunder_v4\tools\generate_thunderv4_mjcf.py
python D:\inovxio\brain\lingtu\sim\packages\robots\doso\thunder_v4\tools\generate_thunderv4_mjcf.py --scene stairs
python D:\inovxio\brain\lingtu\sim\packages\robots\doso\thunder_v4\tools\check_thunderv4_hardware.py
```

Collision primitives are physically enabled but transparent by default. For a
geometry-debug XML, add `--show-collisions` to either generator command.

Generated hardware contract:

- total modeled mass: `45.8086 kg`
- free floating `base_link`
- RobotLab-compatible joint names and policy action order
- actuator order: 12 leg joints first, then 4 wheel joints
- leg actuator limit: `120 Nm`
- wheel actuator limit: `17 Nm`
- leg speed limit: `17.48 rad/s`; wheel speed limit: `44 rad/s`
- Wheel collision radius is copied from the selected URDF; the checked-in
  MJCF uses `0.093 m`.
- `v4_nominal_stand` keyframe: a valid V4 standing reference with no joint-limit target

The upstream V4 URDF reviewed on 2026-09-08 uses `0.095 m` wheel collision
radii. Regenerating from that revision also changes geometry. Use the URDF
associated with the policy's training asset when reproducing an existing
rollout; qualify a radius change separately.

## Navigation geometry

`robot.package.yaml:navigation_geometry` describes distances from `base_link`,
not half the robot's standing height. The fixed torso collision boxes span
Z `[-0.00209, 0.224772] m` in that frame; the navigation envelope uses
`0.05 m` below and `0.35 m` above. The lower envelope is one local-map cell,
leaving `0.04791 m` beneath the rigid torso; the upper envelope retains over
`0.12 m` above its highest box. A symmetric half-standing-height envelope
incorrectly reserves leg space as rigid torso clearance.
Articulated legs contact the support surface and are not rigid torso boxes.
The packaged standing controller's nominal support-to-base height is `0.435 m`
with `0.10 m` allowance for body heave and voxelization. This is not a stair
climbing qualification of the policy.

The simulation resolver carries these values into the physics plan. Product
assembly uses that already-resolved geometry for native navigation: OctoPlanner
checks ground support separately from the torso envelope; Mapd uses the same
below/above envelope for obstacle inflation. Inflation above an obstacle equals
the body's clearance **below** its origin, and vice versa. SCAN consumes those
inflated LiDAR cells directly. Geometry changes require a new RunPlan and mapd
process; an existing inflated bitmap cannot be reinterpreted with smaller bounds.

MuJoCo's feeder checks stability using measured vertical clearance above physical
terrain, not world Z. Its motion-evidence `min_base_height_m` and
`max_base_height_m` use that clearance; trajectory positions still use world Z.
A downward physics query excludes robot geometry and non-colliding decoration.
It is used only for simulation evidence, not as a replacement for LiDAR mapping.
Floor changes therefore do not trigger the height/span gate, while insufficient
clearance, excessive height, missing support and excessive tilt still fail it.

## MuJoCo contact materials

Wheel collision geoms inherit `friction="1.0 0.005 0.0001"` from
`rubber_wheel`. Generic robot collision friction must not override that class
or become the default material of an imported world. The flat ground and
stairs use MuJoCo's default friction; an external world owns its own material
settings.

The failure mechanism, regression checks, and upstream comparison are recorded
in [MUJOCO_CONTACT_FRICTION.md](MUJOCO_CONTACT_FRICTION.md).

## Baseline locomotion policy

`sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_4998.onnx` is the
default MuJoCo locomotion policy, imported from
`thunder_flat_s4_lateral_g2_v2_model_4998.zip/exported/policy.onnx`.
The `thunderv4_flat53` adapter consumes one 53-value frame (`obs[batch,53]`):
body angular velocity, projected gravity, `[vx, vy, wz]`, 12 relative leg
positions, 16 joint velocities, and 16 previous raw actions. It produces 16
Dart-ordered actions with hip/thigh-calf/wheel scales `0.125/0.25/5.0`.
The training reference uses hip/thigh/calf magnitudes `0.20/0.93/1.96`, leg
Kp/Kd `90/6.93`, and wheel Kd `1`. The simulation keeps its 200 Hz low-level
loop, 50 Hz inference rate, 0.5 s startup hold, and physical torque limits.

This G2 model failed the supplied fixed evaluation (lateral contact peaks
616.53/597.57 N). Its activation is **simulation only**, not hardware
qualification. The original report is retained as
`sim/packages/controllers/doso/thunder_v4/locomotion/policy/model_4998_DEPLOYMENT.md`.
`policy_1119.onnx` and its manifest remain available for explicit legacy
comparison; select `quadruped_him` with that model's 5 x 57 observation.

`thunderv4_stairs.xml` is generated from the same V4 robot source as the flat
scene. It only adds three stairs to the world; it must not use an older robot
asset.

Important compatibility note:

The V86 MUJICA recovery policy was trained in IsaacLab with `THUNDER_CFG_V4`
and its Thunder V4 URDF, not with an older small-wheel asset.  Its separate
MuJoCo player uses the same V4 joint names, default pose, action scales, PD
gains, torque limits, and velocity limits.  This is therefore an
IsaacLab-to-MuJoCo transfer check, not an asset substitution.  Policies whose
training asset cannot be identified still need an explicit asset-contract
check before their rollout quality is trusted.

The keyboard player validates joint and actuator names at startup. Its legacy
policy observation reference remains unchanged, while its manual high-stand
pose now stays within the real V4 joint limits. It reads the velocity limits
embedded in the MJCF and fades same-direction drive torque to zero from 90% of
the hardware speed limit; this avoids treating a bare MuJoCo motor as an
unbounded-speed actuator.

For the V86 MUJICA recovery policy's exact IsaacLab-to-MuJoCo control and
observation contract, see `SIM2SIM_V86_RECOVERY.md`.
