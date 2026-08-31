# Thunder V4 MuJoCo Asset

This directory targets the real Thunder V4 small-wheel hardware asset.

Source of truth:

- Upstream repo: `Kitjesen/thunder_assets`
- Default local source URDF: `thirdpart/thunder_assets/thunder_v4/urdf/thunder_v4.urdf`
- Default local source meshes: `thirdpart/thunder_assets/thunder_v4/meshes`
- Local MJCF: `mjcf/thunderv4.xml`

Regenerate:

```powershell
python D:\inovxio\brain\lingtu\sim\robots\doso\thunder_v4\tools\generate_thunderv4_mjcf.py
python D:\inovxio\brain\lingtu\sim\robots\doso\thunder_v4\tools\generate_thunderv4_mjcf.py --scene stairs
python D:\inovxio\brain\lingtu\sim\robots\doso\thunder_v4\tools\check_thunderv4_hardware.py
```

Collision primitives are physically enabled but transparent by default. For a
geometry-debug XML, add `--show-collisions` to either generator command.

Generator guarantees:

- total modeled mass: `45.8086 kg`
- free floating `base_link`
- RobotLab-compatible joint names and policy action order
- actuator order: 12 leg joints first, then 4 wheel joints
- leg actuator limit: `120 Nm`
- wheel actuator limit: `17 Nm`
- leg speed limit: `17.48 rad/s`; wheel speed limit: `44 rad/s`
- V4 small-wheel collision radius: `0.093 m`
- `v4_nominal_stand` keyframe: a valid V4 standing reference with no joint-limit target

## Baseline locomotion policy

`sim/controllers/doso/thunder_v4/locomotion/policy/policy_1119.onnx` is the
default MuJoCo locomotion policy. It consumes
five history frames of the 57-value Brainstem observation (`obs_history[1,285]`)
and produces 16 Dart-ordered actions. The controller package owns its 200 Hz
low-level loop, 50 Hz inference rate, 0.5 s startup standing hold, direct
`[vx, vy, wz]` observation command, action scaling, PD gains, and manifest;
simulation entrypoints must not silently fall back to a different checkpoint.

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
