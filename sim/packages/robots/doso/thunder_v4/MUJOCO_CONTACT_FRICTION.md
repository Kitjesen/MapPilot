# Thunder V4 MuJoCo contact friction

## Failure and correction

The generator annotated every robot collision geom with
`friction="0.9 0.2 0.2"`. Adding `class="rubber_wheel"` afterwards did not
replace those explicit attributes, so the intended wheel friction
`1.0 0.005 0.0001` never took effect. The same generic friction in the global
geom default also changed the material of a world merged with the robot.
The generated stairs repeated the generic friction explicitly.

MuJoCo resolves explicit geom attributes before class defaults. For contacts
between geoms of equal priority, it combines friction coefficients by taking
the componentwise maximum. Correcting only the wheel therefore leaves the
high torsional coefficient from the ground in the contact. See MuJoCo's
[default settings](https://mujoco.readthedocs.io/en/stable/modeling.html#default-settings)
and [contact parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters).

The correction removes the explicit friction override when a collision is
classified as a wheel, removes generic friction from the global geom default,
and removes it from generated stairs. Both checked-in MJCF files carry the
same correction. Other robot collision materials retain their existing values.

The second geom friction coefficient controls torsional friction about the
contact normal. The tested wheel/ground contact has `condim=4`, which includes
torsion; rolling friction requires `condim=6`. Changing `0.2` to `0.005` reduces
the coefficient by a factor of 40, not necessarily the resulting torque by 40.
See [contact dimensions](https://mujoco.readthedocs.io/en/stable/computation/index.html#condim).

## Regression evidence

From the repository root, with its existing MuJoCo simulation dependencies:

```sh
python -m pytest tests/sim/test_thunderv4_mjcf_hardware.py -k "hardware_contract or friction or torsion" -q
python -m ruff check sim/packages/robots/doso/thunder_v4/tools/generate_thunderv4_mjcf.py tests/sim/test_thunderv4_mjcf_hardware.py
```

The three selected tests check compiled wheel/ground/stair materials, the
generator's collision annotation path, and the actual wheel contacts after
the runtime merges the open-field world. All three failed on GitHub main
`348b063a` and passed after this correction on 2026-09-08. The two keyboard
tests are outside this focused selection.

The isolated patch branch also passed all five checks in the repository's
default policy 1119 qualification:

```sh
PYTHONPATH=src python -m sim.scripts.mujoco.continuous_walk --json-out artifacts/contact-friction/policy-1119-fixed.json
```

The full `6 s` drive-phase mean forward speed was `0.5343 m/s`, release
displacement was `0.0792 m`, and release final speed was `0.00059 m/s`.
This is a headless MuJoCo policy/physics component result, without navigation
or hardware. The script also reports an active-phase speed that subtracts its
startup hold; the speed quoted here uses the entire drive phase.

A separate local diagnostic with policy 4998, MuJoCo 3.10.0, a `0.6 m/s`
forward command, `0.5 s` warmup, `6 s` drive, and `2 s` release produced:

| Metric | Original contact materials | Corrected materials |
| --- | ---: | ---: |
| Full drive-phase world-forward mean speed | 0.3188 m/s | 0.5861 m/s |
| End yaw | 45.54 degrees | 0.86 degrees |
| Lateral displacement | 0.8137 m | 0.0312 m |
| Release final speed | 0.0069 m/s | 0.0380 m/s |

A control experiment changing only torsional friction to `0.005` reached
`0.5863 m/s` with `0.83 degrees` end yaw, supporting torsional friction as the
cause of this failure. These figures describe the separate local 4998
diagnostic, whose checkpoint and controller adaptation are not part of this
patch. They do not qualify the repository's default policy, a full navigation
Product, or physical hardware.

## Driver timing follow-up

The first GitHub CI run exposed a second simulation configuration mismatch:
the Host's `MujocoDriverModule` retained the scene's `0.002 s` physics step,
while the live runtime factory applied policy 1119's declared `0.005 s` step.
After the contact correction, the old driver failed the right-lateral motion
gate. Both entrypoints now share the existing policy timing resolution and
apply it before reset. Explicit runtime timestep overrides retain precedence;
kinematic mode and other policies retain their existing scene timing.

With the same corrected asset, seed 7, 200 control steps, and `vy=-0.2 m/s`,
a timestep-only diagnostic measured `-0.1822 m` lateral displacement at
`0.002 s` and `-0.7087 m` at `0.005 s`. Both left and right motion then passed
the existing distance and forward-drift thresholds. No threshold was relaxed.

Two unrelated CI assertions were also repaired: the navigation source check
now follows the existing throttled telemetry helper, and the same-content
file replacement fixture retains the original inode so a filesystem cannot
reuse it during the test. These corrections change test setup/assertions only.

## Upstream ownership and geometry

Reviewed upstream revision:
[`Kitjesen/thunder_assets@8b535d0`](https://github.com/Kitjesen/thunder_assets/tree/8b535d0f885371cc303f7a08056a7c6650cfa9d7).

- The V4 URDF sets no MuJoCo contact friction and the repository has no V4
  MJCF. The faulty friction assignment belongs to LingTu's converter.
- The existing V3 MJCF compiles with wheel friction `1.0 0.005 0.0001`.
  It does not have this explicit friction override. This compilation check
  does not establish V3 policy performance.
- Generating flat and stairs models from that upstream V4 revision with the
  corrected converter passed the generator's hardware checks and compiled
  wheel, ground, and stair friction checks under MuJoCo 3.10.0.
- That upstream URDF uses `0.095 m` wheel collision radii. The checked-in
  LingTu MJCF and the local 4998 training URDF use `0.093 m`. The converter
  preserves whichever source radius is selected; it does not hardcode
  `0.093 m`.

No upstream friction patch is required for this defect. Neither changing an
upstream physical dimension nor adding simulator-specific friction to its
URDF would correct the downstream override. This patch retains the current
LingTu geometry; replacing it with a newer source asset requires a separate
policy/asset qualification.
