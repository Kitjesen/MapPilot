# Unitree Go2 official visual assets

Source: [unitreerobotics/unitree_ros, robots/go2_description](https://github.com/unitreerobotics/unitree_ros/tree/7d6075f7f58588b189b940130e3edab3c839b2df/robots/go2_description).

Pinned upstream commit: `7d6075f7f58588b189b940130e3edab3c839b2df` (retrieved 2026-09-11).
The URDF, seven DAE files, package.xml and LICENSE are unmodified upstream files.
They are distributed under the included **BSD 3-Clause license**, copyright Unitree Robotics.

The viewer preserves all URDF links, joint origins, axes, and mesh transforms at
their original metre scale. The model root is URDF `base`, placed at the published
LingTu `body` pose; no standing-height offset is added to localization Z.
The Z-up URDF tree is converted to the Three.js Y-up scene once. ColladaLoader's
automatic Z-up conversion is removed from each DAE wrapper before assembly.

The scene currently receives body pose, not joint telemetry. It therefore uses
an explicitly nominal display stance for all four legs: hip `0`, thigh `0.8`,
calf `-1.6` radians. These angles are not measured joint positions. No walk cycle
or velocity-driven leg animation is generated. The official meshes do not
include the optional external MID-360 payload or expansion computer.

For a neutral black/white display, the original mesh material segmentation and
brightness are retained while color saturation is removed. Geometry is unchanged.
Repeated meshes are loaded once per model and shared by the corresponding links.

`web/scripts/generate-go2-urdf.py` extracts the small visual/kinematic declaration
used by the browser; run it again after changing the official URDF. Use `--check`
to check that the generated declaration matches the supplied source.
