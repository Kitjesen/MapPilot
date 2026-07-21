# OctoPlanner3D Backend

This directory is the complete product-owned integration boundary for the
default 3D global planner.

```text
navd
  -> GlobalPlanRequest / GlobalPlanResult
  -> octoplanner3d_runtime
  -> vendor/planner
  -> system liboctomap
```

`vendor/` contains only the four OctoPlanner3D files used by the product:
the constrained planner header/source and the optional offline PCD converter
header/source. The ROS2/RViz wrapper, duplicate runtime, sample assets, build
directories, and precompiled libraries are not product inputs.

The online `navd` path reads an immutable, Maps-validated `octomap.ot` snapshot
and does not link PCL. PCL is limited to the standalone offline
`octoplanner3d_pcd_to_octomap` tool. OctoMap must come from the target system
(`liboctomap-dev`); checked-in `.so` files are intentionally rejected so an
x86_64 binary cannot leak into an aarch64 release.

The vendored code derives from
[JackJu-HIT/OctoPlanner3D](https://github.com/JackJu-HIT/OctoPlanner3D), commit
`9a9cc431ea905a5878975cc6fbbce6c9618b31a4`, under the MIT license. LingTu's
copy includes robot-envelope, ground-support, floor-continuity, step/slope,
terminal-tolerance, and cancellation changes. See `LICENSE`.

FAR is a separate optional 2D global planner under `../far`; it is not part of
this vendor tree and does not replace OctoPlanner3D as the product default.
