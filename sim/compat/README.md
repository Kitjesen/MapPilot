# Simulation Compatibility

`sim/compat/` keeps the direct Python simulation engine, fallback sensor code,
and reference assets needed by existing compatibility paths.

## Contents

- `assets/`: Thunder V3 reference MJCF, URDF, and mesh assets.
- `engine/`: the legacy in-process Python engine and its direct-engine worlds.
- `sensors/`: compatibility sensor implementations, including the legacy
  Mid-360 adapter.

## Boundary

This directory is not a Catalog root and does not define the generic runtime.
New packages belong in `sim/packages/`; new runtime behavior belongs in
`sim/runtime/`. Add compatibility code here only when an existing supported
path still requires it.
