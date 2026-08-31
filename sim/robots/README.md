# Simulation Robots

This directory contains robot bodies only: package metadata, MJCF, meshes,
visual projection, frames, and model tools.

- `doso/thunder_v4/` is the supported Thunder V4 simulation body.
- `omni_cart/` is the small differential-drive test body.
- Controllers live under `sim/controllers/`.
- Reusable sensors live under `sim/sensors/`.
- Robot-specific sensor mounting lives under `sim/sensor_rigs/`.
- Product simulation presets live under `sim/presets/`.

Do not add an empty robot directory. A robot enters simulation only when its
body, controller, sensor rig, and at least one preset are usable together.
