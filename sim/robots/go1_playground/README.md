# Go1 Playground Assets

This directory is a placeholder for optional external assets used by the legacy
Go1 demos and `sim/worlds/mujoco/indoor_office.xml`.

The expected external files are:

- `xmls/go1_mjx_feetonly.xml`
- `xmls/sensor_feet.xml`
- `go1_policy.onnx`

`go1_policy.onnx` is an ignored local asset. If a cleanup pass finds this file
under an ad-hoc directory such as `sim/robots/go2/`, move it back here so the
legacy `sim/scripts/go1_indoor_nav.py` path contract remains correct.

These assets are not part of the G4 server closure and are not required for
the current LingTu Thunder/NOVA simulation gates. Keep this directory as an
asset contract marker only; do not treat missing Go1 assets as a failure of the
server-side full-simulation evidence path.
