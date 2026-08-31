# Livox Scan Patterns

This directory stores LiDAR scan patterns that are treated as simulation input
assets, not generated test fixtures.

## `mid360.npy`

- Source: official Livox Gazebo plugin
  `Livox-SDK/livox_laser_simulation/scan_mode/mid360.csv`
- Source commit observed during asset import:
  `1cce1073633a062b92e30243a4c2920e45551bb5`
- Shape: `(800000, 2)`
- Dtype: `float32`
- Columns: `theta_rad, phi_rad`
- Conversion:
  - `theta_rad = radians(Azimuth/deg)`
  - `phi_rad = radians(90 - Zenith/deg)`
- SHA256:
  `448821576a658673e8f7929992c8c0d687eb052657d7b584d038729a83da1bfb`

MuJoCo gates use this asset by default. The legacy golden-spiral scan can only
be enabled explicitly with `--allow-golden-spiral-lidar`.

See `docs/07-testing/simulation/MUJOCO_MID360_FIDELITY.md` for the sensor-fidelity and
acceptance-boundary contract.
