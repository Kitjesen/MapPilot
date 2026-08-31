# Thunder v3 Assets

These files are synced from `Kitjesen/thunder_v3_assets` at commit
`1e6a0530e710208ad375c0bae7df5d6fa8d6ec6a` (2026-05-04).

Canonical upstream-compatible assets:

- `urdf/thunder_v3.urdf`
- `xml/thunder_v3.xml`
- `mjcf/thunder_v3_mujoco.xml`
- `meshes/*.STL`

LingTu runtime adapter:

- `mjcf/thunder_v3_lingtu.xml`

The runtime adapter keeps the latest Thunder v3 geometry, inertial parameters,
joint names, and wheel masses, while adding the `front_camera` and `lidar_link`
interfaces expected by the simulation stack. It also uses position actuators so
the existing LingTu policy and PD-control path can keep writing joint targets.

These are source/reference assets. The runnable Thunder V4 robot lives under
`sim/robots/doso/thunder_v4/`; its locomotion policy lives under
`sim/controllers/doso/thunder_v4/locomotion/`.
