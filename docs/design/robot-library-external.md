# External robot source acquisition

Status: source assets acquired and compile-probed on 2026-10-16. This is an
intake record, not an active robot manifest, controller binding, locomotion
qualification, or catalog promotion.

## Downloaded sources

| Robot | Source and pinned revision | Local source | License retained | MuJoCo source entry |
| --- | --- | --- | --- | --- |
| ANYmal D | [ANYbotics `anymal_d_simple_description`](https://github.com/ANYbotics/anymal_d_simple_description) at `d5bf4dc3a81af81a16b9598fc15c98e339f45a33` | `sim/.imports/external_robots/anymal_d_simple_description` (complete shallow clone, 44 files) | BSD-3-Clause | `urdf/anymal.urdf` |
| Unitree H1 | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/unitree_h1) at `62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35` | `sim/.imports/external_robots/mujoco_menagerie/unitree_h1` (27 files) | BSD-3-Clause | `h1.xml` |
| Franka Emika Panda | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda) at the same revision | `sim/.imports/external_robots/mujoco_menagerie/franka_emika_panda` (80 files) | Apache-2.0 | `panda.xml` |
| Universal Robots UR5e | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/universal_robots_ur5e) at the same revision | `sim/.imports/external_robots/mujoco_menagerie/universal_robots_ur5e` (26 files) | BSD-3-Clause | `ur5e.xml` |
| Boston Dynamics Spot | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/boston_dynamics_spot) at the same revision | `sim/.imports/external_robots/mujoco_menagerie/boston_dynamics_spot` (62 files) | BSD-3-Clause | `spot.xml` |

The Menagerie checkout is shallow and sparse, but each selected model directory
matches its complete Git tree file count. Each model's `LICENSE`, `README.md`,
`CHANGELOG.md`, XML, and referenced asset files are present. None of these four
model directories has a separate `NOTICE` file; no notice file was omitted.

## Compile probe

The repository's existing `.venv` supplied MuJoCo 3.10.0. Each exact entry
above was loaded with `mujoco.MjModel.from_xml_path`. The probe did not create
`MjData`, call forward, or step simulation.

| Robot | Result | Bodies | Joints | `nq` / `nv` / `nu` | Geoms / meshes |
| --- | --- | ---: | ---: | --- | --- |
| ANYmal D simplified URDF | pass | 15 | 14 | 14 / 14 / 0 | 44 / 0 |
| Unitree H1 | pass | 21 | 20 | 26 / 25 / 19 | 53 / 21 |
| Franka Panda | pass | 12 | 9 | 9 / 9 / 8 | 81 / 67 |
| UR5e | pass | 8 | 6 | 6 / 6 / 6 | 29 / 20 |
| Spot | pass | 14 | 13 | 19 / 18 / 12 | 31 / 23 |

Compile success proves that MuJoCo can parse the model and resolve the retained
assets; it does not prove dynamics fidelity, collision quality, control tuning,
locomotion, or compatibility with a LingTu controller.

## Explicitly unavailable or limited

- RobotLab's ANYmal D configuration points to Isaac Lab's cloud/Nucleus
  `Robots/ANYbotics/ANYmal-D/anymal_d.usd`. That login/cloud USD was not fetched.
  The downloaded ANYbotics repository is the correct ANYmal D generation, not
  ANYmal C, but it is the manufacturer's public **simplified** description; its
  README states that the extended description and control software are limited
  to the ANYmal Research community.
- The unmodified ANYmal D URDF compiles with collision geometry, but MuJoCo's
  default URDF import discards its visual-only DAE meshes (`nmesh == 0`) and the
  URDF defines no actuators (`nu == 0`). The DAE/JPG source assets remain present
  for a later, explicit importer conversion; no such migration was performed.
- RobotLab's Isaac-cloud H1 reference was not fetched. The acquired H1 is the
  exact Unitree H1 model distributed by MuJoCo Menagerie under BSD-3-Clause, not
  a claim that it is byte-equivalent to Isaac Lab's USD.

Machine-readable acquisition and compile evidence is in
`build/robot-library-extra-20261016/external-robot-acquisition.json`.
RobotLab's local clone and inventory were not modified.
