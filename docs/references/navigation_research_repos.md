# Navigation Research Repositories

Local clone path: `third_party/research_nav/`.

`third_party/` is intentionally ignored by Git, so these clones are local
reference material, not LingTu runtime code.

| Directory | Upstream | Commit | Use |
| --- | --- | --- | --- |
| `traversability_estimation/` | `https://github.com/leggedrobotics/traversability_estimation` | `14d24c059e1c43466aadf328280adf6394d78039` | Traversability map and footprint/path checking reference. |
| `elevation_mapping/` | `https://github.com/ANYbotics/elevation_mapping` | `f4b082c64a3e660980da53b33c7936a8f2a2ea22` | Robot-centric elevation map reference. |
| `voxblox/` | `https://github.com/ethz-asl/voxblox` | `c8066b04075d2fee509de295346b1c0b788c4f38` | TSDF/ESDF map and planner distance-query reference. |
| `ground_based_autonomy_basic/` | `https://github.com/jizhang-cmu/ground_based_autonomy_basic` | `7ae94b72206430a399ae012f49715cf51fadb0e0` | CMU terrain analysis, local planner, and smart joystick reference. |

Do not import these repositories from `src/`. Extract only reviewed,
license-compatible ideas into LingTu contracts.

Immediate LingTu direction:

```text
Terrain.traversability
  -> hard-risk cells as virtual obstacle points
  -> soft-risk cells as candidate path score penalties
  -> LocalPlanner nanobind and cmu_py backends avoid unsafe terrain
```
