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
| `ERASOR2/` | `https://github.com/url-kaist/ERASOR2.git` | `d43d94f7e06a900456042979e29c3c933a39fd48` | GPLv3 reference for save-time static-map ghost removal. LingTu migration starts with an owned C++ staging adapter under `src/maps/prune/cpp`; do not include upstream ERASOR2 headers/source in product runtime targets unless the GPLv3 boundary is explicitly accepted. |

Do not import these repositories from `src/`. Extract only reviewed,
license-compatible ideas into LingTu contracts.

Immediate LingTu direction:

```text
Terrain.traversability
  -> hard-risk cells as virtual obstacle points
  -> soft-risk cells as candidate path score penalties
  -> LocalPlanner nanobind and cmu_py backends avoid unsafe terrain
```

## Upstream Training and Building-Navigation References

These are design/training references, not runtime dependencies:

| Upstream | What it demonstrates | LingTu use |
| --- | --- | --- |
| [CMU Ground Based Autonomy](https://github.com/jizhang-cmu/ground_based_autonomy_basic) | Offline path library plus online registered-cloud/terrain scoring and separate RViz layers. The local planner is not a learned model. | Keep raw cloud, admitted obstacles, terrain, candidates, and selected path as distinct observability layers. |
| [Isaac Lab velocity locomotion tasks](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity) | Parallel rough-terrain locomotion training with height scans, noise, curriculum, mass/CoM randomization, pushes, and tracking/smoothness/contact rewards. | Train or harden the ThunderV4 locomotion ONNX independently from geometric local planning. |
| [legged_gym](https://github.com/leggedrobotics/legged_gym) | Earlier GPU legged-locomotion environment; upstream now points new work toward Isaac Lab. | Historical baseline only; do not build a new training pipeline around it. |
| [ViPlanner](https://github.com/leggedrobotics/viplanner) | Offline training of a visual/semantic local path predictor from depth and semantic cost maps. | Optional research lane for a learned semantic local planner; export inference artifacts rather than importing its CUDA/ROS runtime into S100P. |
| [Hydra](https://github.com/MIT-SPARK/Hydra) | Real-time hierarchical 3D scene graph with objects, places, rooms, and buildings. | Model the semantic hierarchy and stable spatial IDs, not its ROS runtime. |
| [Clio](https://github.com/MIT-SPARK/Clio) | Task-driven open-set 3D scene graphs and natural-language relevance. | Rank semantic objects/regions for a mission after geometric and floor constraints are satisfied. |
| [Open-RMF demos](https://github.com/open-rmf/rmf_demos) | Level/lift graphs and cross-level task patterns in multi-floor facilities. | Reference the per-level map plus typed portal/lift topology; LingTu still needs its own quadruped connector executor. |

The training boundaries remain separate:

```text
locomotion policy training  -> joint actions / body stability
geometric local planner     -> deterministic path-library collision + terrain score
optional semantic planner   -> learned or rule-based target/cost prior
```

Do not retrain the local planner merely because the locomotion policy is ONNX;
they solve different functions and have different acceptance gates.
