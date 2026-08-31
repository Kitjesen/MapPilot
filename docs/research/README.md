# LingTu Research Notes

Status: non-authoritative research index
Audience: algorithm, portability, and upstream-adoption maintainers

This directory contains research snapshots, upstream evaluations, and detailed
migration coverage. These files support engineering decisions but do not define
the shipped runtime. Current behavior lives in `../architecture/`; active work
lives in `../plans/current-roadmap.md`; acceptance evidence lives in
`../07-testing/`.

| Document | Purpose |
| --- | --- |
| [`dimos_benchmark_matrix.md`](./dimos_benchmark_matrix.md) | DimOS-inspired capability and evidence comparison. |
| [`localization_relocalization_research_repos.md`](./localization_relocalization_research_repos.md) | Reviewed local relocalization research clones and licenses. |
| [`navigation_research_repos.md`](./navigation_research_repos.md) | Reviewed navigation, mapping, and traversability research clones. |
| [`octoplanner3d_multifloor_occupancy.md`](./octoplanner3d_multifloor_occupancy.md) | Multi-floor execution limitations around OctoPlanner3D. |
| [`octoplanner3d_pcd_octomap_mujoco.md`](./octoplanner3d_pcd_octomap_mujoco.md) | PCD/OctoMap versus MuJoCo map-gap investigation. |
| [`perception_sota_evaluation.md`](./perception_sota_evaluation.md) | Dated perception-stack comparison. |
| [`socc_icp_adoption.md`](./socc_icp_adoption.md) | SOCC-ICP adoption record and remaining evidence gates. |
| [`visual_person_following.md`](./visual_person_following.md) | Verified person-following papers, code availability, and LingTu execution plan. |

## Rules

- Record upstream URL, reviewed commit, and license before reusing code.
- State the observation date for claims that can change upstream.
- Never cite a research note as Product, simulation, or field acceptance.
- Delete a note when its useful decisions have moved into a current contract;
  Git history is the archive.
