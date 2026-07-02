# LingTu Plans

This directory is for forward-looking product and architecture plans. A plan is
not a source of truth until the implementation and tests land.

## Current Plans

| Document | Scope |
| --- | --- |
| [`PRD-lingtu-enterprise-runtime-profiles.md`](./PRD-lingtu-enterprise-runtime-profiles.md) | Profile and runtime resolver productization. |
| [`PRD-lingtu-native-slam-navigation-runtime.md`](./PRD-lingtu-native-slam-navigation-runtime.md) | Native SLAM/navigation runtime direction. |
| [`PRD-map-bundle-global-planning-integration.md`](./PRD-map-bundle-global-planning-integration.md) | MapRecord bundle handoff into global planning. |
| [`PRD-map-service-spatial-data-platform.md`](./PRD-map-service-spatial-data-platform.md) | MapService records, bundles, lifecycle events, and planner handoff. |
| [`PRD-slam-transport-navigation-dataflow.md`](./PRD-slam-transport-navigation-dataflow.md) | SLAM, transport, and navigation dataflow. |
| [`native-dds-map-planning-test-migration-plan.md`](./native-dds-map-planning-test-migration-plan.md) | Native DDS, pcd_to_octomap, global/local planning test and migration gates. |
| [`octoplanner3d-map-artifact-closed-loop.md`](./octoplanner3d-map-artifact-closed-loop.md) | OctoPlanner3D map artifact loop. |
| [`simulation-closure-plan.md`](./simulation-closure-plan.md) | Simulation validation closure plan. |

## Rules

- Put shipped architecture in `../architecture/`.
- Put operator commands in `../04-deployment/` or `../QUICKSTART.md`.
- Move completed or obsolete plans to `../archive/` when a current contract
  replaces them.
- Every new PRD should include problem, scope, non-goals, interfaces, acceptance
  checks, and migration risks.
