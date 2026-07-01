# Simulation Comparison Notes

This note compares LingTu's current MuJoCo/ROS2 validation loop with common
robotics simulation patterns. It is intentionally focused on navigation
readiness: localization, global planning, local obstacle avoidance, velocity
safety, and evidence artifacts.

## External Patterns

| Reference | What They Simulate | Evidence Pattern | Takeaway For LingTu |
| --- | --- | --- | --- |
| Nav2 + Gazebo | Robot model, transforms, odometry, sensors, mapping/localization, footprint, planner/controller plugins | System tests bring up Gazebo and make a robot complete navigation tasks while tracking module results | Treat simulator runs as CI gates, not demos. Each planner/controller result needs explicit pass/fail metrics. |
| Isaac Sim + Nav2 | ROS2 bridge publishes `/tf`, `/odom`, `/map`, `/point_cloud`, and `/scan`; occupancy maps are generated from the scene | Goals are sent through Nav2/RViz or programmatic goal packages; map and frame setup are explicit | Our MuJoCo scenes should generate both visual geometry and planner maps from the same metadata, then validate frame contracts. |
| CMU exploration stack | Simulation environments plus local planner, terrain traversability, waypoint following, and visualization | Local planner removes motion primitives occluded by obstacles and selects collision-free local paths | We should not trust global paths alone. Local planner output and obstacle occlusion decisions need their own overlays and metrics. |
| Unitree MuJoCo | Low-level SDK-compatible MuJoCo simulator, isolated DDS domain, loopback interface, terrain tools | Same controller can run against simulation or real robot by switching domain/interface | Keep simulation network/domain isolated from real robot. Preserve a clear sim-to-real switch boundary. |
| Legged Gym / Isaac Lab style locomotion | Massive rough-terrain locomotion training with domain randomization and noisy observations | Sim-to-real confidence comes from randomized friction, mass, observation noise, and pushes | Our policy/gait gate should remain separate from navigation gates and should add disturbance/randomization before claiming gait robustness. |

## Current LingTu Position

LingTu already has the right basic shape:

| Layer | Current State |
| --- | --- |
| Physics/rendering | MuJoCo scenes, ray-cast MID-360-style LiDAR, RGB camera, omni-cart validation vehicle, quadruped policy mode |
| ROS2 algorithm loop | `native_pct_mujoco_gate.py` starts native `localPlanner` and `pathFollower` in isolated ROS domains |
| Global planning assets | `large_terrain_nav_validation.py` generates scene XML, tomogram, map point cloud, route catalog, and per-planner reports |
| Safety evidence | Reports include `path_safety`, `obstacle_clearance`, `trajectory_quality`, hardware-safe flags, and video artifacts |
| Recent finding | On `terrain_complex_slalom`, native PCT can produce a path, but its path samples enter high-cost obstacle cells; A* is safe on the same scene |

## Gaps Against Mature Practice

| Gap | Why It Matters | Concrete Fix |
| --- | --- | --- |
| Planner safety still needs more scenario coverage | PCT can output a path that looks smooth but cuts into inflated obstacles; runtime now checks this but the matrix is still small | Expand seeded validation scenarios and keep `path_safety` as a pass/fail gate in every planner report. |
| PCT repair is still missing after unsafe smoothing | A native planner being available is not enough; the selected route must also be safe | Runtime now falls back to A* on the same active map when PCT fails `path_safety`; add corridor repair before fallback as the next improvement. |
| Local planner evidence is still mostly visual/log based | Collision-free local decisions should be measured, not inferred from video | Persist local path samples, local path clearance, near-field stop count, and slow-down count in the JSON report. |
| Frame/topic contracts are embedded in scripts | Mature stacks make `/tf`, odom, map, scan/cloud, and cmd frames explicit contracts | Add a `sim/contracts.py` or test helper that asserts frames and topic direction for each simulation gate. |
| Camera and LiDAR overlays are useful but not product-grade diagnostics | User needs to see why the robot chose a route, not only where it moved | Add a stable single-scene overlay legend and export separate `global_path`, `local_path`, `trail`, `point_cloud`, and `obstacle_margin` tracks. |
| Gait/policy confidence is mixed with navigation confidence | Kinematic omni-cart proves planning/tracking; it does not prove quadruped contact robustness | Keep `navigation_kinematic`, `navigation_policy`, and `real_robot_dry_run` as separate gates with separate pass criteria. |
| Scenario difficulty is hand-tuned | Mature validation uses repeated generated maps, randomized obstacles, and regression thresholds | Add deterministic scenario seeds and a small matrix: open, corridor, slalom, narrow gate, U-shape trap, dynamic blocker. |

## Recommended Next Implementation Order

1. Introduce a reusable plan safety contract.
   - Inputs: route path, tomogram/costmap, robot radius, inflation margin, route metadata.
   - Outputs: `ok`, `max_cost`, `blocked_sample_count`, `min_clearance_m`, `blocked_samples`, `repair_required`.
   - Use it in `large_terrain_nav_validation.py`, `native_pct_mujoco_gate.py`, and runtime planner dispatch.
   - Current status: `src/nav/services/safety/plan_safety.py` now owns shared grid/path sampling,
     tomogram/backend axis handling, and path safety reports. Simulation gates
     and `GlobalPlannerService` use this shared contract.

2. Reject unsafe global plans instead of downgrading planners.
   - `pct_strict`: fail if explicit PCT output is unsafe.
   - `octoplanner3d_strict`: fail if OctoPlanner3D output is unsafe.
   - Report fields: `selected_planner`, `fallback_reason`, `rejected_plans`.
   - Current status: `GlobalPlannerService` supports explicit
     `plan_safety_policy`: `off`, `observe`, and `reject`.
     `NavigationModule` exposes the policy. Product profiles use `reject`, so
     unsafe output is rejected instead of silently switching to another global
     planner.

3. Upgrade the native local planner gate.
   - Count `/path` updates, near-field estop events, slowdown events, local path min clearance, and local path-to-global progress.
   - Fail if local path repeatedly points into an obstacle or if progress stalls despite nonzero commands.

4. Add a scenario matrix.
   - `open_field`, `long_route`, `narrow_gate`, `complex_slalom`, `u_shape_trap`, `dynamic_crossing`.
   - Each scenario should produce the same JSON schema and optional video.

5. Keep visualization as evidence, not as the gate.
   - Video should show global path, local path, actual trail, point cloud, current target, and unsafe/rejected path segments.
   - JSON remains the source of truth.

## Sources Checked

- Nav2 Getting Started and Gazebo setup docs: simulator launch brings up Nav2, AMCL, robot state publisher, Gazebo, and RViz together.
- Nav2 system tests docs: full-system tests spin up Gazebo tasks, random planning checks, lifecycle bringup/down, keepout/speed-zone behavior, and failure recovery evidence.
- Nav2 concepts and plugin docs: planners, controllers, smoothers, route servers, and behavior servers are separate plugin-backed contracts.
- Nav2 Collision Monitor docs: velocity safety is a final `cmd_vel` filter and should sit below navigation.
- Isaac Sim ROS2 Navigation docs: simulation must publish explicit `/tf`, `/odom`, `/map`, `/point_cloud`, and `/scan` contracts, with occupancy maps generated from the scene.
- CMU exploration environment docs/paper: simulation includes environment models, terrain traversability, local planner, waypoint following, and visualization; local planner selects collision-free motion primitives.
- Unitree MuJoCo docs: simulator mirrors SDK/DDS interfaces and separates simulation from real robot by domain/interface.
- Legged Gym docs: locomotion sim-to-real confidence depends on randomization and disturbance, separate from navigation-route correctness.
