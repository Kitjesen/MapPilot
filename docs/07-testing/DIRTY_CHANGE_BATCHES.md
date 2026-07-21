# Dirty Change Verification Batches

This runbook turns the current dirty worktree into reviewable product batches.
One batch means one user-visible outcome, one focused test set, and one commit.

## Rules

- Stage only the listed paths for the active batch.
- Stop on the first red test or field gate.
- Do not mix docs, UI, simulation assets, and robot runtime changes in one commit.
- Do not restart robot services unless the batch says so.
- Save field evidence under `docs/07-testing/field-runs/<date>-<batch>/`.

Preflight for every batch:

```bash
git status --short
git diff --cached --name-only
```

Push check for every batch:

```bash
SKIP_HEAVY_TESTS=true git push --dry-run origin codex/runtime-boundary-cleanup
```

## Batch 1 - Runtime And Deployment Contract

Outcome: Thunder runtime services, endpoint policy, evidence collection, and
service manager agree on one product runtime shape.

Stage:

```bash
git add \
  cli/logging_util.py \
  cli/runtime_audit.py \
  config/robots/thunder.yaml \
  config/topic_contract.yaml \
  scripts/deploy/cut_release.sh \
  scripts/deploy/deploy_thunder.sh \
  scripts/deploy/s100p/install_services.sh \
  scripts/deploy/sync_sunrise.ps1 \
  scripts/deploy/thunder/install_services.sh \
  scripts/deploy/thunder/install_lingtu_service.sh \
  scripts/deploy/thunder/lingtu.service \
  scripts/deploy/thunder/lingtu-livox-dds.service \
  scripts/deploy/thunder/lingtu-nav-dds.service \
  scripts/deploy/thunder/lingtu-slam-dds.service \
  scripts/deploy/thunder/run_slam_dds.sh \
  scripts/gates/real_runtime_evidence_collect.py \
  scripts/ota/setup_robot.sh \
  src/runtime/adapters/dds/endpoint_runner.py \
  src/runtime/adapters/lcm/endpoint_runner.py \
  src/runtime/adapters/lcm/sources/__init__.py \
  src/runtime/adapters/lcm/sources/brainstem.py \
  src/runtime/blueprints/full_stack_wiring.py \
  src/runtime/blueprints/products/thunder.py \
  src/runtime/blueprints/profile_graph.py \
  src/runtime/blueprints/stacks/autonomy_chain.py \
  src/runtime/blueprints/stacks/composition.py \
  src/runtime/blueprints/stacks/navigation_core.py \
  src/runtime/blueprints/stacks/safety.py \
  src/runtime/blueprints/wires/context.py \
  src/runtime/blueprints/wires/mapping.py \
  src/runtime/blueprints/wires/safety.py \
  src/runtime/blueprints/wires/slam.py \
  src/runtime/introspection/profile_graph.py \
  src/runtime/profiles/catalog/endpoints.py \
  src/runtime/profiles/endpoint_config.py \
  src/runtime/runtime_evidence.py \
  src/runtime/runtime_policy.py \
  src/runtime/service_manager.py \
  src/runtime/tests/test_backend_status.py \
  src/runtime/tests/test_cpp_slam_status_adapter.py \
  src/runtime/tests/test_deployment_service_contracts.py \
  src/runtime/tests/test_external_service_module.py \
  src/runtime/tests/test_full_stack_composition.py \
  src/runtime/tests/test_livox_sdk2_source.py \
  src/runtime/tests/test_nav_chain_efficiency.py \
  src/runtime/tests/test_runtime_catalogs.py \
  src/runtime/tests/test_runtime_evidence.py \
  src/runtime/tests/test_runtime_policy.py \
  src/runtime/tests/test_service_manager.py \
  src/runtime/tests/test_terrain_local_planner_contract.py \
  src/runtime/tests/test_thunder_product_blueprints.py
```

Do not include:

- `src/localization/**`
- `src/nav/**`
- `web/**`
- `sim/**`

Verify:

```bash
python -m pytest \
  src/runtime/tests/test_backend_status.py \
  src/runtime/tests/test_cpp_slam_status_adapter.py \
  src/runtime/tests/test_deployment_service_contracts.py \
  src/runtime/tests/test_external_service_module.py \
  src/runtime/tests/test_full_stack_composition.py \
  src/runtime/tests/test_livox_sdk2_source.py \
  src/runtime/tests/test_nav_chain_efficiency.py \
  src/runtime/tests/test_runtime_catalogs.py \
  src/runtime/tests/test_runtime_evidence.py \
  src/runtime/tests/test_runtime_policy.py \
  src/runtime/tests/test_service_manager.py \
  src/runtime/tests/test_terrain_local_planner_contract.py \
  src/runtime/tests/test_thunder_product_blueprints.py \
  -q
```

Robot gate:

```bash
ssh sunrise@192.168.66.13 "cd ~/data/inovxio/lingtu && bash scripts/lingtu svc status"
```

Commit only if the local tests pass and the robot still reports active
`lidar`, `slam`, `nav_dds`, and `lingtu` services.

## Batch 2 - Gateway Product API

Outcome: Gateway exposes product readiness, session, map, runtime status, and
operator commands without leaking internal module details.

Stage:

```bash
git add \
  src/gateway/gateway_module.py \
  src/gateway/routes/diagnostics.py \
  src/gateway/routes/maps.py \
  src/gateway/routes/operations.py \
  src/gateway/routes/session.py \
  src/gateway/services/control_commands.py \
  src/gateway/services/readiness.py \
  src/gateway/services/runtime_status.py \
  src/gateway/tests/test_gateway_health_contract.py \
  src/gateway/tests/test_gateway_runtime_status.py \
  src/gateway/tests/test_gateway_session_map_contract.py
```

Verify:

```bash
python -m pytest \
  src/gateway/tests/test_gateway_health_contract.py \
  src/gateway/tests/test_gateway_runtime_status.py \
  src/gateway/tests/test_gateway_session_map_contract.py \
  -q
```

Robot gate:

```bash
ssh sunrise@192.168.66.13 "curl -fsS http://127.0.0.1:5050/api/v1/runtime/status >/tmp/runtime_status.json && python3 -m json.tool /tmp/runtime_status.json >/dev/null"
```

Commit only if Gateway returns valid JSON and no robot motion command is active.

## Batch 3 - Native SLAM DDS And Saved-Map Relocalization

Outcome: native C++ SLAM runtime can ingest DDS sensor data, report health, and
attempt saved-map relocalization without ROS2 on the main path.

Stage:

```bash
git add \
  scripts/build/build_3d_bbs.sh \
  scripts/build/build_livox_sdk2_stream.sh \
  scripts/build/build_slam_core.sh \
  scripts/deploy/s100p/genz_icp.service \
  scripts/deploy/s100p/hba.service \
  src/drivers/real/lidar/sdk2_stream/main.cpp \
  src/localization/adapters/ros2/slam_bridge.py \
  src/localization/native_localizer/CMakeLists.txt \
  src/localization/profiles.py \
  src/localization/slam/cpp/CMakeLists.txt \
  src/localization/slam/cpp/bind.cpp \
  src/localization/slam/cpp/cyclone_runtime.cpp \
  src/localization/slam/cpp/fastlio.cpp \
  src/localization/slam/cpp/native_relocalizer.cpp \
  src/localization/slam/cpp/native_relocalizer.hpp \
  src/localization/slam/cpp/slam.cpp \
  src/localization/slam/cpp/slam.hpp \
  src/localization/slam/cpp/slam_control.cpp \
  src/localization/tests/test_native_slam_contract.py \
  src/localization/tests/test_relocalization_service.py \
  src/localization/tests/test_slam_stack_services.py \
  src/runtime/adapters/native/localization_adapter.py \
  src/runtime/adapters/native/relocalization.py
```

Verify:

```bash
python -m pytest \
  src/localization/tests/test_native_slam_contract.py \
  src/localization/tests/test_relocalization_service.py \
  src/localization/tests/test_slam_stack_services.py \
  src/runtime/tests/test_cpp_slam_status_adapter.py \
  -q
```

Robot build:

```bash
ssh sunrise@192.168.66.13 "cd ~/data/inovxio/lingtu && LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF LINGTU_SLAM_BUILD_DDS_RUNTIME=ON LINGTU_SLAM_FASTLIO2=ON bash scripts/build/build_slam_core.sh"
```

Robot gate:

```bash
ssh sunrise@192.168.66.13 "cd ~/data/inovxio/lingtu && bash scripts/lingtu system-acceptance --map <map_name> --goal 1.0 0.0 0.0 --with-relocalization --initial-pose 0 0 0"
```

Commit only if the build passes and relocalization returns a supported state.

## Batch 4 - Navigation Planner And Motion Closure

Outcome: global planner, local planner, path follower, safety mux, and odometry
evidence close the low-speed real robot loop.

Stage:

```bash
git add \
  scripts/build/build_nav_endpoint.sh \
  scripts/build/build_octoplanner3d.sh \
  src/nav/kernel/CMakeLists.txt \
  src/nav/kernel/CMakeLists_nanobind_only.cmake \
  src/nav/kernel/bindings/bind_map_layers.cpp \
  src/nav/kernel/bindings/bind_path_follower.cpp \
  src/maps/include/lingtu/maps/layers/grid.hpp \
  src/nav/kernel/include/nav_kernel/path_follower_core.hpp \
  src/nav/kernel/paths.py \
  src/nav/kernel/src/path_follower_core.cpp \
  src/maps/tests/cpp/grid_layers_test.cpp \
  src/nav/kernel/tests/test_path_follower_core.cpp \
  src/nav/local/path_follower.py \
  src/nav/local/path_follower_backend.py \
  src/nav/local/path_follower_runtime.py \
  src/nav/mission/navigation.py \
  src/nav/mission/runtime/control.py \
  src/nav/mission/runtime/planning.py \
  src/nav/services/frame_transforms.py \
  src/maps/services/pipeline.py \
  src/maps/modules/service.py \
  src/nav/services/plan/contracts.py \
  src/nav/services/plan/factory.py \
  src/nav/cpp/planning/global/octoplanner/vendor/planner/include/global_planner.h \
  src/nav/cpp/planning/global/octoplanner/vendor/planner/src/global_planner.cpp \
  src/nav/cpp/planning/global/octoplanner/CMakeLists.txt \
  src/nav/cpp/planning/global/octoplanner/make_test_octomap.cpp \
  src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.cpp \
  src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp \
  src/nav/cpp/planning/global/octoplanner/octoplanner3d_headless.cpp \
  src/nav/services/plan/global_planner/algorithm/README.md \
  src/nav/services/plan/global_planner/algorithm/octoplanner3d_planner.py \
  src/nav/services/plan/global_planner/algorithm/octoplanner3d_protocol.py \
  src/nav/services/plan/global_planner/algorithm/octoplanner3d_runtime.py \
  src/nav/services/plan/global_planner/artifact_gate.py \
  src/nav/services/plan/global_planner/backend_runtime.py \
  src/nav/services/plan/compat/direct.py \
  src/nav/services/plan/global_planner/service.py \
  src/nav/services/plan/local_planner/output.py \
  src/nav/services/plan/local_planner/service.py \
  src/nav/services/plan/preview.py \
  src/nav/services/safety/plan_safety.py \
  src/nav/services/safety/safety_ring.py \
  src/nav/services/safety/velocity_mux.py \
  src/nav/tests/local/test_autonomy_modules.py \
  src/nav/tests/local/test_autonomy_pipeline.py \
  src/nav/tests/local/test_local_planner_backends.py \
  src/nav/tests/local/test_nav_kernel_paths.py \
  src/nav/tests/local/test_path_follower_backend_adapters.py \
  src/nav/tests/local/test_path_follower_module_backends.py \
  src/nav/tests/local/test_pure_language_fallback_contract.py \
  src/nav/tests/planning_backends/test_octoplanner3d_backend.py \
  src/nav/tests/test_velocity_mux_contract.py \
  src/nav/tests/test_global_planner_contracts.py \
  src/nav/tests/test_nav_modules.py \
  src/nav/tests/test_navigation_frame_contract.py \
  src/nav/tests/test_planning_service_factory.py
```

Verify:

```bash
python -m pytest \
  src/nav/tests/local/test_autonomy_modules.py \
  src/nav/tests/local/test_autonomy_pipeline.py \
  src/nav/tests/local/test_local_planner_backends.py \
  src/nav/tests/local/test_nav_kernel_paths.py \
  src/nav/tests/local/test_path_follower_backend_adapters.py \
  src/nav/tests/local/test_path_follower_module_backends.py \
  src/nav/tests/local/test_pure_language_fallback_contract.py \
  src/nav/tests/planning_backends/test_octoplanner3d_backend.py \
  src/nav/tests/test_velocity_mux_contract.py \
  src/nav/tests/test_global_planner_contracts.py \
  src/nav/tests/test_nav_modules.py \
  src/nav/tests/test_navigation_frame_contract.py \
  src/nav/tests/test_planning_service_factory.py \
  -q
```

Robot gate:

```bash
ssh sunrise@192.168.66.13 "cd ~/data/inovxio/lingtu && bash scripts/lingtu system-acceptance --map <map_name> --goal 0.4 0.0 0.0 --allow-motion --initial-pose 0 0 0"
```

Commit only if `global_path`, `local_path`, `cmd_vel`, and odometry displacement
are all present in the field evidence.

## Batch 5 - Simulation Parity

Outcome: MuJoCo/ThunderV4 simulation remains a useful no-hardware rehearsal for
SLAM, navigation, and robot command flow.

Stage:

```bash
git add \
  sim/engine/core/robot.py \
  sim/engine/mujoco/engine.py \
  sim/engine/mujoco/robot_controller.py \
  sim/robots/README.md \
  sim/robots/thunderv4 \
  sim/scripts/saved_map_relocalize_contract_gate.py \
  sim/tests/test_saved_map_relocalize_contract_gate.py \
  sim/tests/test_sim_runtime_compat.py \
  src/drivers/sim/mujoco/driver.py \
  src/runtime/adapters/lcm/sources/brainstem_sim.py \
  src/runtime/profiles/catalog/simulation_profiles.py \
  src/runtime/tests/test_brainstem_sim_source.py \
  src/runtime/tests/test_sim_nav_e2e.py
```

Verify:

```bash
python -m pytest \
  sim/tests/test_saved_map_relocalize_contract_gate.py \
  sim/tests/test_sim_runtime_compat.py \
  src/runtime/tests/test_brainstem_sim_source.py \
  src/runtime/tests/test_sim_nav_e2e.py \
  -q
```

Commit only if the no-hardware tests pass. Do not use simulation success as a
replacement for Batch 4 field evidence.

## Batch 6 - Planner Tuning UI

Outcome: the dashboard exposes product tuning controls for planner and safety
parameters without changing robot runtime behavior.

Stage:

```bash
git add \
  web/src/App.tsx \
  web/src/components/PlannerTuning.module.css \
  web/src/components/PlannerTuning.tsx \
  web/src/components/TabBar.tsx \
  web/src/components/Topbar.tsx
```

Verify:

```bash
npm --prefix web run lint
npm --prefix web run build
```

Commit only if the UI builds and existing runtime switch behavior still works.

## Batch 7 - Operator Documentation

Outcome: public docs match the verified runtime shape after Batches 1-6 land.

Stage:

```bash
git add \
  README.md \
  docs/03-development/PARAMETER_TUNING.md \
  docs/04-deployment/README.md \
  docs/04-deployment/services/install.sh \
  docs/04-deployment/super_lio_backend.md \
  docs/QUICKSTART.md \
  docs/TUNING.md \
  docs/architecture/ros_frame_contract.md \
  scripts/build/README.md
```

Verify:

```bash
git diff --cached --check
```

Commit only after the earlier batches have passed. Docs must describe verified
behavior, not intended behavior.

## Batch Order

1. Runtime and deployment contract
2. Gateway product API
3. Native SLAM DDS and saved-map relocalization
4. Navigation planner and motion closure
5. Simulation parity
6. Planner tuning UI
7. Operator documentation

The product is not releasable until Batches 1-4 are green on the robot.
