# LingTu Repository Layout

```
brain/lingtu/
├── lingtu.py              # Primary Python entry — sets project root, runs cli.main
├── main_nav.py            # Backwards-compatible alias (same as lingtu.py)
├── lingtu_cli.py          # Target of the pip console script `lingtu`
├── cli/                   # CLI implementation (argparse, profiles, REPL, daemon)
│   ├── bootstrap.py       # init() — extends sys.path to src/ and semantic subpkgs
│   ├── main.py            # main() — argparse, profile resolution, blueprint build
│   ├── profiles_data.py   # ROBOT_PRESETS, PROFILES, _ACTIVE_TOMOGRAM
│   ├── repl.py            # LingTuREPL (cmd.Cmd subclass)
│   ├── run_state.py       # .lingtu/run.{pid,json} for daemon lifecycle
│   ├── runtime_extra.py   # preflight, kill_residual_ports, daemonize, health_check
│   └── ...                # logging_util, paths, term, ui
│
├── src/                   # Python packages + ROS2 packages (colcon)
│   ├── core/              # Module, Blueprint, Transport, NativeModule, Registry, msgs
│   │   ├── blueprints/
│   │   │   ├── full_stack.py     # full_stack_blueprint(): assembles the stacks below
│   │   │   └── stacks/           # composable factory functions
│   │   │       ├── driver.py
│   │   │       ├── lidar.py / sim_lidar.py
│   │   │       ├── slam.py
│   │   │       ├── maps.py
│   │   │       ├── perception.py
│   │   │       ├── memory.py
│   │   │       ├── planner.py
│   │   │       ├── navigation.py
│   │   │       ├── exploration.py  # tare | none (wavefront removed 2026-04-25)
│   │   │       ├── safety.py
│   │   │       └── gateway.py
│   │   └── tests/                # ~1000+ framework tests, no ROS2 needed
│   ├── nav/               # NavigationModule, SafetyRing, CmdVelMux, OccupancyGrid, ESDF, …
│   │   ├── services/      # frame_contract, geofence, map_manager, patrol, task_scheduler
│   │   ├── ros2_waypoint_bridge_module.py  # Extracted ROS2 waypoint bridge
│   │   ├── ros2_grid_bridge_module.py
│   │   ├── ros2_path_bridge_module.py
│   │   └── …
│   ├── semantic/          # Flat perception/ + planner/ + reconstruction/ layout
│   │   ├── perception/    # api/, impl/, srv/, storage/, tests/
│   │   ├── planner/       # AgentLoop, GoalResolver, VisualServo, tests/, resource/
│   │   └── reconstruction/  # server/ + tests/
│   ├── memory/            # SemanticMapper, EpisodicMemory, Tagged, VectorMemory, KG
│   ├── drivers/           # thunder/ (gRPC), sim/ (stub, MuJoCo, ROS2), TeleopModule
│   │   ├── real/          # Real-robot camera drivers
│   │   ├── gnss/          # GNSS receiver driver
│   │   ├── livox_ros_driver2/  # Livox MID-360 hardware driver
│   │   └── tests/         # Driver spec tests
│   ├── gateway/           # GatewayModule (FastAPI), MCPServerModule
│   ├── webrtc/            # Optional WebRTC camera stream module (aiortc)
│   │   └── tests/         # test_webrtc_module.py
│   ├── base_autonomy/     # C++ terrain + local planner + path follower (nanobind)
│   │   └── tests/         # test_autonomy_modules.py
│   ├── global_planning/   # PCT planner — all snake_case
│   │   ├── pct_planner/           # C++ ele_planner.so + tomography
│   │   ├── pct_adapters/          # Python adaptation layer + tests/
│   │   └── pct_planner_runnable/  # Run wrapper
│   ├── slam/              # Fast-LIO2, Point-LIO, PGO, Localizer, GNSS bridge
│   │   └── launch/        # Merged launchers: fastlio2, pointlio, localizer, pgo, hba, genz_icp
│   ├── exploration/       # TARE exploration
│   │   ├── tare_planner/          # C++ TARE submodule
│   │   ├── tests/                 # test_exploration_modules.py
│   │   ├── tare_explorer_module.py
│   │   ├── tare_ros2_bridge_module.py  # ROS2 extraction
│   │   └── exploration_supervisor_module.py
│   ├── reconstruction/    # 3D reconstruction
│   └── legacy/            # Consolidation of 5 legacy dirs (gateway, pct_planner, scripts, semantic, thunder)
│
├── sim/                   # Simulation: engine, worlds, assets, robots, scripts, validation, evaluation
│   ├── engine/            # Sim engine core
│   │   ├── core/          # engine.py, robot.py, sensor.py, world.py
│   │   ├── mujoco/        # MuJoCo runtime
│   │   ├── bridge/        # ROS2/Gazebo/Unity adapters (cmu_unity, gazebo_bridge, …)
│   │   ├── scenarios/     # Scenario definitions
│   │   └── worlds/        # World definitions
│   ├── worlds/            # Scene XML/SDF organized by simulator
│   │   ├── mujoco/        # .xml scene files (building, factory, office, industrial, …)
│   │   └── gazebo/        # .sdf scene files (demo_room, empty, industrial_park)
│   ├── bridge/            # Sim bridge (CMU Unity Lingtu adapter)
│   ├── tests/             # 19 sim integration tests (dataflow, mujoco, relocalize, …)
│   ├── planning/          # Planning simulation (e2e_nav_test, factory_demo, …)
│   ├── experiments/       # Experiment runners + configs (ablation, habitat, eval, …)
│   ├── evaluation/        # Evaluation framework
│   │   └── slam/          # SLAM evaluation
│   ├── validation/        # Full-system validation suite
│   ├── following/         # Person following sim tests
│   │   ├── person/
│   │   ├── perception/
│   │   ├── controller/
│   │   └── metrics/
│   ├── datasets/          # Avia, legkilo, …
│   ├── sensors/           # Simulated sensors (Livox MID-360, WTRTK980)
│   ├── scripts/           # E2E tests, benchmarks, dataflow
│   ├── configs/           # Sim configuration files
│   ├── assets/            # Sim assets (meshes, textures)
│   ├── robots/            # Robot URDF/MJCF definitions
│   ├── maps/              # Sim map files
│   ├── output/            # Sim output artifacts
│   ├── semantic/          # Semantic sim helpers
│   ├── external_scenes/   # Imported external scenes
│   └── launch/            # Sim launch scripts
│
├── config/                # YAML / DDS / robot params (devices.yaml, robot_config.yaml, …)
├── calibration/           # Sensor calibration toolbox (camera, IMU, LiDAR, camera-LiDAR)
│   ├── camera/            # Camera intrinsics (OpenCV checkerboard)
│   ├── imu/               # IMU noise (Allan Variance, Autoliv)
│   ├── lidar_imu/         # LiDAR-IMU extrinsics (HKU-MARS LI-Init)
│   ├── camera_lidar/      # Camera-LiDAR extrinsics (target-less, koide3)
│   ├── apply_calibration.py  # One-click apply → robot_config.yaml + SLAM configs
│   ├── verify.py          # One-click verify (sanity checks + projection chain)
│   └── README.md          # Full SOP with command-line examples
├── launch/                # ROS2 launch — profiles + Gazebo simulation scaffold
│   ├── _robot_config.py
│   ├── profiles/          # SLAM/planner profiles consumed by NativeModule subprocesses
│   │   ├── slam_fastlio2.launch.py
│   │   ├── slam_pointlio.launch.py
│   │   ├── slam_stub.launch.py
│   │   ├── localizer_icp.launch.py
│   │   ├── planner_pct.launch.py
│   │   ├── planner_pct_py.launch.py
│   │   ├── planner_far.launch.py
│   │   └── planner_stub.launch.py
│   └── gazebo_simulation.launch.py  # Root Gazebo simulation scaffold
├── tests/                 # Remaining integration + planning tests (some need ROS2)
│   ├── benchmark/         # Performance benchmarks
│   ├── e2e/               # End-to-end tests
│   ├── integration/       # Integration tests
│   └── scripts/           # Script-based tests
├── tools/                 # Robot-side helpers (dashboards, BPU export, diagnostics)
├── scripts/               # Build helpers, deploy, ota, proto, lingtu CLI shell wrapper
│   ├── build/             # ROS workspace build helpers, fetch_ortools, build_tare, …
│   ├── deploy/            # Installers, systemd, OTA, monitoring
│   │   └── infra/stack/   # Docker + extra systemd + cron/logrotate (legacy deploy stack)
│   ├── ota/               # Package and push to robot
│   ├── proto/             # protoc helpers
│   ├── lingtu             # Bash CLI for ops on the robot (status / map / nav / svc / log)
│   └── …                  # build_dufomap.sh, build_nav_core.sh, doctor.py, ...
├── web/                   # React + Vite dashboard
├── docs/                  # Architecture, guides, ADRs
├── Makefile               # colcon build / test / format / lint / mapping / navigation
├── pyproject.toml         # Root package: lingtu (setuptools)
├── requirements.txt       # S100P runtime deps
└── docker-compose.yml     # Container orchestration
```

## Entry-point precedence

1. `python lingtu.py [profile]` — primary, CLI + REPL.
2. `lingtu` — pip console script (`pyproject.toml` → `lingtu = lingtu_cli:main`).
3. `python main_nav.py` — alias kept for older scripts.
4. `make mapping` / `make navigation` — thin wrappers around `lingtu.py map` / `lingtu.py s100p`.

There are **no** Module-First ROS2 launch files. The Module is the runtime
unit; SLAM and other C++ subsystems are managed via `NativeModule` (see
`docs/archive/MODULE_FIRST_GUIDELINE.md`).

## Where things actually live

| You want… | Read |
|-----------|------|
| The eight architectural rules | `docs/archive/MODULE_FIRST_GUIDELINE.md` |
| Which modules a profile pulls in | `cli/profiles_data.py` + `src/core/blueprints/full_stack.py` |
| All cross-stack wires | `src/core/blueprints/full_stack.py` |
| Which backends are registered for a category | `src/core/registry.py` plus the `@register(...)` calls in each Module file |
| Simulation folder boundaries | `sim/README.md` (worlds, assets, robots, scripts, validation/evaluation — full details in engine/core bridge/sensors/datasets scenarios, mujoco/gazebo scenes, following/) |
| Server-side simulation evidence | `artifacts/server_sim_closure/` generated reports plus `artifacts/server_sim_closure_summary_g4_current.json` |
| Root Gazebo simulation scaffold | `launch/gazebo_simulation.launch.py` (kept at root because ROS-native launchers reference it directly) |
| Robot physical parameters | `config/robot_config.yaml` (single source of truth) |
| Calibration SOP | `calibration/README.md` |
| Operations CLI on the robot | `scripts/lingtu` + `docs/archive/AGENTS.md` |
