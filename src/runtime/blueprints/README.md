# Blueprints - Module Composition & Stack Factories

This package provides the composition layer that assembles individual Modules into running systems. A Blueprint is the only orchestration unit; stack factories bundle related modules into reusable, testable stacks.

## Files

- **`full_stack.py`** - Top-level system blueprint; wires 9 stack factories into a single composable system.
- **`full_stack_wiring.py`** - Explicit cross-stack wire definitions (Safety -> Driver, SLAM -> Nav, VisualServo dual channel, VelocityMux arbitration).
- **`stub.py`** - Stub blueprint for framework testing; creates a minimal module graph with no hardware dependencies.
- **`multi_robot.py`** - Multi-robot orchestration: profile swapping, dual-driver wiring, and StubRobot for testing.
- **`profile_graph.py`** - Directed acyclic graph of named profiles and robot presets; resolves dependency order for `lingtu.py <profile>`.
- **`runtime_endpoint.py`** - Runtime profile discovery; exposes available profiles and their config via the REPL.
- **`simulation_contract.py`** - Simulation backend contract; defines the interface for MuJoCo/Gazebo/ROS2 sim backends.

### `stacks/` Directory

- **`_registry.py`** - Central registry of all stack factory functions; maps factory names to callables.
- **`driver.py`** - `driver(robot)` factory: returns Driver + CameraBridge blueprint (auto-detect Thunder/stub/sim).
- **`localization.py`** - `slam(profile)` factory: returns a localization adapter aliased as SlamBridgeModule; SLAM processes are external runtime services.
- **`maps.py`** - `maps()` factory: returns OccupancyGrid + ESDF + ElevationMap stack.
- **`perception.py`** - `perception(det, enc)` factory: returns Detector + Encoder + Reconstruction stack.
- **`memory.py`** - `memory(save_dir)` factory: returns SemanticMapper + Episodic + Tagged + VectorMemory stack.
- **`planner.py`** - `planner(llm)` factory: returns SemanticPlanner + LLM + VisualServo stack.
- **`navigation.py`** - `navigation(planner)` factory: returns Navigation + autonomy chain (OctoPlanner3D/A*/PCT).
- **`safety.py`** - `safety()` factory: returns SafetyRing + Geofence stack.
- **`gateway.py`** - `gateway(port)` factory: returns Gateway (HTTP/WS/SSE) + MCP + Teleop stack.
- **`exploration.py`** - `exploration(backend)` factory: returns TARE exploration stack.
- **`lidar.py`** - `lidar(ip, enabled)` factory: returns Livox MID-360 hardware driver (decoupled from SLAM).
- **`sim_lidar.py`** - `sim_lidar(scene_xml)` factory: returns simulated PointCloud2 provider from MuJoCo scene geometry.
