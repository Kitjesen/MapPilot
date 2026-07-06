# LingTu Product Mode Runtime Contract

This document defines the runtime chains that top-level product profiles must
wire together. The code contract lives in
`src/runtime/profiles/product_mode_contracts.py`; graph tests live in
`src/runtime/tests/test_profile_graph_snapshots.py`.

## General Rules

Frontend, CLI, and MCP only submit requests and display state. They must not
choose navigation paths directly, and they must not bypass `CmdVelMux` to
control the robot.

## Runtime Identity

Runtime state is described by four fields. Do not collapse them into one name.

| Field | Example | Owner | Meaning |
| --- | --- | --- | --- |
| `profile` | `teleop`, `teleop_avoid`, `map`, `tracking`, `nav`, `inspection`, `tare_explore` | LingTu product catalog | What product behavior is running |
| `endpoint` | `thunder_field`, `mujoco_live`, `replay` | runtime endpoint catalog | Where data comes from and which boundary owns I/O |
| `session_mode` | `mapping`, `navigating`, `exploring` | Gateway session API | Low-level resource session used for service start/stop and map accumulation policy |
| `product_session` | `teleop`, `teleop_avoid`, `mapping`, `tracking`, `navigation`, `inspection`, `exploration` | product mode contract | What the operator is actually doing |
| `slam_mode` | `mapping`, `localization` | `lingtu-slam-dds.service` | Whether SLAM is building a map or localizing against a saved map |

The field robot identity should be read as:

```text
profile=nav
endpoint=thunder_field
session_mode=navigating
product_session=navigation
slam_mode=localization
map=<active map name>
```

`session_mode` is intentionally coarse because it controls shared resources:
`mapping` accumulates live map data, `navigating` uses a saved map and
localization, and `exploring` runs exploration on top of live navigation
readiness. Product UI must display `product_session`, not only
`session_mode`, otherwise `tracking`, `navigation`, and `inspection` all look
the same.

`thunder-nav`, `thunder-map`, and `thunder-explore` are compatibility aliases
only. Deployment defaults must use canonical profiles such as `nav` and `map`.
The endpoint name `thunder_field` remains because it names the physical Thunder
field boundary, not a product mode.

Target points enter only the mission or goal service. Simulation, local-driver,
and compatibility Module graphs can still run the full in-process chain:

```text
nav.mission -> nav.local_planner -> nav.path_follower -> nav.velocity_mux
```

The default physical `thunder_field` endpoint is different: the Python Module
graph keeps mission, goal, map, status, and safety contracts, while C++
`lingtu-nav-dds` owns the field navigation data plane through `/nav/cmd_vel`.

Module-owned velocity commands must pass through:

```text
Teleop / VisualServo / PathFollower / Recovery
  -> nav.velocity_mux
  -> nav.safety.cmd_vel
  -> nav.out.cmd_vel or driver only when that boundary exists in the profile
```

## Velocity Commands And Communication Boundary

The current `thunder_field` field chain does not let the Python process control
the robot directly through gRPC. LingTu keeps operator requests, mission state,
map products, and safety state in the Module graph, while the board-side native
navigation service consumes DDS inputs and publishes the final speed command.

```text
Web / CLI / MCP
  -> GatewayModule / TeleopModule / nav.mission
  -> native field endpoint boundary
  -> lingtu-nav-dds
  -> DDS /nav/cmd_vel
```

Key `thunder_field` runtime settings:

```text
endpoint_transport = dds
command_output_mode = endpoint_only
hardware_control_boundary = dds_endpoint_source
native_navigation_endpoint = lingtu-nav-dds
enable_nav_in = false
enable_nav_out = false
enable_robot_driver = false
```

Field DDS navigation has one `/nav/cmd_vel` writer: C++ `lingtu-nav-dds`. The
old Python nav DDS adapters were removed to prevent duplicate goal, path, and
cmd_vel writers in `thunder_field`.

In this default field branch, the Python graph intentionally does not load
`nav.in`, `nav.out`, `map.out`, `nav.terrain`, `nav.local_planner`, or
`nav.path_follower`.

External velocity entries:

```text
WS /ws/teleop
  {"type":"joy","lx":0.5,"ly":0.0,"az":-0.3}
  -> GatewayModule._teleop_on_joy
  -> TeleopModule.joy_input
  -> TeleopModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel

POST /api/v1/cmd_vel
  {"vx":0.2,"vy":0.0,"wz":0.1}
  -> GatewayModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel

MCPServerModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel
```

In Module-owned simulation/compatibility chains, velocity candidates enter
`nav.velocity_mux` as separate sources:

```text
TeleopModule / GatewayModule / MCPServerModule
  -> teleop_cmd_vel          priority 100
VisualServoModule
  -> visual_servo_cmd_vel    priority 80
nav.mission recovery
  -> recovery_cmd_vel        priority 60
nav.path_follower
  -> path_follower_cmd_vel   priority 40
```

`nav.velocity_mux` selects the highest-priority active velocity source, then
passes the selected command through the near-field collision monitor. Product
modes with maps wire the collision monitor like this:

```text
SlamAdapterModule.odometry
  -> nav.velocity_mux.collision_odometry

TraversabilityCostModule.fused_cost
  -> nav.velocity_mux.collision_costmap
```

`fused_cost` is a costmap for velocity safety. It is not a UI image and not a
raw point cloud.

```python
{
    "grid": ndarray,        # 0..100 cost, 99/100 means hard obstacle
    "resolution": float,
    "origin": [x, y],
    "ts": float,
    "frame_id": "map" | "odom",
    "backend": optional str,
}
```

It is fused from occupancy, ESDF proximity, elevation slope, and terrain
traversability by taking the highest risk. `nav.velocity_mux` projects the
current command through this costmap:

```text
clear projected path      -> pass
near high-cost cells      -> slowdown
hard obstacle / stale map -> stop
missing odometry / map    -> stop
```

Visual servo hot-switch entry:

```text
POST /api/v1/visual_servo
  {"mode":"find","target":"red chair"}
  {"mode":"follow","target":"person in red"}
  {"mode":"stop"}

GatewayModule.servo_target
  -> VisualServoModule.servo_target
```

`find` and `follow` are motion commands, so Safety STOP rejects them. `stop`
only releases VisualServo, so it is allowed while Safety STOP is active. This
entry hot-switches only inside profiles that already load `VisualServoModule`.
It does not dynamically create VisualServo in lightweight profiles such as
`teleop`, `teleop_avoid`, or `map`.

The legacy direct gRPC path is kept only for lightweight, compatibility, or
local direct-driver chains:

```text
nav.velocity_mux.driver_cmd_vel
  -> ThunderDriver.cmd_vel
  -> brainstem gRPC RobotControlStub.Walk(Vector3)
```

This path exists in `ThunderDriver`, but it is not the default `thunder_field`
field output.

## Product Modes

| Profile | Product Mode | Product Session | Required Chain | Forbidden Chain | Switch Policy |
| --- | --- | --- | --- | --- | --- |
| `teleop` | Teleop | `teleop` | Gateway/Teleop/MCP -> CmdVelMux -> Safety -> NavOut | SLAM, global planning, local planning, path following | Cold restart |
| `teleop_avoid` | Teleop with avoidance | `teleop_avoid` | SLAM/map/traversability cost + Teleop -> CmdVelMux -> Safety -> NavOut | mission, local planner, path follower, semantic planner | Cold restart |
| `map` | Mapping | `mapping` | SLAM -> Occupancy/Voxel/Elevation/ESDF/Traversability -> Gateway/MapManager; Teleop -> CmdVelMux -> NavOut | mission, local planner, path follower, semantic planner | Cold restart |
| `tracking` | Tracking | `tracking` | GoalService -> Mission/status -> lingtu-nav-dds -> `/nav/cmd_vel` | semantic planner, Python terrain/local planner/path follower | Same-graph hot-switch candidate |
| `nav` | Navigation | `navigation` | Web/CLI/MCP/semantic goal -> Mission/status -> lingtu-nav-dds -> `/nav/cmd_vel` | Python terrain/local planner/path follower, target directly becoming motor command | Same-graph hot-switch candidate |
| `inspection` | Inspection | `inspection` | Scheduler/Patrol/Semantic -> GoalService -> Mission/status -> lingtu-nav-dds -> `/nav/cmd_vel` | Python terrain/local planner/path follower, inspection task directly controlling chassis | Same-graph hot-switch candidate |
| `tare_explore` | Exploration | `exploration` | Livox/IMU or endpoint SLAM -> maps/traversability -> TARE -> Mission/status -> lingtu-nav-dds -> `/nav/cmd_vel` | Python terrain/local planner/path follower, TARE directly controlling chassis, WavefrontFrontierExplorer enabled at the same time, missing live map input | Cold restart |

`explore` is kept only as a wavefront frontier compatibility/debug entry. It is
not the default product exploration entry. Field exploration defaults to
`tare_explore` or the `thunder-explore` alias.

## Functional Chains

### Teleop Chain

```text
GatewayModule.cmd_vel
MCPServerModule.cmd_vel
TeleopModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel
  -> nav.velocity_mux.driver_cmd_vel
  -> nav.safety.cmd_vel
```

If the active profile has a local navigation output boundary, the same
`driver_cmd_vel` can also wire to `nav.out.cmd_vel`. In default
`thunder_field`, `nav.out` is absent.

### Map Chain

```text
SlamBridgeModule.map_cloud
  -> OccupancyGridModule.map_cloud
  -> VoxelGridModule.map_cloud
  -> ElevationMapModule.map_cloud
  -> nav.maps.map_cloud

OccupancyGridModule.costmap
ESDFModule.esdf
ElevationMapModule.elevation_map
  -> TraversabilityCostModule
  -> GatewayModule.costmap
```

### Module-Owned Navigation Execution Chain

```text
GatewayModule / MCPServerModule / SemanticPlannerModule
  -> nav.goals / nav.mission.goal_pose
  -> nav.mission.global_path
  -> nav.local_planner.global_path
  -> nav.local_planner.local_path
  -> nav.path_follower.local_path
  -> nav.path_follower.cmd_vel
  -> nav.velocity_mux.path_follower_cmd_vel
  -> nav.out.cmd_vel
```

This chain is for simulation, local-driver, and compatibility profiles. It is
not the default `thunder_field` field execution path.

### thunder_field Native Navigation Endpoint Chain

```text
GatewayModule / MCPServerModule / SemanticPlannerModule
  -> nav.goals / nav.mission.goal_pose

SlamAdapterModule.odometry
  -> nav.mission.odometry

SlamAdapterModule.localization_status
  -> nav.mission.localization_status

SlamAdapterModule.map_odom_tf
  -> nav.mission.map_odom_tf

SlamAdapterModule.map_frame_jump_event
  -> nav.mission.map_frame_jump_event

TraversabilityCostModule.fused_cost
  -> nav.mission.costmap

nav.mission.mission_status
  -> GatewayModule / MCPServerModule / TAREExplorerModule

lingtu-nav-dds
  -> DDS /nav/cmd_vel
```

### TARE Exploration Chain

```text
Livox MID-360 / IMU
  -> Fast-LIO2 / SLAM endpoint
  -> SlamAdapterModule.odometry + SlamAdapterModule.map_cloud
  -> OccupancyGridModule / VoxelGridModule / ElevationMapModule / ESDFModule / TraversabilityCostModule
  -> OccupancyGridModule.exploration_grid
  -> TAREExplorerModule
  -> nav.mission.goal_pose / nav.mission.patrol_goals
  -> thunder_field native navigation endpoint chain
```

In `thunder_field` endpoint mode, LiDAR and Fast-LIO2 run as external C++/DDS
services. The Module graph sees their status/map products through
`SlamAdapterModule`, but it does not add `nav.terrain`.

Local, simulation, or managed-SLAM compatibility profiles can still use the
Module-owned terrain branch:

```text
SlamAdapterModule.map_cloud
  -> OccupancyGridModule.map_cloud
  -> VoxelGridModule.map_cloud
  -> ElevationMapModule.map_cloud
  -> nav.terrain.map_cloud

SlamAdapterModule.odometry
  -> OccupancyGridModule.odometry
  -> VoxelGridModule.odometry
  -> ElevationMapModule.odometry
  -> nav.terrain.odometry

OccupancyGridModule.exploration_grid
  -> TAREExplorerModule
  -> nav.mission.goal_pose / nav.mission.patrol_goals
  -> Module-owned navigation execution chain
```

## Current Switch Conclusion

The system currently provides switch prechecks. `tracking`, `nav`, and
`inspection` share the same native-field mission graph and are marked as
same-graph hot-switch candidates. Full profile-level online hot-switching is
still endpoint-dependent; visual servo target switching is the current explicit
runtime hot-switch entry.

Validation commands:

```bash
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_product_modes_required_wires_are_contract_locked -q
python lingtu.py switch-plan teleop nav --json
python lingtu.py switch-plan tracking inspection --json
python lingtu.py switch-plan tare_explore nav --json
```
