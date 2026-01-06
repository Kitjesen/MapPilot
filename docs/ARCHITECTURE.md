# System Architecture

## Overview

The PCT Planner navigation stack follows a **hierarchical three-layer architecture**: Global Planning, Local Planning, and Execution. Each layer operates at different frequencies and abstraction levels.

## Layer Breakdown

### 🌍 Layer 1: Global Planning (1 Hz)

**Purpose**: Generate collision-free paths from start to goal using terrain-aware A* search.

#### Components

##### 1.1 Tomography (`tomography.py`)
- **Function**: Converts raw point cloud into a 5-layer traversability map
- **Outputs**:
  - `layers_t`: Traversability cost (0-1)
  - `trav_grad_x/y`: Cost gradients
  - `layers_g`: Ground clearance
  - `layers_c`: Ceiling clearance
- **Map Format**: Pickle file (`.pkl`) with metadata

##### 1.2 PCT Planner (`global_planner.py` + C++ Core)
- **Input**: Tomogram map + Goal position
- **Algorithm**: 
  1. A* search on traversability grid
  2. Trajectory optimization (quintic splines)
  3. Dynamic obstacle consideration
- **Output**: `/planned_path` (geometry_msgs/Path)

#### Key Topics
- **Subscribe**: `/way_point` (goal), `/Odometry` (robot pose)
- **Publish**: `/planned_path` (global path)

---

### 🚗 Layer 2: Local Planning (10 Hz)

**Purpose**: Real-time obstacle avoidance by selecting optimal paths from a pre-computed library.

#### Components

##### 2.1 Terrain Analysis (`terrainAnalysis.cpp`)
- **Function**: Rolling terrain map with dynamic obstacle filtering
- **Processing**:
  1. **Voxel Stacking**: Accumulate scans in terrain voxels
  2. **Ground Estimation**: Planar voxel-based elevation map
  3. **Dynamic Filtering**: Time-decay for moving obstacles
- **Output**: `/terrain_map` (sensor_msgs/PointCloud2)

##### 2.2 Terrain Analysis Ext (`terrainAnalysisExt.cpp`)
- **Function**: Extended terrain map with connectivity checking
- **Additional Features**:
  - Terrain connectivity validation (BFS)
  - Ceiling filtering for indoor environments
  - Integration with local terrain map
- **Output**: `/terrain_map_ext` (sensor_msgs/PointCloud2)

##### 2.3 Local Planner (`localPlanner.cpp`)
- **Function**: Evaluate 343 candidate paths and select the safest
- **Algorithm**:
  1. **Voxelization**: Map obstacles to path grid
  2. **Collision Check**: Mark blocked paths
  3. **Path Scoring**: Combine direction + cost
  4. **Path Selection**: Choose best free path
- **Output**: `/path` (nav_msgs/Path), `/slow_down` (std_msgs/Int8)

#### Path Library
- **Total Paths**: 343 (7 groups × 49 paths)
- **Coverage**: ±180° azimuth, multiple curvatures
- **Storage**: `paths.ply` + `correspondences.txt`

#### Key Topics
- **Subscribe**: `/cloud_registered`, `/Odometry`, `/joy`, `/planned_path`
- **Publish**: `/path`, `/slow_down`

---

### 🎯 Layer 3: Execution (100 Hz)

**Purpose**: Track the local path using pure pursuit control.

#### Components

##### 3.1 Path Follower (`pathFollower.cpp`)
- **Function**: Pure pursuit path tracking with adaptive lookahead
- **Algorithm**:
  1. **Lookahead Calculation**: `L = L_base + k * v`
  2. **Path Point Search**: Find point at distance L
  3. **Control Law**: `ω = 2v sin(α) / L`
  4. **Speed Control**: Acceleration-limited velocity tracking
- **Output**: `/cmd_vel` (geometry_msgs/TwistStamped)

##### 3.2 Serial Communication (Optional)
- **Function**: Send velocity commands to hardware motor controller
- **Protocol**: Binary format (3 floats + newline)

#### Key Topics
- **Subscribe**: `/path`, `/Odometry`, `/joy`, `/slow_down`, `/speed`
- **Publish**: `/cmd_vel`

---

## Data Flow Diagram

```mermaid
graph TB
    subgraph Sensors
        A[LiDAR] -->|/cloud_registered| B[SLAM]
        B -->|/Odometry| C[Global Planner]
        B -->|/Odometry| D[Terrain Analysis]
        B -->|/Odometry| E[Path Follower]
    end
    
    subgraph Global Layer
        F[Tomography] -->|tomogram.pkl| C
        G[Joystick/UI] -->|/way_point| C
        C -->|/planned_path| H[Local Planner]
    end
    
    subgraph Local Layer
        A -->|/cloud_registered| D
        D -->|/terrain_map| H
        H -->|/path| E
        H -->|/slow_down| E
    end
    
    subgraph Execution Layer
        E -->|/cmd_vel| I[Robot Base]
    end
```

---

## Message Flow Timing

| Layer | Component | Frequency | Latency | CPU % |
|-------|-----------|-----------|---------|-------|
| **Global** | Tomography | On Demand | ~10s | 5% |
| **Global** | PCT Planner | 1 Hz | ~100ms | 10% |
| **Local** | Terrain Analysis | 10 Hz | ~20ms | 15% |
| **Local** | Local Planner | 10 Hz | ~30ms | 20% |
| **Execution** | Path Follower | 100 Hz | ~1ms | 5% |

---

## Control Modes

### 1️⃣ Autonomous Navigation
- **Trigger**: `/joy` axes[2] < -0.1
- **Behavior**: Follow `/path` automatically
- **Speed**: `autonomySpeed` parameter

### 2️⃣ Manual Control
- **Trigger**: `/joy` axes[5] < -0.1
- **Behavior**: Direct joystick control
- **Override**: Bypasses path follower

### 3️⃣ Teleoperation
- **Trigger**: `/joy` axes[2] > -0.1
- **Behavior**: Joystick speed modulation
- **Safety**: Still respects `/slow_down` signals

---

## Safety Mechanisms

### Emergency Stop
- **Trigger**: `/stop` topic or `safetyStop >= 1`
- **Action**: Set velocity to 0
- **Recovery**: Manual reset required

### Slow Down
- **Trigger**: Limited free paths or terrain cost
- **Levels**:
  - Level 1: 25% speed
  - Level 2: 50% speed
  - Level 3: 75% speed

### Collision Avoidance
- **Method**: Path selection from pre-validated library
- **Fallback**: Emergency stop if no free paths

---

## Configuration Hierarchy

```
config/
├── global_planner.yaml    # A* weights, map parameters
├── local_planner.yaml     # Voxel sizes, path selection
├── path_follower.yaml     # Pure pursuit gains
└── terrain_analysis.yaml  # Voxel decay, thresholds
```

**Load Order**: Launch file → Package defaults → Runtime parameters

---

## State Management

### Global Planner
- **State**: Goal position, current map
- **Persistence**: Map cached in memory
- **Reset**: On new map or `/map_clearing`

### Local Planner
- **State**: Terrain voxels (rolling window)
- **Persistence**: 41×41 voxel grid
- **Reset**: On `/map_clearing` or joystick button

### Path Follower
- **State**: Current path, vehicle pose
- **Persistence**: Path until new one received
- **Reset**: On new `/path` message

---

## Failure Modes & Recovery

| Failure | Detection | Recovery |
|---------|-----------|----------|
| **No Global Path** | Empty `/planned_path` | Stop and wait |
| **All Paths Blocked** | Local planner finds no free paths | Emergency stop |
| **Odometry Lost** | Timeout on `/Odometry` | Hold last command |
| **LiDAR Failure** | Timeout on `/cloud_registered` | Use decayed terrain map |
| **Goal Unreachable** | Planning timeout (5s) | Report error to user |

---

## Performance Optimization

### Key Optimizations Applied

1. **Voxel Grid Downsampling**: 0.05m → 0.1m (CPU: -50%)
2. **Adaptive Lookahead**: Dynamic based on speed (Tracking: +30%)
3. **Path Search Memoization**: Start from last point (CPU: -40%)
4. **Terrain Voxel Decay**: Remove old points (Memory: -60%)

### Profiling Results

```
Component          | CPU (%) | Memory (MB) | Notes
-------------------|---------|-------------|-------------------
Global Planner     | 10%     | 800         | Peaks during planning
Local Planner      | 20%     | 200         | Constant
Terrain Analysis   | 15%     | 150         | Varies with scan rate
Path Follower      | 5%      | 50          | Minimal
Total              | ~35%    | ~1200       | Measured on i7-9700K
```

---

## Extension Points

### Adding a New Planner
1. Subscribe to `/Odometry` and `/cloud_registered`
2. Publish to `/planned_path` or `/path`
3. Register in `system_launch.py`

### Custom Cost Functions
1. Modify `tomogram.py` → `trav_kernel()`
2. Rebuild tomogram map
3. Update A* weights in config

### Hardware Integration
1. Implement `/cmd_vel` subscriber on robot
2. OR enable serial port in `pathFollower.yaml`
3. Configure `serialPort` and `baudrate`

---

**Last Updated**: January 2026
**Version**: 2.0 (Post-Refactoring)
