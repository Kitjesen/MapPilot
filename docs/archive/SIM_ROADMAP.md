# LingTu Simulation Stack

> Status snapshot of the MuJoCo-based simulation rig and how it plugs into the
> rest of the navigation stack. This file is **descriptive** (what exists today),
> not aspirational. For setup instructions see `sim/README.md`.

## 1. Two Independent Bridge Paths

The repository ships two separate ways to drive navigation from MuJoCo. They
are not interchangeable — pick the one that matches your stack.

### 1.1 Pure-Python LingTu Modules (default for `python lingtu.py sim`)

```
MuJoCo physics ─→ ROS2 topics ─→ ROS2SimDriverModule ─→ Module graph
                       ↑                                      ↓
                bridge node                              CmdVelMux ─→ Driver
```

| Component | File | Notes |
|-----------|------|-------|
| Bridge node | `sim/bridge/mujoco_ros2_bridge.py` | Publishes `/nav/odometry`, `/nav/map_cloud`, TF (`map`→`odom`→`body`); subscribes `/nav/cmd_vel`. Runs at 50 Hz. |
| Bridge node (legacy/no-ROS) | `sim/bridge/nova_nav_bridge.py` | Same idea but feeds the Python LingTu modules in-process. |
| Driver backend | `src/drivers/sim/ros2_sim_driver.py` (`@register("driver", "sim_ros2")`) | Subscribes the bridge topics; emits `Odometry` + `PointCloud` on Module ports. |
| In-process variant | `src/drivers/sim/mujoco/driver.py` (`@register("driver", "sim_mujoco")`) | MuJoCo runs inside the Lingtu process; no ROS2. |

The `_apply_cmd()` qvel injection that older planning notes flagged as a TODO
is implemented at `sim/bridge/mujoco_ros2_bridge.py:220-244` (and the same
contract in `mujoco_viz_bridge.py:372` and `nova_nav_bridge.py`). The bridge
also clamps roll/pitch each tick to keep the freejoint upright, so the kinematic
robot stays vertical even with no leg model.

### 1.2 ROS2 C++ Autonomy Stack

`sim/launch/sim.launch.py` and the C++ packages under `src/base_autonomy/`
(`terrain_analysis`, `local_planner`, `path_follower`) consume the same
bridge topics. This path is currently exercised by the on-robot deployment
on S100P; the simulator simply replaces the Livox driver with the bridge.

## 2. LiDAR Simulation

Two implementations live side by side in `sim/sensors/livox_mid360.py`:

| Path | When used |
|------|-----------|
| `mujoco_ray_caster` C++ plugin | If the plugin `.so` is present on `MUJOCO_PLUGIN_PATH`. Configured in the world XML's `<sensor plugin="mujoco.sensor.ray_caster_lidar">` block. |
| Pure-Python `mj_multiRay` fallback | Default. ~6400 rays/frame in a golden-angle spiral with σ=0.02 m Gaussian noise. ~50 ms/scan. |

Both produce `(N, 3) float32` xyz packed into `sensor_msgs/PointCloud2` with
`frame_id='odom'`. World XMLs (`sim/worlds/{open_field, building_scene,
factory_scene, spiral_terrain}.xml`) are interchangeable.

## 3. Robot Models

| File | Used by |
|------|---------|
| `sim/robot/thunder.urdf` | Thunder quadruped (16-DOF). Meshes in `sim/assets/meshes/`. Currently the only URDF in this directory. |
| `sim/robots/go2/` | Unitree Go2 MJCF + meshes. Used by `sim/scripts/run_sim.py` Go2 demos. |
| `sim/robots/nova_dog/` | NOVA Dog MJCF (legacy, kept for the person-following benchmark). |

There is no `nova_dog.urdf` in `sim/robot/` — the URDF directory is reserved
for ROS-side TF/visualisation needs.

## 4. Person-Following Sandbox

`sim/following/` is a self-contained behavioural test rig (no ROS2):

```
PersonModule (room-aware walk + ROOM_GRAPH)
  ↓
PerceptionModule (CLIP / YOLO / IoU stub)
  ↓
ControllerModule (FSM: FOLLOW / WAIT / SEARCH / EXPLORE / RECOVER)
  ↓
MetricsModule (logging + bench plots)
```

Run `python sim/scripts/benchmark_following.py` for the canned regression set.

## 5. Profiles vs Sim

`python lingtu.py sim` boots the full Module graph against the ROS2 bridge.
Other simulation-friendly profiles:

| Profile | Driver | Use case |
|---------|--------|----------|
| `stub` | `stub` | Framework-only tests, no physics |
| `dev` | `stub` | Semantic pipeline development |
| `sim` | `sim_ros2` | MuJoCo physics + ROS2 bridge |
| `map` | `sim_ros2` | SLAM mapping in MuJoCo |

## 6. Known Gaps / Limitations

- **No leg-level dynamics.** The robot is a freejoint body with qvel injection;
  contact reasoning is fine but gait and slope handling are not modelled.
- **The 50 Hz tick assumes mujoco's substep `dt=0.002`.** Changing the world
  XML timestep silently changes the velocity scaling.
- **`terrain_analysis_ext.cpp` is on by default in `sim.launch.py`.** Disable it
  with the launch arg if you want the lighter `terrain_analysis` only.
- **Large worlds + Python LiDAR fallback get expensive.** ~50 ms/scan at 6400
  rays leaves limited budget for the planner; build the C++ plugin if you push
  past 10 Hz.

## 7. Useful Files at a Glance

| File | Purpose |
|------|---------|
| `sim/scripts/run_sim.py` | Headless / GUI / no-ROS entrypoint |
| `sim/launch/sim.launch.py` | ROS2 full-stack launch |
| `sim/bridge/mujoco_ros2_bridge.py` | qvel injection + odom + cloud publisher |
| `sim/sensors/livox_mid360.py` | LiDAR plugin + Python fallback |
| `sim/worlds/*.xml` | MuJoCo scene definitions |
| `src/drivers/sim/ros2_sim_driver.py` | Module-side adapter for the bridge |
| `sim/following/` | Behavioural benchmarking sandbox |
