# sim/engine - Python Simulation Engine

Status: development engine used by direct simulation scripts and gates.

`sim/engine` contains Python simulation utilities used by development gates.
It remains hardware-free, but it is
not the canonical generic simulation Runtime. New platform work belongs behind
the interfaces in `sim/runtime/`; protocol conversion belongs in
`sim/adapters/`. Do not add a second coordinator or UE runtime here.

## Package Structure

| Path | Responsibility |
| --- | --- |
| `core/` | Engine abstractions: `SimEngine` (ABC), `SimWorld`, `WorldConfig`, `RobotConfig`, sensor configs (`CameraConfig`, `LidarConfig`, `IMUConfig`), and data types (`RobotState`, `CameraData`, `VelocityCommand`). |
| `mujoco/` | MuJoCo engine implementation: `MuJoCoEngine` (physics stepping, ONNX policy to joint control), `MuJoCoCamera`, `MuJoCoLidar`, `PolicyRunner`. |
| `bridge/` | Explicit Gazebo and CMU compatibility adapters. Native-DDS/MuJoCo adapters live with their gate scripts. |
| `worlds/` | `WorldRegistry` scene XML lookup, alias registration, empty-world fallback. Resolves paths under `sim/worlds/`. |

## Usage

### Import as package

```python
import sys; sys.path.insert(0, "src")

from sim.engine.core import SimEngine, WorldConfig, RobotConfig
from sim.engine.mujoco import MuJoCoEngine
from sim.engine.worlds import WorldRegistry
```

### Full navigation smoke

```bash
python -m sim.scripts.mujoco.product_acceptance --run-plan _ --runner _ --manifest _ --help
```

## MuJoCo LiDAR Backend

`lidar_backend="mujoco_lidar"` uses the official `mujoco-lidar==0.3.3`
distribution installed by the `sim-mujoco` extra:

```bash
uv sync --extra sim-mujoco
```

The LingTu wrapper and call chain are:

```text
src/drivers/sim/mujoco/driver.py
  -> MuJoCoEngine.get_lidar_points()
  -> sim/engine/mujoco/engine.py
  -> MuJoCoLidar.scan()
  -> sim/engine/mujoco/lidar.py
  -> mujoco_lidar.MjLidarWrapper.trace_rays()
```

There is no repository-local `mujoco_lidar` compatibility package and the
runtime rejects a package that resolves inside `src`. Product runs also
require the canonical `sim/assets/livox/mid360.npy` scan pattern; startup
fails if the explicit pattern is missing or has the wrong shape or dtype. A
development-only backend may use the explicit legacy fallback, but that backend
cannot satisfy a Product acceptance gate.

`src/drivers/sim/mujoco` stays as the Module/runtime adapter layer. The
simulation engine and sensor implementation stay in `sim/engine` so driver
code does not own MuJoCo physics or LiDAR ray-casting internals.

## Bridge Architecture

Simulation output can enter LingTu through two families of adapters:

- native-DDS adapters used by the current real-equivalent MuJoCo gates;
- ROS/GZ compatibility bridges used by legacy or delivery-demo gates.

Native-DDS acceptance uses `sim/scripts/mujoco/native_dds_sensors.py`,
`native_navigation_acceptance.py`, and related manifest-driven C++ processes.
Those paths publish LingTu-owned DDS types and must stay disconnected from
physical hardware command subscribers.

Compatibility bridges convert simulation engine output into ROS/GZ topics
for gates that explicitly require them:

| Bridge | Backend | Topics |
| --- | --- | --- |
| `GazeboBridgeConfig` | Gazebo/GZ | Topic name mapping for `ros_gz_bridge` |
| `GazeboRuntimeAdapter` | Gazebo ROS | `/lingtu/gazebo/raw/*` to `/nav/*` normalization |
| `GazeboCmdVelAdapter` | Gazebo | `TwistStamped` to `Twist` conversion for Gazebo diff-drive |

## Entrypoints

Use the native acceptance scripts under `sim/scripts/mujoco/` for Product
evidence.

## Evidence Boundary

Generated reports belong under `artifacts/`, not under `sim/engine`. Simulation
diagnostics must remain simulation-only and should preserve:

- `simulation_only=true`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`

Do not cite ROS/GZ compatibility bridge evidence as native-DDS equivalence.
Use the dedicated MuJoCo native acceptance reports for that claim.
