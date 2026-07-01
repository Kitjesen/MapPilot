# Module-First Guidelines

> Module is the Node. There is no second tier.

These eight rules govern how new code is added to LingTu. They reflect what
is actually enforced by `core/blueprint.py`, `core/module.py`, and the
existing modules; they are not aspirational.

## 1. Module is the only runtime unit

```
Right:  Module (algorithm + In/Out ports) ï¿?Blueprint orchestration ï¿?lingtu.py
Wrong:  Module + ROS2 Node adapter + launch file ï¿?three layers to maintain
```

Each capability has exactly one implementation, living in a Module.
There must not be paired files like `xxx.py` (ROS2 Node) plus
`xxx_module.py` (Module).

## 2. Blueprint is the only orchestration

LingTu is a library. The entry script (`lingtu.py` ï¿?`cli.main.main`) just
parses CLI flags and calls `full_stack_blueprint(**cfg).build()`. Any
script can do the same:

```python
from runtime.blueprint import autoconnect
from runtime.blueprints.stacks import (
    driver, slam, maps, perception, memory, planner,
    navigation, exploration, safety, gateway,
)

system = autoconnect(
    driver("thunder", dog_host="192.168.66.190"),
    slam("localizer"),
    maps(),
    perception("bpu"),
    memory(),
    planner("kimi"),
    navigation("astar"),
    exploration("none"),
    safety(),
    gateway(5050),
).build()
system.start()
```

`ros2 launch` is not used to start Module-First systems. `ros2 run` is not
used either.

## 3. Communication uses In/Out + Transport

```
Module A â”€â”€Out[Odometry]â”€â”€wireâ”€â”€In[Odometry]â”€â”€ï¿?Module B

transport options (per-wire, see runtime.blueprint._resolve_transport):
  None / "local"  ï¿?direct callback, zero latency (default)
  "dds"           ï¿?CycloneDDS for cross-process decoupling
  "shm"           ï¿?shared memory for high-bandwidth payloads
```

Modules don't `import rclpy.publisher` or `rclpy.subscription`. ROS2 QoS
profiles aren't used either; the framework's backpressure policies
(`latest`, `throttle`, `sample`, `buffer`, `all`) take their place.

## 4. C++ nodes go through NativeModule

A C++ executable becomes a child process supervised by Python:

```python
from runtime.native_module import NativeModule, NativeModuleConfig

class AutonomyModule(Module, layer=2):
    def setup(self):
        self._terrain = NativeModule(NativeModuleConfig(
            executable="terrain_analysis_node",
            parameters={"voxel_size": 0.1},
        ))
        self._terrain.start()  # watchdog + auto-restart
```

C++ binaries stay first-class executables, but they are launched and
restarted from Python ï¿?not from a `launch.py` file.

## 5. Sensors connect directly, no ROS2 driver in the middle

```
Right:  Module calls SDK directly ï¿?publishes Out[Image]
        ThunderDriver ï¿?gRPC ï¿?brainstem
        CameraBridgeModule ï¿?SDK ï¿?Out[color_image]/Out[depth_image]

Wrong:  ros2 run livox_driver ï¿?/livox/lidar topic ï¿?rclpy subscriber
```

If a sensor has a Python SDK, the Module talks to it directly. If only a
C++ driver exists, wrap it in a `NativeModule` and read its data via
SHM/DDS.

## 6. There is exactly one message catalogue: `runtime.msgs`

```python
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.geometry import Pose, Twist, PoseStamped
from runtime.msgs.semantic import SceneGraph, SafetyState
from runtime.msgs.sensor import Image, CameraIntrinsics
```

Modules do not `import sensor_msgs.msg`, `nav_msgs.msg`, or `std_msgs.msg`.
ROS2 message types only appear inside the C++ subprocess of a
`NativeModule`.

## 7. Configuration is constructor parameters, not ROS2 parameters

```python
# Right ï¿?Blueprint config
bp.add(DetectorModule, detector="bpu", confidence=0.5)

# Right ï¿?CLI override
python lingtu.py nav --detector bpu

# Wrong ï¿?ROS2 parameters
self.declare_parameter("detector", "yoloe")
```

Configuration that is shared across modules lives in
`config/robot_config.yaml`, loaded via `runtime.config.get_config()`.

## 8. Pluggable backends use the Registry, not `if`

```python
# Register where the implementation lives
@register("detector", "bpu", platforms={"aarch64"})
class BPUDetector: ...

@register("detector", "yoloe")
class YOLOeDetector: ...

# Resolve where the Module lives
DetectorCls = get("detector", args.detector)
```

Modules do not contain `if name == "bpu": from ... import BPU`.

---

## File naming

```
src/nav/
  navigation_module.py        # Module ï¿?the only implementation
  safety_ring_module.py       # Module ï¿?the only implementation
  cmd_vel_mux_module.py       # Module ï¿?the only implementation

src/perception/semantic_perception/
  perception_module.py        # Module
  encoder_module.py           # Module
  instance_tracker.py         # pure algorithm called by the Module

src/drivers/thunder/
  thunder_driver.py / han_dog_module.py   # Module ï¿?gRPC client
```

Patterns that no longer exist in this repo:

- `xxx_node.py` ROS2 Node entry points
- `xxx.launch.py` launch files for Module-First systems
- `xxx_mixin.py` ROS2 mixins
- `launch/subsystems/` and `scripts/legacy/` (deleted in commits 6fd7257
  and fe99873)

The single exception is the SLAM stack: Fast-LIO2 / Point-LIO / Localizer
are kept as ROS2 C++ nodes because they are deeply coupled to TF and PCL.
They are launched and supervised from `localization.SLAMModule` /
`localization.SlamBridgeModule` via `NativeModule`. Their algorithm-side launch
files live under `launch/profiles/` and are loaded by the Module ï¿?not by
a human running `ros2 launch`.

## Verification checklist

After a clean-up pass, the following should all be true:

1. `grep -r "import rclpy" src/` returns hits only inside SlamBridge / Native
   subprocess Python wrappers.
2. `grep -r "class .*Node" src/` returns hits only in C++-driven helper
   files.
3. `python lingtu.py stub` builds and starts a system with zero ROS2
   dependencies.
4. `python lingtu.py s100p` (or `lingtu.py nav`) starts on the S100P with
   SLAM running through `NativeModule`.
5. `python -m pytest src/runtime/tests/ -q` passes (around 1000 framework
   tests; no `rclpy` required).
