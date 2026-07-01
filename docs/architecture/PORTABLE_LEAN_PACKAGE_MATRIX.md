# Portable Lean Package Matrix

This matrix tracks the bottom-level dependencies LingTu must remove from the default runtime path while keeping optional capabilities available as explicit packages or sidecars.

## Target

The default LingTu portable package should run on both Windows and Ubuntu/Linux without ROS2, colcon, `/opt/ros`, Gateway, ML/LLM/perception stacks, vendor hardware packages, or native SLAM/PCT stacks in the baseline install.

The immediate portable simulation target is:

```text
portable_mujoco
  -> no ROS2
  -> no Gateway
  -> no semantic/LLM/vector/perception stack
  -> no Thunder vendor driver
  -> MuJoCo only as an explicit simulation dependency
  -> global planning + local planning + sensor smoke remains runnable
```

## Package tiers

| Tier | Intended package | Allowed default posture | Examples |
| --- | --- | --- | --- |
| Core portable | `lingtu-core-portable` | Small cross-platform runtime | `numpy`, `scipy`, `PyYAML`, `pydantic` |
| Simulation portable | `lingtu-sim-mujoco` | Optional desktop simulation feature | `mujoco`, sim assets, MuJoCo sensor engine |
| Endpoint portable | `lingtu-endpoint-portable` | Optional cross-platform endpoint/replay | local transport, JSONL, portable sidecars, optional LCM/TCP |
| Robot optional | `lingtu-robot-thunder` | Explicit hardware package | `brainstem-api`, `grpcio`, Thunder services |
| Native process optional | `lingtu-slam-native`, planner sidecars | Process/C ABI, not imported by core | FastLIO2/PCT/TARE/GTSAM/PCL/Open3D surfaces |
| Heavy feature pack | opt-in extras | Not default | Gateway, perception, LLM/NLP, vector memory |
| ROS compatibility | `lingtu-ros-compat` | Legacy adapter only | `rclpy`, ROS messages, rosbag tools, launch files |
| Dev/test | dev extras | Never runtime baseline | pytest, ruff, mypy, coverage |

## Dependency declaration files

| File / pyproject group | Role | Rule |
| --- | --- | --- |
| `pyproject.toml` base `dependencies` | Core portable Python compatibility layer | Only `numpy`, `scipy`, `PyYAML`, `pydantic` for now. |
| `requirements-core-portable.txt` | Generic Windows/Ubuntu core install target | Mirrors the base dependency set. |
| `requirements-lite.txt` | Existing Thunder Lite compatibility install target | Must continue to mirror core portable while deployment docs migrate. |
| `requirements-sim-mujoco.txt` | Portable MuJoCo desktop simulation target | Includes `requirements-core-portable.txt` plus `mujoco`. |
| `lingtu[sim-mujoco]` | Optional MuJoCo feature | May include `mujoco`; not part of runtime. |
| `lingtu[endpoint-lcm]` | Optional LCM transport feature | May include `lcm`; local/JSONL endpoint path stays dependency-free. |
| `lingtu[robot-thunder]` / `lingtu[thunder]` | Thunder hardware feature | May include `brainstem-api`; never runtime. |
| `lingtu[ros-compat]` | Explicit ROS compatibility marker | Empty pip extra by design; ROS2 system packages are handled by ROS-specific docs. |
| `lingtu[slam-native]` | Native SLAM/planner marker | Empty pip extra by design until sidecar/C-ABI packaging is defined. |

## Dependency classification

| Dependency / family | Current use | Target tier | Action |
| --- | --- | --- | --- |
| `numpy` | Core math/messages/maps/planning | Core portable | Keep in core for now; later Rust kernels can reduce usage in hot paths. |
| `scipy` | Planning/costmap/math helpers | Core portable for now | Keep temporarily; audit exact functions before replacing with Rust/numpy/local implementations. |
| `PyYAML` | Config/profile loading | Core portable | Keep unless config is moved to JSON/TOML-only. |
| `pydantic` | Config/schema validation | Core portable | Keep for Python compatibility; Rust runtime should own schema later. |
| `mujoco` | Desktop simulation/sensor engine | Simulation portable | Keep out of core; require only for `portable_mujoco` / sim package. |
| `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `launch_ros`, `rosbag2_py` | ROS adapters, legacy launches, ROS bags | ROS compatibility | Move/keep under `src/*/adapters/ros2`, launch, conversion tools; never default. |
| `fastapi`, `uvicorn`, `websockets`, `starlette` | Gateway/MCP/dashboard | Heavy feature pack | Keep out of core; Gateway must be opt-in. |
| `opencv-python-headless` / `cv2`, `Pillow` | Image load/encode/calibration/perception/sim video tools | Vision/perception optional | Core messages may keep lazy image file helpers only; no top-level import in runtime. |
| `torch`, `ultralytics`, `open-clip-torch`, `open_clip` | Semantic perception, tracking, research eval | Heavy feature pack | Keep in perception/semantic package only. `runtime.utils.robustness` may keep lazy CUDA cleanup but must not import torch at module import time. |
| `openai`, `anthropic`, `jieba`, `langchain*` | Semantic planner / LLM clients | Heavy feature pack | Keep under semantic package only. |
| `chromadb` | Vector memory | Heavy feature pack | Keep under memory/vector package only. |
| `brainstem-api`, `grpcio`, `protobuf` | Thunder hardware driver and manager | Robot optional | Keep out of core/sim; isolate behind endpoint/hardware package. |
| `lcm` | Optional transport | Endpoint portable optional | Do not import from core unless endpoint transport explicitly selected. |
| `open3d`, `gtsam`, `pcl`, `rerun` | Native SLAM/PCT/calibration/visualization | Native process optional / heavy feature | Replace default imports with process/C ABI or feature pack. |

## Current scan findings

Observed from a local import scan over `src`, `cli`, `sim`, `scripts`, and `tools`:

| Family | Files with imports | Important examples | Migration direction |
| --- | ---: | --- | --- |
| ROS2 | 71 | `src/*/adapters/ros2/*`, `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/*`, `src/localization/*`, launch files | Keep under `ros-compat`; default package excludes. |
| Gateway | 20 | `src/gateway/*`, `scripts/manager/manager.py`, `sim/scripts/gateway_goal_dry_run_gate.py` | Gateway feature pack only. |
| Vision/OpenCV/PIL | 36 | `src/runtime/msgs/sensor.py` lazy file load, `src/runtime/utils/validation.py` lazy resize, semantic/sim video tools | Lazy helpers can stay; package dependency is optional. |
| ML/LLM/vector/perception | 22+ | `src/semantic/*`, `src/memory/*`, `tools/perception/*` | Heavy feature packs only. |
| Vendor hardware | 9 | `src/drivers/real/thunder/*`, gateway status, manager, generated proto tools | `lingtu-robot-thunder` only. |
| MuJoCo | 23 | `sim/engine/*`, `src/drivers/sim/*`, `sim/scripts/*` | `lingtu-sim-mujoco` optional but first-class. |
| Native heavy | 92 | PCT/GTSAM/Open3D/rerun surfaces | Process/C ABI or feature pack; never core import. |
| LCM | 2 | `src/runtime/transport/factory.py`, `src/runtime/transport/lcm.py` | Endpoint optional; guard imports. |

A top-level import scan of the critical portable runtime surface currently found no heavy top-level imports in:

- `cli/`
- `src/runtime/`
- `src/nav/`
- `src/base_autonomy/`
- `src/drivers/sim/` except MuJoCo test/simulation files
- `sim/validation/`

This is good: most heavy packages are already lazy or isolated, so the next work is packaging boundaries and validators, not a risky deletion spree.

## Immediate replacement / isolation priorities

1. **Generic portable package manifest**
   - Create a `portable_lean` validator before deleting code.
   - Use it to prevent ROS/Gateway/perception/vendor dependencies from entering the portable baseline.

2. **Split MuJoCo from core**
   - Treat `mujoco` as an explicit simulation dependency.
   - `portable_mujoco` may require it, but `portable-lite` should not.

3. **Replace ROS bag default replay**
   - Default replay should be JSONL/portable fixture/MCAP-like.
   - ROS bag readers stay under `ros-compat` or conversion tools.

4. **Isolate PCT/GTSAM/Open3D**
   - Keep PCT/TARE/native planners behind optional process ABI or C ABI.
   - Do not import Open3D/GTSAM from runtime startup.

5. **Reduce SciPy dependency later**
   - Identify actual functions used in portable planning/costmap.
   - Replace small math helpers with Rust kernels or numpy/local code when proven.

6. **Rust kernels first**
   - `src/kernels/nav/path_safety` should enter the unified Rust build next.
   - Later candidates: waypoint/path follower scoring, local planner scoring, costmap helpers.

## Validator

The initial validator is:

```bash
python tools/validate/validate_portable_lean_package.py --profile portable_mujoco --dry-run
```

Use non-dry-run mode to fail CI when the portable baseline becomes heavier:

```bash
python tools/validate/validate_portable_lean_package.py --profile portable_mujoco
```

The validator checks:

- base dependencies remain limited to the core portable set;
- `pyproject.toml` optional groups keep Ubuntu/ROS/native-heavy capabilities explicit;
- `requirements-core-portable.txt` is the generic minimal Windows/Ubuntu install target;
- `requirements-lite.txt` mirrors the core portable set;
- `requirements-sim-mujoco.txt` layers only MuJoCo on top of core portable;
- `portable_mujoco` resolves to a no-ROS/no-Gateway/no-semantic graph;
- the critical portable runtime surface has no forbidden top-level imports;
- MuJoCo stays an explicit simulation feature, not a core dependency.
