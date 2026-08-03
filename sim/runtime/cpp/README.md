# LingTu MuJoCo Runtime

This directory contains the hardware-free, pure C++ MuJoCo runtime used by
future Unreal and headless Adapters. MuJoCo remains the physics and simulation
clock authority. The runtime has no Unreal, DDS, ROS, or LingTu Host dependency.

## Interface

`lingtu::sim::MujocoRuntime` owns one `mjModel` and one `mjData`. It is a
single-owner runtime object: every call must come from the same physics thread. Model
loading allocates the snapshot body registry once; `advance()` and `reset()`
reuse that storage.

`lingtu::sim::PhysicsSceneComposer` is the session-level composition seam. It
consumes a typed `PhysicsScenePlan`, attaches every robot to one world
`mjSpec` with an instance namespace, compiles one `mjModel`, and emits a
generation-scoped `ModelDescriptor`. It does not read YAML or own package
resolution. The Python catalog currently produces the JSON plan; its native
typed loader is a later adapter seam.

Snapshots use the canonical MuJoCo representation:

- right-handed coordinates;
- metres;
- quaternion order `w, x, y, z`;
- monotonically increasing `sequence` inside one `reset_generation`.
- `model_generation` identifies the dense-index generation used by a
  descriptor; consumers must refresh cached indices when it changes.

An Unreal Adapter must copy a snapshot into its own double or ring buffer and
perform coordinate conversion on the Game Thread. It must never expose
`mjData` to Unreal code.

## Build

Use a pinned official MuJoCo SDK with headers, an import/static library, and
the matching runtime DLL. The repository's Python lock currently uses MuJoCo
3.10.0, so the C++ build should use the matching SDK. The Python wheel is
useful for inspecting models but normally contains `mujoco.dll` without the
MSVC import library required by this CMake target.
CTest validates both a minimal falling-body fixture and the repository's full
ThunderV4 MJCF body-pose, fixed-step, pause, resume, and reset contract.

```powershell
cmake -S sim/runtime/cpp -B build/mujoco-runtime `
  -DMUJOCO_ROOT=C:/path/to/mujoco-3.10.0
cmake --build build/mujoco-runtime --config Release
ctest --test-dir build/mujoco-runtime -C Release --output-on-failure
```

Run the headless smoke executable against a real robot model:

```powershell
build/mujoco-runtime/Release/lingtu_mujoco_headless.exe `
  sim/robots/thunderv4/mjcf/thunderv4.xml 100
```
