# LingTu MuJoCo Runtime

This directory contains the hardware-free, pure C++ MuJoCo runtime used by
RobotSimUE and headless host processes. MuJoCo remains the physics and simulation
clock authority. The runtime has no Unreal, DDS, ROS, or LingTu Host dependency.
It is the Physics Runtime layer of
[`docs/architecture/SIM_RUNTIME_CONTRACT.md`](../../../docs/architecture/SIM_RUNTIME_CONTRACT.md).

## Interface

`lingtu::sim::MujocoRuntime` owns one `mjModel` and one `mjData`. It is a
single-owner runtime object: every call must come from the same physics thread. Model
loading allocates the snapshot body registry once; `advance()` and `reset()`
reuse that storage.

`lingtu::sim::PhysicsSceneComposer` is the session-level composition seam. It
consumes a typed `PhysicsScenePlan`, attaches every robot to one world
`mjSpec` with an instance namespace, compiles one `mjModel`, and emits a
generation-scoped `ModelDescriptor`. It does not read YAML or own package
resolution. `MujocoRuntime::from_plan()` owns the composed model and its single
`mjData`, so all robot instances advance on one physics clock.

Each RobotPackage may declare an optional `physics.initial_keyframe`. The
Compiler carries it into `physics.plan.json`; the Composer resolves the
instance-prefixed MuJoCo keyframe, combines its free-joint pose with that
instance's spawn transform, and materializes one session-wide initial state.
`reset()` restores that same composed state for every robot. UE does not own or
reconstruct the nominal pose.

Scenario-owned entities with a compiled `physics_proxy=kinematic` are attached
to the same model as MuJoCo mocap bodies. The process adapter sends one atomic,
generation-stamped pose batch containing both `entity_id` and the resolved
`body_stable_id`. The runtime rejects foreign sessions, stale or future
generations, duplicate/out-of-order sequences, clock mismatches, and incomplete
or unknown entity sets before changing `mjData`. Accepted proxy geometry is
therefore visible to MuJoCo contact and raycast queries while robot bodies remain
under MuJoCo authority. `reset()` restores compiled proxy transforms and clears
the per-generation sequence history.

`sim/runtime/coordinator/` validates `physics.plan.json` by schema, `session_id`,
package identity, referenced paths, and structure. It then translates the plan
into typed process arguments and drives the headless host protocol. JSON parsing,
run allocation, PIDs, and logs stay outside this C++ Runtime.

Snapshots use the canonical MuJoCo representation:

- right-handed coordinates;
- metres;
- quaternion order `w, x, y, z`;
- monotonically increasing `sequence` inside one `reset_generation`;
- `model_generation` identifies the dense-index generation used by a
  descriptor; consumers must refresh cached indices when it changes;
- composed body identity uses `instance_id/frame_id`, independent of temporary
  MuJoCo dense indices and compiled `instance_id__body` names.
- each body publishes world-frame linear and angular velocity together with
  pose, allowing truth odometry and sensor runtimes to consume physics data
  without estimating motion from rendered frames.

An Unreal Adapter must copy a snapshot into its own double or ring buffer and
perform coordinate conversion on the Game Thread. It must never expose
`mjData` to Unreal code.

## Build

Use a pinned official MuJoCo SDK with headers, an import/static library, and
the matching runtime DLL. The repository's Python lock currently uses MuJoCo
3.10.0, so the C++ build should use the matching SDK. The Python wheel is
useful for inspecting models but normally contains `mujoco.dll` without the
MSVC import library required by this CMake target.
CTest validates a minimal falling-body fixture, ThunderV4, and a real
OpenField + ThunderV4 + OmniCart session compiled into one `mjModel`.

```powershell
cmake -S sim/runtime/physics -B build/mujoco-runtime `
  -DMUJOCO_ROOT=C:/path/to/mujoco-3.10.0
cmake --build build/mujoco-runtime --config Release
ctest --test-dir build/mujoco-runtime -C Release --output-on-failure
```

Run the headless smoke executable against a real robot model:

```powershell
build/mujoco-runtime/Release/lingtu_mujoco_headless.exe `
  sim/robots/doso/thunder_v4/mjcf/thunderv4.xml 100
```

Run a resolved multi-robot session through the Runtime Coordinator:

```powershell
python -m sim.catalog `
  sim/scenarios/catalog/thunder_omni_contract/session.yaml `
  --repo-root . --output-dir build/runtime-session

python -m sim.runtime.coordinator build/runtime-session `
  --repo-root . `
  --run-root build/runtime-runs `
  --mujoco-host build/mujoco-runtime/Release/lingtu_mujoco_headless.exe `
  --steps 5 --reset
```

The host starts paused in `READY` and accepts `start`, `advance N`, `pause`,
`reset`, `snapshot`, and `stop`. Each command returns one JSON event; MuJoCo
diagnostics go to the run log and never share the protocol stream.

Export a named MuJoCo reference pose for a visual-runtime staging step. The
consumer receives body truth only and does not parse the source model:

```powershell
build/mujoco-runtime/Release/lingtu_mujoco_snapshot.exe `
  sim/robots/doso/thunder_v4/mjcf/thunderv4.xml v4_nominal_stand `
  > build/unreal-assets/thunderv4-nominal.snapshot.json
```
