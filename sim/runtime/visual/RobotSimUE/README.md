# RobotSimUE

RobotSimUE is the single UE 5.8 host project for LingTu simulation. The Editor target
and all five LingTuSim C++ modules compile on Windows, and the checked-in offline
preview pipeline can materialize the ThunderV4 MJCF visuals in an Unreal scene. The
first live Physics-to-Visual slice also streams immutable MuJoCo truth snapshots over
localhost UDP into the Session mailbox and applies all 21 ThunderV4 body bindings.
RGB/depth render sensors and the exact runtime sensor evidence path are implemented;
the fresh end-to-end playable qualification and a cooked Windows package remain in
progress.

## Boundary decisions

- The native runtime is the only physics and simulation-clock authority.
- UE consumes compiler-produced SessionBundle/Plan views and immutable snapshots.
- The plugin does not parse runtime configuration files or source robot models.
- No simulator-owned data pointer crosses into UE; the physics boundary is opaque.
- One project and one plugin support every robot package; package IDs never create robot-specific Runtime branches.
- UE never writes an Actor transform as physics truth; it applies immutable snapshots for presentation.

## Blueprint presentation status

Blueprint presentation logic can obtain the world's
`LingTu Sim Blueprint Status` subsystem, call `Get Latest Status` for the initial
snapshot, and bind `On Runtime Status Changed` for later updates. The adapter is
read-only: it samples the centralized Session/UI/Visual status at 10 Hz and broadcasts
one structured payload only when that projected status changes.

The payload deliberately keeps requested operator axes, control-admitted velocity, and
observed MuJoCo truth in separate fields. Blueprint widgets and presentation effects may
react to these values, but the subsystem does not publish control requests, mutate robot
Actors as physics truth, or create another runtime authority.

## Compiled artifact boundary

The session runtime may consume only these compiler-produced JSON artifact names:

~~~text
physics.plan.json
visual.plan.json
sensor.plan.json
control.plan.json
scenario.plan.json
transport.intent.json
~~~

Each plan carries the same `session_id`. RobotSimUE validates plan schemas,
session and package identities, referenced paths, and projection structure. It
must not parse a robot model, RobotConfig, the original `session.yaml`, MJCF, or
other source manifests at runtime. Pixel Streaming, when added, will remain a
presentation/input adapter and not an algorithm sensor interface.

## Windows validation state

The current camera/Session/Visual/UI/Sensors worktree was rebuilt on 2026-08-12 with
Unreal Engine 5.8.1, Visual Studio 2022 17.14/MSVC 14.44, and Windows SDK
10.0.26100.0. The camera Automation report passed 11/11 and the full `LingTuSim`
report passed 86/86. This is current Editor build and contract evidence, not a claim
that cooking, packaging, or the end-to-end playable run is complete.

The current machine supplies the .NET Framework reference SDK to UnrealBuildTool through
the supported AutoSDK root `D:\Development\UnrealAutoSDK`. The preview launcher discovers
that path automatically, respects an existing `UE_SDKS_ROOT`, or accepts an explicit
`-UnrealAutoSdkRoot`.

To build directly from this directory:

~~~powershell
$env:UE_ROOT = 'D:\Program Files\Epic Games\UE_5.8'
$env:UE_SDKS_ROOT = 'D:\Development\UnrealAutoSDK'
& "$env:UE_ROOT\Engine\Build\BatchFiles\Build.bat" RobotSimUEEditor Win64 Development "$PWD\RobotSimUE.uproject" -WaitMutex -NoHotReloadFromIDE
~~~

Runtime/test builds disable the optional Create-only `Tripo3DUEBridge` with UBT's
singular `-DisablePlugin=Tripo3DUEBridge`. Runtime launches use the separate engine
runtime switch `-DisablePlugins=...`. Do not silently replace one spelling with the
other: they are parsed by different layers.

The narrow structural verification remains:

~~~powershell
python -m pytest sim/tests/test_robotsimue_skeleton.py -q
~~~

## Unreal MCP and integrated Terminal

The UE 5.8 Editor developer surface uses Epic's official
`ModelContextProtocol`, `AllToolsets`, and `Terminal` plugins. MCP is editor-only,
binds to the local loopback interface, and is not part of the simulation or robot
runtime.

Use the checked-in developer entry point from the repository root:

~~~powershell
& .\sim\runtime\visual\RobotSimUE\Scripts\ue_dev.ps1 status
& .\sim\runtime\visual\RobotSimUE\Scripts\ue_dev.ps1 build
& .\sim\runtime\visual\RobotSimUE\Scripts\ue_dev.ps1 editor
& .\sim\runtime\visual\RobotSimUE\Scripts\ue_dev.ps1 mcp
& .\sim\runtime\visual\RobotSimUE\Scripts\ue_dev.ps1 terminal
~~~

`editor` launches `RobotSimUE` with the MCP server on
`http://127.0.0.1:8000/mcp`. `mcp` performs a real protocol handshake and checks
the three discovery tools plus the aggregate toolset registry. The Editor's
**Tools > Terminal** tab starts PowerShell in the repository root and launches
Codex. The `terminal` action is the external fallback using the same directory.

Codex reads the repository-local `.codex/config.toml` entry named
`unreal-mcp`. A Codex task that was already running before the MCP configuration
was added must be reloaded or started again before the new tool connection appears
in that task; this is client connection lifecycle, not an Unreal server failure.

## ThunderV4 offline preview entry

`Scripts/run_thunderv4_preview.ps1` is an offline preview path for staging a
single ThunderV4 screenshot from compiler/session artifacts and one immutable
Physics Runtime snapshot. It is not the live runtime, does not stream simulator
state into UE, and must not be used as evidence of real-time physics or sensor
operation.

The entry accepts an explicit Physics Runtime host. `-MujocoHost` runs the
compiled `physics.plan.json`; the RobotPackage's optional
`physics.initial_keyframe` is resolved into that plan and applied by the
session-level Composer. The preview has no second hard-coded nominal pose:

~~~powershell
.\Scripts\run_thunderv4_preview.ps1 -MujocoHost C:\path\to\lingtu_mujoco_headless.exe
~~~

If `-MujocoHost` is omitted, it builds the canonical Windows Physics Runtime
host from `sim/runtime/physics` into `build/mujoco-runtime-physics-win` and
passes `-MuJoCoRoot` through to CMake when supplied:

~~~powershell
.\Scripts\run_thunderv4_preview.ps1 -MuJoCoRoot C:\path\to\mujoco-sdk
~~~

When UnrealBuildTool needs a non-system .NET Framework SDK, pass the AutoSDK root:

~~~powershell
.\Scripts\run_thunderv4_preview.ps1 -UnrealAutoSdkRoot D:\Development\UnrealAutoSDK
~~~

The default mode keeps the Editor visible for inspection. Add `-Unattended` to
exit the Editor after the screenshot success sentinel is written. Generated UE
materials are reused by default; add `-RebuildMaterials` only after changing the
material recipe.

## First vertical slice status

1. Completed: resolve ThunderV4 into a deterministic six-artifact SessionBundle.
2. Completed: compile the Windows Editor target and create an immutable Physics Runtime snapshot.
3. Completed: bind generated visual assets by stable IDs and render the first Unreal screenshot.
4. Completed: load SessionBundle and truth snapshots natively in UE with session and generation validation.
5. Completed for the first Windows slice: stream latest-only truth snapshots over
   `127.0.0.1` UDP, publish them through the Session mailbox, and apply one whole frame
   to 21 stable-ID body bindings on the game thread.
6. Completed for the Windows Editor slice: bind RGB/depth render capture plus the
   exact IMU, Mid360, and truth-odom sensor evidence behind `sensor.plan.json`.
7. Completed for the component slice: publish UE viewport input through the Session
   control-intent boundary and display authoritative runtime status in the HUD.
8. Completed: rebuild the current UE modules and pass 11/11 camera plus 86/86 full
   `LingTuSim` Automation tests after the cold-first-frame deadline fix.
9. Next: run one fresh provenance-bound 12-second performance gate. Only after it
   passes may the 60-second stability and foreground `Shift+W` upright/release-stop
   gates run; strict playable qualification remains the final authority.

The offline screenshot remains a nominal-keyframe visual proof and must not be described
as a live-physics capture. Live-runtime evidence is the independent UE execution log with
`LINGTU_LIVE_SNAPSHOT_FIRST` followed by `LINGTU_VISUAL_FRAME_APPLIED`. The localhost
JSON transport is an internal first-slice implementation; it does not replace typed DDS
for algorithm contracts or shared memory for future RGB/depth payloads.
