# UE5 Playable Pre-Acceptance — 2026-08-12

Status: **NO-GO for playable qualification; offline rebuild and Automation GREEN**

This record covers the fixed FactoryPark/ThunderV4 Windows Editor slice. It does not
qualify the UE5 playable product and does not describe SimStudio Web as a game UI.
RobotSimUE Runtime remains the spatial play surface; UE Create and SimStudio retain
their separate authoring and management responsibilities.

## Fixed identity

- Bundle: `build/session-bundles/thunderv4-factory-park-hf-playable-v101-provenance-20260811`
- Session digest: `69d372ad9e5229cedf4feb0f22ac00e50fb67e4e6d5995f1657997dfee68518b`
- Robot: `thunderv4@1.0.1`, instance `thunder_01`
- World: `factory_park_hf@1.1.0`
- Controller: `thunderv4_locomotion@1.0.0`
- Runtime: UE 5.8.1 Editor + owned MuJoCo physics authority

## Native build and Automation

The current camera/UI/Session/Visual/Sensors worktree compiled as
`RobotSimUEEditor Win64 Development`. The optional Create-only
`Tripo3DUEBridge` was disabled for this Runtime/test build with UBT's singular
`-DisablePlugin=Tripo3DUEBridge`; its missing authoring SDK was not hidden by a
Runtime dependency claim.

| Evidence | Result | SHA-256 |
| --- | --- | --- |
| `build/unreal-tests/ubt-owned-direct-20260812-camera-timeout-r3.log` | UBT succeeded; camera production code and C++ Automation compiled and linked | `117374a0f8245c7e1fa46a38abc9ab43f589d442c0bc5c2f4f9268f244231d2c` |
| `build/unreal-tests/automation-camera-timeout-20260812/report/index.json` | 11 succeeded, 0 failed, 0 not-run | `8c7f67251442f77d13e75ea33c98df3c3ff982a41e8435646f02aa4ff975f5c1` |
| `build/unreal-tests/automation-lingtusim-camera-timeout-20260812/report/index.json` | 86 successful states: 67 clean, 19 with expected test-log warnings; 0 failed/not-run | `6ae92968cd984670635b0060632c1aa0f34fc38feb7bf08d2fde8a31e9cd6588` |

The full report includes successful `Runtime.BundleLoader`,
`Sensors.CameraCapture.ReadbackDeadlinePolicy`,
`UI.Runtime.HudScreenshotArgumentsAreExactTriple`, Visual atomic rebind/readiness,
Session, scenario, control, and sensor tests.

Current-tree Python adjacency verification passed 158 tests across camera contracts,
SessionHost, external evidence, RuntimeCoordinator, and the manual launcher. Selected
Ruff `E/F/I/S` checks and `py_compile` passed. Repository-default Ruff still reports
18 pre-existing public-docstring findings in the manual diagnostic launcher; this
record does not call the whole repository lint-clean.

## Latest live evidence

Run `manual-20260812-053817-af85bfa7` failed during `prepare()` before `RUNNING`:

```text
thunder_01.front_rgb: asynchronous GPU readback exceeded its 1.000 second pipeline deadline
```

Authoritative files:

- `build/manual-playable-runs/manual-20260812-053817-af85bfa7/episode_result.json`
- `build/manual-playable-runs/manual-20260812-053817-af85bfa7/session.runtime.json`
- `build/manual-playable-runs/manual-20260812-053817-af85bfa7/logs/sensor-readiness.json`
- `build/manual-playable-runs/manual-20260812-053817-af85bfa7/logs/Unreal.log`

That run produced no short-gate, window-presence, performance, runtime-health, truth,
motion-input, or strict qualification evidence. It therefore proves neither
performance nor playability.

The root cause was a single one-second deadline shared by cold deferred submission,
render-thread enqueue, GPU readback, and even CPU-ready consumption. The replacement
freezes a 30-second per-stage startup budget before the first published frame, keeps a
one-second steady-state budget, starts the GPU clock at the actual render-thread copy,
and never times out `CpuReady`. This replacement is what the successful build and
Automation reports above verify. It has not yet passed a live cold camera startup.

## Remaining acceptance sequence

1. Start from a clean UE/UBT/UAT/MuJoCo/dotnet/compiler process table and revalidate
   the exact provenance-bound bundle.
2. Run one owned Runtime session and require all four facets plus all five sensor
   streams to be current-generation ACTIVE.
3. Execute exactly one 12-second diagnostic performance gate with pre/post owned
   window presence, median real-time factor at least `0.8`, and RGB/depth each at
   least `29 Hz`.
4. Only after that gate passes, run the 60-second stability gate and the separate
   exact-foreground `Shift+W` upright-motion/release-stop gate.
5. Complete the fixed six-maneuver recording/HUD/shutdown evidence bundle and run the
   independent strict verifier.

Any failure consumes that run. It forbids later phases and cannot be repaired by
combining artifacts from another run. At the end of this record's verification, the
UE/UBT/UAT/MuJoCo/dotnet/cl/link/msbuild process table was clean.
