# Dart/Rust Package Migration Plan

LingTu can move toward a Dart/Rust product shape, but not by rewriting every
directory into one language. The target shape is:

- Rust owns robot-side runtime, adapters, tooling, and new portable kernels.
- C++ native engines stay behind C ABI or process ABI when they are already the
  best implementation for SLAM, PCT, terrain, media, or vendor hardware.
- Dart owns operator apps, high-level SDKs, and future Flutter surfaces.
- Python remains the compatibility runtime until package parity is proven.

This lets deployment become simpler without breaking the current Module-first
architecture while the migration is in progress.

## Final Shape

```text
Operator device
  Dart/Flutter app
  Dart SDK
        |
        v
Robot-side stable contracts
  REST / WS / SSE / process ABI / C ABI
        |
        v
Rust runtime + Rust adapters + portable kernels
        |
        v
Native engines where appropriate
  FastLIO2 / PCT / TARE / DUFOMap / media sidecars
```

Python exits only after the Rust runtime can prove the same graph, endpoint,
profile, and fixture behavior.

## Package Order

The machine-readable source of truth is `src/runtime/migration_catalog.py`.

| Phase | Packages | Action |
| ---: | --- | --- |
| 0 | `config`, `launch` | Freeze language-neutral schemas and keep ROS launch as compatibility. |
| 1 | `src/kernels`, `src/nav`, `src/base_autonomy`, `src/localization` | Move compute seams first. Keep heavy SLAM native, but make portable/process ABI explicit. |
| 2 | `src/*/adapters`, `src/adapters`, `src/drivers` | Port stable endpoint and transport adapters to Rust sidecars/crates. |
| 3 | `src/lingtu/plugin_seed.py`, `src/runtime`, `cli` | Build Rust daemon and Rust operations CLI while Python remains authoritative. |
| 4 | `src/gateway`, `src/memory`, `src/lingtu/sdk`, `src/lingtu`, `web` | Move stable product surfaces: Rust gateway/storage, Dart SDK/app. |
| 5 | `src/semantic`, `src/global_planning`, `src/exploration`, `src/gateway/media` | Keep model/native-heavy packages behind process contracts; migrate only pure sub-kernels. |
| 6 | `calibration`, `sim`, `scripts` | Replace repeatable tooling after runtime and kernel contracts are stable. |

## Package Rules

### `config`

Do not migrate config to a language. Treat it as the shared contract consumed by
Python, Rust, and Dart.

### `src/kernels`

This is the first real migration target. Add Rust kernels beside current C++
portable kernels behind C ABI. Good first Rust targets:

- `path_follower`
- `path_safety`
- `waypoint_tracker`

Do not treat the removed FastLIO2 portable path as a reference shape; it was a
lightweight estimator, not a validated LIO implementation.

### `src/nav` and `src/base_autonomy`

Do not rewrite modules first. Extract the algorithm seams first:

- local planner scoring
- terrain analysis
- path following
- safety/path validation
- waypoint tracking

The Python modules continue calling old backends until fixtures prove parity.
Do not migrate `SafetyRingModule`, `CmdVelMux`, or Thunder driver control as
part of the first wave. A new Rust/Dart path may run in shadow mode, but it
must not own `driver_cmd_vel` until the safety chain has passed controlled
hardware soak.

### `src/localization`

Do not directly rewrite full SLAM in Dart or Rust. Keep heavy engines native and
make them portable through C ABI or process ABI only after extracting the real
algorithm and validating it against hardware data.

### `src/runtime`

This is the last major robot-side runtime package, not the first. Rebuild it in
Rust only after kernels and adapters are stable enough to prove:

- same profile graph
- same stream semantics
- same backend selection
- same runtime-spec output
- same health/status behavior

### `cli` and `scripts`

Create `lingtuctl` as a Rust binary. Replace commands one group at a time:

1. `status`, `health`, `runtime-spec`
2. package/build gates
3. endpoint service management
4. map/nav lifecycle
5. robot-only service wrappers

### `src/gateway`, `src/lingtu/sdk`, `web`

Move product surfaces after runtime contracts settle:

- Rust gateway for stable robot-side HTTP/WS/SSE endpoints.
- Dart SDK over the stable gateway contracts.
- Flutter operator app if Dart is desired for user-facing UI.

### `src/semantic`, `src/global_planning`, `src/exploration`, `src/gateway/media`

These are late packages. They depend on ML, optimization, ROS, native engines,
or media stacks. Wrap first, rewrite only isolated pure kernels.

## Acceptance Gates

Every migrated package needs:

- A stable interface contract.
- Golden fixtures or endpoint parity tests.
- Windows x64 smoke.
- Linux x86_64 smoke.
- S100P/aarch64 smoke or a documented hardware gap.
- A fallback path to the previous Python/native implementation.

No package becomes default until the old and new implementations can run in
parallel or produce equivalent fixture outputs.

Control-chain gates are stricter:

- Static contract tests.
- Golden vector parity.
- Windows portable smoke.
- Replay or simulation.
- Non-motion robot-side check.
- Low-speed supervised hardware soak.

Do not let a new implementation publish authoritative `driver_cmd_vel` before
all earlier gates pass.
