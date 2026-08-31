# Current Teleop / Teleop-Avoid Evidence Check

Date: 2026-08-11

Web input contract updated: 2026-08-22. This does not change the field-evidence
status below.

Host: Windows local workspace plus local MuJoCo artifacts under
`C:\Users\99563\.codex\tmp`.

Robot target attempted: `sunrise@192.168.66.13`

## Result

PARTIAL.

The local repository now declares the MuJoCo backend as supporting both
`teleop` and `teleop_avoid`. The `teleop` native MuJoCo diagnostic report from
2026-08-10 is present. The latest local `teleop_avoid`
Product attempts show that the Product can resolve and start the assisted native
chain after the local Windows native nav endpoint artifacts are built, but no
accepted Product report is present.

The Web scene now has a minimal operator-facing Teleop panel. It is gated by
bootstrap `teleop_ws` plus the current Product being `teleop` or
`teleop_avoid`, sends operator velocity samples with deadman semantics, and displays the
Gateway/native ACK scope. This is a Web product-entry fix only; it is not field
motion evidence.

Sunrise was reachable by SSH, but this session could not authenticate:

```text
sunrise@192.168.66.13: Permission denied (publickey,password).
```

Therefore this record is not a Sunrise build, Brainstem, or field-motion pass.

## Local repository state checked

- `config/runtime_graph/envs/sim.yaml` declares:
  - top-level sim support for `teleop`, `teleop_avoid`, and `explore`;
  - MuJoCo backend support for `teleop` and `teleop_avoid`.
- The local Windows native endpoint build was refreshed on 2026-08-11:
  - `build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe`
  - `build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_traversability_dds.exe`
  - `build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_nav_control.exe`
- After those artifacts existed,
  `ProductControl(env="sim", backend="mujoco").resolve("teleop")` resolved a
  subprocess RunPlan with:
  - `driver_bridge`
  - `mujoco_feeder`
  - `nav_runtime`
  - `host_runtime`
- After those artifacts existed,
  `ProductControl(env="sim", backend="mujoco").resolve("teleop_avoid")`
  resolved a subprocess RunPlan with:
  - `driver_bridge`
  - `sensor_publisher`
  - `map_runtime`
  - `mujoco_feeder`
  - `nav_runtime`
  - `traversability_runtime`
  - `host_runtime`

Without the native endpoint artifacts, the same RunPlan resolution fails
closed with `direct process artifact does not exist`; this is expected and must
not be replaced by a Python fallback.

## Web Teleop entry checked

Files added or updated on 2026-08-11:

- `web/src/services/teleopWsClient.ts`
- `web/src/components/TeleopPanel.tsx`
- `web/src/components/TeleopPanel.module.css`
- `web/src/components/SceneView.tsx`
- `web/tests/teleopPanel.test.ts`

Behavior checked:

- Only enables when bootstrap advertises `media.teleop_ws` and the current
  Product is `teleop` or `teleop_avoid`.
- After the operator explicitly connects, W/S/A/D/Q/E are direct hold-to-move
  controls. Shift is an optional 40% precision modifier and Space is immediate
  hold; screen buttons are also hold-to-move controls.
- It sends physical `velocity` samples at 50 Hz using `m/s` and `rad/s`.
  Final-key/button release, Space, blur, page hide, unmount, or close sends zero
  with `deadman=false`, which Gateway treats as hold. Idle browsers send no
  heartbeat. Gateway keeps native Lease, CLAIM, epoch, sequence, and reclaim
  recovery internal; a fresh direction input after hold resumes automatically.
- It displays `ingress_ack`, `control_ack`, `control_rejected`,
  `request_id`, bounded public error/stage, `final_cmd_vel_confirmed`, and
  `motor_confirmed`; raw native receipts and protocol reasons are not exposed.
- `ingress_ack` is presented only as queued operator intent, never as proof
  that the final safe velocity or motor command executed.

## Teleop evidence present

Report:

```text
C:\Users\99563\.codex\tmp\lingtu-teleop-native-run-20260810-final\report.json
```

Scope:

- native typed teleop request path;
- simulated physical driver bridge;
- Driver ACK;
- forbidden Goal rejection;
- stop, cleanup, and zero barrier.

Boundary:

- diagnostic component evidence only;
- not a Brainstem field pass;
- not a Sunrise aarch64 performance pass.

## Teleop-avoid evidence checked

Latest local attempt set:

```text
C:\Users\99563\.codex\tmp\lingtu-teleop-avoid-product-*
```

The newest checked `teleop_avoid` Product attempt directories did not contain a
final `report.json`, `result.json`, `summary.json`, or `acceptance.json`.

Representative Product state:

```text
C:\Users\99563\.codex\tmp\lingtu-teleop-avoid-product-e2e-c90182447eed4065bd90343f13c21331\current.json
```

Important observations from that run:

- Product: `teleop_avoid`
- Env: `sim`
- `mapd.status.json` processed observations and published map layers.
- `traversability.status.json` recorded 485 registered clouds and 436
  `/nav/traversability` publications.
- `nav.status.json` showed:
  - `operator_motion.ack_sent = 0`
  - `teleop.seen = false`
  - `input_gate.reason = driver_control_stale`
  - `driver_control.reason = controller_eof`
  - `driver_acknowledged = false`
  - final command remained zero.

Interpretation:

The assisted Product wiring and fail-closed zero-output behavior are visible,
but this is not evidence that operator input drove `teleop_avoid` through
LocalPlanner, PathFollower, final safety gate, and driver ACK.

## Current blockers

1. `teleop_avoid` needs a fresh Product-level MuJoCo acceptance run that
   produces a final report and proves nonzero operator input, local avoidance,
   final command publication, Driver ACK, stop, cleanup, and zero barrier.
2. Sunrise/aarch64 inspection and build require working SSH credentials or an
   installed key for `sunrise@192.168.66.13`.
3. Brainstem execution is still unverified in current evidence.
4. Native traversability still needs an aarch64 timing check after the current
   fast/slow path work lands.
5. The new Web Teleop panel still needs a browser-driven MuJoCo smoke test and
   then a supervised Sunrise test; the current proof is code-level and unit
   contract level.

## 2026-08-11 verification commands

```powershell
cmake -S src/nav/cpp -B build/nav-cpp/windows-x64-nav-endpoint `
  -DCMAKE_BUILD_TYPE=Release `
  -DLINGTU_NAV_CPP_BUILD_ENDPOINT=ON `
  -DLINGTU_NAV_CPP_BUILD_TESTS=OFF `
  -DLINGTU_NAV_CPP_BUILD_PYTHON=OFF

cmake --build build/nav-cpp/windows-x64-nav-endpoint `
  --config Release `
  --target navd lingtu_traversability_dds lingtu_nav_control `
  --parallel 1 --verbose

.venv\Scripts\python.exe -m pytest sim/tests/test_mujoco_product_acceptance.py -q

cd web
node --experimental-strip-types --test --test-isolation=none tests/teleopPanel.test.ts
npx tsc -b
npx eslint src/services/teleopWsClient.ts src/components/TeleopPanel.tsx `
  src/components/SceneView.tsx tests/teleopPanel.test.ts
```

Results:

- Native endpoint CMake configure: passed.
- Native endpoint build targets: passed with warnings only; 0 errors.
- `sim/tests/test_mujoco_product_acceptance.py`: `20 passed`.
- `web/tests/teleopPanel.test.ts`: `5 passed`.
- Web TypeScript: passed.
- Web focused ESLint: passed.
- Full `npm --prefix web run build -- --mode development` reached Vite after
  TypeScript passed, then stopped at Windows `spawn EPERM` while Vite loaded
  config. This is an environment/tooling blocker, not a TypeScript error.

## Next action

Run the fresh `teleop_avoid` Product acceptance after local contract tests pass,
then repeat on Sunrise once SSH authentication is available.
