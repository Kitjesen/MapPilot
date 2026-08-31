# LingTu Current Status

Status: runtime and documentation snapshot as of 2026-08-19

Active platform decision: Windows x64 and Linux/WSL x86_64 are both required
native MuJoCo Product targets. Windows is not a portable-only or teleop-only
end state. S100P Ubuntu aarch64 remains the separate physical deployment and
field-safety target. The implementation plan is maintained in
`docs/plans/current-roadmap.md`.

## Usability Snapshot

This snapshot separates code-level verification from target and motion acceptance.

| Surface | Status now | Safe use | Still required |
| --- | --- | --- | --- |
| Local MuJoCo/component harnesses | Development/contract-ready | Blueprint/Module development and focused tests | They are not Product lifecycle or hardware evidence |
| Product assembly and switch preview | Contract-ready | Resolve one Product in fixed `env`, inspect its resolved RunPlan, and copy the ProductControl command | A preview never proves the selected host processes started or became ready |
| Gateway, SDK, and map HTTP contracts | Integration-ready | Client development, auth/route contracts, saved-map queries, and read-only switch preview | Typed native map control/query migration is still active |
| ProductControl and env runners | Code and contract verified | Review and test switch, stop, readiness, rollback, and conflict cleanup transactions | Execute one exact transaction per Product and host platform; component runners are not Product evidence |
| Windows-native MuJoCo Product chain | W1-W3 build/compile gates passed; Product acceptance pending | Reproduce the pinned MSVC x64 SLAM build, inspect its staged source/file inventory and SBOM, and compile the eight Windows RunPlan targets | Run every exact Product lifecycle, scenario, cleanup, rollback, and repeatability gate on Windows |
| Linux/WSL-native MuJoCo Product chain | W1 CTest/sanitizer gate passed; Product acceptance pending | Build/test native ELF artifacts in a Linux-only build tree | Complete the Linux/WSL exact Product matrix independently of Windows |
| Native C++ LiDAR/SLAM/map/navigation/driver chain | Windows raw LiDAR/IMU-to-Fast-LIO slice revalidated; full Product chain partial | Reproduce focused native builds and typed readiness diagnostics | Complete W5 on both workstation platforms, then run the separate S100P gates |
| S100P field autonomy and motion | Not accepted | No-motion diagnostics only after deployment | Fresh release provenance, no-motion readiness, fault injection, then bounded supervised motion |

All nine current MuJoCo acceptance manifests that declare `acceptance_scope`
use `coverage=component`; the other two MuJoCo acceptance JSON files do not
declare Product coverage. The current exact Product PASS count is zero.
Overall, the repository is usable for local development and
interface/architecture validation. It is not yet evidence that the current
checkout is a complete Windows/Linux MuJoCo Product release or is
production-ready for autonomous S100P motion.

The Fast-LIO ikd-tree source no longer directly uses `pthread`, `unistd`, or
`usleep`; its narrow concurrency boundary now uses the C++17 standard library.
The retained synchronization covers rebuild/read access, worker shutdown, and
mutation safety. The port also fixes split-axis equal-point deletion,
`working_flag` lifetime, and empty rebuild replay. The cleanup removed
swallowed background exceptions, 100-microsecond polling, the global
range-query lock, the unused benchmark, and excessive fault injection. The
focused CTest passed 20 repetitions on both Windows and Linux; the Linux run
also passed ASan/UBSan and leak checks. Native Linux TSan and dedicated S100P
performance evidence remain open. W1B is also complete: the DDS production runtime fails
configuration when the real Fast-LIO2 backend is disabled, with 45 focused
contract tests passing.

W2 is complete at the Windows build and runtime-integration gate. A fresh build
from the pinned official CycloneDDS 11.0.1 source published and verified a
Visual Studio 2022 `v143` x64 Release `/MD` SDK at
`third_party/integration/cyclonedds-windows-20260817-d/sdk`. Its
build-source/file-inventory record contains
commit `e54e991f75a3e67f8e628da3171122e36ea5b872`, tree
`56508d35826c362782fc8a388cad351a3d491f51`, and passing PE/x64, DLL closure,
IDL smoke, consumer compile/link, and sanitized DLL-search checks. The pinned
vcpkg MSVC x64 dependency prefix produced real Fast-LIO-enabled `slamd.exe`
and `slamctl.exe`; the native suite passed 7/7 CTest. A raw simulated MID-360
LiDAR/IMU stream drove Fast-LIO to `TRACKING`, `slamctl` returned typed status,
the repository readiness loader accepted the exact Product session ID,
and a DDS domain mismatch failed closed. The fresh formal stage at
`build/slam-core-windows-x64/stage` contains exactly 42 files. Its audit passed
21/21 PE files as x64, found zero unresolved non-system imports, and verified
the unchanged-input reuse path. Its SPDX 2.3 SBOM contains 13 packages, 35
files, and 52 relationships, including the exact `LingTu -> CycloneDDS`
`DYNAMIC_LINK` relationship. The stage records 18 vcpkg runtime DLLs under
their owning packages, `ddsc.dll` under CycloneDDS, and only `slamd.exe` and
`slamctl.exe` under the LingTu local build.

The Windows native loader closure is now consistent across the canonical
MuJoCo adapter, navigation, maps, and staged SLAM outputs. Each executable uses
an app-local `ddsc.dll`; the four canonical copies have SHA-256
`203ece8c0b2c00380f632c0d85380f5381354957e0fb78155e1de8cf7d996887`.
This systemically closes the reported `0xC0000135` / missing-`ddsc.dll` failure
class for canonical outputs. Do not run executables from the former nested nav
output, a copied standalone executable, or an old `-root-d` tree. The old trees
were moved under `C:/Users/99563/.codex/tmp/lingtu-stale-builds` for recoverable
quarantine; there is no supported fallback, alias, or junction to them. The
canonical directories are `build/windows-native-dds-adapter/Release`,
`build/maps-windows/Release`,
`build/nav-cpp/windows-x64-nav-endpoint/Release`, and
`build/slam-core-windows-x64/stage/bin`.
With `PATH` limited to the real Windows `System32`, the staged command smokes
reach their documented usage output rather than a loader failure. This is
loader evidence, not Product acceptance.
Windows still requires the centrally installed Microsoft Visual C++
Redistributable x64. It is not copied into application folders, so these
outputs must not be described as fully self-contained. The Windows build helper
and ProductControl preflight both fail closed unless the 64-bit runtime is
registered and `System32` contains
`msvcp140.dll`, `msvcp140_2.dll`, `vcruntime140.dll`,
`vcruntime140_1.dll`, and `vcomp140.dll` as PE32+/x64 files. The minimum
runtime version is derived from the configured MSVC toolset during build and
from the configured Product artifacts' linker versions during startup.

W3 is complete at the RunPlan compile gate. `sim.yaml` selects the canonical
Windows DDS sensor, driver, mapd, traversability, navd, exploration, and Host
client artifacts plus the staged Windows `slam_runtime`; every selected
executable and DLL dependency participates in platform preflight and the resolved
RunPlan. All seven operator Products, with `explore` compiled in both
`live` and `map` variants, resolve as eight Windows RunPlan targets. Fresh
contract evidence is 86 Product-compile tests, 141 runtime-graph tests, and 55
RunPlan identity/environment tests. Startup uses dependency stages 10, 20, 30,
and 50: sensor/driver at 10; feeder, SLAM, traversability, and nav at 20; mapd
at 30; exploration and Host at 50. This compile result is not Product execution
evidence.

RunPlan schema v4 is the only accepted current format; v3 is rejected with an
instruction to switch again. The plan binds real artifact and dependency paths,
and startup checks PE32+/x64 plus the Registry64/System32
VC++ runtime. The abandoned per-process MSVC scheme and its four redundant JSON
files have been removed. The retained CycloneDDS SDK and SLAM-stage
build-source/file-inventory records and SBOM serve a different purpose: they
describe how those packages were assembled.

The Windows navigation preset requires both `LINGTU_CYCLONEDDS_PREFIX` and
`LINGTU_OCTOMAP_PREFIX` and writes the canonical
`build/nav-cpp/windows-x64-nav-endpoint` tree.

Windows Product processes use DDS domain `17`; Linux Product processes remain
on domain `231`. MuJoCo adapter CTests use isolated low-domain slots
`10`-`16`, `18`, and `19`, reject the reserved Product domain `17`, and reject
domain selections whose CycloneDDS fixed ports enter the Windows dynamic-port
range. These test domains are not Product runtime configuration.

W4's `teleop` exact-transaction coordinator and focused tests received an
independent APPROVE review. This does not promote a Product: the MuJoCo
manifests remain `coverage=component`, no report has
`product_acceptance_passed=true`, and the Product PASS count remains zero. W5
has started and remains in progress.
The fresh Windows exact-`teleop` report at
`build/w5-inprocess-exact-teleop-c1d33a584d4f42aa98cf8a767ae320e6/report.json`
has `ok=true` and `blockers=[]`: switch, all four typed-readiness gates, the
attach-only scenario, physical motion (0.164 m path, 0.158 m net displacement),
terminal zero, stop, cleanup, and fault-injected rollback all passed. This is
the first technical exact transaction closure, not a promoted Product result:
the report is `evidence_scope=component_e2e`, the manifest remains
`coverage=component`, `product_acceptance_passed=false`, and repeatability is
still running. Windows and Linux/WSL evidence must be produced independently;
neither platform satisfies the other's gate.
Fresh component evidence is maps 32/32, the 17 selected navigation tests for
three consecutive runs, SLAM 7/7 plus formal staging, adapter endpoint
apply/ACK 50/50, adapter bridge stress 20/20, and the fresh complete adapter
suite 18/18. The IMU test fix adds best-effort DDS discovery warm-up and a read
condition only; production code is unchanged. Do not cite the loader fix as
Product acceptance.

Long-term platform decision: Windows x64 is a first-class runtime target, not a
development convenience. Cross-platform algorithm and concurrency code must use
the shared standard C++17 boundary; platform-specific executables and dependency
closures must be declared explicitly in the RunPlan. Missing or wrong-platform
artifacts fail closed. WSL, mock, contract-only, simulator-truth, or Python
fallback paths must never masquerade as the selected native runtime.

Successful compilation alone will not clear the release gate. The official
FAST_LIO, ikd-Tree, and IKFoM repositories publish GPL-2.0 licenses, so replacing
only the ikd-tree container is not a closed-source distribution solution.
The repository retains upstream licenses and origin notices, but no local hash
or source-lock file is treated as release authorization. Before any Windows or
Linux distribution, document commercial
authorization, a reviewed GPL-compliant release boundary, or a clean-room
replacement decision. Local metadata alone cannot clear the release gate.

The remaining promotion order is maintained in `plans/current-roadmap.md`; dated simulation and field evidence lives under `07-testing/`.

## Documentation Map

Use this file when deciding which document is authoritative.

For documentation placement and the deletion ledger, see `DOCS_TRIAGE.md`.

## Curated Documentation Entry Points

The pages below are task-oriented navigation aids. They do not supersede the
contracts and references listed later in this file.

| Need | Curated entry point |
| --- | --- |
| Understand the inspection product and result-acceptance model | `product/README.md` |
| Choose a local, simulation, or field starting path | `01-getting-started/README.md` |
| Learn Product, Host, Blueprint, Module, and DDS concepts | `02-concepts/README.md` |
| Find the owning development surface | `03-development/README.md` |
| Build a REST, SDK, MCP, SSE, or teleoperation integration | `09-integrations/README.md` |
| Map, navigate, use semantic goals, or explore | `05-guides/README.md` |
| Operate and diagnose a running system | `06-operations/README.md` |
| Understand control ownership, stop/recovery, and movement boundaries | `10-safety/README.md` |
| Prepare a field target without target-specific addresses or credentials | `04-deployment/WEB_GUIDE.md` |
| Select a local, simulation, or no-motion field validation gate | `07-testing/WEB_GUIDE.md` |
| Find CLI, REST, MCP, and configuration references | `08-reference/README.md` |

The root `README.md` is the public documentation home. Keep plans, dated audit
reports, and field-run evidence out of its primary reading paths.

## Status Terms

| Label | Meaning | How to use it |
| --- | --- | --- |
| Current | Describes the supported product behavior or an active contract. | It may be used as an implementation or operator reference. |
| Reference | Lists stable interfaces, commands, configuration, or generated inventory. | Check its scope and generation date before relying on an individual entry. |
| Evidence | Records the result of a dated test, simulation run, or field session. | It supports only the named claim and environment. |
| Plan | Describes intended work, not shipped behavior. | Do not cite it as a runtime contract. |
| Historical | Retained context that requires revalidation before reuse. | Prefer a current replacement or git history. |

## Authoritative Now

| Topic | Source of truth |
| --- | --- |
| Inspection product intent and acceptance semantics | `docs/product/inspection-product.md` |
| System architecture | `docs/architecture/SYSTEM_DESIGN.md` |
| Module, Blueprint, Port/Wire model | `docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md` |
| Runtime Graph env/Product/topic contract | `config/runtime_graph/README.md` |
| Global planning input/output | `docs/architecture/GLOBAL_PLANNING_CONTRACT.md` |
| Saved map types and artifact bundles | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Localization sources, relocalization, DDS topics, loop closure, and SaveMap flow | `docs/architecture/LOCALIZATION_RUNTIME.md` |
| Navigation compute chain | `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md` |
| Navigation capability maturity and evidence gaps | `docs/architecture/NAVIGATION_CAPABILITY_MATRIX.md` |
| Native navigation/map data-plane migration boundary | `docs/architecture/NATIVE_DATA_PLANE_MIGRATION.md` |
| Local planner I/O | `docs/architecture/local_planner_io_contract.md` |
| Frame contract | `docs/architecture/ros_frame_contract.md` |
| Repository placement | `docs/REPO_LAYOUT.md` |
| Runtime quickstart | `docs/QUICKSTART.md` |
| SDK, REST, MCP, SSE, and teleoperation integration usage | `docs/09-integrations/README.md` |
| Software control ownership, motion gate, and stop/recovery usage | `docs/10-safety/README.md` |
| Known product gaps | `docs/known_gaps.md` |
| Active cross-platform MuJoCo delivery plan | `docs/plans/current-roadmap.md` |
| Windows/Linux MuJoCo architecture and evidence boundary | `docs/architecture/SIMULATION_INTEGRATION_CONTRACT.md` |
| Portable and native build commands | `docs/01-getting-started/BUILD_GUIDE.md` |
| Physical runtime Env | `config/runtime_graph/envs/real.yaml` |
| Native DDS IDL and typed message contracts | `src/message/README.md` |
| Native field-driver deployment boundary | `scripts/deploy/thunder/lt-driver.service` |
| MuJoCo native-DDS navigation acceptance | `docs/07-testing/simulation/MUJOCO_NAVIGATION_ACCEPTANCE.md` |
| Native endpoint control-mode promotion gate | `docs/07-testing/simulation/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md` |

## Not Authoritative By Default

| Location | How to use |
| --- | --- |
| `docs/research/` | Upstream evaluations and algorithm investigations. Not product contract or acceptance evidence. |
| `docs/07-testing/field-runs/` | Dated evidence snapshots. They support only the named run and environment. |
| `docs/plans/` | Forward-looking PRDs and migration plans. Not shipped behavior. |
| `.qoder/`, `.hermes/`, `.codex/`, `.omx/` | Tool-generated workspace memory. Regenerate with the owning tool; do not treat as product documentation. |

## Current Product Defaults

| Area | Current position |
| --- | --- |
| Host runtime unit | `Module` |
| Host graph assembly | `Blueprint` |
| Product operation | `ProductControl(env).switch(Product) -> RunPlan -> env runner` (`systemd` for the real env; supervised subprocesses for the MuJoCo sim env) |
| Dataflow model | `Port -> Wire -> Transport` |
| Product global planner | Native `octoplanner3d` by default; native `far` when explicitly selected |
| Saved-map identity | Native C++ `MapStore` owns separate `map_id` and positive `content_epoch` fields; `map:v123` / `map:e123` are invalid, and `map:e...` was never a repository protocol |
| PGO / HBA | Legacy ROS2 PGO/HBA are removed. Fast-LIO2 freezes map/poses/body-local patches/manifest; SaveMap runs ROS-free `lt_pgo --auto-constraints`, which optimizes only with a complete measured adjacent chain plus at least one verified loop. Non-ready graphs preserve raw source with an exact skip code; the private temporary `pose_graph.constraints` is not published. Field quality/performance acceptance is outstanding. |
| UI global path payload | `lingtu.global_plan.v1` |
| ROS 2 role | Compatibility, replay, benchmark, and legacy gate adapter only |
| Physical runtime env | `real` |
| Field command output | `endpoint_only`: logical `/nav/cmd_vel`, DDS wire `rt/nav/cmd_vel`, then `lt-driver.service` |
| Robot motion boundary | `RobotConfig.driver.backend`: Go2 SDK2 by network interface or Doso through the Brainstem Client SDK |
| Gateway / MCP defaults | Gateway `5050`, MCP JSON-RPC `8090` |

## Current Evidence Boundaries

| Claim | Minimum current source |
| --- | --- |
| Local framework health | Focused Python tests and `stub`/`dev` profile inspection |
| MuJoCo native navigation/control behavior | `docs/07-testing/simulation/MUJOCO_NAVIGATION_ACCEPTANCE.md` and matching artifacts/manifests |
| Windows-native MuJoCo Product support | A Windows exact Product report whose selected native artifacts are PE/DLL-only; WSL/Linux evidence does not satisfy this claim |
| Linux-native MuJoCo Product support | A Linux exact Product report whose selected native artifacts are ELF/SO-only; Windows evidence does not satisfy this claim |
| Native endpoint control-mode promotion | `docs/07-testing/simulation/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md`; handwritten JSON summaries are not promotion evidence |
| Physical robot readiness | Fresh field-run evidence under `docs/07-testing/field-runs/`, ProductControl status, and `diagnostics.field.doctor` |
| Historical context | Dated audit, plan, and field-run files only; revalidate before citing as current behavior |

## Current Lab Address Note

Reusable docs should use `$LINGTU_ROBOT_HOST`, `$ROBOT_HOST`, or `<robot-ip>`.
Keep the active lab target in the operator environment or a dated, access-
controlled field record. Do not embed live field endpoints in public or
Web-bundled documentation.
