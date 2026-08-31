# LingTu Current Roadmap

Status: active product roadmap
Updated: 2026-08-25

The target is a native typed-DDS field Product runtime with one owner for every
control-critical output. ROS 2 remains available only through explicit
compatibility, replay, calibration, simulation, and diagnostic entrypoints;
it is not a default Product dependency. Windows x64 and Linux/WSL x86_64 are
both required native MuJoCo Product platforms; S100P Ubuntu aarch64 remains the
separate field-deployment target. This file contains remaining work only.
Shipped contracts live in `docs/architecture/`; run results live in
`docs/07-testing/field-runs/`.

## Fixed Product Boundary

```text
LiDAR / IMU
  -> native sensor process
  -> native SLAM
  -> MapObservation + odometry + registered cloud
  -> mapd (map state and scene) + traversability (control-risk grid)
  -> navd (planning, local avoidance, tracking, final command gate)
  -> /nav/cmd_vel
  -> driver
  -> Brainstem

HostBus <-> Gateway / Agent / MCP
```

Ownership is fixed:

- `navd` is the only field navigation state and `/nav/cmd_vel` writer.
- standalone native traversability is the only `/nav/traversability` writer;
  mapd must not compete for that control topic.
- `mapd` owns realtime map ingestion/layers/scene and all persistent map
  management, including SaveMap and artifacts. ProductControl alone owns
  public map activation.
- the Python Host translates operator/API traffic and consumes native state. It
  does not run a second field planner, map hot path, or command writer.
- Blueprint assembles Modules inside one Host process only. ProductControl
  and its internal SystemdRunner own Product/process lifecycle.
- every field Product that declares a Python Host carries one Blueprint graph
  in the same RunPlan; Product-specific Host lifecycle forks are
  not part of the architecture.
- missing or stale native components fail startup; there is no field Python
  fallback.

## Fixed Platform Decision

Windows-native MuJoCo parity is a P0 delivery requirement, not an optional
follow-up after Linux/WSL:

- all seven operator Products must resolve and run natively on Windows x64 and
  Linux/WSL x86_64, with `explore` accepted separately in live/map variants;
- selected native artifacts in a Windows RunPlan must be PE/DLL, while selected
  native artifacts in a Linux RunPlan must be ELF/SO; shared interpreter-owned
  Python processes remain explicitly declared in both;
- a WSL process launched from Windows is Linux evidence, not Windows-native
  evidence;
- both platforms must preserve the same Product declarations, process roles,
  DDS topics, typed readiness, identity, stop, cleanup, and rollback contracts;
- shared algorithm and concurrency code uses the standard C++17 boundary;
  platform-specific PE/DLL and ELF/SO artifacts remain explicit RunPlan inputs;
- simulator-truth odometry and Python fallback are not substitutes for native
  Fast-LIO SLAM on either platform; and
- Windows/Linux simulation evidence does not replace S100P field safety and
  physical Driver acknowledgement evidence.

## Runtime Surface Classification

| Surface | Classification | Decision |
| --- | --- | --- |
| `Product`, `ProductControl`, `RunPlan` | current field control plane | Keep. Product is compiled once; ProductControl owns switching and applies the exact RunPlan. |
| `Host`, `Blueprint`, `Module` | current scoped Host runtime | Keep. They assemble Gateway, Agent, MCP, semantic logic, adapters, and local/dev Modules; they do not own field processes. |
| `src/nav/cpp`, `navd` | current field navigation owner | Keep and validate as the unique planner/tracker/final-command implementation. |
| `src/maps/cpp/mapd`, native map store/build | sole field map-management owner | Keep. `lt-maps.service` runs mapd; mapd owns realtime layers, persistent products, native SaveMap coordination, and native PGO handoff. |
| Python map-management Modules and pipeline | retired | `MapsModule`, `MapdServiceClient`, and the Python SaveMap pipeline are removed. Gateway retains only stateless same-host UDS transport. |
| Gateway map preview paths | active visualization compatibility | `/api/v1/map/points` remains live accumulated point-cloud JSON compatibility used by Rerun/evidence/Web until those consumers migrate to WS/DDS. `/api/v1/maps/{name}/points` is saved-map JSON; `/api/v1/maps/{name}/pcd` is raw saved-map PCD. |
| Taskless inspection endpoints returning `410` | retired API tombstones | Removed after the client inventory found no callers; task-addressed inspection APIs are the only Gateway contract. |
| `MapManagerModule`, old nav C++ paths, ROS2 TARE path | retired names and paths | Removed. Static contracts prevent their return; do not recreate shims. |
| Root `Testing/` | generated CTest debris | Deleted from maintained files and ignored. |

The former broad Module-centric slogan is retired. The accurate statement is
"Product-defined field runtime with a scoped Module/Blueprint Host."

## Product Modes

The YAML files under `config/runtime_graph/products/` are authoritative.

| Product | SLAM | Saved map | Control owner | Purpose |
| --- | --- | ---: | --- | --- |
| `teleop` | none | no | operator | Direct operator motion through native authority and final safety. |
| `teleop_avoid` | mapping | no | operator | Operator-assisted local detours using live odometry, cloud, maps scene, and standalone traversability. |
| `map` | mapping | no | operator | Build and transactionally save map products. |
| `explore` | mapping or localization | optional | exploration endpoint | No map grows a live map; a map selects saved-map coverage. |
| `nav` | localization | yes | native navigation | Saved-map autonomous navigation. |
| `tracking` | localization | yes | native navigation | Follow a selected RGB-D person through bounded map-frame goals. |
| `inspection` | localization | yes | native navigation | Execute persisted multi-point inspection tasks. |

`explore` is the only public exploration Product. The presence of a map selects
the map/localization variant; absence of a map selects live mapping. TARE may
remain an internal policy name but must not reappear as an operator Product.

Do not reintroduce a localization map requirement into `teleop_avoid`. Its
current acceptance manifest must run mapping/no-map and must not promote a
temporary localization PCD.

## Current Work Board

| Priority | Work | Completion gate | Status |
| --- | --- | --- | --- |
| P0 | Deliver Windows-native MuJoCo parity for every operator Product. | Windows x64 builds/tests every selected native artifact including `slamd.exe`; all seven Products, with `explore` accepted separately as `live` and `map`, resolve RunPlans whose native artifacts are PE/DLL-only and pass the exact ProductControl lifecycle, scenario, terminal-zero, cleanup, and rollback gates without WSL or fallback. | Active; W1-W3 build, runtime-loopback, stage, and eight-target RunPlan compile gates passed; W5 exact Product execution remains |
| P0 | Rebuild a self-contained native release from the current IDL/source closure. | Strict Product preflight selects only release artifacts, reports hashes/mtimes, and has `blockers=[]`; release includes and tests mapd and the DDS probe. | Active |
| P0 | Stabilize MuJoCo mapping/no-map SLAM for `teleop_avoid`. | Native sensor -> SLAM -> MapObservation remains tracking for the full representative run; no covariance/finite-data gate failure; no test-specific production threshold weakening. | Active |
| P0 | Close `teleop_avoid` Product behavior. | Free-space plus obstacle scenarios prove typed operator authority, local path, final command, driver acknowledgement, cleanup, and zero barrier with exact provenance. | Active |
| P0 | Close physical command safety on S100P. | No-motion readiness and fault injection pass before bounded supervised motion; driver executable and ACK freshness are proven. | Blocked on preceding gates |
| P1 | Prove dynamic-obstacle clearing and long-run resource bounds. | `moving_person_clear`, labelled replay, and MID-360 runs meet residual, thin-obstacle, CPU, memory, DDS-byte, and epoch-reset limits. | Gate implemented; accepted run pending |
| P1 | Integrate native velocity smoothing without weakening safety. | Smoother runs before final safety; emergency/stale/cancel zero bypasses ramping; ramp, reversal and rotation are validated before wider use. | `teleop` and `teleop_avoid` wired; MuJoCo/S100P evidence pending |
| P1 | Add a replaceable collision-aware path-smoother contract. | Preserves frame, map generation, endpoints, clearance, curvature, and planner direction/cusps; failure is explicit and fail-closed. | Not started |
| P2 | Productize generic route operations. | Typed route request/status/event lifecycle, closure/reroute semantics, and progress telemetry over `maps::MapGraph`. | Foundations only |
| P2 | Add target following. | Independent Product, typed target observation, lost-target behavior, standoff controller, safety ownership, MuJoCo and field evidence. | Foundations only |
| P2 | Add docking/charging. | Dock database, pose/covariance, staged controller, hardware contact/charge feedback, retry/undock lifecycle, simulation and physical evidence. | Missing |

Detailed capability maturity and upstream adoption decisions live in
[`NAVIGATION_CAPABILITY_MATRIX.md`](../architecture/NAVIGATION_CAPABILITY_MATRIX.md).
Architecture/product gaps are summarized in [`known_gaps.md`](../known_gaps.md).

## P0 Windows-native MuJoCo Execution Plan

The current catalog has Windows commands for the MuJoCo sensor publisher,
driver bridge, SLAM, mapd, traversability, navd, exploration runtime, and Host.
The semantic-map client also has a Windows dynamic-library loader. W1-W3 have
closed the portable concurrency, real Windows SLAM build/runtime, staged DLL
closure, and RunPlan compile gates. W4 has closed the `teleop` coordinator code
slice. W5 exact Product execution is the remaining workstation-platform gate.

### W1A. Make the Fast-LIO thread boundary portable

Status: complete at the code/CTest gate. The narrow boundary uses C++17
concurrency primitives. Background rebuild, shutdown, repeated destruction,
and regression coverage passed 20 CTest repetitions on Windows and 20 on
Linux; Linux ASan/UBSan and leak checks also passed. The port retains the
required rebuild/read synchronization and fixes split-axis equal-point
deletion, `working_flag` lifetime, and empty replay. The cleanup removed
swallowed background exceptions, 100-microsecond polling, the global
range-query lock, the unused benchmark, and excessive fault injection. Native
Linux TSan and dedicated S100P performance evidence remain open. W2 separately proved the complete MSVC x64
Fast-LIO-enabled `slamd.exe` runtime.

- Regression tests cover ikd-tree construction, background rebuild, shutdown,
  and repeated destruction.
- The private POSIX thread boundary is implemented with C++17 concurrency
  facilities while preserving lifecycle and synchronization behavior.
- W2 owns the separate MSVC x64 dependency, complete-runtime build, and DLL
  closure evidence; the completed W1 result remains a distinct source-level
  portability gate.

Linux and S100P provide the former POSIX headers, so the migration was not
needed merely to make the source compile there. It placed thread ownership on
one C++17 contract and tested lifecycle risks shared by all platforms. The
implementation follows the behavior of the local tree and its regression
tests; no external GPL patch was copied into this source.

### W1B. Separate production and contract-only SLAM targets

Status: complete. CMake now rejects a DDS production runtime when the real
Fast-LIO2 backend is disabled. The focused native SLAM contract suite passes
45 tests, including the failure-closed configuration cases.

- Keep contract/mock backends available only to their explicitly named tests
  and development targets.
- A production DDS runtime that selects `fastlio2` must link the real
  Fast-LIO2 target. Configuration or startup must fail closed when that target
  is absent; never label `makeContractBackend("fastlio2")` as a production
  Fast-LIO2 implementation.
- Express this separation in CMake target dependencies and focused tests before
  changing file locations. W1B stops only when a production `slamd` cannot be
  configured, built, or launched with a contract backend masquerading as
  `fastlio2`.

### W2. Produce a reproducible Windows SLAM release

- `scripts/build/prepare_cyclonedds_windows.ps1` builds the source-locked
  official CycloneDDS 11.0.1 SDK with Visual Studio 2022 x64, and
  `verify_cyclonedds_windows_sdk.ps1` validates PE architecture, exact payload
  hashes, IDL generation, consumer link, and DLL closure.
- `scripts/build/prepare_slam_dependencies_windows.ps1` owns the pinned vcpkg
  x64-windows Eigen/PCL/yaml-cpp dependency prefix and binary cache.
- `scripts/build/build_slam_core_windows.ps1` wraps the existing
  `src/localization/slam/cpp` CMake project without duplicating target logic.
  Focused preparation, staging, and wrapper suites cover immutable source and
  package inputs, read-only preflight, exact production flags, CycloneDDS
  build-source/file-inventory drift, path isolation, and staged payload tampering.
- Invoke it with explicit absolute `-DependencyPrefix`, `-CycloneDDSPrefix`,
  `-VcpkgRoot`, `-VcpkgInstallRoot`, and `-VcpkgBinaryCache` values. Pin an
  absolute `-BuildDir` for reproducible evidence. Use `-PreflightOnly` to
  validate Visual Studio 2022 x64, package configs, CycloneDDS 11.0.1 PE
  artifacts and source/file inventory, vcpkg identity, and any existing cache without
  configuring or building.
- Require explicit, ABI-compatible x64 package roots for CycloneDDS, Eigen,
  PCL, and yaml-cpp. Keep this build tree separate from every Linux or other
  CMake-generator tree.
- A fresh build from the pinned official CycloneDDS 11.0.1 source has published
  and verified the Visual Studio 2022 `v143` x64 Release `/MD` SDK at
  `third_party/integration/cyclonedds-windows-20260817-d/sdk`. Its build-source/file-inventory record binds
  commit `e54e991f75a3e67f8e628da3171122e36ea5b872` and tree
  `56508d35826c362782fc8a388cad351a3d491f51`; PE/x64, DLL closure, IDL smoke,
  consumer compile/link, and sanitized DLL-search checks passed. The pinned
  vcpkg MSVC-x64 prefix supplies Eigen, PCL, yaml-cpp, and their transitive
  dependencies. The build output records exact versions, package baseline,
  runtime DLLs, architecture, and hashes; MinGW or Conda binaries are not
  substitutes for that closure.
- Build and test Fast-LIO plus the DDS runtime, producing staged
  `slamd.exe`/`slamctl.exe` and their declared DLL closure.
- Require a clean configure/build/CTest, `--help` smoke, DDS LiDAR/IMU loopback,
  typed SLAM readiness/status, and clean exit from an empty build directory.

Status: complete at the Windows build/runtime gate. The real MSVC x64
Fast-LIO-enabled `slamd.exe` and `slamctl.exe` build passed 7/7 native CTest.
The raw LiDAR/IMU loopback reached `TRACKING`, typed `slamctl` status and the
repository readiness loader passed, and a DDS domain mismatch failed closed.
The fresh formal stage at `build/slam-core-windows-x64/stage` contains 42 files;
its audit passed 21/21 PE files as x64, found zero unresolved non-system imports,
and passed unchanged-input reuse. The SPDX 2.3 inventory has 13 packages, 35
files, and 52 relationships, including the exact `LingTu -> CycloneDDS`
`DYNAMIC_LINK` relationship. It attributes 18 vcpkg DLLs to their packages,
`ddsc.dll` to CycloneDDS, and only `slamd.exe` and `slamctl.exe` to LingTu.
Never point this wrapper at a Linux, WSL, MinGW, MSYS, UCRT64, or other
generator's cache or package prefix.

The canonical Windows MuJoCo adapter, navigation, maps, and staged SLAM output
directories now place `ddsc.dll` beside their executables. All four copies have
SHA-256
`203ece8c0b2c00380f632c0d85380f5381354957e0fb78155e1de8cf7d996887`,
closing the canonical `0xC0000135` missing-DDS-loader failure class. The
canonical directories are `build/windows-native-dds-adapter/Release`,
`build/maps-windows/Release`,
`build/nav-cpp/windows-x64-nav-endpoint/Release`, and
`build/slam-core-windows-x64/stage/bin`. Old `-root-d` trees are quarantined
below `C:/Users/99563/.codex/tmp/lingtu-stale-builds`; they are not build or
acceptance inputs and have no fallback, alias, or junction.

This app-local closure covers CycloneDDS, not the Microsoft runtime. Install
the official Microsoft Visual C++ Redistributable x64 centrally. Do not copy
its CRT DLLs into the application directory or call the output fully
self-contained. Both the Windows MuJoCo build helper and ProductControl
preflight fail closed unless the 64-bit runtime is registered and `System32`
provides PE32+/x64 `msvcp140.dll`,
`msvcp140_2.dll`, `vcruntime140.dll`, `vcruntime140_1.dll`, and
`vcomp140.dll`. The required runtime version follows the configured MSVC
toolset at build time and the exact Product artifacts' linker versions at
startup.

### W3. Enable Windows RunPlan resolution

Status: complete at the RunPlan compile gate. `sim.yaml` now declares the
canonical Windows DDS sensor, driver, mapd, traversability, navd, exploration, and
Host-client artifacts plus the staged Windows `slam_runtime`.

- The Windows `slam_runtime` command consumes the W2 staged artifact.
- The executable and 19 DLL dependencies are declared so platform preflight can
  verify the required files and executable format before startup.
- RunPlan schema v4 rejects v3 explicitly. It binds the selected artifacts and
  dependencies directly and checks PE32+/x64 plus the Registry64/System32 VC++
  runtime before startup. The removed per-process MSVC scheme and its four JSON
  files are not Product inputs. CycloneDDS and SLAM retain their own
  build-source/file-inventory records and SLAM retains its SBOM.
- The navigation preset requires `LINGTU_CYCLONEDDS_PREFIX` and
  `LINGTU_OCTOMAP_PREFIX` and writes only the canonical navigation build tree.
- All seven Products compile on Windows, with `explore` compiled for both
  `live` and `map`, yielding eight selected RunPlan targets with PE/DLL-only
  native artifacts; shared interpreter-owned Python entries remain explicit.
  Fresh evidence is 86 Product-compile tests, 141 runtime-graph tests, and 55
  RunPlan identity/environment tests. The dependency-aware startup stages are
  10 for sensor/driver; 20 for feeder, SLAM, traversability, and nav; 30 for
  mapd; and 50 for exploration/Host. Missing, stale, wrong-architecture, or
  hash-mismatched files must fail before state-root
  creation, map staging, journaling, or child startup.
- Keep Windows Product DDS traffic on domain `17` and Linux Product traffic on
  domain `231`. Adapter CTests use the isolated low-domain slots `10`-`16`,
  `18`, and `19`; they reject reserved domain `17` and any domain whose fixed
  CycloneDDS ports enter the Windows dynamic-port range.

### W4. Close the exact ProductControl acceptance transaction

Status: coordinator code and focused tests complete for the `teleop` vertical
slice; Product evidence is not complete. The manifest remains
`coverage=component`, and no Product currently has an exact Product PASS.

- Reuse `sim/scripts/mujoco/product_acceptance.py` as the single coordinator;
  do not introduce a second lifecycle owner.
- Execute one isolated transaction:
  `ProductControl.switch -> committed current/RunPlan/ledger/readiness ->
  attach-only scenario -> stop_current -> terminal zero/ACK -> empty current
  and ledger -> dead child identities -> rollback fault injection`.
- The `teleop` coordinator binds the committed Product session across current
  state, RunPlan, child ledger, typed readiness, attach-only scenario, guarded
  stop, terminal zero/ACK, cleanup, and rollback verification.
- Convert remaining self-starting component runners to attach-only observers or
  operators. Retain the independent, fail-closed `rollback_verified` promotion
  gate beside lifecycle, stop, and cleanup verification.
- Require Product, `product_session_id`, variant, and saved-map identity to
  match across current state, RunPlan, child ledger, typed readiness, and the
  scenario report.

### W5. Run and promote the Windows matrix

Status: started and in progress. W1-W3 are complete and the `teleop` W4
coordinator code is ready. The fresh Windows exact-`teleop` technical report
has `ok=true` and `blockers=[]`: switch, four typed-readiness gates, attach-only
scenario, physical motion (0.164 m path and 0.158 m net displacement), terminal
zero, stop, cleanup, and rollback all passed. It remains
`evidence_scope=component_e2e` against a `coverage=component` manifest, so
`product_acceptance_passed=false`; the repeatability campaign is active and the
exact Product PASS count remains zero on both target evidence tracks. Windows
and Linux/WSL must be executed and archived independently.

The DLL-loader repair is not a substitute for the W5 matrix. Focused Windows
component evidence is maps 32/32; 17 selected navigation tests for three
consecutive runs; SLAM 7/7 and formal staging; adapter endpoint apply/ACK 50/50;
adapter bridge stress 20/20; and the fresh complete adapter suite 18/18. The
IMU test fix is limited to best-effort DDS discovery warm-up and a read
condition; production code is unchanged. Keep the Product boundary explicit
until the repeatability gates pass.

Run the exact targets in this order: `teleop`, `map`, `explore:live`, the
eight-case `teleop_avoid` matrix, `tracking`, `inspection`, `explore:map`, and
the bound 59.94 m `nav` scenario. Each target must pass from a fresh Windows
state root three consecutive times; navigation must pass five consecutive
times with path length, net displacement, and goal-distance reduction each at
least 50 m.

Only after a target has an archived exact report with `blockers=[]`, complete
artifact provenance, terminal physical zero acknowledgement, no surviving
child, and verified rollback may its manifest move from `coverage=component`
to `coverage=product`. Linux/WSL must pass its own equivalent matrix; neither
platform inherits the other's result.

### Code and directory boundary during W1-W5

Use target ownership before physical relocation:

| Existing location | Ownership during this migration |
| --- | --- |
| `src/localization/fastlio2/` | Fast-LIO algorithm core and ikd-tree; no Product lifecycle or platform package discovery |
| `src/localization/slam/cpp/` | SLAM contract, backend selection, native DDS runtime, and CMake target boundaries |
| Existing ROS wrapper locations | Compatibility/evaluation entrypoints only; excluded from native Product targets |
| `scripts/build/` | Thin platform build wrappers and dependency/artifact verification; no duplicate target logic |
| `config/runtime_graph/envs/sim.yaml` | Platform command and dependency declaration only after its artifact gate passes |
| `sim/scripts/mujoco/` | One exact Product acceptance coordinator plus attach-only scenarios |

Do not reorganize these directories while the target boundary and dual-platform
evidence are changing. First decouple through CMake targets and runtime
contracts, complete W5, and only then evaluate a physical move with import,
include, build, packaging, and evidence paths updated together. This avoids
file churn that adds no portability or acceptance value.

### License gate

The retained FAST-LIO, ikd-Tree, and IKFoM license and origin notices still
require explicit review before distribution. The official FAST_LIO, ikd-Tree,
and IKFoM repositories publish GPL-2.0 licenses; replacing only the ikd-tree
container therefore does not establish that a
closed-source `slamd` may be distributed. Record source origin, commit or
release, modifications, notices, and the licenses of Fast-LIO, ikd-tree,
IKFoM, PCL, Eigen, yaml-cpp, and CycloneDDS. Before a commercial release, make
one documented choice: obtain authorization covering the complete copyright
chain, distribute the affected SLAM boundary in a reviewed GPL-compliant form,
or clean-room replace the GPL-derived SLAM core. This is an engineering release
gate, not a legal conclusion. Do not resolve it by copying code from an
external Windows fork or unmerged patch; use such projects only as feasibility
and test-reference evidence unless their provenance and license obligations
have been reviewed.

### Prohibited substitutes

- Do not run Linux/WSL children and label the result Windows-native.
- Do not add a Windows-only Product topology, Python SLAM fallback, or second
  acceptance lifecycle framework.
- Do not publish MuJoCo truth pose as Fast-LIO evidence.
- Do not mix PE and ELF artifacts or reuse one CMake build directory across
  platforms/generators.
- Do not promote from a dry run, unit test, catalog compile, or component-only
  report.

## Acceptance Order

MuJoCo promotion evidence is platform-specific: a Windows report cannot satisfy
the Linux gate, and a Linux/WSL report cannot satisfy the Windows gate. For each
Product change:

1. Prove focused algorithm and fail-closed tests.
2. Build every affected native artifact from the current schema/source closure.
3. Pass strict preflight with unique writers and no stale artifacts.
4. Pass the named native MuJoCo or replay scenario.
5. Pass target no-motion readiness and fault injection.
6. Run bounded supervised field motion only when the previous gates pass.
7. Add a new date-prefixed field record; do not edit old evidence.

## Source Of Truth

| Topic | Source |
| --- | --- |
| Product declarations | `config/runtime_graph/products/*.yaml` |
| Endpoint/process/topic declarations | `config/runtime_graph/` |
| Product/Host/process ownership | `docs/architecture/SYSTEM_DESIGN.md` |
| Native navigation dataflow | `docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md` |
| Map lifecycle and products | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Field Products | `docs/architecture/FIELD_PRODUCTS.md` |
| Capability maturity | `docs/architecture/NAVIGATION_CAPABILITY_MATRIX.md` |
| Validation gates | `docs/07-testing/README.md` |
| Windows/Linux MuJoCo platform contract | `docs/architecture/SIMULATION_INTEGRATION_CONTRACT.md` |
| Native build commands and current build gaps | `docs/01-getting-started/BUILD_GUIDE.md` |

## Claim Discipline

Until matching evidence exists, do not claim that dynamic people never leave
residuals, velocity smoothing is active in the field, target following or
docking is supported, native SaveMap/PGO has passed field acceptance, or a
Product is complete from unit tests alone. Do not describe WSL execution as
Windows-native support or treat the present Windows `teleop` compile path as
the finished Windows Product matrix.
