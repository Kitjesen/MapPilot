# Known LingTu Gaps

Status: current as of 2026-08-17

This file tracks gaps that matter to architecture, productization, or field
readiness. It should not repeat old closed incidents.

## P0/P1 Gaps

| Gap | Severity | Impact | Current direction |
| --- | --- | --- | --- |
| Windows-native MuJoCo Product parity | P0 | W1-W3 now provide the portable iKD boundary, real MSVC x64 Fast-LIO runtime, fresh audited stage, typed loopback/readiness, one canonical native chain, and eight compiling Windows RunPlan targets. Canonical MuJoCo/nav/maps/SLAM outputs have app-local `ddsc.dll`, and the fresh complete adapter suite passes 18/18, but no Product has an exact PASS. | Continue W5 for all seven Products plus `explore:live/map`, including lifecycle, scenario, stop/zero ACK, cleanup, rollback, and repeatability without WSL or fallback. Track native Linux TSan, dedicated S100P performance, and release controls separately. |
| Exact MuJoCo Product acceptance | P0 | All eight selected targets still declare `coverage=component`; current exact Product PASS count is zero. The first fresh Windows exact-`teleop` technical closure has `ok=true` and `blockers=[]`, including switch, four readiness gates, attach-only scenario, physical motion, terminal zero, stop, cleanup, and rollback. It remains `component_e2e` with `product_acceptance_passed=false`. | Complete the repeatability campaign, then run the same ProductControl-owned exact transaction independently for every Windows and Linux/WSL target before changing any manifest to `coverage=product`. |
| Native release provenance | P0 | Development provenance and the Windows SLAM-stage build-source/file-inventory record and SBOM pass, but release eligibility remains false. | Keep stale-artifact preflight fail-closed. Release additionally requires an externally signed attestation, a full release subject, and publish-pipeline integration; local metadata cannot clear these blockers. |
| Field motion closure | P0 | The native DDS/driver code path is locally locked, but full real-motion closure still needs target-side fault injection and supervised motion evidence. | Keep no-motion route preview, driver watchdog checks, and real motion smoke as separate gates. |
| Dynamic-obstacle residual evidence | P1 | Column carving, rolling occupancy, and decay are implemented, but a current moving-person Product run and long-duration MID-360 evidence are still missing. | Pass `moving_person_clear`, dataset replay, and field residual/resource gates before making a no-ghosting claim. |
| Native map control/query convergence | P1 | Realtime `mapd` is native, while some persistent map control/query paths still cross a thin Python Host facade. | Make typed map control/query the only field path and retain Gateway as translation-only. |
| Velocity and path smoothing | P1 | The ROS-free C++ velocity smoother is enabled in `teleop` and `teleop_avoid` before final safety. MuJoCo/S100P tuning evidence is pending, and a collision-aware path smoother is still absent. | Validate ramp, reversal, rotation and hard stop in both operator-motion Products before claiming field readiness. |
| Route/follow/dock products | P2 | Inspection waypoints are mature, but generic route operations, target following, and docking/charging do not have complete Product contracts and evidence. | Follow the capability matrix; do not present foundations as delivered products. |
| Cross-language transport schema | P2 | DDS/shared-memory payloads are not yet uniformly versioned for long-term Dart/Rust/C++ interop. | Promote typed schemas with explicit version, frame, timestamp, bounds, and compatibility tests. |

## Current Non-Goals

- Do not make ROS 2 the product API.
- Do not expose planner backend internals directly to Gateway or UI.
- Do not add a custom transport framework beyond the existing Port/Wire/Transport
  model until typed schemas are stable.

## Recently Closed

- The canonical Windows MuJoCo adapter, navigation, maps, and staged SLAM
  outputs now place `ddsc.dll` beside their executables; all four copies have
  SHA-256
  `203ece8c0b2c00380f632c0d85380f5381354957e0fb78155e1de8cf7d996887`.
  This closes the canonical `0xC0000135` missing-`ddsc.dll` incident class.
  The canonical directories are `build/windows-native-dds-adapter/Release`,
  `build/maps-windows/Release`,
  `build/nav-cpp/windows-x64-nav-endpoint/Release`, and
  `build/slam-core-windows-x64/stage/bin`. Old `-root-d` trees are quarantined
  below `C:/Users/99563/.codex/tmp/lingtu-stale-builds`; no fallback, alias, or
  junction may select them.
- Windows Products now use DDS domain `17`, Linux Products remain on `231`,
  and adapter CTests use low-domain slots `10`-`16`, `18`, and `19` while
  rejecting domain `17` and Windows dynamic-port overlap.
- Windows native startup now treats the official Microsoft Visual C++
  Redistributable x64 as a central prerequisite. The build helper and
  ProductControl fail closed when the 64-bit registration is absent or any of
  the PE32+/x64 files `msvcp140.dll`, `msvcp140_2.dll`,
  `vcruntime140.dll`, `vcruntime140_1.dll`, or `vcomp140.dll` is missing from
  `System32`. Runtime version requirements are toolset- and artifact-derived;
  the CRT is not copied app-local and the output is not described as fully
  self-contained.
- The Windows MuJoCo DDS publisher/driver boundary, real Fast-LIO
  `slamd.exe`/`slamctl.exe`, pinned dependency closure, formal stage
  source/file inventory and SBOM, raw sensor loopback, typed readiness,
  canonical map/navigation chain, and eight
  Product/variant RunPlan compilation gates now pass. The fresh stage audit is
  42 files, 21/21 x64 PE files, zero unresolved non-system imports, and SPDX
  13 packages/35 files/52 relationships with the exact CycloneDDS dynamic-link
  edge. This closes the former Windows build and W3 declaration gaps, but not
  exact Product parity.
- RunPlan v4 rejects v3 and binds the real artifact/dependency paths and hashes.
  The per-process MSVC scheme and four redundant JSON files were removed;
  startup instead checks PE32+/x64 plus Registry64/System32 VC++ runtime state.
- Fresh component checks passed maps 32/32, 17 selected navigation tests three
  times, SLAM 7/7 plus staging, adapter endpoint apply/ACK 50/50, and adapter
  bridge stress 20/20. The fresh complete adapter suite also passed 18/18 after
  a test-only best-effort discovery warm-up/read condition fix; production code
  was unchanged. Product PASS remains zero.
- Global planning now has a stable wire payload for preview and mission status:
  `lingtu.global_plan.v1`.
- The physical Thunder command boundary is now documented as `endpoint_only`
  through logical `/nav/cmd_vel` (DDS wire `rt/nav/cmd_vel`), unique
  `lt-driver.service`, and remote Brainstem gRPC. Remaining work is field evidence,
  not another default Python DDS writer.
- Current first-party documentation entrypoints, architecture contracts,
  deployment runbooks, package READMEs, and generated API inventories were
  reorganized on 2026-07-28. Superseded plans and duplicate binary reports were
  deleted; research and dated evidence remain explicitly non-authoritative.
