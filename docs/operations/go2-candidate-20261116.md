# Go2 local candidate preparation — 2026-11-16

Local preparation is in progress; no new Go2 release has been qualified or
activated. The operator confirmed that the robot is powered off. No hardware
connection, Product switch or motion command is part of this local pass.

## Source and delivery scope

The initial working branch was `codex/workspace-review-20261116`, based on `d02b427c`,
with concurrent uncommitted navigation and application changes. This describes
the initial test workspace, **not a fixed deployment candidate**. The follow-up
below records its integration into a separate fixed source branch.

## Integration follow-up — 2026-11-17

`codex/converge-20261117` captures the local refinements in `4998f694` and merges
the main/PR30/PR33 lineage at `4dd657c0`. Merge commit `5e660ce6` and its follow-up
fixes are reviewed together in [PR #39](https://github.com/Kitjesen/MapPilot/pull/39).
The candidate source is the final reviewed head of that PR, including the
subsequent main integration and verified fixes recorded below.
The original shared worktree remains a separate, ongoing development workspace.

The integration fixes prepared-cache read/write alignment, preserves terminal
tolerances before the Rust query, and separates configured planning geometry
from instantaneous observed posture. On 2026-11-17 the operator selected SDK-owned
Go2 velocity acceptance. Factory-avoidance setup and measured attitude remain
readiness/health diagnostics. Velocity submissions follow the SDK return value;
SDK failures, zero output and confirmed-stop handling keep their existing semantics.

Local integration evidence includes Rust 79 passed/1 ignored, global planning
20/20, portable navigation 821/821, maps 5/5 and portable driver 15/15. Web's
994-test run passed; subsequent React/lint fixes passed the affected 60-test
group, final lint, both TypeScript projects and the production build. Linux
installer/preparation checks passed 18 tests and the three Bash behaviors.
Visual replay now reserves an empty capture directory for each run and checks
the captured PNG contents and minimum frame count. This directory ownership
provides the frame provenance independently of filesystem timestamp precision.

The current Go2 driver was freshly compiled against the existing real Unitree
SDK2 prefix `/home/thunder/.cache/unitree-sdk2-install`. All 16 non-hardware
CTest cases passed, including isolated-domain DDS IO. The x86_64 ELF loaded
successfully with `--help`; `ldd` resolved both CycloneDDS libraries from that
SDK prefix, and the binary's RPATH selects the same prefix. The initial build
is recorded in `build/converge/go2-sdk-build.log`. The SDK-owned acceptance
rebuild passed the same 16 cases in `build/converge/go2-sdk-only-build.log`;
its source changes are recorded in `go2-sdk-only-source.patch` beside that log.

Logs are retained under `build/converge/`, `build/converge-app-20261117/`,
`build/global-tests-final.log` and `build/nav-portable-tests-final.log` in the
integration worktree. Remote CI belongs to the exact PR head; a local pass or an
earlier PR head's result does not replace it. The slow 35-degree planning case
was not optimized in this integration. No Go2 target package or field motion
is qualified by these results.

The Go2 delivery path remains the existing Linux/aarch64 native build and
release packager. Aarch64 binaries, target SDK linkage and live sensors still
require evidence from the matching target environment.

## Completed local work

The RealSense branch in `scripts/deploy/package_native_release.sh` now rejects
a supplied install prefix that contains `realsense_capture` but omits
`lib/librealsense2.so*`. Previously that supported packaging input could create
an archive whose camera would fail to load on a machine without the SDK.

The production change is three lines. The existing package self-test now
covers the missing SDK and a versioned SDK symlink copied into the archive.
No dependency, installer, Product lifecycle or robot configuration was added.

| Local verification | Result | Evidence scope |
| --- | --- | --- |
| Go2 `nav/standard`, `nav/camera`, `inspection` RunPlan compilation and serialization | All three passed | Pure configuration resolution, no processes |
| Existing Go2 configuration contracts | 17 passed | Driver, network, geometry, support, velocity limits and RunPlan |
| Runtime preparation baseline | 3 passed | Existing Python preparation behavior |
| Directly related release/preparation tests | 12 passed | Selected Python contracts |
| Package/install Bash syntax and package self-test | Passed in WSL | Placeholder binaries and fake ProductControl, including rollback |
| No-motion evaluator and installer contracts on Windows | 18 passed, 7 skipped | Seven Bash-only installer cases require Linux |
| Those seven installer cases in WSL | 7 passed, 8 deselected | Current repository source; saved/explicit Python, failure rollback and help |

The WSL installer follow-up used the existing Python environment without
installing dependencies. It closes the seven Windows skips above; it does not
constitute installation of a real candidate on NX.

The resolved driver is `go2`, with `real + systemd`. `nav/camera` and
`inspection` select `realsense_native`; `nav/standard` does not select a camera
process. The navigation tick is 100 Hz and the local planner is SCAN; this does
not mean SCAN regenerates a trajectory every tick. Robot configuration keeps
`auto_enable=false` and `auto_standup=false`.

Local evidence:

- [Configuration and commands](../../build/go2-local-20261116/config-check.txt)
- [No-motion/installer test output](../../build/go2-local-20261116/deploy-tools.txt)
- [WSL installer behavior test output](../../build/go2-deploy-offline-20261116/installer-wsl.log)
- [Package verification report](../../build/go2-deploy-offline-20261116/report.md)
- [Exact package patch](../../build/go2-deploy-offline-20261116/package.patch)
- [Package self-test output](../../build/go2-deploy-offline-20261116/package-self-test.after.log)

The four deployment-test mismatches recorded in the initial package report
have been reconciled with their current owners. The tests for the driver
entrypoint, both terminal-lifecycle paths and Gateway-before-runtime readiness
all passed in `build/converge/release-contracts-after.xml`. That run reports
101 passed and three Windows Bash-only skips; these four tests have no skips.
The updated assertions follow the current driver diagnostics, terminal owner
files and ProductControl readiness order.

## Remaining build and target work

The available WSL host and Unitree SDK2 libraries are x86_64. A complete Go2
aarch64 package still requires a matching build environment, vendor libraries
and C++/Rust targets. The existing ARM CI artifact contains `slamd`, `slamctl`
and `test_slam_contract`; its three selected SLAM tests passed. The complete
Go2 ARM package remains unverified.

The WSL default discovery paths did not provide `realsense2`, and the package
database had no RealSense development SDK. The full camera build still needs
its SDK prefix. Use the established NX native build after confirming its
installed prerequisites.

Use the final reviewed PR source in a separate candidate checkout with the
matching SDKs on NX:

```bash
export LINGTU_DRIVER_BACKEND=go2
export LINGTU_CAMERA_DRIVER=realsense_native
export LINGTU_UNITREE_SDK2_PREFIX=/opt/unitree_robotics  # Confirm this target path first.
make build BUILD_TYPE=Release
bash scripts/deploy/package_native_release.sh "<unique-candidate-version>" dist
```

The version must use the existing `vX.Y.Z` format, optionally with a prerelease
suffix. Building and packaging do not activate the release. Check the actual
target ELF/dependencies and prepare its Python runtime before installation.
The installer can restart the retained Product, so it is not a preparation-only
command.

The next target evidence is the same candidate's stationary startup, fresh
sensors/localization, zero output and driver stop acknowledgement. Then run the
bounded supervised scenario described in [Testing](../testing.md). Historical
Go2 `.99` evidence and another robot's MuJoCo results do not replace that run.
