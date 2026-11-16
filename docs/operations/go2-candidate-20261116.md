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
The original shared worktree remains a separate, ongoing development workspace.

The integration fixes prepared-cache read/write alignment, preserves terminal
tolerances before the Rust query, and separates configured planning geometry
from instantaneous observed posture. The snapshot had also removed the main
Go2 driver's factory-acquisition and attitude admission checks; those existing
checks were restored without changing configuration, zero-command or stop paths.

Local integration evidence includes Rust 79 passed/1 ignored, global planning
20/20, portable navigation 821/821, maps 5/5 and portable driver 15/15. Web's
994-test run passed; subsequent React/lint fixes passed the affected 60-test
group, final lint, both TypeScript projects and the production build. Linux
installer/preparation checks passed 18 tests and the three Bash behaviors.
The Linux visual replay fixture passed all three tests after fixing its
synthetic frame timestamps; production freshness checks were retained.

Logs are retained under `build/converge/`, `build/converge-app-20261117/`,
`build/global-tests-final.log` and `build/nav-portable-tests-final.log` in the
integration worktree. Remote CI belongs to the exact PR head; a local pass or an
earlier PR head's result does not replace it. The slow 35-degree planning case
was not optimized in this integration. No Go2 target package or field motion
is qualified by these results.

The Go2 delivery path remains the existing Linux/aarch64 native build and
release packager. Windows and WSL/x86_64 test results cannot qualify its ELF
binaries, SDK linkage or sensors.

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

The wider deployment test file still has four failures outside the package
change: one retired driver diagnostic string, two references to the removed
`endpoint/nav/app/goal/terminal.cpp`, and one previous systemd implementation
assertion. They are recorded in the package report and remain for integration;
the narrow pass does not clear those failures.

## Remaining build and target work

This workstation's WSL environment has no aarch64 C++ toolchain/sysroot, ARM
Rust target, Unitree SDK2 prefix or RealSense development SDK. The current MLS
CMake Cargo invocation also assumes a native build. No cross-compilation
framework is being introduced for this candidate; use the established NX
native build after confirming its installed prerequisites.

Once integration provides a fixed, tested source revision, use a separate
candidate checkout and the matching SDKs on NX:

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
