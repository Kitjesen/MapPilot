# Go2 local candidate preparation — 2026-11-16

Local preparation is in progress; no new Go2 release has been qualified or
activated. The operator confirmed that the robot is powered off. No hardware
connection, Product switch or motion command is part of this local pass.

## Source and delivery scope

The working branch was `codex/workspace-review-20261116`, based on `d02b427c`,
with concurrent uncommitted navigation and application changes. This describes
the test workspace, **not a fixed deployment candidate**. PR #37 still needs
integration with main; do not package a moving checkout as an accepted release.

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
