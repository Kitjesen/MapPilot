# Deployment

`scripts/deploy/` owns release packaging, robot installation, and systemd
assets. Product startup remains owned by ProductControl.

## Main commands

Customer instructions are in [`docs/customer-deployment.md`](../../docs/customer-deployment.md),
also included in new native archives. Prepare the locked Python runtime with:

```bash
python3 scripts/deploy/prepare_runtime.py --package-dir <extracted-release>
sudo python3 scripts/deploy/prepare_runtime.py --package-dir <extracted-release> --apply
```

The first command is read-only. The second creates a new version-specific
environment and checks installed imports; it does not activate a release or
start a Product. Use its printed installer command to select that interpreter.
Native system libraries and a complete offline bundle remain separate requirements.

```bash
# Product-scoped checkout build and activation; not a release-package input
bash scripts/deploy/deploy_robot.sh teleop_avoid

# Install current Thunder services
bash scripts/deploy/thunder/install_services.sh field-cpp

# Build the complete native release set, then package it
# Match RobotConfig.driver.backend and RobotConfig.camera.capture_driver.
export LINGTU_DRIVER_BACKEND=go2 LINGTU_CAMERA_DRIVER=realsense_native
make build BUILD_TYPE=Release
bash scripts/deploy/package_native_release.sh "v$(cat VERSION)" dist

# Build and package the Web UI independently (requires Node.js, not a robot)
make package-web

# Install an extracted native release
bash scripts/deploy/install_native_release.sh --package-dir <release-dir>

# Verify release metadata without rewriting files
bash scripts/deploy/sync_versions.sh --check

# Pull the current branch in an existing robot checkout
pwsh scripts/deploy/sync_robot.ps1 -TargetHost <host> -TargetUser <user>
```

## What the field package contains

The field deliverable is one Linux archive for a selected architecture and
robot driver. It combines native runtime artifacts, the Python application
source and operating-mode/device configuration. The Web dashboard is a separate
static archive; building or packaging the robot does not require Node.js.
It is not a Python wheel or a copy of the developer checkout.

```text
build/ + application/configuration sources
  -> install/linux-<arch>/Release/{bin,lib,etc,share}
  -> dist/lingtu-<version>-<arch>-native-release.tar.gz
  -> /opt/lingtu/releases/v<version>
  -> /opt/lingtu/current
```

| Package content | Purpose |
| --- | --- |
| `bin/` | SLAM, maps/prune, navigation, selected robot driver, sensors, control tools and recording/player executables |
| `lib/` | Installed project runtime libraries and explicitly staged SDK libraries |
| `etc/lingtu/`, `config/` | Runtime declarations and configuration defaults; deployed per-unit settings still need to be prepared and preserved |
| `share/lingtu/` | Installed runtime assets and DDS schemas |
| `src/`, `pyproject.toml`, `uv.lock` | Python Host/Gateway and application dependency declarations |
| `metadata.json`, release manifests, `install_nav.sh` | Release identity, artifact inventory and the installer |

The packager supports `aarch64` and `x86_64`. Set the driver for the actual
robot before building: `go2` or `doso`. Changing an architecture label does not
cross-compile the binaries. Build on the target or a compatible Linux build
host with the intended architecture and runtime ABI. Windows development
binaries cannot be included in this field archive.

Use the same camera selection for build and packaging: `realsense_native` for
the current Go2 profile, or `orbbec_native` for the current Thunder profile.
The latter is the default. The packager requires the selected capture binary
and camera SDK runtime. Orbbec includes its SDK/depthengine, while RealSense
includes the matching `librealsense2` runtime from the selected build SDK.
These choices must match the selected RobotConfig, not the name of the
computer running the build.

## Deploy the Web UI independently

`make package-web` builds with `web/package-lock.json` and writes
`dist/lingtu-<version>-web.tar.gz`. The archive contains the root page, `/guide/`
and static assets. Frontend CI also retains `web/dist/` as a separate artifact.
Use a Web build tested with the intended robot release; independent packaging
does not promise compatibility between arbitrary versions.

For the existing same-machine Gateway deployment, extract the Web archive
outside `/opt/lingtu/current`, for example into `/opt/lingtu/web/2.3.0`:

```bash
sudo mkdir -p /opt/lingtu/web/2.3.0
sudo tar -xzf lingtu-2.3.0-web.tar.gz -C /opt/lingtu/web/2.3.0
```

Set `LINGTU_WEB_DIST=/opt/lingtu/web/2.3.0` in the existing
`/etc/lingtu/gateway.env` before activating the new robot release. The persistent
`lt-control` entry reads the directory at service startup. A robot release upgrade leaves this
independent Web directory intact. For a Web-only update, prepare a new directory,
change that setting, wait for any lifecycle operation to finish and restart
`lt-control.service` while stationary. Existing browser connections reconnect.
The Product keeps running and native input expiry handles disconnected manual
control. Restore the previous directory setting and restart the entry to roll
back the Web build.

Without an explicit directory, a development checkout still serves
`web/dist/` when present. Without either, Gateway serves APIs with no dashboard.
An explicitly configured missing or incomplete Web directory is logged as a
configuration error; Gateway keeps serving APIs without mounting a different UI.
Migrating from an older bundled release therefore requires deploying the Web
archive and setting this path before switching the robot release.

For hosting on the operator computer, keep the browser's URLs same-origin:
serve the archive there and reverse-proxy `/api/`, `/ws/`, `/mcp`, `/map/` and
`/robot/` to that robot's Gateway. Preserve WebSocket upgrades and unbuffered
SSE responses. In development, Vite's `ROBOT_HOST=<host>:5050` proxy provides
those paths. WHEP/WebRTC media also needs the robot's advertised media port
(currently TCP 8555) reachable; HTTP proxying alone does not carry that media.
The JPEG camera WebSocket uses the regular `/ws/` proxy. Verify live data and
camera transport on the intended network before claiming remote Web delivery.

## Build and inspect a local candidate

Use a fixed source snapshot, the matching native SDKs and configured dependency
prefixes. Record relevant working-tree changes as well as the commit; the
packager's commit field alone does not describe a dirty source snapshot.
Prepare the dependencies described in [Build](../build/README.md), then run
the complete build above. A mode-scoped checkout deployment is not a substitute
for that build.

The planning core is linked into `navd`; mapd builds and edits OctoMap in its
own process. Standalone OctoPlanner3D PCD conversion tools remain offline
developer tools and are not required inputs to the field release.

Package only after the required build trees are available. Use the release
version from `VERSION`, including the `v` prefix expected by the packager:

```bash
bash scripts/deploy/sync_versions.sh --check
bash scripts/deploy/package_native_release.sh "v$(cat VERSION)" dist
```

For `VERSION=2.3.0` on ARM64, the outputs are:

```text
dist/lingtu-2.3.0-aarch64-native-release.tar.gz
dist/lingtu-2.3.0-aarch64-native-release.sha256
dist/lingtu-2.3.0-aarch64-native-release-manifest.json
dist/lingtu-2.3.0-aarch64-native-release-system-manifest.json
```

On a prepared target, extract the archive and inspect it before installation:

```bash
tar -xzf lingtu-2.3.0-aarch64-native-release.tar.gz
cd lingtu-2.3.0-aarch64-native-release
bash ./install_nav.sh --dry-run
sudo bash ./install_nav.sh
```

The installer rejects an already existing release directory of the same
version. The dry run checks the package and installation inputs; it does not
load sensors, prove library availability or qualify navigation. Real installation
can restart the active Product with its retained robot, map and variant.
Keep the robot stationary and save any needed scans first. On a new machine
without an active Product record, installation does not select a Product.
First-time runtime/service setup belongs to
[Getting started](../../docs/getting-started.md#first-robot-installation).

The installer resolves Python once: an explicit `LINGTU_PYTHON`, then the
existing `python.env`, then the shared interpreter helper's default. The same
interpreter reads metadata/current state and invokes ProductControl for both
activation and rollback. Select an environment containing the locked application
dependencies; do not assume the system `python3` has them.

## Dependencies and current delivery limits

This archive assumes a prepared Linux target. It does not currently include a
Python interpreter/virtual environment or an offline wheelhouse. The Python
selection helper chooses an existing interpreter. The separate
`prepare_runtime.py --apply` entry installs locked Host/Gateway/Agent dependencies into a new environment
and verifies imports; it needs uv plus network access or a prepared uv cache.
The target also needs native shared libraries
not explicitly staged by the install rules, such as the matching SLAM and
transport dependencies. Packaging success does not prove these are installed.

Python CI and the development/container entrypoints now use `uv sync --locked`
with their declared extras. Docker installs non-editable packages into an
independent environment so deleting a build-time source directory cannot break
imports. Linux CI and the native image now call the pinned GTSAM CPU preparation
entrypoint; its runtime libraries are included by CMake installation. The selected
robot SDK still needs its matching preparation. Neither a lock dry-run nor the
package fixture tests prove a full image builds. See the
[GTSAM package instructions](../build/README.md#gtsam-cpu-dependency-package).

Maps, recordings, per-unit calibration and optional perception/policy models
are separate deployment inputs. Retain the selected machine configuration and
saved maps across software upgrades, and prepare model artifacts only for the
capabilities being delivered. An application archive does not replace these
inputs or prove their compatibility.

The release workflow currently selects DOSO on an ARM64 runner; it is not a
Go2/Thunder/architecture qualification matrix. It can publish GitHub Release
and OTA artifacts. Local candidate packaging does not publish them; review
the material and obtain the existing release confirmation before invoking the
publishing workflow. The repository Dockerfile is a separate environment
definition, not this native field installer.

For package/installer fixture checks, use the existing self-test:

```bash
bash scripts/deploy/package_native_release.sh --self-test
```

It uses simulated artifacts and a test control entry, proving package and
installer behavior rather than the target binaries. Complete delivery still
needs a prepared-target install, no-motion checks, and the named robot's
supervised scenario as defined in [Testing](../../docs/testing.md).

## Install layout and rollback compatibility

By default, the packager installs the complete `build/` input set into
`install/linux-<arch>/<config>/{bin,lib,etc,share}` before assembling the
archive. Set `LINGTU_NATIVE_RELEASE_INSTALL_SOURCE` only when CI provides an
equivalent pre-staged prefix. A Product-scoped `deploy_robot.sh` build is not a
complete packager input. Phase-one releases also carry the former `build/`
layout for rollback compatibility, but current services use
`/opt/lingtu/current/bin` and `/opt/lingtu/current/lib`.

Roll out the path change in order: activate one dual-layout release while the
old units are still installed; activate the next dual-layout release so the
first becomes its rollback target; then run `install_services.sh` to install
the canonical units. The installer accepts only a dual-layout `current` tree.
Remove the packaged `build/` copy only after that rollback window closes.

`package_native_release.sh` is the Linux field-release packager. Windows x64
uses the same CMake install rules with a prefix such as
`install/windows-x64/Release`; it is not assembled by this Linux OTA script.

For a Go2 real target, `deploy_robot.sh` first applies the
`driver.network_interface` and `driver.network_address` from RobotConfig through
`configure_go2_network.sh`. The generated NetworkManager connection has no
gateway and cannot replace the target's normal default route. Deployment stops
before building if the configured Go2 probe host is unreachable.

The Product argument is required because its process roles determine the native
build list. To inspect that plan without changing the target:

```bash
LINGTU_DEPLOY_PLAN_ONLY=1 bash scripts/deploy/deploy_robot.sh teleop_avoid
```

## Thunder runtime

`deploy/thunder/` contains:

- `lt-*.service`: installed systemd units;
- `install_services.sh`: catalog-driven installer for the complete stack or one service;
- `install_catalog_service.sh`: shared unit installer;
- `install_driver_service.sh`: driver installer with RobotConfig validation;
- `run_*.sh`: process wrappers used by those units;
- `runtime-env.sh` and `require_product_session.sh`: shared runtime setup.

The native field units are `lt-lidar`, `lt-slam`, `lt-maps`, `lt-terrain`,
`lt-nav`, `lt-driver`, `lt-camera`, `lt-explore`, `lt-gnss`, and `lt-host`.
Installing services does not start a Product.

`99-lingtu-orbbec-gemini335.rules` is the minimal USB permission rule installed
by `tools/robot/setup_network.sh --permanent` for the field camera.

The release runbook is `docs/operations.md`. Stable deployment sync helpers
live here; developer-only network utilities remain under `tools/`.
