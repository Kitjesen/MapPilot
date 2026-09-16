# Deployment

`scripts/deploy/` owns release packaging, robot installation, and systemd
assets. Product startup remains owned by ProductControl.

## Main commands

```bash
# Product-scoped checkout build and activation; not a release-package input
bash scripts/deploy/deploy_robot.sh teleop_avoid

# Install current Thunder services
bash scripts/deploy/thunder/install_services.sh field-cpp

# Build the complete native release set, then package it
LINGTU_DRIVER_BACKEND=<go2-or-doso> make build BUILD_TYPE=Release
bash scripts/deploy/package_native_release.sh <version> <output-dir>

# Install an extracted native release
bash scripts/deploy/install_native_release.sh --package-dir <release-dir>
```

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

The release runbook is `docs/operations.md`; developer-only sync and
network utilities live under `tools/`.
