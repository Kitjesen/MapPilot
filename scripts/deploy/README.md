# Deployment

`scripts/deploy/` owns release packaging, robot installation, and systemd
assets. Product startup remains owned by ProductControl.

## Main commands

```bash
# Build and deploy one Product to the configured robot
bash scripts/deploy/deploy_robot.sh teleop_avoid

# Install current Thunder services
bash scripts/deploy/thunder/install_services.sh field-cpp

# Package or install a native release
bash scripts/deploy/package_native_release.sh <version> <output-dir>
bash scripts/deploy/install_native_release.sh --package-dir <release-dir>

# Deploy one Product and package the resulting checkout
bash scripts/deploy/deploy_robot.sh <product>
bash scripts/deploy/package_native_release.sh <version> <output-dir>
```

The packager consumes one standard prefix at
`install/linux-<arch>/<config>/{bin,lib,etc,share}`. Set
`LINGTU_NATIVE_RELEASE_INSTALL_SOURCE` only when CI provides an equivalent
pre-staged prefix. Phase-one releases also carry the former `build/` layout for
rollback compatibility, but current services use `/opt/lingtu/current/bin`
and `/opt/lingtu/current/lib`.

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

Release runbooks live under `docs/04-deployment/`; developer-only sync and
network utilities live under `tools/`.
