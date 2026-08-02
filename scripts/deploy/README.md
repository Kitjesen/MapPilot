# Deployment Script Index

`scripts/deploy/` is the executable deployment source for LingTu field installs
and service installation helpers. Runbooks and explanatory material belong under
`docs/04-deployment/`.

## Canonical Thunder Entry Points

Use these paths for new deployment work:

- `deploy_thunder.sh` - canonical Thunder field deployment entrypoint. It builds/stages deployment artifacts; set `LINGTU_DEPLOY_PRODUCT=<product>` or pass a Product name only when activation should be delegated to `scripts/lingtu --env real mode switch`.
- `thunder/install_services.sh` - canonical Thunder service installer entrypoint.
- `thunder/install_explore_dds_service.sh` - installs the native exploration DDS
  endpoint (lingtu-explore-dds.service); it starts idle and accepts only typed
  exploration lifecycle commands.
- `thunder/install_nav_dds_service.sh` - installs the native navigation DDS
  endpoint service (`lingtu-nav-dds.service`).
- `thunder/install_driver_service.sh` - installs the native Thunder driver
  (`lingtu-driver.service`) that consumes `rt/nav/cmd_vel` and calls the remote
  Brainstem gRPC endpoint from `/opt/lingtu/config/brainstem.env`.
- `thunder/runtime-env.sh` - Thunder runtime defaults shared by service units.
- `thunder/ros2-env.sh` - ROS 2 compatibility environment for legacy ROS-backed
  services.

## Removed Compatibility Surfaces

The field Python DDS deployment unit, installer, and wrapper were physically
removed. The `env=real` process chain uses `lingtu-nav-dds.service` plus
`lingtu-driver.service`; Python endpoint replay and smoke diagnostics run
explicitly with `PYTHONPATH=src python -m runtime.endpoints.dds.endpoint_runner`.

Exact legacy unit names remain only as negative cleanup tombstones in the real
env conflicts, the driver unit conflicts, and driver installation cleanup. They
are not catalog entries, install modes, release fallbacks, or startable files.

The former S100P deploy alias, ROS2 installer, and unit templates are removed.
`deploy_thunder.sh` is the only deployment entrypoint, and
`thunder/install_services.sh` accepts catalog modes only. Do not recreate a
parallel boot path under `scripts/deploy/s100p/`.
## Documentation Boundary

- Executable source: `scripts/deploy/thunder/`.
- Runbooks, service inventories, OTA notes, and field procedures:
  `docs/04-deployment/`.

Do not copy scripts into `docs/04-deployment/`. Do not move service files as part
of documentation-only hierarchy cleanup.
