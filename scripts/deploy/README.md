# Deployment Script Index

`scripts/deploy/` is the executable deployment source for LingTu field installs
and service installation helpers. Runbooks and explanatory material belong under
`docs/04-deployment/`.

## Canonical Thunder Entry Points

Use these paths for new deployment work:

- `deploy_thunder.sh` - canonical Thunder field deployment entrypoint.
- `thunder/install_services.sh` - canonical Thunder service installer entrypoint.
- 	hunder/install_explore_dds_service.sh - installs the native exploration DDS
  endpoint (lingtu-explore-dds.service); it starts idle and accepts only typed
  exploration lifecycle commands.
- 	hunder/install_nav_dds_service.sh - installs the native navigation DDS
  endpoint service (`lingtu-nav-dds.service`).
- `thunder/install_driver_service.sh` - installs the native Thunder driver
  (`lingtu-driver.service`) that consumes `rt/nav/cmd_vel` and calls the remote
  Brainstem gRPC endpoint from `/opt/lingtu/config/brainstem.env`.
- `thunder/install_lite_service.sh` - installs the Thunder Lite service.
- `thunder/runtime-env.sh` - Thunder runtime defaults shared by service units.
- `thunder/ros2-env.sh` - ROS 2 compatibility environment for legacy ROS-backed
  services.

## Legacy Compatibility

`thunder/install_dds_endpoint_service.sh` and
`lingtu-thunder-dds-endpoint.service` are legacy endpoint compatibility surfaces.
The current product field chain uses `lingtu-nav-dds.service` plus
`lingtu-driver.service`; the driver unit conflicts with the old endpoint units
so there is only one speed exit.

The `s100p` name is retained only for compatibility with existing field scripts
and service files. Do not add new product-facing deployment entrypoints under
`scripts/deploy/s100p/`.

- `deploy_s100p.sh` forwards to `deploy_thunder.sh`.
- `s100p/install_services.sh` installs legacy ROS/SLAM service files and is
  reached from `thunder/install_services.sh ros-compat`.
- `s100p/*.service` remains the compatibility service-file location until those
  units are deliberately migrated.

## Documentation Boundary

- Executable source: `scripts/deploy/thunder/`.
- Compatibility source: `scripts/deploy/s100p/`.
- Runbooks, service inventories, OTA notes, and field procedures:
  `docs/04-deployment/`.

Do not copy scripts into `docs/04-deployment/`. Do not move service files as part
of documentation-only hierarchy cleanup.
