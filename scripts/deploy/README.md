# Deployment Script Index

`scripts/deploy/` is the executable deployment source for LingTu field installs
and service installation helpers. Runbooks and explanatory material belong under
`docs/04-deployment/`.

## Canonical Thunder Entry Points

Use these paths for new deployment work:

- `deploy_thunder.sh` - canonical Thunder field deployment entrypoint.
- `thunder/install_services.sh` - canonical Thunder service installer entrypoint.
- `thunder/install_dds_endpoint_service.sh` - installs the Thunder typed DDS endpoint
  service.
- `thunder/install_lite_service.sh` - installs the Thunder Lite service.
- `thunder/runtime-env.sh` - Thunder runtime defaults shared by service units.
- `thunder/ros2-env.sh` - ROS 2 compatibility environment for legacy ROS-backed
  services.

## Legacy Compatibility

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
