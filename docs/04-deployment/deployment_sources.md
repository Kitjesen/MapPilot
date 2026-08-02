# Deployment Sources

This directory is the explanatory and runbook source for deployment. It should
describe how to install, operate, diagnose, and roll back LingTu deployments.

Executable deployment scripts live under `scripts/deploy/`:

- Canonical Thunder executable source: `scripts/deploy/thunder/`.
- The only deploy entrypoint is `scripts/deploy/deploy_thunder.sh`. It
  deploys/builds artifacts by default; optional activation uses
  `LINGTU_DEPLOY_PRODUCT=<product>` or a Product argument and delegates to
  `scripts/lingtu --env real mode switch`.

No executable deployment source or alias remains under the S100P name. The
former ROS2 installer and systemd unit templates are removed; historical S100P
hardware references are not deployment authority.
