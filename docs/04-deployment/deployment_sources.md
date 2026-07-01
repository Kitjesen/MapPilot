# Deployment Sources

This directory is the explanatory and runbook source for deployment. It should
describe how to install, operate, diagnose, and roll back LingTu deployments.

Executable deployment scripts live under `scripts/deploy/`:

- Canonical Thunder executable source: `scripts/deploy/thunder/`.
- Canonical Thunder deploy entrypoint: `scripts/deploy/deploy_thunder.sh`.
- Legacy compatibility source: `scripts/deploy/s100p/`.

The `s100p` deployment path is retained for compatibility with existing robot
service files and field procedures. New product-facing deployment work should
refer to Thunder naming and use `scripts/deploy/thunder/` unless a procedure is
explicitly about legacy ROS/SLAM compatibility.

Current runbooks in this directory may still mention `s100p` where they describe
legacy service files or historical S100P field state. Those mentions are not new
deployment authority.
