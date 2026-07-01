# Deployment Service References

This directory documents the historical and runbook-facing systemd service set.
It is not the canonical location for new executable deployment entrypoints.

For new Thunder deployment execution, use `scripts/deploy/deploy_thunder.sh` and
`scripts/deploy/thunder/`.

The service files in this directory remain in place until service-file migration
is handled deliberately. Do not move them as part of deployment hierarchy
documentation cleanup.
