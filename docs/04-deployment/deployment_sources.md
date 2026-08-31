# Deployment Sources

This directory is the explanatory and runbook source for deployment. It should
describe how to install, operate, diagnose, and roll back LingTu deployments.

Executable deployment scripts live under `scripts/deploy/`:

- Shared field service catalog: `scripts/deploy/thunder/`. The directory name is
  historical; the same unit catalog is used with the Go2 or DOSO adapter selected
  by RobotConfig and RunPlan.
- The only deploy entrypoint is `scripts/deploy/deploy_robot.sh`. It
  requires `LINGTU_DEPLOY_PRODUCT="$LINGTU_PRODUCT"` or a Product argument,
  builds only that Product's native processes, and delegates activation to
  `scripts/lingtu --robot "$LINGTU_ROBOT" --env real switch`. Set
  `LINGTU_DEPLOY_PLAN_ONLY=1` to inspect the resolved plan without changes.

The current Go2 EDU + external MID-360 deployment prerequisites, limitations,
and field sequence are documented in `go2_edu_mid360_teleop_avoid.md`. That
document is authored by LingTu; linked Unitree/Livox material is reference input,
not deployment authority. Active cross-Product implementation priorities remain
owned by `docs/plans/current-roadmap.md`.

No executable deployment source or alias remains under the S100P name. The
former ROS2 installer and systemd unit templates are removed; historical S100P
hardware references are not deployment authority.
