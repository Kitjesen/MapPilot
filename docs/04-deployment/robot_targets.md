# Robot Deployment Targets

Deployment transport is workstation configuration. It is not part of a
Product, RunPlan, or RobotConfig.

```bash
export LINGTU_TARGET_HOST=ROBOT_IP_OR_HOSTNAME
export LINGTU_TARGET_USER=ROBOT_SSH_USER
ssh "${LINGTU_TARGET_USER}@${LINGTU_TARGET_HOST}"
curl -fsS "http://${LINGTU_TARGET_HOST}:5050/api/v1/health"
```

| Field | Value |
| --- | --- |
| SSH | `ssh ${LINGTU_TARGET_USER}@${LINGTU_TARGET_HOST}` |
| Role | Expansion computer that runs the LingTu real environment |
| Active release | `/opt/lingtu/current` |
| Runtime data | `/var/lib/lingtu` |
| Gateway | `http://${LINGTU_TARGET_HOST}:5050` |
| MCP | `http://${LINGTU_TARGET_HOST}:8090/mcp` |

Use `tools/deploy/sync_robot.ps1` from Windows or the normal release process
to transfer code. The target host may be a Go2 expansion board, an S100P, or
another supported aarch64 Ubuntu computer. Its IP and SSH account do not define
the attached robot backend.

Do not store SSH passwords in this repository. Use an SSH key or a local
credential manager.
