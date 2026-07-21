# Robot Deployment Targets

## Thunder Field Robot

Use variables in portable commands:

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
ssh sunrise@"$LINGTU_HOST"
curl -fsS "http://${LINGTU_HOST}:5050/api/v1/health"
```

| Field | Value |
| --- | --- |
| Primary SSH | `ssh sunrise@${LINGTU_HOST}` |
| LAN SSH | `ssh sunrise@<robot-ip>` |
| SSH user | `sunrise` |
| Role | Primary robot target for deploying LingTu navigation algorithms |
| Primary release path | `/opt/lingtu/current` via `scripts/deploy/cut_release.sh` |
| Legacy OTA path | `/opt/lingtu/nav/current` via `scripts/ota/install_nav.sh` |
| Dev checkout | `~/data/SLAM/navigation` or `~/data/inovxio/lingtu` |
| Gateway | `http://${LINGTU_HOST}:5050` |
| MCP | `http://${LINGTU_HOST}:8090/mcp` |
| Lab NATAPP SSH tunnel | Use only from the private lab runbook; do not copy into public docs or clients. |

Do not store the SSH password in this repository. Use an SSH key, a local
password manager, or a local-only environment variable when automation needs it.

### Lab NATAPP Remote SSH

This section is a current lab example, not a portable deployment contract.
NATAPP is installed on the robot under `/opt/natapp` and runs as the `natapp`
systemd service in that lab.

```text
<lab-natapp-host>:<lab-natapp-port>
  -> robot local port 2222
  -> sshd
```

The robot keeps normal LAN SSH on port `22` and also listens on port `2222` for
the NATAPP TCP tunnel. The sshd drop-in is:

```text
/etc/ssh/sshd_config.d/99-lingtu-natapp-port.conf
```

Verified on 2026-07-05:

```text
systemctl is-enabled natapp -> enabled
systemctl is-active natapp  -> active
ssh -p <lab-natapp-port> sunrise@<lab-natapp-host> -> ubuntu / sunrise / aarch64
sshd listens on :22 and :2222
```

Useful commands:

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
ssh sunrise@"$LINGTU_HOST"
ssh sunrise@"$LINGTU_HOST" 'bash /opt/lingtu/current/scripts/lingtu status'
ssh sunrise@"$LINGTU_HOST" 'bash /opt/lingtu/current/scripts/lingtu health'
```
