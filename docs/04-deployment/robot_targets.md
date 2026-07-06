# Robot Deployment Targets

## Thunder Field Robot

| Field | Value |
| --- | --- |
| Primary SSH | `ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc` |
| LAN SSH | `ssh sunrise@192.168.66.13` |
| SSH user | `sunrise` |
| Role | Primary robot target for deploying LingTu navigation algorithms |
| Primary release path | `/opt/lingtu/current` via `scripts/deploy/cut_release.sh` |
| Legacy OTA path | `/opt/lingtu/nav/current` via `scripts/ota/install_nav.sh` |
| Dev checkout | `~/data/SLAM/navigation` or `~/data/inovxio/lingtu` |
| Gateway | LAN only: `http://192.168.66.13:5050` |
| MCP | LAN only: `http://192.168.66.13:8090/mcp` |
| NATAPP SSH tunnel | `ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc` |

Do not store the SSH password in this repository. Use an SSH key, a local
password manager, or a local-only environment variable when automation needs it.

### NATAPP Remote SSH

NATAPP is installed on the robot under `/opt/natapp` and runs as the `natapp`
systemd service.

```text
fe91fae6a6756695.natapp.cc:12346
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
ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc -> ubuntu / sunrise / aarch64
sshd listens on :22 and :2222
```

Useful commands:

```bash
ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc
ssh sunrise@192.168.66.13
ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc 'bash ~/data/SLAM/navigation/scripts/lingtu status'
ssh -p 12346 sunrise@fe91fae6a6756695.natapp.cc 'bash ~/data/SLAM/navigation/scripts/lingtu health'
```
