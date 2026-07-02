# Robot Deployment Targets

## Thunder Field Robot

| Field | Value |
| --- | --- |
| Host | `192.168.66.13` |
| SSH user | `sunrise` |
| Role | Primary robot target for deploying LingTu navigation algorithms |
| Primary release path | `/opt/lingtu/current` via `scripts/deploy/cut_release.sh` |
| Legacy OTA path | `/opt/lingtu/nav/current` via `scripts/ota/install_nav.sh` |
| Dev checkout | `~/data/SLAM/navigation` or `~/data/inovxio/lingtu` |
| Gateway | `http://192.168.66.13:5050` |
| MCP | `http://192.168.66.13:8090/mcp` |

Do not store the SSH password in this repository. Use an SSH key, a local
password manager, or a local-only environment variable when automation needs it.

Useful commands:

```bash
ssh sunrise@192.168.66.13
ssh sunrise@192.168.66.13 'bash ~/data/SLAM/navigation/scripts/lingtu status'
ssh sunrise@192.168.66.13 'bash ~/data/SLAM/navigation/scripts/lingtu health'
```
