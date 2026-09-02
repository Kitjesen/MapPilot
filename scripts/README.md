# Scripts

`scripts/` contains durable Shell and PowerShell entrypoints. It is not a
Python package; Product implementation, diagnostics, acceptance logic, and
offline utilities live with their owning packages.

## Product lifecycle

Use the installed `lingtu` command or the repository adapter:

```bash
scripts/lingtu switch teleop --robot unitree/go2 --env real
scripts/lingtu status --robot unitree/go2 --env real
scripts/lingtu stop --robot unitree/go2 --env real
```

`scripts/lingtu` only executes `python -m lingtu.control`. It does not contain
Product policy, Gateway calls, map operations, diagnostics, recording, or
systemd logic.

## Directory map

| Path | Contents |
| --- | --- |
| `build/` | Native component builds and required build inputs |
| `deploy/` | Release packaging, robot installation, and systemd units |
| `gates/field/` | Stable P0 field Shell procedures |
| `sim/` | Stable simulation Shell and PowerShell entrypoints |

Developer-only, offline, and integration tools live under `tools/`,
`tests/`, and `integrations/`.

## Common entrypoints

```bash
# Build native field components
bash scripts/build/build_driver.sh
bash scripts/build/build_slam_core.sh
bash scripts/build/build_mapd.sh
bash scripts/build/build_nav_endpoint.sh

# Deploy one Product and install the field runtime
bash scripts/deploy/deploy_robot.sh teleop_avoid
bash scripts/deploy/thunder/install_services.sh field-cpp

# Run explicit diagnostics and gates
PYTHONPATH=src python -m diagnostics.field.doctor
PYTHONPATH=src python -m diagnostics.field.soak --help
PYTHONPATH=src python -m diagnostics.field.system_acceptance --help
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
```

Add a new script only when it is a stable build, deploy, P0 field, or
simulation Shell/PowerShell entrypoint. Python commands belong in the owning
package and run with `python -m`.
