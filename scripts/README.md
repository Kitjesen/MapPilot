# Scripts

`scripts/` contains durable executable entrypoints. Product implementation and
offline utilities do not belong here.

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
| `codegen/` | DDS IDL code generation |
| `deploy/` | Release packaging, robot installation, and systemd units |
| `diagnostics/` | Current field and native component diagnostics |
| `gates/` | Field acceptance and evidence commands |
| `sim/` | Simulation launch, visual acceptance, and Windows packaging |

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
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
python scripts/gates/system_acceptance_gate.py --help
```

Add a new script only when it is a stable build, deploy, field diagnostic,
acceptance, or simulation entrypoint. Otherwise put it in the
owning source, tool, integration, or test tree.
