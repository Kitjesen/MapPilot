# `lingtu` Product CLI

`scripts/lingtu` is a thin robot-side adapter over
`python -m lingtu.control`. It has exactly three actions:

```bash
bash scripts/lingtu switch PRODUCT [--map MAP] [--initial-pose X Y YAW]
bash scripts/lingtu status [--json]
bash scripts/lingtu stop
```

Select the fixed runtime environment and robot explicitly when they are not
already configured:

```bash
bash scripts/lingtu --robot unitree/go2 --env real switch nav --map factory
bash scripts/lingtu --robot doso/thunder_v4 --env sim switch explore
```

ProductControl owns process ordering, readiness, rollback, the current Product
record, and systemd/subprocess execution. The shell script contains no Product
policy and is not a second operations framework.

## Former shortcuts

Use the Product action directly:

| Intent | Current command |
| --- | --- |
| Start mapping | `bash scripts/lingtu switch map` |
| Start navigation | `bash scripts/lingtu switch nav --map MAP` |
| Start navigation with a seed | `bash scripts/lingtu switch nav --map MAP --initial-pose X Y YAW` |
| Start exploration | `bash scripts/lingtu switch explore` |
| Start saved-map exploration | `bash scripts/lingtu switch explore --map MAP` |
| End the active Product | `bash scripts/lingtu stop` |

Map save/list, navigation goals, relocalization of an already running Product,
and exploration task commands are Gateway operations. They are not
`scripts/lingtu` subcommands; use the authenticated routes documented in
[`gateway_rest.md`](../api/gateway_rest.md).

## Diagnostics and services

Diagnostics use their owning command instead of passing through the Product
CLI:

```bash
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
PYTHONPATH=src python -m diagnostics.field.map_artifacts MAP_ID --require-occupancy
PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" --map MAP --goal X Y YAW
```

Use the platform tools for the platform state:

```bash
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
systemctl --no-pager --full status 'lt-*.service'
journalctl -u 'lt-*' -p err -n 100 --no-pager
```

For one runtime stream:

```bash
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
```

## Recording and replay

Recording is owned by the native tools, not ProductControl:

```bash
/opt/lingtu/current/bin/lingtu_recorder record \
  --output-dir /data/recordings/run-001 --dds on --camera off
/opt/lingtu/current/bin/lingtu_recorder status /data/recordings/run-001
/opt/lingtu/current/bin/lingtu_recorder stop /data/recordings/run-001
/opt/lingtu/current/bin/lingtu_dds_player --info /data/recordings/run-001/dds/sensors.mcap
/opt/lingtu/current/bin/lingtu_dds_player \
  /data/recordings/run-001/dds/sensors.mcap --dry-run
```

See [Native Recording and Replay](native_recording.md) for the complete native
contract.
