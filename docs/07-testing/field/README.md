# Field Validation

Status: current reusable field-gate index as of 2026-09-03.

These documents define checks that run on a physical robot or its production
compute target. They are reusable procedures, not proof that a particular run
passed.

## Gate Index

| Gate | Purpose |
| --- | --- |
| [Field mapping acceptance](FIELD_MAPPING_ACCEPTANCE.md) | Proves a saved map has useful trajectory/keyframe coverage and valid navigation artifacts. |
| [Native endpoint field acceptance](NATIVE_ENDPOINT_REFACTOR_FIELD_ACCEPTANCE.md) | Proves the selected native endpoint chain, lifecycle, stop, and field ownership boundaries. |
| [Semantic-memory checklist](SEMANTIC_MEMORY_FIELD_CHECKLIST.md) | Collects reusable semantic-memory field evidence. |
| [Go2 EDU + MID-360 assisted avoidance](../../04-deployment/go2_edu_mid360_teleop_avoid.md) | Deploys and validates the current Go2 `teleop_avoid` chain. |

## Required Order

1. Resolve the exact Product and inspect its RunPlan.
2. Verify target architecture, dependencies, network interfaces, and selected
   native artifacts.
3. Start through ProductControl and collect field no-motion readiness.
4. Verify sensor, SLAM, map, traversability, navigation, and driver freshness
   without publishing motion.
5. Run a separately supervised bounded-motion scenario only when the applicable
   procedure explicitly calls for it.
6. Stop the Product and record final zero-command and driver acknowledgement.

The shared no-motion entry is:

```bash
bash scripts/lingtu status
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
```

For Go2 `teleop_avoid`, use the stricter contract and motion collectors listed
in the dedicated deployment guide.

## Executable P0 Procedures

Executable operator scripts live outside `docs/` under `scripts/gates/field/`:

| Script | Scope |
| --- | --- |
| `go2_mid360_no_motion.sh` | Starts only Go2 MID-360 and Fast-LIO, proves fresh typed DDS data and zero `/nav/cmd_vel`, then stops both processes. |
| `p0_cold_boot.sh` | Product cold start and readiness stability. |
| `p0_mapping.sh` | Mapping, save, artifact checks, and transition to `nav`. |
| `p0_route_safety.sh` | No-motion route preview and command-source check. |
| `p0_goto.sh` | Explicitly confirmed supervised point-goal motion. |
| `p0_estop.sh` | Stop-command and reported-zero timing during supervised motion. |
| `p0_explore.sh` | Explicit exploration start, observation, and stop. |
| `p0_all.sh` | Ordered wrapper; motion targets and exploration remain explicit. |

These scripts are not background health checks. Read the selected script and
the robot-specific deployment guide before executing it.

## Evidence Destination

Every run creates a new date-prefixed record under
[`../field-runs/`](../field-runs/README.md). Do not edit an old record to make a
new release look successful. Large logs and binary captures stay in the run's
external evidence directory; the Markdown record preserves their paths and the
decisive measurements.
