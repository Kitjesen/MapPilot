# CLAUDE.md

This file is a concise Claude-facing supplement. The authoritative repository
operating contract is [`AGENTS.md`](./AGENTS.md); do not maintain a second,
stale copy here.

## Current Project Shape

LingTu is a Module-First autonomous navigation runtime for quadruped robots.

- Runtime unit: `Module`
- Orchestration unit: `Blueprint`
- Product boundary: typed ports, explicit wires, and selected transports
- Physical Thunder field path: native C++ services + typed CycloneDDS at
  process boundaries
- ROS 2 role: explicit compatibility/replay/evaluation only, not the product
  runtime API

Current code/config sources of truth:

- Profiles and robot presets: `src/runtime/profiles/catalog/` via
  `runtime.runtime_profiles`
- Runtime endpoint contract: `config/runtime_graph/endpoints/thunder_field.yaml`
- Native DDS IDL: `src/message/idl/`
- Field driver unit: `scripts/deploy/thunder/lingtu-driver.service`
- Architecture index: `docs/architecture/README.md`
- Current doc routing: `docs/CURRENT.md`

## Facts To Preserve

- `thunder_field` is the physical field endpoint.
- `thunder_field` defaults to `command_output_mode: endpoint_only` and
  `hardware_control_boundary: driver`.
- The native field services include `lingtu-livox-dds.service`,
  `lingtu-slam-dds.service`, `lingtu-traversability-dds.service`,
  `lingtu-nav-dds.service`, and `lingtu-driver.service`.
- `lingtu-driver.service` conflicts with the old Python Thunder DDS endpoint
  services and reads `/opt/lingtu/config/brainstem.env`.
- `lingtu-driver` must connect to a remote Brainstem controller; do not
  document localhost Brainstem as the field default.
- Product global planning defaults to `octoplanner3d`.
- `direct` is for lightweight/direct compatibility paths.
- `pct` and `astar` are legacy/manual comparison paths unless a profile
  explicitly selects them.
- Gateway defaults to port `5050`; MCP JSON-RPC defaults to port `8090`.
- Use `<robot-ip>` or `$LINGTU_HOST` in reusable docs. Do not hard-code lab
  addresses except inside dated evidence or `docs/CURRENT.md`.

## Common Commands

Local inspection:

```bash
uv run --locked python lingtu.py --list --all
uv run --locked python lingtu.py show-config stub --json
uv run --locked python lingtu.py runtime-audit
uv run --locked python lingtu.py runtime-spec nav --endpoint thunder_field --json
```

Focused tests:

```bash
python -m pytest src/runtime/tests/ -q
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py -q
python -m pytest src/runtime/tests/test_thunder_deployment_entrypoints.py -q
python -m pytest src/runtime/tests/test_thunder_service_catalog.py -q
```

Native field build helpers:

```bash
bash scripts/build/build_nav_endpoint.sh
bash scripts/build/build_octoplanner3d.sh
```

Robot-side operations, on the deployed target:

```bash
bash scripts/lingtu status
bash scripts/lingtu doctor
bash scripts/lingtu map start
bash scripts/lingtu map save <map-name>
bash scripts/lingtu nav start <map-name>
```

## Documentation Rules

- Update `AGENTS.md` only when the repository operating contract changes.
- Update `docs/CURRENT.md` when authority/routing changes.
- Update `docs/plans/current-roadmap.md` for active roadmap/gate status.
- Put dated validation evidence under `docs/07-testing/field-runs/`.
- Keep plans and evidence separate from current behavior.
- Do not present local tests, simulation, or no-motion DDS checks as real
  motion closure.
