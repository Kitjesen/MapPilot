# Simulation Sessions

`sim/sessions/` contains SessionSpecs that select exact package versions and
parameters for compilation into one resolved simulation plan.

## Contents

- `products/`: Product-selected simulation presets grouped by robot and mode.
- `examples/`: concrete example SessionSpecs for development and acceptance.
- `game-selection.v1.yaml`: the game-facing list of selectable sessions.

## Entry points

Compile an existing SessionSpec through the single Catalog resolver:

```powershell
python -m sim.catalog resolve sim/sessions/examples/thunder_omni_contract/session.yaml --repo-root . --output-dir build/runtime-session
```

## Boundary

A SessionSpec selects packages but is not itself a package. Session files do
not own package assets, process IDs, ports, shared-memory names, mutable runtime
state, or lifecycle side effects.
