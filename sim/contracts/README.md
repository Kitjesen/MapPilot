# Simulation Contracts

`sim/contracts/` contains shared, versioned data contracts used across package
compilation, runtime, recording, replay, and evaluation.

## Contents

- `schemas/`: JSON Schemas for packages, sessions, resolved plans, transport,
  snapshots, visual projections, and qualification evidence.
- `timebase.py`: exact physics-step and periodic-sensor compatibility checks.

## Boundary

Contracts describe data; they do not select Products, allocate processes, or
run simulation. Existing JSON Schema `$id` values are stable public identities:
change a schema compatibly or add a new version instead of repurposing an ID.
