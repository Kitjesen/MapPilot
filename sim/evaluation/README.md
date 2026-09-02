# Simulation Evaluation

`sim/evaluation/` contains offline metrics, replay gates, retained datasets,
and package qualification records.

## Contents

- `data/`: retained SLAM datasets and OctoPlanner3D fixtures.
- `slam/`: trajectory formats, metrics, dataset manifests, and inspection
  localization readiness checks.
- `package_qualifications/`: versioned qualification records and their evidence.
- `navigation_replay.py`: offline navigation replay and deviation gate.

## Entry points

```powershell
python -m sim.evaluation.navigation_replay --help
```

## Boundary

Evaluation consumes recorded or generated evidence; it does not own simulation
runtime, package definitions, or Product lifecycle. Passing an offline gate is
not evidence that the same behavior works on the field robot.
