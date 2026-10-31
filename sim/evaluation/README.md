# Simulation Evaluation

`sim/evaluation/` contains offline metrics, replay gates, retained datasets,
and package qualification records.

## Contents

- `data/`: retained SLAM datasets and OctoPlanner3D fixtures.
- `slam/`: trajectory formats, metrics, dataset manifests, and inspection
  localization readiness checks.
- `package_qualifications/`: versioned qualification records and their evidence.
- `navigation_replay.py`: offline navigation replay and deviation gate.
- `task_flow.py`: deterministic MuJoCo model-truth to hierarchical-SceneGraph grounding contract.

## Entry points

```powershell
python -m sim.evaluation.navigation_replay --help
python -m sim.evaluation.task_flow --json
```

`task_flow` reads fixed desk and box poses from a small MuJoCo model, then uses
the production task schema, semantic planner, and fast goal resolver with a
fixed structured response for `先去会议室，然后找到桌子旁边的箱子`. The
SceneGraph's perception labels, room hierarchy, and `near` relation are fixture
inputs rather than camera or detector output. The gate checks both symbolic
steps can be grounded against one map identity, while the returned task plan
stays pending and no motion command is emitted. It does not run perception,
global or local planning, task execution, robot physics, or field motion.

## Boundary

Evaluation consumes recorded or generated evidence; it does not own simulation
runtime, package definitions, or Product lifecycle. Passing an offline gate is
not evidence that the same behavior works on the field robot.
