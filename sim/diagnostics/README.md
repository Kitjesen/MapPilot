# Simulation Diagnostics

`sim/diagnostics/` builds simulation-only reports for runtime wiring, DimOS
coverage gaps, and Gazebo frame contracts.

## Contents

- `summary.py`: aggregated simulation diagnostics and report output.
- `dataflow_report.py`: shared data-flow report builders.
- `dimos_gap.py` and `gap_report.py`: DimOS gap analysis CLI and report model.
- `gazebo_tf.py`: Gazebo topic and TF contract smoke check.

## Entry points

```powershell
python -m sim.diagnostics --help
python -m sim.diagnostics.dimos_gap --help
python -m sim.diagnostics.gazebo_tf --help
```

## Boundary

These reports provide simulation evidence only. They do not prove field
readiness, authorize motion, or own Product lifecycle and process control.
