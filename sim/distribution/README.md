# Simulation Distribution

`sim/distribution/` owns platform-specific packaging and packaged-build smoke
policy for simulation deliverables.

## Contents

- `windows/`: RobotSimUE Win64 preflight, deterministic distribution planning,
  packaging, smoke checks, and release policy.

## Entry points

```powershell
python -m sim.distribution.windows preflight
python -m sim.distribution.windows dry-run
python -m sim.distribution.windows package
```

## Boundary

Windows is the only distribution target currently implemented here. This
directory consumes qualified plans and assets; it does not compile sessions,
change runtime behavior, or establish field-hardware readiness.
