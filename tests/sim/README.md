# Simulation Tests

`tests/sim/` contains simulation unit, contract, integration, packaging, and
acceptance regressions in one flat pytest suite.

## Contents

- `conftest.py`: shared pytest setup and fixtures.
- `test_sim_*` and `test_session_*`: layout, Catalog, schema, Session, and
  Product-boundary contracts.
- `test_mujoco_*` and runtime tests: physics, sensors, control, recording,
  replay, and native acceptance behavior.
- `test_robotsimue_*`, `test_ue_*`, and `test_windows_*`: visual frontend,
  Unreal toolchain, and Windows distribution contracts.
- World, asset, importer, and evaluation test modules cover their named
  packages and offline tools.

## Entry points

```powershell
python -m pytest tests/sim/test_sim_layout_contract.py -q
```

## Boundary

These tests prove repository and local simulation contracts at their stated
level. They do not prove S100P behavior, real sensor fidelity, supervised field
motion, or deployment readiness.
