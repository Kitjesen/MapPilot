# Simulation Sensor Boundary

`sim/sensors/` contains standalone sensor simulators and fallback models that
are reused by the simulation engine and validation scripts.

## Files

| File | Role |
| --- | --- |
| `livox_mid360.py` | Livox MID-360 style scan pattern and Python fallback LiDAR interface. |

## Contract

- Sensor implementations in this folder must be hardware-free and deterministic
  enough for repeatable tests.
- Runtime engine wrappers live under `sim/engine/`; this folder should expose
  reusable sensor models, not own the full physics loop.
- Canonical Livox assets live in `sim/assets/livox/`; keep path references
  explicit when a gate depends on the official scan pattern.
- Do not add ROS 2 or robot service startup side effects here.
