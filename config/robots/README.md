# Robots

Robot-specific entry points live under the manufacturer and model:

```text
config/robots/<vendor>/<model>/
├── README.md
├── model.yaml
├── robot.yaml        # physical configuration used by real, when available
└── sensors/
    └── mid360_ltlio.yaml
```

Robot and Env are independent selections. `model.yaml` describes the model and
its sensor configuration; it does not allow or forbid `real` or `sim`.
Physical network and driver values live in the adjacent `robot.yaml`. The
simulation session is selected by `config/system/envs/sim.yaml`.

Repository-owned path segments use lowercase `snake_case`. Vendor and model
identifiers are stable catalog keys, so rename them only together with their
RobotCatalog consumers and simulation package bindings.

The repository currently has these concrete inputs:

| Robot | `real` | `sim` |
| --- | --- | --- |
| `unitree/go2` | Configured | Missing Go2 simulation assets and preset |
| `doso/thunder_v4` | Configured | Configured |
| `paxini/tora_double_one_x` | URDF and navigation envelope; driver/calibration pending | Ideal-base garage navigation test; no wheel-physics Product session |

A missing combination fails because its concrete configuration or assets are
absent, not because the model is restricted to one Env.
