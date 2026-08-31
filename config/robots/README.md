# Robots

Robot-specific entry points live under the manufacturer and model:

```text
config/robots/<company>/<model>/
├── README.md
├── model.yaml
├── robot.yaml        # physical configuration used by real, when available
└── sensors/
    └── mid360_fastlio2.yaml
```

Robot and Env are independent selections. `model.yaml` describes the model and
its sensor configuration; it does not allow or forbid `real` or `sim`.
Physical network and driver values live in the adjacent `robot.yaml`. The
simulation session is selected by `config/runtime_graph/envs/sim.yaml`.

The repository currently has these concrete inputs:

| Robot | `real` | `sim` |
| --- | --- | --- |
| `unitree/go2` | Configured | Missing Go2 simulation assets and preset |
| `doso/thunder_v4` | Configured | Configured |

A missing combination fails because its concrete configuration or assets are
absent, not because the model is restricted to one Env.
