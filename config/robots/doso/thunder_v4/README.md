# DOSO Thunder V4

`model.yaml` describes the Thunder V4 and keeps its MID-360 configuration
independent of Go2. Robot selection does not choose `real` or `sim`.

| Area | Owner |
| --- | --- | --- |
| Motion | `src/drivers/real/motion/robots/doso/` |
| MID-360 / Fast-LIO2 | `config/robots/doso/thunder_v4/sensors/mid360_fastlio2.yaml` |
| Simulation | `sim/robots/doso/thunder_v4/` |
| Deployment | `scripts/deploy/thunder/` |
| Products | `config/runtime_graph/products/` |

The same robot ID now resolves in both environments:

- `real` reads the adjacent `robot.yaml` and native DOSO/Brainstem settings.
- `sim` selects a preset from `sim/presets/doso/thunder_v4/`.

This proves configuration and compilation only; field motion still requires the
normal no-motion and supervised-motion acceptance on the target robot.
