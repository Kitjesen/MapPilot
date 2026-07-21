# Simulation Drivers

`src/drivers/sim/` owns simulation-side driver backends. Real hardware stays
under `src/drivers/real/`.

## Layout

| Path | Role |
| --- | --- |
| `mujoco/` | In-process MuJoCo driver, runtime, scene, sensor, stack, and adapter helpers. |
| `endpoint.py` | Driver-shaped module for externally owned simulation streams. |
| `pointcloud.py` | Static point-cloud provider from MuJoCo scene geometry. |
| `stub.py` | Legacy import shim; canonical stub driver lives in `lingtu.assembly.stub`. |
