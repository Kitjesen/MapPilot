# Simulation adapters

Adapters translate at runtime seams but do not advance physics or own session
lifecycle.

## Contents

| Path | Responsibility |
| --- | --- |
| `dds/` | Convert simulation state, sensors, and commands to LingTu typed DDS contracts. |
| `shm/` | Carry RobotSimUE RGB/depth frames through camera shared memory. |
| `gazebo/` | Keep the explicit ROS/Gazebo compatibility bridge and TF smoke surface. |

## Boundary

Adapters translate protocols. They do not own simulation time, physics,
SessionRuntime lifecycle, Product selection, or motion policy. Add a sibling
only for a distinct transport seam.
