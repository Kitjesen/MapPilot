# Simulation Sensor Boundary

`sim/packages/sensors/` contains canonical SensorPackage manifests and their
package-owned assets.

## Contract

- Package assets stay with their owning SensorPackage.
- Runtime implementations live outside this catalog tree.
- The canonical Mid-360 scan pattern lives at
  `sim/packages/sensors/livox/mid360/assets/mid360.npy`.
- Do not add ROS 2 or robot service startup side effects here.
