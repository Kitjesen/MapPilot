# Simulation Sensor Boundary

`sim/packages/sensors/` contains canonical SensorPackage manifests and their
package-owned assets.

Current implementation, noise switches, producer/rate differences, and inspection
test evidence are recorded in the [sensor runtime status](../../runtime/sensors/README.md).
The original `1.0.0` navigation rig binds five navigation streams. Explicit
Thunder `1.1.0` navigation suites also declare GNSS, thermal, semantic/instance
and acoustic choices; source Sessions disable these four by default, preserving
the original five outputs. See [selection and availability](../../../docs/design/sensor-selection.md).
An explicit experimental [GNSS package](gnss/sensor.package.yaml) and
[Thunder GNSS-only example](../../sessions/examples/thunderv4_gnss/session.yaml)
now resolve to a native DDS endpoint. The example has a synthetic georeference,
defaults to NO_FIX, and does not itself activate the formal ProductControl feeder.
The separate [Product GNSS session](../../sessions/products/doso/thunder_v4/gnss.yaml)
opts in through the existing formal feeder's IMU endpoint. Defaults remain unchanged.

## Contract

- Package assets stay with their owning SensorPackage.
- Runtime implementations live outside this catalog tree.
- The canonical Mid-360 scan pattern lives at
  `sim/packages/sensors/livox/mid360/assets/mid360.npy`.
- Do not add ROS 2 or robot service startup side effects here.
