# ADR-001: Simulation package boundaries

Status: Accepted

The generic simulation Runtime consumes versioned `RobotPackage`,
`ControllerPackage`, `SensorPackage`, `SensorRigPackage`, `WorldPackage`, and
`ScenarioPackage` references. A RobotPackage owns the mechanism, frames,
interfaces, and default references; it does not own a concrete policy or a
sensor category. A SensorRig owns sensor instances and mounts them on generic
robot or world frames.

This keeps adding Go2, H1, or a new camera rig a catalog operation rather than
a Runtime or UE main-loop change. The package manifests are the cross-system
source; Unreal `UPrimaryDataAsset` objects are generated projections.
