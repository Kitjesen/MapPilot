# RobotSimUE content boundary

Generated preview assets may be materialized below `/Game/RobotSim` by the checked-in
offline preview pipeline. They are UE-side projections of a compiled visual recipe, not
a second robot configuration source. Runtime code must consume compiled SessionBundle
artifacts rather than source model files, MJCF, or `RobotConfig`.
