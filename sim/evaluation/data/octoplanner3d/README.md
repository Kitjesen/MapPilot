# OctoPlanner3D Simulation Fixtures

These PCD files are simulation and offline conversion fixtures. They are not
runtime planner assets and are excluded from field releases.

The files originated with the MIT-licensed OctoPlanner3D project used by
LingTu's product integration. They were moved out of the planner source tree so
algorithm code, third-party provenance, and large replay data have separate
ownership boundaries.

Product navigation consumes validated Maps artifacts (`octomap.ot` or, for the
optional FAR backend, `occupancy.npz`). It never resolves these fixture paths.
