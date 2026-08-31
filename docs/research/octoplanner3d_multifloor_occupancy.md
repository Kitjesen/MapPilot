# OctoPlanner3D Multifloor Occupancy Notes

Status: research note; current runtime contracts live in `../architecture/`

## Current conclusion

OctoPlanner3D can produce XYZ routes through supported 3-D occupancy, but that
alone does not prove robot-executable multifloor navigation. The full Product
must also preserve floor identity, route through a valid connector, track the
route locally, and confirm localization on the destination floor.

The product model should remain:

```text
per-floor metric map + support/traversability layer
  <-> typed connector (stairs, ramp, or elevator)
  <-> building topology
```

A single XY-flattened building map cannot distinguish floors that overlap in
plan view.

## Planner semantics

OctoPlanner3D is not a free-space flight planner. It expands 26-neighbor grid
moves, but candidate cells must satisfy ground-support and motion constraints.
Relevant native sources are:

- `src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp`;
- `src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.cpp`;
- `src/nav/cpp/endpoint/nav/input/active/octomap.cpp`;
- `src/nav/cpp/endpoint/nav/runtime/goal/`.

Current Product configuration exposes support, step, slope, floor-preference,
and clearance settings through `src/lingtu/assembly/native_nav.py` and the
native endpoint config.

## Occupancy is not traversability

`octomap.ot` is the saved static 3-D occupancy artifact. It does not classify
floor slabs, stair treads, risers, rails, walls, landings, or unknown support.
Increasing point-cloud density can improve geometry while also increasing the
global search cost.

For legged multifloor navigation, retain the occupancy tree for collision and
derive support/traversability evidence for route cost and admission. Live
registered cloud and traversability belong in the local planning/safety path;
moving obstacles should not be baked into the saved OctoMap.

## Required route constraints

- Start, goal, and every expanded cell must use the same map frame.
- A route must remain on supported geometry.
- Step height and slope must stay within the selected Product limits.
- Floor changes must pass through a declared stair, ramp, or elevator connector.
- Goal arrival must include destination-floor identity, not XY distance alone.

## Current runtime chain

```text
saved map package
  -> native OctoPlanner3D global route
  -> native local planner
  -> embedded path tracking
  -> final native safety and control authority
  -> /nav/cmd_vel
  -> driver
```

Planner-to-local-planner and local-planner-to-tracker handoff are direct C++
calls inside `navd`. DDS global/local path topics are telemetry.

## Evidence boundary

A direct OctoPlanner3D route proves only that the chosen map and options admit a
route. Multifloor Product evidence additionally requires:

- active-map and localization agreement;
- connector selection and transition completion;
- local obstacle/traversability input throughout execution;
- destination-floor localization after transition;
- native goal lifecycle, final command, stop, and driver evidence.

Render route evidence against the same PCD or OctoMap used by the planner, color
by height or occupancy, and record the planner options. A figure assembled from
another map product is not evidence for the route under review.
