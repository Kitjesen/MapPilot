# Local Planner I/O Contract

Status: current for Python Module-owned local-planner I/O; physical `env=real` embeds local planning inside `lingtu-nav-dds`
Audience: navigation Module and compatibility-profile maintainers
Replaced by: not replaced

Canonical implementation document:

- `src/nav/local/README.md`

Code references:

- Module ports: `src/nav/local/local_planner.py`
- Port contract: `src/nav/services/plan/contracts.py`
- Navigation wires: `src/lingtu/assembly/wires/navigation.py`
- SLAM/odometry wires: `src/lingtu/assembly/wires/slam.py`
- Map/traversability wires: `src/lingtu/assembly/wires/mapping.py`

Current status:

| Area | Status |
| --- | --- |
| Mission inputs | Wired: `waypoint`, `global_path`, `clear_path`. |
| Localization inputs | Wired: `odometry`; `map_frame_jump_event` when SLAM/localization is active. |
| Terrain inputs | Wired: `terrain_map`, `terrain_map_ext`, `traversability`; traversability now feeds native C++ grid scoring/near-field stop and the Python fallback scorer. |
| ESDF input | Wired through `TraversabilityCostModule.esdf_field`, but still reserved by the local planner. |
| Optional obstacle overlays | Ports exist, but `boundary`, `added_obstacles`, and `check_obstacle` have no default producer. |
| Outputs | Wired: `local_path` to path follower/safety, `control_hint` to path follower. |

Main remaining gap:

`esdf`, `boundary`, `added_obstacles`, and `check_obstacle` need cleanup:
`esdf` is stored but not scored, and the three optional obstacle overlay ports
do not have default producers.

Traversability status:

- Python fallback: `cmu_py` samples the risk grid on candidate path groups.
- C++ backend: `LocalPlannerCore` exposes `setTraversabilityGrid()` and applies
  the same hard/soft risk thresholds in native scoring and near-field stop.
- Shared obstacle cloud: high-risk virtual traversability obstacles are kept
  only for the `cmu_py` fallback and direct-track clearance checks. The
  preferred `nanobind` backend uses the native C++ grid port instead.
