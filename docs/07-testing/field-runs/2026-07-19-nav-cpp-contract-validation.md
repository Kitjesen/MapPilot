# Navigation C++ Contract Validation - 2026-07-19

Status: **PASS for isolated aarch64 Release build and native CTest. Not field
readiness.**

## Run identity

| Field | Value |
| --- | --- |
| Host | `worm@192.168.66.9` |
| OS | Ubuntu 22.04, Linux aarch64 |
| Test root | `/tmp/lingtu-nav-debug-current` (removed after validation) |
| Product profile | None |
| Product services | Not installed or started |
| Motion output | None |

The run used a reduced source archive and the real local-planner path library.
It configured the endpoint CMake tree from an empty temporary directory, built
the default target set, and ran every test registered by that tree.

## Results

| Gate | Result | Evidence |
| --- | --- | --- |
| Fresh aarch64 Release configure | PASS | CycloneDDS, system OctoMap, PCL, Eigen found |
| Default endpoint build | PASS | Build reached 100 percent from an empty tree |
| Native endpoint CTest | PASS | 32/32 tests |
| Async planning freshness | PASS | Goal, frame, and map revision rejection tests |
| Active map artifact gate | PASS | Hash, frame, active-map, snapshot, and cache tests |
| OctoPlanner behavior | PASS | Same-floor, cancellation, no-air-climb, and map cache tests |
| DDS target ownership | PASS | IDL generated once and linked through `lingtu_dds_messages` |
| Product binaries | PASS | `navd`, `liblingtu_nav_client.so`, and headless planner are ARM aarch64 ELF |

## Contract changes validated

- `GlobalPlanRequest` no longer exposes a map path or backend map type.
- Every endpoint planning result carries `map_id`, version, artifact SHA-256,
  and frame ID.
- `navd` rejects a completed plan when the goal epoch, frame epoch, or active
  map identity changed while planning.
- The active map gate reuses one read-only snapshot for an unchanged map
  revision.
- OctoPlanner reuses the parsed in-memory tree for that revision and reloads it
  after the identity changes.
- Endpoint tests that consume generated DDS types link the message target
  directly instead of depending on another target's build order.

## Defects found during validation

1. The reduced validation archive initially omitted `src/maps/cpp/build`, which
   is source code despite its directory name. The archive rule was corrected.
2. `test_transform_buffer` and `test_traversability_geometry` depended on
   `lingtu_nav_client` only to make a generated header appear. They now link the
   DDS message target explicitly.
3. `test_nav_client` had the same hidden generated-header dependency and now
   links the message target directly.
4. The first full test archive omitted `src/nav/cpp/planning/local/cmu/paths`; after adding the
   real path library, both affected tests passed and the final suite was 32/32.

## Not validated

This run does not prove:

- deployment through the release launcher or systemd;
- live DDS publishers, Gateway readiness, or process restart behavior;
- real MID-360 timing and LiDAR/odometry synchronization;
- planning against a field map with live localization;
- physical obstacle avoidance or any robot motion;
- installation on an image where ROS packages are absent.

The remote temporary source, build tree, and uploaded archives were removed.
