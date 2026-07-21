# Navigation C++ Build

This directory is the canonical CMake entry point and source root for portable
navigation compute and the Linux native endpoint.

Current targets:

| Target | Responsibility |
| --- | --- |
| `lingtu_nav_path_follower` | Path-following control algorithm |
| `lingtu_nav_plan_loop` | Local planning and path-following loop |
| `lingtu_nav_far` | Optional native 2D visibility-graph global planner |
| `lingtu_nav_far_c_api` / `libnav_far` | Versioned FAR C ABI |
| `octoplanner3d_runtime` | Default 3D global planner runtime |
| `navd` | Linux product endpoint, enabled with `LINGTU_NAV_CPP_BUILD_ENDPOINT` |

Rules:

- Product endpoints link targets and do not compile sibling implementation
  files directly.
- DDS generation belongs to `src/message/cpp`; navigation only links the
  generated target.
- ROS, CycloneDDS, filesystem, and environment access stay outside the future
  navigation engine target.
- `nav_kernel` is retained only as the public namespace/extension ABI for the
  portable local-planning types. New product source belongs under this tree,
  not under a second `src/nav/kernel` implementation directory.

## Planning job consistency

`GlobalPlanRequest` contains geometry, planner options, map generation, and an
expected map identity. It does not contain an OctoMap type or artifact path.
The endpoint's backend-specific map gate validates the active Maps artifact and
attaches this identity to every result:

```text
map_id + map version + artifact SHA-256 + frame_id
```

The active artifact is copied to a read-only private snapshot. OctoPlanner3D
uses `octomap.ot`; optional FAR uses trinary `occupancy.npz`. Parsed state is
reused only while identity and generation remain valid.
Before accepting an asynchronous result, `navd` compares its goal epoch, frame
epoch, and map identity with current state. A mismatch stops motion and discards
the path; stale paths never enter `NavLoop`.

Portable build:

```bash
cmake -S src/nav/cpp -B build/nav-cpp -DLINGTU_NAV_CPP_BUILD_TESTS=OFF
cmake --build build/nav-cpp --target lingtu_nav_plan_loop
```

Linux endpoint build:

```bash
cmake -S src/nav/cpp -B build/nav-cpp \
  -DLINGTU_NAV_CPP_BUILD_ENDPOINT=ON
cmake --build build/nav-cpp --target navd
```
