# Relocalization Research Repositories

Status: research repository index; not a runtime dependency contract

Local clone path: `third_party/research_localization/`.

`third_party/` is intentionally ignored by Git. These repositories are local
reference material only. Do not import them directly from `src/`; move only
reviewed, license-compatible code into LingTu-owned modules.

## Current LingTu Baseline

LingTu native relocalization currently uses:

```text
saved map PCD + current Livox scan
  -> optional BBS3D coarse global pose
  -> small_gicp GICP refine when available, otherwise PCL GICP
  -> Fast-LIO2 tracking reset
```

Code path:

| LingTu file | Current role |
| --- | --- |
| `src/localization/slam/cpp/native_relocalizer.cpp` | `NativeRelocalizer`: BBS3D coarse localization plus ICP refine. |
| `src/localization/localizer/src/localizers/bbs3d_global_localizer.cpp` | Thin wrapper around optional `cpu_bbs3d`. |
| `src/localization/localizer/src/localizers/icp_localizer.cpp` | GICP refine implementation: small_gicp when available, otherwise PCL GICP. |
| `src/localization/slam/cpp/fastlio.cpp` | Calls relocalizer and resets Fast-LIO2 tracking state. |
| `src/localization/slam/cpp/slam_control.cpp` | C++ DDS control CLI for `load_map`, `relocalize`, `global_relocalize`, and `query_status`. Relocalization commands use typed DDS request/reply. |
| `src/localization/adapters/relocalization.py` | Thin Python service adapter that shells out to the C++ control CLI; it no longer owns the DDS request schema. |

Important gap: this is still mostly one-shot relocalization. It is not yet a
complete continuous scan-to-saved-map localization stack with prediction,
outlier-aware status, and recovery policy.

## Cloned Repositories

| Directory | Upstream | Commit | License | Main use for LingTu |
| --- | --- | --- | --- | --- |
| `3d_bbs/` | `https://github.com/KOKIAOKI/3d_bbs.git` | `41529a3` | MIT | Keep as the global coarse relocalization reference and build source for `cpu_bbs3d`. |
| `small_gicp/` | `https://github.com/koide3/small_gicp.git` | `57c1106` | MIT | Best candidate to replace LingTu's current PCL ICP refine path. |
| `scancontext_tro/` | `https://github.com/gisbi-kim/scancontext_tro.git` | `b463bb7` | CC BY-NC-SA 4.0 | Algorithm reference only; do not copy into commercial/product code. |
| `TEASER-plusplus/` | `https://github.com/MIT-SPARK/TEASER-plusplus.git` | `52a9c52` | MIT | Optional robust global registration backend after feature matching. |
| `hdl_localization/` | `https://github.com/koide3/hdl_localization.git` | `de2dc77` | BSD-2-Clause | Architecture reference for map-based continuous localization and status metrics. |
| `FAST_LIO_LOCALIZATION/` | `https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION.git` | `2bc274e` | GPL-2.0 | Concept reference only; do not migrate code into LingTu core. |

`hdl_global_localization` is referenced by `hdl_localization` and GitHub search,
but direct `git clone https://github.com/koide3/hdl_global_localization` returned
`Repository not found` from this machine. Treat it as an external design
reference, not a local source asset.

## Repository Findings

| Project | Useful function | Code worth reading | Can migrate code? | Notes |
| --- | --- | --- | --- | --- |
| `3d_bbs` | No-initial-guess global pose search from one gravity-aligned scan and a PCD map. | `bbs3d/include/cpu_bbs3d/bbs3d.hpp`, `bbs3d/src/cpu_bbs3d/bbs3d.cpp`, `test/`, `ros2_test/`. | Yes, license allows it, but LingTu already wraps this through `BBS3DGlobalLocalizer`. Prefer using installed library instead of copying source. | Good for `global_relocalize`. CPU version is enough for S100P unless latency proves too high. |
| `small_gicp` | Fast ICP/GICP/VGICP refine, parallel downsampling, covariance estimation, KD-tree. | `include/small_gicp/registration/registration_helper.hpp`, `include/small_gicp/registration/registration.hpp`, `include/small_gicp/pcl/pcl_registration.hpp`. | Yes. This is the cleanest migration candidate. | Replace `ICPLocalizer` internals first, keep the `ICPLocalizer` public wrapper stable. |
| `scancontext_tro` | Place recognition / candidate retrieval for long-term relocalization. | `cpp/module/Scancontext/Scancontext.h`, `cpp/module/Scancontext/Scancontext.cpp`. | No for product code because license is non-commercial share-alike. | Reimplement the idea if needed: scan descriptor index -> candidate submap -> GICP refine. |
| `TEASER-plusplus` | Certifiably robust registration with many outliers. | `teaser/include/teaser/registration.h`, `teaser/src/registration.cc`, `teaser/src/fpfh.cc`. | Yes, MIT. | Useful only if LingTu adds FPFH/correspondence generation. Not the first migration. |
| `hdl_localization` | Continuous map-based localization design: prediction + NDT/GICP correction + status. | `src/hdl_localization/pose_estimator.cpp`, `apps/`, `launch/`. | Small design pieces only; it is ROS/catkin-heavy. | Strong reference for the service contract: `/odom`, aligned cloud, scan matching status, relocalize service. |
| `FAST_LIO_LOCALIZATION` | Product idea: low-rate global localization fused with high-rate FAST-LIO odometry to suppress drift. | `README.md`, launch/config flow, `src/laserMapping.cpp`. | No, GPL-2.0 and Python2/ROS-era code. | Keep as architecture evidence, not implementation source. |

## Recommended LingTu Migration

### Step 1: Fix the current relocalization contract

Before adding a new algorithm, fix the coordinate contract:

```text
relocalize result must expose:
  map_body
  odom_body
  map_odom = map_body * inverse(odom_body)
  quality
  inlier/fitness diagnostics
```

The current `FastLioBackend::relocalize()` computes through `NativeRelocalizer`
but resets Fast-LIO2 in a way that makes the published `map_odom_tf` contract
ambiguous. Navigation should not accept saved-map localization as ready until
`map_odom_tf` and odometry agree with the active map bounds.

### Step 2: Replace point-to-point ICP refine with GICP

Target file:

```text
src/localization/localizer/src/localizers/icp_localizer.cpp
```

Current status: implemented behind the existing `ICPLocalizer` API.

```text
if small_gicp + OpenMP are available:
  -> small_gicp GICP refine
else:
  -> PCL GICP refine

published diagnostics:
  -> backend, fitness, converged, iterations, inliers, covariance trace
```

Reason: this is the smallest useful production upgrade. It does not change
Gateway, DDS, Fast-LIO2, or navigation contracts.

### Step 3: Make relocalization a C++ service, not Python subprocess glue

Current path:

```text
Gateway HTTP
  -> Python NativeSlamRelocalizationService
  -> subprocess lingtu_slam_control
  -> typed DDS /slam/relocalization/request
  -> C++ SLAM runtime
  -> typed DDS /slam/relocalization/response
```

Target path:

```text
Gateway HTTP or CLI
  -> typed C++ relocalization service endpoint directly
  -> DDS request/reply or direct local service call
  -> SLAM backend
```

Minimum service calls:

| Call | Input | Output |
| --- | --- | --- |
| `load_map` | map directory or `map.pcd` | loaded, map hash, map bounds |
| `seeded_relocalize` | map, initial `map_body` guess, current scan | `map_body`, `map_odom`, quality |
| `global_relocalize` | map, current scan | coarse pose, refined pose, quality |
| `track_against_map` | stream scan + IMU + previous state | continuous corrected odometry |
| `query_status` | none | backend, mode, map_loaded, quality, reason |

Current implementation status:

| Call | Status |
| --- | --- |
| `load_map` | Typed DDS request/reply path exists. |
| `seeded_relocalize` | Typed DDS request/reply path exists; refine diagnostics are returned. |
| `global_relocalize` | Typed DDS request/reply path exists; BBS3D availability still depends on build flags. |
| `query_status` | Typed DDS request/reply path exists. |
| `track_against_map` | Minimal continuous correction loop exists in the C++ runtime: 1 Hz map correction using the existing relocalize backend, `relocalization_max_fitness` rejection, stale-scan rejection, loaded-map bounds rejection, and 3-failure auto-disable. This is not yet a full hdl_localization-style estimator. |

### Step 4: Add candidate retrieval only after Step 1 and Step 2 pass

Do not copy `scancontext_tro` because of license. If large-map global search is
too slow, implement a LingTu-owned descriptor index:

```text
map patches / keyframes
  -> descriptor index
  -> candidate submaps
  -> BBS3D or TEASER++ coarse pose
  -> small_gicp refine
```

### Step 5: Keep TEASER++ optional

TEASER++ is useful when FPFH/correspondence matching is already reliable. It is
not a direct replacement for the current BBS3D + ICP path. Add it only as a
secondary `global_relocalize` engine after `small_gicp` is stable.

## Product Impact

| LingTu capability | Current state | Research repo contribution |
| --- | --- | --- |
| Manual initial pose relocalization | Exists through seeded ICP. | `small_gicp` improves speed/robustness. |
| No-initial-pose relocalization | Exists only when BBS3D is built. | `3d_bbs` keeps this path; `scancontext` idea can reduce search scope later. |
| Continuous saved-map localization | Minimal C++ loop exists with fitness, stale-scan, and map-bound gates. | `hdl_localization` and `FAST_LIO_LOCALIZATION` define the stronger pattern. |
| Drift suppression during navigation | Basic low-rate map correction exists. | Harden with acceptance windows before treating it as production navigation readiness. |
| Commercial code safety | Mixed. | MIT/BSD code can be migrated; GPL/CC-NC code should remain reference-only. |

## Immediate Next Patch

The next code patch should stay focused:

1. Remove the remaining Python -> subprocess control hop by giving Gateway or the field ops process a direct native service client.
2. Fix the navigation readiness gate so relocalized odometry must be inside active map bounds before navigation starts.
3. Add a LingTu-owned ScanContext-style candidate index only if full-map BBS3D is too slow.

Do not start by copying full repositories into `src/`.
