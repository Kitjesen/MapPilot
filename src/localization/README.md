# Localization

`src/localization` is the localization domain. It owns pose sources and localization health, not navigation decisions.

The current Product dataflow, control commands, complete DDS topic inventory,
relocalization/loop-closure status, and SaveMap boundary are documented in
[`docs/architecture/LOCALIZATION_RUNTIME.md`](../../docs/architecture/LOCALIZATION_RUNTIME.md).

## Responsibilities

- Native `slamd` contract: pose, registered cloud, map observation/cloud,
  saved map, localization health, optional GNSS capability diagnostics,
  map->odom, and scene mode.
- Fast-LIO2 LiDAR-inertial odometry and saved-map relocalization adapters.
- Relocalization adapters and saved-map alignment helpers.
- Native GNSS/localization fusion contracts.
- Runtime outputs used by navigation: odometry, map cloud, map->odom transform, localization quality, localization status.

## Boundaries

- Navigation decisions stay in `src/nav`.
- Sensor device ownership stays in `src/drivers`.
- Localization has no ROS2 message/service package. Product process boundaries
  use native typed DDS.
- SLAM does not publish navigation paths or velocity commands. Historical pose
  tracks may be saved as `poses.txt`, but `global_path`, `local_path`,
  `waypoint`, and `cmd_vel` belong to `src/nav`.
- No-ROS localization implementations attach behind `localization/slam/cpp`
  through the `ISlamBackend` contract.

## Main Files

| Path | Category | Role |
| --- | --- | --- |
| `service.py` | public API | Backend-neutral relocalization result, protocol, and stable `Localization` capability. |
| `adapters/relocalization.py` | native adapter | Calls the C++ CycloneDDS SLAM control tool; no ROS dependency. |
| `adapters/status.py` | native adapter | Converts the native SLAM endpoint status stream into localization-domain messages. |
| `adapters/resolver.py` | composition adapter | Resolves the explicit localization adapter selected by Product/env resolution; it never guesses a transport. |
| `slam/cpp/` | realtime | ROS-free C++ `ISlamBackend`, DDS runtime, SLAM control, native relocalization hooks. |
| `fastlio2/` | algorithm asset | Fast-LIO2 supplies the native Product backend. |
| `localizer/` | relocalization | Single ROS-free ICP/BBS3D library and offline `localizer_cli`; `slamd` owns the live DDS runtime. |

Product Hosts identify the external runtime as `native_dds`. The algorithm
backend is `fastlio2`; `mapping` versus `localization` comes only from the
Product's `slam_mode`. Python does not own an in-process SLAM runner or native
binding. Removed legacy backend names are rejected instead of falling back to a
contract-only backend. GNSS device access belongs to the native service under
`src/drivers/real/gnss`; localization consumes its typed DDS output only when a
Product explicitly connects GNSS fusion.

## Semantic Occupancy Localization

SOCC-ICP is a research input, not a vendored runtime. LingTu owns semantic
occupancy and free-space evidence in `src/maps`; localization owns scan-to-map
registration, pose estimates, and health.

The current ROS-free path is:

```text
semantic_map.bin
  -> SemanticMapClient (direct C++ persistence reader)
  -> MapIcp
  -> NativeRelocalizer
  -> Fast-LIO map->odom commit gate
```

`NativeRelocalizer` prefers a sibling `semantic_map.bin` for seeded ICP. If the
artifact exists but is invalid, loading fails closed;
it does not silently fall back to PCD. If no semantic artifact exists, the
saved `map.pcd` remains the explicit geometry source. BBS3D supplies global
coarse search when built, then the same MapIcp refines the result.

The removed Python `SemanticMapModule` is not part of Product assembly. A sibling
artifact therefore comes only from an explicit offline workflow, not from the
field Product's online map stack.

Initial relocalization may establish a large map alignment. Periodic drift
correction is stricter: it requires inlier/covariance diagnostics and rejects
fitness failures, map-bound violations, covariance degeneracy, and corrections
larger than the selected runtime profile permits. The generic backend defaults
are 1 m and 15 degrees; the field S100P profile currently tightens these to
0.15 m translation, 2 degrees yaw, and 5 degrees map-to-odom tilt. Rejected ICP
never changes the active map or `map->odom`.

The current MapIcp uses semantic-map voxel geometry as its target. Semantic
class/confidence weighting and mixed point-to-plane residuals remain a measured
algorithm upgrade, not a claimed property of this baseline.

The adoption decision and product gates are documented in
`docs/research/socc_icp_adoption.md`.

## Loop and PGO Status

The online Fast-LIO2 frontend does not change poses or maps through loop closure.
During capture it freezes `map.pcd`, `poses.txt`, body-local patches, and their
manifest. SaveMap can then run the ROS-free `lt_pgo --auto-constraints` helper on
its private working copy. The helper measures a complete adjacent chain from the
patches, merges verified loop factors, and optimizes only when the chain is
complete and at least one loop is trusted. A non-ready graph leaves the raw
source unchanged and records its precise skip code in `map_optimization.json`.

`pose_graph.constraints` is an optimizer-private temporary input, not a
Fast-LIO2 output or Product artifact, and is not published in the saved map.
The former ROS2 PGO/HBA runtime remains physically removed. This connected
save-time path has local contract coverage, but field loop quality, trajectory
quality, and S100P performance evidence are still outstanding.
