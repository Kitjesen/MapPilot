# Localization

`src/localization` is the localization domain. It owns pose sources and localization health, not navigation decisions.

## Responsibilities

- Native `SlamModule` contract: pose, registered cloud, map cloud, saved map,
  localization health, GNSS fusion health, map->odom, and scene mode.
- LiDAR-inertial odometry assets and adapters: Fast-LIO2, Point-LIO, and
  saved-map relocalization compatibility bridges.
- Relocalization adapters and saved-map alignment helpers.
- GNSS, NTRIP, and GNSS/localization fusion diagnostics.
- Runtime outputs used by navigation: odometry, map cloud, map->odom transform, localization quality, localization status.

## Boundaries

- Navigation decisions stay in `src/nav`.
- Sensor device ownership stays in `src/drivers`.
- ROS2-facing interface files are compatibility-only and stay out of the
  product native runtime.
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
| `slam/module.py` | module boundary | Python Module wrapper for downstream runtime consumers. |
| `fastlio2/`, `pointlio/` | algorithm assets | LIO algorithm packages used by native SLAM backends or compatibility builds. |
| `opt/` | map optimization product entry | Short C++ entry surface for save-time map optimization: `map.*`, `pgo.*`, `hba.*`. |
| `pgo/`, `hba/` | optimization algorithms / legacy wrappers | Existing PGO/HBA algorithm code and ROS2 node wrappers. Product use should go through `opt/`. |
| `localizer/`, `native_localizer/` | relocalization | Saved-map relocalization and localizer command surfaces. |
| `gnss_module.py`, `gnss_serial_driver.py`, `ntrip_client_module.py` | GNSS/RTK | GNSS input, corrections, and diagnostics. |
| `interface/` | compatibility | Legacy ROS 2 message/service contracts for compatibility builds. Not the product default. |

Compatibility configuration names may still use `slam_profile` and `slam.service` for deployed robots. New Python imports should use `localization.*`.

## Semantic Occupancy Localization

SOCC-ICP is a research input, not a vendored runtime. LingTu owns semantic
occupancy and free-space evidence in `src/maps`; localization owns scan-to-map
registration, pose estimates, and health.

The current ROS-free path is:

```text
semantic_map.bin
  -> lingtu_maps semantic C ABI
  -> SemanticMapClient
  -> MapIcp
  -> NativeRelocalizer
  -> Fast-LIO map->odom commit gate
```

`NativeRelocalizer` prefers a sibling `semantic_map.bin` for seeded ICP. If the
artifact exists but is invalid or the ABI is unavailable, loading fails closed;
it does not silently fall back to PCD. If no semantic artifact exists, the
saved `map.pcd` remains the explicit geometry source. BBS3D supplies global
coarse search when built, then the same MapIcp refines the result.

Initial relocalization may establish a large map alignment. Periodic drift
correction is stricter: it requires inlier/covariance diagnostics and rejects
fitness failures, map-bound violations, covariance degeneracy, corrections
larger than 1 m, and yaw jumps larger than 15 degrees. Rejected ICP never
changes the active map or `map->odom`.

The current MapIcp uses semantic-map voxel geometry as its target. Semantic
class/confidence weighting and mixed point-to-plane residuals remain a measured
algorithm upgrade, not a claimed property of this baseline.

The adoption decision and product gates are documented in
`docs/research/socc_icp_adoption.md`.

## Save-Map Optimization Policy

For product behavior, map optimization belongs to the save-map pipeline:

```text
live SLAM map + poses + patches
  -> opt/map artifact check
  -> opt/pgo by default
  -> opt/hba only for high-quality/offline refinement
  -> map.pcd + poses.txt + planner artifacts
```

`maps.services.pipeline.MapPipelineService.save()` calls this optimization
step before rebuilding occupancy and OctoMap artifacts. The default strategy is
`pgo`; callers may request `hba` or `none` through the save-map API. The current
native runners fail closed until the non-ROS PGO/HBA algorithms are connected.

Fast-LIO remains the realtime odometry/mapping front end. PGO can later become
a low-rate online loop-closure backend, but the first supported product path is
save-time optimization. HBA is heavier and should not block the realtime SLAM
loop.
