# Native Product Processes

Status: current Product process boundary contract
Audience: native deployment, Gateway operations, and field-readiness maintainers
Replaced by: not replaced

This document separates three things that were being mixed together:

- C++ real-time product components;
- Gateway/Web API routes that only trigger or observe runtime actions.

`src/gateway/routes/operations.py` belongs to the third category. It exposes
HTTP routes such as SLAM status, restart, runtime switch, relocalization,
visual-servo operations, and WebRTC/memory/diagnostic operations. It must not
own LiDAR ingestion, SLAM, PGO, HBA, or navigation compute.

## Canonical Folders

| Folder | Owns |
| --- | --- |
| `src/drivers/real/lidar/sdk2_stream/` | Livox SDK2 C++ ingestion and optional CycloneDDS publication. |
| `src/localization/slam/cpp/` | Native SLAM contract, Fast-LIO2 backend, DDS runtime, SLAM control, native relocalization support. |
| `src/nav/cpp/endpoint/` | Native navigation endpoint, traversability DDS, nav control, and robot command publication gate. |

## Current Implementation Status

| Component | Current status | Evidence |
| --- | --- | --- |
| LiDAR DDS | Product-native C++ implemented. | `livox_sdk2_stream` in `src/drivers/real/lidar/sdk2_stream/`; deployed as `lt-lidar.service`. |
| SLAM DDS | Product-native C++ implemented. | `slamd` and `slamctl` in `src/localization/slam/cpp/`; deployed as `lt-slam.service`. CycloneDDS is a transport detail, not part of the process name. |
| Relocalization | Product-native support exists in the SLAM C++ contract, with optional BBS3D/small_gicp build support. | `native_relocalizer.cpp` in `src/localization/slam/cpp/`. |
| Navigation DDS | Product-native C++ implemented. OctoPlanner3D is default; FAR is an explicit validated-occupancy option. | `navd`, `lingtu_nav_client`, `lingtu_traversability_dds` in `src/nav/cpp/endpoint/`; deployed as `lt-nav.service` and `lt-terrain.service`. |
| Field driver | Product-native C++ implemented. | `lingtu_driver` in `src/drivers/real/motion/`; deployed as `lt-driver.service`, consumes `rt/nav/cmd_vel`, and delegates to the RobotConfig-selected `Go2` or `Doso` implementation. |

## States

| State | Meaning |
| --- | --- |
| `ready` | Can be used in the current product path. |
| `wip` | Code exists, but there is no supported product command/UI path yet. |
| `legacy` | Old compatibility path only. |
| `exp` | Experimental, not product. |

## PGO And Legacy HBA Boundary

The former ROS2 PGO and HBA packages remain removed. They are not Product
services, runtime processes, or compatibility backends.

The ROS-free save-time path is different: `lt_pgo` and the portable Rust
`pose_graph_opt` kernel are built as native code, and `lt_pgo` is included in
the native release. Fast-LIO2 capture freezes `map.pcd`, `poses.txt`, body-local
patches, and their manifest; it does not publish a factor file. During
`OPTIMIZE_SOURCE`, SaveMap runs `lt_pgo --auto-constraints` on a complete patch
bundle. The helper measures the full adjacent chain with bidirectional trimmed
4DoF point-to-plane registration, derives body-right information from measured
`J^T J / sigma^2`, and merges verified loop factors.

Optimization requires all `N-1` adjacent factors and at least one trusted loop.
Otherwise SaveMap keeps the raw source and writes the precise skip code to
`map_optimization.json`. The temporary `pose_graph.constraints` file is private
to `lt_pgo`, is strictly re-read and removed, and is not published. `lt_pgo`
remains a short-lived save helper, not a resident service or online SLAM loop.
Field loop quality and S100P performance have not yet been accepted.

## How To Use

Start a complete Product through ProductControl:

```bash
bash scripts/lingtu --robot unitree/go2 --env real switch nav --map <map>
```

`scripts/lingtu` is a thin adapter over `python -m lingtu.control switch`.
ProductControl resolves one Product inside `env=real`, publishes the
resolved RunPlan, then applies its processes through the internal systemd runner in
dependency order. Direct `systemctl` calls are reserved for service-level
diagnosis; they are not a second product startup contract.

The `env=real` RunPlan uses the `field_dds_v1` typed DDS
contract. `config/runtime_graph/envs/real.yaml` sets
`localization_adapter=cpp_slam_status`,
`command_output_mode=endpoint_only`, and
`hardware_control_boundary=driver`. The deployed `lt-host.service` also sets
`LINGTU_ENABLE_ROBOT_DRIVER=0`, so the Python process does not open a second
robot hardware writer.

The former Python DDS field unit, endpoint runner, installer, and deployment
wrapper are removed. Diagnostics use the native DDS probe; recording and replay
use MCAP tools.
