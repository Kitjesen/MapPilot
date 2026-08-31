# Environment Map

Status: current map presentation and frame contract
Audience: maps, navigation, Gateway, Web, simulation, and field-readiness contributors
Replaced by: not replaced

## Product concept

An operator sees one **Environment Map**, not a collection of native
processes.  The Web view keeps these meanings separate:

| Layer | What it answers | Current source |
| --- | --- | --- |
| Scene | What geometry has been observed? | `MapScene` point-cloud, occupancy, and ESDF layers from `mapd` |
| Minimum observed elevation | What is the lowest measured height at each cell? | `MapScene` elevation layer from `mapd` |
| Planning cost | What does native navigation currently score? | native navigation status and path telemetry |
| Walking risk | Where must the robot slow, stop, or avoid? | Native traversability runtime |

The minimum observed elevation is a measured surface, not a claimed ground
model. Planning diagnostics and native walking risk remain distinct views.

`mapd` and the traversability runtime may remain separate workers while this
contract is migrated.  That is an implementation choice, not two products.
They become one product surface only when their outputs can be proven to refer
to the same current map coordinate era.

## Planning frames: one environment map, three useful views

The robot does not need three competing maps. It needs one global environment
map and two bounded control views with different lifetimes:

| Frame | Data | Used for | Retained / shown as a map? |
| --- | --- | --- | --- |
| `map` | `MapScene`, elevation/ESDF, and `/nav/traversability` | global route, saved-map context, final swept-path safety | Yes. This is the operator-visible environment frame. |
| `odom` | `/nav/local_traversability` | native local candidate scoring around the robot | No. It is a volatile rolling control window. |
| `body` / `base_link` | current obstacle and footprint checks | immediate slow/stop decision before the single final `/nav/cmd_vel` writer | No. It is a current near-field view, not an accumulated map. |

`/nav/local_traversability` is deliberately a separate, `odom`-framed DDS
topic. The traversability runtime derives it from the current map-risk window
only when a valid `map <- odom` transform and odom pose exist. It is
RELIABLE, VOLATILE, keep-last-one, and expires after 500 ms. Navd uses it only
for local planning; global goals and published global/local paths remain in
`map`. Navd still requires the fresh map-risk grid for the input gate and for
its independent final safety check.

This gives local planning a stable rolling frame without creating another map
authority or another velocity writer. If the local window, odom pose, or
`map <- odom` transform is unavailable or stale, navd falls back to the
existing map-frame planner input; it never treats an old local grid as valid.

## Operator feedback

Gateway exposes the current feedback at:

```text
GET /api/v1/maps/environment/layers
```

The endpoint returns only compact, operator-facing state.  It intentionally
hides transport details such as producer boot IDs, reset epochs, and scan
sequences.

| Layer state | Operator meaning | Expected action |
| --- | --- | --- |
| `ready` | Current data is coherent and fresh. | Use the layer normally. |
| `updating` | A new map is arriving or its identity is not complete yet. | Wait for the current map. |
| `stale` | The most recent data is too old. | Do not treat it as current. |
| `unavailable` | The source has not produced usable data. | Check the product/readiness state. |
| `invalid` | Data exists but lacks the identity needed for a safe display. | Do not retain or overlay it. |
| `diagnostic_only` | The native risk engine is alive, but only sparse diagnostics exist. | Do not draw it as a complete risk map. |

The compact REST endpoint may still report `diagnostic_only` when the status
file has no complete grid.  That status endpoint is not the Web raster source:
the actual native grid is delivered separately through the bounded
`native_traversability` SSE event described below.

## What `reset_epoch` means

Internally, `reset_epoch` is an opaque **map-coordinate era** marker.  SLAM
advances it after a map reset, relocalization reset, or equivalent coordinate
restart.  `observation_sequence` then orders scans within that era.

The user interface does not show these numbers.  It shows the consequence:

```text
地图坐标已更新，风险层正在同步。
```

This prevents an old risk layer from being retained over a newly reset map.
`producer_boot_id` is likewise a restart diagnostic, not an operator field.

## Current truth

`MapScene` provides the scene/elevation/ESDF base with map identity metadata.
The native `/nav/traversability` wire is a typed `OccupancyGrid` carrying
header, geometry, and `uint8` risk cells.  The HostBus client adds a bounded
`reset_epoch` and receive sequence before Gateway/Web projection; it does not
copy the grid through a Python DDS subscription.

Therefore the current product may display:

- a real 3-D minimum-observed-elevation surface (payload is opt-in over SSE);
- global and local paths over the scene;
- a native control-risk heatmap when its frame, sequence, and freshness checks
  pass;
- the separate Python planning-cost preview, never labelled as control risk.

It may **not** display that sparse diagnostic as a complete risk heatmap or a
Nav2-style local costmap.

## Native risk projection

```text
native traversability runtime
  -> /nav/traversability (typed DDS, sole field writer)
  -> native nav client C ABI (latest-only)
  -> HostBus (bounded Python adapter)
  -> Gateway
  -> native_traversability SSE event
```

The Web event carries:

```text
frame_id=map, stamp_s, reset_epoch, sequence
resolution, origin, yaw (currently axis-aligned; non-zero yaw is rejected)
uint8[] data           # 0..100 control risk
source=native_nav_client, control_authority=true
```

Gateway/Web never writes this layer and never uses it as a command source.
Python planning cost remains a separate event and may not claim control
authority.

## Visualization contract

The finished visualization is comparable to an RViz view, while remaining a
product UI rather than a ROS tool:

```text
3-D scene + minimum observed elevation
  + full native walking-risk heatmap (only when identity-verified)
  + global path + local path + robot pose
  + concise layer status and age
```

High-risk downsampling uses maximum pooling so a dangerous cell cannot be
averaged away.  Any map, frame, producer, coordinate-era, or geometry change
clears the old risk texture immediately.  The UI may retain a stale elevation
surface with an explicit stale badge, but it must never retain a stale risk
layer as if it were current.
