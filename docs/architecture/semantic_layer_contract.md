# Semantic Layer Contract

Status: current semantic runtime contract
Audience: perception, memory, decision, and Gateway maintainers

LingTu's semantic runtime is centered on `PerceptionModule.scene_graph`.
`PerceptionModule` is the only Blueprint-facing scene-perception adapter. It
consumes `color_image`, `depth_image`, `camera_info`, `odometry`, and
`map_odom_tf`, then publishes `robot_pose`, `detections_3d`, and `scene_graph`.

## Ownership

- `perception.module.PerceptionModule` owns port callbacks, worker lifecycle,
  readiness, health, and ordered publication.
- `perception.frames.FrameSynchronizer` owns RGB-D and pose selection.
- `perception.pipeline.PerceptionPipeline` owns detection, projection,
  instance tracking, and scene-graph construction.
- `perception.backends` is the sole detector and encoder provider registry.
- There is no standalone production `DetectorModule` or `PerceptionService`.
- Standard encoders remain available to explicit consumers such as
  `VectorMemoryModule`; Product perception does not own an encoder.

```text
color/depth/camera_info/odometry/map_odom_tf
  -> frame synchronization
  -> real RGB-D or sim-scene observation adapter
  -> detect/project/track/scene graph
  -> robot_pose/detections_3d/scene_graph
```

## Dependency Boundary

`PerceptionModule.scene_graph` may fan out to semantic planning, visual servo,
memory, reconstruction, Gateway, and MCP consumers. Consumers depend on the
typed output messages, not detector, projection, tracker, or encoder internals.
Perception must not import navigation, decision, drivers, or Gateway code.

## Failure Semantics

Only a valid synchronized frame on which the observer confirms zero targets is
a negative observation. Blur, stale support data, invalid formats, projection
failure, and inference failure are health events and do not fabricate an empty
scene. If tracking fails, the current untracked detections and a current-frame
fallback scene graph are published; old tracker state is never restamped.

## Building-Scale Navigation

A semantic destination is more than a coordinate. Building-scale routing must
preserve stable building, map, floor, room, connector, and POI identities. The
current scene-perception boundary can describe objects and regions, but labels
such as `stairs` or `elevator` do not constitute a lift or transition adapter.
Those missions remain disabled until the runtime message boundary and site
infrastructure adapters carry the complete transition contract.
