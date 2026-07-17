# Semantic Layer Contract

LingTu's semantic layer is not a linear `Detector -> Encoder -> Memory`
pipeline. The runtime contract is centered on `PerceptionModule.scene_graph`.

## Runtime Boundary

`PerceptionModule` is the primary scene-perception module. It consumes camera
frames, depth, camera intrinsics, and odometry, then publishes:

- `scene_graph`: object, relation, and region context for semantic planning,
  memory, reconstruction, gateway, teleop, and visual servo consumers.
- `detections_3d`: projected object detections for semantic planner consumers.

Detector, encoder, tracker, and projection implementations are capabilities
inside `PerceptionModule`, not peer layers in the default full-stack runtime.

## Clean Layer Names

| Layer | Runtime role | Main modules |
| --- | --- | --- |
| L3 Scene Perception | Build live object and scene context from RGB-D and odometry. | `PerceptionModule` |
| L3 Optional Scene Reconstruction | Maintain RGB-D semantic reconstruction from the same camera and scene context. | `ReconstructionModule` |
| L4 Semantic Memory | Persist and query scene, location, temporal, and vector memory from `scene_graph + odometry`. | `SemanticMapperModule`, `VectorMemoryModule`, `EpisodicMemoryModule`, `TaggedLocationsModule`, `TemporalMemoryModule` |
| L4 Decision | Resolve instructions into navigation goals or visual-servo targets. | `SemanticPlannerModule`, `LLMModule`, `VisualServoModule` |

## Capability Components

`DetectorModule` and `EncoderModule` remain valid standalone modules for
experiments and isolated tests. They are not the default full-stack path.

`PerceptionModule` owns the production scene-perception pipeline:

```text
color/depth/camera_info/odometry
  -> detector backend
  -> optional encoder backend
  -> depth projection
  -> instance tracking
  -> scene_graph + detections_3d
```

## Wiring Contract

`PerceptionModule.scene_graph` fans out to:

- `SemanticPlannerModule`
- `VisualServoModule`
- `SemanticMapperModule`
- `VectorMemoryModule`
- `EpisodicMemoryModule`
- `TemporalMemoryModule`
- `ReconstructionModule`
- `GatewayModule`
- `MCPServerModule`
- optional frontier and teleop consumers

This fan-out is the stable boundary. Consumers must not depend on detector or
encoder internals.

## Non-Goals

- Do not split `PerceptionModule` into detector, encoder, projection, and
  tracker modules until a consumer needs independent scheduling or a shared
  embedding service.
- Do not place memory modules under perception. Memory consumes scene context;
  it does not produce the live scene graph.
- Do not introduce ROS 2 coupling into normal semantic modules.

## Multifloor Semantic Navigation Contract

The desired goal is not a bare coordinate. A building-scale semantic target
must resolve to stable spatial identity:

```text
building_id / map_id / floor_id / room_id / connector_id / poi_id
```

The execution hierarchy is:

```text
natural-language instruction
  -> object/room/POI candidates
  -> select target floor and active map
  -> building topology route through stair/ramp/elevator portals
  -> current-floor global plan
  -> local planner + PathFollower
  -> connector transition and localization verification
  -> target-floor global/local plan
```

The tracker already computes hierarchical `floors`, `rooms`, and
`topology_edges`, and object/tagged-location goals can preserve XYZ. A ROS-free
building mission core now exists under `src/nav/building/`, including explicit
lift transition, active-map switching, native relocalization, and target-floor
verification gates. The current production `SceneGraph` boundary and
semantic-memory topology still do not preserve and route the complete
floor/connector model, and no site lift adapter is configured. Therefore
instructions such as "go to the second-floor meeting room" are still not an
enabled product capability today.

Before enabling such missions, the runtime message boundary must retain room
centers, floor IDs, floor ranges, topology edges, and connector endpoints; the
MapGraph route must then be consumed by building mission orchestration rather
than stored only as metadata. The existing transition executor must be bound to
a real lift infrastructure adapter and site transition geometry. Detection
labels for `stairs` or `elevator` alone are not an infrastructure adapter.
