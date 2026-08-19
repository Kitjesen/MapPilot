# Online Loop-Closure Visualization

LingTu exposes loop-closure verification to operators as a read-only
`MapSceneFrame` layer. The layer is visualization evidence only: it never owns
SLAM correction, pose-graph optimization, map persistence, planning, or motion.

## Scene contract

The canonical layer identity is `localization.loop_constraints`:

```json
{
  "id": "localization.loop_constraints",
  "type": "loop_constraints",
  "frame_id": "map",
  "stamp_s": 100.0,
  "producer_boot_id": "slam-boot-a",
  "reset_epoch": 7,
  "observation_sequence": 42,
  "generation": 9,
  "online": true,
  "identity_verified": true,
  "constraint_semantics": "loop_closure_validation_v1",
  "constraints": [
    {
      "from_index": 1,
      "to_index": 12,
      "from": [1.0, 2.0, 0.3],
      "to": [4.0, -1.0, 0.5],
      "state": "accepted",
      "geometrically_verified": true,
      "rmse_m": 0.12
    }
  ]
}
```

Gateway forwards this small metadata layer through the existing `map_scene`
SSE event. The Web scene renders geometrically verified accepted edges in
green and rejected candidates in amber. It hides the layer when the producer
identity, `map` frame, reset epoch, semantics, or freshness does not match the
current scene.

## Producer boundary

The native online SLAM/PGO owner is the only production producer for this
layer. The legacy ROS 2 PGO `/pgo/loop_markers` topic remains a development RViz
surface and is not bridged into the field Product. The saved-map
`lt_loop_verify` shadow report is also not an online producer: it is an audit
artifact and does not modify the graph.

The Gateway pass-through and Web renderer are implemented. Until a native
online SLAM/PGO process publishes this layer on each coherent MapScene update,
the UI intentionally reports that the current localization backend has not
published online loop verification.
