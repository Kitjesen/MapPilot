# Perception

`src/perception/` owns LingTu's in-process RGB-D scene-perception path. It
turns synchronized camera and localization samples into map-frame detections
and a scene graph. Navigation and Product lifecycle policy do not belong here.

## Layout

| Path | Responsibility |
| --- | --- |
| `module.py` | Blueprint-facing ports, one-slot worker lifecycle, publication, health, and startup readiness |
| `frames.py` | Bounded RGB-D, CameraInfo, odometry, and map-to-odom synchronization |
| `pipeline.py` | The deep detect → project → track → scene-graph implementation |
| `backends.py` | Canonical detector/encoder provider registration and backend construction |
| `detection/` | Detector implementations and detection value types |
| `encoding/` | CLIP-family providers used by their explicit consumers |
| `tracking/` | 3D projection, association, tracked objects, and scene-graph construction |
| `inspection/` | Native inspection-evidence bridge and its Module adapter |
| `reconstruction/` | Optional keyframe recording and volumetric reconstruction |

There is one production scene-perception Module:
`perception.module.PerceptionModule`, registered as `("perception", "scene")`
and assembled under the alias `PerceptionModule`. Standalone factories and a
second service pipeline are deliberately not maintained.

## Data flow

```text
color_image ─┐
depth_image ─┤
camera_info ─┼─> FrameSynchronizer ─> latest-frame worker ─> PerceptionPipeline
odometry ────┤                                      │
map_odom_tf ─┘                                      ├─> robot_pose
                                                    ├─> detections_3d
                                                    └─> scene_graph
```

The port Adapter caches inputs and submits matched samples; detector work does
not run on the camera publisher thread. The worker has one pending slot, so a
new sample replaces pending stale work instead of creating an unbounded queue.

Real Products resolve their declared detector provider. Simulation Products
use the same pipeline with the `sim_scene` observation provider. This is an
implementation substitution inside one Module interface, not a second
simulation-specific perception graph.

## Configuration ownership

Assembly reads `runtime.config.get_config()` once and constructs immutable
`PerceptionSettings` and `DetectorSpec` values. An explicitly declared Product
Host value overrides the corresponding typed value; omitted values retain the
RobotConfig setting.

| Setting | Owner |
| --- | --- |
| Detector selection and explicit Product overrides | Resolved Product Host configuration |
| Detection, projection, tracking, and synchronization defaults | `runtime.config.PerceptionConfig` |
| Camera-to-body calibration and U16 depth scale | `runtime.config.CameraConfig` |
| Runtime intrinsics, distortion, and sensor depth scale | `CameraIntrinsics` input |

`DEPTH_F32` images are metres and therefore use scale `1.0`. `DEPTH_U16`
images use the current valid CameraInfo scale, falling back to
`CameraConfig.depth_scale`. Camera calibration is physical robot data and must
not be duplicated in Product YAML.

## Synchronization and failure semantics

- RGB and depth samples are consumed as one-to-one pairs within the configured
  skew. A newer color frame cannot reuse a depth frame already assigned to an
  older color frame.
- A sample also requires fresh odometry and, when odometry is not already in
  `map`, a fresh map-to-odom transform.
- CameraInfo may update while the Module is running.
- Only a successfully processed frame with a confirmed zero detections advances
  the tracker as a negative observation.
- Blur, stale inputs, unsupported image formats, projection failure, and
  backend failure update health without fabricating an empty observation.
- A tracker failure may emit current-frame untracked detections and a fallback
  graph, but never republishes old tracker state with a new timestamp.
- Successful publication order is `robot_pose`, `detections_3d`, then
  `scene_graph`, all anchored to the source color timestamp and the `map`
  frame.

## Lifecycle and readiness

Setup must construct the selected observation backend. A missing detector or
simulation observer is a setup failure, allowing a Product that marks
`PerceptionModule` critical to fail closed.

Startup readiness requires a live worker and at least one valid synchronized
sample processed by the pipeline; a valid zero-detection result counts. On
stop, the Module first closes its input seam, discards pending work, waits for
the in-flight frame, and lets the worker release each backend object once.

Health retains the established detector and port fields and also exposes
matched, processed, coalesced, and categorized failure counters. Health is
diagnostic state; downstream behavior continues to use typed output ports.

## Dependency boundary

- Perception may depend on `runtime` and on its own detector, projection, and
  tracking implementations.
- Other domains consume perception through ports and registries, not by
  reaching into `PerceptionModule` for its detector, tracker, or encoder.
- Perception does not import navigation, decision, drivers, or Gateway code.
- Perception never selects missions, plans paths, arbitrates safety, or emits
  velocity commands.
- Local and MuJoCo validation do not constitute S100P field evidence.
