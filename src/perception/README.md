# Perception

`src/perception/` owns the production scene-perception path: detection,
embedding, cross-frame tracking, scene-graph construction, inspection
evidence, and 3D reconstruction. Navigation and research planners do not
belong in this package.

## Production layout

| Path | Responsibility |
| --- | --- |
| `perception_module.py` | Blueprint-facing detector → encoder → tracker → scene-graph Module |
| `service.py` | Framework-free perception coordinator for tests and standalone tools |
| `detection/` | Detector backends and registry entries |
| `encoding/` | CLIP-family encoder backends |
| `tracking/` | Projection, association, tracked objects, and scene-graph construction |
| `services/` | Perception-owned runtime services, including scene-graph publication |
| `inspection/` | Native inspection-evidence bridge and Module adapter |
| `reconstruction/` | Volumetric reconstruction |
| `api/` and `impl/` | Narrow factory-based construction path |
| `adapters/ros2/` | Explicit compatibility and offline adapters |

## Boundary

- Perception produces detections, embeddings, tracked objects, scene graphs,
  inspection evidence, and reconstruction artifacts.
- Perception does not select missions, plan paths, or produce velocity
  commands.
- Other domains consume perception through runtime messages, ports, and
  registries instead of importing concrete perception implementations.
- ROS 2 remains an explicit compatibility path; the Product data plane uses
  the declared native/DDS endpoints.

## Current composition

`PerceptionModule` is the aggregate runtime entry. Detector, encoder, and
tracker implementations remain replaceable through the existing registry and
configuration surfaces. Production scene-graph state is owned by
`services/scene_graph_service.py` and `tracking/scene_graph_builder.py`.

Experimental USS-Nav/SCG planners, offline benchmarks, and their demos were
removed from `src/perception/` because no Product or Blueprint consumed them.
New experiments should stay outside installable production packages.
