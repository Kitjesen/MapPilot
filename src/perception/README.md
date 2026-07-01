# Perception

`src/perception/` owns scene perception: detectors, encoders, tracking, scene
graph construction, and reconstruction. It is separate from decision-making.

## Layout

| Area | Files |
| --- | --- |
| Module entry | `perception_module.py`, `detector_module.py`, `encoder_module.py` |
| API/types | `api/`, `detector_base.py`, `tracked_objects.py` |
| Detectors | `yoloe_detector.py`, `yolo_world_detector.py`, `grounding_dino_detector.py`, `bpu_detector.py` |
| Encoders | `clip_encoder.py`, `mobileclip_encoder.py` |
| Tracking/geometry | `instance_tracker.py`, `projection.py`, `geometry_extractor.py` |
| Scene graph | `scg_builder.py`, `scg_path_planner.py`, `global_coverage_mask.py` |
| Reconstruction | `reconstruction/` |
| Tests | `tests/` |
| External adapters | `adapters/ros2/` |

## Boundary

- Perception produces detections, embeddings, scene graphs, and reconstruction
  artifacts.
- It does not choose missions, global paths, local paths, or velocity commands.
- Decision and memory consume perception outputs through runtime ports.
- ROS 2 code stays under `adapters/ros2/`; the top-level perception package is
  not a ROS package and should not own `package.xml`, `setup.py`, or `srv/`.
