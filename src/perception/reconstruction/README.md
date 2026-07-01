# Reconstruction — 3D Volumetric Reconstruction Pipeline

This package provides the 3D reconstruction system: online TSDF fusion from live sensor streams, offline reconstruction from rosbags, color projection, semantic labeling, and dataset export.

## Files

- **`reconstruction_module.py`** — Core TSDF reconstruction Module: fuses depth images into a volumetric map with online keyframe integration.
- **`bag_reader.py`** — ROS bag reader: extracts depth, RGB, and pose streams from recorded rosbag files for offline reconstruction.
- **`color_projector.py`** — Color projection: back-projects RGB pixels onto reconstructed mesh vertices for textured mesh output.
- **`dataset_io.py`** — Dataset I/O: loads and saves reconstruction state (TSDF volume, mesh, metadata) to disk.
- **`dataset_recorder_module.py`** — Online dataset recorder: captures live RGB-D and pose streams during a mission for later use.
- **`keyframe_exporter_module.py`** — Keyframe exporter: selects and exports high-information keyframes for SLAM loop closure.
- **`ply_writer.py`** — PLY file writer: exports reconstructed mesh to standard PLY format with vertex colors.
- **`semantic_labeler.py`** — Semantic labeler: assigns semantic class labels to mesh vertices via detection bbox projection.
- **`server/`** — Sub-package: standalone HTTP server for remote reconstruction query and control.
- **`tests/`** — Unit and integration tests for reconstruction pipeline components.
