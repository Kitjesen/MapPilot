# Real — Hardware Drivers for Physical Robots

This package provides drivers for real robot hardware. Each sub-package encapsulates a complete hardware interface stack, from low-level communication to high-level Module ports.

## `thunder/` — Thunder Robot (gRPC via Brainstem)

- **`__init__.py`** — Package init; exports the Thunder driver and connection surfaces.
- **`blueprints.py`** — Legacy compatibility shim; product blueprints live under `core.blueprints.products.thunder`.
- **`connection.py`** — Connection management: gRPC channel lifecycle, retry, and reconnection to brainstem control service.
- **`han_dog_module.py`** — HanDog hardware module: low-level joint command interface for the Han robot variant.

Camera bridge adapters live under `src/compat/ros2/camera_bridge.py` and are composed by the product/perception stack when camera input is enabled. The Thunder hardware package should not import ROS compatibility modules directly.

## `lidar/` — Livox MID-360 LiDAR Driver

- **`__init__.py`** — Package init; exports `LidarModule`.
- **`lidar.py`** — LiDAR configuration and utilities: IP, port, scan pattern parameters for Livox MID-360.
- **`lidar_module.py`** — LidarModule: hardware driver for Livox MID-360; publishes PointCloud2 via UDP packet capture.
- **`_dds.py`** — DDS integration: publishes lidar data over CycloneDDS for cross-process consumption.
