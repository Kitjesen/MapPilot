"""Real hardware driver backends.

Symmetric to ``drivers/sim``. Runtime drivers and colocated vendor sources live
under this package:

- ``real/motion/``: native motion driver with direct ``Go2`` and ``Doso`` robot implementations.
- ``real/camera/``: Orbbec RGB-D native bridge and SDK source.
- ``real/lidar/``: Livox MID-360 driver, SDK2 source, and ROS2 compatibility.
- ``real/gnss/``: GNSS ROS2 compatibility source.
- ``real/camera_jpeg_relay.py``: camera-only JPEG relay for Gateway.
"""
