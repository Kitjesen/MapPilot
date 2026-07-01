"""LiDAR stream package - Livox MID-360.

Consumes Livox MID-360 data through the ROS-free SDK2 source by default.
``Lidar`` remains the DDS compatibility class:

    from drivers.real.lidar import Lidar

    lidar = Lidar()
    lidar.connect("192.168.1.115")      # subscribes to DDS stream
    lidar.on_cloud(lambda pts: ...)     # numpy (N, 4): x, y, z, intensity
    lidar.on_imu(lambda imu: ...)       # runtime.msgs.sensor.Imu
    cloud = lidar.wait_for_cloud()      # block until first frame
    print(lidar.health)                 # fps, uptime, point count, ...
    lidar.disconnect()

    # context manager
    with Lidar("192.168.1.115") as lidar:
        cloud = lidar.wait_for_cloud()

Blueprint usage:

    from drivers.real.lidar import LidarModule

    bp.add(LidarModule)
    bp.wire("LidarModule", "scan", "nav.terrain", "cloud")

``LidarModule`` uses the SDK2 source. Use ``Lidar`` directly only for legacy
DDS/livox_ros_driver2 compatibility.
"""

from .frames import LivoxPointFrame
from .lidar import Lidar, LidarHealth, LidarState
from .lidar_module import LidarModule
from .sdk2_stream_source import Sdk2Source
from .source import LidarSource

__all__ = [
    "Lidar",
    "LidarHealth",
    "LidarModule",
    "LidarSource",
    "LidarState",
    "LivoxPointFrame",
    "Sdk2Source",
]
