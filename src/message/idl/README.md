# IDL Ownership

LingTu has two DDS schema families:

1. Compatibility DDS binds upstream ROS 2 and Livox message IDL:

- `livox_ros_driver2/msg/CustomMsg`
- `sensor_msgs/msg/Imu`
- `sensor_msgs/msg/PointCloud2`
- `nav_msgs/msg/Odometry`
- `nav_msgs/msg/Path`
- `geometry_msgs/msg/PoseStamped`
- `geometry_msgs/msg/TwistStamped`
- `std_msgs/msg/String`
- `std_msgs/msg/Float32`

2. Native field DDS uses LingTu-owned IDL in `lingtu_slam.idl`.

Current native SLAM boundary types are `lingtu.dds.LivoxFrame`,
`lingtu.dds.Imu`, `lingtu.dds.Odometry`, `lingtu.dds.PointCloud2`,
`lingtu.dds.Float32`, and `lingtu.dds.Text`.

Use the native IDL when the process must not link ROS 2, `rclcpp`, or
`livox_ros_driver2`. Native publishers and subscribers must both use the
LingTu IDL type on the same DDS topic; it is not wire-compatible with a ROS 2
publisher using `livox_ros_driver2/msg/CustomMsg` on that topic.

Direct native publishers/subscribers use `idlc` C-generated types plus the
CycloneDDS C API. Do not require `cyclonedds-python` or CycloneDDS-CXX on the
robot. The fallback C++ structs in `dds_topics.hpp` are contract tags for
portable builds, not DDS wire types.

`/lidar/raw_frame` carries scan-level `LivoxFrame` data for SLAM.
`/lidar/raw_packet` carries packet-level `LivoxFrame` diagnostics only.

Python native DDS product types live in `message.dds_types`. The optional
cyclonedds-python utility in `runtime.adapters.dds.reader` is currently bounded
to camera/GNSS integration and explicit diagnostics or compatibility tools.
It is not a navigation, SLAM, or terrain runtime boundary and it must not own
new LingTu wire contracts. `drivers.real.lidar._dds` is the Livox/ROS2
compatibility type shim for the LiDAR adapter.
