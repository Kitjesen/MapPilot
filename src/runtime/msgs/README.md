# Messages - Typed Data Structures for Module Communication

This package defines the message types exchanged between Modules through
`In[T]` and `Out[T]` ports. Each module specifies its port types explicitly;
messages provide structured, validated data containers.

## Ownership

`runtime.msgs` is the canonical package for in-process Python module message
contracts. `src/message` is the wire-contract registry for cross-process typed
DDS topics, including the matching IDL and C++ type names.

Messages describe data. They do not choose whether data moves by local callback,
SHM, LCM, DDS, or a future protobuf envelope.

## Wire Format Rule

Keep Python module ports typed with these message classes. Serialization is an
adapter concern:

- In-process module graph: pass the Python objects directly.
- Endpoint replay: use versioned endpoint schemas under `runtime.adapters.endpoint_sources`.
- DDS product topics: use `src/message/dds.py` and native C++ aliases from
  `src/message/cpp/dds_topics.hpp`.
- Protobuf: add only for a real non-Python/non-DDS boundary that needs generated
  language bindings.

## Files

- `geometry.py` - Pose, Transform, Twist, Point, Quaternion, and Vector3 message
  types for spatial data exchange.
- `map.py` - Map data-plane messages such as `MapCloudFrame` for full,
  keyframe, and incremental map-cloud updates.
- `nav.py` - Navigation messages: Path, Waypoint, GoalStatus, OccupancyGrid, and
  MapMetaData for planning and control.
- `sensor.py` - Sensor data messages: Image, PointCloud2, Imu, LaserScan, and
  CompressedImage for hardware input.
- `semantic.py` - Semantic messages: Detection, SceneGraph, ObjectLabel, and
  SemanticClass for perception output.
- `scene.py` - Scene graph messages: SceneNode, Relationship, Room, and Topology
  for spatial-semantic hybrid maps.
- `robot.py` - Robot state messages: RobotState, JointState, BatteryState, and
  Temperature for hardware status.
- `gnss.py` - GNSS messages: GpsFix, UtmPose, and NavSatFix for outdoor global
  positioning.
