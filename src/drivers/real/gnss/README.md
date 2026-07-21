# GNSS

`gnss` is the short runtime role for positioning receivers.

Product path:

- `native/module.hpp` and `native/module.cpp` expose the C++ service boundary.
- `native/dds_module.hpp` and `native/dds_module.cpp` publish GNSS DDS topics.
- `native/gnss_dds.cpp` is only the process entrypoint.
- `impl/wtrtk980/` contains the WTRTK-980 serial/NMEA backend.

The native path publishes DDS directly:

- `/gnss/fix` -> `rt/gnss/fix`
- `/gnss/status` -> `rt/gnss/status`
- `/gnss/odom` -> `rt/gnss/odom` when a map origin is configured

ROS2 GNSS reader code is legacy compatibility only and is not present in this
real driver tree. Do not add ROS2 imports to the product GNSS role
implementation.
