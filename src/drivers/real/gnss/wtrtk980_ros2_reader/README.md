# WTRTK-980 ROS2 Reader (C++)

This toolset reads the WitMotion WTRTK-980 GNSS/RTK receiver connected to the
robot and exposes its data through both a C++ CLI and a ROS2 node.

The current implementation is C++ and is split into independent layers so future
changes do not affect unrelated parts.

## Device

The receiver is exposed as:

```bash
/dev/wtrtk980
```

Serial settings:

```text
115200 baud, 8N1, no flow control
NMEA0183 text output
```

The udev rule creates `/dev/wtrtk980` for CH340/CH341 devices with:

```text
idVendor=1a86
idProduct=7523
driver=ch341
```

Because the WTRTK-980 USB-serial chip has no unique serial number, the C++ code
also supports safer auto-detection: it first filters CH340/CH341 candidates, then
reads candidate output and confirms WTRTK/GNSS-style data such as:

```text
#BASEINFOA
$GNRMC
$GNGGA
$GPGSA
$GPGSV
```

## Project Location

```bash
/home/sunrise/tools/wtrtk980_cpp_ws/src/wtrtk980_ros2_reader
```

Built workspace:

```bash
/home/sunrise/tools/wtrtk980_cpp_ws
```

## Decoupled Structure

```text
include/wtrtk980/gps_fix.hpp       data model only
include/wtrtk980/nmea_parser.hpp   NMEA parsing API
include/wtrtk980/serial_detector.hpp  device discovery / auto-detection
include/wtrtk980/serial_reader.hpp serial reading API
src/nmea_parser.cpp                NMEA parsing implementation
src/serial_detector.cpp            CH340 filtering + output-content probing
src/serial_reader.cpp              POSIX serial reader
src/cli_main.cpp                   command-line adapter
src/ros2_node.cpp                  ROS2 adapter
```

Dependency direction:

```text
gps_fix -> nmea_parser -> serial_reader -> CLI / ROS2 node
serial_detector -> serial_reader auto device selection
```

The core library does not depend on ROS2. ROS2 is only an adapter layer.

## Build

```bash
cd /home/sunrise/tools/wtrtk980_cpp_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select wtrtk980_ros2_reader
source install/setup.bash
```

## CLI Usage

Find the WTRTK-980 device:

```bash
ros2 run wtrtk980_ros2_reader wtrtk980_reader --find
```

Read parsed status using auto-detection:

```bash
ros2 run wtrtk980_ros2_reader wtrtk980_reader --device auto --seconds 10
```

Read raw NMEA too:

```bash
ros2 run wtrtk980_ros2_reader wtrtk980_reader --device auto --seconds 10 --raw
```

Indoor/no-fix output is expected to look like:

```text
fix=false quality=0 satellites=0 lat=None lon=None alt_m=None hdop=9999
```

Outdoor valid GNSS should look like:

```text
fix=true quality=1 satellites=20 lat=31.x lon=121.x alt_m=22.x hdop=0.7
```

RTK fixed correction, if configured and achieved, generally appears as:

```text
quality=4
```

## ROS2 Node

Run manually:

```bash
cd /home/sunrise/tools/wtrtk980_cpp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wtrtk980_ros2_reader wtrtk980_ros2_node
```

With explicit parameters:

```bash
ros2 run wtrtk980_ros2_reader wtrtk980_ros2_node --ros-args \
  -p device:=auto \
  -p fix_topic:=/gps/fix \
  -p raw_topic:=/gps/nmea \
  -p frame_id:=wtrtk980 \
  -p publish_raw:=true
```

Published topics:

```text
/gps/fix   sensor_msgs/msg/NavSatFix
/gps/nmea  std_msgs/msg/String
```

Subscribe:

```bash
ros2 topic echo /gps/fix
ros2 topic echo /gps/nmea
```

When indoors or without satellite fix, `/gps/fix` publishes `STATUS_NO_FIX` and
NaN coordinates. In open sky, it publishes real latitude, longitude and altitude.

## C++ API Usage

Core include example:

```cpp
#include "wtrtk980/serial_reader.hpp"

wtrtk980::SerialReader reader("auto", 115200, 1000);
reader.open();
if (auto fix = reader.readUpdate(3000)) {
  if (fix->fix) {
    // use fix->lat, fix->lon, fix->altitude_m, fix->quality, fix->satellites
  }
}
```

Find device only:

```cpp
#include "wtrtk980/serial_detector.hpp"

auto result = wtrtk980::findWtrtk980Device();
// result.device contains /dev/wtrtk980 or another detected serial path
```

## Integration Status

This C++ package is independent. It has not been added to the robot's existing
launch files or startup services. Start it manually first; add it to launch only
after integration decisions are made.
