# RealSense D435i capture

Optional C++ librealsense2 backend for the existing `lingtu_camera_dds`
record/SHM interface. No ROS, Python capture loop, or new DDS topics.

Install the official librealsense2 development/runtime package for the target
Linux machine, then run from the repository root:

```sh
bash scripts/build/build_realsense_native.sh
build/realsense_native/realsense_capture --list-devices
build/realsense_native/realsense_capture --serial-number SERIAL --max-frames 30 > /tmp/d435i.records
```

For a custom SDK prefix pass `-DCMAKE_PREFIX_PATH=/path/to/sdk` to the build script.
The SDK runtime must also be installed on the deployment machine. The normal
release packager includes the capture executable when it has been built.

The existing camera service launcher selects this backend with:

```sh
LINGTU_CAMERA_DRIVER=realsense_native
LINGTU_REALSENSE_SERIAL_NUMBER=SERIAL
```

Set these in the robot's deployment environment. ProductControl remains the
service owner; the selected Product must already declare the camera role. For
the Go2 field deployment, the existing RobotConfig already selects
`realsense_native` and the SDK camera serial. The supported public entry point
is ProductControl's `nav --variant camera`; do not start `lt-camera.service`
or the capture process independently.
Do not launch a competing camera service beside a running Product. The default
LiDAR-only Go2 navigation and mapping Products are unchanged.

RGB8 and aligned Z16 are captured at 640x480/30 Hz by default. Depth is aligned
to the color optical viewpoint with librealsense, normalized to uint16
millimeters using the measured depth unit, and invalid zero depth is retained.
New records from one frameset carry its host reception timestamp in Unix
seconds; this is not a hardware-synchronized LiDAR exposure timestamp. When
color and depth run at different rates, librealsense can reuse the slower
stream's frame. Each stream's SDK frame number is tracked independently, so
that exposure is not published again with a newer timestamp. Color intrinsics
describe the aligned image grid. SDK alignment retains the original depth
samples; pixel association is not a replacement for exact metric geometry or
camera-to-body calibration. Unsupported distortion models fail explicitly
rather than publishing misleading calibration. Row padding is removed.

The D435i IMU is not published and does not replace the MID-360 SLAM IMU.
Go2 camera-to-body extrinsics remain unverified. Image preview is distinct from
using depth for navigation: measuring extrinsics, time alignment, and depth-cloud
filtering is still required before map fusion. On the NX, librealsense reports
the D435i SDK serial used by `realsense_capture`; this can differ from the USB
V4L `/dev/v4l/by-id` identifier. Use `realsense_capture --list-devices` when
checking the configured serial, rather than substituting the V4L identifier.
The camera is available only after the `nav --variant camera` Product is
selected and its capture binary is present in the release package.

References: https://github.com/realsenseai/librealsense/tree/v2.56.5
and the official `examples/align` example.
