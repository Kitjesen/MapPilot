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
service owner; the selected Product must already declare the camera role.
Do not launch a competing camera service beside a running Product. The default
LiDAR-only Go2 navigation and mapping Products are unchanged.

RGB8 and aligned Z16 are captured at 640x480/30 Hz by default. Depth is aligned
to the color optical viewpoint with librealsense, normalized to uint16
millimeters using the measured depth unit, and invalid zero depth is retained.
All three records carry the same host reception timestamp in Unix seconds;
this is not a hardware-synchronized LiDAR exposure timestamp. Color intrinsics
are the shared aligned-image intrinsics. Unsupported distortion models fail
explicitly rather than publishing misleading calibration. Row padding is removed.

The D435i IMU is not published and does not replace the MID-360 SLAM IMU.
Go2 camera-to-body extrinsics remain unverified. Image preview is distinct from
using depth for navigation: measuring extrinsics, time alignment, and depth-cloud
filtering is still required before map fusion. No camera is currently detected on NX.

References: https://github.com/realsenseai/librealsense/tree/v2.56.5
and the official `examples/align` example.
