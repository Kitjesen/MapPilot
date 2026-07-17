# Orbbec deps

Preferred dependency:

```text
OrbbecSDK/
```

This should be the pure Orbbec SDK package, without ROS2 wrapper code.

Temporary fallback:

```text
OrbbecSDK_ROS2/
```

`OrbbecSDK_ROS2` is still accepted during migration because it contains the
pure SDK under:

```text
OrbbecSDK_ROS2/orbbec_camera/SDK
```
