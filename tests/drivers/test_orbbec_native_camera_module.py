from __future__ import annotations

from io import BytesIO
from pathlib import Path

from runtime.registry import clear, restore, snapshot

from drivers.real.camera.native_camera_module import (
    _FMT_DEPTH_U16,
    _FMT_RGB8,
    _HEADER,
    _KIND_COLOR,
    _KIND_DEPTH,
    _KIND_INTRINSICS,
    _MAGIC,
    _VERSION,
    _default_executable,
    _runtime_library_paths,
    OrbbecNativeCameraModule,
    orbbec_native_build_dir,
    orbbec_sdk_root,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]


def _record(kind: int, width: int, height: int, channels: int, fmt: int, payload: bytes):
    return _HEADER.pack(
        _MAGIC,
        _VERSION,
        kind,
        width,
        height,
        channels,
        fmt,
        123.0,
        500.0,
        501.0,
        1.0,
        2.0,
        0.001,
        len(payload),
    ), BytesIO(payload)


def test_orbbec_native_records_publish_lingtu_streams() -> None:
    module = OrbbecNativeCameraModule(rotate=0)
    colors = []
    depths = []
    infos = []
    module.color_image.subscribe(colors.append)
    module.depth_image.subscribe(depths.append)
    module.camera_info.subscribe(infos.append)

    header, payload = _record(_KIND_INTRINSICS, 2, 2, 0, 0, b"")
    module._publish_record(module._decode_record(header, payload))

    color_payload = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255])
    header, payload = _record(_KIND_COLOR, 2, 2, 3, _FMT_RGB8, color_payload)
    module._publish_record(module._decode_record(header, payload))

    depth_payload = bytes([232, 3, 76, 4, 176, 4, 20, 5])
    header, payload = _record(_KIND_DEPTH, 2, 2, 1, _FMT_DEPTH_U16, depth_payload)
    module._publish_record(module._decode_record(header, payload))

    assert infos[-1].fx == 500.0
    assert infos[-1].depth_scale == 0.001
    assert colors[-1].data.shape == (2, 2, 3)
    assert colors[-1].data[0, 0].tolist() == [0, 0, 255]
    assert depths[-1].data.shape == (2, 2)
    assert int(depths[-1].data[0, 0]) == 1000


def test_orbbec_native_paths_use_vendored_sdk_and_driver_build_dir() -> None:
    sdk_root = orbbec_sdk_root()
    build_dir = orbbec_native_build_dir()
    executable = _default_executable()
    library_paths = _runtime_library_paths()

    assert sdk_root.as_posix().endswith(
        "src/drivers/real/camera/OrbbecSDK_ROS2/orbbec_camera/SDK"
    )
    assert build_dir.as_posix().endswith("build/orbbec_native")
    assert executable.parent == build_dir
    assert library_paths[0] == build_dir / "lib"
    assert library_paths[1].parent == sdk_root / "lib"


def test_orbbec_native_health_reports_sdk_boundary() -> None:
    module = OrbbecNativeCameraModule(rotate=0)
    health = module.health()

    assert health["backend"] == "native_orbbec"
    assert health["native_executable"].endswith(("orbbec_capture", "orbbec_capture.exe"))
    assert health["sdk_root"].endswith(
        "src\\drivers\\real\\camera\\OrbbecSDK_ROS2\\orbbec_camera\\SDK"
    ) or health["sdk_root"].endswith(
        "src/drivers/real/camera/OrbbecSDK_ROS2/orbbec_camera/SDK"
    )
    assert health["runtime_library_paths"]


def test_orbbec_sdk_calls_stay_in_native_capture_functions() -> None:
    capture_source = (
        _REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "capture_process.cpp"
    ).read_text(encoding="utf-8")
    sdk_source = (
        _REPO_ROOT
        / "src"
        / "drivers"
        / "real"
        / "camera"
        / "native"
        / "camera.cpp"
    ).read_text(encoding="utf-8")
    sdk_header = (
        _REPO_ROOT
        / "src"
        / "drivers"
        / "real"
        / "camera"
        / "native"
        / "camera.hpp"
    ).read_text(encoding="utf-8")

    assert "#include <libobsensor/ObSensor.hpp>" in sdk_source
    assert "libobsensor" not in sdk_header
    assert "libobsensor" not in capture_source

    for api_name in (
        "struct RosTopics",
        "struct VideoMode",
        "struct CameraConfig",
        "struct DeviceInfo",
        "struct Intrinsics",
        "struct Frame",
        "class Camera",
        "void connect(",
        "void disconnect(",
        "bool is_connected(",
        "DeviceInfo info(",
        "Intrinsics intrinsics(",
        "Frames read(",
        "std::string sdk_config_path",
        "std::string serial_number",
        "std::string uid",
        "int product_id",
        "uint32_t device_index",
        "uint32_t connect_timeout_ms",
    ):
        assert api_name in sdk_header

    for raw_sdk_call in (
        "enableVideoStream(",
        ".queryDeviceList(",
        "->getDeviceBySN(",
        "->getDeviceByUid(",
        "->getPid(",
        "->getDevice(",
        "->start(",
        "->getCameraParam(",
        "->waitForFrames(",
        "->colorFrame(",
        "->depthFrame(",
        "->getValueScale(",
        "->getFormat(",
        "->getWidth(",
        "->getHeight(",
        "->getData(",
        "->getDataSize(",
        "->stop(",
    ):
        assert raw_sdk_call in sdk_source
        assert raw_sdk_call not in capture_source


def test_orbbec_ros2_topic_contract_matches_sdk_defaults() -> None:
    sdk_source = (
        _REPO_ROOT
        / "src"
        / "drivers"
        / "real"
        / "camera"
        / "native"
        / "camera.cpp"
    ).read_text(encoding="utf-8")

    for topic in (
        "color/image_raw",
        "color/camera_info",
        "depth/image_raw",
        "depth/camera_info",
        "depth/points",
        "depth_registered/points",
        "depth/image_unaligned",
        "depth_to_color/image_raw",
        "ir/image_raw",
        "left_ir/image_raw",
        "right_ir/image_raw",
        "gyro/sample",
        "gyro/imu_info",
        "accel/sample",
        "accel/imu_info",
        "gyro_accel/sample",
        "depth_to_color",
        "device_status",
        "depth_filter_status",
        "depth_filters/status",
    ):
        assert topic in sdk_source


def test_orbbec_connect_options_are_exposed_to_native_process() -> None:
    capture_source = (
        _REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "capture_process.cpp"
    ).read_text(encoding="utf-8")
    module_source = (
        _REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native_camera_module.py"
    ).read_text(encoding="utf-8")

    for option in (
        "--serial-number",
        "--uid",
        "--usb-port",
        "--product-id",
        "--device-index",
        "--connect-timeout-ms",
        "--sdk-config",
        "--enable-frame-sync",
        "--color-width",
        "--color-height",
        "--color-fps",
        "--depth-width",
        "--depth-height",
        "--depth-fps",
    ):
        assert option in capture_source

    for option in (
        "--serial-number",
        "--uid",
        "--product-id",
        "--device-index",
        "--connect-timeout-ms",
        "--sdk-config",
        "--enable-frame-sync",
        "--color-width",
        "--depth-width",
    ):
        assert option in module_source


def test_orbbec_module_defaults_come_from_device_config() -> None:
    module = OrbbecNativeCameraModule()

    assert module._product_id == 0x0800
    assert module._color_width == 640
    assert module._depth_width == 640
    assert module._connect_timeout_ms == 10000
    assert module._rotate == 270
    assert module._frame_id == "camera_link"


def test_camera_bridge_module_resolves_native_orbbec_without_ros2() -> None:
    from lingtu.plugin_seed import seed_builtin_plugins
    from runtime.blueprints.adapters.perception_gateway import camera_bridge_module

    state = snapshot()
    try:
        clear()
        seed_builtin_plugins(groups=("camera",), reload_loaded=True, strict=True)
        cls = camera_bridge_module(enable_ros2=False)
        assert cls.__name__ == "OrbbecNativeCameraModule"
        assert cls.__module__ == "drivers.real.camera.native_camera_module"
    finally:
        restore(state)
