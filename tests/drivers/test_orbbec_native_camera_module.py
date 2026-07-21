from __future__ import annotations

from io import BytesIO
from pathlib import Path

import yaml

from drivers.real.camera.native_camera_module import (
    _FMT_DEPTH_U16,
    _FMT_RGB8,
    _HEADER,
    _KIND_COLOR,
    _KIND_DEPTH,
    _KIND_INTRINSICS,
    _MAGIC,
    _VERSION,
    OrbbecNativeCameraModule,
    _default_executable,
    _runtime_library_paths,
    orbbec_native_build_dir,
    orbbec_sdk_root,
    orbbec_sdk_source,
)
from runtime.contracts import CAMERA_BACKEND_ORBBEC, CAMERA_ROLE
from runtime.registry import clear, get, restore, snapshot

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
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
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
    assert infos[-1].D == [0.1, 0.2, 0.3, 0.4, 0.5]
    assert colors[-1].data.shape == (2, 2, 3)
    assert colors[-1].data[0, 0].tolist() == [0, 0, 255]
    assert depths[-1].data.shape == (2, 2)
    assert int(depths[-1].data[0, 0]) == 1000


def test_orbbec_native_paths_use_vendored_sdk_and_driver_build_dir() -> None:
    sdk_root = orbbec_sdk_root()
    build_dir = orbbec_native_build_dir()
    executable = _default_executable()
    library_paths = _runtime_library_paths()

    assert sdk_root.as_posix().endswith("src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2/orbbec_camera/SDK")
    assert build_dir.as_posix().endswith("build/orbbec_native")
    assert executable.parent == build_dir
    assert library_paths[0] == build_dir / "lib"
    assert library_paths[1].parent == sdk_root / "lib"


def test_orbbec_native_prefers_pure_sdk_root_over_ros2_wrapper(monkeypatch, tmp_path) -> None:
    pure_sdk = tmp_path / "OrbbecSDK"
    pure_sdk.mkdir()
    monkeypatch.setenv("LINGTU_ORBBEC_SDK_ROOT", str(pure_sdk))

    assert orbbec_sdk_root() == pure_sdk
    assert orbbec_sdk_source() == "configured"


def test_orbbec_native_health_reports_sdk_boundary() -> None:
    module = OrbbecNativeCameraModule(rotate=0)
    health = module.health()

    assert health["role"] == CAMERA_ROLE
    assert health["device"] == "orbbec_gemini335_main"
    assert health["camera_backend"] == CAMERA_BACKEND_ORBBEC
    assert health["backend"] == "native_orbbec"
    assert health["native_executable"].endswith(("orbbec_capture", "orbbec_capture.exe"))
    assert health["sdk_root"].endswith(
        "src\\drivers\\real\\camera\\deps\\orbbec\\OrbbecSDK_ROS2\\orbbec_camera\\SDK"
    ) or health["sdk_root"].endswith("src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2/orbbec_camera/SDK")
    assert health["sdk_source"] == "ros2_wrapper_fallback"
    assert health["ros2_wrapper_fallback"] is True
    assert health["runtime_library_paths"]


def test_orbbec_sdk_calls_stay_in_native_capture_functions() -> None:
    capture_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "capture_process.cpp").read_text(
        encoding="utf-8"
    )
    service_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "sdk.cpp").read_text(
        encoding="utf-8"
    )
    service_header = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "sdk.hpp").read_text(
        encoding="utf-8"
    )
    sdk_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "impl" / "orbbec" / "camera.cpp").read_text(
        encoding="utf-8"
    )
    sdk_header = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "impl" / "orbbec" / "camera.hpp").read_text(
        encoding="utf-8"
    )
    compat_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera.cpp").read_text(
        encoding="utf-8"
    )
    compat_header = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera.hpp").read_text(
        encoding="utf-8"
    )

    assert "#include <libobsensor/ObSensor.hpp>" in sdk_source
    assert "libobsensor" not in service_header
    assert "libobsensor" not in service_source
    assert "libobsensor" not in sdk_header
    assert "libobsensor" not in capture_source
    assert "libobsensor" not in compat_source
    assert "libobsensor" not in compat_header
    assert "../impl/orbbec/camera.cpp" in compat_source
    assert "../impl/orbbec/camera.hpp" in compat_header

    for api_name in (
        "namespace lingtu::drivers::camera",
        "struct Topics",
        "struct Config",
        "struct DeviceInfo",
        "struct Intrinsics",
        "struct Frame",
        "class Service",
        "virtual void connect(",
        "virtual void disconnect(",
        "virtual bool is_connected(",
        "virtual DeviceInfo info(",
        "virtual Intrinsics intrinsics(",
        "virtual Frames read(",
        "using CameraConfig = Config",
        "std::string sdk_config_path",
        "std::string serial_number",
        "std::string uid",
        "int product_id",
        "uint32_t device_index",
        "uint32_t connect_timeout_ms",
        "double dist_k1",
        "double dist_k2",
        "double dist_p1",
        "double dist_p2",
        "double dist_k3",
    ):
        assert api_name in service_header

    for api_name in (
        "using lingtu::drivers::camera::CameraConfig",
        "using lingtu::drivers::camera::RosTopics",
        "class Camera",
        "public lingtu::drivers::camera::Service",
        "void connect(",
        "void disconnect(",
        "bool is_connected(",
        "DeviceInfo info(",
        "Intrinsics intrinsics(",
        "Frames read(",
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
        "rgbDistortion",
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
        assert raw_sdk_call not in service_header
        assert raw_sdk_call not in service_source


def test_orbbec_ros2_topic_contract_matches_sdk_defaults() -> None:
    service_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "sdk.cpp").read_text(
        encoding="utf-8"
    )

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
        assert topic in service_source


def test_orbbec_native_build_includes_service_sdk() -> None:
    build_script = (_REPO_ROOT / "scripts" / "build" / "build_orbbec_native.sh").read_text(encoding="utf-8")

    assert 'SDK_SRC="${LINGTU_ORBBEC_NATIVE_SDK_SOURCE:-$SRC_DIR/sdk.cpp}"' in build_script
    assert "LINGTU_ORBBEC_SDK_ROOT" in build_script
    assert 'SDK_SOURCE="configured"' in build_script
    assert 'SDK_SOURCE="pure_sdk"' in build_script
    assert 'SDK_SOURCE="ros2_wrapper_fallback"' in build_script
    assert "using OrbbecSDK_ROS2 bundled SDK as compatibility fallback" in build_script
    assert "scripts/build/fetch_orbbec_sdk.sh" in build_script
    assert 'src/drivers/real/camera/deps/orbbec/OrbbecSDK"' in build_script
    assert "OrbbecSDK_ROS2" in build_script
    assert 'IMPL_DIR="${LINGTU_ORBBEC_IMPL_SOURCE_DIR:-src/drivers/real/camera/impl/orbbec}"' in build_script
    assert 'CAMERA_SRC="${LINGTU_ORBBEC_NATIVE_CAMERA_SOURCE:-$IMPL_DIR/camera.cpp}"' in build_script
    assert '"$SDK_SRC"' in build_script


def test_camera_dds_cpp_publisher_has_typed_writers_and_build_target() -> None:
    camera_dds_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera_dds.cpp").read_text(
        encoding="utf-8"
    )
    endpoint_cmake = (_REPO_ROOT / "src" / "nav" / "services" / "endpoint" / "cpp" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    camera_cmake = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    build_script = (_REPO_ROOT / "scripts" / "build" / "build_camera_dds.sh").read_text(encoding="utf-8")
    runner = (_REPO_ROOT / "scripts" / "deploy" / "thunder" / "run_camera_dds.sh").read_text(encoding="utf-8")

    assert "add_executable(lingtu_camera_dds" in camera_cmake
    assert "--target lingtu_camera_dds" in build_script
    assert "LINGTU_CYCLONEDDS_PREFIX" in build_script
    assert "CMAKE_PREFIX_PATH" in build_script
    assert "include/${multiarch}" in build_script
    assert "--parallel" in build_script
    assert "native camera DDS publisher is missing" in build_script
    assert "Skipping navd" in endpoint_cmake
    assert "Skipping lingtu_traversability_dds" in endpoint_cmake
    assert "Skipping lingtu_explore_dds" in endpoint_cmake
    assert "Skipping lingtu_nav_control" in endpoint_cmake
    assert "lingtu_teleop_bridge" not in endpoint_cmake
    assert "Skipping lingtu_motion_mock_dds" in endpoint_cmake
    assert "LINGTU_CAMERA_DDS_BIN" in runner
    assert "ERROR: native camera DDS publisher is missing" in runner

    for writer in ("color_writer_", "depth_writer_", "info_writer_"):
        assert "dds_create_writer(publisher_, topic" in camera_dds_source
        assert writer in camera_dds_source

    for topic in ("kCameraColor", "kCameraDepth", "kCameraInfo"):
        assert topic in camera_dds_source

    for dds_type in ("lingtu_dds_Image_desc", "lingtu_dds_CameraInfo_desc"):
        assert dds_type in camera_dds_source

    assert "msg.depth_scale = header.depth_scale_m" in camera_dds_source
    assert "last_info_header" in camera_dds_source
    assert "next_info_publish_s" in camera_dds_source
    assert "info_header.timestamp_s = now_s" in camera_dds_source

    for write_call in (
        "dds_write(color_writer_, &msg)",
        "dds_write(depth_writer_, &msg)",
        "dds_write(info_writer_, &msg)",
    ):
        assert write_call in camera_dds_source


def test_camera_dds_resyncs_capture_stdout_noise() -> None:
    camera_dds_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera_dds.cpp").read_text(
        encoding="utf-8"
    )

    assert "bool readHeader(RecordHeader& header, int stale_timeout_ms)" in camera_dds_source
    assert "non-record camera capture bytes" in camera_dds_source
    assert "capture.readHeader(header, cfg.capture_stale_timeout_ms)" in camera_dds_source
    assert "capture.readExact(&header, sizeof(header))" not in camera_dds_source
    assert "camera capture produced no bytes before stale timeout" in camera_dds_source
    assert "--capture-stale-timeout-ms" in camera_dds_source


def test_orbbec_connect_options_are_exposed_to_native_process() -> None:
    capture_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "capture_process.cpp").read_text(
        encoding="utf-8"
    )
    module_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "module.py").read_text(encoding="utf-8")

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
        "--startup-frame-timeout-ms",
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


def test_orbbec_capture_fails_fast_when_no_frames() -> None:
    capture_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "capture_process.cpp").read_text(
        encoding="utf-8"
    )
    camera_dds_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera_dds.cpp").read_text(
        encoding="utf-8"
    )
    runner = (_REPO_ROOT / "scripts" / "deploy" / "thunder" / "run_camera_dds.sh").read_text(encoding="utf-8")
    service = (_REPO_ROOT / "scripts" / "deploy" / "thunder" / "lingtu-camera-dds.service").read_text(encoding="utf-8")

    assert "startup_frame_timeout_ms" in capture_source
    assert "no valid Orbbec frames before startup timeout" in capture_source
    assert "camera capture ended before publishing the next record" in camera_dds_source
    assert "LINGTU_CAMERA_STARTUP_FRAME_TIMEOUT_MS" in runner
    assert "LINGTU_CAMERA_STARTUP_FRAME_TIMEOUT_MS" in service


def test_orbbec_module_uses_device_inventory_and_robot_mount_config() -> None:
    module = OrbbecNativeCameraModule()
    robot = yaml.safe_load((_REPO_ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8"))

    assert module._product_id == 0x0800
    assert module._color_width == 640
    assert module._depth_width == 640
    assert module._connect_timeout_ms == 10000
    assert module._rotate == robot["camera"]["rotate"]
    assert module._frame_id == "camera_link"


def test_camera_module_resolves_native_orbbec_without_ros2() -> None:
    from lingtu.plugin_seed import seed_builtin_plugins
    from runtime.adapters.perception_gateway import camera_module

    state = snapshot()
    try:
        clear()
        seed_builtin_plugins(groups=("camera",), reload_loaded=True, strict=True)
        generic_cls = get(CAMERA_ROLE, CAMERA_BACKEND_ORBBEC)
        bridge_cls = get("camera_bridge", "default")
        assert generic_cls.__name__ == "OrbbecNativeCameraModule"
        assert generic_cls.__module__ == "drivers.real.camera.module"
        assert bridge_cls.__name__ == "OrbbecNativeCameraModule"
        assert bridge_cls.__module__ == "drivers.real.camera.module"
        cls = camera_module(enable_ros2=False)
        assert cls.__name__ == "OrbbecNativeCameraModule"
        assert cls.__module__ == "drivers.real.camera.module"
    finally:
        restore(state)
