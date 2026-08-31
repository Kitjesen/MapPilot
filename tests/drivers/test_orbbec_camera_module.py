from __future__ import annotations

from io import BytesIO
from pathlib import Path

from drivers.real.camera.module import (
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
)
from runtime.contracts import CAMERA_BACKEND_ORBBEC, CAMERA_ROLE

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


def test_orbbec_native_runtime_uses_packaged_build_dir() -> None:
    build_dir = orbbec_native_build_dir()
    executable = _default_executable()
    library_paths = _runtime_library_paths()

    assert build_dir.as_posix().endswith("build/orbbec_native")
    assert executable.parent == build_dir
    assert library_paths == (build_dir / "lib", build_dir / "lib" / "extensions")


def test_orbbec_native_health_reports_sdk_boundary() -> None:
    module = OrbbecNativeCameraModule(rotate=0)
    health = module.health()

    assert health["role"] == CAMERA_ROLE
    assert health["device"] == "orbbec_gemini335_main"
    assert health["camera_backend"] == CAMERA_BACKEND_ORBBEC
    assert health["backend"] == "native_orbbec"
    assert health["native_executable"].endswith(("orbbec_capture", "orbbec_capture.exe"))
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
    assert "scripts/build/fetch_orbbec_sdk.sh" in build_script
    assert "build/deps/orbbec-sdk" in build_script
    assert 'cp -a "$SDK_LIB/extensions" "$RUNTIME_LIB/"' in build_script
    assert 'IMPL_DIR="${LINGTU_ORBBEC_IMPL_SOURCE_DIR:-src/drivers/real/camera/impl/orbbec}"' in build_script
    assert 'CAMERA_SRC="${LINGTU_ORBBEC_NATIVE_CAMERA_SOURCE:-$IMPL_DIR/camera.cpp}"' in build_script
    assert '"$SDK_SRC"' in build_script


def test_camera_dds_cpp_publisher_has_typed_writers_and_build_target() -> None:
    camera_dds_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera_dds.cpp").read_text(
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


def test_camera_dds_uses_one_absolute_deadline_for_each_complete_record() -> None:
    camera_dds_source = (_REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera_dds.cpp").read_text(
        encoding="utf-8"
    )

    contract_source = (
        _REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera_record.hpp"
    ).read_text(encoding="utf-8")
    deadline_test = (
        _REPO_ROOT
        / "src"
        / "drivers"
        / "real"
        / "camera"
        / "native"
        / "tests"
        / "test_camera_record_deadline.cpp"
    ).read_text(encoding="utf-8")

    assert "camera_record::RecordDeadline deadline" in camera_dds_source
    assert "camera_record::readExactUntil(" in camera_dds_source
    assert "remainingRecordTimeoutMs(deadline)" in contract_source
    assert "testTrickleBytesCannotExtendRecordDeadline" in deadline_test
    assert "std::this_thread::sleep_for(std::chrono::milliseconds(20))" in deadline_test
    assert "record::makeRecordDeadline(75)" in deadline_test
    assert "non-record camera capture bytes" in camera_dds_source
    assert "camera_record::makeRecordDeadline(cfg.capture_stale_timeout_ms)" in camera_dds_source
    assert "capture.readHeader(header, record_deadline)" in camera_dds_source
    assert "capture.readExact(payload.data(), payload.size(), record_deadline)" in camera_dds_source
    assert "capture.readHeader(header, cfg.capture_stale_timeout_ms)" not in camera_dds_source
    assert "camera record exceeded absolute read deadline" in camera_dds_source
    assert "camera_record::kMinRecordTimeoutMs" in camera_dds_source
    assert "camera_record::kMaxRecordTimeoutMs" in camera_dds_source
    assert "--capture-stale-timeout-ms must be between" in camera_dds_source


def test_camera_record_validation_precedes_allocation_publish_and_counting() -> None:
    native = _REPO_ROOT / "src" / "drivers" / "real" / "camera" / "native"
    camera_dds_source = (native / "camera_dds.cpp").read_text(encoding="utf-8")
    capture_source = (native / "capture_process.cpp").read_text(encoding="utf-8")
    contract_source = (native / "camera_record.hpp").read_text(encoding="utf-8")

    validation = camera_dds_source.index("validateHeader(header);")
    intrinsics_gate = camera_dds_source.index("validateRecordSequence(header, has_info)")
    allocation = camera_dds_source.index("payload.resize(header.payload_size)")
    assert validation < intrinsics_gate < allocation
    assert intrinsics_gate < camera_dds_source.index("color_shm.publish")
    assert intrinsics_gate < camera_dds_source.index("records += 1")
    assert "validateRecordHeader(header)" in camera_dds_source
    header_validation = capture_source.index("validateRecordHeader(header)")
    payload_validation = capture_source.index("validateRecordPayloadPointer(header, payload)")
    header_write = capture_source.index("std::cout.write(reinterpret_cast<const char *>(&header)")
    assert header_validation < payload_validation < header_write
    assert "camera_record::kDepthScaleMetersPerMillimeter" in capture_source
    depth_layout_validation = capture_source.index("validateCanonicalDepthLayout(")
    depth_allocation = capture_source.index("millimeters.resize(layout.pixel_count)")
    assert depth_layout_validation < depth_allocation
    assert "static_cast<const std::uint16_t*>(depth.data)" not in capture_source
    assert "normalizeDepthSampleMillimeters(" in capture_source
    assert 'throw std::runtime_error("camera_record_output_failed")' in capture_source
    assert "timestamp_s > 0.0 ? timestamp_s : nowSeconds()" not in camera_dds_source
    for token in (
        "kUnsupportedKind",
        "kInvalidDimensions",
        "kInvalidColorFormat",
        "kInvalidColorChannels",
        "kInvalidDepthFormat",
        "kInvalidDepthChannels",
        "kPayloadSizeMismatch",
        "kWireByteOrder",
        "offsetof(RecordHeader, payload_size) == 72",
    ):
        assert token in contract_source


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
    service = (_REPO_ROOT / "scripts" / "deploy" / "thunder" / "lt-camera.service").read_text(encoding="utf-8")

    assert "startup_frame_timeout_ms" in capture_source
    assert "no valid Orbbec frames before startup timeout" in capture_source
    assert "camera capture ended before publishing the next record" in camera_dds_source
    assert "LINGTU_CAMERA_STARTUP_FRAME_TIMEOUT_MS" in runner
    assert "LINGTU_CAMERA_STARTUP_FRAME_TIMEOUT_MS" in service


def test_orbbec_module_uses_device_inventory_and_robot_mount_config() -> None:
    module = OrbbecNativeCameraModule()

    assert module._product_id == 0x0800
    assert module._color_width == 640
    assert module._depth_width == 640
    assert module._connect_timeout_ms == 10000
    assert module._rotate == 0
    assert module._frame_id == "camera_link"
