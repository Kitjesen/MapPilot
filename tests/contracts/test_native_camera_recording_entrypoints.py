"""Contracts for the native camera recorder/player boundary."""

# ruff: noqa: D103, S101 - pytest contracts use assertions by design.

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RECORDING = ROOT / "src" / "native" / "recording"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_camera_tools_are_separate_ros_free_native_entrypoints() -> None:
    cmake = _read(RECORDING / "CMakeLists.txt")
    assert "lingtu_camera_recorder" in cmake
    assert "lingtu_camera_player" in cmake

    camera_sources = "\n".join(
        _read(path).lower()
        for path in (RECORDING / "src").glob("camera_*.cpp")
    )
    for forbidden in ("rclcpp", "rosbag", "sensor_msgs", "ros2 ", "ament"):
        assert forbidden not in camera_sources


def test_camera_only_build_can_skip_cyclonedds_and_idlc() -> None:
    script = _read(ROOT / "scripts" / "build" / "build_native_recording.sh")
    assert 'BUILD_DDS="${LINGTU_RECORDING_BUILD_DDS:-ON}"' in script
    assert 'if [ "$BUILD_DDS" = ON ]; then' in script
    assert '-DLINGTU_RECORDING_BUILD_DDS="$BUILD_DDS"' in script


def test_camera_recorder_taps_shm_and_keeps_media_encoder_optional() -> None:
    recorder = _read(RECORDING / "src" / "camera_recorder_main.cpp")
    assert "/lingtu_camera_color" in recorder
    assert "ffmpeg" in recorder.lower()
    assert ".tmp" in _read(RECORDING / "src" / "camera_recording.cpp")

    cmake = _read(RECORDING / "CMakeLists.txt")
    camera_block = cmake.split("lingtu_camera_recorder", 1)[1]
    assert "find_package(FFmpeg" not in camera_block
    assert "find_package(GStreamer" not in camera_block


def test_field_camera_shm_ring_absorbs_segment_rotation_backlog() -> None:
    runner = _read(ROOT / "scripts" / "deploy" / "thunder" / "run_camera_dds.sh")
    service = _read(ROOT / "scripts" / "deploy" / "thunder" / "lingtu-camera-dds.service")

    assert ': "${LINGTU_CAMERA_SHM_SLOT_COUNT:=16}"' in runner
    assert ': "${LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES:=1048576}"' in runner
    assert '--shm-slot-count "${LINGTU_CAMERA_SHM_SLOT_COUNT}"' in runner
    assert '--shm-slot-capacity-bytes "${LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES}"' in runner
    assert "Environment=LINGTU_CAMERA_SHM_SLOT_COUNT=16" in service
    assert "Environment=LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES=1048576" in service


def test_camera_image_shm_inherits_the_latest_camera_info_calibration() -> None:
    producer = _read(ROOT / "src" / "drivers" / "real" / "camera" / "native" / "camera_dds.cpp")

    assert "RecordHeader withCalibrationFrom" in producer
    assert producer.count("withCalibrationFrom(header, last_info_header)") == 2


def test_default_camera_encoder_is_generic_software_not_a_platform_sdk() -> None:
    header = _read(RECORDING / "include" / "lingtu" / "recording" / "camera_linux.hpp")
    recorder = _read(RECORDING / "src" / "camera_recorder_main.cpp")
    cmake = _read(RECORDING / "CMakeLists.txt")

    assert 'std::string codec{"libx264"}' in header
    assert "generic software" in recorder.lower()
    assert "encoder.verify();" in recorder
    assert recorder.index("encoder.verify();") < recorder.index("PosixShmCameraFrameSource source")
    assert "dropped source frame(s); refusing a clean session result" in recorder
    assert "native_camera_software_ffmpeg" in cmake
    for platform_encoder in ("HB_VENC", "SP_ENCODER", "RDK X5"):
        assert platform_encoder not in cmake


def test_camera_player_is_offline_only_and_has_no_live_publish_escape_hatch() -> None:
    player = _read(RECORDING / "src" / "camera_player_main.cpp").lower()
    for expected in ("--list", "--verify", "--extract-window"):
        assert expected in player
    for forbidden in (
        "dds_write",
        "shm_open",
        "--allow-live",
        "/lingtu_camera_color",
    ):
        assert forbidden not in player


def test_camera_index_is_lightweight_json_in_mcap_not_raw_image_payloads() -> None:
    implementation = _read(RECORDING / "src" / "camera_recording.cpp")
    assert "lingtu.camera.v1" in implementation
    assert '"jsonschema"' in implementation
    assert '"json"' in implementation
    schema = implementation.split("kCameraSchema =", 1)[1].split(')json";', 1)[0]
    assert '"payload"' not in schema
    assert '"lingtu.payload_location", "external-segment"' in implementation


def test_camera_depth_is_not_misrepresented_as_lossy_video() -> None:
    readme = _read(RECORDING / "README.md").lower()
    assert "depth" in readme
    assert "lossless" in readme
    assert "not part of the default product" in readme
