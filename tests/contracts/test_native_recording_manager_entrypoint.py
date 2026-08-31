"""Contracts for the product-facing native recording supervisor."""

# ruff: noqa: D103, S101 - pytest contracts use assertions by design.

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RECORDING = ROOT / "src" / "native" / "recording"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_unified_recorder_is_a_ros_free_argv_supervisor() -> None:
    cmake = _read(RECORDING / "CMakeLists.txt")
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")
    process = _read(RECORDING / "src" / "recording_linux.cpp")

    assert "add_executable(lingtu_recorder" in cmake
    assert "RecordingManager" in source
    assert "PosixRecordingProcessFactory" in source
    assert "execvp" in process
    for forbidden in (
        "rclcpp",
        "rosbag",
        "ros2 ",
        "ament",
        "system(",
        "popen(",
        "bash -c",
    ):
        assert forbidden not in (source + process).lower()


def test_unified_recorder_owns_one_session_not_the_product() -> None:
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")
    manager = _read(RECORDING / "src" / "recording_manager.cpp")

    for expected in (
        'command == "start"',
        'command == "record"',
        'command == "status"',
        'command == "stop"',
        '"--root"',
        '"--prefix"',
        '"--output-dir"',
        '"--product-session-id"',
        '"--robot-id"',
        '"--dds"',
        '"--camera"',
    ):
        assert expected in source
    assert "ProductControl" not in source + manager
    assert "manager_pid" in manager
    assert '"session.json.tmp"' in manager
    assert "RecordingContext" in _read(
        RECORDING / "include" / "lingtu" / "recording" / "recording_manager.hpp"
    )
    assert 'environment_or("LINGTU_PRODUCT", "manual")' in source
    assert "LINGTU_PRODUCT_" + "PROFILE" not in source
    assert "product_session_id" in manager


def test_root_control_and_catalog_are_native_owned() -> None:
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")
    catalog = _read(RECORDING / "src" / "recording_catalog.cpp")
    gateway = _read(ROOT / "src" / "gateway" / "services" / "recording.py")

    for expected in (
        "RootControlLock",
        "start_root_recording",
        "print_root_status",
        "print_catalog_list",
        "print_root_manifest",
        "remove_root_session",
        "inspect_recording_catalog",
        '\\"control_version\\":1',
    ):
        assert expected in source
    assert "multiple_recordings_active" in catalog
    assert "recursive_directory_iterator" in catalog
    for forbidden in ("os.scandir", "os.walk", "session.json", "subprocess.Popen"):
        assert forbidden not in gateway


def test_recording_file_management_is_native_catalog_bound_and_path_scoped() -> None:
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")
    catalog_header = _read(RECORDING / "include" / "lingtu" / "recording" / "recording_catalog.hpp")
    routes = _read(ROOT / "src" / "gateway" / "routes" / "recordings.py")
    files = _read(ROOT / "src" / "gateway" / "services" / "recording_files.py")

    for command in ('command == "list"', 'command == "manifest"', 'command == "remove"'):
        assert command in source
    assert "std::vector<RecordingManifestSnapshot> sessions" in catalog_header
    assert '"/api/v1/recordings"' in routes
    assert '"/api/v1/recordings/{session_id}/files/{artifact_path:path}"' in routes
    assert '"/api/v1/recordings/{session_id}"' in routes
    assert "resolve_declared_artifact" in routes
    assert "relative_to(session_root)" in files
    assert "S_ISLNK" in files


def test_sdk_uses_canonical_recording_api_and_keeps_bag_as_method_alias_only() -> None:
    source = _read(ROOT / "src/lingtu/sdk/client.py")
    for endpoint in (
        "/api/v1/recordings/start",
        "/api/v1/recordings/status",
        "/api/v1/recordings/stop",
    ):
        assert endpoint in source
    assert "/api/v1/bag/" not in source

    async_source = _read(ROOT / "src/lingtu/sdk/async_client.py")
    for method in ("recording_start", "recording_status", "recording_stop"):
        assert f"self._client.{method}" in async_source

    assert not (ROOT / "web" / "src" / "components" / "BagRecorder.tsx").exists()
    assert not (ROOT / "web" / "src" / "components" / "BagRecorder.module.css").exists()


def test_status_and_stop_verify_the_live_manager_identity() -> None:
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")

    for expected in (
        "manager_identity_error",
        'std::filesystem::canonical("/proc/self/exe"',
        'arguments[1] != "record"',
        'arguments[index] == "--output-dir"',
        "::kill(static_cast<pid_t>(snapshot.manager_process_id), SIGTERM)",
        "read_manifest(session_directory)",
    ):
        assert expected in source
    assert "stop_recording(" in source
    assert "kill(-process_id" not in source


def test_dds_off_build_keeps_camera_and_unified_manager_available() -> None:
    cmake = _read(RECORDING / "CMakeLists.txt")
    build_script = _read(ROOT / "scripts" / "build" / "build_native_recording.sh")
    assert '(cd "$BUILD_DIR" && ctest --output-on-failure)' in build_script
    assert "ctest --test-dir" not in build_script

    linux_manager_block = cmake.split("add_executable(lingtu_recorder", 1)[1]
    assert "target_link_libraries(lingtu_recorder" in linux_manager_block
    assert "lingtu_recording_process_linux" in linux_manager_block
    assert "LINGTU_RECORDING_HAS_DDS=0" in linux_manager_block
    assert '"$BUILD_DIR/lingtu_recorder"' in build_script


def test_product_context_does_not_compile_a_second_recording_policy() -> None:
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")

    assert '"--dds-preset"' in source
    assert "dds_recording_plan(options.dds_preset, options.dds_topics)" in source
    assert "dds_recording_plan(options.product)" not in source
    assert "selected_topics" in source
    compact_source = " ".join(source.split())
    assert (
        "options.dds_topics.empty() ? topic_plan.required_topics : std::vector<std::string>{}"
        not in compact_source
    )
    assert '"--inspection-task-id"' in source
    assert "requires --product inspection" in source
    assert "requires --product-session-id" in source
    assert "task-bound inspection recording requires selected topic" in source
    assert "required_topics.emplace_back(topic)" in source


def test_manager_manifest_is_atomic_and_fails_closed_on_artifacts() -> None:
    manager = _read(RECORDING / "src" / "recording_manager.cpp")

    assert '"session.json.tmp"' in manager
    assert "fdatasync" in manager
    assert "sync_directory" in manager
    assert "is_regular_file" in manager
    assert "file_size" in manager
    assert "required recording artifact is empty" in manager


def test_native_recorder_storage_admission_is_visible_and_fail_fast() -> None:
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")
    manager = _read(RECORDING / "src" / "recording_manager.cpp")
    manager = manager.replace("\\\"", '"')
    header = _read(
        RECORDING / "include" / "lingtu" / "recording" / "recording_manager.hpp"
    )

    for expected in (
        '"--min-free-gib"',
        "minimum_free_bytes{5 * kBytesPerGib}",
        "parse_gibibytes",
        "spec.minimum_free_bytes = options.minimum_free_bytes",
        '"\\nminimum_free_bytes="',
    ):
        assert expected in source

    assert "std::uint64_t minimum_free_bytes{0}" in header
    assert "available_bytes_at_start" in header
    assert "std::filesystem::space" in manager
    assert "insufficient recording storage: required=" in manager
    assert '\"storage\":{\"minimum_free_bytes\":' in manager
    assert manager.index("std::filesystem::space") < manager.index(
        "std::filesystem::create_directory(spec.session_directory)"
    )
