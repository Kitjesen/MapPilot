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
        '"--run-plan-fingerprint"',
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
    assert "--product-fingerprint" not in source
    assert "LINGTU_PRODUCT_FINGERPRINT" not in source
    assert 'environment_or("LINGTU_PRODUCT", "manual")' in source
    assert "LINGTU_PRODUCT_" + "PROFILE" not in source
    assert "run_plan_fingerprint" in manager


def test_root_control_and_catalog_are_native_owned() -> None:
    source = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")
    catalog = _read(RECORDING / "src" / "recording_catalog.cpp")
    gateway = _read(ROOT / "src" / "gateway" / "services" / "recording.py")

    for expected in (
        "RootControlLock",
        "start_root_recording",
        "print_root_status",
        "inspect_recording_catalog",
        '\\"control_version\\":1',
    ):
        assert expected in source
    assert "multiple_recordings_active" in catalog
    assert "recursive_directory_iterator" in catalog
    for forbidden in ("os.scandir", "os.walk", "session.json", "subprocess.Popen"):
        assert forbidden not in gateway


def test_sdk_uses_canonical_recording_api_and_keeps_bag_as_method_alias_only() -> None:
    for relative in ("src/lingtu/sdk/client.py", "src/lingtu/sdk/async_client.py"):
        source = _read(ROOT / relative)
        for endpoint in (
            "/api/v1/recordings/start",
            "/api/v1/recordings/status",
            "/api/v1/recordings/stop",
        ):
            assert endpoint in source
        assert "/api/v1/bag/" not in source

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
    assert "requires --run-plan-fingerprint" in source
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


def test_field_cli_record_start_is_a_short_native_adapter_with_safe_defaults() -> None:
    source = _read(ROOT / "scripts" / "lingtu")
    start = source.index("cmd_record() {")
    record = source[start : source.index("\n}\n", start) + 3]
    start = source.index("cmd_record_start() {")
    record_start = source[start : source.index("\n}\n", start) + 3]

    assert "lingtu record [SESSION_DIR] [--camera] [native options]" in source
    assert "Usage: lingtu record [start] [SESSION_DIR] [--camera] [native options]" in source
    assert 'start) shift; cmd_record_start "$@"' in record
    assert '*) cmd_record_start "$@"' in record
    assert 'LINGTU_RECORDING_ROOT:-$HOME/data/lingtu/recordings' in record_start
    assert 'manager_args+=("--dds" "on")' in record_start
    assert 'manager_args+=("--camera" "off")' in record_start
    assert 'forwarded+=("--camera" "on")' in record_start
    assert 'camera_value="${1#*=}"' in record_start
    assert 'forwarded+=("--camera" "$camera_value")' in record_start
    assert 'camera must be on or off' in record_start
    assert 'dds_value="${1#*=}"' in record_start
    assert 'forwarded+=("--dds" "$dds_value")' in record_start
    assert 'dds must be auto, on, or off' in record_start
    assert 'exec "$manager" "${manager_args[@]}" "${forwarded[@]}"' in record_start
    assert "ProductControl" not in record_start


def test_field_cli_status_and_stop_forward_to_the_session_manager() -> None:
    source = _read(ROOT / "scripts" / "lingtu")
    start = source.index("cmd_record() {")
    record = source[start : source.index("\n}\n", start) + 3]
    start = source.index("cmd_record_status() {")
    status = source[start : source.index("\n}\n", start) + 3]
    start = source.index("cmd_record_stop() {")
    stop = source[start : source.index("\n}\n", start) + 3]

    assert 'status) shift; cmd_record_status "$@"' in record
    assert 'stop) shift; cmd_record_stop "$@"' in record
    assert 'exec "$manager" "status" "$@"' in status
    assert 'exec "$manager" "stop" "$@"' in stop
    for command in (status, stop):
        assert 'lingtu_recording_binary lingtu_recorder' in command
        assert 'if [ "$#" -lt 1 ]' in command
        assert "ProductControl" not in command


def test_field_cli_resolves_recording_binaries_in_deployment_order() -> None:
    source = _read(ROOT / "scripts" / "lingtu")
    start = source.index("lingtu_recording_binary() {")
    resolver = source[start : source.index("\n}\n", start) + 3]

    probes = (
        "LINGTU_RECORDING_BIN_DIR",
        "/opt/lingtu/current/build/native-recording/$name",
        "$repo_root/build/native-recording/$name",
        'command -v "$name"',
    )
    offsets = [resolver.index(probe) for probe in probes]

    assert offsets == sorted(offsets)
    assert 'if [ -x "$candidate" ]' in resolver
    assert "bash scripts/build/build_native_recording.sh" in resolver


def test_field_cli_can_verify_inspect_and_list_native_recordings() -> None:
    source = _read(ROOT / "scripts" / "lingtu")
    start = source.index("cmd_record_verify() {")
    verify = source[start : source.index("\n}\n", start) + 3]
    start = source.index("cmd_record_info() {")
    info = source[start : source.index("\n}\n", start) + 3]
    start = source.index("cmd_record_topics() {")
    topics = source[start : source.index("\n}\n", start) + 3]

    for command in (verify, info):
        assert '"$target/dds/sensors.mcap"' in command
        assert '"$target/camera_color.mcap"' in command
    assert "lingtu_dds_player" in verify
    assert '"$dds_player" "$dds_mcap" --dry-run' in verify
    assert "lingtu_camera_player" in verify
    assert '"$camera_player" "$camera_mcap" --verify' in verify
    assert "lingtu_dds_player" in info
    assert '"$dds_player" --info "$dds_mcap"' in info
    assert "lingtu_camera_player" in info
    assert '"$camera_player" "$camera_mcap" --list' in info
    assert 'exec "$dds_player" --list-topics' in topics
    assert 'lingtu_recording_binary lingtu_dds_player' in topics

    start = source.index("cmd_record() {")
    record = source[start : source.index("\n}\n", start) + 3]
    assert 'verify) shift; cmd_record_verify "$@"' in record
    assert 'info) shift; cmd_record_info "$@"' in record
    assert 'topics) shift; cmd_record_topics "$@"' in record


def test_field_cli_replay_defaults_to_an_isolated_dds_domain() -> None:
    source = _read(ROOT / "scripts" / "lingtu")
    start = source.index("cmd_replay() {")
    replay = source[start : source.index("\n}\n", start) + 3]
    dispatch = source[source.index("# ── Top-level dispatch ──") :]

    assert "lingtu replay SESSION_DIR|MCAP [player options]" in source
    assert 'dds_mcap="$target/dds/sensors.mcap"' in replay
    assert 'lingtu_recording_binary lingtu_dds_player' in replay
    assert 'lingtu_has_option "--domain" "$@"' in replay
    assert 'replay_domain="${LINGTU_REPLAY_DOMAIN:-84}"' in replay
    assert 'player_args+=("--domain" "$replay_domain")' in replay
    assert 'exec "$dds_player" "$dds_mcap" "${player_args[@]}" "$@"' in replay
    assert 'replay|play) shift; cmd_replay "$@"' in dispatch
    assert 'record|rec|bag) shift; cmd_record "$@"' in dispatch
    for forbidden in ("ProductControl", "ros2 ", "rosbag"):
        assert forbidden not in replay


def test_native_recording_shortcuts_are_documented_for_field_operators() -> None:
    readme = _read(ROOT / "scripts" / "README.md")

    for expected in (
        "scripts/lingtu record",
        "scripts/lingtu record --camera",
        "scripts/lingtu record status <session-dir>",
        "scripts/lingtu record stop <session-dir>",
        "scripts/lingtu record info <session-dir-or-mcap>",
        "scripts/lingtu record topics",
        "scripts/lingtu record verify <session-dir-or-mcap>",
        "scripts/lingtu replay <session-dir-or-mcap>",
        "DDS domain `84`",
        "Ctrl-C",
    ):
        assert expected in readme
