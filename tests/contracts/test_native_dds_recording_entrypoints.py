"""Contracts for the native CycloneDDS recording product entrypoints."""

# ruff: noqa: D103, S101 - pytest contracts use assertions by design.

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RECORDING = ROOT / "src" / "native" / "recording"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_native_recording_build_entrypoint_is_ros_free() -> None:
    text = _read(ROOT / "scripts" / "build" / "build_native_recording.sh").lower()
    for forbidden in ("source /opt/ros", "ros2 ", "ament", "colcon", "rclcpp"):
        assert forbidden not in text
    assert "cyclonedds" in text
    assert "idlc" in text


def test_dds_recorder_links_its_writer_thread_runtime() -> None:
    cmake = _read(RECORDING / "CMakeLists.txt")
    recorder_block = cmake.split("add_executable(lingtu_dds_recorder", 1)[1].split(
        "add_executable(lingtu_dds_player", 1
    )[0]

    assert "target_link_libraries(lingtu_dds_recorder PRIVATE" in recorder_block
    assert "Threads::Threads" in recorder_block


def test_player_defaults_to_an_isolated_domain_and_requires_live_opt_in() -> None:
    text = _read(RECORDING / "src" / "player_main.cpp")
    assert "int domain_id{84}" in text
    assert "--allow-live-domain" in text
    assert "validate_replay_domain" in text
    assert "--domain 0" not in _read(RECORDING / "README.md")


def test_replay_allowlist_excludes_motion_and_control_topics() -> None:
    text = _read(RECORDING / "src" / "recording_core.cpp")
    allowed_block = text.split("kAllowed[]", 1)[1].split("};", 1)[0]
    assert '"/imu/raw"' in allowed_block
    assert '"/slam/registered_cloud"' in allowed_block
    for forbidden in (
        "/nav/cmd_vel",
        "/nav/command/request",
        "/nav/operator_motion/control",
    ):
        assert forbidden not in allowed_block


def test_inspection_timeline_is_recordable_but_never_sensor_replayable() -> None:
    catalog = _read(RECORDING / "src" / "topic_catalog.cpp")
    recorder = _read(RECORDING / "src" / "recorder_main.cpp")
    player = _read(RECORDING / "src" / "player_main.cpp")

    for contract in (
        "kDriverControlState",
        "kNavGoalStatus",
        "kNavState",
        "kOperatorMotionStatus",
        "kNavGlobalPath",
        "kNavLocalPath",
        "kNavCmdVel",
        "kNavInspectionTaskEvent",
        "kNavInspectionEvidenceResult",
    ):
        assert contract in catalog
    assert "find_recording_topic" in recorder
    assert "find_sensor_topic" in player
    assert "find_recording_topic(view.channel->topic)" in player
    assert "skipped_record_only" in player
    assert "--require-topic" in recorder
    assert "required recording topic captured no samples" in recorder
    assert "--inspection-task-id" in recorder
    assert "InspectionTimelineCapture" in recorder
    assert "timeline_capture.verify(options.inspection_task_id)" in recorder
    assert "storage.commit()" in recorder


def test_record_only_envelopes_are_checked_offline_and_terminal_is_evidence_gated() -> None:
    player = _read(RECORDING / "src" / "player_main.cpp")
    cyclone = _read(RECORDING / "src" / "cyclone_cdr.cpp")
    capture = _read(RECORDING / "src" / "inspection_timeline_cdr.cpp")
    verifier = _read(RECORDING / "src" / "inspection_timeline.cpp")

    assert "validate_cdr_payload" in player
    assert "forward_cdr_message" in player
    assert "dds_forwardcdr" in cyclone
    assert "dds_stream_countops" not in cyclone
    assert "dds_stream_read_sample" not in cyclone
    assert "decode_cdr_sample" not in player
    assert "Xcdr1LittleEndianReader" in capture
    assert "kNavInspectionTaskEvent" in capture
    assert "kNavCmdVel" in capture
    assert "kDriverControlState" in capture
    assert "last final cmd_vel before terminal is not zero" in verifier
    assert "driver did not confirm the final zero-output sequence" in verifier
    assert '"--verify-inspection-task"' in player
    assert '"--verify-inspection-task requires --dry-run"' in player
    assert "timeline_capture.observe" in player
    assert "timeline_capture.verify(options.inspection_task_id)" in player


def test_native_format_is_raw_cdr_with_embedded_omg_idl() -> None:
    text = _read(RECORDING / "src" / "mcap_session.cpp")
    assert 'McapWriterOptions options(kMcapProfile)' in text
    assert 'mcap::Compression::None' in text
    assert '"omgidl"' in text
    assert '"cdr"' in text
    assert '"lingtu.idl_type"' in text
    assert 'temporary_path(final_path.string() + ".tmp")' in text
    assert "sync_parent_directory(final_path)" in text


def test_vendored_mcap_is_pinned_and_compression_is_explicitly_disabled() -> None:
    notice = _read(RECORDING / "vendor" / "mcap" / "README.lingtu.md")
    cmake = _read(RECORDING / "CMakeLists.txt")
    assert "releases/cpp/v2.1.3" in notice
    assert "1420296ffcfdcde4b6894c0c1aba0ad083f93dde" in notice
    assert "MCAP_COMPRESSION_NO_LZ4" in cmake
    assert "MCAP_COMPRESSION_NO_ZSTD" in cmake


def test_player_introspection_modes_are_offline_and_do_not_require_a_file_for_listing() -> None:
    player = _read(RECORDING / "src" / "player_main.cpp")
    main = player.split("int main", 1)[1]
    info = player.split("void print_info", 1)[1].split("dds_entity_t checked", 1)[0]

    assert "lingtu_dds_player --list-topics" in player
    assert "lingtu_dds_player --info FILE" in player
    assert "options.input.empty() && !options.list_topics" in player
    assert main.index("if (options.list_topics)") < main.index("resolve_recording_idl")
    assert main.index("if (options.info)") < main.index("resolve_recording_idl")
    assert "dds_create_participant" not in info


def test_replay_writer_disables_lifespan_for_historical_source_timestamps() -> None:
    player = _read(RECORDING / "src" / "player_main.cpp")
    recorder = _read(RECORDING / "src" / "recorder_main.cpp")

    make_qos = player.index("make_qos(sensor_binding->qos_profile)")
    disable_lifespan = player.index("dds_qset_lifespan(qos.get(), DDS_INFINITY)")
    create_writer = player.index("dds_create_writer", disable_lifespan)

    assert make_qos < disable_lifespan < create_writer
    assert "dds_qset_lifespan" not in recorder


def test_recording_idl_resolution_is_relocatable_and_used_by_all_dds_entrypoints() -> None:
    core = _read(RECORDING / "src" / "recording_core.cpp")
    player = _read(RECORDING / "src" / "player_main.cpp")
    recorder = _read(RECORDING / "src" / "recorder_main.cpp")
    supervisor = _read(RECORDING / "src" / "lingtu_recorder_main.cpp")

    environment_idl = core.index('std::getenv("LINGTU_RECORDING_IDL")')
    repository_root = core.index('std::getenv("LINGTU_REPO")')
    executable_relative = core.index('executable_path.parent_path() / ".." / ".." / "src"')
    compile_fallback = core.index("candidates.push_back(compile_time_fallback)")
    assert environment_idl < repository_root < executable_relative < compile_fallback

    for source in (player, recorder, supervisor):
        assert "resolve_recording_idl" in source
        assert "LINGTU_RECORDING_DEFAULT_IDL" in source

    assert "options.idl_path.empty()" in player
    assert "options.idl_path.empty()" in recorder
    assert "options.dds_idl.empty()" in supervisor
