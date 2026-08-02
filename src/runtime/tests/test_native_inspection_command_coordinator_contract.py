from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
ENDPOINT_DIR = ROOT / "src" / "nav" / "cpp" / "endpoint"
INSPECTION_DIR = ENDPOINT_DIR / "inspection"
COORDINATOR_HEADER = INSPECTION_DIR / "inspection_command_coordinator.hpp"
COORDINATOR_SOURCE = INSPECTION_DIR / "inspection_command_coordinator.cpp"
COORDINATOR_TEST = (
    ROOT
    / "src"
    / "nav"
    / "cpp"
    / "tests"
    / "endpoint"
    / "test_inspection_command_coordinator.cpp"
)
ENDPOINT = ENDPOINT_DIR / "nav_native_endpoint.cpp"
ENDPOINT_LOOP = ENDPOINT_DIR / "endpoint_loop.cpp"
DDS_HEADER = ENDPOINT_DIR / "nav_dds_runtime.hpp"
DDS_SOURCE = ENDPOINT_DIR / "nav_dds_runtime.cpp"
CLIENT_DIR = ROOT / "src" / "nav" / "cpp" / "client"
CLIENT_HEADER = CLIENT_DIR / "client.hpp"
CLIENT_SOURCE = CLIENT_DIR / "client.cpp"
CLIENT_C_HEADER = CLIENT_DIR / "client_c.h"
CLIENT_C_SOURCE = CLIENT_DIR / "client_c.cpp"
ENDPOINT_CMAKE = ENDPOINT_DIR / "CMakeLists.txt"
NAV_CPP_CMAKE = ROOT / "src" / "nav" / "cpp" / "CMakeLists.txt"
BUILD_SCRIPT = ROOT / "scripts" / "build" / "build_nav_endpoint.sh"
ENDPOINT_README = ENDPOINT_DIR / "README.md"


def _read(path: Path) -> str:
    assert path.exists(), f"expected file to exist: {path.relative_to(ROOT)}"
    return path.read_text(encoding="utf-8", errors="ignore")


def _block_after(source: str, start: str, end: str) -> str:
    assert start in source, f"missing block start: {start}"
    tail = source.split(start, 1)[1]
    assert end in tail, f"missing block end after {start}: {end}"
    return tail.split(end, 1)[0]


def test_declares_transport_free_inspection_command_coordinator_files() -> None:
    assert COORDINATOR_HEADER.exists()
    assert COORDINATOR_SOURCE.exists()


def test_adds_focused_cpp_test_for_inspection_command_coordinator() -> None:
    test_source = _read(COORDINATOR_TEST)
    assert "InspectionCommandCoordinator" in test_source


def test_dds_runtime_reports_inspection_task_ack_write_success() -> None:
    header = _read(DDS_HEADER)
    source = _read(DDS_SOURCE)

    assert re.search(
        r"bool\s+writeInspectionTaskAck\s*\(\s*const char\s*\*\s*task_id",
        header,
    )
    assert re.search(
        r"bool\s+DdsRuntime::writeInspectionTaskAck\s*\(",
        source,
    )
    ack_body = _block_after(
        source,
        "DdsRuntime::writeInspectionTaskAck(",
        "void DdsRuntime::writeInspectionStatus",
    )
    assert "const dds_return_t result = dds_write(inspection_task_ack_writer_, &msg)" in ack_body
    assert 'logDdsError(result, "dds_write(inspection_task_ack)")' in ack_body
    assert "return result >= 0;" in ack_body


def test_endpoint_cmake_builds_coordinator_into_navd() -> None:
    cmake = _read(ENDPOINT_CMAKE)
    navd_target = _block_after(cmake, "add_executable(navd", "target_link_libraries(navd")

    assert "inspection/inspection_command_coordinator.cpp" in navd_target


def test_portable_cmake_registers_command_coordinator_ctest() -> None:
    cmake = _read(NAV_CPP_CMAKE)

    assert "test_inspection_command_coordinator.cpp" in cmake
    assert "inspection/inspection_command_coordinator.cpp" in cmake
    assert re.search(r"add_test\s*\(\s*NAME\s+test_inspection_command_coordinator\b", cmake)


def test_linux_endpoint_build_script_requires_command_coordinator_ctest() -> None:
    script = _read(BUILD_SCRIPT)

    assert "test_inspection_command_coordinator" in script


def test_native_endpoint_only_forwards_inspection_command_requests_to_coordinator() -> None:
    endpoint = _read(ENDPOINT)
    endpoint_loop = _read(ENDPOINT_LOOP)
    command_block = _block_after(
        endpoint_loop,
        "dds.drainInspectionTaskRequests(",
        "input_gate_state =",
    )

    assert '#include "inspection/inspection_command_coordinator.hpp"' in endpoint
    assert "InspectionCommandCoordinator inspection_command_coordinator" in endpoint
    assert "inspection_command_coordinator.handle" in command_block
    assert "dds.writeInspectionTaskAck" in endpoint
    assert "inspection_runtime.requestStatus()" in endpoint
    assert "dds.writeInspectionTaskAck" not in command_block
    assert "inspection_runtime.requestStatus()" not in command_block

    forbidden_command_ownership = (
        "InspectionAckRecord",
        "inspection_ack_cache",
        "remember_inspection_ack",
        "inspection_executor.Start",
        "inspection_executor.Pause",
        "inspection_executor.Resume",
        "inspection_executor.Cancel",
        "inspection_store->Get",
        "active_map_identity()",
        "control_authority.operatorTakeoverLatched()",
        "motion_stop.clearEndpointMotion",
    )
    for token in forbidden_command_ownership:
        assert token not in command_block


def test_native_endpoint_no_longer_owns_inspection_ack_cache_or_admission() -> None:
    endpoint = _read(ENDPOINT)

    forbidden_tokens = (
        "struct InspectionAckRecord",
        "inspection_ack_order",
        "inspection_ack_cache",
        "remember_inspection_ack",
        "inspection_request_id_empty",
        "unknown_inspection_command",
        "inspection_route_not_found",
        "inspection_route_revision_mismatch",
        "inspection_pause_zero_publish_failed",
        "inspection_cancel_zero_publish_failed",
    )
    for token in forbidden_tokens:
        assert token not in endpoint


def test_command_coordinator_stays_transport_free() -> None:
    combined = _read(COORDINATOR_HEADER) + "\n" + _read(COORDINATOR_SOURCE)
    forbidden_tokens = (
        "DdsRuntime",
        "dds_",
        "dds/dds.h",
        "lingtu_slam.h",
        "lingtu_dds_",
        "nav_dds_runtime",
        "NavLoop",
    )
    for token in forbidden_tokens:
        assert token not in combined


def test_command_coordinator_owns_resume_autonomy_and_zero_downgrade_policy() -> None:
    source = _read(COORDINATOR_SOURCE)
    resume_block = source.split("CommandKind::kResume", 1)[1].split("CommandKind::kCancel", 1)[0]
    pause_block = source.split("CommandKind::kPause", 1)[1].split("CommandKind::kResume", 1)[0]
    cancel_block = source.split("CommandKind::kCancel", 1)[1]

    assert "actions_.operator_takeover_latched()" in resume_block
    assert "inspection_resume_requires_autonomy" in resume_block
    assert resume_block.index("operator_takeover_latched") < resume_block.index("executor_.Resume")

    assert "executor_.RequestPause" in pause_block
    assert "actions_.stop_and_commit" in pause_block
    assert pause_block.index("executor_.RequestPause") < pause_block.index("actions_.stop_and_commit")

    assert "executor_.RequestCancel" in cancel_block
    assert "actions_.stop_and_commit" in cancel_block
    assert cancel_block.index("executor_.RequestCancel") < cancel_block.index("actions_.stop_and_commit")


def test_taskless_inspection_client_and_endpoint_wire_are_removed() -> None:
    client_header = _read(CLIENT_HEADER)
    client_source = _read(CLIENT_SOURCE)
    c_api = _read(CLIENT_C_HEADER) + "\n" + _read(CLIENT_C_SOURCE)
    dds_runtime = _read(DDS_HEADER) + "\n" + _read(DDS_SOURCE)
    endpoint_loop = _read(ENDPOINT_LOOP)
    coordinator = _read(COORDINATOR_HEADER) + "\n" + _read(COORDINATOR_SOURCE)

    for token in (
        "InspectionCommands::start(",
        "InspectionCommands::pause(",
        "InspectionCommands::resume(",
        "InspectionCommands::cancel(",
        "writeInspectionCommand(",
        "lingtu_dds_InspectionCommandRequest",
        "lingtu_dds_InspectionCommandAck",
        "drainInspectionCommands",
        "writeInspectionAck",
        "InspectionCommandWireVersion",
    ):
        assert token not in client_header + client_source + dds_runtime + endpoint_loop + coordinator

    for symbol in (
        "lingtu_nav_client_start_inspection(",
        "lingtu_nav_client_pause_inspection(",
        "lingtu_nav_client_resume_inspection(",
        "lingtu_nav_client_cancel_inspection(",
    ):
        assert symbol not in c_api

    assert "startTask(" in client_header
    assert "drainInspectionTaskRequests" in dds_runtime
    assert "writeInspectionTaskAck" in dds_runtime


def test_endpoint_readme_documents_completed_command_coordinator_split() -> None:
    readme = _read(ENDPOINT_README)

    assert "InspectionCommandCoordinator" in readme
    assert "inspection command" in readme
    assert "completed" in readme.lower()
