from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
NAV_CPP = ROOT / "src/nav/cpp"
ENDPOINT = NAV_CPP / "endpoint"
NAV_ENDPOINT = ENDPOINT / "nav"
STATUS = NAV_ENDPOINT / "status"

CORE_HEADER = STATUS / "control_loop_health.hpp"
CORE_SOURCE = STATUS / "control_loop_health.cpp"
CORE_TEST = ROOT / "tests/nav/cpp/endpoint/test_control_loop_health.cpp"
ENDPOINT_BOOTSTRAP = NAV_ENDPOINT / "main.cpp"
ENDPOINT_LOOP = NAV_ENDPOINT / "runtime" / "loop.cpp"
PUBLISHER_HEADER = STATUS / "nav_status_publisher.hpp"
PUBLISHER_SOURCE = STATUS / "nav_status_publisher.cpp"
WRITER_HEADER = STATUS / "nav_status_writer.hpp"
WRITER_SOURCE = STATUS / "nav_status_writer.cpp"
PORTABLE_CMAKE = NAV_CPP / "CMakeLists.txt"
ENDPOINT_CMAKE = ENDPOINT / "CMakeLists.txt"
TEST_CMAKE = ROOT / "tests/nav/cpp/endpoint/CMakeLists.txt"
BUILD_SCRIPT = ROOT / "scripts/build/build_nav_endpoint.sh"
GATEWAY_RUNTIME_STATUS = ROOT / "src/gateway/services/runtime_status.py"
GATEWAY_STATUS_ROUTES = ROOT / "src/gateway/routes/status.py"


def _read(path: Path) -> str:
    assert path.exists(), f"expected file to exist: {path.relative_to(ROOT)}"
    return path.read_text(encoding="utf-8", errors="ignore")


def _block_after(source: str, start: str, end: str) -> str:
    assert start in source, f"missing block start: {start}"
    tail = source.split(start, 1)[1]
    assert end in tail, f"missing block end after {start}: {end}"
    return tail.split(end, 1)[0]


def test_declares_transport_free_control_loop_health_core_and_behavior_test() -> None:
    core = _read(CORE_HEADER) + "\n" + _read(CORE_SOURCE)
    behavior_test = _read(CORE_TEST)

    for marker in (
        "ControlLoopHealthConfig",
        "ControlLoopSample",
        "MetricDistribution",
        "ControlLoopHealthSnapshot",
        "ControlLoopHealth",
        "MetricDistribution loop_ms",
        "MetricDistribution work_ms",
        "MetricDistribution overrun_ms",
        "deadline_miss_ratio",
        "p95_utilization",
    ):
        assert marker in core

    for forbidden in (
        "DdsRuntime",
        "dds/dds.h",
        "nav_dds_runtime",
        "StatusSnapshotFileWriter",
        "Executor",
    ):
        assert forbidden not in core

    assert "ControlLoopHealth" in behavior_test
    assert "warming_up" in behavior_test
    assert "deadline_miss_ratio_high" in behavior_test
    assert "p95_utilization_high" in behavior_test


def test_navd_observes_each_completed_tick() -> None:
    bootstrap = _read(ENDPOINT_BOOTSTRAP)
    loop = _read(ENDPOINT_LOOP)

    assert '#include "status/control_loop_health.hpp"' in bootstrap
    assert re.search(r"ControlLoopHealth\s+control_loop_health\s*[({]", bootstrap)
    wiring = _block_after(bootstrap, "EndpointLoopContext loop_ctx{", "};")
    assert "control_loop_health" in wiring
    assert "return runEndpointLoop(loop_ctx, g_running);" in bootstrap

    observation = re.search(
        r"control_loop_health\.observe\s*\(\s*"
        r"ControlLoopSample\s*\{(?P<sample>.*?)\}\s*\)\s*;",
        loop,
        flags=re.DOTALL,
    )
    assert observation, "navd must observe one explicit completed-loop sample"
    sample = observation.group("sample")
    for field in ("timing.loop_ms", "timing.sleep_ms", "timing.overrun_ms"):
        assert field in sample

    loop_completed = loop.index("timing.loop_ms = elapsedMs(loop_start);")
    assert loop_completed < observation.start()


def test_status_publisher_samples_health_and_writer_emits_full_public_json() -> None:
    publisher_header = _read(PUBLISHER_HEADER)
    publisher_source = _read(PUBLISHER_SOURCE)
    writer_header = _read(WRITER_HEADER)
    writer_source = _read(WRITER_SOURCE)

    assert re.search(
        r"std::function\s*<\s*ControlLoopHealthSnapshot\s*\(\s*\)\s*>"
        r"\s+sample_loop_health",
        publisher_header,
    )
    sampled_at = publisher_source.index("actions_.sample_loop_health()")
    sample_line_start = publisher_source.rfind("\n", 0, sampled_at) + 1
    write_at = publisher_source.index("writeStatusSnapshot(", sampled_at)
    assert sampled_at < write_at
    assert "ControlLoopHealthSnapshot" in publisher_source[sample_line_start:write_at]
    assert "loop_health" in publisher_source[write_at:]

    assert '#include "status/control_loop_health.hpp"' in writer_header
    assert re.search(r"const\s+ControlLoopHealthSnapshot\s*&", writer_header)

    for json_key in (
        "control_loop_health",
        "ready",
        "healthy",
        "reason",
        "period_ms",
        "window_samples",
        "total_samples",
        "loop_ms",
        "work_ms",
        "overrun_ms",
        "deadline_misses",
        "deadline_miss_ratio",
        "current_miss_streak",
        "max_miss_streak",
        "p95_utilization",
        "max_utilization",
        "mean",
        "p50",
        "p95",
        "p99",
        "max",
    ):
        assert f'\\"{json_key}\\"' in writer_source


def test_both_cmake_surfaces_build_core_and_register_focused_test() -> None:
    portable = _read(PORTABLE_CMAKE)
    endpoint = _read(ENDPOINT_CMAKE)
    tests = _read(TEST_CMAKE)

    assert "LINGTU_NAV_TESTS_DIR" in portable
    assert "test_control_loop_health.cpp" in tests
    assert "control_loop_health.cpp" in tests
    assert re.search(r"add_test\s*\(\s*NAME\s+test_control_loop_health\b", tests)

    navd = _block_after(endpoint, "add_executable(navd", "target_link_libraries(navd")
    assert "status/control_loop_health.cpp" in navd


def test_linux_endpoint_build_gate_requires_control_loop_health_ctest() -> None:
    script = _read(BUILD_SCRIPT)
    assert "for required_test in" in script
    assert "test_control_loop_health" in script


def test_gateway_blocks_only_mature_unhealthy_loop_and_projects_metrics() -> None:
    runtime_status = _read(GATEWAY_RUNTIME_STATUS)
    routes = _read(GATEWAY_STATUS_ROUTES)
    assert '"native_control_loop_health_unavailable"' in runtime_status

    assert '"native_control_loop_unhealthy"' in runtime_status
    assert 'snapshot.get("control_loop_health")' in runtime_status
    assert 'control_loop_health.get("ready") is True' in runtime_status
    assert 'control_loop_health.get("healthy") is not True' in runtime_status
    assert '"control_loop_health": control_loop_health' in runtime_status

    metrics = _block_after(
        routes,
        '"/api/v1/metrics"',
        '"map": {',
    )
    assert "_native_nav_endpoint_status()" in metrics
    assert 'nav_endpoint.get("tick_hz")' in metrics
    assert 'nav_endpoint.get("control_loop_health")' in metrics
    assert '"navigation": {' in metrics
    assert '"control_loop_health": dict(control_loop_health)' in metrics
