import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NAV_CPP = ROOT / "src" / "nav" / "cpp"
ENDPOINT = NAV_CPP / "endpoint"
NAV_ENDPOINT = ENDPOINT / "nav"
CONTROL = NAV_ENDPOINT / "control"

ENDPOINT_BOOTSTRAP = NAV_ENDPOINT / "main.cpp"
ENDPOINT_LOOP = NAV_ENDPOINT / "runtime" / "loop.cpp"
GUARD_HEADER = CONTROL / "guard.hpp"
GUARD_SOURCE = CONTROL / "guard.cpp"
GUARD_TEST = NAV_CPP / "tests" / "endpoint" / "test_control_loop_runtime_guard.cpp"
PORTABLE_CMAKE = NAV_CPP / "CMakeLists.txt"
ENDPOINT_CMAKE = ENDPOINT / "CMakeLists.txt"
BUILD_SCRIPT = ROOT / "scripts" / "build" / "build_nav_endpoint.sh"


def _read(path: Path) -> str:
    assert path.exists(), f"expected file to exist: {path.relative_to(ROOT)}"
    return path.read_text(encoding="utf-8", errors="ignore")


def _block_after(source: str, start: str, end: str) -> str:
    assert start in source, f"missing block start: {start}"
    tail = source.split(start, 1)[1]
    assert end in tail, f"missing block end after {start}: {end}"
    return tail.split(end, 1)[0]


def _lambda_block(source: str, marker: str, end_marker: str = "};") -> str:
    assert marker in source, f"missing lambda marker: {marker}"
    tail = source.split(marker, 1)[1]
    assert end_marker in tail, f"missing lambda end after: {marker}"
    return tail.split(end_marker, 1)[0]


def test_declares_transport_free_runtime_guard_core_and_behavior_test() -> None:
    core = _read(GUARD_HEADER) + "\n" + _read(GUARD_SOURCE)
    behavior_test = _read(GUARD_TEST)

    for marker in (
        "ControlLoopRuntimeGuardConfig",
        "ControlLoopRuntimeGuardDecision",
        "ControlLoopRuntimeGuard",
        "observe(",
        "requestResume(",
        "completeResume(",
        "control_loop_unhealthy",
    ):
        assert marker in core

    for forbidden in (
        "DdsRuntime",
        "dds/dds.h",
        "nav_dds_runtime",
        "MotionStopBarrier",
        "ControlAuthority",
        "Executor",
    ):
        assert forbidden not in core

    for behavior in (
        "testWarmupSnapshotDoesNotTripGuard",
        "testSecondConsecutiveMatureUnhealthySnapshotTripsGuard",
        "testFailedResumeCompletionKeepsGuardLatched",
        "testResumeIsBlockedBeforeRecoveryConfirmation",
    ):
        assert behavior in behavior_test


def test_cmake_and_linux_build_gate_require_runtime_guard_test() -> None:
    portable = _read(PORTABLE_CMAKE)
    endpoint = _read(ENDPOINT_CMAKE)
    script = _read(BUILD_SCRIPT)

    for cmake in (portable, endpoint):
        assert "test_control_loop_runtime_guard.cpp" in cmake
        assert "control/guard.cpp" in cmake
        assert re.search(r"add_test\s*\(\s*NAME\s+test_control_loop_runtime_guard\b", cmake)

    navd_target = _block_after(endpoint, "add_executable(navd", "target_link_libraries(navd")
    assert "control/guard.cpp" in navd_target
    assert "test_control_loop_runtime_guard" in script


def test_navd_evaluates_completed_health_before_motion_tick_work() -> None:
    bootstrap = _read(ENDPOINT_BOOTSTRAP)
    loop_source = _read(ENDPOINT_LOOP)
    loop = _block_after(loop_source, "while (running) {", "const auto input_start")

    assert '#include "control/guard.hpp"' in bootstrap
    assert re.search(r"ControlLoopRuntimeGuard\s+control_loop_guard\s*[({]", bootstrap)
    wiring = _block_after(bootstrap, "EndpointLoopContext loop_ctx{", "};")
    assert "control_loop_health" in wiring
    assert "control_loop_guard" in wiring
    assert "return runEndpointLoop(loop_ctx, g_running);" in bootstrap
    assert re.search(
        r"const\s+auto\s+loop_guard_decision\s*=\s*control_loop_guard\.observe",
        loop,
    )
    assert "control_loop_health.snapshot()" in loop
    assert loop.index("control_loop_guard.observe") < loop.index("RollingSegmentBeginTick")


def test_hold_decision_latches_takeover_and_clears_motion_before_autonomy_work() -> None:
    loop_source = _read(ENDPOINT_LOOP)
    loop = _block_after(loop_source, "while (running) {", "const auto input_start")
    hold_block = _block_after(
        loop,
        "if (loop_guard_decision.hold_motion)",
        "(void)rolling_segment_effect_coordinator.apply",
    )

    assert "control_authority.holdOperatorTakeover()" in hold_block
    assert "operator_resume_required = true" in hold_block
    assert '"control_loop_unhealthy:"' in loop
    assert re.search(
        r"hold_reason\s*=.*loop_guard_decision\.reason",
        loop,
        re.DOTALL,
    )
    assert re.search(
        r"motion_stop\.clearEndpointMotion\s*\(\s*hold_reason\s*\)",
        hold_block,
    )
    assert hold_block.index("control_authority.holdOperatorTakeover") < hold_block.index(
        "operator_resume_required = true"
    )
    assert hold_block.index("operator_resume_required = true") < hold_block.index("motion_stop.clearEndpointMotion")


def test_latched_guard_ticks_keep_zero_fresh_without_reentering_autonomy() -> None:
    loop_source = _read(ENDPOINT_LOOP)
    loop = _block_after(loop_source, "while (running) {", "const auto before_sleep")
    hold_block = _block_after(
        loop,
        "if (loop_guard_decision.hold_motion)",
        "(void)rolling_segment_effect_coordinator.apply",
    )

    assert "else if (!motion_stop.keepZeroFresh())" in hold_block
    assert "autonomy_tick.tick" not in hold_block
    assert "teleop_tick.tick" not in hold_block


def test_every_motion_admission_checks_runtime_guard_latch() -> None:
    bootstrap = _read(ENDPOINT_BOOTSTRAP)
    loop = _read(ENDPOINT_LOOP)
    submit_goal = _block_after(loop, "auto submit_goal = [&]", "auto task_resume_context")
    handle_teleop = _block_after(
        loop,
        "auto handle_teleop = [&]",
        "auto complete_operator_zero_barrier",
    )
    inspection_actions = _block_after(
        bootstrap,
        "InspectionCommandActions inspection_command_actions;",
        "InspectionCommandCoordinator inspection_command_coordinator",
    )
    rolling_context = _block_after(loop, "auto rolling_segment_context = [&]()", "};")
    advance = _block_after(
        loop,
        "GoalReplanRuntimeFrameInput runtime_frame;",
        "const NavigationRuntimeFrameResult runtime_frame_result",
    )
    autonomy = _block_after(
        loop,
        "runtime_actions.run_autonomy = [&]",
        "runtime_actions.apply_autonomy_outputs = [&]",
    )

    guarded_blocks = {
        "goal admission": submit_goal,
        "teleop admission": handle_teleop,
        "inspection command admission actions": inspection_actions,
        "goal advance admission": advance,
        "autonomy tick admission": autonomy,
    }
    for name, block in guarded_blocks.items():
        assert "control_loop_guard" in block, f"{name} must consult the runtime guard"
        assert re.search(
            r"hold_motion|control_hold|resume_allowed|motionAllowed|operatorTakeoverLatched|requestResume",
            block,
        ), f"{name} must turn the guard state into an admission decision"

    assert "drain" + "Legacy" not in loop
    assert "control_loop_guard" in rolling_context
    assert re.search(
        r"hold_motion|resume_allowed|motionAllowed|operatorTakeoverLatched|requestResume",
        rolling_context,
    )


def test_resume_command_passes_through_guard_before_motion_stop_resume() -> None:
    loop = _read(ENDPOINT_LOOP)
    input_gate_helper = _block_after(
        loop,
        "auto evaluate_input_gate =",
        "const auto tick_period =",
    )
    resume_block = _block_after(loop, "auto handle_resume_motion =", "auto handle_teleop =")

    assert "inputs.evaluateGate" in input_gate_helper
    assert "control_loop_guard.requestResume" in resume_block
    assert "motion_stop.resumeAutonomy" in resume_block
    assert "motion_stop.resumeTeleop" in resume_block
    assert resume_block.count("evaluate_input_gate()") >= 2
    assert "control_loop_guard.completeResume" in resume_block
    assert resume_block.index("control_loop_guard.requestResume") < resume_block.index("motion_stop.resumeAutonomy")
    assert resume_block.index("motion_stop.resumeAutonomy") < resume_block.index("control_loop_guard.completeResume")
    assert resume_block.index("evaluate_input_gate()") < resume_block.index("motion_stop.resumeTeleop")
    assert resume_block.rindex("evaluate_input_gate()") > resume_block.index("motion_stop.resumeTeleop")
    assert resume_block.index("motion_stop.resumeTeleop") < resume_block.index("control_loop_guard.completeResume")
    assert re.search(r"completeResume\s*\(\s*result\.accepted", resume_block)


def test_operator_resume_required_clears_only_after_accepted_guard_resume() -> None:
    bootstrap = _read(ENDPOINT_BOOTSTRAP)
    loop = _read(ENDPOINT_LOOP)
    motion_actions = _block_after(
        bootstrap,
        "MotionStopActions motion_stop_actions;",
        "MotionStopBarrier motion_stop(",
    )
    resume_block = _block_after(loop, "auto handle_resume_motion =", "auto handle_teleop =")

    assert "motion_stop_actions.clear_operator_resume_required" in motion_actions
    assert "control_loop_guard.completeResume" in resume_block
    assert re.search(
        r"const\s+auto\s+resume_result\s*=\s*control_loop_guard\.completeResume\s*\(\s*result\.accepted\s*\)",
        resume_block,
    )
    assert re.search(
        r"if\s*\(\s*resume_result\.resume_completed\s*\).*operator_resume_required\s*=\s*false",
        resume_block,
        re.DOTALL,
    )


def test_twenty_hz_period_and_tail_observe_stay_unchanged() -> None:
    bootstrap = _read(ENDPOINT_BOOTSTRAP)
    loop = _read(ENDPOINT_LOOP)

    assert "control_loop_health_config.period_ms = 1000.0 / std::max(1.0, cfg.tick_hz)" in bootstrap
    assert "tick_hz" in bootstrap

    loop_tail = _block_after(
        loop,
        "const auto before_sleep = SteadyClock::now();",
        "last_timing = timing;",
    )
    assert "timing.loop_ms = elapsedMs(loop_start);" in loop_tail
    assert "control_loop_health.observe" in loop_tail
    assert loop_tail.index("timing.loop_ms = elapsedMs(loop_start);") < loop_tail.index("control_loop_health.observe")
    sample = _block_after(loop_tail, "ControlLoopSample{", "})")
    for field in ("timing.loop_ms", "timing.sleep_ms", "timing.overrun_ms"):
        assert field in sample
