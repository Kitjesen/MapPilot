from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NAV = ROOT / "src/nav/cpp/endpoint/nav"


def test_performance_metrics_do_not_own_motion() -> None:
    bootstrap = (NAV / "main.cpp").read_text(encoding="utf-8")
    loop = (NAV / "app/loop.cpp").read_text(encoding="utf-8")
    assert "control_loop_guard" not in bootstrap + loop
    assert "control_loop_health.observe" in loop
    assert "inputs.evaluateGate" in loop
    assert "motion_stop.resumeAutonomy(request)" in loop
    assert "motion_stop.resumeTeleop(request)" in loop


def test_driver_status_does_not_interrupt_navigation() -> None:
    loop = (NAV / "app/loop.cpp").read_text(encoding="utf-8")
    assert "driver_authority_previous" not in loop
    assert "GoalReplanRuntimeInterruption::kDriverAuthorityLost" not in loop


def test_resume_does_not_latch_temporary_input_gaps() -> None:
    loop = (NAV / "app/loop.cpp").read_text(encoding="utf-8")
    resume = loop.split("auto handle_resume_motion =", 1)[1].split("auto handle_teleop =", 1)[0]
    assert "evaluate_input_gate" not in resume
    assert "hold(" not in resume
    assert '"estop_latched"' in resume
    assert "deferred_control_ack && stop_result->accepted" not in loop


def test_idle_zero_failure_remains_a_diagnostic() -> None:
    loop = (NAV / "app/loop.cpp").read_text(encoding="utf-8")
    idle = loop.split("const bool zero_published_this_frame", 1)[1].split(
        "if (!navigation_runtime_controller.terminalPending())", 1
    )[0]
    assert "motion_stop.keepZeroFresh()" in idle
    assert "hold(" not in idle
    assert 'frames.last_error = "idle_zero_publish_failed"' in idle
