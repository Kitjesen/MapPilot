import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NAV_ENDPOINT = ROOT / "src" / "nav" / "cpp" / "endpoint" / "nav"
ENDPOINT_LOOP = NAV_ENDPOINT / "app" / "loop.cpp"
TASK_CANCEL_ROUTER_SOURCE = (
    NAV_ENDPOINT / "command" / "cancel.cpp"
)
NAVIGATION_RUNTIME_SOURCE = NAV_ENDPOINT / "app" / "goal" / "runtime.cpp"
TERMINAL_TRANSACTION_SOURCE = NAVIGATION_RUNTIME_SOURCE
RUNTIME_HEADER = NAV_ENDPOINT / "app" / "goal" / "runtime.hpp"
RUNTIME_SOURCE = NAV_ENDPOINT / "app" / "goal" / "runtime.cpp"
GOAL_PLAN_HEADER = NAV_ENDPOINT / "app" / "goal" / "plan.hpp"
GOAL_PLAN_SOURCE = NAV_ENDPOINT / "app" / "goal" / "plan.cpp"


def _read(path: Path) -> str:
    assert path.exists(), f"expected file to exist: {path.relative_to(ROOT)}"
    return path.read_text(encoding="utf-8", errors="ignore")


def _block_between(source: str, start: str, end: str) -> str:
    assert start in source, f"missing block start: {start}"
    tail = source.split(start, 1)[1]
    assert end in tail, f"missing block end after {start}: {end}"
    return tail.split(end, 1)[0]


def test_runtime_terminal_intent_carries_cancel_stop_policy_for_replay() -> None:
    header = _read(RUNTIME_HEADER)
    source = _read(RUNTIME_SOURCE)
    interrupt = _block_between(
        source,
        "GoalReplanRuntimeCoordinator::interrupt",
        "std::optional<GoalReplanRuntimeResult>",
    )
    cancel_case = _block_between(
        interrupt,
        "case GoalReplanRuntimeInterruption::kCancel:",
        "break;",
    )

    assert "enum class TerminalStopPolicy" in header
    assert "TerminalStopPolicy terminal_stop_policy" in header
    assert "TerminalStopPolicy::kGenericStop" in header
    assert "TerminalStopPolicy::kCancel" in header
    assert "attachDeferredTerminal" in cancel_case
    assert "TerminalStopPolicy::kCancel" in cancel_case


def test_runtime_terminal_policy_declares_distinct_stop_replay_policy() -> None:
    header = _read(RUNTIME_HEADER)
    terminal_policy = _block_between(
        header,
        "enum class TerminalStopPolicy {",
        "};",
    )

    assert re.search(r"\bkStop\b", terminal_policy)


def test_runtime_terminal_intent_carries_stop_policy_for_active_stop() -> None:
    source = _read(RUNTIME_SOURCE)
    interrupt = _block_between(
        source,
        "GoalReplanRuntimeCoordinator::interrupt",
        "std::optional<GoalReplanRuntimeResult>",
    )
    stop_case = _block_between(
        interrupt,
        "case GoalReplanRuntimeInterruption::kStop:",
        "break;",
    )

    assert "attachDeferredTerminal" in stop_case
    assert "NavigationGoalState::Cancelled" in stop_case
    assert "TerminalStopPolicy::kStop" in stop_case


def test_endpoint_replays_active_cancel_terminal_with_cancel_preserving_barrier() -> None:
    runtime = _read(RUNTIME_SOURCE)
    cancel_policy = _block_between(
        runtime, "case TerminalStopPolicy::kCancel:", "case TerminalStopPolicy::kEstop:"
    )
    cancel_method = _block_between(
        runtime,
        "CommandAck GoalReplanRuntimeCoordinator::cancelTask",
        "std::pair<bool, std::string> GoalReplanRuntimeCoordinator::pauseTask",
    )

    assert "motion_stop_.cancelPreservingGoalTerminal" in cancel_policy
    assert "GoalTaskCancelRouter cancel(" in cancel_method
    assert "completeTerminal(result)" in cancel_method
    assert "return cancel.handle(request)" in cancel_method


def test_endpoint_services_terminal_from_planning_cycle_before_autonomy_tick() -> None:
    runtime = _read(NAVIGATION_RUNTIME_SOURCE)

    advance_index = runtime.index("result.planning_result = advancePlanningCycle(frame)")
    autonomy_tick_index = runtime.index("actions.run_autonomy", advance_index)
    service_index = runtime.index("serviceTerminal(result.planning_result)", advance_index)

    assert "planning_schedule.service_terminal" in runtime[advance_index:service_index]
    assert service_index < autonomy_tick_index


def test_endpoint_skips_autonomy_tick_while_terminal_delivery_is_pending() -> None:
    runtime = _read(NAVIGATION_RUNTIME_SOURCE)

    advance_index = runtime.index("result.planning_result = advancePlanningCycle(frame)")
    autonomy_tick_index = runtime.index("actions.run_autonomy", advance_index)
    pre_autonomy = runtime[advance_index:autonomy_tick_index]
    gate_start = runtime.rfind("if (planning_schedule.run_autonomy_tick)", 0, autonomy_tick_index)

    assert "decideGoalTerminalScheduling" in pre_autonomy
    assert "terminalPending()" in pre_autonomy
    assert gate_start > advance_index
    assert "planning_schedule.run_autonomy_tick" in runtime[gate_start:autonomy_tick_index]
    assert "continue;" not in pre_autonomy


def test_endpoint_does_not_cache_pending_terminal_outside_goal_runtime() -> None:
    endpoint = _read(ENDPOINT_LOOP)

    assert "pending_terminal_for_safety_stop" not in endpoint
    assert "preserve_stop([] {})" not in endpoint


def test_runtime_interrupt_closes_deferred_replacement_before_legacy_interrupt_path() -> None:
    source = _read(RUNTIME_SOURCE)
    interrupt = _block_between(
        source,
        "GoalReplanRuntimeCoordinator::interrupt(GoalReplanRuntimeInterruption",
        "std::optional<GoalReplanRuntimeResult>",
    )

    fail_index = interrupt.index("goal_plan_.failDeferredReplacement")
    attach_index = interrupt.index("attachExistingTerminal")
    clear_index = interrupt.index("replacement_plan_in_progress_ = false")
    return_index = interrupt.index("return result;", clear_index)
    switch_index = interrupt.index("switch (interruption)")

    assert fail_index < attach_index < clear_index < return_index < switch_index
    assert "deferredReplacementInterruptionState(interruption)" in interrupt
    assert "interruptionStopPolicy(interruption)" in interrupt
    assert "attachExistingTerminal" in interrupt[fail_index:clear_index]
    assert "return result;" in interrupt[clear_index:switch_index]


def test_runtime_external_goal_rejects_after_terminal_interrupt_without_submit_bypass() -> None:
    runtime = _read(NAVIGATION_RUNTIME_SOURCE)
    submit_goal = _block_between(
        runtime,
        "NavigationRuntimeGoalSubmissionResult GoalReplanRuntimeCoordinator::submitGoal",
        "GoalTerminalResult GoalReplanRuntimeCoordinator::completeTerminal",
    )

    reject_index = submit_goal.index('"goal_terminal_pending"')
    submit_index = submit_goal.index("result.plan_result = goal_plan_.submit")
    interrupt_index = submit_goal.index("interruptAndService(GoalReplanRuntimeInterruption::kNewGoal")

    assert reject_index < submit_index < interrupt_index
    assert "result.preemption_terminal = preemption.terminal" in submit_goal


def test_endpoint_targeted_cancel_routes_hidden_replacement_before_task_not_active() -> None:
    endpoint = _read(ENDPOINT_LOOP)
    runtime = _read(RUNTIME_SOURCE)
    cancel_method = _block_between(
        runtime,
        "CommandAck GoalReplanRuntimeCoordinator::cancelTask",
        "std::pair<bool, std::string> GoalReplanRuntimeCoordinator::pauseTask",
    )
    typed_dispatch = _block_between(
        endpoint,
        "} else if (kind == CommandKind::TaskCancel) {",
        "} else if (kind == CommandKind::TaskPause) {",
    )

    assert "GoalTaskCancelRouter cancel(" in cancel_method
    assert "completeTerminal(result)" in cancel_method
    assert "return cancel.handle(request)" in cancel_method
    assert "runtime.cancelTask" in typed_dispatch
    assert "GoalTaskCancelRequest" in typed_dispatch
    assert "deferred_replacement_request_id" in _read(GOAL_PLAN_HEADER)
    assert "deferred_replacement_goal_epoch" in _read(GOAL_PLAN_SOURCE)
