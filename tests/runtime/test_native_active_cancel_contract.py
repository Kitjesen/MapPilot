from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
ENDPOINT_LOOP = ROOT / "src" / "nav" / "cpp" / "endpoint" / "runtime" / "loop.cpp"
TASK_CANCEL_ROUTER_SOURCE = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "command" / "cancel.cpp"
)
NAVIGATION_RUNTIME_SOURCE = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "runtime" / "navigation.cpp"
)
TERMINAL_TRANSACTION_SOURCE = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "status" / "goal_terminal_transaction.cpp"
)
RUNTIME_HEADER = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "plan" / "goal" / "runtime.hpp"
)
RUNTIME_SOURCE = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "plan" / "goal" / "runtime.cpp"
)
GOAL_PLAN_HEADER = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "plan" / "goal" / "plan.hpp"
)
GOAL_PLAN_SOURCE = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "plan" / "goal" / "plan.cpp"
)


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
    endpoint = _read(ENDPOINT_LOOP)
    router = _read(TASK_CANCEL_ROUTER_SOURCE)
    transaction = _read(TERMINAL_TRANSACTION_SOURCE)
    endpoint_router = _block_between(endpoint, "GoalTaskCancelRouter task_cancel_router(", "while (running)")
    cancel_policy = _block_between(
        transaction, "case TerminalStopPolicy::kCancel:", "break;"
    )
    cancel_block = _block_between(
        router,
        "if (request.task_id == snapshot.deferred_replacement_task_id",
        'return {false, "task_not_active"};',
    )

    assert "motion_stop_.cancelPreservingGoalTerminal" in cancel_policy
    assert "motion_stop.cancelPreservingGoalTerminal" not in cancel_block
    assert "terminal_service_(runtime_result)" in cancel_block
    assert "navigation_runtime_controller.completeTerminal(runtime_result)" in endpoint_router


def test_endpoint_services_terminal_from_planning_cycle_before_autonomy_tick() -> None:
    runtime = _read(NAVIGATION_RUNTIME_SOURCE)

    advance_index = runtime.index("goal_replan_runtime_.advancePlanningCycle(frame)")
    autonomy_tick_index = runtime.index("actions.run_autonomy", advance_index)
    service_index = runtime.index("completeTerminal(result.planning_result)", advance_index)

    assert "planning_schedule.service_terminal" in runtime[advance_index:service_index]
    assert service_index < autonomy_tick_index


def test_endpoint_skips_autonomy_tick_while_terminal_delivery_is_pending() -> None:
    runtime = _read(NAVIGATION_RUNTIME_SOURCE)

    advance_index = runtime.index("goal_replan_runtime_.advancePlanningCycle(frame)")
    autonomy_tick_index = runtime.index("actions.run_autonomy", advance_index)
    pre_autonomy = runtime[advance_index:autonomy_tick_index]
    gate_start = runtime.rfind("if (planning_schedule.run_autonomy_tick)", 0, autonomy_tick_index)

    assert "decideGoalTerminalScheduling" in pre_autonomy
    assert "goal_replan_runtime_.terminalPending()" in pre_autonomy
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
        "GoalReplanRuntimeCoordinator::interrupt",
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
        "NavigationRuntimeController::submitGoal",
        "GoalTerminalTransactionResult",
    )

    interrupt_index = submit_goal.index("interrupt(GoalReplanRuntimeInterruption::kNewGoal")
    retain_index = submit_goal.index("result.preemption_terminal = preemption.terminal_transaction")
    reject_index = submit_goal.index('"goal_terminal_pending"', retain_index)
    submit_index = submit_goal.index("goal_plan_.submit")

    assert interrupt_index < retain_index < reject_index < submit_index
    assert "preemption.runtime_result.terminal_after_stop" in submit_goal[retain_index:reject_index]


def test_endpoint_targeted_cancel_routes_hidden_replacement_before_task_not_active() -> None:
    endpoint = _read(ENDPOINT_LOOP)
    router = _read(TASK_CANCEL_ROUTER_SOURCE)
    handle_cancel = _block_between(
        router,
        "CommandAck GoalTaskCancelRouter::handle",
        "}  // namespace lingtu::nav::endpoint",
    )
    endpoint_router = _block_between(
        endpoint,
        "GoalTaskCancelRouter task_cancel_router(",
        "while (running)",
    )
    typed_dispatch = _block_between(
        endpoint,
        "} else if (kind == CommandKind::TaskCancel) {",
        "} else if (kind == CommandKind::TaskPause) {",
    )

    deferred_match_index = handle_cancel.index(
        "request.task_id == snapshot.deferred_replacement_task_id"
    )
    interrupt_index = handle_cancel.index(
        "GoalReplanRuntimeInterruption::kCancel", deferred_match_index
    )
    service_index = handle_cancel.index("terminal_service_(runtime_result)", interrupt_index)
    accepted_index = handle_cancel.index('"cancel_requested"', service_index)
    not_active_index = handle_cancel.index('"task_not_active"', accepted_index)

    assert deferred_match_index < interrupt_index < service_index < accepted_index < not_active_index
    assert "auto handle_cancel =" not in endpoint
    assert "navigation_runtime_controller.completeTerminal(runtime_result)" in endpoint_router
    assert "task_cancel_router.handle" in typed_dispatch
    assert "GoalTaskCancelRequest" in typed_dispatch
    assert "deferred_replacement_request_id" in _read(GOAL_PLAN_HEADER)
    assert "deferred_replacement_goal_epoch" in _read(GOAL_PLAN_SOURCE)
