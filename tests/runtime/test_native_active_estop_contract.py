from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
ENDPOINT_LOOP = ROOT / "src" / "nav" / "cpp" / "endpoint" / "runtime" / "loop.cpp"
STOP_HEADER = ROOT / "src" / "nav" / "cpp" / "endpoint" / "safety" / "stop.hpp"
RUNTIME_HEADER = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "plan" / "goal" / "runtime.hpp"
)
RUNTIME_SOURCE = (
    ROOT / "src" / "nav" / "cpp" / "endpoint" / "plan" / "goal" / "runtime.cpp"
)


def _read(path: Path) -> str:
    assert path.exists(), f"expected file to exist: {path.relative_to(ROOT)}"
    return path.read_text(encoding="utf-8", errors="ignore")


def _block_between(source: str, start: str, end: str) -> str:
    assert start in source, f"missing block start: {start}"
    tail = source.split(start, 1)[1]
    assert end in tail, f"missing block end after {start}: {end}"
    return tail.split(end, 1)[0]


def test_runtime_terminal_policy_declares_distinct_estop_replay_policy() -> None:
    header = _read(RUNTIME_HEADER)
    terminal_policy = _block_between(
        header,
        "enum class TerminalStopPolicy {",
        "};",
    )

    assert "kEstop" in terminal_policy


def test_active_estop_interrupt_owns_cancelled_terminal_with_latched_reason() -> None:
    source = _read(RUNTIME_SOURCE)
    interrupt = _block_between(
        source,
        "GoalReplanRuntimeCoordinator::interrupt",
        "std::optional<GoalReplanRuntimeResult>",
    )
    estop_case = _block_between(
        interrupt,
        "case GoalReplanRuntimeInterruption::kEstop:",
        "break;",
    )

    assert "attachDeferredTerminal" in estop_case
    assert "NavigationGoalState::Cancelled" in estop_case
    assert "estop_latched" in estop_case
    assert "TerminalStopPolicy::kEstop" in estop_case


def test_motion_stop_declares_estop_preserving_and_no_terminal_apis() -> None:
    header = _read(STOP_HEADER)

    assert "estopPreservingGoalTerminal" in header
    assert "estopWithoutTerminalCommit" in header


def test_endpoint_routes_estop_through_runtime_terminal_policy_and_barrier() -> None:
    endpoint = _read(ENDPOINT_LOOP)
    service_block = _block_between(
        endpoint,
        "auto handle_estop =",
        "auto handle_clear_estop =",
    )

    assert "GoalReplanRuntimeInterruption::kEstop" in service_block
    assert "interruption.terminal_transaction" in service_block
    assert "action_committed" in service_block
    assert "motion_stop.estopWithoutTerminalCommit" in service_block
    assert "estop_latched" in service_block

    transaction = _read(
        ROOT
        / "src"
        / "nav"
        / "cpp"
        / "endpoint"
        / "status"
        / "goal_terminal_transaction.cpp"
    )
    assert re.search(r"case\s+TerminalStopPolicy::kEstop\s*:", transaction)
    assert "motion_stop_.estopPreservingGoalTerminal" in transaction


def test_endpoint_no_active_estop_uses_physical_only_estop_path() -> None:
    endpoint = _read(ENDPOINT_LOOP)
    estop_handler = _block_between(
        endpoint,
        "auto handle_estop =",
        "auto handle_clear_estop =",
    )

    assert "GoalReplanRuntimeInterruption::kEstop" in estop_handler
    assert "motion_stop.estopWithoutTerminalCommit" in estop_handler
    assert not re.search(r"\bmotion_stop\.estop\s*\(", estop_handler)
    assert "estop_latched" in estop_handler
