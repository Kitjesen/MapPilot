# ruff: noqa: D103, S101 - pytest contracts use assertions by design.

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXPLORE_DDS = ROOT / "src" / "explore" / "cpp" / "endpoint" / "main.cpp"


def _source() -> str:
    return EXPLORE_DDS.read_text(encoding="utf-8")


def test_explore_reader_uses_dds_loan_instead_of_per_tick_allocations() -> None:
    source = _source()
    drain_reader = source.split("template <typename T, typename Handler>", maxsplit=1)[1].split(
        "bool parseBool", maxsplit=1
    )[0]

    assert "dds_alloc(" not in drain_reader
    assert "dds_sample_free(" not in drain_reader
    assert "void *samples[kMaxSamples]{};" in drain_reader
    assert "dds_take(reader, samples, infos, kMaxSamples, kMaxSamples)" in drain_reader
    assert "catch (...)" in drain_reader
    assert drain_reader.count("dds_return_loan(reader, samples, count)") == 2


def test_explore_status_path_borrows_snapshot_instead_of_copying_grid() -> None:
    source = _source()
    signature = source.split("std::string statusSnapshot(", maxsplit=1)[1].split(
        ") {", maxsplit=1
    )[0]
    call = source.split("status_writer.submit(statusSnapshot(", maxsplit=1)[1].split(
        "));", maxsplit=1
    )[0]

    assert "const ExplorationSnapshot *snapshot" in signature
    assert "snapshot_route_bound ? &*snapshot : nullptr" in call
    assert "isCanonicalExploreSnapshotBinding(" in source
    assert "config.expected_map_id" in source
    assert "config.expected_map_content_epoch" in source
    assert "snapshot_fresh ? snapshot : std::nullopt" not in source


def test_explore_rejects_foreign_snapshots_before_map_epoch_transition() -> None:
    source = _source()
    handler = source.split("dds.drainSnapshot", maxsplit=1)[1].split(
        "const bool robot_fresh", maxsplit=1
    )[0]

    assert handler.index("isCanonicalExploreSnapshotBinding") < handler.index(
        "const bool map_epoch_changed"
    )
    assert 'run_lifecycle.fail(loop_now_s, "exploration_map_identity_changed"' in handler
    assert "exploration_control.Complete()" in handler
    assert 'cancel_reason = "exploration_map_identity_changed"' in handler


def test_explore_runtime_uses_the_public_dispatching_state() -> None:
    source = _source()

    assert 'runtime_state = "dispatching"' in source
    assert 'runtime_state = "goal_dispatching"' not in source


def test_map_route_uses_product_bound_occupancy() -> None:
    source = _source()

    assert 'config.occupancy_path = envString("EXPLORE_OCCUPANCY_PATH")' in source
    assert "saved_coverage_gate.prepare(config.occupancy_path)" in source
    assert "NAV_MAP_DIR" not in source
    assert "--map-root" not in source
    assert "config.map_root" not in source
    assert 'config.expected_map_id / "occupancy.npz"' not in source
    assert "LINGTU_ACTIVE_OCCUPANCY" not in source


def test_saved_coverage_is_built_once_and_borrowed_without_cell_copy() -> None:
    source = _source()
    planning = source.split("if (!already_committed)", maxsplit=1)[1].split(
        "if (exploration_control.running() && last_decision.done", maxsplit=1
    )[0]

    assert "buildSavedCoverageGrid" not in planning
    assert "input.exploration_grid = std::move(*saved_coverage_grid)" in planning
    assert planning.count("*saved_coverage_grid = std::move(input.exploration_grid)") == 2


def test_explore_navigation_ack_wait_is_tick_driven() -> None:
    source = _source()

    assert '"nav/cpp/client/client.hpp"' not in source
    assert "commands.navigation()" not in source
    assert "dds.drainCommandAck" in source
    assert "goal_command_lane.advance" in source


def test_explore_shutdown_waits_for_terminal_motion_status() -> None:
    source = _source()
    shutdown = source.split(
        'constexpr const char *kShutdownReason = "exploration_endpoint_stopped";',
        maxsplit=1,
    )[1]

    assert "while ((pending.has_value() || pending_segment.has_value())" in shutdown
    assert "dds.drainGoalStatus" in shutdown
    assert "run_lifecycle.fail" in shutdown
    assert "handleCommandAcks(shutdown_tick_s, false)" in shutdown
    assert "exploration_shutdown_motion_stop_unconfirmed" in shutdown


def test_explore_status_exposes_run_event_delivery_health() -> None:
    source = _source()
    status = source.split("std::string statusSnapshot(", maxsplit=1)[1].split(
        "return output.str();", maxsplit=1
    )[0]

    assert "run_event_outbox.diagnostics()" in source
    assert r'\"run_event_outbox\"' in status
    assert r'\"rejected_backpressure\"' in status
    assert r'\"delivery_failures\"' in status
    assert r'\"pending\"' in status
