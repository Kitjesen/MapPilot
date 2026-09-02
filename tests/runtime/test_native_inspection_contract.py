from pathlib import Path

from lingtu.assembly.products import resolve_product_host_config
from nav.adapters.native import inspection_store as native_inspection
from runtime.runtime_interface import TOPIC_ALLOWED_FRAME_IDS, TOPICS


def test_inspection_profile_does_not_enable_python_patrol_runtime() -> None:
    config = resolve_product_host_config(
        "inspection", "real", robot="doso/thunder_v4"
    )

    assert "enable_patrol_routes" not in config
    assert "enable_scheduler" not in config


def test_typed_inspection_topics_are_canonical() -> None:
    assert not hasattr(TOPICS, "inspection_command")
    assert not hasattr(TOPICS, "inspection_ack")
    assert TOPICS.inspection_task_request == "/nav/inspection/task/request"
    assert TOPICS.inspection_task_ack == "/nav/inspection/task/ack"
    assert TOPICS.inspection_status == "/nav/inspection/status"
    assert TOPICS.inspection_evidence_request == "/nav/inspection/evidence/request"
    assert TOPICS.inspection_evidence_result == "/nav/inspection/evidence/result"
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_task_request] == ("map",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_task_ack] == ("map",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_status] == ("map",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_evidence_request] == ("map",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_evidence_result] == ("map",)


def test_inspection_task_events_are_ordered_native_facts() -> None:
    from message.topics import dds_topic_name, topic_spec

    topic = TOPICS.inspection_task_event
    assert topic == "/nav/inspection/task/event"
    assert TOPIC_ALLOWED_FRAME_IDS[topic] == ("map",)
    assert topic_spec(topic).dds_topic == "rt/nav/inspection/task/event"
    assert dds_topic_name(topic) == "rt/nav/inspection/task/event"

    idl = Path("src/message/idl/messages.idl").read_text(encoding="utf-8")
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    runtime = Path("src/nav/cpp/endpoint/nav/dds/runtime.cpp").read_text(encoding="utf-8")
    assert "struct InspectionTaskEvent" in idl
    assert "FlushTaskEvents" in endpoint
    assert "InspectionTaskEventOutbox" in endpoint
    assert "writeInspectionTaskEvent" in runtime


def test_inspection_idl_and_native_endpoint_are_wired() -> None:
    idl = Path("src/message/idl/messages.idl").read_text(encoding="utf-8")
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    controller = Path(
        "src/nav/cpp/endpoint/nav/runtime/inspection/inspection_runtime_controller.cpp"
    ).read_text(encoding="utf-8")
    cmake = Path("src/nav/cpp/endpoint/CMakeLists.txt").read_text(encoding="utf-8")

    assert "struct InspectionCommandRequest" not in idl
    assert "struct InspectionCommandAck" not in idl
    assert "struct InspectionTaskRequest" in idl
    assert "struct InspectionTaskAck" in idl
    assert "struct InspectionStatus" in idl
    assert "command_batch = dds.takeCommands" in endpoint
    assert "std::get_if<InspectionCommandRequest>" in endpoint
    assert "inspection_runtime.tick(" in endpoint
    assert "inspection_runtime.onGoalReached" in endpoint
    assert "executor_.PendingGoal()" in controller
    assert "nav/runtime/inspection/inspection_runtime_controller.cpp" in cmake
    assert "lingtu_inspection_core" in cmake


def test_inspection_action_deadline_is_started_before_evidence_publish() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    controller = Path(
        "src/nav/cpp/endpoint/nav/runtime/inspection/inspection_runtime_controller.cpp"
    ).read_text(encoding="utf-8")
    action_block = controller.split(
        "if (const auto action = executor_.PendingAction()) {", 1
    )[1].split("return result;", 1)[0]
    publish_block = endpoint.split(
        "if (inspection_result.evidence_dispatch) {", 1
    )[1].split("(void)inputs.materializeObstacles", 1)[0]
    completion_block = controller.split(
        "InspectionRuntimeController::completeEvidenceDispatch", 1
    )[1].split("void InspectionRuntimeController::requestStatus", 1)[0]

    assert action_block.index("executor_.OnActionStarted") < action_block.index(
        "result.evidence_dispatch"
    )
    assert "executor_.status().deadline_s" in action_block
    assert endpoint.index("inspection_runtime.tick") < endpoint.index(
        "InspectionEvidenceRequestOutput{"
    )
    assert publish_block.index("InspectionEvidenceRequestOutput{") < publish_block.index(
        "inspection_runtime.completeEvidenceDispatch"
    )
    assert "evidence_request_publish_failed" in completion_block
    assert "executor_.OnActionResult" in completion_block
    assert "input_now" not in publish_block


def test_inspection_evidence_dds_contract_is_complete() -> None:
    from message.topics import dds_topic_name, topic_spec

    request_topic = TOPICS.inspection_evidence_request
    result_topic = TOPICS.inspection_evidence_result
    expected = {
        request_topic: ("rt/nav/inspection/evidence/request", "InspectionEvidenceRequest"),
        result_topic: ("rt/nav/inspection/evidence/result", "InspectionEvidenceResult"),
    }

    for topic, (wire_topic, type_name) in expected.items():
        spec = topic_spec(topic)
        assert spec is not None
        assert spec.type_name == type_name
        assert spec.dds_topic == wire_topic
        assert spec.idl_type == f"lingtu.dds.{type_name}"
        assert dds_topic_name(topic) == wire_topic


def test_native_inspection_library_candidates_cover_repo_and_deployed_builds(
    monkeypatch,
) -> None:
    monkeypatch.delenv("LINGTU_INSPECTION_LIBRARY", raising=False)

    candidates = native_inspection._library_candidates()
    repo_root = Path(native_inspection.__file__).resolve().parents[4]
    library_name = native_inspection._library_names()[0]

    assert repo_root / "src" / "nav" / "inspection" / "build" / library_name in candidates
    assert repo_root / "build" / "nav_endpoint" / "inspection" / library_name in candidates
    assert Path("/opt/lingtu/current/lib") / library_name in candidates


def test_inspection_resume_requires_autonomy_control_before_executor_resume() -> None:
    coordinator = Path(
        "src/nav/cpp/endpoint/nav/runtime/inspection/inspection_command_coordinator.cpp"
    ).read_text(encoding="utf-8")
    resume_block = coordinator.split("CommandKind::kResume", 1)[1].split(
        "CommandKind::kCancel", 1
    )[0]

    assert "actions_.operator_takeover_latched()" in resume_block
    assert "inspection_resume_requires_autonomy" in resume_block
    assert resume_block.index("operator_takeover_latched") < resume_block.index("executor_.Resume")


def test_active_map_change_clears_inspection_motion_immediately() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    controller = Path(
        "src/nav/cpp/endpoint/nav/runtime/inspection/inspection_runtime_controller.cpp"
    ).read_text(encoding="utf-8")
    map_block = controller.split("const InspectionRunState state_before_map_check", 1)[1].split(
        "if (executor_.status().state != InspectionRunState::kPlanning)", 1
    )[0]
    intent_block = endpoint.split(
        "for (const auto &intent : inspection_result.ordered_intents)", 1
    )[1].split("if (inspection_result.goal_dispatch)", 1)[0]

    assert 'executor_.status().reason == "active_map_changed"' in map_block
    assert map_block.index("kStopControlAuthority") < map_block.index(
        'appendClearMotion(result, "inspection_active_map_changed")'
    )
    assert intent_block.index("control_authority.stop()") < intent_block.index(
        "motion_stop.clearEndpointMotion(intent.reason)"
    )


def test_autonomy_input_gate_zero_intent_is_published_without_nav_output() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    controller = Path(
        "src/nav/cpp/endpoint/nav/control/autonomy.cpp"
    ).read_text(encoding="utf-8")

    blocked_branch = controller.split(
        "if (input.path_active && !input.input_gate.ready) {", 1
    )[1].split("return result;", 1)[0]
    projection = endpoint.split("autonomy_result = autonomy_tick.tick", 1)[1].split(
        "switch (autonomy_result.outcome.kind)", 1
    )[0]

    assert "result.publish.cmd_vel = input.publish_cmd_vel" in blocked_branch
    assert "result.publish.command = {}" in blocked_branch
    assert "FinalVelocityOutput{autonomy_result.publish.command}" in projection
    assert "dds.writeCmdVel" not in projection


def test_local_recovery_exhaustion_routes_into_inspection_failure_policy() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    controller = Path(
        "src/nav/cpp/endpoint/nav/control/autonomy.cpp"
    ).read_text(encoding="utf-8")
    runtime = Path(
        "src/nav/cpp/endpoint/nav/runtime/navigation.cpp"
    ).read_text(encoding="utf-8")

    assert "output.recovery_exhausted" in controller
    assert "AutonomyTickOutcomeKind::kGoalFailed" in controller
    outcome_index = runtime.index("goal_replan_runtime_.handleAutonomyOutcome")
    deferred_index = runtime.index(
        "deferred_inspection_completion_ = observation.outcome", outcome_index
    )
    terminal_index = runtime.index("completeTerminal(terminal_candidate)", deferred_index)
    acknowledgement_index = runtime.index(
        "result.terminal_delivery_acknowledged && deferred_inspection_completion_",
        terminal_index,
    )
    completion_index = endpoint.index("runtime_frame_result.inspection_completion")
    failure_index = endpoint.index("inspection_executor.OnNavigationFailed", completion_index)
    assert outcome_index < deferred_index < terminal_index < acknowledgement_index
    assert completion_index < failure_index
    assert "goal_plan.deferFailure(" not in endpoint
    assert "goal_plan.deferActiveTerminal(" not in endpoint


def test_safety_stop_transitions_use_single_coordinator() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(
        encoding="utf-8"
    )
    cancel_router = Path(
        "src/nav/cpp/endpoint/nav/command/cancel.cpp"
    ).read_text(encoding="utf-8")
    coordinator = Path(
        "src/nav/cpp/endpoint/nav/safety/stop.cpp"
    ).read_text(encoding="utf-8")
    transaction = Path(
        "src/nav/cpp/endpoint/nav/status/goal_terminal_transaction.cpp"
    ).read_text(encoding="utf-8")
    runtime = Path(
        "src/nav/cpp/endpoint/nav/runtime/navigation.cpp"
    ).read_text(encoding="utf-8")
    cmake = Path("src/nav/cpp/endpoint/CMakeLists.txt").read_text(encoding="utf-8")

    for transition in (
        "motion_stop.clearEstop(stamp_error)",
        "motion_stop.resumeAutonomy(request)",
    ):
        assert transition in endpoint
    assert "GoalReplanRuntimeInterruption::kCancel" in cancel_router
    assert "terminal_service_(runtime_result)" in cancel_router
    assert "motion_stop.cancel()" not in endpoint
    assert "motion_stop_.cancel()" not in cancel_router
    assert "GoalReplanRuntimeInterruption::kStop" in endpoint
    assert "navigation_runtime_controller.interrupt(" in endpoint
    assert "motion_stop_.stopPreservingGoalTerminal(" in transaction
    assert "motion_stop_.stopWithoutTerminalCommit(" in transaction
    assert "motion_stop.stop()" not in endpoint
    assert "GoalReplanRuntimeInterruption::kEstop" in endpoint
    assert "motion_stop_.estopPreservingGoalTerminal(" in transaction
    assert "motion_stop.estopWithoutTerminalCommit(" in endpoint
    assert "motion_stop.estop(reason)" not in endpoint
    assert "GoalReplanRuntimeInterruption::kDriverAuthorityLost" in endpoint
    assert 'motion_stop.clearEndpointMotion("driver_control_lost:" + driver_blocker)' in endpoint
    assert "motion_stop.driverAuthorityLost(driver_blocker)" not in endpoint
    assert "auto confirm_last_zero" not in endpoint
    assert "goal_terminal_transaction_.advance(runtime_result" in runtime
    terminal_block = transaction.split(
        "GoalTerminalTransaction::advance", 1
    )[1].split("GoalTerminalTransaction::stopWhileTerminalPending", 1)[0]
    confirmation_block = coordinator.split(
        "MotionStopBarrier::confirmAndCommitTerminal", 1
    )[1].split("MotionStopResult MotionStopBarrier::confirmLastZero", 1)[0]
    assert "motion_stop_.stopPreservingGoalTerminal(terminal.commit)" in terminal_block
    assert "motion_stop_.commitGoalTerminalAfterStop" in terminal_block
    assert confirmation_block.index("confirmLastZero") < confirmation_block.index(
        "commit_terminal()"
    )
    assert "nav/safety/stop.cpp" in cmake


def test_shutdown_uses_ticketed_runtime_terminal_instead_of_direct_goal_abort() -> None:
    runtime_header = Path(
        "src/nav/cpp/endpoint/nav/runtime/goal/runtime.hpp"
    ).read_text(encoding="utf-8")
    coordinator = Path(
        "src/nav/cpp/endpoint/nav/safety/stop.cpp"
    ).read_text(encoding="utf-8")
    delivery_header = Path(
        "src/nav/cpp/endpoint/nav/status/goal_terminal_status_delivery.hpp"
    ).read_text(encoding="utf-8")
    transaction = Path(
        "src/nav/cpp/endpoint/nav/status/goal_terminal_status_delivery.cpp"
    ).read_text(encoding="utf-8")

    assert "kShutdown" in runtime_header
    assert "struct ShutdownTransactionResult" in delivery_header
    assert "std::string reason" in delivery_header
    assert "advanceShutdownTransaction(" in delivery_header
    assert "GoalReplanRuntimeInterruption::kShutdown" in transaction
    assert "motion_stop.finalShutdownWithoutTerminalCommit()" in transaction
    assert "motion_stop.finalShutdownPreservingGoalTerminal(" in transaction
    assert "goal_terminal_delivery.flushAndAcknowledge(" in transaction
    assert "decideShutdownExit(" in transaction
    for reason in (
        "shutdown_terminal_delivery_pending",
        "shutdown_zero_confirm_pending",
        "shutdown_complete",
    ):
        assert f'"{reason}"' in transaction
    assert 'deferGoalAbort("navd_shutdown")' not in coordinator


def test_shutdown_post_loop_retries_stop_and_terminal_delivery_before_exit() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(
        encoding="utf-8"
    )
    shutdown_block = endpoint.split("auto shutdown = [&]() -> int {", 1)[1].split(
        "auto drain_sensors", 1
    )[0]
    shutdown_loop = shutdown_block.split("while (true) {", 1)[1]

    assert "while (true) {" in shutdown_block
    assert "advanceShutdownTransaction(" in shutdown_loop
    assert "decision.allow_exit" in shutdown_loop
    assert "motion_stop.keepZeroFresh()" in shutdown_loop
    assert shutdown_loop.index("advanceShutdownTransaction(") < shutdown_loop.index(
        "decision.allow_exit"
    ) < shutdown_loop.index("motion_stop.keepZeroFresh()") < shutdown_loop.index(
        "std::this_thread::sleep_for(tick_period)"
    )
    assert "kPendingLogInterval" in shutdown_block
    assert "next_pending_log" in shutdown_block
    log_guard = shutdown_loop.index(
        "if (log_now >= next_pending_log)"
    )
    log_message = shutdown_loop.index("navd shutdown pending: %s")
    log_reason = shutdown_loop.index("transaction.reason.c_str()")
    log_advance = shutdown_loop.index(
        "next_pending_log = log_now + kPendingLogInterval"
    )
    assert log_guard < log_message < log_reason < log_advance
    assert "GoalReplanRuntimeInterruption::kShutdown" not in shutdown_block
    assert "goal_terminal_delivery.flushAndAcknowledge(" not in shutdown_block
    assert "motion_stop.finalShutdownWithoutTerminalCommit()" not in shutdown_block
    assert "motion_stop.finalShutdownPreservingGoalTerminal(" not in shutdown_block
    assert "if (!shutdown_result.success)" not in shutdown_block
    assert "deadline" not in shutdown_loop.lower()
    assert "ShutdownTransactionRetryLimit" not in shutdown_loop
    assert "return 1;" not in shutdown_loop
    assert "break;" not in shutdown_loop
    assert shutdown_loop.count("return ") == 1
    assert "return 0;" in shutdown_loop


def test_inspection_periodic_file_io_is_off_the_motion_loop() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    bootstrap = Path("src/nav/cpp/endpoint/nav/main.cpp").read_text(
        encoding="utf-8"
    )
    cmake = Path("src/nav/cpp/endpoint/CMakeLists.txt").read_text(encoding="utf-8")
    motion_loop = endpoint.split("while (running) {", 1)[1]

    assert "inspection_status_writer.submit(inspection_status)" in endpoint
    assert endpoint.index("inspection_status_writer.submit(inspection_status)") < endpoint.index(
        "while (running) {"
    )
    assert "inspection_store->PutStatus" not in endpoint
    assert "inspection_store->PutStatus" not in bootstrap
    assert "GetActiveMap()" not in motion_loop
    assert "InspectionActiveMap{cfg.map_identity.map_id" in bootstrap
    assert "ActiveInspectionMapCache" not in bootstrap
    assert "nav/status/inspection_status_file_writer.cpp" in cmake


def test_endpoint_feeds_final_inspection_point_progress_to_watchdog() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    controller = Path(
        "src/nav/cpp/endpoint/nav/runtime/inspection/inspection_runtime_controller.cpp"
    ).read_text(encoding="utf-8")

    assert controller.count("executor_.OnNavigationProgress") == 1
    assert "active_point_->x_m" in controller
    assert "active_point_->y_m" in controller
    assert "InspectionRuntimeRobotPosition" in endpoint
    assert "map_body->position.x" in endpoint
    assert "map_body->position.y" in endpoint


def test_post_arrival_inspection_actions_fail_closed_on_localization_gate() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(encoding="utf-8")
    controller = Path(
        "src/nav/cpp/endpoint/nav/runtime/inspection/inspection_runtime_controller.cpp"
    ).read_text(encoding="utf-8")

    gate_index = endpoint.index("inputs.evaluateGate(")
    orchestration_index = endpoint.index("inspection_runtime.tick")
    assert gate_index < orchestration_index
    assert "inspectionPostArrivalState" in endpoint
    assert "localizationGateBlocked" in endpoint
    assert "localizationGateBlocked(input_gate_state, gate_cfg)" in endpoint
    assert "state.localization_health_age_s" in endpoint
    assert "inspection_post_arrival_localization_pause" in endpoint
    pause_index = endpoint.index("inspection_post_arrival_localization_pause")
    assert pause_index < orchestration_index
    assert controller.index("executor_.Tick") < controller.index("executor_.PendingAction")


def test_periodic_nav_status_assembly_is_off_main_loop() -> None:
    endpoint = Path("src/nav/cpp/endpoint/nav/runtime/loop.cpp").read_text(
        encoding="utf-8"
    )
    publisher = Path(
        "src/nav/cpp/endpoint/nav/status/nav_status_publisher.cpp"
    ).read_text(encoding="utf-8")
    adapter = Path(
        "src/nav/cpp/endpoint/nav/status/nav_status_endpoint_adapter.cpp"
    ).read_text(encoding="utf-8")
    cmake = Path("src/nav/cpp/endpoint/CMakeLists.txt").read_text(encoding="utf-8")

    assert "nav_status.publishIfDue(" in endpoint
    assert "writeStatusSnapshot(" not in endpoint
    assert "next_status" not in endpoint
    assert "writeStatusSnapshot(" in publisher
    assert "StatusRuntimeState statusRuntimeStateFromEndpoint" in adapter
    assert "status/nav_status_publisher.cpp" in cmake
    assert "status/nav_status_endpoint_adapter.cpp" in cmake
