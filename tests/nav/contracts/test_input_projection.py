from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
ENDPOINT_ROOT = REPO_ROOT / "src/nav/cpp/endpoint"
ENDPOINT_DIR = ENDPOINT_ROOT / "nav"
MAIN_PATH = ENDPOINT_DIR / "main.cpp"
LOOP_PATH = ENDPOINT_DIR / "runtime" / "loop.cpp"
CMAKE_PATH = ENDPOINT_ROOT / "CMakeLists.txt"
ENDPOINT_BUILD_SCRIPT = REPO_ROOT / "scripts/build/build_nav_endpoint.sh"
NATIVE_MOTION_WORKFLOW = REPO_ROOT / ".github/workflows/native-motion-build.yml"
PROJECTOR_HPP = ENDPOINT_DIR / "input/projector.hpp"
PROJECTOR_SOURCES = tuple(
    ENDPOINT_DIR / "input" / name for name in ("pose.cpp", "map.cpp", "health.cpp")
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def _assert_in_order(source: str, markers: list[str]) -> None:
    cursor = -1
    for marker in markers:
        next_cursor = source.find(marker, cursor + 1)
        assert next_cursor != -1, f"missing ordered marker after {cursor}: {marker}"
        cursor = next_cursor


def _block_after(source: str, start: str, end: str) -> str:
    assert start in source, f"missing block start: {start}"
    tail = source.split(start, 1)[1]
    assert end in tail, f"missing block end after {start}: {end}"
    return tail.split(end, 1)[0]


def _main_loop() -> str:
    return _block_after(
        _read(LOOP_PATH),
        "while (running)",
        "current_timing = nullptr;",
    )


def _sensor_drain_stage() -> str:
    return _block_after(
        _read(LOOP_PATH),
        "auto drain_sensors =",
        "auto drain_operator_motion =",
    )


def _operator_motion_drain_stage() -> str:
    return _block_after(
        _read(LOOP_PATH),
        "auto drain_operator_motion =",
        "auto drain_commands =",
    )


def _command_drain_stage() -> str:
    return _block_after(
        _read(LOOP_PATH),
        "auto drain_commands =",
        "auto advance_runtime =",
    )


def test_native_input_projector_files_exist() -> None:
    assert PROJECTOR_HPP.exists()
    assert all(path.exists() for path in PROJECTOR_SOURCES)


def test_navd_build_wires_input_projector_and_endpoint_loop() -> None:
    cmake = _read(CMAKE_PATH)
    navd_target = _block_after(cmake, "add_executable(navd", "target_link_libraries(navd")
    for source in ("input/pose.cpp", "input/map.cpp", "input/health.cpp"):
        assert source in navd_target
    assert "runtime/loop.cpp" in navd_target


def test_linux_endpoint_build_requires_input_projector_regression_test() -> None:
    build_script = _read(ENDPOINT_BUILD_SCRIPT)
    workflow = _read(NATIVE_MOTION_WORKFLOW)
    assert "test_input_projector" in build_script
    assert "bash scripts/build/build_nav_endpoint.sh" in workflow


def test_nav_native_main_includes_and_constructs_input_projector() -> None:
    main = _read(MAIN_PATH)
    main_body = main.split("int main(int argc, char **argv)", 1)[1]

    assert '#include "input/projector.hpp"' in main
    assert "InputProjector" in main_body
    assert "InputProjector inputs(" in main_body


def test_endpoint_loop_batches_dds_input_before_runtime_work() -> None:
    loop = _main_loop()

    _assert_in_order(
        loop,
        [
            "drain_sensors(timing);",
            "command_batch = dds.takeCommands(steadySeconds());",
            "drain_operator_motion();",
            "drain_commands();",
            "const double publish_now = advance_runtime(input_start, timing);",
            "publish_state(publish_now, timing);",
        ],
    )


def test_sensor_batch_delegates_projection_to_input_projector() -> None:
    sensors = _sensor_drain_stage()
    assert "inputs.apply(dds.takeSensors(steadySeconds()), timing);" in sensors
    assert "dds.drain" not in sensors


def test_endpoint_loop_uses_projector_and_only_typed_motion_inputs() -> None:
    main = _read(MAIN_PATH)
    main_body = main.split("int main(int argc, char **argv)", 1)[1]
    sensors = _sensor_drain_stage()
    operator_motion = _operator_motion_drain_stage()
    commands = _command_drain_stage()
    loop = _main_loop()
    endpoint_input = sensors + operator_motion + commands + loop
    input_assembly = _block_after(
        _read(LOOP_PATH),
        "auto drain_commands =",
        "auto advance_runtime =",
    )

    assert "InputSnapshot input_snapshot" not in input_assembly
    assert "input_gate.evaluate(" not in input_assembly
    for marker in [
        "sensorOriginFromBody(",
        "cloudToXyzh(",
        "clear_planner_terrain_inputs()",
        "live_obstacles.updateFromScan",
    ]:
        assert marker not in sensors

    assert "runEndpointLoop(loop_ctx, g_running)" in main_body
    for marker in (
        "CommandBatch command_batch",
        "command_batch.ordered",
        "std::get_if<OperatorMotionControlSample>",
        "std::get_if<OperatorMotionInputSample>",
        "std::get_if<NavigationCommandSample>",
        "std::get_if<InspectionCommandRequest>",
    ):
        assert marker in endpoint_input
    assert "dds.drain" not in endpoint_input
    assert "drain" + "Legacy" not in endpoint_input


def test_epoch_reset_keeps_synchronous_external_side_effects_in_main() -> None:
    main = _read(MAIN_PATH)
    reset_block = _block_after(
        main,
        "auto reset_navigation_epoch = [&](",
        "InputActions inputs_actions;",
    )

    assert "rolling_segment.step(RollingSegmentObserveInvalidInput" in reset_block
    assert "inspection_executor.Pause" in reset_block
    assert "goal_replan_runtime_ptr->interrupt" in reset_block
    assert "sync_goal_plan_diagnostics" in reset_block
    assert "clear_motion_outputs" in reset_block
    assert "PathEcho" not in main
    assert "path_echo" not in main


def test_stop_confirm_loop_reuses_driver_projection_without_full_odometry_projection() -> None:
    main = _read(MAIN_PATH)
    stop_block = _block_after(
        main,
        "auto wait_for_stop_confirmation =",
        "MotionStopActions motion_stop_actions;",
    )

    assert "OdometrySpeedMonitor stop_speed_monitor" in stop_block
    assert "const StopConfirmationState confirmation_state = confirmation.state();" in stop_block
    assert "projectDriverControl" in stop_block
    assert "projectOdometry" not in stop_block


def test_projector_owns_sensor_input_projection_only() -> None:
    assert PROJECTOR_HPP.exists()
    assert all(path.exists() for path in PROJECTOR_SOURCES)
    projector = "\n".join(_read(path) for path in (PROJECTOR_HPP, *PROJECTOR_SOURCES))

    for marker in [
        "InputGate",
        "SensorBatch",
        "clearPlannerInputs",
        "MotionLayer",
        "apply(SensorBatch",
        "projectTf",
        "projectOdometry",
        "projectDriverControl",
        "projectCloud",
        "projectTerrainMap",
        "projectTraversability",
        "projectLocalizationHealth",
    ]:
        assert marker in projector

    for marker in [
        "decodeGoal",
        "decodePath",
        "commandIngressRequestFromDds",
        "InspectionRuntimeController",
        "RollingSegmentCommand",
        "ExplorationSegmentRequest",
    ]:
        assert marker not in projector
