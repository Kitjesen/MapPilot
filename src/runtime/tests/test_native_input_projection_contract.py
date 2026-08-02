from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
ENDPOINT_DIR = REPO_ROOT / "src/nav/cpp/endpoint"
MAIN_PATH = ENDPOINT_DIR / "nav_native_endpoint.cpp"
LOOP_PATH = ENDPOINT_DIR / "endpoint_loop.cpp"
CMAKE_PATH = ENDPOINT_DIR / "CMakeLists.txt"
ENDPOINT_BUILD_SCRIPT = REPO_ROOT / "scripts/build/build_nav_endpoint.sh"
NATIVE_MOTION_WORKFLOW = REPO_ROOT / ".github/workflows/native-motion-build.yml"
PROJECTOR_HPP = ENDPOINT_DIR / "input/nav_input_state_projector.hpp"
PROJECTOR_CPP = ENDPOINT_DIR / "input/nav_input_state_projector.cpp"


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


def _global_input_loop() -> str:
    return _block_after(
        _read(LOOP_PATH),
        "while (running)",
        "dds.drainExplorationSegmentRequests(",
    )


def test_native_input_state_projector_files_exist() -> None:
    assert PROJECTOR_HPP.exists()
    assert PROJECTOR_CPP.exists()


def test_navd_build_wires_input_state_projector_and_endpoint_loop() -> None:
    cmake = _read(CMAKE_PATH)
    navd_target = _block_after(cmake, "add_executable(navd", "target_link_libraries(navd")
    assert "input/nav_input_state_projector.cpp" in navd_target
    assert "endpoint_loop.cpp" in navd_target


def test_linux_endpoint_build_requires_input_projector_regression_test() -> None:
    build_script = _read(ENDPOINT_BUILD_SCRIPT)
    workflow = _read(NATIVE_MOTION_WORKFLOW)
    assert "test_nav_input_state_projector" in build_script
    assert "bash scripts/build/build_nav_endpoint.sh" in workflow


def test_nav_native_main_includes_and_constructs_input_state_projector() -> None:
    main = _read(MAIN_PATH)
    main_body = main.split("int main(int argc, char **argv)", 1)[1]

    assert '#include "input/nav_input_state_projector.hpp"' in main
    assert "NavInputStateProjector" in main_body
    assert "input_projector" in main_body


def test_endpoint_loop_keeps_globally_interleaved_dds_drain_order() -> None:
    loop = _global_input_loop()

    _assert_in_order(
        loop,
        [
            "dds.drainTf(",
            "dds.drainOdometry(",
            "dds.drainDriverControlState(",
            "dds.drainExplorationExecutionGrids(",
            "dds.drainCloud(",
            "dds.drainTerrainMap(",
            "dds.drainTerrainMapExt(",
            "dds.drainMapClearing(",
            "dds.drainCloudClearing(",
            "dds.drainTraversability(",
            "dds.drainLocalizationHealth(",
            "dds.drainOperatorMotionControls(",
            "dds.drainOperatorMotionSamples(",
            "dds.drainCommandRequests(",
            "dds.drainInspectionTaskRequests(",
        ],
    )


def test_sensor_drains_delegate_projection_to_input_state_projector() -> None:
    loop = _global_input_loop()
    sensor_drain_starts = [
        "dds.drainTf(",
        "dds.drainOdometry(",
        "dds.drainDriverControlState(",
        "dds.drainCloud(",
        "dds.drainTerrainMap(",
        "dds.drainTerrainMapExt(",
        "dds.drainMapClearing(",
        "dds.drainCloudClearing(",
        "dds.drainTraversability(",
        "dds.drainLocalizationHealth(",
    ]

    for start in sensor_drain_starts:
        block = _block_after(loop, start, "});")
        assert "input_projector." in block, f"{start} must delegate to input_projector"


def test_endpoint_loop_uses_projector_and_only_typed_motion_inputs() -> None:
    main = _read(MAIN_PATH)
    main_body = main.split("int main(int argc, char **argv)", 1)[1]
    loop = _global_input_loop()
    input_assembly = _block_after(
        _read(LOOP_PATH),
        "dds.drainCommandRequests(",
        "dds.drainExplorationSegmentRequests(",
    )

    assert "InputSnapshot input_snapshot" not in input_assembly
    assert "input_gate.evaluate(" not in input_assembly
    for marker in [
        "sensorOriginFromBody(",
        "cloudToXyzh(",
        "clear_planner_terrain_inputs()",
        "live_obstacles.updateFromScan",
    ]:
        assert marker not in loop

    assert "runEndpointLoop(loop_ctx, g_running)" in main_body
    for marker in (
        "dds.drainOperatorMotionControls(",
        "dds.drainOperatorMotionSamples(",
        "dds.drainCommandRequests(",
        "dds.drainInspectionTaskRequests(",
    ):
        assert marker in loop
    assert "drain" + "Legacy" not in loop


def test_epoch_reset_keeps_synchronous_external_side_effects_in_main() -> None:
    main = _read(MAIN_PATH)
    reset_block = _block_after(
        main,
        "auto reset_navigation_epoch = [&](",
        "NavInputStateProjectorActions input_projector_actions;",
    )

    assert "rolling_segment.step(RollingSegmentObserveInvalidInput" in reset_block
    assert "inspection_executor.Pause" in reset_block
    assert "goal_replan_runtime_ptr->interrupt" in reset_block
    assert "sync_goal_plan_diagnostics" in reset_block
    assert "clear_motion_outputs" in reset_block
    assert "path_echo.reset" in reset_block


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
    assert PROJECTOR_CPP.exists()
    projector = _read(PROJECTOR_HPP) + "\n" + _read(PROJECTOR_CPP)

    for marker in [
        "InputGate",
        "SensorOrigin",
        "cloudToXyzh",
        "clearPlannerInputs",
        "LiveObstacleLayer",
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
