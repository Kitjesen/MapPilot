from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_server_setup_runs_multifloor_closure_without_robot_motion():
    native_script = (
        REPO_ROOT / "sim/scripts/setup_linux_validation_host.sh"
    ).read_text(encoding="utf-8")
    ros_compat_script = (
        REPO_ROOT / "scripts/compat/ros2/setup_fastlio2_validation_host.sh"
    ).read_text(encoding="utf-8")
    legacy_script = REPO_ROOT / "scripts/deploy/setup_server_ros_pct.sh"

    assert not legacy_script.exists()
    assert 'RUN_MULTIFLOOR="${LINGTU_RUN_MULTIFLOOR:-1}"' in native_script
    assert 'RUN_NAV_KERNEL="${LINGTU_RUN_NAV_KERNEL:-1}"' in native_script
    assert (
        'PCT_BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"'
        in native_script
    )
    assert (
        'RUN_ROUTECHECK_PREFLIGHT="${LINGTU_RUN_ROUTECHECK_PREFLIGHT:-1}"'
        in native_script
    )
    assert (
        'SETUP_CLOSURE_MAX_REPORT_AGE_S='
        '"${LINGTU_SETUP_CLOSURE_MAX_REPORT_AGE_S:-21600}"'
        in native_script
    )
    assert "build_nav_kernel_runtime" in native_script
    assert (
        'bash "${ROOT}/scripts/build/build_nav_kernel.sh" --clean'
        in native_script
    )
    assert "building nav_kernel nanobind runtime" in native_script
    assert "building PCT Rust GPMP runtime" in native_script
    assert (
        "scripts/build/build_rust_kernels.py "
        "--target gpmp_trajectory_optimizer --release"
        in native_script
    )
    assert (
        'local out_dir="${ROOT}/src/nav/services/plan/global_planner/'
        'algorithm/pct/runtime/rust/${arch}"'
        in native_script
    )
    assert 'if [[ "${PCT_BUILD_LEGACY_NATIVE}" == "1" ]]; then' in native_script
    assert "building PCT legacy native/GTSAM runtime" in native_script
    assert "LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 \\" in native_script
    assert "multi-floor exploration/local-planning closure gate" in native_script
    assert "Gateway routecheck preflight closure gate, non-motion" in native_script
    assert "sim/scripts/routecheck_preflight_gate.py" in native_script
    assert "server simulation closure summary for setup-generated gates" in native_script
    assert "sim/scripts/server_sim_closure.py" in native_script
    assert "artifacts/server_sim_closure_summary_setup.json" in native_script
    assert "--max-report-age-s" in native_script
    assert "--required-only" in native_script
    assert '"${SETUP_CLOSURE_MAX_REPORT_AGE_S}"' in native_script
    assert "--route matrix" in native_script
    assert "--frontier-loop" in native_script
    assert "--local-planner-backend nanobind" in native_script
    assert "--require-production-local-planner" in native_script
    assert (
        "artifacts/server_sim_closure/multifloor_exploration/report.json"
        in native_script
    )
    assert "cmd_vel" in native_script
    assert "physical hardware" in native_script
    assert (
        'if [[ "${RUN_PCT}" == "1" ]]; then\n'
        '    log "PCT runtime inspection"'
        in native_script
    )
    assert "local focused_tests=()" in native_script
    assert "focused_tests+=(src/runtime/tests/test_pct_runtime.py)" in native_script
    assert "src/runtime/tests/test_gateway_commands.py" not in native_script
    assert "src/runtime/tests/test_gateway_route_split.py" not in native_script

    native_forbidden = (
        "LINGTU_INSTALL_ROS2",
        "LINGTU_RUN_ROS2_FASTLIO2",
        "LINGTU_RUN_ROS2_LOCAL_PLANNER",
        "source_ros_if_present",
        "/opt/ros",
        "colcon",
        "rosdep",
        "ros2 pkg",
        "FishROS",
    )
    for token in native_forbidden:
        assert token not in native_script

    assert "detect_ros_distro" in ros_compat_script
    assert "install_ros2_official" in ros_compat_script
    assert "install_ros2_fishros" in ros_compat_script
    assert "source_ros_if_present" in ros_compat_script
    assert "colcon build" in ros_compat_script
    assert "--packages-up-to fastlio2" in ros_compat_script
    assert "ros2 pkg executables fastlio2" in ros_compat_script
    assert "ros2 pkg prefix livox_ros_driver2" in ros_compat_script

    lowered_compat = ros_compat_script.lower()
    for token in ("pct", "mujoco", "nav_kernel", "multifloor", "routecheck"):
        assert token not in lowered_compat

def test_p0_scripts_use_current_gateway_contracts():
    goto = (REPO_ROOT / "docs/07-testing/p0_goto.sh").read_text(
        encoding="utf-8"
    )
    estop = (REPO_ROOT / "docs/07-testing/p0_estop.sh").read_text(
        encoding="utf-8"
    )
    mapping = (REPO_ROOT / "docs/07-testing/p0_mapping.sh").read_text(
        encoding="utf-8"
    )
    route_safety = (
        REPO_ROOT / "docs/07-testing/p0_route_safety.sh"
    ).read_text(encoding="utf-8")
    explore = (REPO_ROOT / "docs/07-testing/p0_explore.sh").read_text(
        encoding="utf-8"
    )

    assert "/api/v1/navigation/status" in goto
    assert "/api/v1/nav/status" not in goto
    assert 'GOAL_X="${LINGTU_P0_GOAL_X:-${1:-}}"' in goto
    assert 'GOAL_Y="${LINGTU_P0_GOAL_Y:-${2:-}}"' in goto
    assert 'GOAL_X="${1:-2.0}"' not in goto
    assert 'GOAL_Y="${2:-0.0}"' not in goto
    assert "p0_route_safety.sh" in goto
    assert "Type RUN" in goto
    assert '\\"frame_id\\":\\"map\\"' in goto
    assert '\\"client_id\\":\\"p0_goto\\"' in goto
    assert "P0-04 Goto" in goto

    goto_preview_index = goto.index('p0_route_safety.sh" "$GOAL_X" "$GOAL_Y"')
    goto_confirm_index = goto.index('if [[ "$answer" != "RUN" ]]')
    goto_post_index = goto.index("curl -sf -X POST http://localhost:5050/api/v1/goal")
    assert goto_preview_index < goto_confirm_index < goto_post_index

    assert "POST /api/v1/stop" in estop
    assert "GET /api/v1/state" in estop
    assert "PRE_STOP_SPEED" in estop
    assert "cleanup_stop" in estop
    assert "STOP_ON_EXIT=1" in estop
    assert "p0-estop-cleanup" in estop
    assert "current_speed_mps" in estop
    assert "P0-05 E-stop" in estop
    assert "/api/v1/safety/state" not in estop
    assert "curl -sf http://localhost:5050/api/v1/cmd_vel" not in estop

    assert 'scripts/lingtu" map save "$MAP_NAME"' in mapping
    assert 'scripts/lingtu" nav start "$MAP_NAME"' in mapping
    assert "/api/v1/map/save" in mapping
    assert "/api/v1/maps/operations/{operation_id}" in mapping
    assert "ProductControl transaction boundary" in mapping
    assert "metadata.json" in mapping
    assert "occupancy.npz" in mapping
    assert "octomap.ot" in mapping
    assert "/api/v1/map/activate" not in mapping
    assert "/api/v1/session/start" not in mapping
    assert "action=save" not in mapping
    assert "action=set_active" not in mapping
    assert "SAVE_PATH=" not in mapping
    assert "SAVED_MAP_DIR=" in mapping
    assert "resolve_map_root" in mapping
    assert "tomogram.pickle" not in mapping
    assert "LINGTU_SLAM_MAP" not in mapping
    assert "NAV_MAP_DIR" in mapping
    assert "~/data/nova/maps" in mapping
    assert "data/inovxio/data/maps" not in mapping
    assert "/api/v1/navigation/plan" in route_safety
    assert "p0_route_safety" in route_safety
    assert "path_safety" in route_safety
    assert "active_cmd_source" in route_safety
    assert "/api/v1/goal" not in route_safety
    assert "/api/v1/cmd_vel" not in route_safety

    assert 'scripts/lingtu" explore status' in explore
    assert 'scripts/lingtu" explore task start' in explore
    assert 'scripts/lingtu" explore stop' in explore
    assert "curl -sf -X POST http://localhost:5050/api/v1/explore/start" not in explore
    assert "curl -sf -X POST http://localhost:5050/api/v1/explore/stop" not in explore
    assert "explore Product" in explore
    assert "tare_explore" not in explore
    assert "exploring=true" in explore


def test_l2_hook_runs_stub_build_and_offline_start_smoke():
    hook = (REPO_ROOT / "docs/07-testing/install_hooks.sh").read_text(
        encoding="utf-8"
    )

    assert "running stub blueprint smoke" in hook
    assert "lingtu.assembly.profile_builder" in hook
    assert "lingtu.assembly.full_stack" not in hook
    assert "full_stack_blueprint" not in hook
    assert "stub profile build OK" in hook
    assert "stub profile start OK" in hook
    assert "runtime.start()" in hook
    assert "runtime.stop()" in hook
    assert "failed_modules" in hook
    assert "'enable_native': False" in hook
    assert "'enable_gateway': False" in hook
    assert "'enable_map_modules': False" in hook
    assert "'run_startup_checks': False" in hook
    assert "profile='stub'" not in hook
    assert 'profile="stub"' not in hook


def test_l25_fresh_closure_wrapper_enforces_report_age():
    script = (REPO_ROOT / "docs/07-testing/l25_fresh_closure.sh").read_text(
        encoding="utf-8"
    )
    readme = (REPO_ROOT / "docs/07-testing/README.md").read_text(
        encoding="utf-8"
    )
    policy = (REPO_ROOT / "docs/07-testing/COMMIT_PUSH_POLICY.md").read_text(
        encoding="utf-8"
    )

    assert "LINGTU_L25_MAX_REPORT_AGE_S:-21600" in script
    assert "sim/scripts/server_sim_closure.py" in script
    assert 'PYTHON_BIN="${PYTHON:-}"' in script
    assert 'command -v python3' in script
    assert '"$PYTHON_BIN" sim/scripts/server_sim_closure.py' in script
    assert "--max-report-age-s" in script
    assert "--strict" in script
    assert "artifacts/server_sim_closure_summary_all.json" in script
    assert "`l25_fresh_closure.sh` remains a compatibility aggregator" in readme
    assert "LINGTU_L25_MAX_REPORT_AGE_S" in script
    assert "bash docs/07-testing/l25_fresh_closure.sh" in policy


def test_p0_field_runbook_matches_script_contracts():
    p0_all = (REPO_ROOT / "docs/07-testing/p0_all.sh").read_text(
        encoding="utf-8"
    )
    p0_cold_boot = (REPO_ROOT / "docs/07-testing/p0_cold_boot.sh").read_text(
        encoding="utf-8"
    )
    p0_explore = (REPO_ROOT / "docs/07-testing/p0_explore.sh").read_text(
        encoding="utf-8"
    )
    p0_scripts = [
        REPO_ROOT / "docs/07-testing/l25_fresh_closure.sh",
        REPO_ROOT / "docs/07-testing/p0_all.sh",
        REPO_ROOT / "docs/07-testing/p0_cold_boot.sh",
        REPO_ROOT / "docs/07-testing/p0_estop.sh",
        REPO_ROOT / "docs/07-testing/p0_explore.sh",
        REPO_ROOT / "docs/07-testing/p0_goto.sh",
        REPO_ROOT / "docs/07-testing/p0_mapping.sh",
        REPO_ROOT / "docs/07-testing/p0_route_safety.sh",
    ]

    assert 'scripts/lingtu" map start' in p0_cold_boot
    assert 'scripts/lingtu" explore start' in p0_explore
    assert 'scripts/lingtu" explore start --map "$EXPLORE_MAP"' in p0_explore
    assert "confirm_after_preview" not in p0_all
    assert 'run_one "P0-03 route safety"' not in p0_all
    assert 'run_one "P0-03/P0-04 route preview + goto"' in p0_all
    assert 'return "$code"' in p0_all
    assert "LINGTU_P0_GOAL_X" in p0_all
    assert "LINGTU_P0_EXPLORE_MAP" in p0_all
    assert "tare_explore" not in p0_all
    assert "mode switch explore" not in p0_all
    assert "mode switch explore" not in p0_explore
    assert "/api/v1/session/start" not in p0_all
    assert "/api/v1/session/end" not in p0_all
    assert "session_start" not in p0_all
    assert "session_end" not in p0_all
    assert "profile switch" not in p0_all
    assert "fastlio2" not in p0_all
    assert "localizer" not in p0_all

    fail_return_index = p0_all.index('return "$code"')
    goto_index = p0_all.index('run_one "P0-03/P0-04 route preview + goto"')
    assert fail_return_index < goto_index

    for script in p0_scripts:
        script_text = script.read_text(encoding="utf-8")
        script_text.encode("ascii")
        if script.name.startswith("p0_"):
            assert "/api/v1/session/start" not in script_text
            assert "/api/v1/session/end" not in script_text
            assert "/api/v1/map/activate" not in script_text
            assert "sudo systemctl" not in script_text
            assert "systemctl stop" not in script_text
            assert "systemctl start" not in script_text
            if script.name == "p0_mapping.sh":
                assert "tomogram.pickle" not in script_text
