from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_server_setup_runs_multifloor_closure_without_robot_motion():
    script = (REPO_ROOT / "scripts/deploy/setup_server_ros_pct.sh").read_text(
        encoding="utf-8"
    )

    assert 'RUN_MULTIFLOOR="${LINGTU_RUN_MULTIFLOOR:-1}"' in script
    assert 'RUN_NAV_KERNEL="${LINGTU_RUN_NAV_KERNEL:-1}"' in script
    assert 'RUN_ROS2_LOCAL_PLANNER="${LINGTU_RUN_ROS2_LOCAL_PLANNER:-0}"' in script
    assert 'RUN_ROS2_FASTLIO2="${LINGTU_RUN_ROS2_FASTLIO2:-0}"' in script
    assert 'PCT_BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"' in script
    assert 'RUN_ROUTECHECK_PREFLIGHT="${LINGTU_RUN_ROUTECHECK_PREFLIGHT:-1}"' in script
    assert 'SETUP_CLOSURE_MAX_REPORT_AGE_S="${LINGTU_SETUP_CLOSURE_MAX_REPORT_AGE_S:-21600}"' in script
    assert "build_nav_kernel_runtime" in script
    assert 'bash "${ROOT}/scripts/build/build_nav_kernel.sh" --clean' in script
    assert "building nav_kernel nanobind runtime for production local planning" in script
    assert "ros_compat_requested" in script
    assert "skipping legacy ROS2 local_planner build" in script
    assert "LINGTU_RUN_ROS2_LOCAL_PLANNER=1 is deprecated and ignored" in script
    assert "skipping optional ROS2 fastlio2 build" in script
    assert "if ros_compat_requested; then" in script
    assert "packages+=(python3-colcon-common-extensions python3-rosdep)" in script
    assert '|| "${RUN_ROS2_LOCAL_PLANNER}" == "1"' not in script
    assert "--packages-up-to local_planner" not in script
    assert "ros2 pkg executables local_planner" not in script
    assert "verify_ros2_local_planner_setup" not in script
    assert "building PCT Rust GPMP runtime" in script
    assert "scripts/build/build_rust_kernels.py --target gpmp_trajectory_optimizer --release" in script
    assert 'local out_dir="${ROOT}/src/nav/services/plan/global_planner/algorithm/pct/runtime/rust/${arch}"' in script
    assert 'if [[ "${PCT_BUILD_LEGACY_NATIVE}" == "1" ]]; then' in script
    assert "building PCT legacy native/GTSAM runtime for parity baselines" in script
    assert 'LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 \\' in script
    assert "multi-floor exploration/local-planning closure gate" in script
    assert "Gateway routecheck preflight closure gate, non-motion" in script
    assert "sim/scripts/routecheck_preflight_gate.py" in script
    assert "server simulation closure summary for setup-generated gates" in script
    assert "sim/scripts/server_sim_closure.py" in script
    assert "artifacts/server_sim_closure_summary_setup.json" in script
    assert "--max-report-age-s" in script
    assert "--required-only" in script
    assert '"${SETUP_CLOSURE_MAX_REPORT_AGE_S}"' in script
    assert "--route matrix" in script
    assert "--frontier-loop" in script
    assert "--local-planner-backend nanobind" in script
    assert "--require-production-local-planner" in script
    assert "artifacts/server_sim_closure/multifloor_exploration/report.json" in script
    assert "cmd_vel" in script
    assert "physical hardware" in script
    assert 'if [[ "${RUN_PCT}" == "1" ]]; then\n    log "PCT runtime inspection"' in script
    assert 'focused_tests=(\n    src/runtime/tests/test_gateway_commands.py' in script
    assert 'focused_tests+=(src/runtime/tests/test_pct_runtime.py)' in script


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

    assert "/api/v1/maps" in mapping
    assert "action=save" in mapping
    assert "action=set_active" in mapping
    assert "/api/v1/map/save" not in mapping
    assert "/api/v1/map/activate" not in mapping
    assert "SAVE_PATH=" in mapping
    assert 'd.get("map_dir") or d.get("path")' in mapping
    assert "json_payload action=save" in mapping
    assert "json_payload action=set_active" in mapping
    assert 'basename "$ACTIVE_TARGET"' in mapping
    assert "NAV_MAP_DIR" in mapping
    assert "~/data/nova/maps" in mapping
    assert "data/inovxio/data/maps" not in mapping

    assert "/api/v1/navigation/plan" in route_safety
    assert "p0_route_safety" in route_safety
    assert "path_safety" in route_safety
    assert "active_cmd_source" in route_safety
    assert "/api/v1/goal" not in route_safety
    assert "/api/v1/cmd_vel" not in route_safety

    assert "/api/v1/explore/status" in explore
    assert "/api/v1/explore/start" in explore
    assert "/api/v1/explore/stop" in explore
    assert "explore or tare_explore" in explore
    assert "exploring=true" in explore


def test_l2_hook_runs_stub_build_and_offline_start_smoke():
    hook = (REPO_ROOT / "docs/07-testing/install_hooks.sh").read_text(
        encoding="utf-8"
    )

    assert "running stub blueprint smoke" in hook
    assert "runtime.blueprints.profile_builder" in hook
    assert "runtime.blueprints.full_stack" not in hook
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
    assert "bash docs/07-testing/l25_fresh_closure.sh" in readme
    assert "LINGTU_L25_MAX_REPORT_AGE_S" in readme
    assert "bash docs/07-testing/l25_fresh_closure.sh" in policy


def test_p0_field_runbook_matches_script_contracts():
    readme = (REPO_ROOT / "docs/07-testing/README.md").read_text(
        encoding="utf-8"
    )
    p0_all = (REPO_ROOT / "docs/07-testing/p0_all.sh").read_text(
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

    assert "/api/v1/navigation/status" in readme
    assert "/api/v1/navigation/plan" in readme
    assert "path_safety.ok=true" in readme
    assert "/api/v1/session/start" in readme
    assert "LINGTU_P0_GOAL_X" in readme
    assert "type `RUN`" in readme
    assert "POST /api/v1/stop" in readme
    assert "/api/v1/state" in readme
    assert "/api/v1/explore/status" in readme
    assert "LINGTU_P0_RUN_EXPLORE=1" in readme
    assert "four scripts" not in readme
    assert "P0-03 no-motion route safety preview" in readme
    assert "P0-06 exploration start/stop" in readme
    assert "VelocityMux` outputs `Twist.zero()`" not in readme
    assert "watchdog log entry" not in readme
    assert "session_start \"mapping\" \"fastlio2\"" in p0_all
    assert "session_start \"navigating\" \"localizer\" \"$MAP_NAME\"" in p0_all
    assert "confirm_after_preview" in p0_all
    assert 'return "$code"' in p0_all
    assert "LINGTU_P0_GOAL_X" in p0_all
    assert "explore/tare_explore" in p0_all

    fail_return_index = p0_all.index('return "$code"')
    route_index = p0_all.index('run_one "P0-03 route safety"')
    confirm_index = p0_all.index("\nconfirm_after_preview\n")
    goto_index = p0_all.index('run_one "P0-04 goto"')
    assert fail_return_index < route_index
    assert route_index < confirm_index < goto_index

    for script in p0_scripts:
        script.read_text(encoding="utf-8").encode("ascii")
