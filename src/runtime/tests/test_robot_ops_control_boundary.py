from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "lingtu"


def _source() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def _function(source: str, name: str) -> str:
    marker = f"{name}() {{"
    body = source.split(marker, 1)[1]
    return body.split("\n}", 1)[0]


def test_robot_ops_uses_env_as_the_outer_runtime_selector() -> None:
    source = _source()
    mode = _function(source, "cmd_mode")

    assert 'LINGTU_ENV="${LINGTU_ENV:-real}"' in source
    assert "--env real|sim" in source
    assert '"$py" -m lingtu.control switch "$target" --env "$LINGTU_ENV"' in mode
    assert "--adapter" not in _function(source, "mode_switch_usage")


def test_runtime_spec_uses_a_profile_adapter_not_a_top_level_endpoint() -> None:
    usage = _function(_source(), "cmd_runtime_spec_usage")

    assert "--adapter PROFILE_ADAPTER" in usage
    assert "--endpoint" not in usage


def test_doctor_bash_wrapper_is_thin_and_read_only() -> None:
    source = _source()
    doctor = _function(source, "cmd_doctor")
    assert '"$py" -m diagnostics.field.doctor' in doctor
    assert '--gateway-url "$GW" --env "$LINGTU_ENV"' in doctor
    for forbidden in (
        "systemctl",
        "journalctl",
        "RunPlan.load",
        "cmd_doctor_json",
        "<<'PY'",
    ):
        assert forbidden not in doctor


def test_svc_mutations_delegate_to_active_product_control() -> None:
    source = _source()
    svc = _function(source, "cmd_svc")

    assert "lingtu_control reapply" in svc
    assert 'lingtu_control restart --process "$target"' in svc
    assert "lingtu_control stop" in svc
    assert "svc_process() {" not in source
    assert "lidar|slam|maps|traversability|nav|driver|camera|explore|host)" in svc
    restart = svc.split("\n        restart)", 1)[1].split("\n        reapply)", 1)[0]
    for rejected in (
        "lingtu-nav-dds.service",
        "fastlio2",
        "localization",
        "nav_dds",
    ):
        assert rejected not in restart
    assert "\n        start)" not in svc
    assert "\n        apply)" not in svc
    assert "start|apply" not in svc
    assert "svc|service)" not in source
    assert "reapply" + "-active" not in svc
    assert "restart" + "-active" not in svc
    assert "stop" + "-active" not in svc
    for forbidden in (
        "systemctl restart",
        "systemctl stop",
        "svc_force_stop_unit",
        "svc_restart_robot_stack",
        "svc_restart_localization_chain",
        "svc_restart_lidar_chain",
    ):
        assert forbidden not in svc


def test_svc_status_uses_the_real_run_plan_process_inventory() -> None:
    svc = _function(_source(), "cmd_svc")

    expected_rows = {
        "lidar": "lingtu-livox-dds.service",
        "slam": "lingtu-slam-dds.service",
        "maps": "mapd.service",
        "traversability": "lingtu-traversability-dds.service",
        "nav": "lingtu-nav-dds.service",
        "driver": "lingtu-driver.service",
        "camera": "lingtu-camera-dds.service",
        "explore": "lingtu-explore-dds.service",
        "host": "lingtu.service",
    }
    for label, unit in expected_rows.items():
        assert f"print_service_row {unit} {label}" in svc


def test_status_and_inspect_delegate_explanation_to_run_plan_owner() -> None:
    source = _source()
    status = _function(source, "cmd_status")
    inspect = _function(source, "cmd_inspect")

    assert '"$py" -m lingtu.explain status --explain' in status
    assert '"$py" -m lingtu.explain inspect "$@" --env "$LINGTU_ENV"' in inspect
    assert "python - <<" not in status
    assert "python - <<" not in inspect


def test_nav_relocalization_is_a_product_switch() -> None:
    source = _source()
    nav = _function(source, "cmd_nav")

    assert 'cmd_mode switch nav --map "$map" --initial-pose "$x" "$y" "$yaw"' in nav
    assert 'cmd_mode switch nav --map "$map" --relocalize' in nav
    assert "/api/v1/slam/relocalize" not in nav
    assert "/api/v1/slam/auto_relocalize" not in nav


def test_removed_bash_lifecycle_helpers_do_not_remain() -> None:
    source = _source()

    assert "/api/v1/slam/switch" not in source
    for helper in (
        "slam_dds_set_mode",
        "slam_dds_map_pcd_for_name",
        "svc_force_stop_unit",
        "svc_start_unit",
        "svc_wait_gateway_ready",
        "svc_wait_unit_active",
        "svc_restart_robot_stack",
        "routecompare_switch_backend",
        "routecompare_wait_terminal",
        "routecompare_cleanup",
        "lingtu_routecheck_cleanup",
        "motion_smoke_cleanup",
    ):
        assert f"{helper}() {{" not in source


def test_obsolete_route_experiment_commands_are_physically_absent() -> None:
    source = _source()

    for removed in (
        "cmd_removed_route_experiment()",
        "cmd_routecheck()",
        "cmd_routecompare()",
        "routecheck|route-check",
        "routecompare|route-compare",
        "super-lio-routecheck",
        "super_lio_routecheck",
        "super-lio-routecompare",
        "super_lio_routecompare",
    ):
        assert removed not in source


def test_obsolete_slam_experiment_commands_are_physically_absent() -> None:
    source = _source()

    for removed in (
        "cmd_removed_slam_experiment()",
        "cmd_slamcheck()",
        "cmd_slamcompare()",
        "slamcheck|slam-check",
        "slamcompare|slam-compare",
        "super-lio-compare",
        "super_lio_compare",
        "super-lio-smoke",
        "super_lio_smoke",
    ):
        assert removed not in source


def test_obsolete_plan_preview_command_is_physically_absent() -> None:
    source = _source()

    for removed in (
        "plan_preview_usage()",
        "cmd_plan_preview()",
        "plan-preview|planpreview|preview-plan",
        '$SCRIPT_DIR/planning/plan_preview.py',
    ):
        assert removed not in source
    assert not (ROOT / "scripts" / "planning" / "plan_preview.py").exists()


def test_motion_and_acceptance_bash_bodies_are_single_gate_delegations() -> None:
    source = _source()

    assert _function(source, "cmd_motion_smoke").strip() == (
        'run_python_gate motion_smoke_gate.py "$@"'
    )
    assert _function(source, "cmd_system_acceptance").strip() == (
        'run_python_gate system_acceptance_gate.py "$@"'
    )


def test_python_gates_own_motion_and_acceptance_orchestration() -> None:
    support = (
        ROOT / "scripts" / "gates" / "field_gate_support.py"
    ).read_text(encoding="utf-8")
    motion = (ROOT / "scripts" / "gates" / "motion_smoke_gate.py").read_text(
        encoding="utf-8"
    )
    acceptance = (
        ROOT / "scripts" / "gates" / "system_acceptance_gate.py"
    ).read_text(encoding="utf-8")

    assert "lingtu.control" in support
    assert "run_product_control" in motion
    assert '"switch", "nav", "--env"' in motion
    assert '"stop-session"' in motion
    assert "motion_smoke_summary.json" in motion
    assert "run_product_control" in acceptance
    assert '"switch", "nav", "--env"' in acceptance
    assert '"stop-session"' in acceptance
    assert 'root / "summary.json"' in acceptance

