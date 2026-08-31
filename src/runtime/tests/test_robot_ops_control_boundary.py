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
    switch = _function(source, "cmd_switch")

    assert 'LINGTU_ENV="${LINGTU_ENV:-real}"' in source
    assert 'LINGTU_ROBOT="${LINGTU_ROBOT:-}"' in source
    assert "--env real|sim" in _function(source, "product_switch_usage")
    assert 'lingtu_control switch "$target" --env "$LINGTU_ENV"' in switch
    assert "--adapter" not in _function(source, "product_switch_usage")


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


def test_svc_is_read_only_process_status() -> None:
    source = _source()
    svc = _function(source, "cmd_svc")

    for forbidden in ("lingtu_control", "systemctl restart", "systemctl stop"):
        assert forbidden not in svc
    assert "Usage: lingtu svc status" in svc


def test_svc_status_uses_the_real_run_plan_process_inventory() -> None:
    svc = _function(_source(), "cmd_svc")

    expected_rows = {
        "lidar": "lt-lidar.service",
        "slam": "lt-slam.service",
        "maps": "lt-maps.service",
        "traversability": "lt-terrain.service",
        "nav": "lt-nav.service",
        "driver": "lt-driver.service",
        "camera": "lt-camera.service",
        "explore": "lt-explore.service",
        "host": "lt-host.service",
    }
    for label, unit in expected_rows.items():
        assert f"print_service_row {unit} {label}" in svc


def test_field_cli_does_not_expose_the_retired_explain_surface() -> None:
    source = _source()
    status = _function(source, "cmd_status")

    assert "lingtu.explain" not in source
    assert "\ncmd_inspect() {" not in source
    assert "Usage: lingtu status" in status


def test_nav_relocalization_is_a_product_switch() -> None:
    source = _source()
    nav = _function(source, "cmd_nav")

    assert 'cmd_switch nav --map "$map" --initial-pose "$x" "$y" "$yaw"' in nav
    assert 'cmd_switch nav --map "$map" --relocalize' in nav
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
    assert '"stop"' in motion
    assert "motion_smoke_summary.json" in motion
    assert "run_product_control" in acceptance
    assert '"switch", "nav", "--env"' in acceptance
    assert '"stop"' in acceptance
    assert 'root / "summary.json"' in acceptance

