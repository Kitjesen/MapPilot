"""Keep native OctoPlanner3D defaults aligned with Product intent."""

from __future__ import annotations

from pathlib import Path

from lingtu.assembly.compiler import compile_run_plan

REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SERVICE = REPO_ROOT / "scripts" / "deploy" / "thunder" / "lt-nav.service"
NAV_RUNNER = REPO_ROOT / "scripts" / "deploy" / "thunder" / "run_nav_dds.sh"
NAV_ENDPOINT_CONFIG = REPO_ROOT / "src" / "nav" / "cpp" / "endpoint" / "nav" / "runtime" / "config" / "parse.cpp"

def test_native_navd_consumes_the_compiled_octoplanner_environment() -> None:
    plan = compile_run_plan("nav", "real", robot="unitree/go2")
    env_names = {
        name
        for name in plan.native_process_environment
        if name.startswith("LINGTU_NAV_OCTO_")
    }
    service = NAV_SERVICE.read_text(encoding="utf-8")
    runner = NAV_RUNNER.read_text(encoding="utf-8")
    endpoint_config = NAV_ENDPOINT_CONFIG.read_text(encoding="utf-8")

    assert env_names
    assert "run_nav_dds.sh" in service
    assert "source /opt/lingtu/config/thunder-runtime-env.sh" in runner
    assert 'exec "${LINGTU_NAV_DDS_BIN}"' in runner
    assert "nav_args" not in runner
    assert 'case "${LINGTU_SLAM_MODE}" in' in runner
    assert "mapping|none)" in runner
    for env_name in env_names:
        assert f'"{env_name}"' in endpoint_config
