from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_p0_scripts_use_current_gateway_contracts():
    goto = (REPO_ROOT / "scripts/gates/field/p0_goto.sh").read_text(
        encoding="utf-8"
    )
    estop = (REPO_ROOT / "scripts/gates/field/p0_estop.sh").read_text(
        encoding="utf-8"
    )
    mapping = (REPO_ROOT / "scripts/gates/field/p0_mapping.sh").read_text(
        encoding="utf-8"
    )
    route_safety = (
        REPO_ROOT / "scripts/gates/field/p0_route_safety.sh"
    ).read_text(encoding="utf-8")
    explore = (REPO_ROOT / "scripts/gates/field/p0_explore.sh").read_text(
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

    assert "/api/v1/map/save" in mapping
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
    assert "LINGTU_SLAM_MAP" not in mapping
    assert "NAV_MAP_DIR" in mapping
    assert "/var/lib/lingtu/maps" in mapping
    assert "${MAP_DIR" not in mapping
    assert "~/data/nova/maps" not in mapping
    assert "data/inovxio/data/maps" not in mapping
    assert "/api/v1/navigation/plan" in route_safety
    assert "p0_route_safety" in route_safety
    assert "planner" in route_safety
    assert "path_safety" not in route_safety
    assert "active_cmd_source" in route_safety
    assert "/api/v1/goal" not in route_safety
    assert "/api/v1/cmd_vel" not in route_safety

    assert "curl -sf -X POST http://localhost:5050/api/v1/explore/start" not in explore
    assert "curl -sf -X POST http://localhost:5050/api/v1/explore/stop" not in explore
    assert "explore Product" in explore
    assert "tare_explore" not in explore
    assert "exploring=true" in explore


def test_p0_field_runbook_matches_script_contracts():
    p0_all = (REPO_ROOT / "scripts/gates/field/p0_all.sh").read_text(
        encoding="utf-8"
    )
    p0_explore = (REPO_ROOT / "scripts/gates/field/p0_explore.sh").read_text(
        encoding="utf-8"
    )
    p0_scripts = [
        REPO_ROOT / "scripts/gates/field/p0_all.sh",
        REPO_ROOT / "scripts/gates/field/p0_cold_boot.sh",
        REPO_ROOT / "scripts/gates/field/p0_estop.sh",
        REPO_ROOT / "scripts/gates/field/p0_explore.sh",
        REPO_ROOT / "scripts/gates/field/p0_goto.sh",
        REPO_ROOT / "scripts/gates/field/p0_mapping.sh",
        REPO_ROOT / "scripts/gates/field/p0_route_safety.sh",
    ]

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
