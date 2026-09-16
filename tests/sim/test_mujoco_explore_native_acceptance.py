
from __future__ import annotations

import json
from collections.abc import Sequence
from pathlib import Path
from types import SimpleNamespace

import pytest
from sim.scripts.mujoco import explore_native_acceptance as acceptance
from sim.scripts.mujoco import native_navigation_acceptance as native
from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "config" / "acceptance" / "mujoco" / "explore.json"
MAP_MANIFEST = ROOT / "config" / "acceptance" / "mujoco" / "explore_map.json"
SESSION_ID = "product-explore-test-session"


def _plan(route: str = "live") -> SimpleNamespace:
    requires_map = route == "map"
    return SimpleNamespace(
        product="explore",
        product_variant=route,
        lifecycle={
            "slam_mode": "localization" if requires_map else "mapping",
            "requires_map": requires_map,
        },
        native_process_environment={"LINGTU_DDS_DOMAIN_ID": "17"},
        processes=tuple(
            SimpleNamespace(name=name) for name in acceptance._REQUIRED_PROCESSES
        ),
    )


def _good_evidence(route: str = "live") -> dict[str, object]:
    counters = (
        {
            "plans": 1,
            "goals_accepted": 1,
            "goal_status_messages": 1,
            "segment_requests": 0,
        }
        if route == "map"
        else {
            "plans": 1,
            "segment_requests": 1,
            "segment_ack_messages": 1,
            "segment_status_messages": 1,
        }
    )
    return {
        "control": {"returncode": 0, "stdout": "accepted explore start"},
        "stop": {"returncode": 0, "stdout": "accepted explore stop"},
        "stop_zero": True,
        "product_contract": {"route": route},
        "timeline": [{"route": route, "pending_goal": None, "counters": counters}],
        "nav": {
            "counters": {
                "paths": 1,
                "global_path_points": 2,
                "cmd_vel_published": 4,
            }
        },
        "min_global_path_points": 2,
    }


def test_manifest_matches_map_free_explore_product() -> None:
    manifest = native._load_manifest(MANIFEST)
    evidence = acceptance.product_contract_evidence(manifest)

    assert evidence["ok"] is True
    assert evidence["blockers"] == []
    assert manifest["product_contract"]["route"] == "live"
    assert manifest["product_contract"]["requires_map"] is False


def test_map_manifest_matches_saved_map_explore_product() -> None:
    manifest = native._load_manifest(MAP_MANIFEST)
    evidence = acceptance.product_contract_evidence(manifest)

    assert evidence["ok"] is True
    assert evidence["blockers"] == []
    assert manifest["product_contract"]["route"] == "map"
    assert manifest["product_contract"]["slam_mode"] == "localization"
    assert manifest["product_contract"]["requires_map"] is True


def test_component_manifests_limit_terminal_claims_to_observed_boundaries() -> None:
    acceptance_dir = ROOT / "config" / "acceptance" / "mujoco"
    manifests = {
        name: native._load_manifest(acceptance_dir / name)
        for name in ("navigation.json", "tracking.json", "explore.json", "explore_map.json")
    }
    navigation_claims = manifests["navigation.json"]["acceptance_scope"]["claims"]
    assert any("terminal exact Driver stop" in claim for claim in navigation_claims)
    tracking_scope = manifests["tracking.json"]["acceptance_scope"]
    assert any("terminal navigation state" in claim for claim in tracking_scope["claims"])
    assert all("Driver stop" not in claim for claim in tracking_scope["claims"])
    assert any("Driver stop" in claim for claim in tracking_scope["excluded_claims"])
    for name in ("explore.json", "explore_map.json"):
        assert all(
            "terminal" not in claim
            for claim in manifests[name]["acceptance_scope"]["claims"]
        )


def test_report_keeps_component_evidence_outside_product_pass(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest = native._load_manifest(MANIFEST)
    monkeypatch.setattr(
        acceptance,
        "prepare_runtime",
        lambda _args: {
            "ok": True,
            "blockers": [],
            "manifest": manifest,
            "details": {},
        },
    )
    monkeypatch.setattr(acceptance, "validate_runner_plan", lambda *_args, **_kwargs: _plan())
    monkeypatch.setattr(
        acceptance,
        "run_attached",
        lambda **_kwargs: {"ok": True, "blockers": []},
    )
    report_path = tmp_path / "report.json"
    args = acceptance.build_parser().parse_args(
        [
            "--artifact-dir",
            str(tmp_path),
            "--json-out",
            str(report_path),
            "--run-plan",
            str(tmp_path / "plan.json"),
            "--product-session-id",
            SESSION_ID,
        ]
    )

    report = acceptance.run(args)

    assert json.loads(report_path.read_text(encoding="utf-8"))[
        "product_acceptance_passed"
    ] is False
    assert report["acceptance_scope"]["coverage"] == "component"
    assert report["acceptance_evaluated"] is True
    assert report["evidence_scope"] == "component_e2e"


@pytest.mark.parametrize(("route", "manifest_path"), (("live", MANIFEST), ("map", MAP_MANIFEST)))
def test_committed_run_plan_selects_the_manifest_route(
    route: str,
    manifest_path: Path,
) -> None:
    manifest = native._load_manifest(manifest_path)

    assert acceptance._require_manifest_matches_plan(_plan(route), manifest) == route

    wrong = "map" if route == "live" else "live"
    with pytest.raises(ValueError, match="does not match"):
        acceptance._require_manifest_matches_plan(_plan(wrong), manifest)


def test_live_evaluation_requires_segment_flow_and_zero_stop() -> None:
    good = _good_evidence("live")
    assert acceptance.evaluate_case(good)["ok"] is True

    bad = dict(good)
    bad["timeline"] = [
        {"route": "map", "pending_goal": {"request_id": "wrong"}, "counters": {}}
    ]
    result = acceptance.evaluate_case(bad)

    assert result["ok"] is False
    assert "explore_route_not_live" in result["blockers"]
    assert "live_route_used_generic_goal" in result["blockers"]
    assert "explore_evidence_missing:segment_requests" in result["blockers"]


def test_map_evaluation_requires_goal_flow_and_global_path() -> None:
    good = _good_evidence("map")
    assert acceptance.evaluate_case(good)["ok"] is True

    bad = dict(good)
    bad["timeline"] = [
        {
            "route": "map",
            "counters": {
                "plans": 1,
                "goals_accepted": 0,
                "goal_status_messages": 0,
                "segment_requests": 1,
            },
        }
    ]
    bad["nav"] = {
        "counters": {"paths": 1, "global_path_points": 1, "cmd_vel_published": 4}
    }
    result = acceptance.evaluate_case(bad)

    assert "explore_evidence_missing:goals_accepted" in result["blockers"]
    assert "explore_evidence_missing:goal_status_messages" in result["blockers"]
    assert "map_route_used_live_segment" in result["blockers"]
    assert "native_global_path_points_missing" in result["blockers"]


def test_attached_runner_uses_active_status_and_typed_control(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nav_status = tmp_path / "nav.status.json"
    explore_status = tmp_path / "explore.status.json"
    nav_status.write_text(
        json.dumps(
            {
                "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                "counters": {"paths": 1, "cmd_vel_published": 4},
            }
        ),
        encoding="utf-8",
    )
    explore_status.write_text(
        json.dumps(
            {
                "route": "live",
                "pending_goal": None,
                "counters": {
                    "plans": 1,
                    "segment_requests": 1,
                    "segment_ack_messages": 1,
                    "segment_status_messages": 1,
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        teleop,
        "_ready_path",
        lambda _plan, process, _root: (
            nav_status if process == "nav_runtime" else explore_status
        ),
    )
    calls: list[tuple[str, ...]] = []

    def run_control(_binary: Path, arguments: Sequence[str], **_kwargs: object) -> dict[str, object]:
        calls.append(tuple(arguments))
        return {
            "returncode": 0,
            "stdout": (
                "accepted explore start" if arguments[1] == "start" else "accepted explore stop"
            ),
        }

    monkeypatch.setattr(teleop, "_run_control", run_control)
    clock = iter((0.0, 0.2, 2.0, 3.0, 3.1))
    monkeypatch.setattr(acceptance.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(acceptance.time, "sleep", lambda _seconds: None)
    args = acceptance.build_parser().parse_args(
        ["--artifact-dir", str(tmp_path), "--duration-s", "1", "--domain-id", "17"]
    )
    prepared = {
        "manifest": native._load_manifest(MANIFEST),
        "binaries": {"navigation_control": tmp_path / "lingtu_nav_control.exe"},
    }

    report = acceptance.run_attached(
        plan=_plan(),
        run_plan_path=tmp_path / "plan.json",
        product_session_id=SESSION_ID,
        prepared=prepared,
        args=args,
    )

    assert report["ok"] is True
    assert report["mode"] == "attach_only"
    assert calls[0][:3] == ("explore", "start", SESSION_ID)
    assert calls[1][:2] == ("explore", "stop")
