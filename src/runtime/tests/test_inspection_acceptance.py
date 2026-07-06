from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import json
import sys


def _field_check(*, ok: bool = True) -> dict:
    return {
        "schema_version": "lingtu.product_field_check.v1",
        "ok": ok,
        "summary": "PASS" if ok else "FAIL",
        "mode": "field",
        "blockers": [] if ok else ["Thunder field evidence is not passing"],
        "advisories": [],
    }


def _candidate(*, feasible: bool = True) -> dict:
    return {
        "_http_status": 200,
        "ok": True,
        "status": "preview_feasible" if feasible else "preview_infeasible",
        "target": {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "source": "saved_location",
            "target_type": "saved_location",
            "location_name": "pump_room",
        },
        "preview": {
            "ok": True,
            "feasible": feasible,
            "count": 12 if feasible else 0,
            "planner": "pct",
            "distance_m": 8.5,
            "reasons": [] if feasible else ["blocked"],
        },
        "reasons": [] if feasible else ["blocked"],
    }


def test_inspection_acceptance_passes_field_ready_targets():
    from runtime.diagnostics.inspection_acceptance import build_inspection_acceptance

    field_check = _field_check()
    field_check["frontier_preview"] = {
        "status": "PASS",
        "ok": True,
        "candidate_source": "traversable_frontier",
        "candidate": {
            "source": "traversable_frontier",
            "centroid_3d": [1.0, 2.0, 0.25],
            "reachable_score": 0.82,
            "support_type": "flat",
            "reasons": [],
            "preview": True,
            "command_published": False,
        },
        "command_published": False,
    }
    field_check["runtime_switch"] = {
        "status": "PASS",
        "ok": True,
        "read_only": True,
        "dry_run": True,
        "motion": False,
        "publishes": [],
        "from": {"endpoint": "mujoco_live"},
        "to": {"endpoint": "thunder_field"},
    }

    payload = build_inspection_acceptance(
        field_check=field_check,
        targets=[{"location_name": "pump_room"}],
        candidates=[_candidate()],
        locations={"count": 1, "locations": [{"name": "pump_room"}]},
        gateway_url="http://robot:5050",
    )

    assert payload["ok"] is True
    assert payload["summary"] == "PASS"
    assert payload["target_count"] == 1
    assert payload["targets"][0]["status"] == "PASS"
    assert payload["targets"][0]["non_motion"] is True
    assert payload["targets"][0]["command_published"] is False
    assert payload["frontier_preview"]["status"] == "PASS"
    assert payload["frontier_preview"]["candidate_source"] == "traversable_frontier"
    assert payload["frontier_preview"]["command_published"] is False
    assert payload["runtime_switch"]["status"] == "PASS"
    assert payload["runtime_switch"]["dry_run"] is True
    assert payload["runtime_switch"]["motion"] is False
    assert payload["evidence"]["frontier_preview"]["candidate"]["source"] == (
        "traversable_frontier"
    )
    assert payload["evidence"]["runtime_switch"]["to"]["endpoint"] == "thunder_field"


def test_inspection_acceptance_formats_runtime_switch_preflight():
    from cli.runtime_display import format_inspection_acceptance
    from runtime.diagnostics.inspection_acceptance import build_inspection_acceptance

    field_check = _field_check()
    field_check["runtime_switch"] = {
        "status": "PASS",
        "ok": True,
        "read_only": True,
        "dry_run": True,
        "motion": False,
        "publishes": [],
        "from": {"endpoint": "mujoco_live"},
        "to": {"endpoint": "thunder_field"},
    }

    payload = build_inspection_acceptance(
        field_check=field_check,
        targets=[{"location_name": "pump_room"}],
        candidates=[_candidate()],
        gateway_url="http://robot:5050",
    )

    output = format_inspection_acceptance(payload)

    assert "LingTu Inspection Acceptance: PASS" in output
    assert (
        "Runtime switch: status=PASS dry_run=true motion=false publishes=none "
        "from=mujoco_live to=thunder_field"
    ) in output


def test_inspection_acceptance_blocks_when_field_ready_fails():
    from runtime.diagnostics.inspection_acceptance import build_inspection_acceptance

    payload = build_inspection_acceptance(
        field_check=_field_check(ok=False),
        targets=[{"location_name": "pump_room"}],
        candidates=[_candidate()],
    )

    assert payload["ok"] is False
    assert payload["summary"] == "BLOCKED"
    assert "field: Thunder field evidence is not passing" in payload["blockers"]


def test_inspection_acceptance_fails_infeasible_goal_candidate_preview():
    from runtime.diagnostics.inspection_acceptance import build_inspection_acceptance

    payload = build_inspection_acceptance(
        field_check=_field_check(),
        targets=[{"location_name": "pump_room"}],
        candidates=[_candidate(feasible=False)],
    )

    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert payload["pass_count"] == 0
    assert payload["targets"][0]["status"] == "FAIL"
    assert payload["targets"][0]["preview_feasible"] is False
    assert payload["targets"][0]["command_published"] is False
    assert "blocked" in payload["targets"][0]["reasons"]
    assert "target pump_room failed: blocked" in payload["blockers"]


def test_inspection_acceptance_aggregates_multiple_points_with_motion_safety():
    from runtime.diagnostics.inspection_acceptance import build_inspection_acceptance

    field_check = _field_check()
    field_check["navigation"] = {
        "route_preview": "PASS",
        "route_preview_non_motion": True,
        "route_preview_published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
    }

    payload = build_inspection_acceptance(
        field_check=field_check,
        targets=[
            {"location_name": "pump_room", "source": "saved_location"},
            {"location_name": "valve_bank", "source": "saved_location"},
        ],
        candidates=[_candidate(), _candidate(feasible=False)],
        locations={
            "count": 2,
            "locations": [{"name": "pump_room"}, {"name": "valve_bank"}],
        },
    )

    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert payload["target_count"] == 2
    assert payload["pass_count"] == 1
    assert payload["fail_count"] == 1
    assert all(target["command_published"] is False for target in payload["targets"])
    assert payload["motion_safety"] == {
        "status": "PASS",
        "ok": True,
        "non_motion": True,
        "command_published": False,
        "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
    }
    assert "target valve_bank failed: blocked" in payload["blockers"]


def test_inspection_acceptance_blocks_motion_safety_when_preview_published_command():
    from runtime.diagnostics.inspection_acceptance import build_inspection_acceptance

    field_check = _field_check()
    field_check["navigation"] = {
        "route_preview": "FAIL",
        "route_preview_non_motion": True,
        "route_preview_published": {"goal_pose": 0, "cmd_vel": 1, "stop_cmd": 0},
    }

    payload = build_inspection_acceptance(
        field_check=field_check,
        targets=[{"location_name": "pump_room", "source": "saved_location"}],
        candidates=[_candidate()],
        locations={"count": 1, "locations": [{"name": "pump_room"}]},
    )

    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert payload["pass_count"] == 1
    assert payload["motion_safety"]["status"] == "FAIL"
    assert payload["motion_safety"]["command_published"] is True
    assert payload["motion_safety"]["published"]["cmd_vel"] == 1
    assert "motion safety failed" in payload["blockers"]


def test_inspection_acceptance_blocks_motion_safety_when_publish_counter_missing():
    from runtime.diagnostics.inspection_acceptance import build_inspection_acceptance

    field_check = _field_check()
    field_check["navigation"] = {
        "route_preview": "FAIL",
        "route_preview_non_motion": True,
        "route_preview_published": {"goal_pose": 0, "cmd_vel": 0},
    }

    payload = build_inspection_acceptance(
        field_check=field_check,
        targets=[{"location_name": "pump_room", "source": "saved_location"}],
        candidates=[_candidate()],
        locations={"count": 1, "locations": [{"name": "pump_room"}]},
    )

    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert payload["motion_safety"]["status"] == "FAIL"
    assert payload["motion_safety"]["published"] == {
        "goal_pose": 0,
        "cmd_vel": 0,
        "stop_cmd": None,
    }
    assert "motion safety failed" in payload["blockers"]


def test_inspection_acceptance_fails_missing_requested_saved_location(monkeypatch):
    import runtime.diagnostics.inspection_acceptance as module

    def _fake_field_check(**kwargs):
        return _field_check()

    def _fake_request(base_url, path, *, timeout_sec, body=None):
        if path == "/api/v1/locations":
            return {"_http_status": 200, "count": 0, "locations": []}
        raise AssertionError("missing saved location should not call goal_candidate")

    monkeypatch.setattr(module, "collect_product_field_check", _fake_field_check)
    monkeypatch.setattr(module, "_request_json", _fake_request)

    payload = module.collect_inspection_acceptance(
        gateway_url="http://robot:5050",
        timeout_sec=1.0,
        mode="field",
        points=["missing_name"],
    )

    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert payload["pass_count"] == 0
    assert payload["targets"][0]["name"] == "missing_name"
    assert payload["targets"][0]["status"] == "FAIL"
    assert payload["targets"][0]["reasons"] == ["location_not_found"]
    assert "target missing_name failed: location_not_found" in payload["blockers"]


def test_inspection_acceptance_collects_locations_and_goal_candidates(monkeypatch):
    import runtime.diagnostics.inspection_acceptance as module

    requests: list[tuple[str, dict | None]] = []

    def _fake_field_check(**kwargs):
        assert kwargs["mode"] == "field"
        return _field_check()

    def _fake_request(base_url, path, *, timeout_sec, body=None):
        requests.append((path, dict(body) if body else None))
        if path == "/api/v1/locations":
            return {
                "_http_status": 200,
                "count": 2,
                "locations": [
                    {"name": "pump_room", "x": 1.0, "y": 2.0, "tags": ["daily"]},
                    {"name": "dock", "x": 3.0, "y": 4.0, "tags": ["other"]},
                ],
            }
        assert path == "/api/v1/navigation/goal_candidate"
        assert body["preview"] is True
        assert body["client_id"] == module.DEFAULT_CLIENT_ID
        return _candidate()

    monkeypatch.setattr(module, "collect_product_field_check", _fake_field_check)
    monkeypatch.setattr(module, "_request_json", _fake_request)

    payload = module.collect_inspection_acceptance(
        gateway_url="http://robot:5050",
        timeout_sec=1.0,
        mode="field",
        tag="daily",
    )

    assert payload["ok"] is True
    assert payload["target_count"] == 1
    assert payload["targets"][0]["name"] == "pump_room"
    assert requests[0][0] == "/api/v1/locations"
    assert requests[1] == (
        "/api/v1/navigation/goal_candidate",
        {
            "location_name": "pump_room",
            "label": "pump_room",
            "source": "saved_location",
            "target_type": "saved_location",
            "preview": True,
            "client_id": module.DEFAULT_CLIENT_ID,
        },
    )


def test_inspection_acceptance_rejects_coordinate_payloads_before_goal_candidate(monkeypatch):
    import runtime.diagnostics.inspection_acceptance as module

    requests: list[tuple[str, dict | None]] = []

    def _fake_field_check(**kwargs):
        return _field_check()

    def _fake_request(base_url, path, *, timeout_sec, body=None):
        requests.append((path, dict(body) if body else None))
        if path == "/api/v1/locations":
            return {"_http_status": 200, "count": 0, "locations": []}
        raise AssertionError("invalid inspection point should not call goal_candidate")

    monkeypatch.setattr(module, "collect_product_field_check", _fake_field_check)
    monkeypatch.setattr(module, "_request_json", _fake_request)

    payload = module.collect_inspection_acceptance(
        gateway_url="http://robot:5050",
        timeout_sec=1.0,
        mode="field",
        points=[{"x": 1.0, "y": 2.0, "z": 0.0, "label": "pump"}],
    )

    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert payload["targets"][0]["name"] == "pump"
    assert payload["targets"][0]["target_type"] == "invalid"
    assert payload["targets"][0]["reasons"] == [
        "inspection targets must be saved location names"
    ]
    assert requests == [("/api/v1/locations", None)]


def test_inspection_check_cli_writes_json(monkeypatch, tmp_path, capsys):
    import cli.main as main_mod
    import runtime.diagnostics.inspection_acceptance as module

    out_path = tmp_path / "inspection.json"

    def _fake_collect(**kwargs):
        assert kwargs["points"] == ["pump_room", "valve_bank"]
        assert kwargs["tag"] == "daily"
        return {
            "schema_version": module.INSPECTION_ACCEPTANCE_SCHEMA_VERSION,
            "ok": True,
            "summary": "PASS",
            "field_summary": "PASS",
            "gateway_url": kwargs["gateway_url"],
            "target_count": 2,
            "pass_count": 2,
            "fail_count": 0,
            "targets": [],
            "blockers": [],
            "advisories": [],
        }

    monkeypatch.setattr(module, "collect_inspection_acceptance", _fake_collect)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "inspection-check",
            "--gateway-url",
            "http://robot:5050",
            "--point",
            "pump_room",
            "--point",
            "valve_bank",
            "--tag",
            "daily",
            "--json-out",
            str(out_path),
        ],
    )

    main_mod.main()

    assert capsys.readouterr().out == ""
    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["summary"] == "PASS"
    assert payload["gateway_url"] == "http://robot:5050"
