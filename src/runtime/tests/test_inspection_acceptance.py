from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]


def _field_check(*, ok: bool = True) -> dict:
    return {
        "schema_version": "lingtu.product_field_check.v1",
        "ok": ok,
        "summary": "PASS" if ok else "FAIL",
        "mode": "field",
        "blockers": [] if ok else ["Real runtime evidence is not passing"],
        "advisories": [],
    }


def _candidate(*, feasible: bool = True) -> dict:
    return {
        "_http_status": 200,
        "ok": True,
        "target": {
            "source": "saved_location",
            "target_type": "saved_location",
            "location_name": "pump_room",
        },
        "preview": {
            "ok": True,
            "feasible": feasible,
            "count": 12 if feasible else 0,
            "planner": "octoplanner3d",
            "reasons": [] if feasible else ["blocked"],
        },
        "reasons": [] if feasible else ["blocked"],
    }


def _build(candidate: dict, *, field_ok: bool = True) -> dict:
    from diagnostics.field.inspection import build_inspection_acceptance

    return build_inspection_acceptance(
        field_check=_field_check(ok=field_ok),
        targets=[{"location_name": "pump_room", "source": "saved_location"}],
        candidates=[candidate],
        locations={"count": 1, "locations": [{"name": "pump_room"}]},
    )


def test_inspection_accepts_feasible_non_motion_preview():
    payload = _build(_candidate())

    assert payload["ok"] is True


def test_inspection_rejects_infeasible_preview():
    payload = _build(_candidate(feasible=False))

    assert payload["ok"] is False
    assert payload["targets"][0]["preview_feasible"] is False
    assert "target pump_room failed: blocked" in payload["blockers"]


def test_inspection_blocks_when_field_is_not_ready():
    payload = _build(_candidate(), field_ok=False)

    assert payload["summary"] == "BLOCKED"
    assert "field: Real runtime evidence is not passing" in payload["blockers"]


def test_inspection_result_does_not_mirror_unrelated_runtime_payloads():
    payload = _build(_candidate())

    assert "locations" not in payload
    assert "frontier_preview" not in payload
    assert "runtime_switch" not in payload
    assert set(payload["evidence"]) == {"field_check"}
