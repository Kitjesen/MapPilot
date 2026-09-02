from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]


def _gateway_acceptance(
    *,
    ok: bool = True,
    readiness_ok: bool = True,
    evidence_ok: bool = True,
    mode: str = "field",
) -> dict:
    blockers = []
    if not readiness_ok:
        blockers.append("Gateway readiness is not passing")
    if not evidence_ok:
        blockers.append("field acceptance requires passing real-runtime-evidence")
    return {
        "ok": ok and readiness_ok and evidence_ok,
        "mode": mode,
        "blockers": blockers,
        "advisories": [],
        "checks": {
            "gateway_contract": {"ok": ok},
            "runtime_mode": {"ok": True, "command_sink": "driver"},
            "gateway_observability": {
                "ok": True,
            },
            "readiness": {"ok": readiness_ok},
            "localization": {"ok": True},
            "navigation": {"can_send_goal": True},
            "real_runtime_evidence": {
                "ok": evidence_ok,
                "data_flow_ok": evidence_ok,
                "cmd_vel_sent_to_hardware": evidence_ok,
                "report_age_s": 12.0,
                "runtime_contract": "real",
            },
        },
    }


def test_field_summary_passes_with_gateway_readiness_and_real_evidence():
    from diagnostics.field.field_check import build_product_field_check

    payload = build_product_field_check(_gateway_acceptance())

    assert payload["ok"] is True
    assert payload["summary"] == "PASS"


def test_field_summary_fails_when_gateway_readiness_fails():
    from diagnostics.field.field_check import build_product_field_check

    payload = build_product_field_check(_gateway_acceptance(readiness_ok=False))

    assert payload["ok"] is False
    assert "Gateway readiness is not passing" in payload["blockers"]


def test_field_summary_fails_without_real_runtime_evidence():
    from diagnostics.field.field_check import build_product_field_check

    payload = build_product_field_check(_gateway_acceptance(evidence_ok=False))

    assert payload["ok"] is False
    assert payload["blockers"] == ["field acceptance requires passing real-runtime-evidence"]


def test_field_summary_reports_saved_map_artifacts():
    from diagnostics.field.field_check import build_product_field_check

    payload = build_product_field_check(
        _gateway_acceptance(),
        map_gate={
            "ok": True,
            "map_id": "active",
            "artifacts": {
                "octomap": {"exists": True, "format_ok": True},
                "occupancy_grid": {"exists": True, "format_ok": True},
            },
            "blockers": [],
        },
    )

    assert payload["map"] == {
        "active": "active",
        "artifacts": "PASS",
        "octomap": "PASS",
        "occupancy": "PASS",
    }


def test_field_summary_reports_driver_command_delivery():
    from diagnostics.field.field_check import build_product_field_check

    payload = build_product_field_check(_gateway_acceptance())

    assert payload["navigation"]["driver_command"] == "PASS"


def test_non_motion_summary_marks_motion_only_checks_unchecked():
    from diagnostics.field.field_check import build_product_field_check

    payload = build_product_field_check(_gateway_acceptance(mode="non_motion"))

    assert payload["summary"] == "PASS"
    assert payload["navigation"]["can_send_goal"] == "UNCHECKED"
    assert payload["navigation"]["driver_command"] == "UNCHECKED"
    assert payload["evidence"]["field_runtime"] == "UNCHECKED"


def test_simulation_summary_only_checks_goal_readiness():
    from diagnostics.field.field_check import build_product_field_check

    payload = build_product_field_check(_gateway_acceptance(mode="simulation"))

    assert payload["navigation"]["can_send_goal"] == "PASS"
    assert payload["navigation"]["driver_command"] == "UNCHECKED"
    assert payload["evidence"]["field_runtime"] == "UNCHECKED"


def test_map_validation_uses_map_id_without_exposing_native_directory(monkeypatch, tmp_path):
    import runtime.endpoints.mapd as mapd_transport
    from diagnostics.field.field_check import validate_map

    other = tmp_path / "mapd-root" / "warehouse"

    class DifferentMapd:
        def service(self, action, **kwargs):
            assert action == "validate_artifacts"
            assert kwargs["map_id"] == "warehouse"
            return {
                "success": True,
                "map_dir": str(other),
                "gate": {"ok": True, "blockers": []},
            }

    monkeypatch.setattr(mapd_transport, "MapClient", DifferentMapd)

    result = validate_map(
        "warehouse",
        require_octomap=False,
        require_occupancy=False,
        expected_data_source=None,
        expected_source_profile=None,
        expected_frame_id=None,
    )

    assert result["ok"] is True
    assert result["map_id"] == "warehouse"
    assert "map_dir" not in result
