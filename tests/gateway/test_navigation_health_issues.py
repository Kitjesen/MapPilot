from pathlib import Path

from gateway.services.health_issues import navigation_health_issues
from gateway.services.health_issues import ERROR_DEFINITIONS
from gateway.services.health_issues import navigation_health_advisories


def test_slow_motion_worker_is_diagnostic_only():
    snapshot = {
        "control_loop_health": {"ready": True, "healthy": True, "period_ms": 10.0},
        "timing_ms": {"motion_update_last": 12.5, "overrun": 0.0},
    }
    assert navigation_health_issues(snapshot) == []
    advisories = navigation_health_advisories(snapshot)
    assert advisories[0]["code"] == 21001
    assert advisories[0]["name"] == "motion_update_over_period"
    assert advisories[0]["source"] == "navigation.motion_worker"
    assert advisories[0]["impact"] == "diagnostic_only"
    assert advisories[0]["duration_ms"] == 12.5


def test_control_loop_and_input_gate_have_different_motion_impacts():
    issues = navigation_health_issues({
        "control_loop_health": {
            "ready": True,
            "healthy": False,
            "reason": "deadline_miss_ratio_high",
            "period_ms": 10.0,
            "deadline_miss_ratio": 0.08,
            "p95_utilization": 0.91,
        },
        "input_gate": {"ready": False, "reason": "cloud_stale"},
        "control_authority": {"resume_required": True},
    })
    by_code = {issue["code"]: issue for issue in issues}
    assert by_code[21102]["impact"] == "motion_degrading"
    assert by_code[21201]["impact"] == "motion_hold"
    assert by_code[21301]["impact"] == "motion_hold"


def test_estop_is_motion_blocking():
    issues = navigation_health_issues({"control_authority": {"estop_latched": True}})
    assert issues[0]["code"] == 21401
    assert issues[0]["impact"] == "motion_blocking"


def test_motion_worker_failure_is_degrading_until_input_gate_expires():
    issues = navigation_health_issues({
        "frame_gate": {"motion_worker_error": "bad cloud"},
    })
    assert issues[0]["code"] == 21002
    assert issues[0]["impact"] == "motion_degrading"


def test_numeric_registry_is_published_in_the_handbook():
    handbook = (Path(__file__).resolve().parents[2] / "docs" / "error-codes.md").read_text()
    for code in ERROR_DEFINITIONS:
        assert f"| `{int(code)}` |" in handbook
