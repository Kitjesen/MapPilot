"""Calendar dispatch retains native task admission and never replays an occurrence."""

import asyncio
from datetime import date, datetime
from types import SimpleNamespace

import pytest
from fastapi.responses import JSONResponse
from pydantic import ValidationError

from gateway.services.inspection_schedules import (
    BEIJING,
    InspectionScheduler,
    InspectionScheduleRequest,
    InspectionScheduleStore,
    next_due,
)


def stamp(text):
    return datetime.fromisoformat(text).replace(tzinfo=BEIJING).timestamp()


def plan(**changes):
    return {"name": "Morning", "route_id": "route-a", "map_id": "field-map", "revision": 1,
            "start_date": "2026-11-11", "end_date": "2026-11-11", "local_time": "10:00",
            "frequency": "once", "weekdays": [], "timezone": "Asia/Shanghai", "enabled": True, **changes}


def clear_status():
    return {"task": {"state": "IDLE"}, "control": {"authority": "NONE", "resume_required": False},
            "goal_admission": {"state": "ACCEPTING"}, "motion": {"permission": "CLEAR"}}


def scheduler(tmp_path, dispatch, clock, status=None):
    return InspectionScheduler(SimpleNamespace(), dispatch, path=tmp_path / "schedules.db", clock=clock,
                               status=status or clear_status)


def occurrences(service):
    return service.store.snapshot(date(2026, 11, 1), date(2026, 11, 30))["occurrences"]


def test_date_rules_and_inclusive_weekly_range():
    weekly = plan(frequency="weekly", end_date="2026-11-16", weekdays=[0, 4])
    assert next_due(weekly, stamp("2026-11-11T00:00")) == stamp("2026-11-13T10:00")
    assert next_due(weekly, stamp("2026-11-13T10:00:01")) == stamp("2026-11-16T10:00")
    assert next_due(weekly, stamp("2026-11-16T10:00:01")) is None
    for invalid in [plan(end_date="2026-11-10"), plan(frequency="weekly", weekdays=[]),
                    plan(weekdays=[True]), plan(local_time="25:00"), plan(timezone="UTC")]:
        with pytest.raises(ValidationError):
            InspectionScheduleRequest.model_validate(invalid)


def test_once_is_claimed_once_and_deletion_retains_history(tmp_path):
    store = InspectionScheduleStore(tmp_path / "dates.db", clock=lambda: stamp("2026-11-11T09:00"))
    saved = store.save(plan())
    due = store.claim_due(stamp("2026-11-11T10:00"))
    assert len(due) == 1 and not store.claim_due(stamp("2026-11-11T10:00"))
    store.finish(due[0]["id"], "submitted", task_id="native-task")
    store.delete(saved["id"])
    snapshot = store.snapshot(date(2026, 11, 11), date(2026, 11, 11))
    assert snapshot["schedules"] == [] and snapshot["occurrences"][0]["task_id"] == "native-task"


@pytest.mark.asyncio
async def test_due_dispatch_and_same_tick_busy_are_not_queued(tmp_path):
    now = [stamp("2026-11-11T09:00")]
    calls = []

    async def dispatch(body):
        calls.append(body)
        return {"accepted": True, "task_id": "native-task"}

    service = scheduler(tmp_path, dispatch, lambda: now[0])
    service.store.save(plan(name="First"))
    service.store.save(plan(name="Second"))
    now[0] = stamp("2026-11-11T10:00")
    await service.tick()
    await service.tick()
    assert len(calls) == 1 and calls[0].revision == 1
    assert sorted(item["state"] for item in occurrences(service)) == ["rejected", "submitted"]
    assert any(item["reason"] == "inspection_schedule_task_active" for item in occurrences(service))


@pytest.mark.parametrize("axis,value,reason", [
    ("task", {"state": "UNKNOWN", "reason": "navigation_state_stale"}, "navigation_state_stale"),
    ("task", {"state": "EXECUTING"}, "inspection_schedule_task_active"),
    ("control", {"authority": "OPERATOR", "reason": "operator_control_active"}, "operator_control_active"),
    ("control", {"authority": "NONE", "resume_required": True}, "inspection_schedule_control_unavailable"),
    ("motion", {"permission": "ESTOPPED", "reason": "estop_latched"}, "estop_latched"),
])
@pytest.mark.asyncio
async def test_live_admission_refuses_unready_or_occupied_robot(tmp_path, axis, value, reason):
    now = [stamp("2026-11-11T09:00")]
    calls = []
    status = clear_status()
    status[axis] = value

    async def dispatch(body):
        calls.append(body)

    service = scheduler(tmp_path, dispatch, lambda: now[0], lambda: status)
    service.store.save(plan())
    now[0] = stamp("2026-11-11T10:00")
    await service.tick()
    assert calls == [] and occurrences(service)[0]["reason"] == reason


@pytest.mark.asyncio
async def test_restart_misses_past_time_and_keeps_unconfirmed_submission_unknown(tmp_path):
    now = [stamp("2026-11-11T09:00")]

    async def dispatch(body):
        raise AssertionError("restart must not submit")

    service = scheduler(tmp_path, dispatch, lambda: now[0])
    service.store.save(plan(frequency="daily", end_date="2026-11-13"))
    now[0] = stamp("2026-11-12T10:00:05")
    await service.tick(restarting=True)
    assert [item["state"] for item in occurrences(service)] == ["missed", "missed"]
    pending = service.store.claim_due(stamp("2026-11-13T10:00"))[0]
    reopened = InspectionScheduleStore(service.path, clock=lambda: now[0])
    snapshot = reopened.snapshot(date(2026, 11, 13), date(2026, 11, 13))
    assert snapshot["occurrences"][0]["id"] == pending["id"]
    assert snapshot["occurrences"][0]["state"] == "unknown"


@pytest.mark.asyncio
async def test_late_occurrence_and_changed_claim_never_dispatch(tmp_path):
    now = [stamp("2026-11-11T09:00")]
    calls = []

    async def dispatch(body):
        calls.append(body)

    service = scheduler(tmp_path, dispatch, lambda: now[0])
    saved = service.store.save(plan())
    claimed = service.store.claim_due(stamp("2026-11-11T10:00"))[0]
    disabled = service.store.save(plan(enabled=False), saved["id"])
    assert disabled["next_run_at"] is None
    assert service.store.dispatch_allowed(claimed) is False
    service.store.save(plan(), saved["id"])
    now[0] = stamp("2026-11-11T10:02")
    await service.tick()
    assert calls == []


@pytest.mark.asyncio
async def test_disabling_during_status_probe_prevents_submission(tmp_path):
    now = [stamp("2026-11-11T09:00")]
    calls = []

    async def dispatch(body):
        calls.append(body)

    service = scheduler(tmp_path, dispatch, lambda: now[0])
    saved = service.store.save(plan())

    def status():
        service.store.save(plan(enabled=False), saved["id"])
        return clear_status()

    service.status = status
    now[0] = stamp("2026-11-11T10:00")
    await service.tick()
    assert not calls and occurrences(service)[0]["reason"] == "schedule_changed"


@pytest.mark.asyncio
async def test_unknown_receipt_is_recorded_and_not_retried(tmp_path):
    now = [stamp("2026-11-11T09:00")]
    calls = []

    async def dispatch(body):
        calls.append(body)
        return JSONResponse({"error": "native_transport_unavailable"}, status_code=503)

    service = scheduler(tmp_path, dispatch, lambda: now[0])
    service.store.save(plan())
    now[0] = stamp("2026-11-11T10:00")
    await service.tick()
    await service.tick()
    assert len(calls) == 1 and occurrences(service)[0]["state"] == "unknown"


@pytest.mark.asyncio
async def test_same_instance_restart_closes_pending_claims_without_dispatch(tmp_path):
    now = [stamp("2026-11-11T09:00")]
    calls = []

    async def dispatch(body):
        calls.append(body)

    service = scheduler(tmp_path, dispatch, lambda: now[0])
    service.store.save(plan())
    service.store.save(plan(name="Second"))
    service.store.claim_due(stamp("2026-11-11T10:00"))
    service._stop.set()
    await service._run()
    assert {item["state"] for item in occurrences(service)} == {"unknown"}
    assert calls == []
    service._stop.set()
    await service._run()
    assert {item["state"] for item in occurrences(service)} == {"unknown"}


@pytest.mark.asyncio
async def test_schedule_api_reuses_real_inspection_start_transaction(monkeypatch, tmp_path):
    from tests.gateway.test_gateway_inspection_routes import _client, _route

    monkeypatch.setenv("LINGTU_INSPECTION_SCHEDULE_DB_PATH", str(tmp_path / "schedule-api.db"))
    client, native = _client(monkeypatch)
    route = _route()
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    service = client.app.state.gateway._inspection_scheduler
    now = [stamp("2026-11-11T09:00")]
    service.clock = lambda: now[0]
    service.status = clear_status
    response = client.post("/api/v1/inspection/schedules", json=plan())
    assert response.status_code == 201, response.text
    saved = response.json()
    assert client.get("/api/v1/inspection/schedules?start=2026-11-11&end=2026-11-11").json()["scheduler_running"] is False
    now[0] = stamp("2026-11-11T10:00")
    await service.tick()
    record = occurrences(service)[0]
    assert record["state"] == "submitted", record
    assert native.calls[0][0] == "task_start" and native.calls[0][3] == 1
    assert native.calls[0][-1] == record["request_id"]
    assert client.app.state.gateway._recording.starts[0]["task_id"] == record["task_id"]
    wrong = {**plan(), "revision": 2}
    assert client.put("/api/v1/inspection/schedules/" + saved["id"], json=wrong).status_code == 409
    assert client.delete("/api/v1/inspection/schedules/" + saved["id"]).status_code == 200
    assert len(occurrences(service)) == 1


@pytest.mark.asyncio
async def test_real_task_boundary_respects_an_operator_lease(monkeypatch, tmp_path):
    from tests.gateway.test_gateway_inspection_routes import _client, _route

    monkeypatch.setenv("LINGTU_INSPECTION_SCHEDULE_DB_PATH", str(tmp_path / "lease.db"))
    client, native = _client(monkeypatch)
    assert client.post("/api/v1/inspection/routes", json=_route()).status_code == 200
    gw = client.app.state.gateway
    gw._lease = SimpleNamespace(check=lambda _client: False, to_dict=lambda: {"holder": "operator"})
    service = gw._inspection_scheduler
    now = [stamp("2026-11-11T09:00")]
    service.clock = lambda: now[0]
    service.status = clear_status
    assert client.post("/api/v1/inspection/schedules", json=plan()).status_code == 201
    now[0] = stamp("2026-11-11T10:00")
    await service.tick()
    assert native.calls == [] and not gw._recording.starts
    assert occurrences(service)[0]["reason"] == "control_lease"
