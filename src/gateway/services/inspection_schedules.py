"""Persistent inspection dates and one-shot dispatch into the existing task boundary."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sqlite3
import threading
import time
import uuid
from datetime import date, datetime, timedelta, timezone
from datetime import time as local_time
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

LOGGER = logging.getLogger(__name__)
BEIJING = timezone(timedelta(hours=8), "Asia/Shanghai")


class InspectionScheduleRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str = Field(min_length=1, max_length=120)
    route_id: str = Field(min_length=1, max_length=128)
    map_id: str = Field(min_length=1, max_length=128)
    revision: int = Field(gt=0, strict=True)
    start_date: date
    end_date: date
    local_time: str = Field(pattern=r"^(?:[01]\d|2[0-3]):[0-5]\d$")
    frequency: Literal["once", "daily", "weekly"] = "once"
    weekdays: list[Annotated[int, Field(ge=0, le=6, strict=True)]] = Field(default_factory=list)
    timezone: Literal["Asia/Shanghai"] = "Asia/Shanghai"
    enabled: bool = Field(default=False, strict=True)

    @model_validator(mode="after")
    def validate_range(self):
        self.name = self.name.strip()
        if not self.name or self.end_date < self.start_date:
            raise ValueError("name and an ordered date range are required")
        self.weekdays = sorted(set(self.weekdays))
        if self.frequency == "weekly" and not self.weekdays:
            raise ValueError("weekly schedules require weekdays")
        if self.frequency == "once" and self.end_date != self.start_date:
            raise ValueError("one-off schedules require one date")
        return self


def next_due(definition: dict, after: float) -> float | None:
    start = date.fromisoformat(definition["start_date"])
    end = date.fromisoformat(definition["end_date"])
    day = max(start, datetime.fromtimestamp(after, BEIJING).date())
    at = local_time.fromisoformat(definition["local_time"])
    while day <= end:
        candidate = datetime.combine(day, at, BEIJING).timestamp()
        selected = definition["frequency"] != "weekly" or day.weekday() in definition["weekdays"]
        if selected and candidate >= after:
            return candidate
        if definition["frequency"] == "once":
            return None
        day += timedelta(days=1)
    return None


def iso(timestamp: float | None) -> str | None:
    return datetime.fromtimestamp(timestamp, BEIJING).isoformat() if timestamp is not None else None


class InspectionScheduleStore:
    def __init__(self, path: str | Path, *, clock=time.time):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.clock = clock
        self.lock = threading.RLock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        with self.db:
            self.db.executescript("""
                CREATE TABLE IF NOT EXISTS schedules (
                    id TEXT PRIMARY KEY, definition TEXT NOT NULL,
                    created_at REAL NOT NULL, updated_at REAL NOT NULL,
                    version INTEGER NOT NULL, next_due REAL
                );
                CREATE TABLE IF NOT EXISTS schedule_occurrences (
                    id TEXT PRIMARY KEY, schedule_id TEXT NOT NULL, scheduled_for REAL NOT NULL,
                    state TEXT NOT NULL, reason TEXT NOT NULL, task_id TEXT NOT NULL,
                    request_id TEXT NOT NULL, UNIQUE(schedule_id,scheduled_for)
                );
            """)
            self.db.execute("UPDATE schedule_occurrences SET state='unknown',reason='host_restarted' WHERE state='dispatching'")

    def interrupt_pending(self, reason: str) -> None:
        with self.lock, self.db:
            self.db.execute("UPDATE schedule_occurrences SET state='unknown',reason=? WHERE state='dispatching'", (reason,))

    @staticmethod
    def public(row) -> dict:
        definition = json.loads(row["definition"])
        return {"id": row["id"], **definition,
                "created_at": row["created_at"], "updated_at": row["updated_at"],
                "next_run_at": iso(row["next_due"]) if definition["enabled"] else None}

    def save(self, definition: dict, schedule_id: str | None = None) -> dict:
        now = self.clock()
        due = next_due(definition, now)
        if definition["enabled"] and due is None:
            raise ValueError("schedule_has_no_future_occurrence")
        with self.lock, self.db:
            if schedule_id is None:
                schedule_id = f"schedule-{uuid.uuid4().hex}"
                self.db.execute("INSERT INTO schedules VALUES (?,?,?,?,?,?)",
                                (schedule_id, json.dumps(definition), now, now, 1, due))
            else:
                if self.db.execute("SELECT 1 FROM schedules WHERE id=?", (schedule_id,)).fetchone() is None:
                    raise KeyError("schedule_not_found")
                self.db.execute("UPDATE schedules SET definition=?,updated_at=?,version=version+1,next_due=? WHERE id=?",
                                (json.dumps(definition), now, due, schedule_id))
            return self.public(self.db.execute("SELECT * FROM schedules WHERE id=?", (schedule_id,)).fetchone())

    def delete(self, schedule_id: str) -> dict:
        with self.lock, self.db:
            if self.db.execute("DELETE FROM schedules WHERE id=?", (schedule_id,)).rowcount == 0:
                raise KeyError("schedule_not_found")
            return {"deleted": True, "id": schedule_id}

    def snapshot(self, start: date, end: date) -> dict:
        beginning = datetime.combine(start, local_time.min, BEIJING).timestamp()
        ending = datetime.combine(end + timedelta(days=1), local_time.min, BEIJING).timestamp()
        with self.lock:
            schedules = [self.public(row) for row in self.db.execute("SELECT * FROM schedules ORDER BY created_at,id")]
            schedules = [item for item in schedules if item["start_date"] <= end.isoformat() and item["end_date"] >= start.isoformat()]
            occurrences = [{**dict(row), "scheduled_for": iso(row["scheduled_for"])} for row in self.db.execute(
                "SELECT * FROM schedule_occurrences WHERE scheduled_for>=? AND scheduled_for<? ORDER BY scheduled_for,id",
                (beginning, ending))]
            return {"timezone": "Asia/Shanghai", "schedules": schedules, "occurrences": occurrences}

    def claim_due(self, now: float, *, restarting: bool = False) -> list[dict]:
        claimed = []
        with self.lock, self.db:
            for row in self.db.execute("SELECT * FROM schedules WHERE next_due<=? ORDER BY next_due,id", (now,)).fetchall():
                definition = json.loads(row["definition"])
                if not definition["enabled"]:
                    continue
                due = row["next_due"]
                while due is not None and due <= now:
                    request_id = f"scheduled-{row['id']}-{int(due)}"
                    missed = restarting or now - due > 60
                    occurrence = {"id": request_id, "schedule_id": row["id"], "scheduled_for": due,
                                  "state": "missed" if missed else "dispatching",
                                  "reason": "scheduler_offline" if restarting else "start_window_missed" if missed else "",
                                  "task_id": "", "request_id": request_id}
                    inserted = self.db.execute("INSERT OR IGNORE INTO schedule_occurrences VALUES (?,?,?,?,?,?,?)",
                                               tuple(occurrence.values())).rowcount
                    if inserted and not missed:
                        claimed.append({**occurrence, "definition": definition, "version": row["version"]})
                    due = next_due(definition, due + 1)
                self.db.execute("UPDATE schedules SET next_due=? WHERE id=?", (due, row["id"]))
        return claimed

    def dispatch_allowed(self, occurrence: dict) -> bool:
        with self.lock:
            row = self.db.execute("SELECT definition,version FROM schedules WHERE id=?", (occurrence["schedule_id"],)).fetchone()
            return bool(row and row["version"] == occurrence["version"] and json.loads(row["definition"])["enabled"])

    def finish(self, occurrence_id: str, state: str, reason: str = "", task_id: str = "") -> None:
        with self.lock, self.db:
            self.db.execute("UPDATE schedule_occurrences SET state=?,reason=?,task_id=? WHERE id=? AND state='dispatching'",
                            (state, reason, task_id, occurrence_id))


def dispatch_rejection(status: dict) -> str | None:
    task = status.get("task", {})
    if task.get("state") not in {"IDLE", "SUCCESS", "FAILED", "CANCELLED"}:
        return str(task.get("reason") or "inspection_schedule_task_active")
    control = status.get("control", {})
    if control.get("resume_required") or control.get("authority") not in {"AUTONOMY", "NONE"}:
        return str(control.get("reason") or "inspection_schedule_control_unavailable")
    admission = status.get("goal_admission", {})
    if admission.get("state") != "ACCEPTING":
        return str(admission.get("reason") or "inspection_schedule_admission_blocked")
    motion = status.get("motion", {})
    if motion.get("permission") != "CLEAR":
        return str(motion.get("reason") or "inspection_schedule_motion_held")
    return None


class InspectionScheduler:
    def __init__(self, gw: Any, dispatch, *, path: str | Path | None = None, clock=time.time, status=None):
        self.gw = gw
        self.dispatch = dispatch
        self.clock = clock
        self.status = status
        root = Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state")
        self.path = path or os.environ.get("LINGTU_INSPECTION_SCHEDULE_DB_PATH") or root / "lingtu" / "inspection-schedules.sqlite3"
        self._store: InspectionScheduleStore | None = None
        self._store_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._task: asyncio.Task | None = None

    @property
    def store(self) -> InspectionScheduleStore:
        with self._store_lock:
            if self._store is None:
                self._store = InspectionScheduleStore(self.path, clock=self.clock)
            return self._store

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and not self._stop.is_set()

    async def tick(self, *, restarting: bool = False) -> None:
        from gateway.navigation.status import build_navigation_status
        from gateway.schemas import InspectionTaskStartRequest

        occupied = False
        for item in self.store.claim_due(self.clock(), restarting=restarting):
            if not self.store.dispatch_allowed(item):
                self.store.finish(item["id"], "rejected", "schedule_changed")
                continue
            try:
                status = self.status() if self.status else build_navigation_status(self.gw)
                reason = "inspection_schedule_task_active" if occupied else dispatch_rejection(status)
                if reason:
                    self.store.finish(item["id"], "rejected", reason)
                    continue
                if not self.store.dispatch_allowed(item):
                    self.store.finish(item["id"], "rejected", "schedule_changed")
                    continue
                definition = item["definition"]
                response = await self.dispatch(InspectionTaskStartRequest(
                    route_id=definition["route_id"], map_id=definition["map_id"],
                    revision=definition["revision"], request_id=item["request_id"]))
                if hasattr(response, "status_code"):
                    payload = json.loads(response.body)
                    reason = str(payload.get("detail", {}).get("native_reason") or payload.get("error") or "inspection_dispatch_failed")
                    state = "rejected" if response.status_code < 500 else "unknown"
                    self.store.finish(item["id"], state, reason)
                    occupied = occupied or state == "unknown"
                else:
                    payload = response.model_dump() if hasattr(response, "model_dump") else response
                    if payload.get("accepted") is True and payload.get("task_id"):
                        self.store.finish(item["id"], "submitted", task_id=payload["task_id"])
                        occupied = True
                    else:
                        self.store.finish(item["id"], "unknown", "inspection_receipt_unconfirmed")
                        occupied = True
            except asyncio.CancelledError:
                self.store.finish(item["id"], "unknown", "scheduler_stopped")
                raise
            except Exception:
                LOGGER.exception("Scheduled inspection dispatch failed")
                self.store.finish(item["id"], "unknown", "inspection_dispatch_unconfirmed")
                occupied = True

    async def _run(self) -> None:
        self.store.interrupt_pending("scheduler_restarted")
        try:
            await self.tick(restarting=True)
            while not self._stop.is_set():
                await asyncio.sleep(1)
                await self.tick()
        finally:
            self.store.interrupt_pending("scheduler_stopped")

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()

        def run():
            self._loop = asyncio.new_event_loop()
            try:
                self._task = self._loop.create_task(self._run())
                self._loop.run_until_complete(self._task)
            except asyncio.CancelledError:
                pass
            except Exception:
                LOGGER.exception("Inspection scheduler stopped")
            finally:
                self._task = None
                self._loop.close()
                self._loop = None

        self._thread = threading.Thread(target=run, name="inspection-schedules", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        loop, task = self._loop, self._task
        if loop is not None and task is not None:
            loop.call_soon_threadsafe(task.cancel)
        if self._thread is not None:
            self._thread.join(timeout=5)
