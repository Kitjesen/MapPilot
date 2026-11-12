"""Calendar management for saved inspection route schedules."""

from __future__ import annotations

import asyncio
from datetime import date, datetime, timedelta

from fastapi import HTTPException

from gateway.services.inspection_schedules import BEIJING, InspectionScheduler, InspectionScheduleRequest


def register_inspection_schedule_routes(app, gw, dispatch, validate_route) -> None:
    scheduler = getattr(gw, "_inspection_scheduler", None)
    if scheduler is None:
        scheduler = InspectionScheduler(gw, dispatch)
        gw._inspection_scheduler = scheduler
    else:
        scheduler.dispatch = dispatch

    @app.get("/api/v1/inspection/schedules")
    async def list_inspection_schedules(start: date | None = None, end: date | None = None):
        first = start or datetime.now(BEIJING).date().replace(day=1)
        last = end or first + timedelta(days=41)
        if last < first:
            raise HTTPException(422, detail={"reason": "schedule_invalid_date_range"})
        result = await asyncio.to_thread(scheduler.store.snapshot, first, last)
        return {**result, "scheduler_running": scheduler.running}

    async def save(body: InspectionScheduleRequest, schedule_id: str | None = None):
        invalid = await validate_route(body)
        if invalid is not None:
            return invalid
        try:
            return await asyncio.to_thread(scheduler.store.save, body.model_dump(mode="json"), schedule_id)
        except KeyError as exc:
            raise HTTPException(404, detail={"reason": "schedule_not_found"}) from exc
        except ValueError as exc:
            raise HTTPException(422, detail={"reason": str(exc)}) from exc

    @app.post("/api/v1/inspection/schedules", status_code=201)
    async def create_inspection_schedule(body: InspectionScheduleRequest):
        return await save(body)

    @app.put("/api/v1/inspection/schedules/{schedule_id}")
    async def update_inspection_schedule(schedule_id: str, body: InspectionScheduleRequest):
        return await save(body, schedule_id)

    @app.delete("/api/v1/inspection/schedules/{schedule_id}")
    async def delete_inspection_schedule(schedule_id: str):
        try:
            return await asyncio.to_thread(scheduler.store.delete, schedule_id)
        except KeyError as exc:
            raise HTTPException(404, detail={"reason": "schedule_not_found"}) from exc
