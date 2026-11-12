"""Read-only discovery and snapshots from the same sources used by conversation tools."""

import asyncio

from fastapi import HTTPException, Query
from fastapi.responses import FileResponse, StreamingResponse

from gateway.services import robot_tools


def register_observation_routes(app, gw):
    @app.get("/api/v1/robot/capabilities")
    async def robot_capabilities():
        return await robot_tools.invoke(gw, "get_robot_capabilities", {})

    @app.get("/api/v1/robot/sensors/{sensor}")
    async def robot_sensor(sensor: str):
        if sensor not in robot_tools.SENSORS:
            raise HTTPException(404, detail="Unknown robot sensor")
        result = await robot_tools.invoke(gw, "read_robot_sensor", {"sensor": sensor})
        if not result["success"]:
            raise HTTPException(503, detail=result)
        return result


    async def monitor_call(method, *args):
        try:
            return await asyncio.to_thread(method, *args)
        except FileNotFoundError as error:
            raise HTTPException(404, detail=str(error)) from error
        except ValueError as error:
            raise HTTPException(409, detail=str(error)) from error
        except OSError as error:
            raise HTTPException(503, detail=str(error)) from error

    @app.get("/api/v1/robot/imu/live", summary="Live samples from the selected robot IMU adapter")
    async def imu_live(after: int = Query(default=0, ge=0)):
        return await monitor_call(gw._imu_monitor.snapshot, after)

    @app.post("/api/v1/robot/imu/records", summary="Start durable IMU monitoring record")
    async def start_imu_record():
        return await monitor_call(gw._imu_monitor.start_record)

    @app.get("/api/v1/robot/imu/records", summary="List saved IMU monitoring records")
    async def imu_records():
        return {"records": await monitor_call(gw._imu_monitor.records)}

    @app.post("/api/v1/robot/imu/records/{identity}/stop", summary="Stop exactly this IMU record")
    async def stop_imu_record(identity: str):
        return await monitor_call(gw._imu_monitor.stop_record, identity)

    @app.get("/api/v1/robot/imu/records/{identity}", summary="Review a recorded IMU trace")
    async def imu_record(identity: str, start: float = Query(default=0, ge=0, allow_inf_nan=False),
                         end: float = Query(default=1e12, ge=0, allow_inf_nan=False)):
        if end < start:
            raise HTTPException(422, detail="Record end precedes its start")
        return await monitor_call(gw._imu_monitor.read_record, identity, start, end)

    @app.get("/api/v1/robot/imu/records/{identity}/download", summary="Download complete IMU samples as CSV or JSONL")
    async def imu_download(identity: str, format: str = Query(default="csv", pattern="^(csv|jsonl)$")):
        path = await monitor_call(gw._imu_monitor.path, identity)
        record = await monitor_call(gw._imu_monitor._metadata, path)
        if record["state"] == "recording":
            raise HTTPException(409, detail="Stop this IMU record before downloading")
        if format == "jsonl":
            return FileResponse(path, media_type="application/x-ndjson", filename=identity + ".jsonl")
        return StreamingResponse(gw._imu_monitor.csv(identity), media_type="text/csv; charset=utf-8",
                                 headers={"Content-Disposition": f'attachment; filename="{identity}.csv"'})
