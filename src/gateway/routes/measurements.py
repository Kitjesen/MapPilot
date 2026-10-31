"""Live measurement metadata and lossless raw downloads; no control surface."""

from __future__ import annotations

import json
from io import BytesIO
from typing import Any

import numpy as np
from fastapi.responses import JSONResponse, Response

from gateway.services.measurement_preview import measurement_preview
from gateway.services.measurements import measurement_arrays


def register_measurement_routes(app: Any, gw: Any = None) -> None:
    def unavailable():
        return JSONResponse(
            {"error": "measurement_unavailable", "status": "disabled", "sensors": [], "read_only": True},
            status_code=503,
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/v1/measurements", tags=["measurements"])
    def list_measurements():
        if getattr(gw, "_measurements", None) is None:
            return unavailable()
        return JSONResponse(gw._measurements.snapshot(), headers={"Cache-Control": "no-store"})

    def lookup(sensor_id: str):
        if getattr(gw, "_measurements", None) is None:
            return unavailable()
        try:
            return gw._measurements.get(sensor_id)
        except KeyError:
            return JSONResponse({"error": "unknown_sensor", "sensor_id": sensor_id}, status_code=404)

    @app.get("/api/v1/measurements/{sensor_id}", tags=["measurements"])
    def get_measurement(sensor_id: str):
        record = lookup(sensor_id)
        if isinstance(record, Response):
            return record
        return JSONResponse(record[0], headers={"Cache-Control": "no-store"})

    @app.get("/api/v1/measurements/{sensor_id}/raw", tags=["measurements"])
    def get_raw_measurement(sensor_id: str):
        record = lookup(sensor_id)
        if isinstance(record, Response):
            return record
        status, sample = record
        if sample is None or not status["fresh"]:
            return JSONResponse(
                {"error": status["status"], "measurement": status},
                status_code=503 if sample is None else 409,
                headers={"Cache-Control": "no-store"},
            )
        stream = BytesIO()
        payload: dict[str, Any] = dict(measurement_arrays(sample))
        payload["metadata_json"] = np.array(json.dumps(status, allow_nan=False))
        np.savez_compressed(stream, **payload)
        return Response(stream.getvalue(), media_type="application/x-npz", headers={"Cache-Control": "no-store"})

    @app.get("/api/v1/measurements/{sensor_id}/preview", tags=["measurements"])
    def get_measurement_preview(sensor_id: str):
        record = lookup(sensor_id)
        if isinstance(record, Response):
            return record
        status, sample = record
        if sample is None or not status["fresh"]:
            return JSONResponse(
                {"error": status["status"], "measurement": status},
                status_code=503 if sample is None else 409,
                headers={"Cache-Control": "no-store"},
            )
        return JSONResponse(
            {"measurement": status, "display": measurement_preview(sample)},
            headers={"Cache-Control": "no-store"},
        )
