"""Typed readiness for simulation processes."""

from __future__ import annotations

import json
import math
import os
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from lingtu.switch_contracts import is_product_session_id

LOCAL_ENDPOINT_SCHEMA = "lingtu.sim.local_endpoint.v1"
SIM_FEEDER_SCHEMA = "lingtu.sim.feeder_ready.v1"
HOST_READY_SCHEMA = "lingtu.sim.host_ready.v1"
HOST_READY_FILE = "host.ready.json"
NAV_STATUS_SCHEMA = "lingtu.nav.endpoint.status.v1"
SLAM_STATUS_SCHEMA = "lingtu.slam.status_snapshot.v1"
MAPD_STATUS_SCHEMA = "lingtu.maps.runtime.v1"
TRAVERSABILITY_STATUS_SCHEMA = "lingtu.traversability.status.v2"
EXPLORE_STATUS_SCHEMA = "lingtu.explore.status.v2"

_SAFE_BASENAME = re.compile(
    r"[A-Za-z0-9](?:[A-Za-z0-9_.-]{0,126}[A-Za-z0-9])?\Z"
)
_READINESS_CONTRACTS = {
    ("lidar_publisher", "lidar.ready.json"): (
        "local_endpoint",
        LOCAL_ENDPOINT_SCHEMA,
        "lidar_publisher",
        "ltu1-v1",
    ),
    ("imu_publisher", "imu.ready.json"): (
        "local_endpoint",
        LOCAL_ENDPOINT_SCHEMA,
        "imu_publisher",
        "ltu1-v1",
    ),
    ("camera_publisher", "camera.ready.json"): (
        "local_endpoint",
        LOCAL_ENDPOINT_SCHEMA,
        "camera_publisher",
        "ltu1-v1",
    ),
    ("driver_bridge", "driver.ready.json"): (
        "local_endpoint",
        LOCAL_ENDPOINT_SCHEMA,
        "driver_bridge",
        "driver-v2",
    ),
    ("mujoco_feeder", "mujoco_feeder.ready.json"): (
        "sim_feeder",
        SIM_FEEDER_SCHEMA,
        "mujoco_feeder",
        "mujoco-feeder-v1",
    ),
    ("host_runtime", "host.ready.json"): (
        "host",
        HOST_READY_SCHEMA,
        "host",
        "host-process-v1",
    ),
    ("nav_runtime", "nav.status.json"): (
        "nav_status",
        NAV_STATUS_SCHEMA,
        "navd",
        "nav-status-v1",
    ),
    ("slam_runtime", "slam.status.json"): (
        "slam_status",
        SLAM_STATUS_SCHEMA,
        "slamd",
        "slam-status-v1",
    ),
    ("map_runtime", "mapd.status.json"): (
        "mapd_status",
        MAPD_STATUS_SCHEMA,
        "mapd",
        "maps-status-v1",
    ),
    ("traversability_runtime", "traversability.status.json"): (
        "traversability_status",
        TRAVERSABILITY_STATUS_SCHEMA,
        "lingtu_traversability_dds",
        "traversability-status-v2",
    ),
    ("explore_runtime", "explore.status.json"): (
        "explore_status",
        EXPLORE_STATUS_SCHEMA,
        "lingtu_explore_dds",
        "explore-status-v2",
    ),
}
_SIM_FEEDER_ENDPOINTS = (
    {"protocol": "driver-v2", "role": "driver_bridge"},
    {"protocol": "ltu1-v1", "role": "lidar_publisher"},
    {"protocol": "ltu1-v1", "role": "imu_publisher"},
    {"protocol": "ltu1-v1", "role": "camera_publisher"},
)
_SIM_FEEDER_ENDPOINT_OPTIONS = (
    _SIM_FEEDER_ENDPOINTS[:1],
    (_SIM_FEEDER_ENDPOINTS[0], _SIM_FEEDER_ENDPOINTS[3]),
    _SIM_FEEDER_ENDPOINTS[:3],
    _SIM_FEEDER_ENDPOINTS,
)
_MAX_READINESS_BYTES = 16 * 1024
_MAX_NAV_STATUS_BYTES = 8 * 1024 * 1024
_ROTATING_STATUS_ADAPTERS = frozenset(
    {
        "nav_status",
        "slam_status",
        "mapd_status",
        "traversability_status",
        "explore_status",
    }
)
class SimReadinessError(RuntimeError):
    """A typed simulation readiness declaration could not be trusted."""


class SimReadinessPending(SimReadinessError):
    """Trusted readiness evidence describes a process still starting."""


@dataclass(frozen=True)
class ReadinessExpectation:
    """The exact named adapter contract expected for one process."""

    adapter: str
    schema: str
    role: str
    protocol: str

    def __post_init__(self) -> None:
        if any(
            not isinstance(value, str)
            for value in (self.adapter, self.schema, self.role, self.protocol)
        ):
            raise ValueError("readiness expectation is not an approved adapter contract")
        identity = (self.adapter, self.schema, self.role, self.protocol)
        if identity not in _READINESS_CONTRACTS.values():
            raise ValueError("readiness expectation is not an approved adapter contract")


def load_typed_readiness(
    path: str | os.PathLike[str],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
    process: str,
    started_wall_ns: int,
    lidar_required: bool = True,
    imu_required: bool = True,
    camera_required: bool = False,
    expected_control_mode: str | None = None,
    expected_slam_mode: str | None = None,
    expected_explore_route: str | None = None,
) -> Mapping[str, Any]:
    """Load one fresh readiness file and return sanitized typed evidence."""

    _validate_load_inputs(
        expectation=expectation,
        product_session_id=product_session_id,
        product=product,
        process=process,
        started_wall_ns=started_wall_ns,
    )
    if expectation.adapter == "nav_status":
        if (
            not isinstance(expected_control_mode, str)
            or expected_control_mode not in {"autonomy", "teleop", "teleop_avoid"}
        ):
            raise SimReadinessError("nav readiness requires an exact control mode")
    elif expected_control_mode is not None:
        raise SimReadinessError("control mode is only valid for nav readiness")
    if expectation.adapter == "slam_status":
        if expected_slam_mode not in {"mapping", "localization"}:
            raise SimReadinessError("slam readiness requires an exact mode")
    elif expected_slam_mode is not None:
        raise SimReadinessError("slam mode is only valid for slam readiness")
    if expectation.adapter == "explore_status":
        if expected_explore_route not in {"live", "map"}:
            raise SimReadinessError("explore readiness requires an exact route")
    elif expected_explore_route is not None:
        raise SimReadinessError("explore route is only valid for explore readiness")
    candidate = Path(path)
    rotating_status = expectation.adapter in _ROTATING_STATUS_ADAPTERS
    try:
        info = candidate.stat()
        raw = candidate.read_bytes()
    except OSError as exc:
        if rotating_status:
            raise SimReadinessPending(
                "rotating readiness file is temporarily unavailable"
            ) from exc
        raise SimReadinessError("readiness file is unavailable") from exc
    if info.st_mtime_ns < started_wall_ns:
        raise SimReadinessError("readiness file predates the process start")
    limit = _MAX_NAV_STATUS_BYTES if rotating_status else _MAX_READINESS_BYTES
    if len(raw) > limit:
        raise SimReadinessError("readiness file exceeds the byte limit")
    try:
        payload = _decode_json(raw)
    except SimReadinessError as exc:
        if rotating_status:
            raise SimReadinessPending("readiness file is still being published") from exc
        raise
    if expectation.adapter == "local_endpoint":
        _validate_local_endpoint(
            payload,
            expectation,
            product_session_id,
            readiness_name=candidate.name,
        )
    elif expectation.adapter == "sim_feeder":
        _validate_sim_feeder(
            payload,
            expectation=expectation,
            product_session_id=product_session_id,
            product=product,
            process=process,
            lidar_required=lidar_required,
            imu_required=imu_required,
            camera_required=camera_required,
        )
    elif expectation.adapter == "host":
        _validate_host(
            payload,
            expectation=expectation,
            product_session_id=product_session_id,
            product=product,
            process=process,
        )
    elif expectation.adapter == "nav_status":
        _validate_nav_status(
            payload,
            expectation=expectation,
            product_session_id=product_session_id,
            product=product,
            expected_control_mode=expected_control_mode,
        )
    elif expectation.adapter == "slam_status":
        _validate_slam_status(
            payload,
            expectation=expectation,
            product_session_id=product_session_id,
            product=product,
            expected_mode=expected_slam_mode,
            started_wall_ns=started_wall_ns,
        )
    elif expectation.adapter == "mapd_status":
        _validate_mapd_status(
            payload,
            expectation=expectation,
            product_session_id=product_session_id,
            product=product,
        )
    elif expectation.adapter == "traversability_status":
        _validate_traversability_status(
            payload,
            expectation=expectation,
            product_session_id=product_session_id,
            product=product,
        )
    else:
        if not isinstance(expected_explore_route, str):
            raise SimReadinessError("explore readiness requires an exact route")
        _validate_explore_status(
            payload,
            expectation=expectation,
            product_session_id=product_session_id,
            product=product,
            expected_route=expected_explore_route,
        )
    return _readiness_summary(
        payload,
        expectation=expectation,
        product_session_id=product_session_id,
        product=product,
        process=process,
    )


def readiness_expectation_for_process(
    process: str,
    target: str,
) -> ReadinessExpectation | None:
    """Return the typed readiness adapter for a known Product child."""

    contract = _READINESS_CONTRACTS.get((process, target))
    return ReadinessExpectation(*contract) if contract is not None else None


def publish_host_readiness(
    session_root: str | os.PathLike[str],
    *,
    product_session_id: str,
    product: str,
    process: str,
) -> Path:
    """Publish the ready document owned by the simulation Host process."""

    destination = Path(session_root) / HOST_READY_FILE
    payload = {
        "product_session_id": product_session_id,
        "env": "sim",
        "process": process,
        "product": product,
        "protocol": "host-process-v1",
        "ready": True,
        "role": "host",
        "schema": HOST_READY_SCHEMA,
    }
    temporary = destination.with_name(
        f".{destination.name}.{os.urandom(8).hex()}.tmp"
    )
    try:
        temporary.write_text(
            json.dumps(payload, separators=(",", ":")),
            encoding="utf-8",
        )
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination


def validate_feeder_readiness(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Return one validated MuJoCo feeder readiness document."""
    document = dict(payload) if isinstance(payload, Mapping) else None
    if document is None:
        raise SimReadinessError("simulation feeder readiness must be a mapping")
    expectation = ReadinessExpectation(
        adapter="sim_feeder",
        schema=SIM_FEEDER_SCHEMA,
        role="mujoco_feeder",
        protocol="mujoco-feeder-v1",
    )
    endpoints = document.get("endpoints")
    roles = (
        {
            entry.get("role")
            for entry in endpoints
            if type(entry) is dict and isinstance(entry.get("role"), str)
        }
        if type(endpoints) is list
        else set()
    )
    _validate_sim_feeder(
        document,
        expectation=expectation,
        product_session_id=document.get("product_session_id"),
        product=document.get("product"),
        process=document.get("process"),
        lidar_required="lidar_publisher" in roles,
        imu_required="imu_publisher" in roles,
        camera_required="camera_publisher" in roles,
    )
    return document


def _validate_load_inputs(
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
    process: str,
    started_wall_ns: int,
) -> None:
    if not isinstance(expectation, ReadinessExpectation):
        raise TypeError("expectation must be a ReadinessExpectation")
    if not is_product_session_id(product_session_id):
        raise SimReadinessError("expected product_session_id is invalid")
    for name, value in (("product", product), ("process", process)):
        if not isinstance(value, str) or not value or value != value.strip():
            raise SimReadinessError(f"expected {name} must be non-empty trimmed text")
    if (
        isinstance(started_wall_ns, bool)
        or not isinstance(started_wall_ns, int)
        or started_wall_ns <= 0
    ):
        raise SimReadinessError("started_wall_ns must be a positive integer")


def _decode_json(raw: bytes) -> dict[str, Any]:
    try:
        payload = json.loads(raw)
    except (UnicodeError, ValueError, TypeError) as exc:
        raise SimReadinessError("readiness file is not valid JSON") from exc
    if type(payload) is not dict:
        raise SimReadinessError("readiness JSON must be an object")
    return payload


def _readiness_summary(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
    process: str,
) -> Mapping[str, Any]:
    return {
        "adapter": expectation.adapter,
        "source_schema": expectation.schema,
        "role": expectation.role,
        "protocol": expectation.protocol,
        "product_session_id": product_session_id,
        "product": product,
        "process": process,
        "ready": True,
        "details": _summary_details(payload, expectation.adapter),
    }


def _summary_details(payload: Mapping[str, Any], adapter: str) -> dict[str, Any]:
    if adapter == "local_endpoint":
        return {
            "host": payload["host"],
            "port": payload["port"],
            "auth_file": payload["auth_file"],
        }
    if adapter == "sim_feeder":
        return {
            "endpoints": list(payload["endpoints"]),
            "session_id": payload["session_id"],
        }
    if adapter == "host":
        return {}
    if adapter == "nav_status":
        return {"control_mode": payload["control_mode"]}
    if adapter == "slam_status":
        return {
            "mode": payload["mode"],
            "product_session_id": payload["native_product"]["product_session_id"],
            "saved_map_points": payload["saved_map_points"],
            "map_loaded": payload["map_loaded"],
            "map_odom_tf_valid": bool(
                isinstance(payload.get("map_odom_tf"), Mapping)
                and payload["map_odom_tf"].get("valid") is True
            ),
        }
    if adapter == "mapd_status":
        return {
            "live": payload["live"],
            "reset_epoch": payload["reset_epoch"],
            "observation_sequence": payload["observation_sequence"],
            "processed_observations": payload["processed_observations"],
        }
    if adapter == "explore_status":
        map_identity = payload["map"]
        return {
            "route": payload["route"],
            "map": {
                field: map_identity[field]
                for field in (
                    "frame_id",
                    "map_id",
                    "map_content_epoch",
                    "reset_epoch",
                    "generation",
                    "live",
                )
            },
        }
    return {
        "has_odom": payload["has_odom"],
        "has_map_odom_tf": payload["has_map_odom_tf"],
        "last_points": payload["last_points"],
    }


def _validate_local_endpoint(
    payload: Mapping[str, Any],
    expectation: ReadinessExpectation,
    product_session_id: str,
    *,
    readiness_name: str,
) -> None:
    if (
        payload.get("schema") != expectation.schema
        or payload.get("ready") is not True
        or payload.get("role") != expectation.role
        or payload.get("protocol") != expectation.protocol
        or payload.get("product_session_id") != product_session_id
    ):
        raise SimReadinessError("local endpoint readiness identity is invalid")
    if payload.get("host") != "127.0.0.1":
        raise SimReadinessError("local endpoint host must be 127.0.0.1")
    port = payload.get("port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SimReadinessError("local endpoint port is invalid")
    auth_file = payload.get("auth_file")
    if not isinstance(auth_file, str) or _SAFE_BASENAME.fullmatch(auth_file) is None:
        raise SimReadinessError("local endpoint auth_file must be a safe basename")
    if auth_file == readiness_name:
        raise SimReadinessError("local endpoint readiness and auth files must differ")


def _validate_sim_feeder(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: object,
    product: object,
    process: object,
    lidar_required: bool = True,
    imu_required: bool = True,
    camera_required: bool = False,
) -> None:
    expected_text = {
        "schema": expectation.schema,
        "role": expectation.role,
        "protocol": expectation.protocol,
        "product_session_id": product_session_id,
        "product": product,
        "env": "sim",
        "backend": "mujoco",
        "process": process,
    }
    if any(
        not isinstance(payload.get(field), str) or payload.get(field) != expected
        for field, expected in expected_text.items()
    ):
        raise SimReadinessError("simulation feeder readiness identity is invalid")
    if (
        not payload.get("product_session_id")
        or payload["product_session_id"] != payload["product_session_id"].strip()
        or not payload.get("product")
        or payload["product"] != payload["product"].strip()
        or not payload.get("process")
        or payload["process"] != payload["process"].strip()
    ):
        raise SimReadinessError("simulation feeder readiness identity is invalid")
    if payload.get("ready") is not True or payload.get("first_physics_step_applied") is not True:
        raise SimReadinessError("simulation feeder has not applied its first physics step")
    session_id = payload.get("session_id")
    if not isinstance(session_id, str) or not session_id or session_id != session_id.strip():
        raise SimReadinessError("simulation feeder session_id is invalid")
    for field in ("model_generation", "reset_generation"):
        value = payload.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise SimReadinessError("simulation feeder generation is invalid")
    if not isinstance(lidar_required, bool):
        raise SimReadinessError("simulation feeder sensor requirement is invalid")
    if not isinstance(imu_required, bool) or not isinstance(camera_required, bool):
        raise SimReadinessError("simulation feeder sensor requirement is invalid")
    endpoints = payload.get("endpoints")
    if type(endpoints) is not list or tuple(endpoints) not in _SIM_FEEDER_ENDPOINT_OPTIONS:
        raise SimReadinessError("simulation feeder endpoints are invalid")
    expected = [_SIM_FEEDER_ENDPOINTS[0]]
    for required, endpoint in zip(
        (lidar_required, imu_required, camera_required),
        _SIM_FEEDER_ENDPOINTS[1:],
        strict=True,
    ):
        if required:
            expected.append(endpoint)
    if endpoints != expected:
        raise SimReadinessError("simulation feeder endpoints are invalid")


def _validate_host(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
    process: str,
) -> None:
    expected = {
        "schema": expectation.schema,
        "ready": True,
        "role": expectation.role,
        "protocol": expectation.protocol,
        "product_session_id": product_session_id,
        "product": product,
        "env": "sim",
        "process": process,
    }
    if any(payload.get(field) != value for field, value in expected.items()):
        raise SimReadinessError("Host readiness identity is invalid")


def _validate_nav_status(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
    expected_control_mode: str | None,
) -> None:
    if (
        payload.get("schema_version") != expectation.schema
        or payload.get("endpoint") != expectation.role
        or payload.get("control_mode") != expected_control_mode
    ):
        raise SimReadinessError("nav readiness identity is invalid")
    stamp_s = payload.get("stamp_s")
    if (
        isinstance(stamp_s, bool)
        or not isinstance(stamp_s, (int, float))
        or not math.isfinite(float(stamp_s))
        or float(stamp_s) <= 0.0
    ):
        raise SimReadinessError("nav readiness stamp is invalid")
    native_product = payload.get("native_product")
    if type(native_product) is not dict:
        raise SimReadinessError("nav readiness Product identity is invalid")
    if (
        native_product.get("product") != product
        or native_product.get("product_session_id") != product_session_id
    ):
        raise SimReadinessError("nav readiness Product identity is invalid")
    driver_control = payload.get("driver_control")
    if type(driver_control) is not dict:
        raise SimReadinessError("nav readiness Driver control evidence is invalid")
    driver_values = tuple(
        driver_control.get(field) for field in ("received", "ready", "fresh")
    )
    if any(type(value) is not bool for value in driver_values):
        raise SimReadinessError("nav readiness Driver control evidence is invalid")
    if not all(driver_values):
        raise SimReadinessPending(
            "nav readiness is waiting for fresh Driver control evidence"
        )
    input_gate = payload.get("input_gate")
    if type(input_gate) is not dict:
        raise SimReadinessError("nav readiness input gate evidence is invalid")
    gate_ready = input_gate.get("ready")
    driver_ready = input_gate.get("driver_control_ready")
    gate_reason = input_gate.get("reason")
    if (
        type(gate_ready) is not bool
        or type(driver_ready) is not bool
        or not isinstance(gate_reason, str)
        or not gate_reason
        or gate_reason != gate_reason.strip()
    ):
        raise SimReadinessError("nav readiness input gate evidence is invalid")
    if not gate_ready or not driver_ready:
        raise SimReadinessPending(f"nav readiness input gate is pending: {gate_reason}")
    if gate_reason != "ready":
        raise SimReadinessError("nav readiness input gate evidence is inconsistent")
    loop = payload.get("control_loop_health")
    if type(loop) is not dict:
        raise SimReadinessError("nav readiness control loop evidence is invalid")
    loop_values = tuple(loop.get(field) for field in ("ready", "healthy"))
    if any(type(value) is not bool for value in loop_values):
        raise SimReadinessError("nav readiness control loop evidence is invalid")
    if not all(loop_values):
        raise SimReadinessPending("nav readiness control loop is still starting")


def _validate_slam_status(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
    expected_mode: str | None,
    started_wall_ns: int,
) -> None:
    context = "slam readiness"
    if (
        payload.get("schema_version") != expectation.schema
        or payload.get("source") != "cpp_cyclone_slam"
        or payload.get("backend") != "fastlio2"
        or payload.get("mode") != expected_mode
    ):
        raise SimReadinessError(f"{context} identity is invalid")
    _validate_status_product_identity(
        payload,
        product_session_id=product_session_id,
        product=product,
        context=context,
    )
    written_at_s = _require_status_number(
        payload,
        "snapshot_written_at_s",
        context=context,
        positive=True,
    )
    if written_at_s * 1_000_000_000 < started_wall_ns:
        raise SimReadinessError(f"{context} snapshot predates the process start")
    if not _require_status_bool(payload, "alive", context=context):
        raise SimReadinessPending(f"{context} is still starting")

    imu_input = payload.get("imu_input")
    if type(imu_input) is not dict:
        raise SimReadinessError(f"{context} input/output evidence is invalid")
    accepted_imu = _require_status_int(
        imu_input,
        "accepted_frames",
        context=context,
    )
    rates = tuple(
        _require_status_number(payload, field, context=context)
        for field in ("imu_input_hz", "lidar_input_hz")
    )
    if accepted_imu <= 0 or any(rate <= 0.0 for rate in rates):
        raise SimReadinessPending(f"{context} input/output evidence is pending")
    if not _require_status_bool(payload, "has_odom", context=context):
        raise SimReadinessPending(f"{context} output is pending")
    registered_points = _require_status_int(
        payload,
        "registered_points",
        context=context,
    )
    if registered_points <= 0:
        raise SimReadinessPending(f"{context} output is pending")
    observation_sequence = _require_status_int(
        payload,
        "observation_sequence",
        context=context,
    )
    if observation_sequence <= 0:
        raise SimReadinessPending(f"{context} map observation is pending")
    map_points = _require_status_int(payload, "map_points", context=context)
    if map_points <= 0:
        raise SimReadinessPending(f"{context} map output is pending")

    map_loaded = _require_status_bool(payload, "map_loaded", context=context)
    saved_map_points = _require_status_int(
        payload,
        "saved_map_points",
        context=context,
    )
    if expected_mode == "localization":
        if not map_loaded or saved_map_points <= 0:
            raise SimReadinessPending(f"{context} saved map is pending")
        if payload.get("state") != "TRACKING":
            raise SimReadinessPending(f"{context} tracking is pending")
        if not _valid_map_odom_tf(payload.get("map_odom_tf")):
            raise SimReadinessPending(f"{context} map-odom transform is pending")


def _valid_map_odom_tf(value: object) -> bool:
    return bool(
        type(value) is dict
        and value.get("valid") is True
        and value.get("frame_id") == "map"
        and value.get("child_frame_id") == "odom"
    )


def _require_status_bool(
    payload: Mapping[str, Any],
    field: str,
    *,
    context: str,
) -> bool:
    value = payload.get(field)
    if type(value) is not bool:
        raise SimReadinessError(f"{context} {field} is invalid")
    return value


def _require_status_int(
    payload: Mapping[str, Any],
    field: str,
    *,
    context: str,
    positive: bool = False,
) -> int:
    value = payload.get(field)
    if isinstance(value, bool) or not isinstance(value, int):
        raise SimReadinessError(f"{context} {field} is invalid")
    if value < (1 if positive else 0):
        raise SimReadinessError(f"{context} {field} is invalid")
    return value


def _require_status_number(
    payload: Mapping[str, Any],
    field: str,
    *,
    context: str,
    positive: bool = False,
    minimum: float = 0.0,
) -> float:
    value = payload.get(field)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or (positive and float(value) <= 0.0)
        or (not positive and float(value) < minimum)
    ):
        raise SimReadinessError(f"{context} {field} is invalid")
    return float(value)


def _require_status_text(
    payload: Mapping[str, Any],
    field: str,
    *,
    context: str,
    allow_empty: bool = False,
) -> str:
    value = payload.get(field)
    if (
        not isinstance(value, str)
        or value != value.strip()
        or (not allow_empty and not value)
    ):
        raise SimReadinessError(f"{context} {field} is invalid")
    return value


def _validate_status_product_identity(
    payload: Mapping[str, Any],
    *,
    product_session_id: str,
    product: str,
    context: str,
) -> None:
    native_product = payload.get("native_product")
    if (
        type(native_product) is not dict
        or native_product.get("product") != product
        or native_product.get("product_session_id") != product_session_id
    ):
        raise SimReadinessError(f"{context} Product identity is invalid")


def _validate_mapd_status(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
) -> None:
    context = "mapd readiness"
    if (
        payload.get("schema_version") != expectation.schema
        or payload.get("process") != expectation.role
    ):
        raise SimReadinessError(f"{context} identity is invalid")
    _validate_status_product_identity(
        payload,
        product_session_id=product_session_id,
        product=product,
        context=context,
    )
    status = _require_status_text(payload, "status", context=context)
    errors = tuple(
        _require_status_text(payload, field, context=context, allow_empty=True)
        for field in ("engine_error", "input_error", "output_error")
    )
    if any(errors):
        raise SimReadinessError(f"{context} reports an active error")

    running = _require_status_bool(payload, "running", context=context)
    live = _require_status_bool(payload, "live", context=context)
    ready = _require_status_bool(payload, "ready", context=context)
    capacity_limited = _require_status_bool(
        payload, "capacity_limited", context=context
    )
    publications_ready = _require_status_bool(
        payload, "required_publications_ready", context=context
    )
    current_generation_published = _require_status_bool(
        payload, "current_generation_published", context=context
    )
    if capacity_limited:
        raise SimReadinessError(f"{context} exceeded its runtime capacity")

    reset_epoch = _require_status_int(payload, "reset_epoch", context=context)
    observation_sequence = _require_status_int(
        payload, "observation_sequence", context=context
    )
    generation = _require_status_int(payload, "generation", context=context)
    accepted = _require_status_int(
        payload, "accepted_observations", context=context
    )
    processed = _require_status_int(
        payload, "processed_observations", context=context
    )
    received = _require_status_int(payload, "dds_received", context=context)
    decoded = _require_status_int(payload, "dds_decoded", context=context)
    unhealthy_writers = _require_status_int(
        payload, "dds_unhealthy_writers", context=context
    )
    if unhealthy_writers != 0:
        raise SimReadinessError(f"{context} reports an unhealthy DDS writer")

    if not running:
        raise SimReadinessError(f"{context} process is not running")
    if not live or observation_sequence == 0 or generation == 0:
        if status != "waiting_for_observation":
            raise SimReadinessError(f"{context} startup state is inconsistent")
        raise SimReadinessPending("mapd readiness is waiting for its first observation")
    if reset_epoch == 0:
        raise SimReadinessError(f"{context} live observation identity is invalid")
    if min(accepted, processed, received, decoded) == 0:
        raise SimReadinessPending("mapd readiness is waiting for an accepted observation")
    if (
        not publications_ready
        or not current_generation_published
    ):
        raise SimReadinessPending(
            "mapd readiness is publishing the current generation"
        )
    if not ready or status != "ready":
        raise SimReadinessError(f"{context} ready state is inconsistent")


def _validate_traversability_status(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
) -> None:
    context = "traversability readiness"
    if (
        payload.get("schema_version") != expectation.schema
        or payload.get("endpoint") != expectation.role
    ):
        raise SimReadinessError(f"{context} identity is invalid")
    _validate_status_product_identity(
        payload,
        product_session_id=product_session_id,
        product=product,
        context=context,
    )
    has_odom = _require_status_bool(payload, "has_odom", context=context)
    has_map_odom_tf = _require_status_bool(
        payload, "has_map_odom_tf", context=context
    )
    frame_contract = payload.get("frame_contract")
    if type(frame_contract) is not dict:
        raise SimReadinessError(f"{context} frame contract is invalid")
    for field, expected in (
        ("geometry_frame", "map"),
        ("header_frame", "map"),
    ):
        if frame_contract.get(field) != expected:
            raise SimReadinessError(f"{context} frame contract is invalid")
    odom_frame = _require_status_text(
        frame_contract, "odom_input_frame", context=context, allow_empty=True
    )
    cloud_frame = _require_status_text(
        frame_contract, "cloud_input_frame", context=context, allow_empty=True
    )
    if cloud_frame not in {"", "body"}:
        raise SimReadinessError(f"{context} frame contract is invalid")
    last_error = _require_status_text(
        frame_contract, "last_error", context=context, allow_empty=True
    )

    counters = payload.get("counters")
    safety_grid = payload.get("safety_grid")
    if type(counters) is not dict or type(safety_grid) is not dict:
        raise SimReadinessError(f"{context} counters are invalid")
    odom_count = _require_status_int(counters, "odom", context=context)
    cloud_count = _require_status_int(
        counters, "registered_clouds", context=context
    )
    published = _require_status_int(counters, "published", context=context)
    last_points = _require_status_int(payload, "last_points", context=context)
    observed_cells = _require_status_int(
        safety_grid, "observed_before_overlays_cells", context=context
    )
    if cloud_count > 0 and cloud_frame != "body":
        raise SimReadinessError(f"{context} frame contract is invalid")

    if (
        not has_odom
        or not has_map_odom_tf
        or not odom_frame
        or min(odom_count, cloud_count, published, last_points, observed_cells) == 0
    ):
        raise SimReadinessPending(
            "traversability readiness is waiting for synchronized map-frame input"
        )
    if last_error == "map_odom_tf_stamp_invalid":
        raise SimReadinessPending(
            "traversability readiness is waiting to recover from transient frame input"
        )
    if last_error != "none":
        raise SimReadinessError(
            f"{context} reports a frame error: {last_error or 'empty'}"
        )


def _validate_explore_status(
    payload: Mapping[str, Any],
    *,
    expectation: ReadinessExpectation,
    product_session_id: str,
    product: str,
    expected_route: str,
) -> None:
    context = "explore readiness"
    if (
        payload.get("schema_version") != expectation.schema
        or payload.get("endpoint") != expectation.role
        or payload.get("route") != expected_route
    ):
        raise SimReadinessError(f"{context} identity is invalid")
    if (
        payload.get("product") != product
        or payload.get("product_session_id") != product_session_id
    ):
        raise SimReadinessError(f"{context} Product identity is invalid")
    if not _require_status_bool(payload, "ready", context=context):
        raise SimReadinessPending(f"{context} is waiting for {expected_route} inputs")
    _validate_explore_map(payload.get("map"), route=expected_route, context=context)


def _validate_explore_map(value: object, *, route: str, context: str) -> None:
    if type(value) is not dict:
        raise SimReadinessError(f"{context} map identity is invalid")
    if value.get("frame_id") != "map" or value.get("live") is not (route == "live"):
        raise SimReadinessError(f"{context} map identity is invalid")
    _require_status_int(value, "reset_epoch", context=context, positive=True)
    _require_status_int(value, "generation", context=context, positive=True)
    if route == "live":
        if value.get("map_id") != "" or value.get("map_content_epoch") != 0:
            raise SimReadinessError(f"{context} live map identity is invalid")
        return
    _require_status_text(value, "map_id", context=context)
    _require_status_int(value, "map_content_epoch", context=context, positive=True)


__all__ = [
    "EXPLORE_STATUS_SCHEMA",
    "HOST_READY_FILE",
    "HOST_READY_SCHEMA",
    "LOCAL_ENDPOINT_SCHEMA",
    "MAPD_STATUS_SCHEMA",
    "NAV_STATUS_SCHEMA",
    "SIM_FEEDER_SCHEMA",
    "SLAM_STATUS_SCHEMA",
    "TRAVERSABILITY_STATUS_SCHEMA",
    "ReadinessExpectation",
    "SimReadinessError",
    "SimReadinessPending",
    "load_typed_readiness",
    "publish_host_readiness",
    "readiness_expectation_for_process",
    "validate_feeder_readiness",
]
