"""Public system RPCs retain product state and omit internal diagnostics."""

import copy
import json
import logging
import socket
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest
from fastapi import HTTPException
from google.protobuf.json_format import MessageToDict
from starlette.responses import JSONResponse

from gateway.rpc import codec, services
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import error_pb2 as error_pb
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.server import start

INTERNAL = "INTERNAL_ONLY_MARKER"
CAS = "exact-product-session-token"


@contextmanager
def serve(gateway):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(gateway, address)
    with grpc.insecure_channel(address) as channel:
        try:
            grpc.channel_ready_future(channel).result(timeout=3)
            yield system_rpc.SystemStub(channel)
        finally:
            server.stop(0).wait(3)


@pytest.fixture(autouse=True)
def credentials(monkeypatch):
    monkeypatch.delenv("LINGTU_GRPC_API_KEY", raising=False)
    monkeypatch.delenv("LINGTU_GRPC_REQUIRE_API_KEY", raising=False)
    monkeypatch.delenv("LINGTU_GRPC_TLS_CERT_FILE", raising=False)
    monkeypatch.delenv("LINGTU_GRPC_TLS_KEY_FILE", raising=False)


def public_cases():
    session = {"mode": "localization", "active_map": "yard", "product": "nav", "product_session_id": CAS}
    navigation = {
        "task": {"state": "EXECUTING", "task_id": "task-1", "reason": ""},
        "goal_admission": {"state": "BLOCKED", "reason": "task_active"},
        "control": {"authority": "AUTONOMY", "resume_required": False, "reason": "autonomy_active"},
        "motion": {"permission": "CLEAR", "observation": "UNKNOWN", "stop_confirmation": "UNKNOWN", "reason": ""},
    }
    state = {
        "ts": 123.0,
        "session": session,
        "teleop": {"active": False, "clients": 0},
        "safety": {"level": None, "ok": True, "stop_active": False, "motion_allowed": True},
        "localization": {"state": "tracking", "ready": None, "active_map": "yard", "pose_fresh": False, "reasons": []},
        "navigation": navigation,
    }
    state_source = copy.deepcopy(state)
    state_source.update({key: INTERNAL for key in ("lease", "links", "server", "media", "runtime", "map", "scene", "path")})
    for name in ("session", "teleop", "safety", "localization"):
        state_source[name]["raw"] = {"processes": INTERNAL}
    state_source["navigation"]["endpoint"] = INTERNAL
    for axis in navigation:
        state_source["navigation"][axis]["evidence"] = INTERNAL
    readiness = {
        "status": "degraded", "ready": False, "data_ready": None,
        "non_motion_safe": True, "reasons": ["localization:lost"], "advisories": [],
    }
    health = {
        "status": "degraded", "modules_ok": 1, "modules_fail": 1,
        "has_odom": False, "slam_hz": 0, "map_points": 12,
        "modules": {"driver": "error", "slam": "ok"},
        "sensors": {"lidar": {"status": "unknown", "available": None, "cloud_hz": 0}},
    }
    health_source = copy.deepcopy(health)
    health_source.update({"brainstem": {"driver_status_file": INTERNAL}, "gateway": {"port": INTERNAL}})
    health_source["modules"]["diagnostic"] = {"process": INTERNAL}
    health_source["sensors"]["lidar"].update({"ip": INTERNAL, "path": INTERNAL, "driver": {"arguments": INTERNAL}})
    return [
        ("State", "build_state_snapshot", state_source, state),
        ("Session", None, {**session, "run_plan": INTERNAL, "backend": INTERNAL}, session),
        ("Readiness", "build_readiness_snapshot", {
            **readiness, "runtime": {"endpoints": INTERNAL}, "modules": INTERNAL,
            "product_contract": INTERNAL, "startup_state": INTERNAL,
        }, readiness),
        ("Health", "build_health", health_source, health),
        ("Capabilities", "build_app_capabilities", {
            "features": {"mapping": True, "teleop": False, "runtime_dataflow": True, "sessions": True, "internal": INTERNAL},
            "runtime_products": {"env": INTERNAL, "backend": INTERNAL, "products": {
                "map": {"available": True, "variants": [INTERNAL], "reason": INTERNAL},
                "nav": {"available": None, "run_plan": INTERNAL},
            }},
            "endpoints": INTERNAL, "probes": INTERNAL, "validation_gates": INTERNAL,
            "realtime": INTERNAL, "policy": INTERNAL,
        }, {"features": {"mapping": True, "teleop": False}, "products": {
            "map": {"available": True}, "nav": {"available": None},
        }}),
        ("ProductStatus", "_forward", {
            "available": True,
            "current": {"ok": True, "status": "active", "product": "nav", "product_session_id": CAS,
                        "processes": INTERNAL, "evidence": INTERNAL, "env": INTERNAL, "robot": INTERNAL},
            "operation": INTERNAL, "degraded": INTERNAL,
        }, {"available": True, "current": {"ok": True, "status": "active", "product": "nav", "product_session_id": CAS}}),
    ]


@pytest.mark.parametrize(("method", "owner", "source", "expected"), public_cases())
def test_public_system_snapshots_keep_business_state_without_mutating_diagnostics(monkeypatch, method, owner, source, expected):
    before = copy.deepcopy(source)
    gateway = SimpleNamespace(_session_snapshot=lambda: source)
    if owner is not None:
        def read(*_args):
            if method == "Readiness":
                return source, 503
            if method == "ProductStatus":
                return JSONResponse(source)
            return source

        monkeypatch.setattr(services, owner, read)
    with serve(gateway) as client:
        snapshot = getattr(client, method)(common_pb.Empty(), timeout=3)
    value = MessageToDict(snapshot.fields)
    assert value == expected
    assert INTERNAL not in json.dumps(value)
    assert source == before


@pytest.mark.parametrize("source", [
    {"available": None},
    {"available": False, "current": None},
    {"available": True, "current": {"ok": True, "status": "stopped"}},
])
def test_product_status_preserves_absent_unknown_and_stopped_fields(monkeypatch, source):
    monkeypatch.setattr(services, "_forward", lambda _path: JSONResponse(source))
    with serve(SimpleNamespace()) as client:
        result = client.ProductStatus(common_pb.Empty(), timeout=3)
    assert MessageToDict(result.fields) == source


def test_unexpected_exception_is_private_but_logged_with_traceback(monkeypatch, caplog):
    def failed(_gateway):
        raise RuntimeError(f"cannot open /etc/doso/{INTERNAL}/state.json")

    monkeypatch.setattr(services, "build_state_snapshot", failed)
    with caplog.at_level(logging.ERROR, logger=codec.__name__), serve(SimpleNamespace()) as client:
        with pytest.raises(grpc.RpcError) as error:
            client.State(common_pb.Empty(), timeout=3)
    assert error.value.code() == grpc.StatusCode.INTERNAL
    assert error.value.details() == "Internal service error"
    metadata = dict(error.value.trailing_metadata())
    detail = error_pb.ErrorDetail.FromString(metadata["doso-error-bin"])
    assert detail.reason_code == "internal_error"
    assert MessageToDict(detail.details) == {"reason_code": "internal_error"}
    assert INTERNAL not in str(error.value)
    assert any(record.exc_info and INTERNAL in str(record.exc_info[1]) for record in caplog.records)


def test_owner_http_failure_keeps_its_business_code_and_detail(monkeypatch):
    original = {"reason_code": "owner_access_denied", "message": "Owner access required"}

    def denied(_gateway):
        raise HTTPException(status_code=403, detail=original)

    monkeypatch.setattr(services, "build_state_snapshot", denied)
    with serve(SimpleNamespace()) as client:
        with pytest.raises(grpc.RpcError) as error:
            client.State(common_pb.Empty(), timeout=3)
    assert error.value.code() == grpc.StatusCode.PERMISSION_DENIED
    detail = error_pb.ErrorDetail.FromString(dict(error.value.trailing_metadata())["doso-error-bin"])
    assert detail.reason_code == "owner_access_denied"
    assert MessageToDict(detail.details)["detail"] == original
