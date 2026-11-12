from __future__ import annotations

import inspect
import json
import socket
import threading
import time
from uuid import uuid4

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.product_control import register_product_control_routes
from lingtu.control import ProductControl
from lingtu.control_server import ControlOperations, ControlRequestError, create_server, public_routes
from lingtu.switch_contracts import ProcessFailed, ProcessReport, SwitchReport


class FakeControl:
    robot = "unitree/go2"
    env = "real"

    def __init__(self):
        self.calls = []
        self.entered = threading.Event()
        self.release = threading.Event()
        self.failure = None

    def status(self, **kwargs):
        return {"status": "active", "product": "map", "product_session_id": "old"}

    def switch(self, product, **kwargs):
        self.calls.append((product, kwargs))
        self.entered.set()
        assert self.release.wait(10), "test did not release the switch"
        if self.failure:
            raise RuntimeError(self.failure)
        return {"ok": True, "status": "active", "product_session_id": "new"}

    def stop(self, **kwargs):
        self.calls.append(("stop", kwargs))
        self.entered.set()
        assert self.release.wait(10), "test did not release the stop"
        if self.failure:
            raise RuntimeError(self.failure)
        return {"ok": True, "status": "stopped", "stop_evidence": {"confirmed": True}}


def request(product="nav", **changes):
    return {
        "request_id": str(uuid4()), "product": product,
        "map_name": "office" if product == "nav" else None,
        "expected_product_session_id": "old", **changes,
    }


def finish(operations, request_id):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        result = operations.get(request_id)
        if result["state"] != "running":
            return result
        time.sleep(0.005)
    pytest.fail("operation did not finish")


def test_repeated_request_executes_once_and_survives_transport_restart(tmp_path):
    control = FakeControl()
    operations = ControlOperations(control, tmp_path, variant="camera")
    body = request()
    try:
        assert operations.submit(body)["state"] == "running"
        assert control.entered.wait(1)
        assert operations.submit(body)["request_id"] == body["request_id"]
        with pytest.raises(ControlRequestError, match="不同切换"):
            operations.submit({**body, "map_name": "other"})
        with pytest.raises(ControlRequestError, match="正在执行"):
            operations.submit(request("map"))
        control.release.set()
        assert finish(operations, body["request_id"])["state"] == "succeeded"
    finally:
        control.release.set()
        operations.close()
    restored = ControlOperations(control, tmp_path)
    try:
        assert restored.submit(body)["state"] == "succeeded"
        assert len(control.calls) == 1
        assert control.calls[0] == ("nav", {
            "map_name": "office", "variant": "camera",
            "expected_product_session_id": "old", "state_dir": tmp_path,
            "initial_pose": None,
        })
    finally:
        restored.close()


def test_interrupted_request_is_never_replayed(tmp_path):
    body = request()
    (tmp_path / "web-control-operations.json").write_text(json.dumps({
        body["request_id"]: {"request_id": body["request_id"], "request": body, "state": "running"},
    }), encoding="utf-8")
    control = FakeControl()
    operations = ControlOperations(control, tmp_path)
    try:
        assert operations.submit(body)["state"] == "interrupted"
        assert control.calls == []
    finally:
        operations.close()


def test_failure_remains_a_failure_in_recovered_receipt(tmp_path):
    control = FakeControl()
    control.failure = "localization_not_ready"
    control.release.set()
    operations = ControlOperations(control, tmp_path)
    try:
        body = request()
        operations.submit(body)
        result = finish(operations, body["request_id"])
        assert result["state"] == "failed"
        assert result["message"] == "localization_not_ready"
        control.failure = None
        seeded = request(initial_pose={"x": 1.0, "y": -2.0, "z": 0.32, "yaw": 1.57})
        operations.submit(seeded)
        assert finish(operations, seeded["request_id"])["state"] == "succeeded"
        assert control.calls[-1][1]["initial_pose"] == (1.0, -2.0, 0.32, 1.57)
        call = inspect.signature(ProductControl.switch).bind(control, "nav", **control.calls[-1][1])
        call.apply_defaults()
        assert call.arguments["relocalize"] is True
        with pytest.raises(ControlRequestError, match="不同切换"):
            operations.submit({**seeded, "initial_pose": {**seeded["initial_pose"], "z": 0.0}})
    finally:
        operations.close()


@pytest.mark.parametrize("changes", [
    {"product": "teleop"}, {"map_name": ""}, {"request_id": "../bad"},
    {"expected_product_session_id": None},
    {"initial_pose": {"x": 1, "y": 2, "yaw": 0}},
    {"initial_pose": {"x": 1, "y": 2, "z": float("nan"), "yaw": 0}},
    {"initial_pose": {"x": 1, "y": 2, "z": 0, "yaw": float("inf")}},
    {"initial_pose": {"x": True, "y": 2, "z": 0, "yaw": 0}},
    {"product": "map", "map_name": None, "initial_pose": {"x": 0, "y": 0, "z": 0, "yaw": 0}},
])
def test_invalid_requests_do_not_switch(tmp_path, changes):
    control = FakeControl()
    operations = ControlOperations(control, tmp_path)
    try:
        with pytest.raises(ControlRequestError):
            operations.submit(request(**changes))
        assert control.calls == []
    finally:
        operations.close()


@pytest.mark.parametrize("product", ["nav", "map"])
def test_gateway_restart_does_not_own_or_cancel_switch(tmp_path, monkeypatch, product):
    control = FakeControl()
    operations = ControlOperations(control, tmp_path)
    server = create_server(operations, 0)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    monkeypatch.setenv("LINGTU_CONTROL_PORT", str(server.server_port))
    monkeypatch.setenv("http_proxy", "http://127.0.0.1:1")
    monkeypatch.setenv("no_proxy", "")

    def gateway():
        app = FastAPI()
        register_product_control_routes(app)
        return TestClient(app)

    body = request(product, map_name="office", initial_pose={"x": 1, "y": 2, "z": 0.3, "yaw": 0.4})
    try:
        with gateway() as client:
            assert client.get("/api/v1/product-control").json()["current"]["product_session_id"] == "old"
            response = client.post("/api/v1/product-control/switch", json=body)
            assert response.status_code == 202
        assert control.entered.wait(1)
        with gateway() as restarted:
            path = f"/api/v1/product-control/operations/{body['request_id']}"
            assert restarted.get(path).json()["state"] == "running"
            control.release.set()
            finish(operations, body["request_id"])
            assert restarted.get(path).json()["state"] == "succeeded"
            assert restarted.post("/api/v1/product-control/switch", json=body).json()["state"] == "succeeded"
        assert len(control.calls) == 1
        assert control.calls[0][0] == product
        assert control.calls[0][1]["map_name"] == "office"
        assert control.calls[0][1]["initial_pose"] == (1, 2, 0.3, 0.4)
    finally:
        control.release.set()
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()
        operations.close()


def test_expected_session_rejects_stale_browser_before_resolution(tmp_path, monkeypatch):
    control = ProductControl(robot="unitree/go2", env="real", process_env={})
    monkeypatch.setattr(control, "_current_plan_and_path", lambda root: (None, None, "new"))
    calls = []
    monkeypatch.setattr(control, "_switch", lambda *args, **kwargs: calls.append(args))
    with pytest.raises(RuntimeError, match="已改变"):
        control.switch("map", state_dir=tmp_path, expected_product_session_id="old")
    assert calls == []


def test_stop_uses_durable_operation_and_existing_owner(tmp_path, monkeypatch):
    control = FakeControl()
    operations = ControlOperations(control, tmp_path)
    server = create_server(operations, 0)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    monkeypatch.setenv("LINGTU_CONTROL_PORT", str(server.server_port))
    body = {"request_id": "stop-1", "expected_product_session_id": "old"}
    app = FastAPI()
    register_product_control_routes(app)
    try:
        with TestClient(app) as http:
            accepted = http.post("/api/v1/product-control/stop", json=body)
            assert accepted.status_code == 202
            assert accepted.json()["state"] == "running"
        assert control.entered.wait(1)
        assert operations.submit(body, action="stop")["state"] == "running"
        with pytest.raises(ControlRequestError, match="正在执行"):
            operations.submit(request())
        with pytest.raises(ControlRequestError, match="不同切换"):
            operations.submit({**body, "expected_product_session_id": "replacement"}, action="stop")
        control.release.set()
        result = finish(operations, "stop-1")
        assert result["state"] == "succeeded"
        assert result["result"]["stop_evidence"] == {"confirmed": True}
        assert control.calls == [("stop", {"expected_product_session_id": "old", "state_dir": tmp_path})]
    finally:
        control.release.set()
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()
        operations.close()
    restored = ControlOperations(control, tmp_path)
    try:
        assert restored.submit(body, action="stop")["state"] == "succeeded"
        assert len(control.calls) == 1
        with pytest.raises(ControlRequestError):
            restored.submit({"request_id": "bad-stop"}, action="stop")
    finally:
        restored.close()


def test_stop_failure_is_retained_without_claiming_success(tmp_path):
    control = FakeControl()
    control.release.set()
    control.failure = "current Product session does not match"
    operations = ControlOperations(control, tmp_path)
    try:
        operations.submit({"request_id": "stop-failed", "expected_product_session_id": "old"}, action="stop")
        result = finish(operations, "stop-failed")
        assert result["state"] == "failed"
        assert result["message"] == control.failure
        assert result["result"] is None
    finally:
        operations.close()


def test_public_control_preserves_errors_and_receipts(tmp_path):
    control = FakeControl()
    control.release.set()
    operations = ControlOperations(control, tmp_path)
    app = FastAPI()
    app.include_router(public_routes(operations))
    prefix = "/api/v1/product-control"
    try:
        with TestClient(app) as client:
            status = client.get(prefix)
            assert status.status_code == 200
            assert status.headers["cache-control"] == "no-store"
            assert client.get(f"{prefix}/operations/missing").status_code == 404
            for content in [b"", b"invalid", b"[]", b" " * 8193]:
                assert client.post(f"{prefix}/stop", content=content).status_code == 400
            assert control.calls == []
            body = {"request_id": "public-stop", "expected_product_session_id": "old"}
            assert client.post(f"{prefix}/stop", json=body).status_code == 202
            assert finish(operations, "public-stop")["state"] == "succeeded"
            assert client.get(f"{prefix}/operations/public-stop").json()["state"] == "succeeded"
            assert client.post(f"{prefix}/stop", json=body).json()["state"] == "succeeded"
            assert client.post(f"{prefix}/stop", json={
                **body, "expected_product_session_id": "replacement",
            }).status_code == 409
            assert len(control.calls) == 1
    finally:
        operations.close()


def test_partial_stop_retains_report(tmp_path, monkeypatch):
    control = FakeControl()
    report = ProcessReport(
        product="nav", env="real", action="stop", status="failed",
        stopped=["lt-host.service"], stop_evidence={"host": {"process_stopped": True}},
        error="failed to stop lt-nav.service",
    )

    def stop(**kwargs):
        raise ProcessFailed(report)

    monkeypatch.setattr(control, "stop", stop)
    operations = ControlOperations(control, tmp_path)
    try:
        operations.submit({"request_id": "partial-stop", "expected_product_session_id": "old"}, action="stop")
        result = finish(operations, "partial-stop")
        assert result["state"] == "failed"
        assert result["message"] == report.error
        assert result["result"] == report.as_dict()
    finally:
        operations.close()
    restored = ControlOperations(control, tmp_path)
    try:
        assert restored.get("partial-stop")["result"] == report.as_dict()
    finally:
        restored.close()


def test_matching_session_calls_existing_switch_once(tmp_path, monkeypatch):
    control = ProductControl(robot="unitree/go2", env="real", process_env={})
    monkeypatch.setattr(control, "_current_plan_and_path", lambda root: (None, None, "old"))
    calls = []

    def switch(request, **kwargs):
        calls.append(request)
        return SwitchReport(current_product="nav", target_product="map", env="real",
                            ok=True, status="active", dry_run=False)

    monkeypatch.setattr(control, "_switch", switch)
    assert control.switch("map", state_dir=tmp_path, expected_product_session_id="old")["ok"]
    assert len(calls) == 1


@pytest.mark.parametrize(("env", "public_port", "gateway_port"), [("real", 5050, 5053), ("sim", 15050, 5050)])
def test_serve_cli_uses_fixed_robot_env_and_no_product_side_effect(tmp_path, monkeypatch, env, public_port, gateway_port):
    import lingtu.control_server as module
    from lingtu.control import main

    calls = []
    monkeypatch.setattr(module, "serve", lambda control, **kwargs: calls.append((control, kwargs)))
    assert main(["serve", "--robot", "unitree/go2", "--env", env, "--state-dir", str(tmp_path)]) == 0
    assert len(calls) == 1
    assert calls[0][0].robot == "unitree/go2"
    assert calls[0][1]["state_dir"] == tmp_path
    assert calls[0][1]["public_port"] == public_port
    assert calls[0][1]["gateway_port"] == gateway_port


def test_public_port_conflict_preserves_receipts(tmp_path):
    from lingtu.control_server import serve

    ledger = tmp_path / "web-control-operations.json"
    original = json.dumps({"pending": {"state": "running"}})
    ledger.write_text(original, encoding="utf-8")
    with socket.socket() as occupied:
        occupied.bind(("0.0.0.0", 0))
        occupied.listen()
        with pytest.raises(OSError):
            serve(FakeControl(), state_dir=tmp_path, port=0,
                  public_port=occupied.getsockname()[1], gateway_port=5053)
    assert ledger.read_text(encoding="utf-8") == original


def test_serve_owns_both_listeners(tmp_path, monkeypatch):
    import httpx
    import uvicorn

    import gateway.entry as entry
    import lingtu.control_server as module
    from lingtu.control_server import serve

    ports = {}
    public_sockets = []

    original_create = module.create_server

    def create(operations, port):
        server = original_create(operations, port)
        ports["control"] = server.server_port
        return server

    def app(gateway_port, *, control_routes):
        ports["gateway"] = gateway_port
        application = FastAPI()
        application.include_router(control_routes)
        return application

    def run(_server, *, sockets):
        response = httpx.get(f"http://127.0.0.1:{ports['control']}/status", trust_env=False)
        assert response.status_code == 200
        assert response.json()["current"]["product"] == "map"
        with TestClient(_server.config.app) as client:
            assert client.get("/api/v1/product-control").json() == response.json()
        assert sockets[0].getsockname()[1] != ports["control"]
        public_sockets.extend(sockets)

    monkeypatch.setattr(entry, "build_entry_app", app)
    monkeypatch.setattr(module, "create_server", create)
    monkeypatch.setattr(uvicorn.Server, "run", run)
    serve(FakeControl(), state_dir=tmp_path, port=0, public_port=0, gateway_port=5053)
    assert ports["gateway"] == 5053
    assert public_sockets and all(item.fileno() == -1 for item in public_sockets)


def test_duplicate_server_cannot_rewrite_pending_receipts(tmp_path):
    from lingtu.control_server import serve

    ledger = tmp_path / "web-control-operations.json"
    original = json.dumps({"pending": {"state": "running"}})
    ledger.write_text(original, encoding="utf-8")
    with create_server(None, 0) as running_server:
        with pytest.raises(OSError):
            serve(FakeControl(), state_dir=tmp_path, port=running_server.server_port)
    assert ledger.read_text(encoding="utf-8") == original


def test_unwritable_receipt_does_not_switch_or_leave_queue_busy(tmp_path, monkeypatch):
    control = FakeControl()
    operations = ControlOperations(control, tmp_path)
    body = request()
    original_persist = operations._persist
    try:
        def fail():
            raise OSError("disk full")

        monkeypatch.setattr(operations, "_persist", fail)
        with pytest.raises(OSError, match="disk full"):
            operations.submit(body)
        assert control.calls == []
        monkeypatch.setattr(operations, "_persist", original_persist)
        control.release.set()
        operations.submit(body)
        assert finish(operations, body["request_id"])["state"] == "succeeded"
    finally:
        control.release.set()
        operations.close()
