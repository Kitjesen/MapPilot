"""Loopback HTTP transport for ProductControl, outside the managed Host."""

from __future__ import annotations

import copy
import json
import math
import re
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from lingtu.product_lock import resolve_product_state_dir
from lingtu.switch_contracts import SwitchFailed


class ControlRequestError(ValueError):
    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


class ControlOperations:
    """Retain request receipts while ProductControl restarts the Host."""

    def __init__(self, control: Any, state_dir: Path, *, variant: str | None = None) -> None:
        self.control = control
        self.state_dir = state_dir
        self.variant = variant
        self.path = state_dir / "web-control-operations.json"
        self._lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="product-control")
        self._operations: dict[str, dict[str, Any]] = {}
        if self.path.exists():
            self._operations = json.loads(self.path.read_text(encoding="utf-8"))
            for operation in self._operations.values():
                if operation["state"] == "running":
                    operation.update(
                        state="interrupted",
                        message="控制服务已重启，原切换结果未确认；请核对当前运行模式。",
                    )
            self._persist()

    def _persist(self) -> None:
        self.state_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self._operations, ensure_ascii=False), encoding="utf-8")
        temporary.replace(self.path)

    def snapshot(self) -> dict[str, Any]:
        current = self.control.status(state_dir=self.state_dir)
        with self._lock:
            latest = next(reversed(self._operations.values()), None)
            return {
                "available": True,
                "robot": self.control.robot,
                "env": self.control.env,
                "current": current,
                "operation": copy.deepcopy(latest),
            }

    def get(self, request_id: str) -> dict[str, Any]:
        with self._lock:
            operation = self._operations.get(request_id)
            if operation is None:
                raise ControlRequestError("切换请求不存在", 404)
            return copy.deepcopy(operation)

    def submit(self, body: dict[str, Any]) -> dict[str, Any]:
        request_id = body.get("request_id")
        if not isinstance(request_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", request_id):
            raise ControlRequestError("Invalid request_id")
        product = body.get("product")
        map_name = body.get("map_name")
        expected = body.get("expected_product_session_id")
        if product not in {"map", "nav"}:
            raise ControlRequestError("此入口只支持建图和导航")
        if product == "nav" and (not isinstance(map_name, str) or not map_name.strip()):
            raise ControlRequestError("导航需要选择已保存地图")
        if product == "map" and map_name is not None:
            raise ControlRequestError("新建图不能绑定保存地图")
        if not isinstance(expected, str):
            raise ControlRequestError("缺少当前产品会话，请刷新状态")
        request = {
            "request_id": request_id,
            "product": product,
            "map_name": map_name.strip() if isinstance(map_name, str) else None,
            "expected_product_session_id": expected,
        }
        pose = body.get("initial_pose")
        if pose is not None:
            if product != "nav":
                raise ControlRequestError("初始位姿只用于保存地图导航")
            if not isinstance(pose, dict) or set(pose) != {"x", "y", "z", "yaw"}:
                raise ControlRequestError("初始位姿需要 x、y、z 和 yaw")
            if any(type(value) not in (int, float) or not math.isfinite(value)
                   for value in pose.values()):
                raise ControlRequestError("初始位姿必须是有限数值")
            request["initial_pose"] = dict(pose)
        with self._lock:
            previous = self._operations.get(request_id)
            if previous:
                if previous["request"] != request:
                    raise ControlRequestError("同一请求编号不能用于不同切换", 409)
                return copy.deepcopy(previous)
            if any(op["state"] == "running" for op in self._operations.values()):
                raise ControlRequestError("已有模式切换正在执行", 409)
            operation = {
                "request_id": request_id,
                "request": request,
                "state": "running",
                "created_at": time.time(),
                "message": "正在启动建图" if product == "map" else "正在加载地图并定位",
                "result": None,
            }
            self._operations[request_id] = operation
            while len(self._operations) > 32:
                del self._operations[next(iter(self._operations))]
            try:
                self._persist()
            except OSError:
                del self._operations[request_id]
                raise
            self._pool.submit(self._execute, request)
            return copy.deepcopy(operation)

    def _execute(self, request: dict[str, Any]) -> None:
        result = None
        try:
            pose = request.get("initial_pose")
            localization = {} if pose is None else {
                "initial_pose": tuple(pose[key] for key in ("x", "y", "z", "yaw")),
                "relocalize": True,
            }
            result = self.control.switch(
                request["product"],
                map_name=request["map_name"],
                variant=self.variant,
                expected_product_session_id=request["expected_product_session_id"],
                state_dir=self.state_dir,
                **localization,
            )
            committed = result.get("ok") and result.get("status") in {"active", "already_active"}
            state = "succeeded" if committed else "failed"
            message = "模式切换完成" if state == "succeeded" else str(result.get("error") or "切换未完成")
        except SwitchFailed as exc:
            result = exc.report.as_dict()
            state, message = "failed", str(exc)
        except Exception as exc:
            state, message = "failed", str(exc) or type(exc).__name__
        with self._lock:
            self._operations[request["request_id"]].update(
                state=state, message=message, result=result, finished_at=time.time(),
            )
            self._persist()

    def close(self) -> None:
        self._pool.shutdown(wait=True)


class ControlHTTPServer(ThreadingHTTPServer):
    operations: ControlOperations

    def server_bind(self) -> None:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.allow_reuse_address = False
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def create_server(operations: ControlOperations | None, port: int = 5051) -> ControlHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            pass

        def _reply(self, status: int, body: dict[str, Any]) -> None:
            encoded = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(encoded)
            except (BrokenPipeError, ConnectionResetError):
                pass  # The Host can exit before forwarding the durable receipt.

        def do_GET(self) -> None:
            try:
                if self.path == "/status":
                    self._reply(200, server.operations.snapshot())
                elif self.path.startswith("/operations/"):
                    self._reply(200, server.operations.get(self.path.removeprefix("/operations/")))
                else:
                    self._reply(404, {"message": "Not found"})
            except ControlRequestError as exc:
                self._reply(exc.status, {"message": str(exc)})
            except Exception as exc:
                self._reply(503, {"message": str(exc)})

        def do_POST(self) -> None:
            if self.path != "/switch":
                self._reply(404, {"message": "Not found"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192:
                    raise ControlRequestError("Invalid request length")
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ControlRequestError("Expected a JSON object")
                self._reply(202, server.operations.submit(body))
            except ControlRequestError as exc:
                self._reply(exc.status, {"message": str(exc)})
            except (ValueError, UnicodeError):
                self._reply(400, {"message": "Invalid JSON request"})
            except OSError as exc:
                self._reply(503, {"message": f"无法保存切换请求：{exc}"})

    server = ControlHTTPServer(("127.0.0.1", port), Handler)
    if operations is not None:
        server.operations = operations
    return server


def serve(control: Any, *, state_dir: Path | None = None, port: int = 5051,
          variant: str | None = None) -> None:
    if not control.robot:
        raise ValueError("serve requires a fixed --robot")
    # Bind first: a duplicate process must not rewrite the active server's receipts.
    with create_server(None, port) as server:
        operations = ControlOperations(control, resolve_product_state_dir(state_dir), variant=variant)
        server.operations = operations
        try:
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
        finally:
            operations.close()
