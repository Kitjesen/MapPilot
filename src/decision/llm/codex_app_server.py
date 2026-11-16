"""Codex App Server client used by the LingTu Codex provider.

Codex owns authentication and CODEX_HOME. LingTu owns the provider boundary,
conversation persistence, and the robot permission boundary.
"""

from __future__ import annotations

import base64
import json
import os
import queue
import shutil
import subprocess
import tempfile
import threading
import time
from collections.abc import Callable, Iterable
from pathlib import Path

from decision.llm.client import LLMError


class CodexAppServer:
    def __init__(self, command: str | None = None, codex_home: str | None = None):
        self.command = command or os.environ.get("CODEX_BIN") or shutil.which("codex") or "codex"
        self.codex_home = codex_home or os.environ.get("CODEX_HOME", "")
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._messages: queue.Queue[dict] = queue.Queue()
        self._write_lock = threading.Lock()
        self._rpc_lock = threading.RLock()
        self._next_id = 0
        self._initialized = False
        self._login: dict | None = None

    def _start(self) -> None:
        if self._process is not None and self._process.poll() is None:
            return
        env = os.environ.copy()
        if self.codex_home:
            env["CODEX_HOME"] = self.codex_home
        try:
            self._process = subprocess.Popen(
                [self.command, "app-server", "--stdio"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                env=env,
            )
        except OSError as exc:
            raise LLMError(f"无法启动 Codex App Server：{exc}") from exc
        self._messages = queue.Queue()
        self._initialized = False
        self._reader = threading.Thread(target=self._read_output, name="codex-app-server", daemon=True)
        self._reader.start()
        self._request("initialize", {
            "clientInfo": {"name": "lingtu", "title": "LingTu", "version": "2.3.0"},
            "capabilities": {"experimentalApi": True, "optOutNotificationMethods": None},
        })
        self._notify("initialized", {})
        self._initialized = True

    def _read_output(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        try:
            for line in process.stdout:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(value, dict):
                    self._messages.put(value)
        finally:
            self._messages.put({"_eof": True})

    def _write(self, value: dict) -> None:
        process = self._process
        if process is None or process.stdin is None or process.poll() is not None:
            raise LLMError("Codex App Server 已退出。")
        with self._write_lock:
            process.stdin.write(json.dumps(value, ensure_ascii=False) + "\n")
            process.stdin.flush()

    def _notify(self, method: str, params: dict) -> None:
        self._write({"jsonrpc": "2.0", "method": method, "params": params})

    def _respond(self, request_id, result: dict | None = None, error: dict | None = None) -> None:
        message = {"jsonrpc": "2.0", "id": request_id}
        message["error" if error is not None else "result"] = error if error is not None else (result or {})
        self._write(message)

    def _handle_server_request(self, message: dict, approval_handler: Callable[[dict], str] | None = None) -> None:
        method = message.get("method")
        request_id = message.get("id")
        if request_id is None:
            return
        if method in {"item/commandExecution/requestApproval", "item/fileChange/requestApproval",
                      "item/permissions/requestApproval"}:
            decision = "decline"
            if approval_handler is not None:
                try:
                    decision = approval_handler(message)
                except Exception:
                    decision = "decline"
            self._respond(request_id, {"decision": decision})
            return
        self._respond(request_id, error={"code": -32601, "message": "LingTu does not expose this App Server request"})

    def _take(self, timeout: float = 30.0, approval_handler: Callable[[dict], str] | None = None,
              stop: threading.Event | None = None) -> dict:
        deadline = time.monotonic() + timeout
        while True:
            if stop is not None and stop.is_set():
                raise LLMError("Codex turn 已取消。")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise LLMError("Codex App Server 请求超时。")
            try:
                message = self._messages.get(timeout=min(remaining, 0.1) if stop is not None else remaining)
            except queue.Empty:
                continue
            if message.get("_eof"):
                raise LLMError("Codex App Server 提前退出。")
            if message.get("method") and message.get("id") is not None:
                if message["method"] == "item/tool/call" and stop is not None:
                    return message
                self._handle_server_request(message, approval_handler)
                continue
            return message

    def _request(self, method: str, params: dict, timeout: float = 30.0) -> dict:
        with self._rpc_lock:
            self._next_id += 1
            request_id = self._next_id
            self._write({"jsonrpc": "2.0", "id": request_id, "method": method, "params": params})
            deadline = time.monotonic() + timeout
            while True:
                message = self._take(max(0.1, deadline - time.monotonic()))
                if message.get("id") != request_id:
                    continue
                if "error" in message:
                    raise LLMError(str(message["error"].get("message", f"Codex 请求失败：{method}")))
                return message.get("result") or {}

    def _account(self) -> dict:
        self._start()
        return self._request("account/read", {})

    @staticmethod
    def _account_connected(result: dict) -> bool:
        return isinstance(result.get("account"), dict) and bool(result["account"].get("type"))

    def status(self) -> dict:
        with self._rpc_lock:
            try:
                result = self._account()
            except LLMError as exc:
                return {"provider": "codex", "connected": False, "available": False,
                        "inference_verified": False, "auth_mode": "app_server",
                        "reason": "codex_unavailable", "error": str(exc)}
            return {"provider": "codex", "connected": self._account_connected(result),
                    "inference_verified": False, "auth_mode": "app_server",
                    "codex_home": result.get("codexHome") or self.codex_home or None}

    def start_login(self) -> dict:
        with self._rpc_lock:
            self._start()
            result = self._request("account/login/start", {"type": "chatgpt"})
            login_id = result.get("loginId")
            auth_url = result.get("authUrl")
            if not isinstance(login_id, str) or not isinstance(auth_url, str):
                raise LLMError("Codex App Server 没有返回有效登录地址。")
            self._login = {"id": login_id, "state": "pending", "auth_url": auth_url,
                           "verification_url": auth_url, "expires_at": time.time() + 900, "interval": 5}
            return dict(self._login)

    def poll_login(self, login_id: str) -> dict:
        with self._rpc_lock:
            if not self._login or self._login["id"] != login_id:
                raise ValueError("login_not_found")
            if self._login["state"] != "pending":
                return dict(self._login)
            if time.time() >= self._login["expires_at"]:
                self._login["state"] = "expired"
                return dict(self._login)
            if self._account_connected(self._account()):
                self._login["state"] = "approved"
            return dict(self._login)

    def cancel_login(self, login_id: str) -> dict:
        with self._rpc_lock:
            if not self._login or self._login["id"] != login_id:
                raise ValueError("login_not_found")
            if self._login["state"] == "pending":
                self._request("account/login/cancel", {"loginId": login_id})
                self._login["state"] = "cancelled"
            return dict(self._login)

    def disconnect(self) -> dict:
        with self._rpc_lock:
            self._start()
            self._request("account/logout", {})
            self._login = None
            result = {"provider": "codex", "connected": False, "inference_verified": False,
                      "auth_mode": "app_server", "codex_home": self.codex_home or None}
            return result

    @staticmethod
    def _input(messages: Iterable[dict]) -> tuple[list[dict], list[str]]:
        inputs: list[dict] = []
        temporary: list[str] = []
        for message in messages:
            if message.get("type") == "function_call":
                inputs.append({"type": "text", "text":
                    f"assistant tool {message['name']} ({message['call_id']}): {message['arguments']}"})
                continue
            if message.get("type") == "function_call_output":
                output = message["output"]
                content = output if isinstance(output, list) else [{"type": "text", "text": str(output)}]
                message = {"role": f"tool {message['call_id']}", "content": [
                    {"type": "input_image", "image_url": part["image_url"]}
                    if part.get("type") == "image" else part for part in content]}
            content = message.get("content", "")
            if not isinstance(content, list):
                if content:
                    inputs.append({"type": "text", "text": f"{message.get('role', 'user')}: {content}"})
                continue
            text_parts = [str(part.get("text", "")) for part in content
                          if isinstance(part, dict) and part.get("type") in {"input_text", "text"}]
            if text_parts:
                inputs.append({"type": "text", "text": f"{message.get('role', 'user')}: {' '.join(text_parts)}"})
            for part in content:
                if not isinstance(part, dict) or part.get("type") != "input_image":
                    continue
                url = part.get("image_url")
                if not isinstance(url, str) or "," not in url or not url.startswith("data:"):
                    continue
                header, encoded = url.split(",", 1)
                suffix = ".jpg" if "jpeg" in header or "jpg" in header else ".png"
                path = tempfile.NamedTemporaryFile(prefix="lingtu-codex-", suffix=suffix, delete=False).name
                Path(path).write_bytes(base64.b64decode(encoded))
                temporary.append(path)
                inputs.append({"type": "localImage", "path": path})
        return inputs or [{"type": "text", "text": "user: "}], temporary

    def chat(self, messages: list[dict], *, model: str, cwd: str | None = None,
             permission_mode: str = "default",
             reasoning: str = "default",
             on_event: Callable[[dict], None] | None = None,
             approval_handler: Callable[[dict], str] | None = None,
             tools: list[dict] | None = None, instructions: str = "",
             invoke_tool: Callable[[str, dict, str], dict] | None = None,
             stop: threading.Event | None = None) -> str:
        with self._rpc_lock:
            if not self._account_connected(self._account()):
                raise LLMError("请先在 Codex App Server 中登录 ChatGPT。")
            writable = permission_mode == "full_access"
            approval_policy = "never" if writable else "on-request"
            registered = {tool["name"] for tool in tools or []}
            stop = stop or threading.Event()
            thread = self._request("thread/start", {
                "cwd": cwd or str(Path.cwd()), "model": model,
                "approvalPolicy": approval_policy, "sandbox": "workspace-write" if permission_mode != "read_only" else "read-only",
                "ephemeral": True, "threadSource": "lingtu",
                "baseInstructions": instructions or None,
                "dynamicTools": [{"type": "function", "name": tool["name"],
                                  "description": tool.get("description", ""),
                                  "inputSchema": tool.get("parameters", {"type": "object"})}
                                 for tool in tools or []],
            })
            thread_id = (thread.get("thread") or {}).get("id")
            if not isinstance(thread_id, str) or not thread_id:
                raise LLMError("Codex App Server 没有返回 thread ID。")
            self._next_id += 1
            request_id = self._next_id
            inputs, temporary = self._input(messages)
            self._write({"jsonrpc": "2.0", "id": request_id, "method": "turn/start", "params": {
                "threadId": thread_id, "input": inputs, "approvalPolicy": approval_policy,
                "effort": "medium" if reasoning == "default" else reasoning,
                "sandbox": "workspace-write" if permission_mode != "read_only" else "read-only",
            }})
            started = False
            turn_id = None
            results: dict[str, tuple[str, dict, dict]] = {}
            chunks: list[str] = []
            try:
                while True:
                    message = self._take(120.0, approval_handler, stop)
                    if message.get("id") == request_id and not message.get("method"):
                        if "error" in message:
                            raise LLMError(str(message["error"].get("message", "Codex turn 启动失败")))
                        started = True
                        turn_id = ((message.get("result") or {}).get("turn") or {}).get("id")
                        continue
                    method = message.get("method")
                    params = message.get("params") or {}
                    if method == "item/tool/call":
                        name, arguments, call_id = params.get("tool"), params.get("arguments"), params.get("callId")
                        try:
                            if params.get("threadId") != thread_id or name not in registered or invoke_tool is None:
                                raise ValueError("Tool is not registered for this conversation turn")
                            if not isinstance(arguments, dict) or not isinstance(call_id, str) or not call_id:
                                raise ValueError("Invalid dynamic tool call")
                            if call_id in results:
                                old_name, old_arguments, response = results[call_id]
                                if name != old_name or arguments != old_arguments:
                                    raise ValueError("Conflicting dynamic tool call ID")
                            else:
                                try:
                                    response = invoke_tool(name, arguments, call_id)
                                except Exception as exc:
                                    response = {"success": False, "contentItems": [{"type": "inputText", "text": str(exc)}]}
                                results[call_id] = (name, arguments, response)
                        except Exception as exc:
                            response = {"success": False, "contentItems": [{"type": "inputText", "text": str(exc)}]}
                        self._respond(message["id"], response)
                    elif method == "item/agentMessage/delta" and params.get("threadId", thread_id) == thread_id:
                        delta = params.get("delta")
                        if isinstance(delta, str):
                            chunks.append(delta)
                            if on_event:
                                on_event({"type": "text_delta", "delta": delta})
                    elif method == "turn/completed" and params.get("threadId") == thread_id:
                        turn = params.get("turn") or {}
                        usage = turn.get("usage")
                        if on_event and isinstance(usage, dict):
                            on_event({"type": "codex_event", "method": "turn/usage/updated",
                                      "params": {"threadId": thread_id, "usage": usage}})
                        status_value = turn.get("status")
                        status = (status_value.get("type") if isinstance(status_value, dict)
                                  else status_value)
                        if status not in {None, "completed"}:
                            raise LLMError(f"Codex turn 结束状态：{status}")
                        return "".join(chunks)
                    elif method == "account/updated" and not started:
                        continue
                    elif on_event and params.get("threadId", thread_id) == thread_id and method in {
                        "item/started", "item/completed", "turn/started", "turn/plan/updated",
                        "thread/status/changed", "mcpServer/startupStatus/updated",
                    }:
                        on_event({"type": "codex_event", "method": method, "params": params})
            finally:
                if stop.is_set():
                    if turn_id:
                        self._next_id += 1
                        self._write({"jsonrpc": "2.0", "id": self._next_id, "method": "turn/interrupt",
                                     "params": {"threadId": thread_id, "turnId": turn_id}})
                    elif self._process is not None and self._process.poll() is None:
                        self._process.terminate()
                for path in temporary:
                    Path(path).unlink(missing_ok=True)

    def close(self) -> None:
        with self._rpc_lock:
            process = self._process
            self._process = None
            self._initialized = False
            if process is None or process.poll() is not None:
                return
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)


app_server = CodexAppServer()
