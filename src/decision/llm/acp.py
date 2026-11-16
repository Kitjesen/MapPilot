"""Small ACP stdio client for external coding agents.

The external process owns its model and tools. LingTu only forwards a text turn
and collects assistant text; robot tools stay in the LingTu Agent loop.
"""

from __future__ import annotations

import asyncio
import json
import os
import shlex
import shutil
import subprocess
from collections.abc import Callable, Iterable
from dataclasses import replace

from decision.llm.client import LLMClientBase, LLMConfig, LLMError

ACP_AGENTS = {
    "acp:grok": {
        "label": "Grok Build · ACP",
        "command_env": "LINGTU_ACP_COMMAND",
        "args_env": "LINGTU_ACP_ARGS",
    },
    "acp:kilo": {
        "label": "Kilo Code · ACP",
        "command_env": "LINGTU_KILO_COMMAND",
        "args_env": "LINGTU_KILO_ARGS",
        "command": "kilo",
        "args": ("acp",),
        "install_url": "https://kilo.ai/docs/cli",
        "install_command": "npm install -g @kilocode/cli",
        "login_command": "kilo auth login",
    },
    "acp:kiro": {
        "label": "Kiro CLI · ACP",
        "command_env": "LINGTU_KIRO_COMMAND",
        "args_env": "LINGTU_KIRO_ARGS",
        "command": "kiro-cli",
        "args": ("acp",),
        "install_url": "https://kiro.dev/docs/cli",
        "install_command": "See the Kiro CLI installation guide",
        "login_command": "kiro-cli login",
    },
    "acp:opencode": {
        "label": "OpenCode · ACP",
        "command_env": "LINGTU_OPENCODE_COMMAND",
        "args_env": "LINGTU_OPENCODE_ARGS",
        "command": "opencode",
        "args": ("acp",),
        "install_url": "https://opencode.ai/docs",
        "install_command": "npm install -g opencode-ai",
        "login_command": "opencode auth login",
    },
    "acp:registry": {
        "label": "ACP Registry Agent",
        "command_env": "LINGTU_ACP_REGISTRY_COMMAND",
        "args_env": "LINGTU_ACP_REGISTRY_ARGS",
    },
}


def acp_agent_config(default: LLMConfig, model: str) -> LLMConfig:
    spec = ACP_AGENTS[model]
    command = os.environ.get(spec["command_env"], spec.get("command", ""))
    args = tuple(shlex.split(os.environ.get(spec["args_env"], ""), posix=os.name != "nt"))
    if not args:
        args = tuple(spec.get("args", ()))
    return replace(default, backend="acp", model=model.removeprefix("acp:"), api_key_env="",
                   command=command, command_args=args, timeout_sec=120, max_retries=0)


def acp_agent_choices(default: LLMConfig):
    for model, spec in ACP_AGENTS.items():
        config = acp_agent_config(default, model)
        yield config, model, spec["label"], acp_command_available(config)


def _command(config: LLMConfig) -> list[str]:
    raw = config.command.strip()
    if not raw and config.model in {"", "grok"}:
        raw = os.environ.get("LINGTU_ACP_COMMAND", "").strip()
    if not raw:
        return []
    args = list(config.command_args)
    if not args:
        args = shlex.split(os.environ.get("LINGTU_ACP_ARGS", ""), posix=os.name != "nt")
    return [*shlex.split(raw, posix=os.name != "nt"), *args]


def acp_command_available(config: LLMConfig | None = None) -> bool:
    command = _command(config or LLMConfig(backend="acp"))
    return bool(command and (shutil.which(command[0]) or os.path.isfile(command[0])))


def acp_provider_statuses(default: LLMConfig | None = None) -> list[dict]:
    """Return local ACP command status and copyable setup instructions.

    This deliberately never installs software or claims that an external Agent
    is logged in.  ACP owns authentication; LingTu can only report whether the
    configured executable is present and what it reports for ``--version``.
    """
    base = default or LLMConfig(backend="acp")
    providers: list[dict] = []
    for model, spec in ACP_AGENTS.items():
        config = acp_agent_config(base, model)
        command = _command(config)
        executable = command[0] if command else ""
        available = acp_command_available(config)
        version = None
        if available:
            try:
                result = subprocess.run(
                    [executable, "--version"],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    check=False,
                )
                output = (result.stdout or result.stderr).strip()
                if output:
                    version = output.splitlines()[0][:160]
            except (OSError, subprocess.SubprocessError):
                # A command can exist while its version probe is unavailable;
                # that does not change command availability.
                pass
        providers.append({
            "id": model,
            "label": spec["label"],
            "kind": "acp",
            "command": executable or None,
            # Do not echo custom launch arguments; they can contain provider secrets.
            "args": list(spec.get("args", ())),
            "available": available,
            "version": version,
            "authenticated": None,
            "auth_owner": "provider",
            "command_env": spec["command_env"],
            "args_env": spec["args_env"],
            "install_url": spec.get("install_url"),
            "install_command": spec.get("install_command"),
            "login_command": spec.get("login_command"),
        })
    return providers


def _text(value: object) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        return "\n".join(part for part in (_text(item) for item in value) if part)
    if isinstance(value, dict):
        if value.get("type") == "text":
            return _text(value.get("text", ""))
        return _text(value.get("content", ""))
    return ""


def _prompt(messages: Iterable[dict]) -> str:
    parts = []
    for message in messages:
        role = message.get("role", "user")
        content = _text(message.get("content", ""))
        if content:
            parts.append(f"{role}: {content}")
    return "\n\n".join(parts)


class AcpClient(LLMClientBase):
    """One ACP process per turn; no background agent or robot tool bridge."""

    def is_available(self) -> bool:
        return acp_command_available(self.config)

    async def chat(self, messages: list[dict], temperature: float | None = None,
                   *, permission_mode: str = "read_only",
                   approval_handler: Callable[[dict], str] | None = None) -> str:
        command = _command(self.config)
        if not command:
            raise LLMError("未配置 ACP 命令。请设置 LINGTU_ACP_COMMAND，或配置 Grok Build CLI。")
        if not acp_command_available(self.config):
            raise LLMError(f"ACP 命令不可用：{command[0]}")
        try:
            process = await asyncio.create_subprocess_exec(
                *command,
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
                cwd=self.config.cwd or None,
            )
        except OSError as exc:
            raise LLMError(f"无法启动 ACP Agent：{exc}") from exc

        next_id = 0

        async def respond_request(message: dict) -> None:
            request_id = message.get("id")
            method = message.get("method")
            if request_id is None:
                return
            if method != "session/request_permission":
                result = None
            else:
                decision = "decline"
                if permission_mode == "full_access":
                    decision = "accept"
                elif approval_handler is not None:
                    try:
                        decision = approval_handler(message)
                    except Exception:
                        decision = "decline"
                if decision == "accept":
                    options = (message.get("params") or {}).get("options") or []
                    option_id = next((item.get("optionId") or item.get("id") for item in options
                                      if isinstance(item, dict)), None)
                    result = {"outcome": {"outcome": "selected", "optionId": option_id}} if option_id else {
                        "outcome": {"outcome": "cancelled"}}
                else:
                    result = {"outcome": {"outcome": "cancelled"}}
            process.stdin.write((json.dumps({"jsonrpc": "2.0", "id": request_id,
                                              "result": result} if result is not None else {
                                                  "jsonrpc": "2.0", "id": request_id,
                                                  "error": {"code": -32601, "message": "LingTu does not expose this ACP request"}}) + "\n").encode())
            await process.stdin.drain()

        async def request(method: str, params: dict) -> dict:
            nonlocal next_id
            next_id += 1
            request_id = next_id
            process.stdin.write((json.dumps({"jsonrpc": "2.0", "id": request_id,
                                              "method": method, "params": params}) + "\n").encode())
            await process.stdin.drain()
            while True:
                line = await asyncio.wait_for(process.stdout.readline(), timeout=self.config.timeout_sec)
                if not line:
                    raise LLMError("ACP Agent 提前退出。")
                message = json.loads(line)
                if message.get("method") and message.get("id") is not None:
                    await respond_request(message)
                    continue
                if message.get("id") != request_id:
                    continue
                if "error" in message:
                    raise LLMError(str(message["error"].get("message", "ACP 请求失败")))
                return message.get("result") or {}

        async def run() -> str:
            await request("initialize", {"protocolVersion": 1, "clientCapabilities": {},
                                          "clientInfo": {"name": "lingtu", "version": "2.3"}})
            session = await request("session/new", {"cwd": self.config.cwd or os.getcwd(), "mcpServers": []})
            session_id = session.get("sessionId")
            if not isinstance(session_id, str) or not session_id:
                raise LLMError("ACP Agent 没有返回会话 ID。")
            prompt_id = next_id + 1
            process.stdin.write((json.dumps({"jsonrpc": "2.0", "id": prompt_id, "method": "session/prompt",
                                              "params": {"sessionId": session_id,
                                                         "prompt": [{"type": "text", "text": _prompt(messages)}]}}) + "\n").encode())
            await process.stdin.drain()
            chunks: list[str] = []
            while True:
                line = await asyncio.wait_for(process.stdout.readline(), timeout=self.config.timeout_sec)
                if not line:
                    raise LLMError("ACP Agent 在回复前退出。")
                message = json.loads(line)
                if message.get("method") and message.get("id") is not None:
                    await respond_request(message)
                    continue
                if message.get("method") == "session/update":
                    update = (message.get("params") or {}).get("update") or {}
                    if update.get("sessionUpdate") == "agent_message_chunk":
                        chunk = _text(update.get("content"))
                        if chunk:
                            chunks.append(chunk)
                    continue
                if message.get("id") == prompt_id:
                    if "error" in message:
                        raise LLMError(str(message["error"].get("message", "ACP 回复失败")))
                    return "".join(chunks) or _text((message.get("result") or {}).get("text", ""))

        try:
            return await asyncio.wait_for(run(), timeout=self.config.timeout_sec)
        except asyncio.TimeoutError:
            raise LLMError(f"ACP Agent 超时（{self.config.timeout_sec:g} 秒）。") from None
        finally:
            if process.stdin is not None and not process.stdin.is_closing():
                process.stdin.close()
                await process.stdin.wait_closed()
            if process.returncode is None:
                process.terminate()
                try:
                    await asyncio.wait_for(process.wait(), timeout=2)
                except asyncio.TimeoutError:
                    process.kill()
                    await process.wait()

