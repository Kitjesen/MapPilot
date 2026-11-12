"""Operator-owned MCP configuration; credentials stay in the Host environment."""

import json
import os
import re
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class ServerConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    transport: Literal["stdio", "streamable_http", "sse"]
    command: str | None = None
    args: list[str] = Field(default_factory=list)
    cwd: str | None = None
    url: str | None = None
    env_names: list[str] = Field(default_factory=list)
    headers_env: dict[str, str] = Field(default_factory=dict)
    allowed_tools: list[str] | None = None
    read_only_tools: list[str] = Field(default_factory=list)
    tool_aliases: dict[str, str] = Field(default_factory=dict)
    timeout_seconds: float = Field(default=20, gt=0, le=120)

    @model_validator(mode="after")
    def transport_fields(self):
        if self.transport == "stdio":
            if not self.command or not self.command.strip() or self.url or self.headers_env:
                raise ValueError("stdio requires a command and cannot use URL/headers")
        elif not self.url or not self.url.startswith(("http://", "https://")) or self.command or self.args or self.cwd or self.env_names:
            raise ValueError("HTTP/SSE requires an HTTP URL and cannot use process arguments")
        return self


def load_servers(path: Path | None = None) -> dict[str, ServerConfig]:
    if path is None:
        root = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
        path = root / "lingtu/mcp.json"
    if not path.exists():
        return {}
    document = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(document, dict) or set(document) != {"mcpServers"} or not isinstance(document["mcpServers"], dict):
        raise ValueError("Expected an object containing mcpServers")
    servers = {}
    for name, value in document["mcpServers"].items():
        if not re.fullmatch(r"[a-zA-Z0-9_-]{1,32}", name):
            raise ValueError("MCP server IDs must contain 1-32 letters, numbers, underscores or hyphens")
        servers[name] = ServerConfig.model_validate(value)
    return servers
