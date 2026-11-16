"""Descriptive Agent environment registry; remote execution stays out of Gateway."""

from __future__ import annotations

import json
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from gateway.services.agent_workspace import WorkspaceError


class AgentEnvironmentStore:
    """Persist local and configured remote workspace endpoints."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._lock = threading.RLock()

    def list(self, local_root: str | Path) -> list[dict[str, Any]]:
        with self._lock:
            local = {
                "environment_id": "local",
                "name": "本机工作区",
                "endpoint": "local",
                "cwd": str(Path(local_root).resolve()),
                "status": "ready",
                "capabilities": ["git", "worktree", "checkpoint"],
            }
            return [local, *[item for item in self._read() if item.get("environment_id") != "local"]]

    def add(self, name: str, endpoint: str, cwd: str,
            capabilities: list[str] | None = None) -> dict[str, Any]:
        name, endpoint, cwd = name.strip(), endpoint.strip(), cwd.strip()
        if not name or len(name) > 80:
            raise WorkspaceError("invalid_environment_name")
        if not endpoint or len(endpoint) > 512 or "\n" in endpoint:
            raise WorkspaceError("invalid_environment_endpoint")
        if not cwd or len(cwd) > 512 or "\n" in cwd:
            raise WorkspaceError("invalid_environment_cwd")
        with self._lock:
            value = {
                "environment_id": f"env_{uuid.uuid4().hex[:12]}",
                "name": name,
                "endpoint": endpoint,
                "cwd": cwd,
                "status": "configured",
                "capabilities": sorted({str(item).strip() for item in (capabilities or []) if str(item).strip()}),
                "created_at": time.time(),
            }
            values = self._read()
            values.append(value)
            self._write(values)
            return value

    def delete(self, environment_id: str) -> None:
        if environment_id == "local":
            raise WorkspaceError("local_environment_protected")
        with self._lock:
            values = self._read()
            kept = [item for item in values if item.get("environment_id") != environment_id]
            if len(kept) == len(values):
                raise WorkspaceError("environment_not_found")
            self._write(kept)

    def _read(self) -> list[dict[str, Any]]:
        if not self.path.exists():
            return []
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise WorkspaceError("environment_metadata_invalid") from exc
        if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
            raise WorkspaceError("environment_metadata_invalid")
        return value

    def _write(self, value: list[dict[str, Any]]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.path)
