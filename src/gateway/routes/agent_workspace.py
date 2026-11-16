"""Development Agent workspace and Git checkpoint routes."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from fastapi import HTTPException, Query

from gateway.schemas import (
    AgentEnvironmentCreateRequest,
    AgentWorkspaceCheckpointRequest,
    AgentWorkspaceCreateRequest,
)
from gateway.services.agent_environment import AgentEnvironmentStore
from gateway.services.agent_workspace import AgentWorkspaceStore, WorkspaceError


def _store(gw) -> AgentWorkspaceStore:
    current = getattr(gw, "_agent_workspace_store", None)
    if current is not None:
        return current
    configured_root = os.environ.get("LINGTU_AGENT_REPO_ROOT")
    repo_root = Path(configured_root) if configured_root else Path.cwd()
    try:
        current = AgentWorkspaceStore(repo_root)
    except WorkspaceError as exc:
        raise HTTPException(503, detail={"reason": exc.reason, "message": exc.message}) from exc
    gw._agent_workspace_store = current
    return current


def _run(operation):
    try:
        return operation()
    except WorkspaceError as exc:
        status = 404 if exc.reason.endswith("_not_found") else 409
        if exc.reason in {"repository_not_found", "workspace_metadata_invalid", "environment_metadata_invalid"}:
            status = 503
        raise HTTPException(status, detail={"reason": exc.reason, "message": exc.message}) from exc
    except (OSError, subprocess.SubprocessError) as exc:
        raise HTTPException(503, detail={"reason": "workspace_unavailable", "message": str(exc)}) from exc


def _environment_store(gw) -> AgentEnvironmentStore:
    current = getattr(gw, "_agent_environment_store", None)
    if current is not None:
        return current
    root = Path(os.environ.get("LINGTU_AGENT_REPO_ROOT") or Path.cwd()).resolve()
    current = AgentEnvironmentStore(root / ".lingtu" / "agent-environments.json")
    gw._agent_environment_store = current
    return current


def register_agent_workspace_routes(app, gw) -> None:
    """Expose only development workspaces; robot runtime has a separate owner."""

    @app.get("/api/v1/agent/environments")
    async def list_agent_environments():
        return {"environments": _run(lambda: _environment_store(gw).list(_store(gw).repo_root))}

    @app.post("/api/v1/agent/environments", status_code=201)
    async def create_agent_environment(body: AgentEnvironmentCreateRequest):
        return _run(lambda: _environment_store(gw).add(
            body.name, body.endpoint, body.cwd, body.capabilities))

    @app.delete("/api/v1/agent/environments/{environment_id}")
    async def delete_agent_environment(environment_id: str):
        _run(lambda: _environment_store(gw).delete(environment_id))
        return {"environment_id": environment_id, "deleted": True}

    @app.get("/api/v1/agent/workspaces")
    async def list_agent_workspaces():
        return {"workspaces": _run(lambda: _store(gw).list())}

    @app.post("/api/v1/agent/workspaces", status_code=201)
    async def create_agent_workspace(body: AgentWorkspaceCreateRequest):
        return _run(lambda: _store(gw).create(
            body.name,
            branch=body.branch,
            base_revision=body.base_revision,
            conversation_id=body.conversation_id,
        ))

    @app.delete("/api/v1/agent/workspaces/{workspace_id}")
    async def delete_agent_workspace(workspace_id: str, force: bool = Query(default=False)):
        return _run(lambda: _store(gw).delete(workspace_id, force=force))

    @app.get("/api/v1/agent/workspaces/{workspace_id}/checkpoints")
    async def list_agent_checkpoints(workspace_id: str):
        return {"workspace_id": workspace_id, "checkpoints": _run(lambda: _store(gw).checkpoints(workspace_id))}

    @app.post("/api/v1/agent/workspaces/{workspace_id}/checkpoints", status_code=201)
    async def create_agent_checkpoint(workspace_id: str, body: AgentWorkspaceCheckpointRequest):
        return _run(lambda: _store(gw).create_checkpoint(workspace_id, body.label, body.message))

    @app.post("/api/v1/agent/workspaces/{workspace_id}/checkpoints/{checkpoint_id}/restore")
    async def restore_agent_checkpoint(workspace_id: str, checkpoint_id: str,
                                       force: bool = Query(default=False)):
        return _run(lambda: _store(gw).restore_checkpoint(workspace_id, checkpoint_id, force=force))
