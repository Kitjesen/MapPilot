"""Agent workspaces are isolated Git worktrees with recoverable checkpoints."""

from __future__ import annotations

import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.agent_workspace import register_agent_workspace_routes
from gateway.services.agent_workspace import AgentWorkspaceStore, WorkspaceError


def git(path: Path, *args: str) -> str:
    result = subprocess.run(["git", *args], cwd=path, text=True, capture_output=True, check=True)
    return result.stdout.strip()


@pytest.fixture
def repository(tmp_path: Path) -> Path:
    git(tmp_path, "init", "-q")
    git(tmp_path, "config", "user.name", "Test")
    git(tmp_path, "config", "user.email", "test@example.invalid")
    (tmp_path / "README.md").write_text("initial\n", encoding="utf-8")
    git(tmp_path, "add", "README.md")
    git(tmp_path, "commit", "-qm", "initial")
    return tmp_path


def test_worktree_checkpoint_restore_and_delete(repository: Path):
    store = AgentWorkspaceStore(repository)
    workspace = store.create("navigation agent", conversation_id="conv-1")
    path = Path(workspace["path"])
    assert path.is_dir() and workspace["branch"].startswith("lingtu/agent/navigation-agent-")
    assert store.list()[0]["owner_conversation_id"] == "conv-1"

    (path / "notes.txt").write_text("first\n", encoding="utf-8")
    checkpoint = store.create_checkpoint(workspace["workspace_id"], "first pass")
    assert checkpoint["committed_changes"] is True
    (path / "notes.txt").write_text("second\n", encoding="utf-8")
    with pytest.raises(WorkspaceError) as error:
        store.restore_checkpoint(workspace["workspace_id"], checkpoint["checkpoint_id"])
    assert error.value.reason == "workspace_dirty"
    (path / "temporary.txt").write_text("discarded\n", encoding="utf-8")
    restored = store.restore_checkpoint(workspace["workspace_id"], checkpoint["checkpoint_id"], force=True)
    assert restored["restored"] is True
    assert (path / "notes.txt").read_text(encoding="utf-8") == "first\n"
    assert not (path / "temporary.txt").exists()

    assert store.delete(workspace["workspace_id"])["deleted"] is True
    assert not path.exists()
    assert store.list() == []


def test_workspace_rejects_unsafe_branch(repository: Path):
    with pytest.raises(WorkspaceError) as error:
        AgentWorkspaceStore(repository).create("safe", branch="../outside")
    assert error.value.reason == "invalid_branch"


def test_http_workspace_routes_and_main_worktree_protection(repository: Path):
    store = AgentWorkspaceStore(repository)
    app = FastAPI()
    gateway = SimpleNamespace(_agent_workspace_store=store)
    register_agent_workspace_routes(app, gateway)
    with TestClient(app) as client:
        created = client.post("/api/v1/agent/workspaces", json={"name": "api"})
        assert created.status_code == 201
        workspace_id = created.json()["workspace_id"]
        assert client.get("/api/v1/agent/workspaces").json()["workspaces"][0]["workspace_id"] == workspace_id
        checkpoint = client.post(
            f"/api/v1/agent/workspaces/{workspace_id}/checkpoints",
            json={"label": "clean"},
        )
        assert checkpoint.status_code == 201
        assert client.get(f"/api/v1/agent/workspaces/{workspace_id}/checkpoints").status_code == 200
        assert client.delete(f"/api/v1/agent/workspaces/{workspace_id}").status_code == 200
        assert client.get(f"/api/v1/agent/workspaces/{workspace_id}/checkpoints").status_code == 404

        data = store._read()
        data["workspaces"].append({"workspace_id": "main", "path": str(repository), "checkpoints": []})
        store._write(data)
        assert client.delete("/api/v1/agent/workspaces/main").status_code == 409


def test_environment_routes_keep_remote_state_explicit(repository: Path):
    store = AgentWorkspaceStore(repository)
    app = FastAPI()
    gateway = SimpleNamespace(_agent_workspace_store=store)
    register_agent_workspace_routes(app, gateway)
    with TestClient(app) as client:
        listed = client.get("/api/v1/agent/environments")
        assert listed.status_code == 200
        assert listed.json()["environments"][0]["status"] == "ready"
        created = client.post("/api/v1/agent/environments", json={
            "name": "build host", "endpoint": "ssh://builder", "cwd": "/workspace/lingtu",
            "capabilities": ["git", "terminal"],
        })
        assert created.status_code == 201
        environment = created.json()
        assert environment["status"] == "configured"
        assert client.delete(f"/api/v1/agent/environments/{environment['environment_id']}").status_code == 200
        assert client.delete("/api/v1/agent/environments/local").status_code == 409
