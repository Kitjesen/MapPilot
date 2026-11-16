"""Git workspaces owned by the development Agent, not by robot runtime."""

from __future__ import annotations

import json
import re
import subprocess
import threading
import time
import uuid
from pathlib import Path
from typing import Any


class WorkspaceError(ValueError):
    """An expected workspace operation failure with a stable API reason."""

    def __init__(self, reason: str, message: str | None = None) -> None:
        super().__init__(reason)
        self.reason = reason
        self.message = message or reason


class AgentWorkspaceStore:
    """Own worktree metadata and checkpoints for one source repository."""

    def __init__(self, repo_root: str | Path, metadata_path: str | Path | None = None) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.workspace_root = self.repo_root / ".lingtu" / "agent-workspaces"
        self.metadata_path = Path(metadata_path or (self.repo_root / ".lingtu" / "agent-workspaces.json"))
        self._lock = threading.RLock()

        if not self.repo_root.is_dir():
            raise WorkspaceError("repository_not_found")
        if self._git("rev-parse", "--is-inside-work-tree").stdout.strip() != "true":
            raise WorkspaceError("repository_not_found")

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            records = self._read().get("workspaces", [])
            return [self._view(record) for record in records]

    def create(
        self,
        name: str,
        *,
        branch: str | None = None,
        base_revision: str = "HEAD",
        conversation_id: str | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            slug = self._slug(name)
            workspace_id = f"ws_{uuid.uuid4().hex[:12]}"
            branch_name = branch or f"lingtu/agent/{slug}-{workspace_id[3:]}"
            parts = branch_name.split("/")
            if (not re.fullmatch(r"[A-Za-z0-9._/-]+", branch_name)
                    or branch_name.startswith("/") or any(part in {"", ".", ".."} for part in parts)):
                raise WorkspaceError("invalid_branch")
            base = self._git("rev-parse", "--verify", f"{base_revision}^{{commit}}").stdout.strip()
            path = (self.workspace_root / f"{slug}-{workspace_id[3:]}").resolve()
            self._ensure_inside_root(path)
            if any(item["branch"] == branch_name for item in self._read().get("workspaces", [])):
                raise WorkspaceError("workspace_branch_exists")

            self.workspace_root.mkdir(parents=True, exist_ok=True)
            try:
                self._git("worktree", "add", "-b", branch_name, str(path), base)
            except WorkspaceError as exc:
                raise WorkspaceError("worktree_create_failed", exc.message) from exc

            record = {
                "workspace_id": workspace_id,
                "name": name.strip(),
                "path": str(path),
                "branch": branch_name,
                "base_revision": base,
                "created_at": time.time(),
                "owner_conversation_id": conversation_id,
                "checkpoints": [],
            }
            try:
                data = self._read()
                data.setdefault("workspaces", []).append(record)
                self._write(data)
            except Exception:
                self._git("worktree", "remove", "--force", str(path), check=False)
                raise
            return self._view(record)

    def delete(self, workspace_id: str, *, force: bool = False) -> dict[str, Any]:
        with self._lock:
            data = self._read()
            record = self._find(data, workspace_id)
            path = self._workspace_path(record)
            if path == self.repo_root:
                raise WorkspaceError("main_worktree_protected")
            args = ["worktree", "remove"]
            if force:
                args.append("--force")
            args.append(str(path))
            try:
                self._git(*args)
            except WorkspaceError as exc:
                raise WorkspaceError("worktree_remove_failed", exc.message) from exc
            data["workspaces"] = [item for item in data["workspaces"] if item["workspace_id"] != workspace_id]
            self._write(data)
            return {"workspace_id": workspace_id, "deleted": True}

    def checkpoints(self, workspace_id: str) -> list[dict[str, Any]]:
        with self._lock:
            record = self._find(self._read(), workspace_id)
            self._workspace_path(record)
            return list(record.get("checkpoints", []))

    def create_checkpoint(self, workspace_id: str, label: str, message: str | None = None) -> dict[str, Any]:
        with self._lock:
            data = self._read()
            record = self._find(data, workspace_id)
            path = self._workspace_path(record)
            self._git_at(path, "add", "-A")
            changed = self._git_at(path, "diff", "--cached", "--quiet", check=False).returncode != 0
            if changed:
                commit_message = message.strip() if message and message.strip() else f"checkpoint: {label.strip()}"
                self._git_at(
                    path,
                    "-c", "user.name=LingTu Agent",
                    "-c", "user.email=agent@lingtu.local",
                    "commit", "-m", commit_message,
                )
            commit = self._git_at(path, "rev-parse", "HEAD").stdout.strip()
            checkpoint = {
                "checkpoint_id": f"cp_{uuid.uuid4().hex[:12]}",
                "label": label.strip(),
                "message": message.strip() if message else None,
                "commit": commit,
                "created_at": time.time(),
                "committed_changes": changed,
            }
            record.setdefault("checkpoints", []).append(checkpoint)
            self._write(data)
            return checkpoint

    def restore_checkpoint(self, workspace_id: str, checkpoint_id: str, *, force: bool = False) -> dict[str, Any]:
        with self._lock:
            data = self._read()
            record = self._find(data, workspace_id)
            checkpoint = next((item for item in record.get("checkpoints", [])
                               if item["checkpoint_id"] == checkpoint_id), None)
            if checkpoint is None:
                raise WorkspaceError("checkpoint_not_found")
            path = self._workspace_path(record)
            if not force and self._git_at(path, "status", "--short").stdout.strip():
                raise WorkspaceError("workspace_dirty")
            self._git_at(path, "cat-file", "-e", f"{checkpoint['commit']}^{{commit}}")
            self._git_at(path, "reset", "--hard", checkpoint["commit"])
            if force:
                self._git_at(path, "clean", "-fd")
            return {
                "workspace_id": workspace_id,
                "checkpoint_id": checkpoint_id,
                "restored": True,
                "commit": checkpoint["commit"],
                "untracked_files_removed": force,
            }

    def _view(self, record: dict[str, Any]) -> dict[str, Any]:
        path = Path(record["path"]).resolve()
        return {
            **record,
            "path_exists": path.is_dir(),
            "checkpoint_count": len(record.get("checkpoints", [])),
        }

    def _workspace_path(self, record: dict[str, Any]) -> Path:
        path = Path(record["path"]).resolve()
        self._ensure_inside_root(path)
        if path == self.repo_root:
            raise WorkspaceError("main_worktree_protected")
        return path

    def _ensure_inside_root(self, path: Path) -> None:
        try:
            path.relative_to(self.workspace_root.resolve())
        except ValueError as exc:
            raise WorkspaceError("workspace_path_outside_root") from exc

    def _find(self, data: dict[str, Any], workspace_id: str) -> dict[str, Any]:
        for record in data.get("workspaces", []):
            if record.get("workspace_id") == workspace_id:
                return record
        raise WorkspaceError("workspace_not_found")

    @staticmethod
    def _slug(name: str) -> str:
        value = re.sub(r"[^A-Za-z0-9._-]+", "-", name.strip()).strip(".-")
        return value[:48] or "workspace"

    def _read(self) -> dict[str, Any]:
        if not self.metadata_path.exists():
            return {"version": 1, "workspaces": []}
        try:
            data = json.loads(self.metadata_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise WorkspaceError("workspace_metadata_invalid") from exc
        if not isinstance(data, dict) or not isinstance(data.get("workspaces", []), list):
            raise WorkspaceError("workspace_metadata_invalid")
        return data

    def _write(self, data: dict[str, Any]) -> None:
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.metadata_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.metadata_path)

    def _git(self, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        return self._git_at(self.repo_root, *args, check=check)

    @staticmethod
    def _git_at(cwd: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            ["git", *args], cwd=cwd, text=True, capture_output=True, timeout=30, check=False,
        )
        if check and result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise WorkspaceError("git_failed", detail or "git command failed")
        return result
