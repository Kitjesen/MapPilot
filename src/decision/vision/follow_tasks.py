"""Retained Follow task records used by the visual-servo facade."""

from __future__ import annotations

import time
from collections import OrderedDict
from collections.abc import Iterator, MutableMapping
from copy import deepcopy
from typing import Any


class FollowTaskLedger(MutableMapping[str, dict[str, Any]]):
    """Own Follow task retention and child-task bookkeeping."""

    def __init__(self, retention: int = 64) -> None:
        self._retention = max(1, int(retention))
        self._tasks: OrderedDict[str, dict[str, Any]] = OrderedDict()

    def __getitem__(self, key: str) -> dict[str, Any]:
        return self._tasks[key]

    def __setitem__(self, key: str, value: dict[str, Any]) -> None:
        self._tasks[key] = value
        self._trim()

    def __delitem__(self, key: str) -> None:
        del self._tasks[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._tasks)

    def __len__(self) -> int:
        return len(self._tasks)

    @property
    def retention_label(self) -> str:
        return f"host_last_{self._retention}_executions"

    def create(
        self,
        *,
        follow_task_id: str,
        product_session_id: str,
        request_id: str,
        client_id: str,
        target: str,
        target_id: str,
    ) -> dict[str, Any]:
        task = {
            "follow_task_id": follow_task_id,
            "product_session_id": product_session_id,
            "request_id": request_id,
            "client_id": client_id,
            "target": target,
            "target_id": target_id,
            "state": "running",
            "terminal": False,
            "result": None,
            "reason": "",
            "children": {},
            "created_at": time.time(),
        }
        self[follow_task_id] = task
        return task

    def find_request(self, request_id: str, client_id: str) -> dict[str, Any] | None:
        for task in self.values():
            if (task["request_id"], task["client_id"]) == (request_id, client_id):
                return task
        return None

    def snapshot(
        self,
        follow_task_id: str,
        *,
        current: bool,
        details: dict[str, Any] | None = None,
        phase: str = "",
        reason: str = "",
    ) -> dict[str, Any] | None:
        task = self.get(follow_task_id)
        if task is None:
            return None
        view = deepcopy(task)
        view.pop("client_id", None)
        view["found"] = True
        view["kind"] = "follow"
        view["available_operations"] = [] if task["terminal"] else ["cancel"]
        view["retention"] = self.retention_label
        if current and not task["terminal"]:
            view["details"] = details or {}
            view["phase"] = phase
            if task["state"] != "cancelling":
                view["reason"] = reason
        return view

    def finish_cancel(self, follow_task_id: str, details: dict[str, Any]) -> bool:
        task = self.get(follow_task_id)
        if task is None or task["state"] != "cancelling":
            return False
        task.update(
            state="cancelled",
            terminal=True,
            result="cancelled",
            finished_at=time.time(),
            phase="cancelled",
            details=details,
        )
        return True

    @staticmethod
    def trim_child_history(task: dict[str, Any]) -> None:
        completed = [
            key for key, child in task["children"].items()
            if child.get("terminal")
        ]
        for key in completed[:-128]:
            del task["children"][key]

    def _trim(self) -> None:
        while len(self._tasks) > self._retention:
            self._tasks.popitem(last=False)
