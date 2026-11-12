"""Instructions, follow and inspection tasks with explicit execution identities."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import quote, urlencode

from ._common import _download_file, _new_request_id
from .models import CommandResult

if TYPE_CHECKING:
    from .client import LingTuClient


class TaskClient:
    """Submit and control tasks without conflating their owner identities."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    def send_instruction(
        self,
        text: str,
        *,
        run_id: str | None = None,
        control_only: bool = False,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Submit text or a scoped control to the existing backend task owner.

        ``control_only`` prevents a new task from starting. Acceptance does not
        prove execution; detailed owner state and rejection reasons stay in raw.
        """
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body = {
            "text": text,
            "control_only": control_only,
            "request_id": resolved_request_id,
            "client_id": client_id,
        }
        if run_id is not None:
            body["run_id"] = run_id
        result = self._client._command("/api/v1/instruction", body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    def get_agent_run(self, run_id: str) -> dict[str, Any]:
        """Read a run retained by the current Host, preserving found=False."""
        return self._client._get(self._agent_run_path(run_id))

    def cancel_agent_run(
        self,
        run_id: str,
        *,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Request cancellation of this run; inspect raw for stop confirmation."""
        path = self._agent_run_path(run_id)
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        result = self._client._command(
            f"{path}/cancel",
            {"request_id": resolved_request_id, "client_id": client_id},
        )
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    @staticmethod
    def _agent_run_path(run_id: str) -> str:
        normalized_run_id = str(run_id or "").strip()
        if not normalized_run_id:
            raise ValueError("run_id is required")
        return f"/api/v1/agent/runs/{quote(normalized_run_id, safe='')}"

    def start_follow(
        self,
        *,
        description: str | None = None,
        target_id: str | None = None,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Start one owned Follow execution and return its stable identity."""
        normalized_description = str(description or "").strip()
        normalized_target_id = str(target_id or "").strip()
        if not normalized_description and not normalized_target_id:
            raise ValueError("description or target_id is required")

        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body = {
            "mode": "follow",
            "request_id": resolved_request_id,
            "client_id": client_id,
        }
        if normalized_description:
            body["target"] = normalized_description
        if normalized_target_id:
            body["target_id"] = normalized_target_id
        result = self._client._command("/api/v1/visual_servo", body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        if result.accepted and not (result.follow_task_id and result.product_session_id):
            raise RuntimeError("Follow start receipt is missing follow_task_id or product_session_id")
        return result

    def get_follow_task(
        self,
        follow_task_id: str,
        product_session_id: str,
    ) -> dict[str, Any]:
        """Read one Follow execution using its original Product-run identity."""
        encoded_task_id, normalized_session_id = self._follow_reference(follow_task_id, product_session_id)
        query = urlencode({"product_session_id": normalized_session_id})
        return self._client._get(f"/api/v1/follow/{encoded_task_id}?{query}")

    def cancel_follow_task(
        self,
        follow_task_id: str,
        product_session_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Cancel exactly one Follow execution in its original Product run."""
        encoded_task_id, normalized_session_id = self._follow_reference(follow_task_id, product_session_id)
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        result = self._client._command(
            f"/api/v1/follow/{encoded_task_id}/cancel",
            {
                "product_session_id": normalized_session_id,
                "reason": str(reason or "").strip() or "operator_cancel",
                "request_id": resolved_request_id,
                "client_id": client_id,
            },
        )
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    @staticmethod
    def _follow_reference(
        follow_task_id: str,
        product_session_id: str,
    ) -> tuple[str, str]:
        normalized_task_id = str(follow_task_id or "").strip()
        normalized_session_id = str(product_session_id or "").strip()
        if not normalized_task_id:
            raise ValueError("follow_task_id is required")
        if not normalized_session_id:
            raise ValueError("product_session_id is required")
        return quote(normalized_task_id, safe=""), normalized_session_id

    def inspection_routes(self, map_id: str | None = None) -> dict[str, Any]:
        """List stored inspection routes, optionally for one map."""
        path = "/api/v1/inspection/routes"
        normalized_map_id = str(map_id or "").strip()
        if normalized_map_id:
            path = f"{path}?{urlencode({'map_id': normalized_map_id})}"
        return self._client._get(path)

    def inspection_route(self, route_id: str, *, map_id: str | None = None) -> dict[str, Any]:
        """Read one stored native inspection route."""
        path = f"/api/v1/inspection/routes/{quote(route_id, safe='')}"
        if map_id is not None:
            path += "?" + urlencode({"map_id": map_id})
        return self._client._get(path)

    def save_inspection_route(self, route: dict[str, Any]) -> dict[str, Any]:
        """Create or update a route with its map epoch and revision."""
        return self._client._request_json("/api/v1/inspection/routes", route)

    def delete_inspection_route(self, route_id: str, *, map_id: str | None = None) -> dict[str, Any]:
        """Delete one stored route through the native inspection owner."""
        path = f"/api/v1/inspection/routes/{quote(route_id, safe='')}"
        if map_id is not None:
            path += "?" + urlencode({"map_id": map_id})
        return self._client._request_json(path, method="DELETE")

    def inspection_tasks(
        self, *, map_id: str | None = None, route_id: str | None = None,
        include_terminal: bool = False, limit: int = 20,
    ) -> dict[str, Any]:
        """List retained inspection task projections with server-side filters."""
        parameters = {key: value for key, value in {
            "map_id": map_id, "route_id": route_id,
            "include_terminal": str(include_terminal).lower(), "limit": limit,
        }.items() if value is not None}
        return self._client._get("/api/v1/inspection/tasks?" + urlencode(parameters))

    def list_inspection_evidence(self, *, limit: int = 20) -> dict[str, Any]:
        """List recent evidence verified by the inspection store."""
        return self._client._get("/api/v1/inspection/evidence?" + urlencode({"limit": limit}))

    def inspection_evidence(self, evidence_id: str) -> dict[str, Any]:
        """Read one verified evidence manifest."""
        return self._client._get(f"/api/v1/inspection/evidence/{quote(evidence_id, safe='')}")

    def review_inspection_evidence(self, evidence_id: str, verdict: str, *, note: str = "") -> dict[str, Any]:
        """Record a human finding while retaining the original capture result."""
        return self._client._request_json(
            f"/api/v1/inspection/evidence/{quote(evidence_id, safe='')}/review",
            {"verdict": verdict, "note": note},
        )

    def download_inspection_artifact(self, evidence_id: str, kind: str, target: str | Path) -> Path:
        """Download one verified evidence artifact by kind."""
        path = f"/api/v1/inspection/evidence/{quote(evidence_id, safe='')}/artifacts/{quote(kind, safe='')}"
        return _download_file(self._client, path, target)

    def download_inspection_measurement(self, evidence_id: str, sensor_id: str, target: str | Path) -> Path:
        """Download a verified measurement using its sensor identity."""
        path = f"/api/v1/inspection/evidence/{quote(evidence_id, safe='')}/measurements/{quote(sensor_id, safe='')}"
        return _download_file(self._client, path, target)

    def start_inspection(
        self,
        route_id: str,
        *,
        map_id: str | None = None,
        revision: int = 0,
        request_id: str | None = None,
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Submit a stored route for native inspection execution."""
        normalized_route_id = str(route_id or "").strip()
        if not normalized_route_id:
            raise ValueError("route_id is required")
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body: dict[str, Any] = {
            "route_id": normalized_route_id,
            "revision": revision,
            "request_id": resolved_request_id,
        }
        normalized_map_id = str(map_id or "").strip()
        if normalized_map_id:
            body["map_id"] = normalized_map_id
        if expected_product_session_id is not None:
            body["expected_product_session_id"] = expected_product_session_id
        if expected_env is not None:
            body["expected_env"] = expected_env
        result = self._client._command("/api/v1/inspection/tasks", body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    def inspection_task(self, task_id: str) -> dict[str, Any]:
        """Read the fact-backed state of one inspection task."""
        encoded_task_id = self._inspection_task_id(task_id)
        return self._client._get(f"/api/v1/inspection/tasks/{encoded_task_id}")

    def pause_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_pause",
        request_id: str | None = None,
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Request a pause for one inspection task."""
        return self._control_inspection(
            task_id, "pause", reason, request_id,
            expected_product_session_id=expected_product_session_id, expected_env=expected_env,
        )

    def resume_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_resume",
        request_id: str | None = None,
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Request resumption of one paused inspection task."""
        return self._control_inspection(
            task_id, "resume", reason, request_id,
            expected_product_session_id=expected_product_session_id, expected_env=expected_env,
        )

    def cancel_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Request cancellation of one inspection task."""
        return self._control_inspection(
            task_id, "cancel", reason, request_id,
            expected_product_session_id=expected_product_session_id, expected_env=expected_env,
        )

    def inspection_report(self, task_id: str) -> dict[str, Any]:
        """Read the business report for one inspection task."""
        encoded_task_id = self._inspection_task_id(task_id)
        return self._client._get(f"/api/v1/inspection/tasks/{encoded_task_id}/report")

    def _control_inspection(
        self,
        task_id: str,
        action: str,
        reason: str,
        request_id: str | None,
        *,
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        encoded_task_id = self._inspection_task_id(task_id)
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body = {
            "reason": str(reason or "").strip() or f"operator_{action}",
            "request_id": resolved_request_id,
        }
        if expected_product_session_id is not None:
            body["expected_product_session_id"] = expected_product_session_id
        if expected_env is not None:
            body["expected_env"] = expected_env
        result = self._client._command(f"/api/v1/inspection/tasks/{encoded_task_id}/{action}", body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    @staticmethod
    def _inspection_task_id(task_id: str) -> str:
        normalized = str(task_id or "").strip()
        if not normalized:
            raise ValueError("task_id is required")
        return quote(normalized, safe="")
