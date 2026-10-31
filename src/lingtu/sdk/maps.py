"""Saved maps, durable save operations, artifacts and named places."""

from __future__ import annotations

import tempfile
import time
import urllib.request
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import quote

from ._common import _CONNECTION_ERROR, _new_request_id
from .models import (
    CommandResult,
    MapInfo,
    MapList,
    _mapping,
)

if TYPE_CHECKING:
    from .client import LingTuClient


def _validate_save_request_identity(payload: Mapping[str, Any], expected: str) -> None:
    operation = _mapping(payload.get("operation"))
    for container in (payload, operation):
        returned = str(container.get("request_id") or "").strip()
        if returned and returned != expected:
            raise RuntimeError(f"Map save request identity mismatch: expected {expected}, got {returned}")


def _validate_save_operation_identity(payload: Mapping[str, Any], expected: str) -> None:
    operation = _mapping(payload.get("operation"))
    for container in (payload, operation):
        returned = str(container.get("operation_id") or "").strip()
        if returned and returned != expected:
            raise RuntimeError(f"Map save operation identity mismatch: expected {expected}, got {returned}")


def _save_operation_failure_reason(
    payload: Mapping[str, Any],
    fallback: str = "",
) -> str:
    operation = _mapping(payload.get("operation"))
    return str(
        operation.get("reason_code")
        or operation.get("message")
        or operation.get("error")
        or operation.get("detail")
        or payload.get("reason_code")
        or payload.get("message")
        or payload.get("error")
        or fallback
        or payload.get("detail")
        or "unknown reason"
    )


def _save_response_is_connection_unknown(
    payload: Mapping[str, Any],
    fallback: str = "",
) -> bool:
    operation = _mapping(payload.get("operation"))
    candidates = (
        operation.get("error"),
        operation.get("reason_code"),
        payload.get("error"),
        payload.get("reason_code"),
        fallback,
    )
    return any(str(value or "").strip().lower() == _CONNECTION_ERROR for value in candidates)


def _validated_map_save_receipt(
    receipt: CommandResult,
    expected_request_id: str,
) -> tuple[str, bool]:
    """Validate a map-save receipt and return its operation identity."""
    _validate_save_request_identity(receipt.raw, expected_request_id)
    if receipt.request_id and receipt.request_id != expected_request_id:
        raise RuntimeError(
            f"Map save request identity mismatch: expected {expected_request_id}, got {receipt.request_id}"
        )

    operation_id = str(receipt.operation_id or "").strip()
    if operation_id:
        _validate_save_operation_identity(receipt.raw, operation_id)

    submission_unknown = _save_response_is_connection_unknown(
        receipt.raw,
        receipt.message,
    )
    if (not receipt.ok or not receipt.accepted) and not submission_unknown:
        reason = _save_operation_failure_reason(receipt.raw, receipt.message)
        raise RuntimeError(f"Map save request rejected: {reason}")
    return operation_id, submission_unknown


class MapClient:
    """Saved maps, durable save operations, artifacts and named places."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    def list(self) -> MapList:
        """List saved maps on the robot."""
        raw = self._client._get("/api/v1/slam/maps")
        return self._parse_map_list(raw)

    def save(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Queue a durable save of the current SLAM map."""
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body: dict[str, Any] = {"request_id": resolved_request_id}
        if name:
            body["name"] = name
        result = self._client._command("/api/v1/map/save", body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    def save_and_wait(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
        timeout: float = 300.0,
        poll_interval: float = 1.0,
    ) -> dict[str, Any]:
        """Save the current map and wait for its durable terminal result."""
        deadline = time.monotonic() + timeout
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        while True:
            receipt = self.save(
                name,
                request_id=resolved_request_id,
            )
            operation_id, submission_unknown = _validated_map_save_receipt(
                receipt,
                resolved_request_id,
            )
            if receipt.ok and receipt.accepted and operation_id:
                break
            if not submission_unknown:
                raise RuntimeError("Map save response did not include operation_id")
            if time.monotonic() >= deadline:
                raise TimeoutError("Map save admission remained unknown until the timeout")
            time.sleep(poll_interval)
        state = ""
        while time.monotonic() < deadline:
            response = self.operation(operation_id)
            _validate_save_operation_identity(response, operation_id)
            operation = _mapping(response.get("operation"))
            state = str(operation.get("state") or "").strip().upper()
            if state == "SUCCEEDED":
                return response
            if state in {"FAILED", "CANCELLED", "CANCELED"}:
                raise RuntimeError(f"Map save {state}: {_save_operation_failure_reason(response)}")
            reason_code = str(operation.get("reason_code") or response.get("reason_code") or "").strip().lower()
            transient = reason_code == "operation_not_found" or _save_response_is_connection_unknown(response)
            explicit_error = (
                response.get("ok") is False
                or response.get("success") is False
                or response.get("error") is not None
                or response.get("detail") is not None
                or operation.get("error") is not None
                or operation.get("detail") is not None
                or state in {"ERROR", "REJECTED", "NOT_FOUND"}
            )
            if explicit_error and not transient:
                raise RuntimeError(f"Map save status query failed: {_save_operation_failure_reason(response)}")
            time.sleep(poll_interval)
        raise TimeoutError(f"Map save did not complete within {timeout}s (state={state or 'UNKNOWN'})")

    def operation(self, operation_id: str) -> dict[str, Any]:
        """Return the durable state of one map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return self._client._get(f"/api/v1/maps/operations/{encoded_operation_id}")

    def cancel_operation(self, operation_id: str) -> CommandResult:
        """Request cancellation of one map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return self._client._command(f"/api/v1/maps/operations/{encoded_operation_id}/cancel")

    def retry_operation(self, operation_id: str) -> CommandResult:
        """Retry one failed map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return self._client._command(f"/api/v1/maps/operations/{encoded_operation_id}/retry")

    def download_pcd(self, name: str, target: str | Path) -> Path:
        """Stream a saved map PCD artifact to ``target`` atomically."""
        destination = Path(target)
        encoded_name = quote(name, safe="")
        request = urllib.request.Request(self._client.base_url + f"/api/v1/maps/{encoded_name}/pcd")
        if self._client._api_key:
            request.add_header("X-API-Key", self._client._api_key)
        temporary_path: Path | None = None
        try:
            with urllib.request.urlopen(request, timeout=self._client._timeout) as response:
                content_length = response.headers.get("Content-Length")
                expected_bytes = int(content_length) if content_length is not None else None
                received_bytes = 0
                with tempfile.NamedTemporaryFile(
                    mode="wb",
                    dir=destination.parent,
                    prefix=f".{destination.name}.",
                    suffix=".part",
                    delete=False,
                ) as output:
                    temporary_path = Path(output.name)
                    while True:
                        chunk = response.read(1024 * 1024)
                        if not chunk:
                            break
                        output.write(chunk)
                        received_bytes += len(chunk)
                    if expected_bytes is not None and received_bytes != expected_bytes:
                        raise OSError(
                            f"Incomplete map download: expected {expected_bytes} bytes, received {received_bytes}"
                        )
            temporary_path.replace(destination)
        except BaseException:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
            raise
        return destination

    def rename(self, old_name: str, new_name: str) -> CommandResult:
        """Rename a saved map."""
        return self._client._command("/api/v1/map/rename", {"old_name": old_name, "new_name": new_name})

    def reset_cloud(self) -> CommandResult:
        """Clear the accumulated map cloud (visualisation only)."""
        return self._client._command("/api/v1/map_cloud/reset")

    def points(self) -> dict[str, Any]:
        """Get current live map point cloud as JSON."""
        return self._client._get("/api/v1/map/points")

    def saved_points(self, name: str) -> dict[str, Any]:
        """Get a saved map point cloud as JSON."""
        return self._client._get(f"/api/v1/maps/{name}/points")

    def locations(self) -> dict[str, Any]:
        """List tagged navigation locations."""
        return self._client._get("/api/v1/locations")

    def tag_location(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_current_pose: bool = False,
    ) -> CommandResult:
        """Create or update a tagged navigation location."""
        body: dict[str, Any] = {"name": name, "use_current_pose": use_current_pose}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return self._client._command("/api/v1/locations", body)

    def delete_location(self, name: str) -> CommandResult:
        """Delete a tagged navigation location."""
        return self._client._command(f"/api/v1/locations/{quote(name, safe='')}", method="DELETE")

    @staticmethod
    def _parse_map_list(raw: dict[str, Any]) -> MapList:
        active = raw.get("active", raw.get("active_map"))
        maps_raw = raw.get("maps", [])
        maps: list[MapInfo] = []
        for m in maps_raw:
            if isinstance(m, str):
                maps.append(MapInfo(name=m, raw={"name": m}))
            elif isinstance(m, dict):
                maps.append(
                    MapInfo(
                        name=m.get("name", ""),
                        has_pcd=m.get("has_pcd", False),
                        is_active=m.get("is_active", False) or m.get("name") == active,
                        size_bytes=m.get("size_bytes", 0),
                        raw=m,
                    )
                )
        return MapList(maps=maps, active_map=active, raw=raw)
