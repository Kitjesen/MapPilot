"""Operator-facing state for the layered environment map."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

_SCHEMA_VERSION = "lingtu.environment_map.status.v1"
_DEFAULT_FRESHNESS_LIMIT_S = 5.0


@dataclass(frozen=True)
class _SceneSnapshot:
    source_stamp_s: float | None
    frame_id: str | None
    map_id: str | None
    reset_epoch: int | None
    scene_identity_valid: bool
    has_elevation: bool
    elevation_identity_valid: bool


@dataclass(frozen=True)
class _DynamicResidualSnapshot:
    source_stamp_s: float | None
    product_session_id: str | None
    reset_epoch: int | None
    ready: bool


def _as_mapping(value: Any) -> Mapping[str, Any] | None:
    if isinstance(value, Mapping):
        return value
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        try:
            payload = to_dict(include_payload=False)
        except TypeError:
            payload = to_dict()
        return payload if isinstance(payload, Mapping) else None
    return None


def _finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _positive_int(value: Any) -> bool:
    try:
        return int(value) > 0
    except (TypeError, ValueError):
        return False


def _positive_int_or_none(value: Any) -> int | None:
    if not _positive_int(value):
        return None
    return int(value)


def _identity_valid(metadata: Mapping[str, Any], frame_id: str | None) -> bool:
    return bool(
        frame_id
        and str(metadata.get("producer_boot_id") or "").strip()
        and _positive_int(metadata.get("reset_epoch"))
        and _positive_int(metadata.get("observation_sequence"))
        and _positive_int(metadata.get("generation"))
    )


def _layer_payload(
    *,
    label: str,
    state: str,
    message_code: str,
    updated_at_s: float | None,
    displayable: bool,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "label": label,
        "state": state,
        "message_code": message_code,
        "displayable": displayable,
    }
    if updated_at_s is not None:
        payload["updated_at_s"] = updated_at_s
    return payload


class EnvironmentMapFeedback:
    """Retain one small, identity-checked view of the current MapScene.

    The public interface deliberately exposes operator states rather than wire
    identity fields.  Producer/epoch/sequence metadata is checked here so a
    later full risk layer can join the same scene without teaching callers
    those internal details.
    """

    def __init__(self, *, freshness_limit_s: float = _DEFAULT_FRESHNESS_LIMIT_S) -> None:
        if not math.isfinite(freshness_limit_s) or freshness_limit_s <= 0.0:
            raise ValueError("freshness_limit_s must be a finite positive value")
        self._freshness_limit_s = freshness_limit_s
        self._lock = threading.Lock()
        self._scene: _SceneSnapshot | None = None
        self._dynamic_residual: _DynamicResidualSnapshot | None = None
        self._reset_pending = False

    def observe_scene(self, frame: Any) -> None:
        """Accept the current coherent MapScene, never its heavy raster payload."""

        raw = _as_mapping(frame)
        if raw is None:
            return

        frame_id = str(raw.get("frame_id") or "").strip() or None
        map_id = str(raw.get("map_id") or "").strip() or None
        frame_metadata = raw.get("metadata")
        metadata = frame_metadata if isinstance(frame_metadata, Mapping) else {}
        layers = raw.get("layers")
        elevation_metadata: Mapping[str, Any] | None = None
        if isinstance(layers, list):
            for item in layers:
                layer = _as_mapping(item)
                if layer is None:
                    continue
                layer_type = str(layer.get("type") or layer.get("layer_type") or "").strip().lower()
                identifier = str(layer.get("id") or layer.get("name") or "").strip().lower()
                topic = str(layer.get("topic") or "").strip()
                if layer_type not in {"grid", "elevation"} or (
                    identifier != "maps.elevation" and topic != "/maps/elevation"
                ):
                    continue
                layer_metadata = layer.get("metadata")
                elevation_metadata = dict(metadata)
                if isinstance(layer_metadata, Mapping):
                    elevation_metadata.update(layer_metadata)
                break

        scene = _SceneSnapshot(
            source_stamp_s=_finite_float(raw.get("ts")),
            frame_id=frame_id,
            map_id=map_id,
            reset_epoch=_positive_int_or_none(metadata.get("reset_epoch")),
            scene_identity_valid=_identity_valid(metadata, frame_id),
            has_elevation=elevation_metadata is not None,
            elevation_identity_valid=(
                _identity_valid(elevation_metadata, frame_id) if elevation_metadata is not None else False
            ),
        )
        with self._lock:
            self._scene = scene
            self._reset_pending = False

    def observe_dynamic_residual(self, evidence: Any) -> None:
        """Retain explicit residual-clearance evidence for inspection admission."""

        raw = _as_mapping(evidence)
        snapshot = None
        if raw is not None:
            product_session_id = str(raw.get("product_session_id") or "").strip() or None
            snapshot = _DynamicResidualSnapshot(
                source_stamp_s=_finite_float(raw.get("ts")),
                product_session_id=product_session_id,
                reset_epoch=_positive_int_or_none(raw.get("reset_epoch")),
                ready=raw.get("ready") is True,
            )
        with self._lock:
            self._dynamic_residual = snapshot

    def clear(self) -> None:
        """Drop the old coordinate era before the next MapScene is accepted."""

        with self._lock:
            self._scene = None
            self._dynamic_residual = None
            self._reset_pending = True

    def inspection_admission(
        self,
        *,
        map_readiness_checked: bool,
        map_readiness_reason: str | None,
        product_session_id: str | None,
        now_s: float | None = None,
    ) -> dict[str, Any]:
        """Compose mapd readiness with scene-bound residual evidence.

        ``map_readiness_reason`` remains owned by HostBus. This method does not
        reconstruct mapd state; it only binds that verdict to the scene seen by
        Gateway and to separately supplied dynamic-residual evidence.
        """

        now = time.time() if now_s is None else now_s
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        with self._lock:
            scene = self._scene
            dynamic_residual = self._dynamic_residual
            reset_pending = self._reset_pending

        scene_layer, _elevation_layer = self._scene_layers(
            scene,
            reset_pending=reset_pending,
            now_s=now,
        )
        blockers: list[str] = []
        if scene_layer["state"] != "ready":
            blockers.append(str(scene_layer["message_code"]))

        normalized_map_reason = str(map_readiness_reason or "").strip() or None
        if not map_readiness_checked:
            blockers.append(normalized_map_reason or "map_readiness_evidence_missing")
        elif normalized_map_reason is not None:
            blockers.append(normalized_map_reason)

        expected_session_id = str(product_session_id or "").strip() or None
        if expected_session_id is None:
            blockers.append("product_session_id_missing")

        residual_fresh: bool | None = None
        residual_session_matches: bool | None = None
        residual_reset_matches: bool | None = None
        if dynamic_residual is None:
            blockers.append("dynamic_residual_evidence_missing")
        else:
            if dynamic_residual.source_stamp_s is None:
                blockers.append("dynamic_residual_timestamp_missing")
            else:
                residual_fresh = (
                    0.0 <= now - dynamic_residual.source_stamp_s <= self._freshness_limit_s
                )
                if not residual_fresh:
                    blockers.append("dynamic_residual_stale")
            if not dynamic_residual.ready:
                blockers.append("dynamic_residual_not_ready")
            residual_session_matches = bool(
                expected_session_id is not None
                and dynamic_residual.product_session_id == expected_session_id
            )
            if not residual_session_matches:
                blockers.append("dynamic_residual_session_mismatch")
            residual_reset_matches = bool(
                scene is not None
                and scene.reset_epoch is not None
                and dynamic_residual.reset_epoch == scene.reset_epoch
            )
            if not residual_reset_matches:
                blockers.append("dynamic_residual_reset_mismatch")

        blockers = list(dict.fromkeys(blockers))
        return {
            "ready": not blockers,
            "blockers": blockers,
            "scene": {
                "state": scene_layer["state"],
                "message_code": scene_layer["message_code"],
            },
            "mapd": {
                "readiness_checked": map_readiness_checked,
                "ready": map_readiness_checked and normalized_map_reason is None,
                "reason": normalized_map_reason,
            },
            "dynamic_residual": {
                "present": dynamic_residual is not None,
                "ready": dynamic_residual.ready if dynamic_residual is not None else False,
                "fresh": residual_fresh,
                "session_matches": residual_session_matches,
                "reset_matches": residual_reset_matches,
            },
        }

    def snapshot(
        self,
        *,
        traversability_status: Mapping[str, Any] | None,
        nav_endpoint_status: Mapping[str, Any] | None,
        now_s: float | None = None,
    ) -> dict[str, Any]:
        """Return a small user-facing projection of all environment-map layers."""

        now = time.time() if now_s is None else now_s
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        with self._lock:
            scene = self._scene
            reset_pending = self._reset_pending

        scene_layer, elevation_layer = self._scene_layers(scene, reset_pending=reset_pending, now_s=now)
        traversability_layer = self._traversability_layer(
            traversability_status,
            nav_endpoint_status,
            now_s=now,
        )
        layers = {
            "scene": scene_layer,
            "elevation": elevation_layer,
            "traversability": traversability_layer,
        }
        state, message_code = self._overall_state(layers)
        return {
            "schema_version": _SCHEMA_VERSION,
            "state": state,
            "message_code": message_code,
            "map_id": scene.map_id if scene is not None else None,
            "frame_id": scene.frame_id if scene is not None else None,
            "layers": layers,
            "limitations": ["native_risk_grid_available_via_sse_only"],
            "ts": now,
        }

    def _scene_layers(
        self,
        scene: _SceneSnapshot | None,
        *,
        reset_pending: bool,
        now_s: float,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        if scene is None:
            state = "updating" if reset_pending else "unavailable"
            message_code = "map_scene_reset_pending" if reset_pending else "awaiting_map_scene"
            scene_layer = _layer_payload(
                label="场景地图",
                state=state,
                message_code=message_code,
                updated_at_s=None,
                displayable=False,
            )
            return scene_layer, _layer_payload(
                label="最低观测高程",
                state=state,
                message_code=message_code,
                updated_at_s=None,
                displayable=False,
            )

        age_s = None if scene.source_stamp_s is None else max(0.0, now_s - scene.source_stamp_s)
        if age_s is None:
            scene_state, scene_code = "updating", "map_scene_timestamp_pending"
        elif age_s > self._freshness_limit_s:
            scene_state, scene_code = "stale", "map_scene_stale"
        elif not scene.scene_identity_valid:
            scene_state, scene_code = "updating", "map_scene_identity_pending"
        else:
            scene_state, scene_code = "ready", "map_scene_current"
        scene_layer = _layer_payload(
            label="场景地图",
            state=scene_state,
            message_code=scene_code,
            updated_at_s=scene.source_stamp_s,
            displayable=scene_state in {"ready", "stale"},
        )

        if not scene.has_elevation:
            elevation_state, elevation_code = "unavailable", "elevation_layer_not_available"
        elif scene_state == "stale":
            elevation_state, elevation_code = "stale", "elevation_stale"
        elif not scene.elevation_identity_valid:
            elevation_state, elevation_code = "invalid", "elevation_identity_incomplete"
        elif scene_state != "ready":
            elevation_state, elevation_code = "updating", "elevation_waiting_for_scene"
        else:
            elevation_state, elevation_code = "ready", "elevation_current"
        elevation_layer = _layer_payload(
            label="最低观测高程",
            state=elevation_state,
            message_code=elevation_code,
            updated_at_s=scene.source_stamp_s,
            displayable=elevation_state in {"ready", "stale"},
        )
        return scene_layer, elevation_layer

    def _traversability_layer(
        self,
        traversability_status: Mapping[str, Any] | None,
        nav_endpoint_status: Mapping[str, Any] | None,
        *,
        now_s: float,
    ) -> dict[str, Any]:
        status = traversability_status if isinstance(traversability_status, Mapping) else {}
        counters = status.get("counters")
        published = 0
        if isinstance(counters, Mapping):
            try:
                published = max(0, int(counters.get("published") or 0))
            except (TypeError, ValueError):
                published = 0
        nav_has_risk = None
        if isinstance(nav_endpoint_status, Mapping) and "has_traversability" in nav_endpoint_status:
            nav_has_risk = nav_endpoint_status.get("has_traversability") is True
        source_stamp_s = _finite_float(status.get("stamp_s"))
        age_s = None if source_stamp_s is None else max(0.0, now_s - source_stamp_s)

        if not status or published <= 0 or nav_has_risk is False:
            state, message_code = "unavailable", "traversability_not_published"
        elif source_stamp_s is None:
            state, message_code = "unavailable", "traversability_status_timestamp_missing"
        elif age_s is not None and age_s > self._freshness_limit_s:
            state, message_code = "stale", "traversability_stale"
        else:
            state, message_code = "diagnostic_only", "native_grid_available_via_sse"
        return _layer_payload(
            label="原生行走风险",
            state=state,
            message_code=message_code,
            updated_at_s=source_stamp_s,
            displayable=False,
        )

    @staticmethod
    def _overall_state(layers: Mapping[str, Mapping[str, Any]]) -> tuple[str, str]:
        scene_state = str(layers["scene"]["state"])
        elevation_state = str(layers["elevation"]["state"])
        traversability_state = str(layers["traversability"]["state"])
        if "stale" in {scene_state, elevation_state, traversability_state}:
            return "stale", "wait_for_current_environment_map"
        if scene_state == "updating":
            return "updating", "environment_map_updating"
        if scene_state == "unavailable" or traversability_state == "unavailable":
            return "unavailable", "environment_map_unavailable"
        if scene_state != "ready" or elevation_state != "ready":
            return "updating", "environment_map_updating"
        return "ready", "environment_map_ready"
