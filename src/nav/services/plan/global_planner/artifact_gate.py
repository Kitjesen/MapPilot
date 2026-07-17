"""Saved-map artifact gate for GlobalPlanner."""

from __future__ import annotations

import logging
from typing import Any

from nav.services.plan.global_planner.artifacts import SavedMapArtifacts
from runtime.runtime_interface import TOPICS, topic_default_frame_id

logger = logging.getLogger(__name__)


class GlobalPlannerArtifactGateMixin:
    """Resolve and validate saved-map artifacts through the maps domain."""

    def _map_artifact_gate_required_by_config(self) -> bool:
        override = getattr(self, "_map_artifact_gate_required", None)
        if override is not None:
            return bool(override)
        return self._is_octoplanner3d()

    def _default_map_artifact_gate(self) -> dict[str, Any]:
        required = self._map_artifact_gate_required_by_config()
        return {
            "required": required,
            "ok": True,
            "reason": "not_checked",
            "blockers": [],
        }

    def _saved_map_artifacts(self) -> SavedMapArtifacts:
        return SavedMapArtifacts.from_runtime(self._map_path)

    def _active_artifact_path(self, filename: str) -> str:
        return self._saved_map_artifacts().active_artifact(filename)

    def _resolve_map_path(self, name: str | None = None) -> str:
        return self._saved_map_artifacts().planner_map_path(name or self._planner_name)

    def _resolve_map_artifact_path(self) -> str:
        map_path = self._resolve_map_path(self._planner_name)
        if map_path:
            logger.info("GlobalPlanner: using map artifact: %s", map_path)
        return map_path

    def _validate_map_artifact_gate(self) -> dict[str, Any]:
        if not self._map_artifact_gate_required_by_config():
            return {
                "schema_version": "lingtu.saved_map_artifacts.gate.v1",
                "required": False,
                "ok": True,
                "reason": "disabled_by_runtime_profile",
                "planner": self._planner_name,
                "blockers": [],
            }
        expected_frame_id = self._expected_saved_map_frame_id or topic_default_frame_id(TOPICS.saved_map_cloud)
        if self._is_octoplanner3d():
            bundle = self._saved_map_artifacts().planner_map_bundle("octoplanner3d")
            octomap_path = self._resolve_map_path("octoplanner3d")
            if not octomap_path:
                return {
                    "schema_version": "lingtu.saved_map_artifacts.gate.v1",
                    "required": True,
                    "ok": False,
                    "reason": "octomap_required_for_octoplanner3d_planner",
                    "planner": self._planner_name,
                    "octomap": "",
                    "expected_frame_id": expected_frame_id,
                    "blockers": ["octomap required for octoplanner3d planner"],
                }
            gate = self._saved_map_artifacts().validate_artifact_path(
                octomap_path,
                require_octomap=True,
                expected_frame_id=expected_frame_id,
            )
            gate["required"] = True
            gate["planner"] = self._planner_name
            gate["octomap"] = str(octomap_path)
            if bundle:
                gate["map_bundle"] = bundle
            gate["expected_frame_id"] = expected_frame_id
            gate["reason"] = (
                "saved_map_artifact_ok" if gate.get("ok") is True else "saved_map_artifact_missing_or_invalid"
            )
            return gate

        return {
            "required": False,
            "ok": True,
            "reason": "not_required_for_planner",
            "planner": self._planner_name,
            "blockers": [],
        }

    def _map_artifact_gate_blocks(self) -> bool:
        return bool(self._map_artifact_gate.get("required", False)) and self._map_artifact_gate.get("ok") is not True

    def _map_artifact_gate_failure_reason(self) -> str:
        blockers = [str(item) for item in (self._map_artifact_gate.get("blockers") or []) if str(item)]
        detail = "; ".join(blockers) if blockers else "unknown blocker"
        return f"saved map artifact gate failed: {detail}"

    def _refresh_map_artifact_gate(self) -> dict[str, Any]:
        self._map_artifact_gate = self._validate_map_artifact_gate()
        return self._map_artifact_gate

    @staticmethod
    def _map_artifact_gate_identity(gate: dict[str, Any]) -> tuple[str, ...]:
        bundle = gate.get("map_bundle")
        bundle = bundle if isinstance(bundle, dict) else {}
        artifacts = gate.get("artifacts")
        artifacts = artifacts if isinstance(artifacts, dict) else {}

        def artifact_identity(name: str) -> tuple[str, str, str]:
            artifact = artifacts.get(name)
            artifact = artifact if isinstance(artifact, dict) else {}
            return (
                str(artifact.get("path") or ""),
                str(artifact.get("sha256") or ""),
                str(artifact.get("source_map_sha256") or ""),
            )

        return (
            str(bundle.get("map_id") or ""),
            str(bundle.get("version_id") or ""),
            str(gate.get("checked_frame_id") or ""),
            *artifact_identity("map_pcd"),
            *artifact_identity("octomap"),
            *artifact_identity("occupancy_grid"),
        )

    def _reject_changed_map_artifact_gate(self) -> None:
        gate = dict(self._map_artifact_gate)
        blockers = [str(item) for item in (gate.get("blockers") or []) if str(item)]
        blockers.append("saved map artifacts changed during planning")
        gate["ok"] = False
        gate["reason"] = "saved_map_artifact_changed_during_plan"
        gate["blockers"] = blockers
        self._map_artifact_gate = gate
