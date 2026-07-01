"""Saved-map artifact gate for GlobalPlanner."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from nav.services.plan.global_planner.artifacts import SavedMapArtifacts
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.same_source_map_artifacts import validate_saved_map_artifact_dir

logger = logging.getLogger(__name__)


class GlobalPlannerArtifactGateMixin:
    def _default_map_artifact_gate(self) -> dict[str, Any]:
        required = self._is_pct_planner() or self._is_octoplanner3d()
        return {
            "required": required,
            "ok": True,
            "reason": "not_checked",
            "blockers": [],
        }
    def _map_dir_candidates(self) -> list[Path]:
        return list(self._saved_map_artifacts().map_dirs)
    def _saved_map_artifacts(self) -> SavedMapArtifacts:
        return SavedMapArtifacts.from_runtime(self._tomogram)
    def _active_artifact_path(self, filename: str) -> str:
        return self._saved_map_artifacts().active_artifact(filename)
    def _resolve_map_path(self, name: str | None = None) -> str:
        return self._saved_map_artifacts().planner_map_path(name or self._planner_name)
    def _resolve_tomogram_path(self) -> str:
        map_path = self._resolve_map_path(self._planner_name)
        if map_path:
            logger.info("GlobalPlanner: using map artifact: %s", map_path)
        return map_path
    def _validate_map_artifact_gate(self) -> dict[str, Any]:
        expected_frame_id = (
            self._expected_saved_map_frame_id
            or topic_default_frame_id(TOPICS.saved_map_cloud)
        )
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
            gate = validate_saved_map_artifact_dir(
                Path(octomap_path).resolve().parent,
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
                "saved_map_artifact_ok"
                if gate.get("ok") is True
                else "saved_map_artifact_missing_or_invalid"
            )
            return gate

        if not self._is_pct_planner():
            return {
                "required": False,
                "ok": True,
                "reason": "not_required_for_planner",
                "planner": self._planner_name,
                "blockers": [],
            }
        tomogram_path = self._resolve_map_path("pct")
        bundle = self._saved_map_artifacts().planner_map_bundle("pct")
        if not tomogram_path:
            return {
                "schema_version": "lingtu.saved_map_artifacts.gate.v1",
                "required": True,
                "ok": False,
                "reason": "tomogram_required_for_pct_planner",
                "planner": self._planner_name,
                "tomogram": "",
                "expected_frame_id": expected_frame_id,
                "blockers": ["tomogram required for pct planner"],
            }
        gate = validate_saved_map_artifact_dir(
            Path(tomogram_path).resolve().parent,
            require_tomogram=True,
            expected_frame_id=expected_frame_id,
        )
        gate["required"] = True
        gate["planner"] = self._planner_name
        gate["tomogram"] = str(tomogram_path)
        if bundle:
            gate["map_bundle"] = bundle
        gate["expected_frame_id"] = expected_frame_id
        if gate.get("ok") is True:
            gate["reason"] = "saved_map_artifact_ok"
        else:
            gate["reason"] = "saved_map_artifact_missing_or_invalid"
        return gate
    def _map_artifact_gate_blocks(self) -> bool:
        return (
            bool(self._map_artifact_gate.get("required", False))
            and self._map_artifact_gate.get("ok") is not True
        )
    def _map_artifact_gate_failure_reason(self) -> str:
        blockers = [
            str(item)
            for item in (self._map_artifact_gate.get("blockers") or [])
            if str(item)
        ]
        detail = "; ".join(blockers) if blockers else "unknown blocker"
        return f"saved map artifact gate failed: {detail}"
