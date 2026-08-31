"""Durable scene authoring above trusted world-specific compilers."""

from __future__ import annotations

import copy
from collections.abc import Mapping
from typing import Any

from .scene_tools import FactoryParkSceneTool
from .store import StudioStore


class SceneDraftService:
    """Persist only scene batches accepted by the authoritative compiler.

    A SceneDraft is a Studio authoring resource.  It does not mutate a
    WorldPackage and it is deliberately unable to launch Blender, Unreal, or
    MuJoCo.  Promotion into versioned package content remains a separate
    operation and trust boundary.
    """

    def __init__(self, store: StudioStore, scene_tool: FactoryParkSceneTool) -> None:
        if not isinstance(store, StudioStore):
            raise TypeError("store must be a StudioStore")
        if not isinstance(scene_tool, FactoryParkSceneTool):
            raise TypeError("scene_tool must be a FactoryParkSceneTool")
        self.store = store
        self.scene_tool = scene_tool

    def _payload(self, batch: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(batch, Mapping):
            raise TypeError("scene batch must be an object")
        detached_batch = copy.deepcopy(dict(batch))
        validation = self.scene_tool.validate_element_batch(detached_batch)
        return {
            "schema": "lingtu.sim.studio.scene-draft-payload.v1",
            "scene_tool": validation["scene_tool"],
            "world_package": validation["world_package"],
            "layout_digest": validation["layout_digest"],
            "batch_digest": validation["digest"],
            "batch": detached_batch,
            "validation": copy.deepcopy(validation),
        }

    def create_scene_draft(
        self,
        batch: Mapping[str, Any],
        *,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Validate and durably create a scene draft."""

        return self.store.create_scene_draft(
            self._payload(batch),
            idempotency_key=idempotency_key,
        ).to_dict()

    def get_scene_draft(self, scene_draft_id: str) -> dict[str, Any]:
        """Return one scene draft by opaque identifier."""

        return self.store.get_scene_draft(scene_draft_id).to_dict()

    def list_scene_drafts(self) -> list[dict[str, Any]]:
        """Return scene drafts in deterministic creation order."""

        return [record.to_dict() for record in self.store.list_scene_drafts()]

    def update_scene_draft(
        self,
        scene_draft_id: str,
        *,
        revision: int,
        batch: Mapping[str, Any],
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Validate and CAS-update one scene draft."""

        return self.store.update_scene_draft(
            scene_draft_id,
            expected_revision=revision,
            payload=self._payload(batch),
            idempotency_key=idempotency_key,
        ).to_dict()


__all__ = ["SceneDraftService"]
