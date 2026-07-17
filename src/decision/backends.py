"""Decision layer backend dependency manager."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    pass  # avoid circular imports

logger = logging.getLogger(__name__)


class BackendManager:
    """Centralized dependency lookup and degradation manager for the decision layer.

    Manages references to external modules (VectorMemory, TaggedLocations,
    TopologySemGraph, LLM) with graceful degradation when modules are unavailable.
    """

    def __init__(self, system_modules: dict[str, Any]):
        self.vector_memory = self._safe_get(system_modules, "VectorMemoryModule")
        self.tagged_locations = self._safe_get(system_modules, "TaggedLocationsModule")
        self.topo_graph = self._safe_get(system_modules, "TopologySemGraph")
        self.llm_module = self._safe_get(system_modules, "LLMModule")
        self.semantic_mapper = self._safe_get(system_modules, "SemanticMapperModule")
        self._system_modules = system_modules

    def _safe_get(self, modules: dict[str, Any], name: str) -> Any | None:
        """Look up a module by name with graceful degradation."""
        mod = modules.get(name)
        if mod is None:
            logger.warning("[BackendManager] %s not loaded, feature degraded", name)
        return mod

    def get(self, name: str) -> Any | None:
        """Generic module lookup."""
        return self._system_modules.get(name)

    @property
    def has_vector_memory(self) -> bool:
        return self.vector_memory is not None

    @property
    def has_tagged_locations(self) -> bool:
        return self.tagged_locations is not None

    @property
    def has_topo_graph(self) -> bool:
        return self.topo_graph is not None

    @property
    def has_llm(self) -> bool:
        return self.llm_module is not None

    @property
    def has_semantic_mapper(self) -> bool:
        return self.semantic_mapper is not None

    def query_vector_memory(self, instruction: str, top_k: int = 3) -> list | None:
        """Query vector memory for location matches. Returns None if unavailable."""
        if not self.has_vector_memory:
            return None
        try:
            return self.vector_memory.query_location(instruction, top_k=top_k)
        except Exception as e:
            logger.warning("[BackendManager] vector_memory query failed: %s", e)
            return None

    def get_tagged_location(self, tag: str) -> Any | None:
        """Look up a tagged location. Returns None if unavailable."""
        if not self.has_tagged_locations:
            return None
        try:
            return self.tagged_locations.get_tag(tag)
        except Exception as e:
            logger.warning("[BackendManager] tagged_locations lookup failed: %s", e)
            return None

    def summary(self) -> dict[str, bool]:
        """Return a summary of backend availability for health checks."""
        return {
            "vector_memory": self.has_vector_memory,
            "tagged_locations": self.has_tagged_locations,
            "topo_graph": self.has_topo_graph,
            "llm": self.has_llm,
            "semantic_mapper": self.has_semantic_mapper,
        }
