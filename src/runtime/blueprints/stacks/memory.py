"""Memory stack: SemanticMapper + EpisodicMemory + TaggedLocations + VectorMemory."""

from __future__ import annotations

import logging
import os

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import optional_stack_module

logger = logging.getLogger(__name__)

DEFAULT_SEMANTIC_DIR = os.path.join(os.path.expanduser("~"), ".nova", "semantic")


def memory(save_dir: str = "", **config) -> Blueprint:
    """Semantic memory: KG + topology + episodes + tagged locations."""
    bp = Blueprint()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    SemanticMapperModule = optional_stack_module(
        "semantic",
        "mapper",
        seed_group="memory",
        fallback="memory.modules.semantic_mapper_module.SemanticMapperModule",
    )
    if SemanticMapperModule is not None:
        bp.add(SemanticMapperModule, alias="SemanticMapperModule", save_dir=save_dir)
    else:
        logger.warning("SemanticMapperModule not available")

    EpisodicMemoryModule = optional_stack_module(
        "memory",
        "episodic",
        seed_group="memory",
        fallback="memory.modules.episodic_module.EpisodicMemoryModule",
    )
    if EpisodicMemoryModule is not None:
        bp.add(EpisodicMemoryModule, alias="EpisodicMemoryModule")

    TaggedLocationsModule = optional_stack_module(
        "memory",
        "tagged_locations",
        seed_group="memory",
        fallback="memory.modules.tagged_locations_module.TaggedLocationsModule",
    )
    if TaggedLocationsModule is not None:
        bp.add(TaggedLocationsModule, alias="TaggedLocationsModule")

    VectorMemoryModule = optional_stack_module(
        "vector_memory",
        "default",
        seed_group="memory",
        fallback="memory.modules.vector_memory_module.VectorMemoryModule",
    )
    if VectorMemoryModule is not None:
        bp.add(VectorMemoryModule, alias="VectorMemoryModule", persist_dir=save_dir)

    TemporalMemoryModule = optional_stack_module(
        "memory",
        "temporal",
        seed_group="memory",
        fallback="memory.modules.temporal_memory_module.TemporalMemoryModule",
    )
    if TemporalMemoryModule is not None:
        bp.add(TemporalMemoryModule, alias="TemporalMemoryModule", save_dir=save_dir)

    MissionLoggerModule = optional_stack_module(
        "memory",
        "mission_logger",
        seed_group="memory",
        fallback="memory.modules.mission_logger_module.MissionLoggerModule",
    )
    if MissionLoggerModule is not None:
        bp.add(MissionLoggerModule, alias="MissionLoggerModule")

    return bp
