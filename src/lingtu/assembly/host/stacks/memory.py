"""Optional semantic-context stack for Hosts that enable semantic planning."""

from __future__ import annotations

import logging
import os

from runtime.host import HostBuilder
from runtime.plugin_resolution import optional_stack_module

logger = logging.getLogger(__name__)

DEFAULT_SEMANTIC_DIR = os.path.join(os.path.expanduser("~"), ".nova", "semantic")


def memory(save_dir: str = "", **config) -> HostBuilder:
    """Build the optional semantic-context modules for one Host."""
    bp = HostBuilder()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    SemanticMapperModule = optional_stack_module(
        "semantic",
        "mapper",
        seed_group="memory",
        fallback="memory.modules.semantic.SemanticMapperModule",
    )
    if SemanticMapperModule is not None:
        bp.add(SemanticMapperModule, alias="SemanticMapperModule", save_dir=save_dir)
    else:
        logger.warning("SemanticMapperModule not available")

    EpisodicMemoryModule = optional_stack_module(
        "memory",
        "episodic",
        seed_group="memory",
        fallback="memory.modules.episodic.EpisodicMemoryModule",
    )
    if EpisodicMemoryModule is not None:
        bp.add(EpisodicMemoryModule, alias="EpisodicMemoryModule")

    TaggedLocationsModule = optional_stack_module(
        "memory",
        "tagged_locations",
        seed_group="memory",
        fallback="memory.modules.locations.TaggedLocationsModule",
    )
    if TaggedLocationsModule is not None:
        bp.add(
            TaggedLocationsModule,
            alias="TaggedLocationsModule",
            json_path=os.path.join(save_dir, "tagged_locations.json"),
        )

    VectorMemoryModule = optional_stack_module(
        "vector_memory",
        "default",
        seed_group="memory",
        fallback="memory.modules.vector.VectorMemoryModule",
    )
    if VectorMemoryModule is not None:
        bp.add(VectorMemoryModule, alias="VectorMemoryModule", persist_dir=save_dir)

    TemporalMemoryModule = optional_stack_module(
        "memory",
        "temporal",
        seed_group="memory",
        fallback="memory.modules.temporal.TemporalMemoryModule",
    )
    if TemporalMemoryModule is not None:
        bp.add(TemporalMemoryModule, alias="TemporalMemoryModule", save_dir=save_dir)

    return bp
