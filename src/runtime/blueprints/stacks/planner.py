"""Planner stack: SemanticPlanner + AgentPlanner + LLM + VisualServo."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import optional_stack_module, stack_module

from .memory import DEFAULT_SEMANTIC_DIR

logger = logging.getLogger(__name__)


def planner(
    llm: str = "kimi",
    save_dir: str = "",
    vla_backend: str = "",
    **config,
) -> Blueprint:
    """Semantic planning: goal resolution + agent loop + LLM reasoning + visual servo + VLA."""
    bp = Blueprint()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    try:
        SemanticPlannerModule = stack_module(
            "semantic_planner",
            "default",
            seed_group="decision",
            fallback="decision.modules.semantic_planner.SemanticPlannerModule",
        )
        LLMModule = stack_module(
            "llm",
            "pluggable",
            seed_group="llm",
            fallback="decision.modules.llm.LLMModule",
        )
        bp.add(
            SemanticPlannerModule,
            alias="SemanticPlannerModule",
            save_dir=save_dir,
            llm_backend=llm,
        )
        bp.add(LLMModule, alias="LLMModule", backend=llm)
    except ImportError as e:
        logger.warning("Semantic planner not available: %s", e)

    # AgentPlannerModule: multi-turn agent loop (extracted from SemanticPlannerModule)
    try:
        AgentPlannerModule = stack_module(
            "agent_planner",
            "default",
            seed_group="decision",
            fallback="decision.modules.agent_planner.AgentPlannerModule",
        )
        bp.add(
            AgentPlannerModule,
            alias="AgentPlannerModule",
            llm_backend=llm,
        )
    except ImportError as e:
        logger.warning("Agent planner not available: %s", e)

    VisualServoModule = optional_stack_module(
        "visual_servo",
        "default",
        seed_group="decision",
        fallback="decision.modules.visual_servo.VisualServoModule",
    )
    if VisualServoModule is not None:
        bp.add(VisualServoModule, alias="VisualServoModule")

    if vla_backend:
        VLAModule = optional_stack_module(
            "vla",
            "default",
            seed_group="decision",
            fallback="decision.modules.vla.VLAModule",
        )
        if VLAModule is not None:
            bp.add(
                VLAModule,
                alias="VLAModule",
                backend=vla_backend,
            )

    return bp
