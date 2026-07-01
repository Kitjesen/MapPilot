"""Planner stack: SemanticPlanner + LLM + VisualServo."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import optional_stack_module, stack_module

from .memory import DEFAULT_SEMANTIC_DIR

logger = logging.getLogger(__name__)


def planner(llm: str = "kimi", save_dir: str = "", **config) -> Blueprint:
    """Semantic planning: goal resolution + LLM reasoning + visual servo."""
    bp = Blueprint()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    try:
        SemanticPlannerModule = stack_module(
            "semantic_planner",
            "default",
            seed_group="decision",
            fallback="decision.modules.semantic_planner_module.SemanticPlannerModule",
        )
        LLMModule = stack_module(
            "llm",
            "pluggable",
            seed_group="llm",
            fallback="decision.modules.llm_module.LLMModule",
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

    VisualServoModule = optional_stack_module(
        "visual_servo",
        "default",
        seed_group="decision",
        fallback="decision.modules.visual_servo_module.VisualServoModule",
    )
    if VisualServoModule is not None:
        bp.add(VisualServoModule, alias="VisualServoModule")

    return bp
