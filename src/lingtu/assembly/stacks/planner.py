"""Semantic planning stack."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.plugin_resolution import optional_stack_module, stack_module

from .memory import DEFAULT_SEMANTIC_DIR

logger = logging.getLogger(__name__)


def planner(
    llm: str = "kimi",
    save_dir: str = "",
    vla_backend: str = "",
    enable_semantic_planning: bool = True,
    **config,
) -> Blueprint:
    """Build visual following with optional semantic goal resolution."""
    bp = Blueprint()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    if enable_semantic_planning:
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

    VisualServoModule = stack_module(
        "visual_servo",
        "default",
        seed_group="decision",
        fallback="decision.modules.visual_servo.VisualServoModule",
    )
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
