"""Semantic planning stack."""

from __future__ import annotations

import logging

from runtime.host import HostBuilder
from runtime.plugin_resolution import optional_stack_module, stack_module

from .memory import DEFAULT_SEMANTIC_DIR

logger = logging.getLogger(__name__)


def planner(
    llm: str = "kimi",
    save_dir: str = "",
    vla_backend: str = "",
    enable_semantic_planning: bool = True,
    *,
    llm_model: str = "",
    llm_base_url: str = "",
    llm_api_key_env: str = "",
    llm_timeout_sec: float = 10.0,
    vision_service_url: str = "",
    person_text_model_path: str = "",
    person_text_device: str = "auto",
    person_appearance_model: str = "",
    person_appearance_device: str = "auto",
    person_appearance_cpu_threads: int = 4,
    person_appearance_min_similarity: float = 0.65,
    person_appearance_margin: float = 0.03,
    **config,
) -> HostBuilder:
    """Build visual following with optional semantic goal resolution."""
    bp = HostBuilder()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    try:
        LLMModule = stack_module(
            "llm",
            "pluggable",
            seed_group="llm",
            fallback="decision.modules.llm.LLMModule",
        )
        bp.add(
            LLMModule,
            alias="LLMModule",
            backend=llm,
            model=llm_model,
            base_url=llm_base_url,
            api_key_env=llm_api_key_env,
            timeout_sec=llm_timeout_sec,
        )
    except ImportError as e:
        logger.warning("LLM module not available: %s", e)

    if enable_semantic_planning:
        try:
            SemanticPlannerModule = stack_module(
                "semantic_planner",
                "default",
                seed_group="decision",
                fallback="decision.modules.semantic_planner.SemanticPlannerModule",
            )
            AgentPlannerModule = stack_module(
                "agent_planner", "default", seed_group="decision",
                fallback="decision.modules.agent_planner.AgentPlannerModule",
            )
            bp.add(
                SemanticPlannerModule,
                alias="SemanticPlannerModule",
                save_dir=save_dir,
                llm_backend=llm,
                llm_model=llm_model,
                llm_base_url=llm_base_url,
                llm_api_key_env=llm_api_key_env,
                llm_timeout_sec=llm_timeout_sec,
                product_session_id=str(config.get("product_session_id", "")),
            )
            bp.add(
                AgentPlannerModule,
                alias="AgentPlannerModule",
                llm_backend=llm,
                llm_model=llm_model,
                llm_base_url=llm_base_url,
                llm_api_key_env=llm_api_key_env,
                llm_timeout_sec=llm_timeout_sec,
            )
        except ImportError as e:
            logger.warning("Semantic planner not available: %s", e)

    VisualServoModule = stack_module(
        "visual_servo",
        "default",
        seed_group="decision",
        fallback="decision.modules.visual_servo.VisualServoModule",
    )
    bp.add(
        VisualServoModule, alias="VisualServoModule",
        vision_service_url=vision_service_url,
        person_text_model_path=person_text_model_path,
        person_text_device=person_text_device,
        person_appearance_model=person_appearance_model,
        person_appearance_device=person_appearance_device,
        person_appearance_cpu_threads=person_appearance_cpu_threads,
        person_appearance_min_similarity=person_appearance_min_similarity,
        person_appearance_margin=person_appearance_margin,
    )

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
