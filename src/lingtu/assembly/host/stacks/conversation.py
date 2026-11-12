"""Conversational Agent without a perception or semantic-navigation stack."""

from runtime.host import HostBuilder
from runtime.plugin_resolution import stack_module


def conversation(llm: str = "kimi", **config) -> HostBuilder:
    module = stack_module("agent_planner", "default", seed_group="decision",
                          fallback="decision.modules.agent_planner.AgentPlannerModule")
    return HostBuilder().add(
        module, alias="AgentPlannerModule", llm_backend=llm,
        llm_model=str(config.get("llm_model", "")),
        llm_base_url=str(config.get("llm_base_url", "")),
        llm_api_key_env=str(config.get("llm_api_key_env", "")),
        llm_timeout_sec=float(config.get("llm_timeout_sec", 10.0)),
    )
