import pytest

from lingtu.assembly.compile_plan import compile_run_plan, host_builder_from_run_plan


@pytest.mark.parametrize("env,robot", [("real", "unitree/go2"), ("real", "doso/thunder_v4"), ("sim", "doso/thunder_v4")])
@pytest.mark.parametrize("product", ["map", "nav"])
def test_standard_products_include_conversation_without_perception(robot, product, env):
    plan = compile_run_plan(product, env, robot=robot)
    builder = host_builder_from_run_plan(plan)
    names = list(builder.module_names)
    assert names.count("AgentPlannerModule") == 1
    assert not {"PerceptionModule", "SemanticPlannerModule", "VisualServoModule", "LLMModule"} & set(names)
    assert plan.host_config["enable_conversation"] is True
    assert plan.host_config["enable_semantic"] is False
    agent = next(entry for entry in builder._entries if entry.name == "AgentPlannerModule")
    assert agent.config["llm_model"] == plan.host_config["llm_model"]
    assert plan.host_config["llm_api_key_env"] == "MOONSHOT_API_KEY"
    assert names == list(plan.modules)


def test_semantic_variant_keeps_one_agent():
    plan = compile_run_plan("nav", "real", robot="unitree/go2", product_variant="semantic")
    assert list(host_builder_from_run_plan(plan).module_names).count("AgentPlannerModule") == 1
