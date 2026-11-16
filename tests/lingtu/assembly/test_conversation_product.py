import pytest

from lingtu.assembly.compile_plan import compile_run_plan, host_builder_from_run_plan

pytestmark = pytest.mark.usefixtures("allow_unbuilt_process_artifacts")


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


@pytest.mark.parametrize("env,robot", [("real", "unitree/go2"), ("sim", "doso/thunder_v4")])
def test_tracking_conversation_uses_visual_servo_without_semantic_planner(env, robot, allow_unbuilt_process_artifacts):
    plan = compile_run_plan("tracking", env, robot=robot)
    names = list(host_builder_from_run_plan(plan).module_names)
    assert names.count("AgentPlannerModule") == 1
    assert "VisualServoModule" in names and "PerceptionModule" in names
    assert "SemanticPlannerModule" not in names
    assert plan.host_config["enable_conversation"] is True
    assert names == list(plan.modules)
    assert plan.host_config["llm_timeout_sec"] == 30.0
    llm = next(entry for entry in host_builder_from_run_plan(plan)._entries if entry.name == "LLMModule")
    assert llm.config["timeout_sec"] == 30.0
