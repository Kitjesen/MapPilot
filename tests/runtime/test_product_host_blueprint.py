from __future__ import annotations

import pytest

from lingtu.assembly.compiler import (
    blueprint_for_resolved_product,
    blueprint_from_run_plan,
    compile_run_plan,
)
from lingtu.assembly.plugins import install_builtin_plugin_catalog
from lingtu.assembly.products import resolve_product_host_config
from lingtu.assembly.products.host import host_blueprint

install_builtin_plugin_catalog()


def test_direct_host_blueprint_defaults_to_stub_driver() -> None:
    blueprint = host_blueprint(
        slam_profile="none",
        enable_semantic=False,
        enable_gateway=False,
        enable_navigation=False,
        run_startup_checks=False,
    )

    assert "StubDogModule" in blueprint.export_graph().module_names


@pytest.mark.parametrize("product", ("teleop", "map", "nav"))
def test_real_product_host_builds_without_python_robot_driver(product: str) -> None:
    config = resolve_product_host_config(product, "real", robot="unitree/go2")
    assert config["enable_robot_driver"] is False
    assert "enable_map_modules" not in config
    blueprint = blueprint_for_resolved_product(product, config)
    assert "ThunderDriver" not in blueprint.export_graph().module_names


def test_inspection_product_only_loads_declared_semantic_planning_modules() -> None:
    config = resolve_product_host_config("inspection", "real", robot="unitree/go2")
    blueprint = blueprint_for_resolved_product("inspection", config)
    names = blueprint.export_graph().module_names

    assert "SemanticPlannerModule" in names
    assert "LLMModule" in names
    assert "AgentPlannerModule" not in names
    assert "VisualServoModule" in names


@pytest.mark.parametrize(
    ("env", "robot", "env_config"),
    (
        ("real", "unitree/go2", None),
        ("sim", "doso/thunder_v4", {"backend": "mujoco"}),
    ),
)
def test_tracking_product_builds_only_the_person_following_host(
    env: str,
    robot: str | None,
    env_config: dict[str, str] | None,
    allow_unbuilt_process_artifacts: None,
) -> None:
    plan = compile_run_plan(
        "tracking",
        env,
        robot=robot,
        env_config=env_config,
    )
    blueprint = blueprint_from_run_plan(plan)
    graph = blueprint.export_graph()
    names = set(graph.module_names)
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in graph.explicit_wires
    }

    assert plan.has_process("camera")
    assert {
        "camera",
        "PerceptionModule",
        "VisualServoModule",
        "GatewayModule",
        "host.bus",
        "nav.commands",
        "nav.goals",
    } <= names
    assert not {
        "SemanticPlannerModule",
        "LLMModule",
        "SemanticMapperModule",
        "EpisodicMemoryModule",
        "TaggedLocationsModule",
        "VectorMemoryModule",
        "TemporalMemoryModule",
        "MissionLoggerModule",
        "InspectionEvidenceModule",
    }.intersection(names)
    assert {
        "camera.color_image->PerceptionModule.color_image",
        "camera.depth_image->PerceptionModule.depth_image",
        "camera.camera_info->PerceptionModule.camera_info",
        "PerceptionModule.detections_3d->VisualServoModule.detections_3d",
        "PerceptionModule.robot_pose->VisualServoModule.robot_pose",
        "PerceptionModule.scene_graph->GatewayModule.scene_graph",
        "GatewayModule.servo_target->VisualServoModule.servo_target",
        "VisualServoModule.goal_pose->nav.goals.visual_goal_request",
        "VisualServoModule.goal_cancel->nav.goals.visual_cancel_request",
        "nav.goals.goal_status->VisualServoModule.goal_status",
        "VisualServoModule.servo_status->GatewayModule.visual_servo_status",
    } <= wires
