from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from lingtu.assembly.compile_plan import compile_run_plan, host_builder_from_run_plan
from runtime.config import load_config
from runtime.utils.calibration_check import run_calibration_check


def test_real_nav_semantic_variant_compiles_the_existing_semantic_stack() -> None:
    standard = compile_run_plan("nav", "real", robot="unitree/go2")
    plan = compile_run_plan(
        "nav",
        "real",
        robot="unitree/go2",
        product_variant="semantic",
    )
    builder = host_builder_from_run_plan(plan)

    assert plan.product_variant == "semantic"
    assert plan.native_nav == standard.native_nav
    assert {process.name for process in plan.processes} == {
        "lidar",
        "slam",
        "maps",
        "nav",
        "driver",
        "camera",
        "video",
        "host",
    }
    assert plan.process("video").required is False
    assert {
        "/camera/color/image_raw",
        "/camera/depth/image_raw",
        "/camera/color/camera_info",
        "/nav/semantic/views/request",
        "/nav/semantic/views/result",
    } <= set(plan.required_topics)
    assert {
        "PerceptionModule",
        "LLMModule",
        "SemanticPlannerModule",
        "AgentPlannerModule",
        "VisualServoModule",
    } <= set(builder.module_names)
    assert builder.module_names == plan.modules
    assert builder.required_module_names == plan.critical_modules


def test_semantic_variant_resolves_explicit_model_and_detector_contract() -> None:
    plan = compile_run_plan(
        "nav",
        "real",
        robot="unitree/go2",
        product_variant="semantic",
    )

    assert plan.host_config["detector"] == "yolo_world"
    assert plan.host_config["default_classes"] == "person . bottle . cup"
    assert plan.host_config["llm"] == "kimi"
    assert plan.host_config["llm_model"] == "kimi-k3"
    assert plan.host_config["llm_base_url"] == "https://api.moonshot.cn/v1"
    assert plan.host_config["llm_api_key_env"] == "MOONSHOT_API_KEY"
    assert plan.host_config["llm_timeout_sec"] == 30.0
    assert plan.host_config["enable_semantic"] is True
    assert plan.host_config["enable_semantic_planning"] is True
    assert plan.host_config["run_startup_checks"] is True


def test_semantic_variant_keeps_robot_selection_and_calibration_boundary() -> None:
    go2 = compile_run_plan(
        "nav",
        "real",
        robot="unitree/go2",
        product_variant="semantic",
    )
    thunder = compile_run_plan(
        "nav",
        "real",
        robot="doso/thunder_v4",
        product_variant="semantic",
    )
    go2_config = yaml.safe_load(
        Path("config/robots/unitree/go2/robot.yaml").read_text(encoding="utf-8")
    )

    assert go2.robot == "unitree/go2"
    assert thunder.robot == "doso/thunder_v4"
    assert go2.host_config["driver_backend"] == "go2"
    assert thunder.host_config["driver_backend"] == "doso"
    assert go2.host_config["detector"] == thunder.host_config["detector"] == "yolo_world"
    assert go2_config["calibration"]["camera"]["status"] == "unverified"
    preflight = run_calibration_check(
        config=load_config("config/robots/unitree/go2/robot.yaml"),
        require_camera=True,
        require_slam=True,
    )
    assert preflight.ok is False
    assert any("Camera calibration is unverified" in error for error in preflight.errors)


def test_existing_nav_variants_and_sim_support_remain_unchanged() -> None:
    default = compile_run_plan("nav", "real", robot="unitree/go2")
    standard = compile_run_plan(
        "nav",
        "real",
        robot="unitree/go2",
        product_variant="standard",
    )
    camera = compile_run_plan(
        "nav",
        "real",
        robot="unitree/go2",
        product_variant="camera",
    )

    assert default == standard
    assert default.product_variant == "standard"
    assert default.host_config["detector"] == "bpu"
    assert camera.host_config["detector"] == "bpu"
    assert "SemanticPlannerModule" not in default.modules
    assert "SemanticPlannerModule" not in camera.modules
    with pytest.raises(ValueError, match="does not support Product 'nav' variant 'semantic'"):
        compile_run_plan(
            "nav",
            "sim",
            robot="doso/thunder_v4",
            product_variant="semantic",
            env_config={"backend": "mujoco"},
        )
