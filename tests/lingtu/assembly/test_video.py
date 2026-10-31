from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

import pytest

from lingtu.assembly.build_scripts import product_native_build_scripts
from lingtu.assembly.compile_plan import compile_run_plan
from lingtu.assembly.config.product import resolve_product_host_runtime
from lingtu.assembly.graph import RuntimeGraph, load_runtime_graph
from lingtu.control import ProductControl
from lingtu.run_plan import RunPlan


@pytest.mark.parametrize("product", ["map", "nav", "tracking"])
def test_preview_plan_owns_video_without_weakening_camera_or_navigation(product, tmp_path):
    plan = compile_run_plan(product, "real", robot="unitree/go2", product_variant="camera")
    video = plan.process("video")
    assert video.target == "lt-video.service"
    assert video.required is False
    assert all(process.required for process in plan.processes if process.name != "video")
    assert plan.process("camera").order < video.order < plan.process("host").order
    assert plan.stop_before_start.index(video.target) < plan.stop_before_start.index(plan.process("camera").target)
    assert plan.process_environment["LINGTU_WEBRTC_URL"] == (
        plan.process_environment["LINGTU_VIDEO_LISTEN"] + "/whep/endpoint"
    )
    assert plan.process_environment["LINGTU_VIDEO_STATUS_FILE"] == "/dev/shm/lingtu/video_status.json"
    assert RunPlan.load(plan.write(tmp_path / "plan.json")) == plan
    assert "scripts/build/build_camera_video.sh" in product_native_build_scripts(product, product_variant="camera")


@pytest.mark.parametrize("product", ["nav", "tracking"])
def test_standard_plan_does_not_start_or_advertise_native_video(product):
    plan = compile_run_plan(product, "real", robot="unitree/go2", product_variant="standard")
    assert plan.has_process("camera") is (product == "tracking")
    assert not plan.has_process("video")
    assert plan.process_environment["LINGTU_WEBRTC_URL"] == ""
    assert all(process.required for process in plan.processes)
    assert "scripts/build/build_camera_video.sh" not in product_native_build_scripts(
        product,
        product_variant="standard",
    )


def test_tracking_preview_declares_camera_observation_and_preview_capability():
    plan = compile_run_plan("tracking", "real", robot="unitree/go2", product_variant="camera")
    assert {
        "/camera/color/image_raw",
        "/camera/depth/image_raw",
        "/camera/color/camera_info",
    } <= set(plan.required_topics)
    assert "camera_preview" in plan.required_capabilities
    assert {"PerceptionModule", "VisualServoModule"} <= set(plan.critical_modules)


def test_sim_tracking_keeps_standard_observation_chain_without_native_preview():
    standard = resolve_product_host_runtime(
        "tracking",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
    )
    assert standard.product_variant == "standard"
    assert "video" not in standard.product_spec["processes"]
    assert "camera_preview" not in standard.product_spec["capabilities"]

    with pytest.raises(ValueError, match="does not support Product 'tracking' variant 'camera'"):
        resolve_product_host_runtime(
            "tracking",
            "sim",
            robot="doso/thunder_v4",
            product_variant="camera",
            env_config={"backend": "mujoco"},
        )


@pytest.mark.parametrize("optional", [["nav"], ["missing"], ["video", "video"]])
def test_product_cannot_weaken_a_core_role_or_hide_a_typo(optional):
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["variants"]["camera"]["optional_processes"] = optional
    graph = RuntimeGraph(root=graph.root, topics=graph.topics, products=products, envs=graph.envs)
    with pytest.raises(ValueError, match=r"optional|duplicate"):
        compile_run_plan("nav", "real", robot="unitree/go2", product_variant="camera", graph=graph)


def test_previous_plan_requires_regeneration_instead_of_assuming_criticality():
    plan = compile_run_plan("nav", "real", robot="unitree/go2", product_variant="camera")
    payload = plan.as_dict()
    payload["identity"]["schema"] = "lingtu.run_plan.v10"
    with pytest.raises(ValueError, match="unsupported RunPlan schema"):
        RunPlan.from_dict(payload)


def test_current_product_exposes_preview_failure_and_recovery(monkeypatch, tmp_path):
    detail = {"available": True, "active": True, "ready": False, "blockers": ["source_stale"]}
    runner = SimpleNamespace(inspect=lambda process: dict(detail))
    control = ProductControl(runner=runner, robot="unitree/go2", env="real", process_env={})
    plan = compile_run_plan("map", "real", robot="unitree/go2", product_variant="camera")
    monkeypatch.setattr(control, "_current_plan_and_path", lambda root: (plan, tmp_path / "plan.json", "product-test"))
    result = control.status(state_dir=tmp_path)
    assert result["status"] == "active" and result["ok"] is True
    assert result["processes"]["video"]["ready"] is False
    assert result["degraded"]["video"]["reason"] == "source_stale"

    detail.update(ready=True, blockers=[])
    assert control.status(state_dir=tmp_path)["degraded"] == {}
