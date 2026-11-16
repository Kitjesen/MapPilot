from __future__ import annotations

import pytest

import decision.modules.visual_servo as visual_servo_module
from decision.modules.visual_servo import VisualServoModule
from lingtu.assembly.compile_plan import (
    compile_run_plan,
    host_builder_from_run_plan,
)
from lingtu.assembly.config.product import resolve_product_host_config
from lingtu.assembly.graph.processes import ProcessArtifact
from lingtu.assembly.host.build import host_builder
from lingtu.assembly.host.plugin_catalog import install_builtin_plugin_catalog
from perception.backends import DinoEncoderProvider

install_builtin_plugin_catalog()


@pytest.mark.parametrize(("env", "port"), [("real", 5053), ("sim", 5050)])
def test_gateway_port_reaches_host(env, port, allow_unbuilt_process_artifacts) -> None:
    plan = compile_run_plan("map", env, robot="unitree/go2" if env == "real" else "doso/thunder_v4")
    builder = host_builder_from_run_plan(plan)
    gateway = next(entry for entry in builder._entries if entry.alias == "GatewayModule")
    assert plan.host_config["gateway_port"] == port
    assert gateway.config["port"] == port


def test_direct_host_builder_defaults_to_stub_driver() -> None:
    builder = host_builder(
        slam_profile="none",
        enable_semantic=False,
        enable_gateway=False,
        enable_navigation=False,
        run_startup_checks=False,
    )

    assert "StubDogModule" in builder.export_graph().module_names


def test_host_config_reaches_visual_servo_person_encoders() -> None:
    builder = host_builder(
        slam_profile="none",
        enable_gateway=False,
        enable_navigation=False,
        run_startup_checks=False,
        person_text_model_path="models/clip",
        person_text_device="cuda",
        person_appearance_model="models/dinov3.onnx",
        person_appearance_device="cpu",
        person_appearance_cpu_threads=8,
        person_appearance_min_similarity=0.72,
        person_appearance_margin=0.08,
    )

    entry = next(
        item for item in builder._entries if item.alias == "VisualServoModule"
    )
    assert entry.config == {
        "vision_service_url": "",
        "person_text_model_path": "models/clip",
        "person_text_device": "cuda",
        "person_appearance_model": "models/dinov3.onnx",
        "person_appearance_device": "cpu",
        "person_appearance_cpu_threads": 8,
        "person_appearance_min_similarity": 0.72,
        "person_appearance_margin": 0.08,
    }


@pytest.mark.parametrize("product", ("nav", "inspection"))
def test_product_model_config_reaches_all_semantic_clients(product: str) -> None:
    config = resolve_product_host_config(
        product, "real", robot="unitree/go2",
        product_variant="semantic" if product == "nav" else None,
    )
    config.update(
        llm="qwen",
        llm_model="selected-model",
        llm_base_url="https://example.invalid/v1",
        llm_api_key_env="TEST_LLM_KEY",
        llm_timeout_sec=7.5,
    )
    builder = host_builder(config, product=product)
    entries = {item.alias: item for item in builder._entries}

    for alias in ("SemanticPlannerModule", "AgentPlannerModule"):
        assert {key: entries[alias].config[key] for key in (
            "llm_backend", "llm_model", "llm_base_url", "llm_api_key_env", "llm_timeout_sec",
        )} == {
            "llm_backend": "qwen",
            "llm_model": "selected-model",
            "llm_base_url": "https://example.invalid/v1",
            "llm_api_key_env": "TEST_LLM_KEY",
            "llm_timeout_sec": 7.5,
        }
    assert entries["LLMModule"].config == {
        "backend": "qwen",
        "model": "selected-model",
        "base_url": "https://example.invalid/v1",
        "api_key_env": "TEST_LLM_KEY",
        "timeout_sec": 7.5,
    }


def test_host_vision_service_reaches_detector_and_visual_servo() -> None:
    builder = host_builder(
        slam_profile="none", enable_gateway=False, enable_navigation=False,
        run_startup_checks=False, detector="remote",
        vision_service_url="http://127.0.0.1:8000",
    )
    entries = {item.alias: item for item in builder._entries}
    assert entries["VisualServoModule"].config["vision_service_url"] == "http://127.0.0.1:8000"
    assert entries["PerceptionModule"].config["detector"].server_url == "http://127.0.0.1:8000"


def test_remote_follow_encoders_and_vision_client_stay_independent_of_text_llm(monkeypatch) -> None:
    created = {}

    class FakeEncoder:
        supports_vision = True

        def load_model(self):
            self.loaded = True

        def is_available(self):
            return True

        async def chat(self, messages, **kwargs):
            return "MATCH"

        def shutdown(self):
            self.loaded = False

    class FakeProvider:
        def __init__(self, kind):
            self.kind = kind

        def create(self, **config):
            client = FakeEncoder()
            created[self.kind] = (client, config)
            return client

    monkeypatch.setattr(visual_servo_module, "get", lambda category, name: FakeProvider(name))
    module = VisualServoModule(
        vision_service_url="http://127.0.0.1:8000",
    )
    module.setup()
    module.on_system_modules({})
    assert set(created) == {"clip_remote", "dino_remote", "remote"}
    assert all(client.loaded for client, _ in created.values())
    assert all(config == {"server_url": "http://127.0.0.1:8000"} for _, config in created.values())
    assert module._person_tracker._clip_encoder is created["clip_remote"][0]
    assert module._person_tracker._dino_encoder is created["dino_remote"][0]
    assert module._vision_client is created["remote"][0]
    module.stop()
    assert not created["clip_remote"][0].loaded
    assert not created["dino_remote"][0].loaded
    assert created["remote"][0].loaded


def test_local_dino_stays_onboard_with_remote_clip_and_vision(monkeypatch) -> None:
    created = {}

    class FakeEncoder:
        def load_model(self):
            pass

    class FakeProvider:
        def __init__(self, kind):
            self.kind = kind

        def create(self, **config):
            encoder = FakeEncoder()
            created[self.kind] = (encoder, config)
            return encoder

    monkeypatch.setattr(visual_servo_module, "get", lambda category, name: FakeProvider(name))
    module = VisualServoModule(
        person_appearance_model="models/dinov3.engine",
        vision_service_url="http://127.0.0.1:8000",
    )
    module.setup()

    assert set(created) == {"clip_remote", "dino", "remote"}
    assert created["dino"][1] == {
        "model_path": "models/dinov3.engine", "device": "auto", "cpu_threads": 4,
    }
    assert module._person_tracker._dino_encoder is created["dino"][0]
    assert module._person_tracker._clip_encoder is created["clip_remote"][0]
    assert module._vision_client is created["remote"][0]


def test_visual_servo_setup_loads_independent_clip_and_dino_encoders(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeEncoder:
        def __init__(self, kind: str) -> None:
            self.kind = kind
            self.loaded = False

        def load_model(self) -> None:
            self.loaded = True

    created: dict[str, tuple[FakeEncoder, dict[str, object]]] = {}

    class FakeProvider:
        def __init__(self, kind: str) -> None:
            self.kind = kind

        def create(self, **config: object) -> FakeEncoder:
            encoder = FakeEncoder(self.kind)
            created[self.kind] = (encoder, config)
            return encoder

    monkeypatch.setattr(
        visual_servo_module,
        "get",
        lambda category, name: FakeProvider(name),
    )
    module = VisualServoModule(
        person_text_model_path="models/clip",
        person_text_device="cuda",
        person_appearance_model="models/dinov3.onnx",
        person_appearance_device="cpu",
        person_appearance_cpu_threads=8,
        person_appearance_min_similarity=0.72,
        person_appearance_margin=0.08,
    )

    module.setup()

    clip_encoder, clip_config = created["clip"]
    dino_encoder, dino_config = created["dino"]
    assert clip_config == {"model_path": "models/clip", "device": "cuda"}
    assert dino_config == {
        "model_path": "models/dinov3.onnx",
        "device": "cpu",
        "cpu_threads": 8,
    }
    assert clip_encoder.loaded is True
    assert dino_encoder.loaded is True
    assert module._person_tracker._clip_encoder is clip_encoder
    assert module._person_tracker._dino_encoder is dino_encoder
    assert module._person_tracker._dino_min_similarity == 0.72
    assert module._person_tracker._dino_margin == 0.08


def test_visual_servo_defaults_dino_cpu_threads_to_four(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    created = {}

    class FakeEncoder:
        def load_model(self) -> None:
            pass

    class FakeProvider:
        @staticmethod
        def create(**config: object) -> FakeEncoder:
            created.update(config)
            return FakeEncoder()

    monkeypatch.setattr(visual_servo_module, "get", lambda _category, _name: FakeProvider())
    module = VisualServoModule(person_appearance_model="models/dinov3.onnx")

    module.setup()

    assert created["cpu_threads"] == 4


def test_dino_provider_passes_explicit_cpu_threads(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    created = {}

    class FakeDinoEncoder:
        def __init__(self, **config: object) -> None:
            created.update(config)

    monkeypatch.setattr("perception.encoding.dino.DinoEncoder", FakeDinoEncoder)

    DinoEncoderProvider.create(
        model_path="models/dinov3.onnx",
        device="cpu",
        cpu_threads=8,
    )

    assert created == {
        "model_path": "models/dinov3.onnx",
        "device": "cpu",
        "cpu_threads": 8,
    }


def test_dino_cpu_threads_survive_immutable_run_plan(allow_unbuilt_process_artifacts) -> None:
    plan = compile_run_plan(
        "tracking",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
        overrides={"person_appearance_cpu_threads": 8},
    )

    host_config = plan.host_config
    assert host_config["person_appearance_cpu_threads"] == 8
    host_config["person_appearance_cpu_threads"] = 2
    assert plan.host_config["person_appearance_cpu_threads"] == 8


def test_tracking_only_requests_people_without_changing_other_product_vocabularies() -> None:
    tracking = resolve_product_host_config("tracking", "real", robot="unitree/go2")
    inspection = resolve_product_host_config("inspection", "real", robot="unitree/go2")
    assert tracking["default_classes"] == "person"
    assert "default_classes" not in inspection


@pytest.mark.parametrize("product", ("teleop", "map", "nav"))
def test_real_product_host_builds_without_python_robot_driver(product: str) -> None:
    config = resolve_product_host_config(product, "real", robot="unitree/go2")
    assert config["enable_robot_driver"] is False
    assert "enable_map_modules" not in config
    builder = host_builder(config, product=product)
    assert "ThunderDriver" not in builder.export_graph().module_names


def test_inspection_product_only_loads_declared_semantic_planning_modules() -> None:
    config = resolve_product_host_config("inspection", "real", robot="unitree/go2")
    builder = host_builder(config, product="inspection")
    names = builder.export_graph().module_names

    assert "SemanticPlannerModule" in names
    assert "LLMModule" in names
    assert "AgentPlannerModule" in names
    assert "VisualServoModule" in names
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in builder.export_graph().explicit_wires
    }
    assert {
        "host.bus.navigation_goal_status->nav.goals.navigation_goal_status",
        "SemanticPlannerModule.nav_command->nav.goals.goal_command",
        "nav.goals.goal_status->SemanticPlannerModule.goal_status",
        "nav.goals.task_status->SemanticPlannerModule.navigation_goal_status",
        "PerceptionModule.observation->SemanticPlannerModule.observation",
        "PerceptionModule.robot_pose->AgentPlannerModule.robot_pose",
        "PerceptionModule.observation_image->AgentPlannerModule.observation_image",
        "PerceptionModule.robot_pose->VectorMemoryModule.robot_pose",
        "host.bus.navigation_state->VectorMemoryModule.navigation_state",
    } <= wires
    assert not any(wire.endswith("->SemanticPlannerModule.odometry") for wire in wires)
    assert not any(wire.startswith("AgentPlannerModule.goal_pose->") for wire in wires)
    assert not any(wire.startswith("AgentPlannerModule.servo_target->") for wire in wires)


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
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    )
    plan = compile_run_plan(
        "tracking",
        env,
        robot=robot,
        env_config=env_config,
    )
    host_builder = host_builder_from_run_plan(plan)
    graph = host_builder.export_graph()
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
        "AgentPlannerModule",
        "SemanticMapperModule",
        "EpisodicMemoryModule",
        "TaggedLocationsModule",
        "VectorMemoryModule",
        "TemporalMemoryModule",
        "InspectionEvidenceModule",
    }.intersection(names)
    assert "LLMModule" in names
    llm_entries = [entry for entry in host_builder._entries if entry.alias == "LLMModule"]
    assert len(llm_entries) == 1
    assert llm_entries[0].config == {
        "backend": "kimi",
        "model": "kimi-k3",
        "base_url": "https://api.moonshot.cn/v1",
        "api_key_env": "MOONSHOT_API_KEY",
        "timeout_sec": 30.0,
    }
    assert {
        "camera.color_image->PerceptionModule.color_image",
        "camera.depth_image->PerceptionModule.depth_image",
        "camera.camera_info->PerceptionModule.camera_info",
        "PerceptionModule.observation->VisualServoModule.observation",
        "PerceptionModule.scene_graph->GatewayModule.scene_graph",
        "GatewayModule.servo_target->VisualServoModule.servo_target",
        "VisualServoModule.goal_pose->nav.goals.visual_goal_request",
        "VisualServoModule.goal_cancel->nav.goals.visual_cancel_request",
        "nav.goals.goal_status->VisualServoModule.goal_status",
        "VisualServoModule.servo_status->GatewayModule.visual_servo_status",
    } <= wires
    assert "camera.color_image->VisualServoModule.color_image" not in wires
