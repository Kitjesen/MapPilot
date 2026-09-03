from __future__ import annotations

import inspect
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest
import yaml

from decision.modules.visual_servo import VisualServoModule
from lingtu.assembly.compiler import blueprint_from_run_plan, compile_run_plan
from lingtu.assembly.products.host import host_blueprint
from lingtu.assembly.stacks import perception as perception_stack
from lingtu.assembly.stacks.composition import compose_full_stack_modules
from lingtu.assembly.wires.context import ODOMETRY_CONSUMERS, build_wiring_context
from lingtu.assembly.wires.slam import localization_specs, odometry_fanout_specs
from perception.backends import create_observation_source
from runtime.config import (
    CameraConfig,
    DetectorConfig,
    PerceptionConfig,
    RobotConfig,
    TrackingConfig,
    load_config,
)
from runtime.graph.processes import ProcessArtifact
from runtime.module import Module

REPO_ROOT = Path(__file__).resolve().parents[2]


class _FakePerception(Module, layer=3):
    pass


def _perception_entry(blueprint):
    return next(entry for entry in blueprint._entries if entry.name == "PerceptionModule")


def test_product_and_stack_interfaces_do_not_expose_perception_encoder() -> None:
    for product in ("tracking", "inspection"):
        data = yaml.safe_load(
            (REPO_ROOT / "config" / "runtime_graph" / "products" / f"{product}.yaml").read_text(
                encoding="utf-8"
            )
        )
        assert "encoder" not in data["host"]

    assert "encoder" not in inspect.signature(perception_stack.perception).parameters
    assert "encoder" not in inspect.signature(compose_full_stack_modules).parameters
    with pytest.raises(ValueError, match="encoder"):
        perception_stack.perception(encoder="mobileclip", enable_camera=False)
    with pytest.raises(ValueError, match="encoder"):
        host_blueprint(encoder="mobileclip", run_startup_checks=False)


def test_perception_config_owns_sync_thresholds_but_not_camera_depth_scale() -> None:
    config = PerceptionConfig()

    assert not hasattr(config, "depth_scale")
    assert config.max_rgbd_skew_s == pytest.approx(0.05)
    assert config.max_odom_age_s == pytest.approx(0.10)
    assert config.max_map_odom_age_s == pytest.approx(0.50)


def test_robot_yaml_loads_camera_depth_scale_and_perception_sync_thresholds(
    tmp_path: Path,
) -> None:
    path = tmp_path / "robot.yaml"
    path.write_text(
        """
camera:
  depth_scale: 0.004
perception:
  depth_scale: 99
  max_rgbd_skew_s: 0.07
  max_odom_age_s: 0.12
  max_map_odom_age_s: 0.55
  detector:
    confidence_threshold: 0.8
""",
        encoding="utf-8",
    )

    config = load_config(str(path))

    assert config.camera.depth_scale == pytest.approx(0.004)
    assert not hasattr(config.perception, "depth_scale")
    assert config.perception.max_rgbd_skew_s == pytest.approx(0.07)
    assert config.perception.max_odom_age_s == pytest.approx(0.12)
    assert config.perception.max_map_odom_age_s == pytest.approx(0.55)
    assert config.perception.detector.confidence_threshold == pytest.approx(0.8)


def test_perception_stack_resolves_one_typed_configuration(monkeypatch) -> None:
    robot_config = RobotConfig(
        camera=CameraConfig(position_x=0.37, depth_scale=0.0025),
        perception=PerceptionConfig(
            default_classes="person . extinguisher",
            skip_frames=4,
            min_depth=0.42,
            max_depth=8.5,
            laplacian_threshold=73.0,
            max_rgbd_skew_s=0.08,
            max_odom_age_s=0.14,
            max_map_odom_age_s=0.61,
            detector=DetectorConfig(
                confidence_threshold=0.57,
                iou_threshold=0.39,
                max_detections=31,
                min_box_size_px=9,
                model_size="s",
            ),
            tracking=TrackingConfig(
                merge_distance=0.74,
                iou_threshold=0.28,
                max_objects=91,
            ),
        ),
    )
    calls = 0

    def fake_get_config() -> RobotConfig:
        nonlocal calls
        calls += 1
        return robot_config

    monkeypatch.setattr(perception_stack, "get_config", fake_get_config, raising=False)
    monkeypatch.setattr(perception_stack, "stack_module", lambda *args, **kwargs: _FakePerception)

    blueprint = perception_stack.perception(detector="bpu", enable_camera=True)
    entry = _perception_entry(blueprint)
    settings = entry.config["settings"]
    detector = entry.config["detector"]

    assert calls == 1
    assert settings.default_classes == "person . extinguisher"
    assert settings.min_depth == pytest.approx(0.42)
    assert settings.max_depth == pytest.approx(8.5)
    assert settings.u16_depth_scale == pytest.approx(0.0025)
    assert settings.laplacian_threshold == pytest.approx(73.0)
    assert settings.merge_distance == pytest.approx(0.74)
    assert settings.tracking_iou_threshold == pytest.approx(0.28)
    assert settings.max_objects == 91
    assert detector.name == "bpu"
    assert detector.confidence == pytest.approx(0.57)
    assert detector.iou_threshold == pytest.approx(0.39)
    assert detector.max_detections == 31
    assert detector.min_box_size_px == 9
    assert detector.model_size == "s"
    assert entry.config["skip_frames"] == 4
    assert entry.config["max_rgbd_skew_s"] == pytest.approx(0.08)
    assert entry.config["max_odom_age_s"] == pytest.approx(0.14)
    assert entry.config["max_map_odom_age_s"] == pytest.approx(0.61)
    assert entry.config["camera_to_body"][0][3] == pytest.approx(0.37)
    assert not {
        "detector_type",
        "encoder_type",
        "confidence_threshold",
        "tracking_iou_threshold",
        "detector_iou_threshold",
    }.intersection(entry.config)

    with pytest.raises(FrozenInstanceError):
        settings.min_depth = 1.0
    with pytest.raises(FrozenInstanceError):
        detector.name = "yoloe"


def test_explicit_product_values_only_override_the_named_typed_values(monkeypatch) -> None:
    robot_config = RobotConfig(
        camera=CameraConfig(depth_scale=0.003),
        perception=PerceptionConfig(
            skip_frames=3,
            detector=DetectorConfig(
                confidence_threshold=0.41,
                iou_threshold=0.37,
                max_detections=29,
                min_box_size_px=7,
                model_size="m",
            ),
        ),
    )
    monkeypatch.setattr(perception_stack, "get_config", lambda: robot_config, raising=False)
    monkeypatch.setattr(perception_stack, "stack_module", lambda *args, **kwargs: _FakePerception)

    blueprint = perception_stack.perception(
        detector="yolo_world",
        enable_camera=False,
        confidence=0.83,
        perception_skip_frames=6,
    )
    entry = _perception_entry(blueprint)
    detector = entry.config["detector"]

    assert detector.name == "yolo_world"
    assert detector.confidence == pytest.approx(0.83)
    assert detector.iou_threshold == pytest.approx(0.37)
    assert detector.max_detections == 29
    assert detector.min_box_size_px == 7
    assert detector.model_size == "m"
    assert entry.config["skip_frames"] == 6


def test_perception_receives_explicit_selected_odometry() -> None:
    assert "PerceptionModule" in ODOMETRY_CONSUMERS
    context = build_wiring_context(
        {"SlamAdapterModule", "PerceptionModule"},
        driver_module="ThunderDriver",
        slam_profile="native_dds",
    )

    wires = {
        (spec.out_module, spec.out_port, spec.in_module, spec.in_port)
        for spec in odometry_fanout_specs(context)
    }

    assert (
        "SlamAdapterModule",
        "odometry",
        "PerceptionModule",
        "odometry",
    ) in wires

    localization_wires = {
        (spec.out_module, spec.out_port, spec.in_module, spec.in_port)
        for spec in localization_specs(context)
    }
    assert (
        "SlamAdapterModule",
        "map_odom_tf",
        "PerceptionModule",
        "map_odom_tf",
    ) in localization_wires


def test_visual_servo_does_not_reach_into_perception_for_an_encoder() -> None:
    class _Encoder:
        def encode_text(self, values):
            return values

        def encode_image(self, image):
            return image

    class _Perception:
        image_encoder = _Encoder()

    visual_servo = VisualServoModule()
    visual_servo.on_system_modules({"PerceptionModule": _Perception()})

    assert not hasattr(visual_servo, "_perception_module")
    assert visual_servo._person_tracker.has_image_selector is False


def test_sim_run_plan_builds_complete_sim_scene_detector_spec(monkeypatch) -> None:
    monkeypatch.setattr(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    )
    plan = compile_run_plan(
        "tracking",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
    )

    blueprint = blueprint_from_run_plan(plan)
    entry = _perception_entry(blueprint)
    detector = entry.config["detector"]

    assert detector.name == "sim_scene"
    assert detector.world == (
        "sim/packages/worlds/industrial_park/physics/industrial_park_scene.xml"
    )
    assert [entity["entity_id"] for entity in detector.scenario_entities] == [
        "person_01"
    ]
    source = create_observation_source(
        detector,
        min_depth=entry.config["settings"].min_depth,
        max_depth=entry.config["settings"].max_depth,
        u16_depth_scale=entry.config["settings"].u16_depth_scale,
    )
    source.load()
    assert source.health()["loaded"] is True
    source.close()


def test_sim_run_plan_forces_sim_scene_over_explicit_detector_override(monkeypatch) -> None:
    monkeypatch.setattr(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    )

    plan = compile_run_plan(
        "tracking",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
        overrides={"detector": "yoloe"},
    )

    detector = _perception_entry(blueprint_from_run_plan(plan)).config["detector"]

    assert detector.name == "sim_scene"
    assert detector.world == (
        "sim/packages/worlds/industrial_park/physics/industrial_park_scene.xml"
    )
