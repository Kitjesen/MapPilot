import pytest

pytestmark = [pytest.mark.sim]

from lingtu.assembly.products.thunder import thunder_blueprint


def test_sim_blueprint_wires_real_semantic_pipeline():
    robot = "sim_mujoco"
    driver_name = "MujocoDriverModule"
    camera_source = "camera"

    system = thunder_blueprint(
        robot=robot,
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        enable_map_modules=False,
    ).build()

    assert driver_name in system.modules
    assert camera_source in system.modules
    assert "PerceptionModule" in system.modules
    assert "EncoderModule" not in system.modules
    assert "VisualServoModule" in system.modules
    assert "SemanticPlannerModule" in system.modules

    connections = set(system.connections)

    def has_conn(src_mod: str, dst_mod: str, dst_port: str) -> bool:
        """Check if any connection exists from src_mod to dst_mod.dst_port."""
        return any(c[0] == src_mod and c[2] == dst_mod and c[3] == dst_port for c in connections)

    assert has_conn(camera_source, "PerceptionModule", "color_image")
    assert has_conn(camera_source, "PerceptionModule", "depth_image")
    assert has_conn(camera_source, "PerceptionModule", "camera_info")
    assert has_conn(camera_source, "VisualServoModule", "color_image")
    assert has_conn(camera_source, "VisualServoModule", "depth_image")
    assert has_conn(camera_source, "VisualServoModule", "camera_info")
    assert has_conn("PerceptionModule", "SemanticPlannerModule", "scene_graph")
    assert has_conn("PerceptionModule", "VisualServoModule", "scene_graph")
    assert has_conn("PerceptionModule", "SemanticPlannerModule", "detections")
    assert has_conn("SemanticMapperModule", "SemanticPlannerModule", "topo_summary")
    assert has_conn("SemanticMapperModule", "SemanticPlannerModule", "room_graph")


def test_sim_mujoco_blueprint_enables_camera_for_semantics():
    system = thunder_blueprint(
        robot="sim_mujoco",
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        enable_map_modules=False,
    ).build()

    driver = system.get_module("MujocoDriverModule")
    assert driver._enable_camera is True
    assert driver._publish_camera is False
    assert driver._publish_lidar is False
    assert driver._publish_imu is False
    assert "camera" in system.modules
    assert "lidar" not in system.modules

    connections = set(system.connections)
    assert (
        "MujocoDriverModule",
        "camera_image",
        "PerceptionModule",
        "color_image",
    ) not in connections
    assert (
        "MujocoDriverModule",
        "camera_image",
        "VisualServoModule",
        "color_image",
    ) not in connections
    assert (
        "camera",
        "color_image",
        "PerceptionModule",
        "color_image",
    ) in connections
    assert (
        "camera",
        "camera_info",
        "VisualServoModule",
        "camera_info",
    ) in connections


def test_semantic_blueprint_can_opt_into_standalone_encoder():
    system = thunder_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        enable_map_modules=False,
        enable_standalone_encoder=True,
    ).build()

    assert "PerceptionModule" in system.modules
    assert "EncoderModule" in system.modules
