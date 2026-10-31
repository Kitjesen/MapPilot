import pytest

pytestmark = [pytest.mark.sim]

from lingtu.assembly.host.build import host_builder


def test_sim_host_builder_wires_real_semantic_pipeline():
    robot = "sim_mujoco"
    driver_name = "MujocoDriverModule"
    camera_source = "camera"

    system = host_builder(
        robot=robot,
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        run_startup_checks=False,
    ).build()

    assert driver_name in system.modules
    assert camera_source in system.modules
    assert "PerceptionModule" in system.modules
    assert "VisualServoModule" in system.modules
    assert "AgentPlannerModule" in system.modules
    assert "SemanticPlannerModule" in system.modules

    connections = set(system.connections)

    def has_conn(src_mod: str, dst_mod: str, dst_port: str) -> bool:
        """Check if any connection exists from src_mod to dst_mod.dst_port."""
        return any(c[0] == src_mod and c[2] == dst_mod and c[3] == dst_port for c in connections)

    assert has_conn(camera_source, "PerceptionModule", "color_image")
    assert has_conn(camera_source, "PerceptionModule", "depth_image")
    assert has_conn(camera_source, "PerceptionModule", "camera_info")
    assert has_conn("PerceptionModule", "SemanticPlannerModule", "observation")
    for port in ("scene_graph", "robot_pose", "observation_image", "detections"):
        assert not has_conn("PerceptionModule", "SemanticPlannerModule", port)
    assert has_conn("PerceptionModule", "VisualServoModule", "observation")
    assert has_conn("host.bus", "PerceptionModule", "navigation_state")
    assert not has_conn("PerceptionModule", "VisualServoModule", "color_image")
    assert not has_conn("PerceptionModule", "VisualServoModule", "detections_3d")
    assert not has_conn(camera_source, "VisualServoModule", "color_image")
    assert has_conn("PerceptionModule", "AgentPlannerModule", "robot_pose")
    assert has_conn("PerceptionModule", "AgentPlannerModule", "observation_image")
    assert has_conn("SemanticMapperModule", "SemanticPlannerModule", "topo_summary")
    assert has_conn("SemanticMapperModule", "SemanticPlannerModule", "room_graph")


def test_sim_mujoco_host_builder_enables_camera_for_semantics():
    system = host_builder(
        robot="sim_mujoco",
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        run_startup_checks=False,
    ).build()

    driver = system.get_module("MujocoDriverModule")
    assert driver._enable_camera is True
    assert driver._publish_camera is False
    assert driver._publish_lidar is False
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
        "camera",
        "color_image",
        "PerceptionModule",
        "color_image",
    ) in connections
