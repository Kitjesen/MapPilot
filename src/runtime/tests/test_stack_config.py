from __future__ import annotations

from runtime.blueprints.stacks.stack_config import (
    driver_stack_config,
    exploration_stack_config,
    needs_lidar_for_slam,
    perception_stack_config,
)


def test_driver_stack_config_adds_map_odom_for_in_process_drivers() -> None:
    config = driver_stack_config(
        {},
        slam_profile="none",
        driver_module="StubDogModule",
        enable_semantic=False,
    )

    assert config["odom_frame_id"] == "map"


def test_driver_stack_config_enables_mujoco_camera_for_semantic_mode() -> None:
    config = driver_stack_config(
        {},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=True,
    )

    assert config["enable_camera"] is True


def test_perception_stack_config_records_driver_module() -> None:
    config = perception_stack_config({"detector_debug": True}, driver_module="ThunderDriver")

    assert config["detector_debug"] is True
    assert config["_driver_cls_name"] == "ThunderDriver"


def test_perception_stack_config_does_not_auto_enable_ros2_camera_bridge() -> None:
    config = perception_stack_config({}, driver_module="ROS2SimDriverModule")

    assert config == {"_driver_cls_name": "ROS2SimDriverModule"}


def test_exploration_stack_config_keeps_only_supported_bridge_keys() -> None:
    config = exploration_stack_config(
        {
            "exploration_backend": "tare",
            "tare_scenario": "tunnel",
            "exploration_auto_start": False,
            "way_point_topic": "/waypoint",
            "ignored": "value",
        }
    )

    assert config == {
        "backend": "tare",
        "tare_scenario": "tunnel",
        "auto_start": False,
        "way_point_topic": "/waypoint",
    }


def test_needs_lidar_for_slam_only_for_managed_lidar_slam() -> None:
    assert needs_lidar_for_slam("fastlio2") is True
    assert needs_lidar_for_slam("localizer") is True
    assert needs_lidar_for_slam("bridge") is False
    assert needs_lidar_for_slam("none") is False
    assert needs_lidar_for_slam("super_lio") is False
