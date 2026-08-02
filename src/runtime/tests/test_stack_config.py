from __future__ import annotations

from lingtu.assembly.stacks.stack_config import (
    driver_stack_config,
    exploration_owner,
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


def test_driver_stack_config_aligns_mujoco_no_slam_cloud_frame() -> None:
    config = driver_stack_config(
        {},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=False,
    )

    assert config["odom_frame_id"] == "map"
    assert config["map_cloud_frame_id"] == "map"


def test_driver_stack_config_disables_mujoco_legacy_sensors_by_default() -> None:
    config = driver_stack_config(
        {},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=False,
    )

    assert config["publish_lidar"] is False
    assert config["publish_imu"] is False


def test_driver_stack_config_enables_mujoco_camera_for_semantic_mode() -> None:
    config = driver_stack_config(
        {},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=True,
    )

    assert config["enable_camera"] is True
    assert config["publish_camera"] is False


def test_driver_stack_config_can_keep_legacy_mujoco_driver_camera() -> None:
    config = driver_stack_config(
        {"use_driver_camera": True},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=True,
    )

    assert config["publish_camera"] is True


def test_driver_stack_config_disables_mujoco_driver_lidar_when_role_exists() -> None:
    config = driver_stack_config(
        {"enable_lidar": True},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=False,
    )

    assert config["publish_lidar"] is False


def test_driver_stack_config_disables_mujoco_driver_lidar_for_managed_slam() -> None:
    config = driver_stack_config(
        {},
        slam_profile="fastlio2",
        driver_module="MujocoDriverModule",
        enable_semantic=False,
    )

    assert config["publish_lidar"] is False


def test_driver_stack_config_can_keep_legacy_mujoco_driver_lidar() -> None:
    config = driver_stack_config(
        {"enable_lidar": True, "use_driver_lidar": True},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=False,
    )

    assert config["publish_lidar"] is True


def test_driver_stack_config_disables_mujoco_driver_imu_when_role_exists() -> None:
    config = driver_stack_config(
        {"enable_imu": True},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=False,
    )

    assert config["publish_imu"] is False


def test_driver_stack_config_can_keep_legacy_mujoco_driver_imu() -> None:
    config = driver_stack_config(
        {"enable_imu": True, "use_driver_imu": True},
        slam_profile="none",
        driver_module="MujocoDriverModule",
        enable_semantic=False,
    )

    assert config["publish_imu"] is True


def test_perception_stack_config_records_driver_module() -> None:
    config = perception_stack_config({"detector_debug": True}, driver_module="ThunderDriver")

    assert config["detector_debug"] is True
    assert config["_driver_cls_name"] == "ThunderDriver"


def test_perception_stack_config_uses_independent_mujoco_camera_role() -> None:
    config = perception_stack_config({}, driver_module="MujocoDriverModule")

    assert config["_driver_cls_name"] == "MujocoDriverModule"
    assert config["enable_camera"] is True
    assert config["camera_backend"] == "sim"
    assert config["use_driver_camera"] is False


def test_perception_stack_config_keeps_external_driver_config_minimal() -> None:
    config = perception_stack_config({}, driver_module="ExternalDriverModule")

    assert config == {"_driver_cls_name": "ExternalDriverModule"}


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
        "owner": "module",
        "tare_scenario": "tunnel",
        "auto_start": False,
        "way_point_topic": "/waypoint",
    }


def test_native_tare_endpoint_is_the_single_exploration_owner() -> None:
    config = {
        "exploration_backend": "tare",
        "native_navigation_endpoint": "lingtu-nav-dds",
    }

    assert exploration_owner(config) == "native"
    assert exploration_stack_config(config)["owner"] == "native"


def test_needs_lidar_for_slam_only_for_managed_lidar_slam() -> None:
    assert needs_lidar_for_slam("fastlio2") is True
    assert needs_lidar_for_slam("localizer") is True
    assert needs_lidar_for_slam("genz") is True
    assert needs_lidar_for_slam("bridge") is False
    assert needs_lidar_for_slam("none") is False
