from __future__ import annotations

from dataclasses import fields
from pathlib import Path

import pytest
import yaml

import lingtu.assembly.products.configuration as product_configuration
from lingtu.assembly.products import (
    resolve_product_host_config as _resolve_product_host_config,
)
from lingtu.assembly.products import (
    resolve_product_host_runtime as _resolve_product_host_runtime,
)
from lingtu.assembly.products.configuration import (
    EnvConfig,
    ResolvedProductHostConfig,
)
from lingtu.assembly.products.configuration import (
    resolve_env_spec as _resolve_env_spec,
)
from runtime.config import RobotConfig

ROOT = Path(__file__).resolve().parents[3]
REAL_ROBOT = "unitree/go2"
SIM_ROBOT = "doso/thunder_v4"


def _robot_for_env(env: str) -> str:
    return SIM_ROBOT if env == "sim" else REAL_ROBOT


def resolve_env_spec(env: str, **kwargs):
    kwargs.setdefault("robot", _robot_for_env(env))
    return _resolve_env_spec(env, **kwargs)


def resolve_product_host_runtime(product: str, env: str, **kwargs):
    kwargs.setdefault("robot", _robot_for_env(env))
    return _resolve_product_host_runtime(product, env, **kwargs)


def resolve_product_host_config(product: str, env: str, **kwargs):
    kwargs.setdefault("robot", _robot_for_env(env))
    return _resolve_product_host_config(product, env, **kwargs)


def test_robot_is_required_instead_of_inferred_from_env() -> None:
    with pytest.raises(ValueError, match=r"(?i)robot"):
        product_configuration._load_robot_model(None)


@pytest.mark.parametrize("robot", (REAL_ROBOT, SIM_ROBOT))
def test_sim_env_accepts_any_known_robot_before_asset_selection(robot: str) -> None:
    resolved = resolve_env_spec("sim", robot=robot)

    assert resolved.name == "sim"
    assert resolved.robot == robot
    assert resolved.config.backend == "mujoco"


def test_real_env_reports_missing_adjacent_robot_config() -> None:
    with pytest.raises((FileNotFoundError, ValueError), match=r"robot\.yaml|RobotConfig"):
        resolve_env_spec("real", robot=SIM_ROBOT)


def test_mid360_is_required_by_the_product_not_by_real_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    model_dir = ROOT / "config" / "robots" / "unitree" / "go2"
    broken_mid360 = {
        "sensors": {
            "mid360": {
                "fastlio2_config": "sensors/missing.yaml",
                "extrinsic_profile": "go2_mid360",
            }
        }
    }
    monkeypatch.setattr(
        product_configuration,
        "_load_robot_model",
        lambda robot: (REAL_ROBOT, model_dir, broken_mid360),
    )

    teleop = resolve_product_host_runtime("teleop", "real", robot=REAL_ROBOT)

    assert teleop.env_spec.slam_config_ref is None
    with pytest.raises(FileNotFoundError, match="MID-360 Fast-LIO2 config not found"):
        resolve_product_host_runtime("teleop_avoid", "real", robot=REAL_ROBOT)


def test_real_product_resolution_includes_robot_identity() -> None:
    resolved = resolve_product_host_runtime("nav", "real")

    assert tuple(field.name for field in fields(ResolvedProductHostConfig)) == (
        "robot",
        "product",
        "env",
        "config",
        "product_spec",
        "env_spec",
        "product_variant",
    )
    assert resolved.robot == "unitree/go2"
    assert resolved.product == "nav"
    assert resolved.env == "real"
    assert resolved.product_variant is None
    assert resolved.config["_env"] == "real"
    assert resolved.product_spec["slam_mode"] == "localization"
    assert resolved.env_spec.name == "real"
    assert "_env_backend" not in resolved.config
    assert "_driver_backend" not in resolved.config
    assert not hasattr(resolved, "driver_backend")


def test_real_env_uses_selected_robot_typed_config() -> None:
    env = resolve_env_spec("real")

    assert env.name == "real"
    assert env.robot_config_ref == (
        ROOT / "config" / "robots" / "unitree" / "go2" / "robot.yaml"
    ).resolve()
    assert isinstance(env.robot_config, RobotConfig)
    assert env.robot_config.driver.backend == "go2"
    assert env.robot_config.driver.network_interface
    assert env.robot_config.gnss.enabled is False
    assert env.robot_config.gnss.fusion.enabled is False


def test_go2_real_env_uses_go2_collision_geometry() -> None:
    env = resolve_env_spec("real", robot="unitree/go2")

    assert env.robot_config is not None
    geometry = env.robot_config.geometry
    assert geometry.vehicle_length == pytest.approx(0.76)
    assert geometry.vehicle_width == pytest.approx(0.31)
    assert geometry.collision_cylinder_radius == pytest.approx(0.25)
    assert geometry.collision_cylinder_offset == pytest.approx(0.18)
    assert geometry.collision_hard_margin == pytest.approx(0.10)


def test_real_product_keeps_native_driver_values_out_of_host_config() -> None:
    env = resolve_env_spec("real")
    resolved = resolve_product_host_runtime("nav", "real")

    assert env.robot_config is not None
    assert resolved.config["driver_backend"] == env.robot_config.driver.backend
    assert "driver_network_interface" not in resolved.config
    assert "driver_network_address" not in resolved.config
    assert "driver_probe_ip" not in resolved.config
    assert "_robot_config_ref" not in resolved.config
    assert "_slam_config_ref" not in resolved.config
    assert "dog_host" not in resolved.config
    assert "dog_port" not in resolved.config
    assert resolved.config["_endpoint_transport"] == "dds"
    assert resolved.config["_endpoint_contract"] == "field_dds_v1"


def test_sim_product_uses_the_robot_backend() -> None:
    resolved = resolve_product_host_runtime("map", "sim")

    assert resolved.robot == "doso/thunder_v4"
    assert resolved.config["_env_backend"] == "mujoco"

    with pytest.raises(ValueError, match="unsupported sim backend"):
        resolve_product_host_runtime(
            "map",
            "sim",
            env_config={"backend": "mujoco_native_dds"},
        )


@pytest.mark.parametrize("profile", ("fastlio2", "localizer", "pointlio", "genz"))
def test_product_resolution_rejects_slam_identity_overrides(profile: str) -> None:
    with pytest.raises(TypeError, match="reserved Product resolution key"):
        resolve_product_host_config("map", "sim", slam_profile=profile)


@pytest.mark.parametrize(
    ("product", "variant", "expected_mode", "expected_profile"),
    (
        ("map", None, "mapping", "native_dds"),
        ("nav", None, "localization", "native_dds"),
        ("explore", "live", "mapping", "native_dds"),
        ("explore", "map", "localization", "native_dds"),
        ("teleop", None, "none", "none"),
    ),
)
def test_product_slam_mode_and_runtime_identity_are_independent(
    product: str,
    variant: str | None,
    expected_mode: str,
    expected_profile: str,
) -> None:
    config = resolve_product_host_config(
        product,
        "sim",
        product_variant=variant,
    )

    assert config["slam_mode"] == expected_mode
    assert config["slam_profile"] == expected_profile


@pytest.mark.parametrize("backend", ["mujoco_native", "mujoco_host"])
def test_sim_product_rejects_retired_mujoco_backends(backend: str) -> None:
    with pytest.raises(ValueError, match="unsupported sim backend"):
        resolve_product_host_runtime(
            "map",
            "sim",
            env_config={"backend": backend},
        )


def test_sim_backend_is_internal_configuration_not_a_third_identity() -> None:
    resolved = resolve_product_host_runtime(
        "teleop_avoid",
        "sim",
        env_config=EnvConfig(backend="mujoco"),
    )

    assert resolved.product == "teleop_avoid"
    assert resolved.env == "sim"
    assert resolved.config["_env"] == "sim"
    assert resolved.config["_env_backend"] == "mujoco"
    assert resolved.config["_endpoint_transport"] == "dds"
    assert resolved.config["_endpoint_contract"] == "field_dds_v1"


@pytest.mark.parametrize(
    ("profile", "error"),
    (
        ("missing_mid360", "unknown LiDAR extrinsic profile"),
        ("thunder_v4_mid360", "extrinsic profile differs from RobotConfig"),
    ),
)
def test_lidar_product_rejects_invalid_mid360_profile(
    monkeypatch: pytest.MonkeyPatch,
    profile: str,
    error: str,
) -> None:
    model = yaml.safe_load((ROOT / "config" / "robots" / "unitree" / "go2" / "model.yaml").read_text(encoding="utf-8"))
    model["sensors"]["mid360"]["extrinsic_profile"] = profile
    monkeypatch.setattr(
        product_configuration,
        "_load_robot_model",
        lambda robot: (
            "unitree/go2",
            ROOT / "config" / "robots" / "unitree" / "go2",
            model,
        ),
    )

    with pytest.raises(ValueError, match=error):
        resolve_product_host_runtime("teleop_avoid", "real", robot=REAL_ROBOT)


def test_real_product_uses_typed_robot_geometry_values() -> None:
    env = resolve_env_spec("real")
    resolved = resolve_product_host_runtime("teleop_avoid", "real")

    assert env.robot_config is not None
    assert env.robot_config.gnss.enabled is False
    assert env.robot_config.gnss.fusion.enabled is False
    assert resolved.config["vehicle_length_m"] == env.robot_config.geometry.vehicle_length
    assert resolved.config["vehicle_width_m"] == env.robot_config.geometry.vehicle_width
    assert resolved.config["sensor_offset_x_m"] == env.robot_config.lidar.offset_x
    assert resolved.config["sensor_offset_y_m"] == env.robot_config.lidar.offset_y
    assert resolved.config["sensor_offset_z_m"] == env.robot_config.lidar.offset_z


def test_sim_product_does_not_import_real_robot_geometry() -> None:
    resolved = resolve_product_host_runtime(
        "teleop_avoid",
        "sim",
        env_config=EnvConfig(backend="mujoco"),
    )

    assert "vehicle_length_m" not in resolved.config
    assert "sensor_offset_x_m" not in resolved.config


def test_sim_backend_rejects_retired_cmu_unity_backend() -> None:
    with pytest.raises(ValueError, match="unsupported sim backend 'cmu_unity'"):
        resolve_product_host_runtime(
            "explore",
            "sim",
            product_variant="map",
            env_config={"backend": "cmu_unity"},
        )


@pytest.mark.parametrize("env", ["legacy_real_target", "mujoco_host", "hardware"])
def test_product_resolution_rejects_non_env_identity(env: str) -> None:
    with pytest.raises(ValueError, match="unknown Env"):
        resolve_product_host_runtime("map", env)


@pytest.mark.parametrize(
    "reserved",
    [
        {"driver_backend": "thunder"},
        {"robot": "stub"},
        {"env": "sim"},
        {"_env": "sim"},
        {"_env_backend": "gazebo"},
        {"vehicle_width_m": 0.1},
    ],
)
def test_product_overrides_cannot_change_env_or_hardware_identity(
    reserved: dict[str, object],
) -> None:
    with pytest.raises(TypeError, match="reserved Product resolution key"):
        resolve_product_host_config("map", "real", overrides=reserved)


def test_env_config_rejects_unknown_fields() -> None:
    with pytest.raises(TypeError, match="unsupported env_config field"):
        resolve_product_host_runtime(
            "map",
            "sim",
            env_config={"backend": "mujoco", "endpoint": "mujoco_live"},
        )


def test_local_planner_selection_keeps_nav_as_the_product() -> None:
    resolved = resolve_product_host_runtime(
        "nav",
        "sim",
        local_planner="scan",
        env_config={"backend": "mujoco"},
    )

    assert resolved.product == "nav"
    assert resolved.product_variant is None
    assert resolved.product_spec["native_nav"]["local_planner"] == "scan"


def test_real_env_rejects_scan_until_field_qualification() -> None:
    with pytest.raises(ValueError, match="has not qualified local planner 'scan'"):
        resolve_product_host_runtime(
            "nav",
            "real",
            local_planner="scan",
        )


def test_local_planner_selection_rejects_unknown_algorithm() -> None:
    with pytest.raises(ValueError, match="local_planner must be cmu or scan"):
        resolve_product_host_runtime(
            "nav",
            "real",
            local_planner="other",
        )


def test_local_planner_selection_requires_a_product_declaration() -> None:
    with pytest.raises(ValueError, match="does not declare selectable local planners"):
        resolve_product_host_runtime(
            "teleop",
            "real",
            local_planner="scan",
        )
