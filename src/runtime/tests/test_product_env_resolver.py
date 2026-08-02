from __future__ import annotations

from dataclasses import fields
from pathlib import Path

import pytest

from lingtu.assembly.products import (
    resolve_product_host_config,
    resolve_product_host_runtime,
)
from lingtu.assembly.products.configuration import (
    EnvConfig,
    ResolvedProductHostConfig,
    resolve_env_spec,
)
from runtime.config import RobotConfig

ROOT = Path(__file__).resolve().parents[3]


def test_real_product_resolution_has_only_product_env_runtime_identity() -> None:
    resolved = resolve_product_host_runtime("nav", "real")

    assert tuple(field.name for field in fields(ResolvedProductHostConfig)) == (
        "product",
        "env",
        "config",
        "product_variant",
    )
    assert resolved.product == "nav"
    assert resolved.env == "real"
    assert resolved.product_variant is None
    assert resolved.config["_env"] == "real"
    assert "_env_backend" not in resolved.config
    assert "_profile_adapter" not in resolved.config
    assert "profile_adapter" not in resolved.config
    assert "_driver_backend" not in resolved.config
    assert not hasattr(resolved, "profile_adapter")
    assert not hasattr(resolved, "driver_backend")


def test_real_env_references_canonical_typed_robot_config() -> None:
    env = resolve_env_spec("real")

    assert env.name == "real"
    assert env.robot_config_ref == (ROOT / "config" / "robot_config.yaml").resolve()
    assert isinstance(env.robot_config, RobotConfig)
    assert env.robot_config.driver.dog_host == "127.0.0.1"


def test_real_product_uses_typed_robot_config_driver_values() -> None:
    env = resolve_env_spec("real")
    resolved = resolve_product_host_runtime("nav", "real")

    assert env.robot_config is not None
    assert resolved.config["dog_host"] == env.robot_config.driver.dog_host
    assert resolved.config["dog_port"] == env.robot_config.driver.dog_port
    assert resolved.config["_endpoint_transport"] == "dds"
    assert resolved.config["_endpoint_contract"] == "thunder_dds_v1"


def test_sim_product_requires_an_explicit_supported_backend() -> None:
    with pytest.raises(ValueError, match=r"env_config\.backend is required"):
        resolve_product_host_runtime("map", "sim")

    with pytest.raises(ValueError, match="unsupported sim backend"):
        resolve_product_host_runtime(
            "map",
            "sim",
            env_config={"backend": "mujoco_native_dds"},
        )


def test_sim_backend_is_internal_configuration_not_a_third_identity() -> None:
    resolved = resolve_product_host_runtime(
        "map",
        "sim",
        env_config=EnvConfig(backend="mujoco_host"),
    )

    assert resolved.product == "map"
    assert resolved.env == "sim"
    assert resolved.config["_env"] == "sim"
    assert resolved.config["_env_backend"] == "mujoco_host"
    assert "_profile_adapter" not in resolved.config


def test_sim_backend_rejects_retired_cmu_unity_backend() -> None:
    with pytest.raises(ValueError, match="unsupported sim backend 'cmu_unity'"):
        resolve_product_host_runtime(
            "explore",
            "sim",
            product_variant="map",
            env_config={"backend": "cmu_unity"},
        )


def test_gazebo_supports_live_explore_but_rejects_saved_map_variant() -> None:
    live = resolve_product_host_runtime(
        "explore",
        "sim",
        product_variant="live",
        env_config={"backend": "gazebo"},
    )

    assert live.product == "explore"
    assert live.product_variant == "live"

    with pytest.raises(
        ValueError,
        match=r"does not support Product 'explore' variant 'map'",
    ):
        resolve_product_host_runtime(
            "explore",
            "sim",
            product_variant="map",
            env_config={"backend": "gazebo"},
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
        {"profile_adapter": "thunder_dds"},
        {"_profile_adapter": "thunder_dds"},
        {"env": "sim"},
        {"_env": "sim"},
        {"_env_backend": "gazebo"},
    ],
)
def test_product_overrides_cannot_change_env_or_hardware_identity(
    reserved: dict[str, str],
) -> None:
    with pytest.raises(TypeError, match="reserved Product resolution key"):
        resolve_product_host_config("map", "real", overrides=reserved)


def test_env_config_rejects_unknown_fields() -> None:
    with pytest.raises(TypeError, match="unsupported env_config field"):
        resolve_product_host_runtime(
            "map",
            "sim",
            env_config={"backend": "mujoco_host", "endpoint": "mujoco_live"},
        )
