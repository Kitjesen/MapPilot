from __future__ import annotations

from lingtu.assembly.wires.context import WiringContext
from lingtu.assembly.wires.slam import map_specs


def test_slam_adapter_does_not_fan_out_to_retired_python_map_layers() -> None:
    context = WiringContext(
        names=frozenset(
            {
                "SlamAdapterModule",
                "maps.service",
                "GatewayModule",
                "ReconstructionModule",
            }
        ),
        driver_module="driver",
        slam_module="SlamAdapterModule",
        camera_src="camera",
        color_out="color_image",
        nav_odom_src="SlamAdapterModule",
    )

    specs = map_specs(context)
    observation_targets = {
        spec.in_module
        for spec in specs
        if spec.out_module == "SlamAdapterModule"
        and spec.out_port == "map_observation"
    }
    frame_targets = {
        spec.in_module
        for spec in specs
        if spec.out_module == "SlamAdapterModule"
        and spec.out_port == "map_cloud_frame"
    }

    assert observation_targets == {"ReconstructionModule"}
    assert frame_targets == set()
    assert not any(spec.out_port == "map_cloud" or spec.in_port == "map_cloud" for spec in specs)


def test_only_mujoco_driver_publishes_reconstruction_map_observations() -> None:
    def specs_for(driver: str):
        return map_specs(
            WiringContext(
                names=frozenset({driver, "ReconstructionModule", "GatewayModule"}),
                driver_module=driver,
                slam_module="",
                camera_src=driver,
                color_out="camera_image",
                nav_odom_src=driver,
            )
        )

    mujoco = specs_for("MujocoDriverModule")
    endpoint = specs_for("SimEndpointDriverModule")

    assert any(spec.out_port == "map_observation" for spec in mujoco)
    assert not any(spec.out_port == "map_observation" for spec in endpoint)
