from __future__ import annotations

from lingtu.assembly.wires.context import WiringContext
from lingtu.assembly.wires.slam import map_cloud_specs


def test_slam_adapter_does_not_double_feed_map_observation_consumers() -> None:
    map_layers = {
        "OccupancyGridModule",
        "ElevationMapModule",
        "VoxelGridModule",
    }
    context = WiringContext(
        names=frozenset(
            {
                "SlamAdapterModule",
                "maps.service",
                "nav.terrain",
                "RerunBridgeModule",
                "GatewayModule",
                *map_layers,
            }
        ),
        robot="thunder",
        driver_module="driver",
        slam_profile="bridge",
        slam_module="SlamAdapterModule",
        scene_xml="",
        enable_semantic=False,
        camera_src="camera",
        color_out="color_image",
        nav_odom_src="SlamAdapterModule",
    )

    specs = map_cloud_specs(context)
    observation_targets = {
        spec.in_module
        for spec in specs
        if spec.out_module == "SlamAdapterModule"
        and spec.out_port == "map_observation"
    }
    raw_cloud_targets = {
        spec.in_module
        for spec in specs
        if spec.out_module == "SlamAdapterModule"
        and spec.out_port == "map_cloud"
    }

    assert map_layers <= observation_targets
    assert map_layers.isdisjoint(raw_cloud_targets)
    assert {"nav.terrain", "RerunBridgeModule", "GatewayModule"} <= raw_cloud_targets
