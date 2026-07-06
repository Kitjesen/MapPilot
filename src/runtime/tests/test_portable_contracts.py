from __future__ import annotations

import ast
from pathlib import Path

from runtime.portable.contracts import (
    CommandSink,
    PortableCommandFrame,
    PortablePlanningFrame,
    PortableSensorFrame,
    SensorSource,
)


FORBIDDEN_IMPORT_ROOTS = {
    "mujoco",
    "rclpy",
    "sensor_msgs",
    "nav_msgs",
    "geometry_msgs",
    "std_msgs",
    "tf2_ros",
    "launch_ros",
    "pcl",
    "pcl_conversions",
    "pcl_ros",
    "fastapi",
    "uvicorn",
    "websockets",
    "torch",
    "ultralytics",
    "openai",
    "anthropic",
    "chromadb",
    "brainstem_api",
    "grpc",
    "open3d",
    "gtsam",
    "rerun",
}


def test_portable_frames_default_construct_without_adapters() -> None:
    sensor = PortableSensorFrame(source="unit")
    command = PortableCommandFrame(sink="unit")
    planning = PortablePlanningFrame(source="unit")

    assert sensor.source == "unit"
    assert sensor.has_point_cloud is False
    assert sensor.has_camera_bundle is False
    assert command.sink == "unit"
    assert command.requests_stop is False
    assert command.safety_owner == "cmd_vel_mux"
    assert planning.has_plan is False


def test_portable_contract_protocols_are_structural() -> None:
    assert SensorSource.__name__ == "SensorSource"
    assert CommandSink.__name__ == "CommandSink"



def test_portable_contracts_do_not_import_adapter_or_heavy_modules() -> None:
    path = Path("src/runtime/portable/contracts.py")
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imports: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name.split(".", 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module.split(".", 1)[0])

    assert imports.isdisjoint(FORBIDDEN_IMPORT_ROOTS)
