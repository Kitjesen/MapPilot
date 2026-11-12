import asyncio
import base64
import time
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.responses import Response
from fastapi.testclient import TestClient

from gateway.routes.observations import register_observation_routes
from gateway.services import robot_tools
from gateway.services.conversation_tools import workbench_tools
from runtime.msgs.sensor import JointState
from runtime.utils.binary_codec import encode_pointcloud
from runtime.utils.python_analysis import run


class Source:
    def capabilities(self):
        return {"provider": "test-12", "sensors": {"joints": {"joint_count": 12}, "imu": {"clock": "source_elapsed"}}}

    def sample(self, sensor, timeout):
        assert timeout == 2
        return {"joints": [{"id": i, "position_rad": i / 10} for i in range(12)]} if sensor == "joints" else {"angular_velocity_rad_s": {"z": .3}}


def test_discovery_does_not_probe_and_does_not_invent_a_layout(monkeypatch):
    monkeypatch.setattr(robot_tools, "read_navigation_status", lambda: None)
    result = robot_tools.capabilities(SimpleNamespace(_observations=Source(), _all_modules={}))
    assert result["provider"] == "test-12"
    assert result["sensors"]["joints"]["joint_count"] == 12
    assert not result["sensors"]["motors"]["configured"]
    assert not result["camera"]["available"]
    assert not result["navigation"]["configured"]


@pytest.mark.asyncio
async def test_tools_use_selected_source_and_analysis_uses_its_actual_values():
    tools, invoke = workbench_tools(SimpleNamespace(_observations=Source()))
    assert next(item for item in tools if item["function"]["name"] == "run_python")["writes"]
    joints = await invoke("read_robot_sensor", {"sensor": "joints"})
    assert len(joints["values"]["joints"]) == 12
    result = await invoke("run_python", {"code": "print(sum(j['position_rad'] for j in data['joints']['values']['joints']))"})
    assert result["success"] and float(result["stdout"].strip()) == pytest.approx(6.6)
    _, new_turn = workbench_tools(SimpleNamespace(_observations=Source()))
    result = await new_turn("run_python", {"code": "print(data)"})
    assert result["stdout"].strip() == "{}"


@pytest.mark.asyncio
async def test_missing_sensor_is_not_zero_and_typed_joint_cache_keeps_names():
    result = await robot_tools.invoke(SimpleNamespace(), "read_robot_sensor", {"sensor": "imu"})
    assert not result["success"] and "not configured" in result["error"]
    joint = JointState(time.time(), "go2", [f"joint-{i}" for i in range(12)], [1.] * 12, [2.] * 12, [3.] * 12)
    sample = robot_tools.sensor_snapshot(SimpleNamespace(_latest_joint_state=joint), "joints")
    assert sample["fresh"] and sample["robot"] == "go2"
    assert sample["values"]["joints"][11] == {"id": 11, "name": "joint-11", "position_rad": 1., "velocity_rad_s": 2., "torque_nm": 3.}
    joint.ts -= 5
    assert not robot_tools.sensor_snapshot(SimpleNamespace(_latest_joint_state=joint), "joints")["fresh"]


@pytest.mark.asyncio
async def test_lidar_preserves_frame_and_time_samples_and_closes_subscription():
    points = np.arange(15000, dtype=np.float32).reshape(-1, 3) / 100
    buffer = encode_pointcloud(points, frame_id="lidar", epoch=7, stamp_s=123.5, sequence=9, stream_kind="scan")
    closed = []
    subscription = SimpleNamespace(read=lambda timeout: buffer, close=lambda: closed.append(True))
    gw = SimpleNamespace(_cloud_viewer=SimpleNamespace(scan_subscribe_sync=lambda: subscription))
    result = await robot_tools.invoke(gw, "inspect_lidar", {})
    assert result["success"] and result["frame_id"] == "lidar" and result["source_stamp_s"] == 123.5
    assert result["point_count"] == 5000 and len(result["points"]) == 4096 * 3 and result["preview_sample"]
    assert closed == [True] and "collision" in result["note"]
    subscription.read = lambda timeout: None
    result = await robot_tools.invoke(gw, "inspect_lidar", {})
    assert not result["success"] and closed == [True, True]


def test_camera_keeps_pixels_and_does_not_invent_capture_time(monkeypatch):
    monkeypatch.setattr(robot_tools, "snapshot_jpeg", lambda gw: Response(content=b"jpeg", media_type="image/jpeg"))
    result = robot_tools.camera_snapshot(SimpleNamespace(_all_modules={}))
    assert base64.b64decode(result["image_data_url"].split(",")[1]) == b"jpeg"
    assert "source_stamp_s" not in result and "cached" in result["note"]


def test_http_discovery_and_sensor_use_the_same_owner(monkeypatch):
    monkeypatch.setattr(robot_tools, "read_navigation_status", lambda: None)
    app = FastAPI()
    register_observation_routes(app, SimpleNamespace(_observations=Source(), _all_modules={}))
    with TestClient(app) as client:
        assert client.get("/api/v1/robot/capabilities").json()["provider"] == "test-12"
        assert len(client.get("/api/v1/robot/sensors/joints").json()["values"]["joints"]) == 12
        assert client.get("/api/v1/robot/sensors/motors").status_code == 503
        assert client.get("/api/v1/robot/sensors/unknown").status_code == 404


@pytest.mark.asyncio
async def test_python_errors_output_limit_and_cancel():
    result = await run("raise ValueError('bad measurement')", {})
    assert not result["success"] and "bad measurement" in result["stderr"] and result["exit_code"] != 0
    result = await run("print('x' * 70000)", {})
    assert result["success"] and result["output_truncated"] and len(result["stdout"]) == 65536
    task = asyncio.create_task(run("import time; time.sleep(30)", {}))
    await asyncio.sleep(.2)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task


def test_imu_snapshot_uses_real_source_stream_and_closes_it():
    import threading
    from concurrent.futures import ThreadPoolExecutor

    import grpc
    from google.protobuf.empty_pb2 import Empty

    from runtime.adapters.brainstem.generated.brainstem_api.cms_pb2 import Imu
    from runtime.adapters.brainstem.source import Observations

    cancelled = threading.Event()
    def stream(request, context):
        context.add_callback(cancelled.set)
        event = Imu()
        event.gyroscope.z = .42
        event.quaternion.w = 1
        event.timestamp.seconds = 7
        event.timestamp.nanos = 123
        yield event
        cancelled.wait(3)
    server = grpc.server(ThreadPoolExecutor(max_workers=1))
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler("brainstem.api.v1.RobotControl", {
        "ListenImu": grpc.unary_stream_rpc_method_handler(stream, request_deserializer=Empty.FromString,
                                                         response_serializer=lambda event: event.SerializeToString()),
    }),))
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        source = Observations({"LINGTU_DRIVER_TARGET": f"127.0.0.1:{port}"})
        result = source.sample("imu")
        assert result["angular_velocity_rad_s"]["z"] == pytest.approx(.42)
        assert result["source_elapsed"] == {"seconds": 7, "nanos": 123}
        assert "linear_acceleration_m_s2" not in result
        assert cancelled.wait(2)
    finally:
        server.stop(0).wait(3)


@pytest.mark.asyncio
async def test_python_deadline_stops_the_analysis():
    result = await run("import time; time.sleep(30)", {})
    assert not result["success"] and result["error"] == "Analysis exceeded 10 seconds"


@pytest.mark.asyncio
async def test_monitor_tools_request_ui_without_touching_sources():
    tools, invoke = workbench_tools(SimpleNamespace())
    definition = next(item for item in tools if item["function"]["name"] == "open_sensor_monitor")
    assert not definition["writes"]
    for sensor in definition["function"]["parameters"]["properties"]["sensor"]["enum"]:
        assert await invoke("open_sensor_monitor", {"sensor": sensor}) == {"success": True, "monitor": sensor}
    assert not (await invoke("open_sensor_monitor", {"sensor": "missing"}))["success"]
    assert await invoke("open_imu_monitor", {}) == {"success": True, "monitor": "imu"}
