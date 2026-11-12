"""Read robot observations through the selected source; never authorize motion."""

import asyncio
import base64
import json
import time

import grpc

from gateway.routes.camera import snapshot_jpeg
from gateway.services.media_status import build_camera_status
from gateway.services.native_status import read_navigation_status
from gateway.services.runtime_status import robot_model
from runtime.msgs.numpy_compat import np
from runtime.utils.binary_codec import decode_pointcloud_frame

SENSORS = ("imu", "joints", "motors", "voltages")


def capabilities(gw):
    source = getattr(gw, "_observations", None)
    description = source.capabilities() if source is not None else {"provider": None, "sensors": {}}
    sensors = {name: {"configured": name in description["sensors"], **description["sensors"].get(name, {})}
               for name in SENSORS}
    joint = getattr(gw, "_latest_joint_state", None)
    if joint is not None:
        sensors["joints"] = {"configured": True, "provider": "joint_state", "names": joint.names,
                             "joint_count": len(joint.names), "clock": "unix", "units": "rad, rad/s, N m"}
    return {"success": True, "robot": robot_model(gw), "provider": description["provider"], "sensors": sensors,
            "camera": build_camera_status(gw),
            "lidar": {"configured": getattr(gw, "_cloud_viewer", None) is not None, "kind": "scan", "units": "m"},
            "navigation": {"configured": read_navigation_status() is not None},
            "note": "Configured means an adapter exists, not proof of a live measurement. Read a sample to verify."}


def sensor_snapshot(gw, sensor):
    joint = getattr(gw, "_latest_joint_state", None)
    if sensor == "joints" and joint is not None:
        age = time.time() - joint.ts
        return {"success": True, "sensor": sensor, "provider": "joint_state", "robot": joint.robot_model,
                "source_stamp_s": joint.ts, "source_clock": "unix", "source_age_s": age, "fresh": -1 <= age <= 2,
                "values": {"joints": [
                    {"id": i, "name": name, "position_rad": joint.position[i],
                     "velocity_rad_s": joint.velocity[i], "torque_nm": joint.effort[i]}
                    for i, name in enumerate(joint.names)]}}
    source = getattr(gw, "_observations", None)
    description = source.capabilities() if source is not None else {"provider": None, "sensors": {}}
    if sensor not in description["sensors"]:
        raise ValueError(f"{sensor}: observation adapter is not configured for this robot")
    values = source.sample(sensor, timeout=2)
    return {"success": True, "sensor": sensor, "provider": description["provider"],
            "received_at_s": time.time(), "source_clock": description["sensors"][sensor].get("clock", "not_reported"),
            "values": values}


def camera_snapshot(gw):
    response = snapshot_jpeg(gw)
    if response.status_code != 200:
        error = json.loads(response.body)
        return {"success": False, "error": json.dumps(error, ensure_ascii=False)}
    return {"success": True, "sensor": "camera", "status": build_camera_status(gw),
            "image_data_url": "data:image/jpeg;base64," + base64.b64encode(response.body).decode("ascii"),
            "received_at_s": time.time(), "note": "Snapshot may be cached; source time is not reported by this JPEG endpoint."}


def lidar_snapshot(gw):
    viewer = getattr(gw, "_cloud_viewer", None)
    if viewer is None:
        raise ValueError("LiDAR viewer source is not configured")
    subscription = viewer.scan_subscribe_sync()
    try:
        buffer = subscription.read(timeout=2)
        if buffer is None:
            raise ValueError("No LiDAR scan arrived within 2 seconds")
        frame = decode_pointcloud_frame(buffer)
    finally:
        subscription.close()
    points = frame.points
    finite = points[np.isfinite(points).all(axis=1)]
    sample = finite if len(finite) <= 4096 else finite[np.linspace(0, len(finite) - 1, 4096, dtype=np.int64)]
    return {"success": True, "sensor": "lidar", "frame_id": frame.frame_id, "epoch": str(frame.epoch),
            "source_stamp_s": frame.stamp_s, "sequence": frame.sequence, "received_at_s": time.time(),
            "point_count": len(points), "preview_sample": len(sample) < len(points),
            "bounds_m": {"min": finite.min(axis=0).tolist(), "max": finite.max(axis=0).tolist()} if len(finite) else None,
            "points": sample.reshape(-1).tolist(),
            "note": "Display scan, possibly sampled. Not the authoritative collision map or proof of physical contact."}


def navigation_snapshot():
    status = read_navigation_status()
    if status is None:
        raise ValueError("Native navigation status is unavailable")
    return {"success": True, "sensor": "navigation", "received_at_s": time.time(), "values": status,
            "note": "Native diagnostic snapshot; use its source timestamps/readiness. This does not authorize movement."}


async def invoke(gw, name, args):
    try:
        if name == "get_robot_capabilities":
            return capabilities(gw)
        action = {"read_robot_sensor": lambda: sensor_snapshot(gw, args["sensor"]),
                  "capture_camera": lambda: camera_snapshot(gw), "inspect_lidar": lambda: lidar_snapshot(gw),
                  "inspect_navigation": navigation_snapshot}[name]
        return await asyncio.to_thread(action)
    except grpc.RpcError as error:
        return {"success": False, "error": error.details(), "code": error.code().name}
    except (ValueError, OSError, StopIteration) as error:
        return {"success": False, "error": str(error)}
