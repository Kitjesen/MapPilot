"""Forward source observation events without resampling or synthesizing frames."""

import grpc

from gateway.services.media_status import build_camera_status
from gateway.services.telemetry_normalizers import build_scene_graph_response

from . import camera, cloud
from .codec import fields, invoke
from .generated import common_pb2 as common_pb
from .generated import observations_pb2 as pb
from .generated import observations_pb2_grpc as rpc
from .source import Source


def imu_sample(event):
    if not event.HasField("gyroscope") or not event.HasField("quaternion"):
        raise ValueError("Brainstem IMU event is missing angular velocity or orientation")
    sample = pb.ImuSample(
        angular_velocity_rad_s=pb.Vector3(x=event.gyroscope.x, y=event.gyroscope.y, z=event.gyroscope.z),
        orientation_world_to_body=pb.Quaternion(
            w=event.quaternion.w, x=event.quaternion.x, y=event.quaternion.y, z=event.quaternion.z,
        ),
    )
    if event.HasField("linear_acceleration"):
        acceleration = event.linear_acceleration
        sample.linear_acceleration_m_s2.CopyFrom(pb.Vector3(x=acceleration.x, y=acceleration.y, z=acceleration.z))
    if event.HasField("timestamp"):
        sample.source_elapsed.CopyFrom(event.timestamp)
    return sample


def joint_sample(event):
    sample = pb.JointSample()
    if event.HasField("timestamp"):
        sample.source_elapsed.CopyFrom(event.timestamp)
    kind = event.WhichOneof("data")
    if kind == "single_joint":
        joint = event.single_joint
        if joint.id >= 16:
            raise ValueError("Brainstem joint ID must be between 0 and 15")
        sample.joints.add(
            id=joint.id, position_rad=joint.position, velocity_rad_s=joint.velocity,
            torque_nm=joint.torque, status_code=joint.status,
        )
    elif kind == "all_joints":
        frame = event.all_joints
        arrays = [getattr(frame, field).values for field in ("position", "velocity", "torque", "status")]
        if any(len(values) != 16 for values in arrays):
            raise ValueError("Brainstem joint frame requires 16 positions, velocities, torques and statuses")
        for joint_id, (position, velocity, torque, status) in enumerate(zip(*arrays)):
            sample.joints.add(
                id=joint_id, position_rad=position, velocity_rad_s=velocity, torque_nm=torque, status_code=status,
            )
    else:
        raise ValueError("Brainstem joint event is missing joint data")
    return sample


class Observations(Source, rpc.ObservationsServicer):
    def Scene(self, request, context):
        with self.gw._state_lock:
            scene = self.gw._sg_json
        return common_pb.Snapshot(fields=fields(build_scene_graph_response(scene)))

    def CameraStatus(self, request, context):
        return common_pb.Snapshot(fields=fields(invoke(context, build_camera_status, self.gw)))

    def CameraFrame(self, request, context):
        return camera.frame(self.gw, context)

    def WatchCamera(self, request, context):
        return camera.watch(self.gw, context, self._watch_slots)

    def WatchPointCloud(self, request, context):
        return cloud.watch(self.gw, request, context, self._watch_slots)

    def WatchImu(self, request, context):
        return self._watch("imu", imu_sample, context)

    def WatchJoints(self, request, context):
        return self._watch("joints", joint_sample, context)

    def Motors(self, request, context):
        event = self._read("motors", context)
        if len(event.motors) != 16 or {motor.id for motor in event.motors} != set(range(16)):
            context.abort(grpc.StatusCode.UNAVAILABLE, "Brainstem motor status requires unique IDs 0 through 15")
        return pb.MotorStates(motors=[pb.MotorState(
            id=motor.id, online=motor.online, status_code=motor.status_code,
            temperature_c=motor.temperature, errors=motor.errors,
        ) for motor in event.motors])

    def MotorVoltages(self, request, context):
        event = self._read("voltages", context)
        if len(event.values) != 16:
            context.abort(grpc.StatusCode.UNAVAILABLE, "Brainstem motor voltage requires 16 values")
        return pb.MotorVoltagesResponse(volts=event.values)
