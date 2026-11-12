"""Forward source observation events without resampling or synthesizing frames."""

import math

import grpc

from gateway.services.media_status import build_camera_status
from gateway.services.telemetry_normalizers import build_scene_graph_response
from runtime.contracts import CAMERA_ROLE

from . import camera, cloud
from .codec import fields, invoke
from .generated import common_pb2 as common_pb
from .generated import observations_pb2 as pb
from .generated import observations_pb2_grpc as rpc
from .source import Source


def imu_sample(event):
    from runtime.adapters.brainstem.telemetry import values

    return pb.ImuSample(**values("imu", event))


def joint_sample(event):
    from runtime.adapters.brainstem.telemetry import values

    return pb.JointSample(**values("joints", event))


class Observations(Source, rpc.ObservationsServicer):
    def CameraCalibration(self, request, context):
        camera.authorize(context)
        source = (getattr(self.gw, "_all_modules", {}) or {}).get(CAMERA_ROLE)
        read = getattr(source, "get_camera_intrinsics", None)
        intrinsics = read() if callable(read) else None
        if intrinsics is None:
            context.abort(grpc.StatusCode.UNAVAILABLE, "Source camera calibration is unavailable")
        values = [intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                  intrinsics.dist_k1, intrinsics.dist_k2, intrinsics.dist_p1,
                  intrinsics.dist_p2, intrinsics.dist_k3]
        if (not all(math.isfinite(value) for value in values)
                or intrinsics.fx <= 0 or intrinsics.fy <= 0
                or not 0 < intrinsics.width < 2**32 or not 0 < intrinsics.height < 2**32):
            context.abort(grpc.StatusCode.UNAVAILABLE, "Source camera calibration is invalid")
        return pb.CameraCalibration(
            width_px=intrinsics.width, height_px=intrinsics.height,
            fx_px=intrinsics.fx, fy_px=intrinsics.fy,
            cx_px=intrinsics.cx, cy_px=intrinsics.cy,
            frame_id=intrinsics.frame_id,
            distortion_model=intrinsics.distortion_model, distortion=values[4:],
        )

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

    def CameraSample(self, request, context):
        return camera.sample(self.gw, context)

    def WatchCameraSamples(self, request, context):
        return camera.watch(self.gw, context, self._watch_slots, samples=True)

    def WatchPointCloud(self, request, context):
        return cloud.watch(self.gw, request, context, self._watch_slots)

    def WatchImu(self, request, context):
        return self._watch("imu", imu_sample, context)

    def WatchJoints(self, request, context):
        return self._watch("joints", joint_sample, context)

    def Motors(self, request, context):
        event = self._read("motors", context)
        from runtime.adapters.brainstem.telemetry import values

        try:
            return pb.MotorStates(**values("motors", event))
        except ValueError as error:
            context.abort(grpc.StatusCode.UNAVAILABLE, str(error))

    def MotorVoltages(self, request, context):
        event = self._read("voltages", context)
        from runtime.adapters.brainstem.telemetry import values

        try:
            return pb.MotorVoltagesResponse(**values("voltages", event))
        except ValueError as error:
            context.abort(grpc.StatusCode.UNAVAILABLE, str(error))
