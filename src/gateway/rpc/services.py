"""Typed RPC handlers call the same operations as the Web interface."""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from urllib.parse import quote
from uuid import UUID

import grpc

from gateway.maps.routes import get_operation, list_maps, save_map
from gateway.navigation.actions import cancel, go
from gateway.navigation.status import build_navigation_status
from gateway.navigation.tasks import query_navigation_task_status
from gateway.routes.health import build_health
from gateway.routes.operations import relocalize
from gateway.routes.product_control import _forward
from gateway.schemas import (
    AgentTaskCancelRequest,
    CancelRequest,
    GoalRequest,
    InstructionRequest,
    LocalizationRelocalizationRequest,
    MapSaveRequest,
    StopRequest,
    SwitchProfileRequest,
)
from gateway.services.app_bootstrap import build_app_capabilities
from gateway.services.body import change as change_body
from gateway.services.instructions import cancel_run, get_run, send
from gateway.services.native_status import read_json_snapshot
from gateway.services.readiness import build_readiness_snapshot
from gateway.services.resume import resume
from gateway.services.runtime_status import build_localization_status, robot_model, runtime_identity
from gateway.services.state_snapshot import build_state_snapshot
from gateway.services.stop import clear_emergency_stop, stop, stop_motion
from gateway.services.telemetry_normalizers import build_path_response

from . import (
    control,
    external,
    faults,
    inspection,
    kinematics,
    location,
    navigation_control,
    system_state,
    system_view,
    task_run,
    telemetry,
    teleop,
)
from .codec import _abort, command, fields, invoke, map_operation, model, pose, product_operation, receipt, task
from .generated import common_pb2 as common_pb
from .generated import localization_pb2_grpc as localization_rpc
from .generated import maps_pb2 as maps_pb
from .generated import maps_pb2_grpc as maps_rpc
from .generated import motion_pb2 as motion_pb
from .generated import motion_pb2_grpc as motion_rpc
from .generated import navigation_pb2 as navigation_pb
from .generated import navigation_pb2_grpc as navigation_rpc
from .generated import system_pb2 as system_pb
from .generated import system_pb2_grpc as system_rpc
from .generated import tasks_pb2 as tasks_pb
from .generated import tasks_pb2_grpc as tasks_rpc
from .map_content import point_cloud
from .map_grid import grid
from .motion_state import body_state, control_status
from .observations import Observations
from .source import Source

PROTOCOL_VERSION = 2
# Bind identity to the loaded release before the installer changes current.
_RELEASE_METADATA = Path(__file__).resolve().parents[3] / "metadata.json"


class Service:
    def __init__(self, gateway):
        self.gw = gateway


class System(Source, system_rpc.SystemServicer):
    def Handshake(self, request, context):
        if request.protocol_version != PROTOCOL_VERSION:
            _abort(
                context,
                grpc.StatusCode.FAILED_PRECONDITION,
                {
                    "reason_code": "protocol_mismatch",
                    "requested_protocol": request.protocol_version,
                    "supported_protocol": PROTOCOL_VERSION,
                    "client_id": request.client_id,
                    "sdk_version": request.sdk_version,
                },
                "unsupported DOSO protocol version",
            )
        if not request.client_id.strip():
            _abort(
                context,
                grpc.StatusCode.INVALID_ARGUMENT,
                {"reason_code": "invalid_client_id"},
                "client_id is required",
            )
        info = system_pb.SystemInfo()
        robot_id = os.environ.get("DOSO_DEVICE_ID")
        if robot_id:
            try:
                identity = UUID(robot_id)
            except ValueError:
                identity = None
            if identity is None or identity.version != 4 or str(identity) != robot_id:
                _abort(
                    context,
                    grpc.StatusCode.FAILED_PRECONDITION,
                    {"reason_code": "invalid_robot_id"},
                    "DOSO_DEVICE_ID must be a canonical UUID v4",
                )
            info.robot_id = robot_id
        selected_model = robot_model(self.gw)
        if selected_model:
            info.robot_version = selected_model
        metadata = read_json_snapshot(str(_RELEASE_METADATA)) or {}
        firmware_version = metadata.get("version")
        if isinstance(firmware_version, str) and firmware_version.strip():
            info.firmware_version = firmware_version.strip()
        return system_pb.HandshakeReply(protocol_version=PROTOCOL_VERSION, info=info)

    def Ping(self, request, context):
        return system_pb.Ping(status="ok")

    def State(self, request, context):
        return common_pb.Snapshot(fields=fields(system_view.state(invoke(context, build_state_snapshot, self.gw))))

    def RobotStatus(self, request, context):
        return invoke(context, system_state.robot_status, self.gw)

    def Health(self, request, context):
        return common_pb.Snapshot(fields=fields(system_view.health(invoke(context, build_health, self.gw))))

    def Faults(self, request, context):
        return faults.read()

    def WatchFaults(self, request, context):
        return telemetry.watch(context, self._watch_slots, faults.read, interval=0.02)

    def Readiness(self, request, context):
        result, _ = invoke(context, build_readiness_snapshot, self.gw)
        return common_pb.Snapshot(fields=fields(system_view.readiness(result)))

    def Capabilities(self, request, context):
        return common_pb.Snapshot(fields=fields(system_view.capabilities(invoke(context, build_app_capabilities, self.gw))))

    def Session(self, request, context):
        return common_pb.Snapshot(fields=fields(system_view.session(invoke(context, self.gw._session_snapshot))))

    def ProductStatus(self, request, context):
        return common_pb.Snapshot(fields=fields(system_view.product_status(invoke(context, _forward, "/status"))))


class Motion(Source, motion_rpc.MotionServicer):
    def Controller(self, request, context):
        return external.state(self.gw)

    def ExternalControl(self, requests, context):
        return external.control(self.gw, requests, context, self._watch_slots)

    def Velocity(self, request, context):
        return kinematics.velocity(self.gw, context)

    def WatchVelocity(self, request, context):
        return telemetry.watch(context, self._watch_slots, lambda: self.Velocity(request, context))

    def Odometry(self, request, context):
        return kinematics.odometry(self.gw, context)

    def WatchOdometry(self, request, context):
        return telemetry.watch(context, self._watch_slots, lambda: self.Odometry(request, context), interval=0.1)

    def StandUp(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, change_body, self.gw, body, "stand"), request.request_id)

    def SitDown(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, change_body, self.gw, body, "sit"), request.request_id)

    def EnableMotors(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, change_body, self.gw, body, "enable_motors"), request.request_id)

    def DisableMotors(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, change_body, self.gw, body, "disable_motors"), request.request_id)

    def Profile(self, request, context):
        profile = self._read("profile", context)
        return motion_pb.ProfileInfo(
            current=profile.current, available=profile.available,
            descriptions=profile.descriptions, current_description=profile.current_description,
        )

    def SwitchProfile(self, request, context):
        metadata = command(context, request.command)
        if not request.name.strip():
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "profile name is required")
        body = model(context, SwitchProfileRequest, name=request.name, **metadata)
        return receipt(context, invoke(context, change_body, self.gw, body, "switch_profile"), metadata["request_id"])

    def Teleop(self, requests, context):
        return teleop.teleop(self.gw, requests, context, self._watch_slots)

    def Stop(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, stop, self.gw, body), request.request_id)

    def StopMotion(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, stop_motion, self.gw, body), request.request_id)

    def ClearEmergencyStop(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, clear_emergency_stop, self.gw, body), request.request_id)

    def Status(self, request, context):
        return common_pb.Snapshot(fields=fields(invoke(context, build_navigation_status, self.gw)))

    def BodyState(self, request, context):
        return body_state(self._read("body", context))

    def ControlStatus(self, request, context):
        return control_status(self._read("control", context))

    def Resume(self, request, context):
        body = model(context, StopRequest, **command(context, request))
        return receipt(context, invoke(context, resume, self.gw, body), request.request_id)

    def ControlState(self, request, context):
        return control.state(self.gw)

    def WatchControlState(self, request, context):
        return telemetry.watch(context, self._watch_slots, lambda: self.ControlState(request, context), interval=0.02)

    def WatchBodyState(self, request, context):
        return self._watch("body", body_state, context)

    def WatchState(self, request, context):
        return self._watch_state(request, context, 0.01)

    def WatchStateAt(self, request, context):
        if not 1 <= request.frequency_hz <= 100:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "motion state frequency must be between 1 and 100 Hz")
        return self._watch_state(request, context, 1.0 / request.frequency_hz)

    def _watch_state(self, request, context, interval):
        if not self._watch_slots.acquire(blocking=False):
            context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "motion state watch limit reached")
        try:
            stopped = threading.Event()
            context.add_callback(stopped.set)
            while context.is_active():
                started = time.monotonic()
                yield self.Status(request, context)
                if stopped.wait(max(0.0, interval - (time.monotonic() - started))):
                    return
        finally:
            self._watch_slots.release()


class Localization(Service, localization_rpc.LocalizationServicer):
    def __init__(self, gateway, watch_slots=None):
        super().__init__(gateway)
        self._watch_slots = watch_slots if watch_slots is not None else threading.BoundedSemaphore(8)

    def Status(self, request, context):
        return common_pb.Snapshot(fields=fields(invoke(context, build_localization_status, self.gw)))

    def State(self, request, context):
        return invoke(context, telemetry.localization_state, self.gw)

    def Location(self, request, context):
        return location.read()

    def WatchLocation(self, request, context):
        return telemetry.watch(context, self._watch_slots, location.read, interval=1.0)

    def Pose(self, request, context):
        return telemetry.map_pose(self.gw, context)

    def WatchPose(self, request, context):
        return telemetry.watch(
            context, self._watch_slots,
            lambda: telemetry.map_pose(self.gw, context, require_fresh=True),
        )

    def Relocalize(self, request, context):
        metadata = command(context, request.command)
        values = {
            "map_name": request.map_name,
            "request_id": metadata["request_id"],
            "mode": "seeded" if request.HasField("initial_pose") else "global",
        }
        if request.HasField("initial_pose"):
            values["initial_pose"] = pose(request.initial_pose)
        body = model(context, LocalizationRelocalizationRequest, **values)
        return receipt(context, invoke(context, relocalize, self.gw, body), metadata["request_id"])


class Maps(Service, maps_rpc.MapsServicer):
    def __init__(self, gateway, watch_slots=None):
        super().__init__(gateway)
        self._watch_slots = watch_slots if watch_slots is not None else threading.BoundedSemaphore(8)

    def PointCloud(self, request, context):
        return point_cloud(self.gw, request, context, self._watch_slots)

    def Grid(self, request, context):
        return grid(self.gw, request, context, self._watch_slots)

    def List(self, request, context):
        result = invoke(context, list_maps, self.gw)
        records = []
        for entry in result.get("maps", []):
            item = {"name": entry} if isinstance(entry, str) else entry
            record = maps_pb.MapRecord(
                name=item.get("name", ""),
                has_pcd=item.get("has_pcd") is True,
                has_octomap=item.get("has_octomap") is True,
                activation_blockers=item.get("activation_blockers") or [],
            )
            if isinstance(item.get("can_activate"), bool):
                record.can_activate = item["can_activate"]
            records.append(record)
        return maps_pb.MapList(maps=records, active=result.get("active") or "", details=fields(result))

    def Save(self, request, context):
        metadata = command(context, request.command)
        body = model(
            context,
            MapSaveRequest,
            request_id=metadata["request_id"],
            **({"name": request.name} if request.HasField("name") else {}),
        )
        return receipt(context, invoke(context, save_map, self.gw, body), metadata["request_id"])

    def Operation(self, request, context):
        return map_operation(invoke(context, get_operation, self.gw, request.operation_id))

    def Switch(self, request, context):
        metadata = command(context, request.command)
        products = {maps_pb.PRODUCT_MAP: "map", maps_pb.PRODUCT_NAV: "nav"}
        if request.product not in products:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "product must be map or nav")
        body = {
            "request_id": metadata["request_id"],
            "product": products[request.product],
            "expected_product_session_id": request.expected_product_session_id,
        }
        if request.HasField("map_name"):
            body["map_name"] = request.map_name
        if request.HasField("initial_pose"):
            body["initial_pose"] = pose(request.initial_pose)
        return product_operation(invoke(context, _forward, "/switch", body))

    def ProductOperation(self, request, context):
        return product_operation(invoke(context, _forward, f"/operations/{quote(request.request_id, safe='')}"))


class Navigation(Service, navigation_rpc.NavigationServicer):
    def __init__(self, gateway, watch_slots=None):
        super().__init__(gateway)
        # shortcut: synchronous streams occupy workers; use async execution when scaling beyond this limit.
        self._watch_slots = watch_slots if watch_slots is not None else threading.BoundedSemaphore(8)

    def GoWithOptions(self, request, context):
        if not any(request.HasField(name) for name in ("max_speed_mps", "acceptance_radius_m")):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "at least one goal option is required")
        return self.Go(request, context)

    def Go(self, request, context):
        metadata = command(context, request.command)
        if not request.HasField("pose"):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "pose is required")
        options = {name: getattr(request, name) for name in ("max_speed_mps", "acceptance_radius_m") if request.HasField(name)}
        body = model(context, GoalRequest, **metadata, **pose(request.pose), frame_id="map", **options)
        return receipt(context, invoke(context, go, self.gw, body), metadata["request_id"])

    def PreviewPlan(self, request, context):
        from .navigation import preview

        return preview(self.gw, request, context)

    def SetSpeedLimit(self, request, context):
        return navigation_control.control(self.gw, request, context, "speed_limit")

    def Status(self, request, context):
        return common_pb.Snapshot(fields=fields(invoke(context, build_navigation_status, self.gw)))

    def Path(self, request, context):
        with self.gw._state_lock:
            path, robot = self.gw._last_path, self.gw._odom
        return common_pb.Snapshot(fields=fields(build_path_response(path, robot)))

    def Route(self, request, context):
        return telemetry.navigation_path(self.gw, context)

    def WatchPath(self, request, context):
        return telemetry.watch(context, self._watch_slots, lambda: self.Route(request, context))

    def Task(self, request, context):
        return task(invoke(context, query_navigation_task_status, self.gw, request.task_id))

    def Cancel(self, request, context):
        metadata = command(context, request.command)
        body = model(context, CancelRequest, **metadata, task_id=request.task_id, reason=request.reason)
        return receipt(context, invoke(context, cancel, self.gw, body, request.task_id), metadata["request_id"])

    def Pause(self, request, context):
        return navigation_control.control(self.gw, request, context, "pause")

    def Resume(self, request, context):
        return navigation_control.control(self.gw, request, context, "resume")

    def Watch(self, request, context):
        if not self._watch_slots.acquire(blocking=False):
            context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "navigation watch limit reached")
        try:
            stopped = threading.Event()
            if not context.add_callback(stopped.set):
                return
            next_tick = time.monotonic()
            while context.is_active():
                state = self.Task(request, context)
                yield state
                if state.found and state.state in (
                    navigation_pb.TASK_SUCCESS, navigation_pb.TASK_FAILED, navigation_pb.TASK_CANCELLED,
                ):
                    return
                next_tick += 0.02
                now = time.monotonic()
                if now - next_tick > 0.02:
                    next_tick = now
                if stopped.wait(max(0.0, next_tick - now)):
                    return
        finally:
            self._watch_slots.release()


class Tasks(Service, tasks_rpc.TasksServicer):
    def __init__(self, gateway, watch_slots=None):
        super().__init__(gateway)
        self._watch_slots = watch_slots if watch_slots is not None else threading.BoundedSemaphore(8)

    def Send(self, request, context):
        metadata = command(context, request.command)
        body = model(
            context,
            InstructionRequest,
            **metadata,
            text=request.text,
            control_only=request.control_only,
            **({"run_id": request.run_id} if request.HasField("run_id") else {}),
        )
        return receipt(context, invoke(context, send, self.gw, body), metadata["request_id"])

    def Get(self, request, context):
        if not request.run_id.strip():
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "run_id is required")
        result = invoke(context, get_run, self.gw, request.run_id)
        return tasks_pb.Run(
            run_id=result.get("run_id") or request.run_id, found=result.get("found") is True, details=fields(result)
        )

    def Watch(self, request, context):
        return task_run.watch(context, self._watch_slots, lambda: self.Get(request, context))

    def Cancel(self, request, context):
        metadata = command(context, request.command)
        body = model(context, AgentTaskCancelRequest, **metadata)
        return receipt(context, invoke(context, cancel_run, self.gw, request.run_id, body), metadata["request_id"])

    def InspectionRoutes(self, request, context):
        return inspection.routes(self.gw, request, context)

    def InspectionRoute(self, request, context):
        return inspection.route(self.gw, request, context)

    def PutInspectionRoute(self, request, context):
        return inspection.put_route(self.gw, request, context)

    def DeleteInspectionRoute(self, request, context):
        return inspection.delete_route(self.gw, request, context)

    def StartInspection(self, request, context):
        return inspection.start(self.gw, request, context)

    def InspectionTask(self, request, context):
        return inspection.task(self.gw, request, context)

    def InspectionTasks(self, request, context):
        return inspection.tasks(self.gw, request, context)

    def WatchInspection(self, request, context):
        return task_run.watch(context, self._watch_slots, lambda: self.InspectionTask(request, context),
                              terminal=lambda task: task.terminal)

    def PauseInspection(self, request, context):
        return inspection.control(self.gw, request, context, "pause")

    def ResumeInspection(self, request, context):
        return inspection.control(self.gw, request, context, "resume")

    def CancelInspection(self, request, context):
        return inspection.control(self.gw, request, context, "cancel")

    def InspectionReport(self, request, context):
        return inspection.report(self.gw, request, context)
