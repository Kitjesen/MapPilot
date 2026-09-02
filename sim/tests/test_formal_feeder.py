from __future__ import annotations

import inspect
import json
import os
import signal
import threading
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterator

import pytest

from drivers.real.camera.shm import ShmFrameReader, StreamKind
from lingtu.run_plan import RunPlan
from lingtu.sim.stop import (
    MOTION_STOP_SCHEMA,
    load_motion_stop_evidence,
    process_launch_id,
    publish_motion_stop_evidence,
)
from runtime.graph import (
    ProcessArtifact,
    ProcessCommand,
    ProcessReadiness,
    ProcessSpec,
)
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import POINT_DTYPE
from sim.scripts.mujoco import formal_feeder as feeder
from sim.scripts.mujoco.driver_bridge_session import (
    DriverBridgeCommand,
    DriverBridgeStoppedEvidence,
)
from sim.scripts.mujoco.evidence import (
    FEEDER_STATUS_FILENAME,
    FEEDER_STATUS_SCHEMA,
    MAX_TRAJECTORY_SAMPLES,
    MOTION_EVIDENCE_FILENAME,
    MOTION_EVIDENCE_SCHEMA,
    load_motion_evidence,
    publish_motion_evidence,
)
from sim.scripts.mujoco.native_sensor_records import (
    HEADER as SENSOR_RECORD_HEADER,
)
from sim.scripts.mujoco.native_sensor_records import (
    RECORD_CAMERA,
    RECORD_CLOUD,
    RECORD_IMU,
    RECORD_ODOM_PRIOR,
    RECORD_REGISTERED_CLOUD,
)

PRODUCT_SESSION_ID = "1" * 32
SESSION_ID = "test-session"
BRIDGE_BOOT_ID = "c" * 32
CONTROLLER_BOOT_ID = "d" * 32


def _minimal_simulation(
    *,
    world_mjcf: str = "sim/packages/worlds/runplan/world.xml",
    imu_hz: float = 200.0,
    mid360_hz: float = 10.0,
    include_sensors: bool = True,
    include_camera: bool = False,
    camera_hz: float = 30.0,
    depth_camera_hz: float | None = None,
    initial_keyframe: str | None = None,
    navigation_fixture_raw_overlay: Any = False,
) -> dict[str, Any]:
    global_policy = {
        "owner": "world",
        "timestep_s": 0.005,
        "integrator": "rk4",
        "solver": "newton",
        "iterations": 100,
        "gravity_mps2": [0.0, 0.0, -9.81],
    }
    session = {
        "schema": "lingtu.sim.session.v1",
        "session_id": SESSION_ID,
        "mujoco_version": "3.2.0",
        "seed": 0,
        "world": "test_world@1.0.0",
        "robots": ["test_robot@1.0.0"],
        "runtime": {
            "backend": "mujoco",
            "mode": "headless",
            "required_bindings": ["physics"],
        },
    }
    world_package = {
        "id": "test_world",
        "version": "1.0.0",
        "kind": "world",
        "manifest": "sim/packages/worlds/test_world/world.package.yaml",
    }
    robot_package = {
        "id": "test_robot",
        "version": "1.0.0",
        "kind": "robot",
        "manifest": "sim/packages/robots/test_robot/robot.package.yaml",
    }
    controller_package = {
        "id": "test_controller",
        "version": "1.0.0",
        "kind": "controller",
        "manifest": "sim/packages/controllers/test/controller.package.yaml",
    }
    world = {
        "package": world_package,
        "mjcf": world_mjcf,
    }
    robots = [
        {
            "instance_id": "robot_01",
            "namespace": "robot_01",
            "package": robot_package,
            "controller": controller_package,
            "sensor_rig": None,
            "spawn": {
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "model": {
                "mjcf": "sim/packages/robots/test_robot/robot.xml",
                "attach_root": "base_link",
                "root_joint": "root",
                "initial_keyframe": initial_keyframe,
            },
            "frames": [
                {"name": "base_link", "role": "body"},
                {
                    "name": "lidar_link",
                    "role": "sensor_mount",
                    "parent_frame": "base_link",
                },
                {
                    "name": "lidar_site",
                    "role": "sensor_origin",
                    "parent_frame": "lidar_link",
                },
            ],
            "semantic": {"class": "test_robot"},
        }
    ]
    session_id = session["session_id"]
    sensor_plan = {
        "schema": "lingtu.sim.sensor-plan.v1",
        "session_id": session_id,
        "env": "sim",
        "backends": {},
        "streams": {
            "rgb": (
                [
                    {
                        "instance_id": "robot_01",
                        "sensor_id": "robot_01.front_rgb",
                        "frame_id": "robot_01/camera_link",
                        "rate_hz": camera_hz,
                    }
                ]
                if include_camera
                else []
            ),
            "depth": (
                [
                    {
                        "instance_id": "robot_01",
                        "sensor_id": "robot_01.front_depth",
                        "frame_id": "robot_01/camera_link",
                        "rate_hz": (
                            camera_hz
                            if depth_camera_hz is None
                            else depth_camera_hz
                        ),
                    }
                ]
                if include_camera
                else []
            ),
            "truth_odom": [],
            "imu": (
                [
                    {
                        "instance_id": "robot_01",
                        "sensor_id": "robot_01.imu",
                        "frame_id": "robot_01/imu",
                        "rate_hz": imu_hz,
                    }
                ]
                if include_sensors
                else []
            ),
            "mid360": (
                [
                    {
                        "instance_id": "robot_01",
                        "sensor_id": "robot_01.mid360",
                        "frame_id": "robot_01/lidar_link",
                        "raycast_frame_stable_id": "robot_01/lidar_site",
                        "rate_hz": mid360_hz,
                        "navigation_fixture_raw_overlay": navigation_fixture_raw_overlay,
                    }
                ]
                if include_sensors
                else []
            ),
        },
    }
    control_plan = {
        "schema": "lingtu.sim.control-plan.v1",
        "session_id": session_id,
        "controllers": [
            {
                "instance_id": "robot_01",
                "controller_id": "robot_01.test_controller",
                "package": controller_package,
                "policy": {"artifact": "sim/packages/robots/test_robot/policy.pt"},
                "timing": {"inference_hz": 50, "low_level_hz": 200},
                "actuator_channels": ["joint_a", "joint_b"],
            }
        ],
    }
    return {
        "schema": "lingtu.run_plan.simulation.v1",
        "session_source": "test/minimal.json",
        "session": session,
        "physics_plan": {
            "schema": "lingtu.sim.physics-plan.v1",
            "session_id": session_id,
            "composition": {
                "model_kind": "single_mjmodel",
                "composer": "mjs_attach_v1",
                "namespace_separator": "__",
                "state_authority": "mujoco",
            },
            "global_policy": global_policy,
            "world": world,
            "robots": robots,
        },
        "visual_plan": {"schema": "lingtu.sim.visual-plan.v1", "session_id": session_id},
        "sensor_plan": sensor_plan,
        "control_plan": control_plan,
        "transport_intent": {
            "schema": "lingtu.sim.transport-intent.v1",
            "session_id": session_id,
        },
        "scenario_plan": None,
    }


def _run_plan(
    *,
    product: str = "teleop_avoid",
    lidar: bool = True,
    slam: bool = False,
    colocate_slam: bool = True,
    truth_localization: bool = False,
    mid360_hz: float = 10.0,
    initial_keyframe: str | None = None,
    navigation_fixture_raw_overlay: Any = False,
    camera: bool = False,
    camera_hz: float = 30.0,
    depth_camera_hz: float | None = None,
    enable_camera: Any | None = None,
    lidar_endpoint: str = "lidar_publisher",
    imu_endpoint: str = "imu_publisher",
    camera_endpoint: str = "camera_publisher",
) -> RunPlan:
    artifact = ProcessArtifact(
        path="sim/scripts/mujoco/formal_feeder.py",
    )

    def process(name: str, provides: tuple[str, ...]) -> ProcessSpec:
        return ProcessSpec(
            name=name,
            manager="direct",
            target=f"{name}-target",
            order=10,
            timeout_s=5,
            lifecycle="mode",
            command=ProcessCommand(
                argv=("python", artifact.path),
                cwd=".",
                env=(),
                artifact=artifact,
                readiness=ProcessReadiness("process"),
            ),
            provides=provides,
        )

    processes = [
        process("mujoco_feeder", ()),
        process("driver", ("driver",)),
    ]
    if lidar:
        roles = ("lidar", *(("slam",) if slam and colocate_slam else ()))
        processes.append(process(lidar_endpoint, roles))
        processes.append(process(imu_endpoint, ("imu",)))
    if camera:
        processes.append(process(camera_endpoint, ("camera",)))
    if slam and (not lidar or not colocate_slam):
        processes.append(process("slam_runtime", ("slam",)))
    return RunPlan.create(
        product=product,
        env="sim",
        robot="doso/thunder_v4",
        process_control="subprocess",
        modules=(),
        processes=tuple(processes),
        available_processes=tuple(processes),
        support_processes=("mujoco_feeder",),
        stop_before_start=(),
        contracts=("lingtu.product.nav.v1",),
        critical_modules=(),
        route_contract=None,
        host_config=(
            {}
            if enable_camera is None
            else {"enable_camera": enable_camera}
        ),
        lifecycle={},
        native_process_environment={
            "NAV_GLOBAL_PLANNER": "octoplanner3d",
            "LINGTU_NAV_CONTROL_MODE": "test",
            "LINGTU_NAV_PUBLISH_CMD_VEL": "0",
            "LINGTU_NAV_CHECK_OBSTACLE": "0",
            "LINGTU_NAV_USE_TRAVERSABILITY_COST": "0",
            "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": "0",
            "LINGTU_TELEOP_LOCAL_PLANNER": "0",
            **(
                {
                    "LINGTU_SLAM_CONFIG": (
                        "src/localization/fastlio2/config/sim_mid360.yaml"
                    )
                }
                if truth_localization
                else {}
            ),
        },
        simulation=_minimal_simulation(
            mid360_hz=mid360_hz,
            include_sensors=lidar,
            include_camera=camera,
            camera_hz=camera_hz,
            depth_camera_hz=depth_camera_hz,
            initial_keyframe=initial_keyframe,
            navigation_fixture_raw_overlay=navigation_fixture_raw_overlay,
        ),
        native_nav={
            "global_planner": "octoplanner3d",
            "control_mode": "test",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
        },
    )


def _command(
    kind: str,
    sequence: int,
    *,
    walk: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> DriverBridgeCommand:
    return DriverBridgeCommand(
        bridge_boot_id=BRIDGE_BOOT_ID,
        controller_boot_id=CONTROLLER_BOOT_ID,
        bridge_command_seq=sequence,
        kind=kind,
        producer_boot_id=(f"{PRODUCT_SESSION_ID[:32]}:1234:567890" if kind == "nav" else ""),
        output_sequence=91 if kind == "nav" else 0,
        walk_x=walk[0],
        walk_y=walk[1],
        walk_z=walk[2],
    )


class FakeSensorClient:
    def __init__(self, events: list[Any], role: str, all_records: list[bytes]) -> None:
        self.events = events
        self.role = role
        self.all_records = all_records
        self.records: list[bytes] = []

    def write(self, payload: bytes) -> int:
        self.records.append(payload)
        self.all_records.append(payload)
        self.events.append((self.role, payload[4]))
        return len(payload)

    def close(self) -> None:
        self.events.append(("sensor_close", self.role))


class FakeDriverClient:
    def __init__(self, events: list[Any]) -> None:
        self.events = events

    def send_line(self, line: str) -> None:
        self.events.append(("send", line.split("\t", 1)[0]))

    def recv_line(self) -> str:
        raise AssertionError("the fake session owns scripted protocol input")

    def close(self) -> None:
        self.events.append("driver_close")


class FakeEngine:
    def __init__(
        self,
        events: list[Any],
        *,
        fail_first_step: bool = False,
        unstable_at_step: int | None = None,
    ) -> None:
        self.events = events
        self.fail_first_step = fail_first_step
        self.unstable_at_step = unstable_at_step
        self.steps = 0
        self.sim_time = 0.0
        self.step_periods: list[float] = []
        self.last_state: Any | None = None
        self.position = np.array([0.0, 0.0, 0.4], dtype=np.float64)
        self.lidar_calls = 0
        self.camera_calls = 0
        self.camera_result: Any = SimpleNamespace(
            rgb=np.zeros((480, 640, 3), dtype=np.uint8),
            depth=np.ones((480, 640), dtype=np.float32),
            intrinsics=(500.0, 501.0, 320.0, 240.0),
        )

    def step_sensor_tick(self, command: Any, dt_s: float) -> Any:
        if self.fail_first_step and self.steps == 0:
            raise RuntimeError("synthetic engine failure")
        self.steps += 1
        self.sim_time += float(dt_s)
        self.step_periods.append(float(dt_s))
        self.events.append(
            (
                "step",
                self.steps,
                float(command.linear_x),
                float(command.linear_y),
                float(command.angular_z),
            )
        )
        self.position[0] += float(command.linear_x) * float(dt_s)
        self.position[1] += float(command.linear_y) * float(dt_s)
        if self.unstable_at_step is not None and self.steps >= self.unstable_at_step:
            self.position[2] = 1.5
        self.last_state = SimpleNamespace(
            position=self.position.copy(),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            linear_velocity=np.array([command.linear_x, command.linear_y, 0.0]),
            angular_velocity=np.array([0.0, 0.0, command.angular_z]),
            imu_gyro=np.array([0.0, 0.0, command.angular_z]),
            imu_projected_gravity=np.array([0.0, 0.0, -1.0]),
            imu_linear_acceleration=np.array([0.0, 0.0, 9.80665]),
        )
        return self.last_state

    def get_robot_state(self) -> Any:
        assert self.last_state is not None
        return self.last_state

    def get_lidar_points(self, sample_count: int | None = None) -> Any:
        self.lidar_calls += 1
        count = min(4, int(sample_count or 4))
        points = np.zeros((count, 4), dtype=np.float32)
        points[:, 0] = np.arange(count, dtype=np.float32) + 1.0
        points[:, 3] = 100.0
        return points

    def capture_lidar_snapshot(self) -> None:
        return None

    def get_lidar_points_from_snapshot(
        self,
        _snapshot: Any,
        sample_count: int | None = None,
    ) -> Any:
        return self.get_lidar_points(sample_count=sample_count)

    def get_camera_data(self, camera_name: str = "front_camera") -> Any:
        assert camera_name == "front_camera"
        self.camera_calls += 1
        self.events.append(("camera", self.camera_calls))
        return self.camera_result

    def close(self) -> None:
        self.events.append("engine_close")


class FakeSession:
    def __init__(
        self,
        services: FakeServices,
        *,
        nav: bool,
        pending_nav_on_stop: bool,
        deactivate_failure: bool,
    ) -> None:
        self.services = services
        self.nav = nav
        self.nav_received = False
        self.pending_nav_on_stop = pending_nav_on_stop
        self.deactivate_failure = deactivate_failure
        self.pending_stop_drained = False
        self.activation = _command("activation_zero", 1)
        self.nav_command = _command("nav", 2, walk=(0.5, -0.25, 0.75))
        self.stop_nav = _command("nav", 2, walk=(0.25, 0.0, -0.5))
        stop_sequence = 3 if pending_nav_on_stop else (3 if nav else 2)
        self.deactivation = _command("deactivate_zero", stop_sequence)
        self.confirm_count = 0

    def activate(self) -> DriverBridgeCommand:
        self.services.events.append("activate")
        return self.activation

    def receive_command(self) -> DriverBridgeCommand:
        self.nav_received = True
        self.services.events.append("receive_nav")
        return self.nav_command

    def complete_step(self, command: DriverBridgeCommand, *, step_seq: int) -> None:
        self.services.events.append(("applied", command.kind, step_seq))

    def heartbeat(self, *, step_seq: int) -> None:
        self.services.events.append(("heartbeat", step_seq))

    def confirm_ready(self) -> None:
        self.confirm_count += 1
        self.services.events.append(("ready", self.confirm_count))
        if self.nav and self.confirm_count == 2:
            assert self.services.stop_event is not None
            self.services.stop_event.set()

    def begin_deactivate(self) -> DriverBridgeCommand:
        if self.deactivate_failure:
            self.services.events.append("deactivate_failed")
            raise RuntimeError("synthetic deactivate failure")
        if self.pending_nav_on_stop and not self.pending_stop_drained:
            self.pending_stop_drained = True
            self.services.events.append("deactivate_pending_nav")
            return self.stop_nav
        self.services.events.append("deactivate_zero")
        return self.deactivation

    def wait_stopped(self) -> DriverBridgeStoppedEvidence:
        self.services.events.append("stopped")
        return DriverBridgeStoppedEvidence(
            bridge_boot_id=BRIDGE_BOOT_ID,
            controller_boot_id=CONTROLLER_BOOT_ID,
            bridge_command_seq=self.deactivation.bridge_command_seq,
            applied_step_seq=self.services.engine.steps,
            kind="deactivate_zero",
            producer_boot_id="",
            output_sequence=0,
            walk_x=0.0,
            walk_y=0.0,
            walk_z=0.0,
            terminal_ack=True,
        )

    def close(self) -> None:
        self.services.events.append("session_close")


class FakeServices:
    def __init__(
        self,
        *,
        nav: bool = False,
        pending_nav_on_stop: bool = False,
        stop_after_readiness: bool = True,
        stop_after_waits: int | None = None,
        overshoot_wait: int | None = None,
        overshoot_factor: float = 4.5,
        fail_first_step: bool = False,
        unstable_at_step: int | None = None,
        publish_real_artifacts: bool = False,
        evidence_failure: bool = False,
        deactivate_failure: bool = False,
        readiness_failure_after_write: bool = False,
        status_failures: int = 0,
    ) -> None:
        self.events: list[Any] = []
        self.now = 0.0
        self.wait_count = 0
        self.stop_event: Any | None = None
        self.stop_requested = False
        self.nav = nav
        self.stop_after_readiness = stop_after_readiness
        self.stop_after_waits = stop_after_waits
        self.overshoot_wait = overshoot_wait
        self.overshoot_factor = overshoot_factor
        self.publish_real_artifacts = publish_real_artifacts
        self.evidence_failure = evidence_failure
        self.readiness_failure_after_write = readiness_failure_after_write
        self.status_failures = status_failures
        self.engine = FakeEngine(
            self.events,
            fail_first_step=fail_first_step,
            unstable_at_step=unstable_at_step,
        )
        self.sensor = SimpleNamespace(records=[])
        self.sensors = {
            role: FakeSensorClient(self.events, role, self.sensor.records)
            for role in ("lidar_publisher", "imu_publisher", "camera_publisher")
        }
        self.driver = FakeDriverClient(self.events)
        self.session = FakeSession(
            self,
            nav=nav,
            pending_nav_on_stop=pending_nav_on_stop,
            deactivate_failure=deactivate_failure,
        )
        self.readiness_payload: dict[str, Any] | None = None
        self.evidence_payload: dict[str, Any] | None = None
        self.motion_payload: dict[str, Any] | None = None
        self.status_payloads: list[dict[str, Any]] = []
        self.build_kwargs: dict[str, Any] | None = None
        self.connected_paths: list[Path] = []
        self.wait_timeouts: list[float] = []
        self.expected_product_session_id = PRODUCT_SESSION_ID
        self.camera_writers: dict[str, Any] = {}

    def build_engine(self, **kwargs: Any) -> FakeEngine:
        self.events.append("build_engine")
        self.build_kwargs = kwargs
        return self.engine

    def resolve_artifact(self, value: str) -> Path:
        self.events.append(("resolve_artifact", value))
        return Path(value)

    def snapshot_artifacts(
        self, session_root: Path, config: Any
    ) -> tuple[Path, Path, Path]:
        assert session_root.is_dir()
        return (
            self.resolve_artifact(config.world),
            self.resolve_artifact(config.robot.model),
            self.resolve_artifact(config.robot.policy),
        )

    def connect_sensor(
        self, path: Path, *, role: str, product_session_id: str, timeout_s: float
    ) -> FakeSensorClient:
        assert product_session_id == self.expected_product_session_id and timeout_s > 0.0
        self.events.append("connect_sensor")
        self.connected_paths.append(path)
        return self.sensors[role]

    def connect_driver(
        self, path: Path, *, product_session_id: str, timeout_s: float
    ) -> FakeDriverClient:
        assert product_session_id == self.expected_product_session_id and timeout_s > 0.0
        self.events.append("connect_driver")
        self.connected_paths.append(path)
        return self.driver

    def create_camera_writer(
        self,
        path: Path,
        *,
        stream_kind: Any,
        slot_capacity: int,
    ) -> Any:
        writer = SimpleNamespace(
            path=path,
            stream_kind=stream_kind,
            slot_capacity=slot_capacity,
            published=[],
            publish=lambda **kwargs: writer.published.append(kwargs) or len(writer.published),
            close=lambda: self.events.append(("camera_writer_close", path.name)),
        )
        self.camera_writers[path.name] = writer
        self.events.append(("camera_writer", path.name))
        return writer

    def create_session(
        self, driver: FakeDriverClient, *, expected_product_session_id: str
    ) -> FakeSession:
        assert driver is self.driver
        assert expected_product_session_id == self.expected_product_session_id
        return self.session

    def driver_input_available(self, driver: FakeDriverClient) -> bool:
        assert driver is self.driver
        return self.nav and not self.session.nav_received

    def command_available(
        self, session: FakeSession, driver: FakeDriverClient
    ) -> bool:
        assert session is self.session
        return self.driver_input_available(driver)

    def monotonic(self) -> float:
        return self.now

    def wall_time(self) -> float:
        return 1_800_000_000.0 + self.now

    def wait(self, stop_event: Any, timeout_s: float) -> bool:
        self.stop_event = stop_event
        self.wait_count += 1
        self.wait_timeouts.append(timeout_s)
        multiplier = (
            self.overshoot_factor
            if self.overshoot_wait == self.wait_count
            else 1.0
        )
        self.now += timeout_s * multiplier
        if self.stop_after_waits is not None and self.wait_count > self.stop_after_waits:
            stop_event.set()
        if self.stop_requested:
            stop_event.set()
        return stop_event.is_set()

    @contextmanager
    def wait_scope(self, stop_event: Any) -> Iterator[Any]:
        yield lambda timeout_s: self.wait(stop_event, timeout_s)

    def publish_readiness(
        self, session_root: Path, target: str, payload: dict[str, Any]
    ) -> Path:
        self.events.append("publish_readiness")
        self.readiness_payload = dict(payload)
        if self.stop_after_readiness:
            self.stop_requested = True
        if self.publish_real_artifacts:
            document = feeder.validate_feeder_readiness(payload)
            result = feeder._publish_session_bytes(
                session_root,
                target,
                json.dumps(document, allow_nan=False, separators=(",", ":")).encode(),
            )
        else:
            result = session_root / target
        if self.readiness_failure_after_write:
            raise RuntimeError("synthetic readiness publisher post-write failure")
        return result

    def publish_evidence(
        self, *, session_root: Path, target: str, payload: dict[str, Any]
    ) -> dict[str, Any]:
        self.events.append("publish_evidence")
        if self.evidence_failure:
            raise RuntimeError("synthetic evidence failure")
        self.evidence_payload = dict(payload)
        if self.publish_real_artifacts:
            return dict(
                publish_motion_stop_evidence(
                    session_root=session_root,
                    target=target,
                    payload=payload,
                    bound_publisher=lambda name, raw: feeder._publish_session_bytes(
                        session_root, name, raw
                    ),
                )
            )
        return payload

    def publish_status(
        self, *, session_root: Path, payload: dict[str, Any]
    ) -> dict[str, Any]:
        self.events.append(("publish_status", payload["state"]))
        if self.status_failures:
            self.status_failures -= 1
            raise RuntimeError("synthetic status failure")
        self.status_payloads.append(dict(payload))
        if self.publish_real_artifacts:
            return dict(
                feeder.publish_feeder_status(
                    session_root=session_root,
                    payload=payload,
                    bound_publisher=lambda name, raw: feeder._publish_session_bytes(
                        session_root, name, raw
                    ),
                )
            )
        return payload

    def publish_motion(
        self, *, session_root: Path, payload: dict[str, Any]
    ) -> dict[str, Any]:
        self.events.append("publish_motion")
        self.motion_payload = dict(payload)
        if self.publish_real_artifacts:
            return dict(
                publish_motion_evidence(
                    session_root=session_root,
                    payload=payload,
                    bound_publisher=lambda name, raw: feeder._publish_session_bytes(
                        session_root, name, raw
                    ),
                )
            )
        return payload


def _identity(
    tmp_path: Path,
    *,
    plan: RunPlan | None = None,
) -> Any:
    session_root = tmp_path / "session"
    session_root.mkdir()
    resolved_plan = plan or _run_plan()
    plan_path = resolved_plan.write(
        session_root / f"plan-{PRODUCT_SESSION_ID}.json"
    )

    def publish_session_bytes(
        filename: str, raw: bytes
    ) -> tuple[Path, os.stat_result]:
        destination = session_root / filename
        destination.write_bytes(raw)
        return destination, destination.stat()

    return SimpleNamespace(
        plan=resolved_plan,
        run_plan_path=plan_path,
        open_run_plan=lambda: plan_path.open("rb"),
        close=lambda: None,
        product_session_id=PRODUCT_SESSION_ID,
        product=resolved_plan.product,
        env="sim",
        backend="mujoco",
        session_root=session_root,
        session_io_root=session_root,
        publish_session_bytes=publish_session_bytes,
        driver_readiness_path=session_root / "driver.ready.json",
        motion_stop_path=session_root / "mujoco_feeder.stop.json",
        motion_evidence_path=session_root / MOTION_EVIDENCE_FILENAME,
    )


def _args(*extra: str) -> list[str]:
    return [
        *extra,
    ]


def test_artifact_snapshot_is_independent_of_validated_source_paths(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository.mkdir()
    session_root.mkdir()
    artifacts = {
        "worlds/world.xml": b"<mujoco/>\n",
        "robots/robot.xml": b"<mujoco/>\n",
        "robots/meshes/foot.stl": b"solid foot\nendsolid foot\n",
        "controllers/policy.bin": b"policy-v1",
    }
    for relative, raw in artifacts.items():
        path = repository / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    config = SimpleNamespace(
        world="worlds/world.xml",
        robot=SimpleNamespace(
            model="robots/robot.xml",
            package_root="robots",
            policy="controllers/policy.bin",
        ),
    )
    monkeypatch.setattr(feeder, "_REPOSITORY_ROOT", repository)

    world, robot, policy = feeder._Services.snapshot_artifacts(session_root, config)
    (repository / "worlds/world.xml").write_bytes(b"tampered")

    assert world.read_bytes() == artifacts["worlds/world.xml"]
    assert robot.read_bytes() == artifacts["robots/robot.xml"]
    assert (robot.parent / "meshes" / "foot.stl").read_bytes() == artifacts[
        "robots/meshes/foot.stl"
    ]
    assert policy.read_bytes() == artifacts["controllers/policy.bin"]


def _run(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    services: FakeServices,
    *extra: str,
    plan: RunPlan | None = None,
) -> tuple[int, Any, int]:
    identity = _identity(tmp_path, plan=plan)
    services.expected_product_session_id = identity.product_session_id
    calls = 0

    def current(_environment: Any) -> Any:
        nonlocal calls
        calls += 1
        return identity.plan, identity.session_root, identity.product_session_id

    monkeypatch.setattr(feeder, "_SERVICES", services)
    monkeypatch.setattr(feeder, "_load_run_plan", current)
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", PRODUCT_SESSION_ID)
    rc = feeder.main(_args(*extra))
    return rc, identity, calls


def test_public_surface_and_source_exclude_old_orchestrators() -> None:
    assert str(inspect.signature(feeder.main)) == "(argv: 'Sequence[str] | None' = None) -> 'int'"
    source = Path(feeder.__file__).read_text(encoding="utf-8")
    for forbidden in (
        "native_dds_sensors",
        "RuntimeGraph",
        "Popen",
        "os.kill",
        "pid_file",
        "acceptance",
    ):
        assert forbidden not in source


def test_simulation_artifact_resolution_is_repository_bound() -> None:
    artifact = Path(feeder.__file__).resolve()
    relative = artifact.relative_to(feeder._REPOSITORY_ROOT).as_posix()

    assert feeder._Services.resolve_artifact(relative) == artifact
    with pytest.raises(feeder.FormalFeederError, match="escapes repository"):
        feeder._Services.resolve_artifact("../outside.bin")


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (
            lambda physics, control, sensor: physics["robots"].append(
                dict(physics["robots"][0])
            ),
            "exactly one physics robot",
        ),
        (
            lambda physics, control, sensor: control["controllers"][0].update(
                {"instance_id": "other_robot"}
            ),
            "does not own the physics robot",
        ),
        (
            lambda physics, control, sensor: physics["robots"][0]["spawn"].update(
                {"quaternion_wxyz": [2.0, 0.0, 0.0, 0.0]}
            ),
            "quaternion must be normalized",
        ),
        (
            lambda physics, control, sensor: sensor["streams"].update({"imu": []}),
            "exactly one imu stream",
        ),
        (
            lambda physics, control, sensor: sensor["streams"]["mid360"][0].pop(
                "raycast_frame_stable_id"
            ),
            "raycast_frame_stable_id",
        ),
        (
            lambda physics, control, sensor: sensor["streams"]["mid360"][0].update(
                {"raycast_frame_stable_id": "other_robot/lidar_site"}
            ),
            "raycast frame is outside the robot namespace",
        ),
        (
            lambda physics, control, sensor: physics["robots"][0]["frames"][2].update(
                {"parent_frame": "base_link"}
            ),
            "raycast frame is not attached to the LiDAR body",
        ),
    ),
)
def test_robot_snapshot_binding_fails_closed_before_engine_start(
    mutate: Any,
    message: str,
) -> None:
    simulation = _minimal_simulation()
    physics = simulation["physics_plan"]
    control = simulation["control_plan"]
    sensor = simulation["sensor_plan"]
    mutate(physics, control, sensor)

    with pytest.raises(feeder.FormalFeederError, match=message):
        feeder._single_robot_config(
            physics_plan=physics,
            control_plan=control,
            sensor_plan=sensor,
            sensors_enabled=True,
        )


@pytest.mark.parametrize(
    "forbidden",
    (
        "--product-session-id",
        "--product",
        "--env",
        "--session-root",
        "--sensor-endpoint",
        "--driver-endpoint",
        "--pid-file",
        "--status-file",
        "--acceptance-report",
    ),
)
def test_main_rejects_free_runtime_identity_and_artifact_paths(
    monkeypatch: pytest.MonkeyPatch,
    forbidden: str,
) -> None:
    assert feeder.main([*_args(), forbidden, "unsafe"]) == 2


@pytest.mark.parametrize(
    "extra",
    (
        ("--odom-prior-hz", "1"),
        ("--imu-hz", "0"),
        ("--imu-hz", "501"),
        ("--lidar-hz", "0"),
        ("--lidar-hz", "31"),
        ("--session-id", " padded "),
        ("--model-generation", "-1"),
        ("--reset-generation", "1.5"),
    ),
)
def test_main_rejects_noncanonical_sensor_or_snapshot_arguments(
    monkeypatch: pytest.MonkeyPatch,
    extra: tuple[str, str],
) -> None:
    base = _args()
    option = base.index(extra[0]) if extra[0] in base else -1
    if option >= 0:
        del base[option : option + 2]
    assert feeder.main([*base, *extra]) == 2


def test_main_uses_one_identity_fixed_endpoints_and_exact_startup_order(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, identity, identity_calls = _run(monkeypatch, tmp_path, services)
    plan = RunPlan.load(identity.run_plan_path)
    session_id = plan.simulation["session"]["session_id"]

    assert rc == 0
    assert identity_calls == 1
    assert services.connected_paths == [
        identity.session_root / "lidar.ready.json",
        identity.session_root / "imu.ready.json",
        identity.driver_readiness_path,
    ]
    assert services.events.index("build_engine") < services.events.index("connect_sensor")
    assert services.events.index("connect_sensor") < services.events.index("connect_driver")
    assert services.events.index("connect_driver") < services.events.index("activate")
    activation_step = services.events.index(("step", 1, 0.0, 0.0, 0.0))
    activation_applied = services.events.index(("applied", "activation_zero", 1))
    heartbeat_step = services.events.index(("step", 2, 0.0, 0.0, 0.0))
    heartbeat = services.events.index(("heartbeat", 2))
    ready = services.events.index(("ready", 1))
    published = services.events.index("publish_readiness")
    assert activation_step < activation_applied < heartbeat_step < heartbeat < ready < published
    assert services.readiness_payload == {
        "backend": "mujoco",
        "endpoints": [
            {"protocol": "driver-v2", "role": "driver_bridge"},
            {"protocol": "ltu1-v1", "role": "lidar_publisher"},
            {"protocol": "ltu1-v1", "role": "imu_publisher"},
        ],
        "env": "sim",
        "product_session_id": identity.product_session_id,
        "first_physics_step_applied": True,
        "model_generation": 0,
        "process": "mujoco_feeder",
        "product": "teleop_avoid",
        "protocol": "mujoco-feeder-v1",
        "ready": True,
        "reset_generation": 0,
        "role": "mujoco_feeder",
        "schema": "lingtu.sim.feeder_ready.v1",
        "session_id": session_id,
    }
    assert services.build_kwargs is not None
    assert services.build_kwargs["world"] == Path("sim/packages/worlds/runplan/world.xml")
    assert services.build_kwargs["robot_xml"] == Path(
        "sim/packages/robots/test_robot/robot.xml"
    )
    assert services.build_kwargs["base_body_name"] == "base_link"
    assert services.build_kwargs["lidar_body_name"] == "lidar_link"
    assert services.build_kwargs["lidar_site_name"] == "lidar_site"
    assert services.build_kwargs["physics_timestep_s"] == pytest.approx(0.005)
    assert services.build_kwargs["start"] == [0.0, 0.0, 0.0]
    assert services.build_kwargs["start_orientation_wxyz"] == [1.0, 0.0, 0.0, 0.0]
    assert services.build_kwargs["initial_keyframe"] is None
    assert services.build_kwargs["policy_path"] == Path(
        "sim/packages/robots/test_robot/policy.pt"
    )
    assert services.build_kwargs["policy_freq_hz"] == 50.0
    assert services.build_kwargs["controller_actuator_names"] == [
        "joint_a",
        "joint_b",
    ]
    assert "leg_joint_names" not in services.build_kwargs
    assert services.build_kwargs["mid360_samples_per_frame"] == 20000


def test_sensor_records_are_sent_only_to_their_endpoint(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=False, stop_after_waits=25)

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(slam=True, camera=True, enable_camera=True),
    )

    assert rc == 0
    assert {record[4] for record in services.sensors["imu_publisher"].records} == {
        RECORD_IMU
    }
    assert {record[4] for record in services.sensors["lidar_publisher"].records} <= {
        RECORD_CLOUD,
        RECORD_ODOM_PRIOR,
        RECORD_REGISTERED_CLOUD,
    }
    assert {record[4] for record in services.sensors["camera_publisher"].records} == {
        RECORD_CAMERA
    }
    assert [event for event in services.events if event[0] == "sensor_close"] == [
        ("sensor_close", "lidar_publisher"),
        ("sensor_close", "imu_publisher"),
        ("sensor_close", "camera_publisher"),
    ]


def test_lidar_raycast_runs_only_when_the_ten_hz_scan_is_due() -> None:
    events: list[Any] = []
    records: list[bytes] = []
    engine = FakeEngine(events)
    state = engine.step_sensor_tick(SimpleNamespace(linear_x=0.0, linear_y=0.0, angular_z=0.0), 0.005)
    pipeline = feeder._SensorPipeline(
        services=SimpleNamespace(),
        lidar=FakeSensorClient(events, "lidar", records),
        imu=FakeSensorClient(events, "imu", records),
        engine=engine,
        imu_hz=200.0,
        lidar_hz=10.0,
        samples_per_frame=4000,
        max_points=20000,
        publish_odom_prior=False,
        publish_registered_cloud_fixture=False,
        navigation_fixture_raw_overlay=False,
        started_s=0.0,
        started_wall_s=1000.0,
    )
    for index in range(1, 20):
        now_s = index * 0.005
        pipeline.publish_step(
            state,
            tick=feeder._Tick(now_s=now_s, due_s=now_s, skipped=0),
        )

    assert engine.lidar_calls == 0

    pipeline.publish_step(
        state,
        tick=feeder._Tick(now_s=0.1, due_s=0.1, skipped=0),
    )
    pipeline.close()

    assert engine.lidar_calls == 1


def test_odom_prior_is_not_blocked_by_slow_lidar_raycast() -> None:
    events: list[Any] = []
    records: list[bytes] = []
    frame_started = threading.Event()
    first_odom_written = threading.Event()
    release_frame = threading.Event()
    second_odom_written = threading.Event()

    class SignalingSensorClient(FakeSensorClient):
        def write(self, payload: bytes) -> int:
            written = super().write(payload)
            if payload[4] == RECORD_ODOM_PRIOR:
                odom_count = sum(
                    record[4] == RECORD_ODOM_PRIOR for record in self.records
                )
                first_odom_written.set()
                if odom_count >= 2:
                    second_odom_written.set()
            return written

    class BlockingLidarEngine(FakeEngine):
        def get_lidar_points_from_snapshot(
            self,
            snapshot: Any,
            sample_count: int | None = None,
        ) -> Any:
            frame_started.set()
            assert release_frame.wait(timeout=1.0)
            return super().get_lidar_points_from_snapshot(
                snapshot,
                sample_count=sample_count,
            )

    engine = BlockingLidarEngine(events)
    state = engine.step_sensor_tick(
        SimpleNamespace(linear_x=0.0, linear_y=0.0, angular_z=0.0),
        0.005,
    )
    pipeline = feeder._SensorPipeline(
        services=SimpleNamespace(),
        lidar=SignalingSensorClient(events, "lidar", records),
        imu=FakeSensorClient(events, "imu", records),
        engine=engine,
        imu_hz=200.0,
        lidar_hz=10.0,
        samples_per_frame=4000,
        max_points=20000,
        publish_odom_prior=True,
        publish_registered_cloud_fixture=False,
        navigation_fixture_raw_overlay=False,
        started_s=0.0,
        started_wall_s=1000.0,
    )
    try:
        pipeline.publish_step(
            state,
            tick=feeder._Tick(now_s=0.1, due_s=0.1, skipped=0),
        )
        assert frame_started.wait(timeout=0.5)
        assert first_odom_written.wait(timeout=0.5)
        assert sum(record[4] == RECORD_ODOM_PRIOR for record in records) == 1

        pipeline.publish_step(
            state,
            tick=feeder._Tick(now_s=0.105, due_s=0.105, skipped=0),
        )
        assert second_odom_written.wait(timeout=0.1)
    finally:
        release_frame.set()
        pipeline.close()


def test_sensor_timestamp_uses_observation_time_when_physics_is_late() -> None:
    events: list[Any] = []
    records: list[bytes] = []
    engine = FakeEngine(events)
    state = engine.step_sensor_tick(
        SimpleNamespace(linear_x=0.0, linear_y=0.0, angular_z=0.0),
        0.005,
    )
    pipeline = feeder._SensorPipeline(
        services=SimpleNamespace(),
        lidar=FakeSensorClient(events, "lidar", records),
        imu=FakeSensorClient(events, "imu", records),
        engine=engine,
        imu_hz=200.0,
        lidar_hz=10.0,
        samples_per_frame=4000,
        max_points=20000,
        publish_odom_prior=False,
        publish_registered_cloud_fixture=False,
        navigation_fixture_raw_overlay=False,
        started_s=0.0,
        started_wall_s=1000.0,
    )

    pipeline.publish_step(
        state,
        tick=feeder._Tick(now_s=0.5, due_s=0.1, skipped=0),
    )
    pipeline.close()

    imu_record = next(record for record in records if record[4] == RECORD_IMU)
    assert SENSOR_RECORD_HEADER.unpack_from(imu_record)[2] == 1_000_500_000_000


def test_deadline_loop_uses_one_reversible_waiter_scope(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=False, stop_after_waits=2)
    events: list[str] = []

    @contextmanager
    def waiter(stop_event: Any) -> Iterator[Any]:
        events.append("enter")

        def wait(timeout_s: float) -> bool:
            events.append("wait")
            return services.wait(stop_event, timeout_s)

        try:
            yield wait
        finally:
            events.append("exit")

    monkeypatch.setattr(services, "wait_scope", waiter)

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 0
    assert events[0] == "enter"
    assert "wait" in events
    assert events[-1] == "exit"


def test_formal_feeder_uses_the_high_resolution_monotonic_clock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(feeder.time, "monotonic", lambda: 15.625)
    monkeypatch.setattr(feeder.time, "perf_counter", lambda: 0.005)

    assert feeder._Services.monotonic() == 0.005


@pytest.mark.parametrize(
    "plan",
    (
        _run_plan(lidar_endpoint="wrong_lidar"),
        _run_plan(imu_endpoint="wrong_imu"),
        _run_plan(
            lidar=False,
            camera=True,
            enable_camera=True,
            camera_endpoint="wrong_camera",
        ),
    ),
)
def test_missing_required_sensor_endpoint_fails_before_engine_start(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    plan: RunPlan,
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=plan)

    assert rc == 1
    assert services.build_kwargs is None
    assert "connect_sensor" not in services.events


def test_camera_only_connects_and_closes_only_camera(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, identity, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(lidar=False, camera=True, enable_camera=True),
    )

    assert rc == 0
    assert services.connected_paths == [
        identity.session_root / "camera.ready.json",
        identity.driver_readiness_path,
    ]
    assert services.readiness_payload is not None
    assert services.readiness_payload["endpoints"] == [
        {"protocol": "driver-v2", "role": "driver_bridge"},
        {"protocol": "ltu1-v1", "role": "camera_publisher"},
    ]
    assert [event for event in services.events if event[0] == "sensor_close"] == [
        ("sensor_close", "camera_publisher")
    ]


def test_formal_feeder_forwards_the_run_plan_initial_keyframe(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(initial_keyframe="test_nominal_stand"),
    )

    assert rc == 0
    assert services.build_kwargs is not None
    assert services.build_kwargs["initial_keyframe"] == "test_nominal_stand"


def test_formal_feeder_rejects_unstable_pose_before_readiness(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    services = FakeServices(stop_after_readiness=True)
    services.engine.position[2] = 1.5

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    assert "publish_readiness" not in services.events
    assert services.motion_payload is None
    assert "MuJoCo base pose is outside the stability gate" in capsys.readouterr().err


def test_post_readiness_instability_still_publishes_terminal_zero_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        unstable_at_step=3,
    )

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    unstable_step = services.events.index(("step", 3, 0.0, 0.0, 0.0))
    zero = services.events.index("deactivate_zero")
    zero_step = services.events.index(("step", 4, 0.0, 0.0, 0.0))
    zero_applied = services.events.index(("applied", "deactivate_zero", 4))
    stopped = services.events.index("stopped")
    evidence = services.events.index("publish_evidence")
    assert unstable_step < zero < zero_step < zero_applied < stopped < evidence
    assert services.evidence_payload is not None
    assert services.evidence_payload["outcome"] == "zero_applied"
    assert services.evidence_payload["command_kind"] == "deactivate_zero"
    assert services.evidence_payload["terminal_ack"] is True
    assert services.evidence_payload["walk_x"] == 0.0
    assert services.evidence_payload["walk_y"] == 0.0
    assert services.evidence_payload["walk_z"] == 0.0
    assert services.motion_payload is None
    assert "MuJoCo base pose is outside the stability gate" in capsys.readouterr().err


def test_post_readiness_nav_instability_applies_command_before_emergency_zero(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        nav=True,
        stop_after_readiness=False,
        unstable_at_step=3,
    )

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    nav_step = services.events.index(("step", 3, 0.5, -0.25, 0.75))
    nav_applied = services.events.index(("applied", "nav", 3))
    recovery_step = services.events.index(("step", 4, 0.0, 0.0, 0.0))
    recovery_heartbeat = services.events.index(("heartbeat", 4))
    recovery_ready = services.events.index(("ready", 2))
    zero = services.events.index("deactivate_zero")
    assert (
        nav_step
        < nav_applied
        < recovery_step
        < recovery_heartbeat
        < recovery_ready
        < zero
    )
    assert services.evidence_payload is not None
    assert services.evidence_payload["terminal_ack"] is True


def test_failed_emergency_shutdown_does_not_publish_stop_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        unstable_at_step=3,
        deactivate_failure=True,
    )

    rc, identity, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    assert "publish_readiness" in services.events
    assert "deactivate_failed" in services.events
    assert "publish_evidence" not in services.events
    assert services.evidence_payload is None
    assert not identity.motion_stop_path.exists()
    assert "emergency shutdown failed" in capsys.readouterr().err


def test_emergency_shutdown_never_physically_drains_pending_navigation(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        pending_nav_on_stop=True,
        stop_after_readiness=False,
        unstable_at_step=3,
    )

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    pending = services.events.index("deactivate_pending_nav")
    assert not any(
        isinstance(event, tuple) and event[0] == "step"
        for event in services.events[pending + 1 :]
    )
    assert "stopped" not in services.events
    assert "publish_evidence" not in services.events
    assert services.evidence_payload is None


def test_post_write_readiness_failure_still_publishes_real_terminal_zero(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("LINGTU_PROCESS_LAUNCH_ID", "f" * 64)
    services = FakeServices(
        stop_after_readiness=False,
        publish_real_artifacts=True,
        readiness_failure_after_write=True,
    )

    rc, identity, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    assert (identity.session_root / "mujoco_feeder.ready.json").is_file()
    evidence = load_motion_stop_evidence(
        session_root=identity.session_root,
        target=identity.motion_stop_path.name,
        product_session_id=identity.product_session_id,
        product="teleop_avoid",
        process="mujoco_feeder",
        launch_id=process_launch_id(),
    )
    assert evidence["outcome"] == "zero_applied"
    assert evidence["command_kind"] == "deactivate_zero"
    assert evidence["terminal_ack"] is True
    assert services.motion_payload is None


def test_nav_is_physically_stepped_before_applied_and_stop_zero_precedes_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(nav=True, stop_after_readiness=False)

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 0
    nav_step = next(
        index
        for index, event in enumerate(services.events)
        if isinstance(event, tuple) and event[:2] == ("step", 3)
    )
    nav_applied = services.events.index(("applied", "nav", 3))
    assert services.events[nav_step][2:] == (0.5, -0.25, 0.75)
    assert nav_step < nav_applied
    stop_step = max(
        index
        for index, event in enumerate(services.events)
        if isinstance(event, tuple) and event[0] == "step"
    )
    stop_applied = services.events.index(
        ("applied", "deactivate_zero", services.engine.steps)
    )
    stopped = services.events.index("stopped")
    evidence = services.events.index("publish_evidence")
    motion_evidence = services.events.index("publish_motion")
    assert services.events[stop_step][2:] == (0.0, 0.0, 0.0)
    assert stop_step < stop_applied < stopped < evidence < motion_evidence
    assert services.motion_payload is not None
    assert services.motion_payload["commanded_motion_observed"] is True
    assert services.motion_payload["nonzero_command_count"] == 1
    assert services.motion_payload["nonzero_physics_steps"] >= 2
    assert services.motion_payload["net_displacement_xy_m"] > 0.0
    assert services.motion_payload["path_length_xy_m"] >= services.motion_payload[
        "net_displacement_xy_m"
    ]
    assert services.motion_payload["pose_sample_count"] >= 3
    assert services.motion_payload["min_base_height_m"] == pytest.approx(0.4)
    assert services.motion_payload["max_base_height_m"] == pytest.approx(0.4)
    assert services.motion_payload["max_abs_roll_rad"] == pytest.approx(0.0)
    assert services.motion_payload["max_abs_pitch_rad"] == pytest.approx(0.0)
    assert services.motion_payload["start_yaw_rad"] == pytest.approx(0.0)
    assert services.motion_payload["end_yaw_rad"] == pytest.approx(0.0)
    assert len(services.motion_payload["trajectory"]) >= 2
    assert services.motion_payload["first_output_sequence"] == 91
    assert services.motion_payload["last_output_sequence"] == 91


def test_motion_evidence_includes_coasting_path_between_nonzero_commands(
    tmp_path: Path,
) -> None:
    def state(x: float) -> Any:
        return SimpleNamespace(
            position=np.array([x, 0.0, 0.4], dtype=np.float64),
            orientation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        )

    evidence = feeder._PhysicalMotionEvidence()
    evidence.observe(
        _command("nav", 1, walk=(0.5, 0.0, 0.0)),
        before=state(0.0),
        after=state(1.0),
        step_seq=1,
    )
    evidence.observe(
        _command("activation_zero", 2),
        before=state(1.0),
        after=state(3.0),
        step_seq=2,
    )
    evidence.observe(
        _command("nav", 2, walk=(0.5, 0.0, 0.0)),
        before=state(3.0),
        after=state(4.0),
        step_seq=3,
    )
    stopped = DriverBridgeStoppedEvidence(
        bridge_boot_id=BRIDGE_BOOT_ID,
        controller_boot_id=CONTROLLER_BOOT_ID,
        bridge_command_seq=3,
        applied_step_seq=4,
        kind="deactivate_zero",
        producer_boot_id="",
        output_sequence=0,
        walk_x=0.0,
        walk_y=0.0,
        walk_z=0.0,
        terminal_ack=True,
    )

    published = publish_motion_evidence(
        session_root=tmp_path,
        payload=evidence.payload(
            product_session_id=PRODUCT_SESSION_ID,
            product="teleop_avoid",
            stopped=stopped,
        ),
        environment={"LINGTU_PROCESS_LAUNCH_ID": "e" * 64},
    )

    assert published["path_length_xy_m"] == pytest.approx(4.0)
    assert published["net_displacement_xy_m"] == pytest.approx(4.0)
    assert published["end_position_m"] == [4.0, 0.0, 0.4]
    assert published["start_yaw_rad"] == pytest.approx(0.0)
    assert published["end_yaw_rad"] == pytest.approx(0.0)
    assert published["trajectory"][0][1:4] == [0.0, 0.0, 0.4]
    assert published["trajectory"][-1][1:4] == [4.0, 0.0, 0.4]
    assert published["nonzero_command_count"] == 2
    assert published["nonzero_physics_steps"] == 2
    assert published["last_motion_step_seq"] == 3


def test_motion_trajectory_decimates_without_changing_aggregate_metrics() -> None:
    def state(x: float) -> Any:
        return SimpleNamespace(
            position=np.array([x, 0.0, 0.4], dtype=np.float64),
            orientation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        )

    evidence = feeder._PhysicalMotionEvidence()
    steps = 10_000
    for step in range(1, steps + 1):
        evidence.observe(
            _command("nav", step, walk=(0.5, 0.0, 0.0)),
            before=state((step - 1) * 0.1),
            after=state(step * 0.1),
            step_seq=step,
        )

    assert len(evidence.trajectory) <= MAX_TRAJECTORY_SAMPLES
    assert evidence.trajectory[0][1:4] == [0.0, 0.0, 0.4]
    assert evidence.trajectory[-1][1:4] == [steps * 0.1, 0.0, 0.4]
    assert evidence.path_length_xy_m == pytest.approx(steps * 0.1)
    assert evidence.pose_sample_count == steps
    assert evidence.nonzero_command_count == steps
    assert evidence.nonzero_physics_steps == steps
    replay = feeder._PhysicalMotionEvidence()
    for step in range(steps + 1):
        replay._append_trace((step * 0.1, 0.0, 0.4), 0.0, step)
    assert replay.trajectory == evidence.trajectory


def test_shutdown_drains_one_protocol_pending_nav_before_exact_zero(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        pending_nav_on_stop=True,
        stop_after_readiness=True,
    )

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 0
    pending = services.events.index("deactivate_pending_nav")
    pending_step = next(
        index
        for index, event in enumerate(services.events[pending + 1 :], pending + 1)
        if isinstance(event, tuple) and event[0] == "step"
    )
    pending_applied = services.events.index(("applied", "nav", 3))
    zero = services.events.index("deactivate_zero")
    zero_step = next(
        index
        for index, event in enumerate(services.events[zero + 1 :], zero + 1)
        if isinstance(event, tuple) and event[0] == "step"
    )
    assert pending < pending_step < pending_applied < zero < zero_step
    assert services.events[pending_step][2:] == (0.25, 0.0, -0.5)
    assert services.events[zero_step][2:] == (0.0, 0.0, 0.0)


def test_absolute_sensor_deadlines_catch_up_late_slots_without_dropping(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=14,
        overshoot_wait=4,
        overshoot_factor=50.0,
    )
    plan = _run_plan(mid360_hz=1.0)

    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=plan)

    assert rc == 0
    active_steps = services.engine.steps - 3  # two startup steps and one stop zero
    record_types = [record[4] for record in services.sensor.records]
    assert record_types.count(2) == active_steps
    assert active_steps > 14
    assert record_types.count(1) == 0
    assert services.wait_timeouts
    assert all(timeout > 0.0 for timeout in services.wait_timeouts)
    assert max(services.wait_timeouts) <= 1.0 / 200.0 + 1e-9


def test_feeder_status_reports_caught_up_scheduler_and_terminal_state(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("LINGTU_PROCESS_LAUNCH_ID", "e" * 64)
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=80,
        overshoot_wait=4,
        overshoot_factor=50.0,
        publish_real_artifacts=True,
    )

    rc, identity, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(
            mid360_hz=10.0,
            camera=True,
            enable_camera=True,
        ),
    )

    assert rc == 0
    status = json.loads((identity.session_root / FEEDER_STATUS_FILENAME).read_text())
    assert status["schema"] == FEEDER_STATUS_SCHEMA
    assert status["product_session_id"] == identity.product_session_id
    assert status["product_session_id"] == "1" * 32
    assert status["process"] == "mujoco_feeder"
    assert status["state"] == "stopped"
    assert status["window_s"] > 0.0
    assert services.events.index(("publish_status", "stopped")) > services.events.index(
        "deactivate_zero"
    )
    first = services.status_payloads[0]
    assert first["state"] == "running"
    assert first["window_s"] == 0.0
    assert first["streams"]["camera_rgbd"]["scheduled_count"] == 0
    assert first["streams"]["camera_rgbd"]["published_count"] == 0
    for name in ("imu", "lidar", "camera_rgbd"):
        stream = status["streams"][name]
        assert stream["scheduled_count"] == (
            stream["published_count"] + stream["dropped_count"]
        )
        assert stream["actual_hz"] == pytest.approx(
            stream["published_count"] / status["window_s"]
        )
        assert stream["dropped_count"] == 0
        assert stream["max_schedule_lateness_ms"] > 0.0


def test_camera_warmup_and_readiness_time_do_not_create_runtime_drops(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=25,
    )
    original_camera = services.engine.get_camera_data
    original_readiness = services.publish_readiness
    camera_warmed = False

    def delayed_camera(camera_name: str = "front_camera") -> Any:
        nonlocal camera_warmed
        if not camera_warmed:
            services.now += 0.25
            camera_warmed = True
        return original_camera(camera_name)

    def delayed_readiness(
        identity: Any, target: str, payload: dict[str, Any]
    ) -> Path:
        services.now += 0.25
        return original_readiness(identity, target, payload)

    services.engine.get_camera_data = delayed_camera  # type: ignore[method-assign]
    services.publish_readiness = delayed_readiness  # type: ignore[method-assign]

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(camera=True, enable_camera=True),
    )

    assert rc == 0
    stopped = services.status_payloads[-1]
    assert stopped["window_s"] == pytest.approx(26 / 200.0)
    assert {
        name: stream["dropped_count"]
        for name, stream in stopped["streams"].items()
    } == {"camera_rgbd": 0, "imu": 0, "lidar": 0}


def test_running_status_failure_retries_without_stopping_sensors(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=220,
        status_failures=1,
    )

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 0
    assert services.status_payloads[0]["state"] == "running"
    assert services.status_payloads[-1]["state"] == "stopped"
    assert services.status_payloads[0]["window_s"] >= 1.0
    assert len(services.sensors["imu_publisher"].records) > 0


def test_failed_status_is_best_effort_and_preserves_original_failure(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        unstable_at_step=3,
        status_failures=1,
    )

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    assert ("publish_status", "running") in services.events
    assert ("publish_status", "failed") in services.events
    assert services.events.index(("publish_status", "failed")) > services.events.index(
        "deactivate_zero"
    )
    assert services.status_payloads[-1]["state"] == "failed"


def test_failed_status_balances_the_due_imu_slot_when_write_fails(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=False)

    def fail_write(payload: bytes) -> int:
        raise RuntimeError("synthetic IMU write failure")

    services.sensors["imu_publisher"].write = fail_write  # type: ignore[method-assign]

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 1
    failed = services.status_payloads[-1]
    assert failed["state"] == "failed"
    assert failed["streams"]["imu"]["scheduled_count"] > 0
    assert failed["streams"]["imu"]["published_count"] == 0
    assert (
        failed["streams"]["imu"]["dropped_count"]
        == failed["streams"]["imu"]["scheduled_count"]
    )


def test_truth_localization_publishes_pose_and_registered_cloud_with_one_timestamp(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=25,
    )

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(slam=True),
    )

    assert rc == 0
    lidar_records = services.sensors["lidar_publisher"].records
    record_types = [record[4] for record in lidar_records]
    assert record_types.count(3) > 0
    assert record_types.count(4) > 0
    for index, record in enumerate(lidar_records):
        if record[4] != 4:
            continue
        assert index > 0
        assert lidar_records[index - 1][4] == 1
        registered_stamp = int.from_bytes(record[8:16], "little")
        assert any(
            candidate[4] == 3
            and int.from_bytes(candidate[8:16], "little") == registered_stamp
            for candidate in lidar_records[:index]
        )


def test_product_slam_process_receives_only_raw_lidar_and_imu_records(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=25,
    )

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(slam=True, colocate_slam=False),
    )

    assert rc == 0
    record_types = [record[4] for record in services.sensor.records]
    assert record_types.count(1) > 0
    assert record_types.count(2) > 0
    assert record_types.count(3) == 0
    assert record_types.count(RECORD_REGISTERED_CLOUD) == 0


def test_product_truth_localization_sends_prior_only_to_the_slam_process(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=25,
    )

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(
            slam=True,
            colocate_slam=False,
            truth_localization=True,
        ),
    )

    assert rc == 0
    record_types = [record[4] for record in services.sensor.records]
    assert record_types.count(RECORD_ODOM_PRIOR) > 0
    assert record_types.count(RECORD_REGISTERED_CLOUD) == 0


def test_free_navigation_fixture_publishes_ground_without_raw_returns(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=25,
    )
    services.engine.get_lidar_points = lambda sample_count=None: np.array(
        [[0.25, 0.80, 0.70, 99.0]],
        dtype=np.float32,
    )

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(slam=True),
    )

    assert rc == 0
    registered = next(
        record
        for record in services.sensor.records
        if record[4] == RECORD_REGISTERED_CLOUD
    )
    _, _, _, _, point_count, payload_bytes = SENSOR_RECORD_HEADER.unpack_from(
        registered
    )
    assert payload_bytes == point_count * POINT_DTYPE.itemsize
    points = np.frombuffer(
        registered,
        dtype=POINT_DTYPE,
        count=point_count,
        offset=SENSOR_RECORD_HEADER.size,
    )
    assert point_count > 4
    np.testing.assert_allclose(points["z"], -0.4, atol=1e-6)
    assert not bool(
        np.any(
            np.isclose(points["x"], 0.25)
            & np.isclose(points["y"], 0.80)
            & np.isclose(points["z"], 0.30)
        )
    )


def test_obstacle_navigation_fixture_preserves_raw_obstacle_return(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=25,
    )
    services.engine.get_lidar_points = lambda sample_count=None: np.array(
        [[0.25, 0.80, 0.70, 99.0]],
        dtype=np.float32,
    )

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(
            slam=True,
            navigation_fixture_raw_overlay=True,
        ),
    )

    assert rc == 0
    registered = next(
        record
        for record in services.sensor.records
        if record[4] == RECORD_REGISTERED_CLOUD
    )
    _, _, _, _, point_count, _ = SENSOR_RECORD_HEADER.unpack_from(registered)
    points = np.frombuffer(
        registered,
        dtype=POINT_DTYPE,
        count=point_count,
        offset=SENSOR_RECORD_HEADER.size,
    )
    assert bool(np.any(np.isclose(points["z"], -0.4)))
    assert bool(
        np.any(
            np.isclose(points["x"], 0.25)
            & np.isclose(points["y"], 0.80)
            & np.isclose(points["z"], 0.30)
            & np.isclose(points["intensity"], 99.0)
        )
    )


def test_navigation_fixture_rejects_nonboolean_raw_overlay(
    tmp_path: Path,
) -> None:
    plan = _run_plan(
        slam=True,
        navigation_fixture_raw_overlay="false",
    )
    identity = _identity(tmp_path, plan=plan)

    with pytest.raises(
        feeder.FormalFeederError,
        match="navigation_fixture_raw_overlay must be bool",
    ):
        feeder._run_plan_runtime_config(feeder._parser().parse_args([]), identity.plan)


def test_truth_localization_requires_one_physical_lidar_slam_owner(
    tmp_path: Path,
) -> None:
    plan = _run_plan(lidar=False, slam=True)
    identity = _identity(tmp_path, plan=plan)

    with pytest.raises(
        feeder.FormalFeederError,
        match="simulation truth localization requires lidar",
    ):
        feeder._run_plan_runtime_config(feeder._parser().parse_args([]), identity.plan)


def test_signal_callback_used_by_main_only_sets_the_shared_stop_event(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=False)
    observed: dict[str, Any] = {}

    @contextmanager
    def handlers(stop_event: Any) -> Iterator[None]:
        observed["event"] = stop_event
        yield

    original_publish = services.publish_readiness

    def publish_and_signal(
        session_root: Path, target: str, payload: dict[str, Any]
    ) -> Path:
        result = original_publish(session_root, target, payload)
        feeder._request_stop(observed["event"], signal.SIGTERM, None)
        return result

    services.publish_readiness = publish_and_signal  # type: ignore[method-assign]
    monkeypatch.setattr(feeder, "_installed_stop_handlers", handlers)

    rc, _, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 0
    assert observed["event"].is_set()


def test_clean_stop_publishes_launch_bound_stop_and_motion_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launch_id = "formal-feeder-launch"
    monkeypatch.setenv("LINGTU_PROCESS_LAUNCH_ID", launch_id)
    services = FakeServices(
        stop_after_readiness=True,
        publish_real_artifacts=True,
    )

    rc, identity, _ = _run(monkeypatch, tmp_path, services)

    assert rc == 0
    evidence = load_motion_stop_evidence(
        session_root=identity.session_root,
        target=identity.motion_stop_path.name,
        product_session_id=identity.product_session_id,
        product="teleop_avoid",
        process="mujoco_feeder",
        launch_id=process_launch_id(),
    )
    assert evidence["schema"] == MOTION_STOP_SCHEMA
    motion = load_motion_evidence(
        session_root=identity.session_root,
        product_session_id=identity.product_session_id,
        product="teleop_avoid",
        process="mujoco_feeder",
        launch_id=process_launch_id(),
    )
    assert motion["schema"] == MOTION_EVIDENCE_SCHEMA
    assert motion["commanded_motion_observed"] is False
    assert motion["nonzero_physics_steps"] == 0
    assert motion["net_displacement_xy_m"] == 0.0
    assert motion["pose_sample_count"] > 0
    assert evidence["launch_id"] == launch_id
    assert motion["launch_id"] == launch_id
    assert {path.name for path in identity.session_root.iterdir()} == {
        f"plan-{identity.product_session_id}.json",
        "mujoco_feeder.ready.json",
        FEEDER_STATUS_FILENAME,
        MOTION_EVIDENCE_FILENAME,
        "mujoco_feeder.stop.json",
    }


@pytest.mark.parametrize("failure", ("engine", "evidence"))
def test_failure_is_nonzero_and_never_claims_unproven_stop(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    failure: str,
) -> None:
    services = FakeServices(
        stop_after_readiness=True,
        fail_first_step=failure == "engine",
        evidence_failure=failure == "evidence",
    )

    rc, identity, _ = _run(monkeypatch, tmp_path, services)

    assert rc != 0
    assert not identity.motion_stop_path.exists()
    assert not identity.motion_evidence_path.exists()
    if failure == "engine":
        assert "publish_readiness" not in services.events
        assert "publish_evidence" not in services.events
    else:
        assert services.evidence_payload is None


@pytest.mark.parametrize(
    "override",
    (
        ("--session-id", SESSION_ID),
        ("--model-generation", "1"),
        ("--reset-generation", "1"),
        ("--world", "open_field"),
        ("--imu-hz", "200"),
        ("--lidar-hz", "10"),
        ("--start", "1,2,3"),
        ("--policy-path", "other-policy.pt"),
    ),
)
def test_product_runplan_path_rejects_identity_and_physical_overrides(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    override: tuple[str, str],
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, _, _ = _run(monkeypatch, tmp_path, services, *override)

    assert rc != 0
    assert services.build_kwargs is None
    assert "connect_sensor" not in services.events
    assert "connect_driver" not in services.events


def test_teleop_without_lidar_role_runs_driver_physics_without_sensor_endpoint(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)
    plan = _run_plan(product="teleop", lidar=False)

    rc, identity, _ = _run(monkeypatch, tmp_path, services, plan=plan)

    assert rc == 0
    assert services.connected_paths == [identity.driver_readiness_path]
    assert "connect_sensor" not in services.events
    assert "connect_driver" in services.events
    assert services.build_kwargs is not None
    assert services.engine.steps >= 3
    assert services.engine.step_periods == pytest.approx([0.005, 0.005, 0.005])
    assert services.sensor.records == []
    assert services.readiness_payload is not None
    assert services.readiness_payload["product"] == "teleop"
    assert services.readiness_payload["endpoints"] == [
        {"protocol": "driver-v2", "role": "driver_bridge"},
    ]
    assert services.status_payloads[-1]["streams"] == {}


@pytest.mark.parametrize(
    ("camera", "enable_camera"),
    ((False, True), (True, "true")),
)
def test_enabled_camera_requires_boolean_config_and_process_provider_before_engine_build(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    camera: bool,
    enable_camera: Any,
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(camera=camera, enable_camera=enable_camera),
    )

    assert rc == 1
    assert services.build_kwargs is None
    assert "connect_sensor" not in services.events
    assert services.engine.camera_calls == 0


def test_camera_first_rgbd_bundle_precedes_feeder_readiness(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, identity, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(camera=True, enable_camera=True),
    )

    assert rc == 0
    assert services.build_kwargs is not None
    camera_configs = services.build_kwargs["camera_configs"]
    assert len(camera_configs) == 1
    assert vars(camera_configs[0]) == {
        "name": "front_camera",
        "width": 640,
        "height": 480,
        "fovy": 60.0,
        "render_depth": True,
        "depth_near": 0.1,
        "depth_far": 10.0,
        "fps": 30.0,
    }
    assert services.engine.camera_calls == 1
    camera_records = [record for record in services.sensor.records if record[4] == RECORD_CAMERA]
    assert len(camera_records) == 3
    first_camera_record = next(
        index
        for index, event in enumerate(services.events)
        if isinstance(event, tuple)
        and event[0] == "camera_publisher"
        and event[1] == RECORD_CAMERA
    )
    assert first_camera_record < services.events.index("publish_readiness")
    assert set(services.camera_writers) == {
        "camera_color.shm",
        "camera_depth.shm",
        "camera_info.shm",
    }
    assert all(
        writer.path.parent == identity.session_root
        for writer in services.camera_writers.values()
    )
    assert len(services.camera_writers["camera_color.shm"].published) == 1
    assert len(services.camera_writers["camera_depth.shm"].published) == 1
    assert len(services.camera_writers["camera_info.shm"].published) == 1


def test_camera_writes_reader_valid_rgb_depth_and_intrinsics_shm(
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)
    services.create_camera_writer = feeder._Services.create_camera_writer  # type: ignore[method-assign]
    pipeline = feeder._CameraPipeline(
        services=services,
        client=services.sensors["camera_publisher"],
        engine=services.engine,
        session_root=tmp_path,
        started_s=0.0,
        started_wall_s=1_800_000_000.0,
        rate_hz=30.0,
    )
    try:
        pipeline.publish_initial()
        snapshots = {}
        for name in ("color", "depth", "info"):
            with ShmFrameReader(
                tmp_path / f"camera_{name}.shm",
                max_age_s=None,
            ) as reader:
                snapshots[name] = reader.read_latest(
                    now_ns=1_800_000_000_000_000_000
                )
    finally:
        pipeline.close()

    color = snapshots["color"]
    depth = snapshots["depth"]
    info = snapshots["info"]
    assert color is not None and color.stream_kind is StreamKind.COLOR
    assert color.encoding == "rgb8" and len(color.payload) == 640 * 480 * 3
    assert depth is not None and depth.stream_kind is StreamKind.DEPTH
    assert depth.encoding == "16UC1" and len(depth.payload) == 640 * 480 * 2
    assert info is not None and info.stream_kind is StreamKind.INFO
    assert (info.fx, info.fy, info.cx, info.cy) == (500.0, 501.0, 320.0, 240.0)
    assert color.timestamp_ns == depth.timestamp_ns == info.timestamp_ns


def test_camera_thirty_hz_deadlines_skip_late_slots_without_bursting(
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=False)
    pipeline = feeder._CameraPipeline(
        services=services,
        client=services.sensors["camera_publisher"],
        engine=services.engine,
        session_root=tmp_path,
        started_s=0.0,
        started_wall_s=1_800_000_000.0,
        rate_hz=30.0,
    )
    period_s = 1.0 / 30.0
    try:
        pipeline.publish_initial()
        pipeline.publish_due(monotonic_s=period_s - 1e-6)
        pipeline.publish_due(monotonic_s=period_s)
        pipeline.publish_due(monotonic_s=4.5 * period_s)
        pipeline.publish_due(monotonic_s=4.5 * period_s)
        pipeline.publish_due(monotonic_s=5.0 * period_s)
    finally:
        pipeline.close()

    records = services.sensors["camera_publisher"].records
    rgb_timestamps = [
        SENSOR_RECORD_HEADER.unpack_from(record)[2]
        for record in records
        if record[28 + 6] == 2
    ]
    depth_timestamps = [
        SENSOR_RECORD_HEADER.unpack_from(record)[2]
        for record in records
        if record[28 + 6] == 3
    ]
    assert rgb_timestamps == depth_timestamps
    assert rgb_timestamps == [
        int((1_800_000_000.0 + offset) * 1_000_000_000)
        for offset in (0.0, period_s, 4.5 * period_s, 5.0 * period_s)
    ]
    assert len(services.camera_writers["camera_color.shm"].published) == 4
    assert len(services.camera_writers["camera_depth.shm"].published) == 4
    assert len(services.camera_writers["camera_info.shm"].published) == 1


def test_camera_timestamp_uses_observation_time_when_physics_is_late(
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=False)
    pipeline = feeder._CameraPipeline(
        services=services,
        client=services.sensors["camera_publisher"],
        engine=services.engine,
        session_root=tmp_path,
        started_s=0.0,
        started_wall_s=1_800_000_000.0,
        rate_hz=30.0,
    )
    try:
        pipeline.publish_due(
            monotonic_s=1.0 / 30.0,
            observed_s=0.5,
        )
    finally:
        pipeline.close()

    rgb_record = next(
        record
        for record in services.sensors["camera_publisher"].records
        if record[28 + 6] == 2
    )
    assert SENSOR_RECORD_HEADER.unpack_from(rgb_record)[2] == 1_800_000_000_500_000_000


def test_camera_uses_runplan_thirty_hz_without_bursting_or_republishing_intrinsics(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(
        stop_after_readiness=False,
        stop_after_waits=90,
        overshoot_wait=20,
        overshoot_factor=100.0,
    )

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(camera=True, enable_camera=True),
    )

    assert rc == 0
    camera_records = [record for record in services.sensor.records if record[4] == RECORD_CAMERA]
    kinds = [record[28 + 6] for record in camera_records]
    assert kinds.count(1) == 1
    assert kinds.count(2) == services.engine.camera_calls
    assert kinds.count(3) == services.engine.camera_calls
    assert services.engine.camera_calls < 1 + services.wait_count
    assert len(services.camera_writers["camera_info.shm"].published) == 1
    assert len(services.camera_writers["camera_color.shm"].published) == services.engine.camera_calls
    assert len(services.camera_writers["camera_depth.shm"].published) == services.engine.camera_calls


def test_camera_rejects_mismatched_rgb_depth_rates_before_engine_build(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)
    plan = _run_plan(
        camera=True,
        enable_camera=True,
        camera_hz=30.0,
        depth_camera_hz=15.0,
    )

    rc, _, _ = _run(monkeypatch, tmp_path, services, plan=plan)

    assert rc == 1
    assert services.build_kwargs is None
    assert services.engine.camera_calls == 0


@pytest.mark.parametrize(
    "camera_result",
    (
        None,
        SimpleNamespace(
            rgb=np.zeros((479, 640, 3), dtype=np.uint8),
            depth=np.ones((480, 640), dtype=np.float32),
            intrinsics=(500.0, 501.0, 320.0, 240.0),
        ),
    ),
)
def test_camera_missing_frame_or_shape_drift_fails_closed_before_readiness(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    camera_result: Any,
) -> None:
    services = FakeServices(stop_after_readiness=True)
    services.engine.camera_result = camera_result

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(camera=True, enable_camera=True),
    )

    assert rc == 1
    assert "publish_readiness" not in services.events
    assert services.status_payloads[-1]["state"] == "failed"
    assert services.status_payloads[-1]["streams"]["camera_rgbd"] == {
        "expected_hz": 30.0,
        "scheduled_count": 0,
        "published_count": 0,
        "dropped_count": 0,
        "actual_hz": 0.0,
        "max_schedule_lateness_ms": 0.0,
    }


def test_disabled_camera_does_not_configure_render_or_publish_camera_records(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    services = FakeServices(stop_after_readiness=True)

    rc, _, _ = _run(
        monkeypatch,
        tmp_path,
        services,
        plan=_run_plan(lidar=False, camera=True, enable_camera=False),
    )

    assert rc == 0
    assert services.build_kwargs is not None
    assert services.build_kwargs.get("camera_configs") in (None, [])
    assert services.engine.camera_calls == 0
    assert services.camera_writers == {}
    assert not any(record[4] == RECORD_CAMERA for record in services.sensor.records)
    assert "connect_sensor" not in services.events
