"""Read-only UE presentation of an already-running formal MuJoCo session."""

from __future__ import annotations

import json
import socket
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from sim.runtime.coordinator.coordinator import load_physics_plan
from sim.runtime.coordinator.live_snapshot import UdpLoopbackSnapshotPublisher
from sim.runtime.coordinator.run_allocation import RunAllocation
from sim.runtime.coordinator.unreal_process import (
    MeasurementPresentationBinding,
    UnrealLaunchProfile,
    UnrealProcess,
)

import mujoco
from lingtu.run_plan import RunPlan


class FormalVisual:
    """Own only the visual child and its one-way snapshot transport."""

    def __init__(
        self,
        plan: RunPlan,
        session_root: Path,
        product_session_id: str,
        read_state: Callable,
        *,
        editor: Path,
        repo_root: Path,
        model_generation: int,
        reset_generation: int,
        process_factory=UnrealProcess,
        publisher_factory=UdpLoopbackSnapshotPublisher,
    ) -> None:
        self._plan = plan
        self._root = Path(repo_root).resolve()
        self._run_dir = Path(session_root).resolve() / "visual" / product_session_id
        self._product_id = product_session_id
        self._read_state = read_state
        self._editor = editor
        self._model_generation = model_generation
        self._reset_generation = reset_generation
        self._process_factory = process_factory
        self._publisher_factory = publisher_factory
        self._process: Any = None
        self._publisher = None
        self._allocation: RunAllocation | None = None
        self._sequence = 0
        self._last_snapshot: dict | None = None

    def start(self) -> None:
        """Materialize compiled inputs and start only the presentation child."""
        if self._process is not None:
            raise RuntimeError("formal visual is already started")
        try:
            simulation = self._plan.simulation
            camera_mappings = {}
            for kind in ("rgb", "depth"):
                for stream in simulation["sensor_plan"]["streams"].get(kind, []):
                    if stream.get("source") != "unreal_camera":
                        continue
                    if (simulation["session"]["runtime"]["mode"] != "unreal"
                            or stream.get("owner") != "visual" or stream.get("transport") != "camera_shm"):
                        raise ValueError("invalid formal unreal_camera source route")
                    sensor_id = stream["sensor_id"]
                    camera_mappings[sensor_id] = f"lingtu.sim.camera_shm.{self._product_id}.{sensor_id}"
            presentation = MeasurementPresentationBinding(
                self._product_id,
                f"http://127.0.0.1:{self._plan.host_config['gateway_port']}",
            )
            environment = self._plan.native_process_environment
            for process in self._plan.processes:
                if "camera" in process.provides and process.command is not None:
                    environment.update(dict(process.command.env))
                    break
            domain = environment.get("LINGTU_DDS_DOMAIN_ID", "")
            if not domain.isdecimal() or not 0 <= int(domain) <= 232:
                raise ValueError("formal visual requires the declared Product DDS domain")
            self._run_dir.mkdir(parents=True, exist_ok=False)
            bundle = self._run_dir / "bundle"
            bundle.mkdir()
            (bundle / "session.yaml").write_text(yaml.safe_dump(simulation["session"]), encoding="utf-8")
            for key in (
                "physics_plan",
                "visual_plan",
                "sensor_plan",
                "control_plan",
                "transport_intent",
                "scenario_plan",
            ):
                if simulation.get(key) is not None:
                    (bundle / (key.replace("_", ".") + ".json")).write_text(
                        json.dumps(simulation[key], allow_nan=False),
                        encoding="utf-8",
                    )
            self._physics_plan = replace(
                load_physics_plan(bundle, self._root),
                model_generation=self._model_generation,
                reset_generation=self._reset_generation,
            )
            self._bind_bodies(simulation["visual_plan"])
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                reservation.bind(("127.0.0.1", 0))
                port = reservation.getsockname()[1]
                logs = self._run_dir / "logs"
                logs.mkdir()
                allocation = RunAllocation(
                    run_id=self._product_id,
                    run_dir=self._run_dir,
                    artifact_root=self._root,
                    log_dir=logs,
                    ports={"visual_snapshot_udp": port},
                    shm=camera_mappings,
                    session_id=self._physics_plan.session_id,
                    boot_id=self._product_id,
                    dds_domain=int(domain),
                )
                allocation.path.write_text(allocation.to_json(), encoding="utf-8")
                self._allocation = allocation
                self._publisher = self._publisher_factory(port)
                self._process = self._process_factory(
                    self._editor,
                    self._root / "sim/runtime/visual/RobotSimUE/RobotSimUE.uproject",
                    simulation["visual_plan"]["world"]["level"],
                    launch_profile=UnrealLaunchProfile.PLAYABLE_SDK_QUIET,
                    measurement_presentation=presentation,
                )
            self._process.start(bundle_dir=bundle, allocation=allocation, plan=self._physics_plan, snapshot_port=port)
        except BaseException as error:
            try:
                self.close()
            except BaseException as cleanup_error:
                if hasattr(error, "add_note"):
                    error.add_note(f"formal visual cleanup also failed: {cleanup_error}")
            raise

    def _bind_bodies(self, visual_plan: dict) -> None:
        bindings = set()
        entities = list(visual_plan["robots"]) + list(visual_plan.get("scenario_entities", []))
        for robot in visual_plan["robots"]:
            entities.extend(robot.get("payloads", []))
        for entity in entities:
            instance = entity.get("instance_id", entity.get("entity_id"))
            projection = json.loads((self._root / entity["projection"]["path"]).read_text(encoding="utf-8"))
            for component in projection["components"]:
                frame = component.get("local_body_id", component.get("body_frame"))
                if not isinstance(frame, str) or not frame:
                    raise ValueError("visual projection component has no body frame")
                bindings.add((instance, frame))
        self._bindings = []
        with self._read_state() as (model, _):
            self._model = model
            self._scratch = mujoco.MjData(model)
            for instance, frame in sorted(bindings):
                name = f"{instance}__{frame}"
                body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                if (
                    body_id < 0
                    and len(self._physics_plan.robots) == 1
                    and instance == self._physics_plan.robots[0].instance_id
                ):
                    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, frame)
                if body_id < 0:
                    raise ValueError(f"missing visual body binding: {instance}/{frame}")
                self._bindings.append((instance, frame, body_id))

    @property
    def allocation(self) -> RunAllocation:
        if self._allocation is None or self._process is None:
            raise RuntimeError("formal visual is not started")
        return self._allocation

    def publish(self) -> dict:
        """Copy current truth without advancing or changing the physics lease."""
        if self._process is None or self._publisher is None:
            raise RuntimeError("formal visual is not started")
        if self._process.poll() is not None:
            raise RuntimeError("Unreal visual process exited")
        with self._read_state() as (model, data):
            if model is not self._model:
                raise ValueError("formal visual physics model changed")
            scratch = self._scratch
            scratch.qpos[:] = data.qpos
            scratch.qvel[:] = data.qvel
            scratch.mocap_pos[:] = data.mocap_pos
            scratch.mocap_quat[:] = data.mocap_quat
            mujoco.mj_kinematics(model, scratch)
            mujoco.mj_comPos(model, scratch)
            mujoco.mj_comVel(model, scratch)
            bodies = []
            for instance, frame, body_id in self._bindings:
                velocity = np.zeros(6)
                mujoco.mj_objectVelocity(model, scratch, mujoco.mjtObj.mjOBJ_BODY, body_id, velocity, 0)
                bodies.append(
                    {
                        "stable_id": f"{instance}/{frame}",
                        "instance_id": instance,
                        "frame_id": frame,
                        "position_m": scratch.xpos[body_id].tolist(),
                        "quaternion_wxyz": scratch.xquat[body_id].tolist(),
                        "linear_velocity_mps": velocity[3:].tolist(),
                        "angular_velocity_rps": velocity[:3].tolist(),
                    }
                )
            sim_time = float(data.time)
        self._sequence += 1
        snapshot = {
                "event": "snapshot",
                "session_id": self._physics_plan.session_id,
                "model_generation": self._model_generation,
                "reset_generation": self._reset_generation,
                "sequence": self._sequence,
                "sim_time_ns": round(sim_time * 1e9),
                "physics_step": round(sim_time / self._physics_plan.global_policy.timestep_s),
                "bodies": bodies,
            }
        self._publisher.publish(snapshot)
        self._last_snapshot = snapshot
        return snapshot

    def republish(self) -> dict:
        """Repeat frozen startup truth; never invent a sample while time is stopped."""
        if self._process is None or self._publisher is None or self._last_snapshot is None:
            raise RuntimeError("formal visual has no startup snapshot")
        if self._process.poll() is not None:
            raise RuntimeError("Unreal visual process exited")
        self._publisher.publish(self._last_snapshot)
        return self._last_snapshot

    def close(self) -> None:
        """Release owned resources, including after a partially failed start."""
        process, self._process = self._process, None
        publisher, self._publisher = self._publisher, None
        try:
            if process is not None:
                process.terminate()
        finally:
            if publisher is not None:
                publisher.close()

    def ready(self) -> bool:
        """Read only this allocation's UE-owned visual activation evidence.

        This format has no Product ID, sequence, or time association. It is not
        evidence of sensor readiness, measurement freshness, or motion admission.
        """
        if self._process is None:
            return False
        path = self._run_dir / "logs" / "visual-readiness.json"
        try:
            evidence = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return False
        expected = {
            "schema": "lingtu.sim.sensor-readiness-evidence.v1",
            "source_id": "robotsimue-visual",
            "basis": "truth_snapshot_applied_to_visual_bindings",
            "session_id": self._physics_plan.session_id,
            "model_generation": self._model_generation,
            "reset_generation": self._reset_generation,
        }
        if not isinstance(evidence, dict) or any(
            type(evidence.get(key)) is not type(value) or evidence.get(key) != value for key, value in expected.items()
        ):
            raise ValueError("invalid visual readiness schema or source identity")
        visual = evidence.get("visual")
        if (
            not isinstance(visual, dict)
            or not isinstance(visual.get("state"), str)
            or visual["state"] not in {"PREPARED", "ACTIVE"}
        ):
            raise ValueError("invalid visual readiness state")
        state: str = visual["state"]
        return state == "ACTIVE"
