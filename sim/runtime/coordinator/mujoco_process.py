"""Process adapter for the headless C++ MuJoCo runtime host."""

from __future__ import annotations

import json
import math
import os
import queue
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, TextIO

from sim.runtime.process_owner import ProcessShutdownSnapshot, ProcessTreeOwner
from sim.runtime.windows_cpu_isolation import validate_windows_affinity_mask

from .coordinator import CoordinatorError, PhysicsPlan, RunAllocation

if TYPE_CHECKING:
    from sim.runtime.control import ActuatorCommand
    from sim.runtime.scenario import ScenarioSnapshot


@dataclass(frozen=True)
class _ActuatorBinding:
    instance_id: str
    command_type: str
    channels: tuple[str, ...]


class MujocoProcess:
    """Host one MujocoRuntime through its line-oriented control protocol."""

    def __init__(
        self,
        executable: Path,
        *,
        timeout_s: float = 15.0,
        affinity_mask: int | None = None,
        sample_stride_steps: int = 1,
    ) -> None:
        if (
            isinstance(sample_stride_steps, bool)
            or not isinstance(sample_stride_steps, int)
            or not 1 <= sample_stride_steps <= 4096
        ):
            raise ValueError("sample_stride_steps must be an integer in [1, 4096]")
        self._executable = Path(executable).resolve()
        self._timeout_s = timeout_s
        self._sample_stride_steps = sample_stride_steps
        self._affinity_mask = (
            validate_windows_affinity_mask(affinity_mask, "MuJoCo affinity mask")
            if affinity_mask is not None
            else None
        )
        self._process: subprocess.Popen[str] | None = None
        self._process_owner: ProcessTreeOwner | None = None
        self._stderr: TextIO | None = None
        self._events: queue.Queue[str | None] = queue.Queue()
        self._reader: threading.Thread | None = None
        self._actuator_bindings: dict[str, _ActuatorBinding] = {}
        self._request_lock = threading.Lock()
        self._last_shutdown: ProcessShutdownSnapshot | None = None

    @property
    def pid(self) -> int | None:
        """Return the child process ID while the process is owned."""

        return self._process.pid if self._process is not None else None

    @property
    def last_shutdown(self) -> ProcessShutdownSnapshot | None:
        """Return immutable facts from the most recent owned host closure."""

        return self._last_shutdown

    def prepare(
        self, plan: PhysicsPlan, allocation: RunAllocation
    ) -> dict[str, Any]:
        """Launch the C++ host and wait for its READY event."""

        if self._process is not None:
            raise CoordinatorError("MuJoCo process is already prepared")
        self._last_shutdown = None
        if not self._executable.is_file():
            raise CoordinatorError(
                f"MuJoCo host executable does not exist: {self._executable}"
            )
        command = self._command(plan)
        stderr_path = allocation.log_dir / "physics.stderr.log"
        self._stderr = stderr_path.open("w", encoding="utf-8")
        creationflags = (
            subprocess.CREATE_NO_WINDOW
            if os.name == "nt" and hasattr(subprocess, "CREATE_NO_WINDOW")
            else 0
        )
        try:
            self._process_owner = (
                ProcessTreeOwner(affinity_mask=self._affinity_mask)
                if self._affinity_mask is not None
                else ProcessTreeOwner()
            )
            self._process = subprocess.Popen(  # noqa: S603 - validated executable, argv only
                command,
                cwd=plan.repo_root,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=self._stderr,
                text=True,
                encoding="utf-8",
                errors="strict",
                bufsize=1,
                env=allocation.child_environment(),
                **self._process_owner.popen_options(creationflags=creationflags),
            )
            self._process_owner.attach(self._process)
            stdout = self._process.stdout
            if stdout is None:
                raise CoordinatorError("MuJoCo host stdout pipe was not created")
            self._reader = threading.Thread(
                target=self._read_stdout,
                args=(stdout,),
                name="mujoco-headless-stdout",
                daemon=True,
            )
            self._reader.start()
            return self._read_event()
        except BaseException as launch_error:
            try:
                self._terminate()
            except BaseException as cleanup_error:
                _add_exception_note(
                    launch_error,
                    f"MuJoCo failed-launch cleanup also failed: {cleanup_error}",
                )
            raise

    def start(self) -> dict[str, Any]:
        """Tell the host to enter RUNNING."""

        return self._request("start")

    def advance(self, steps: int) -> dict[str, Any]:
        """Advance the host and return its snapshot event."""

        return self._request(f"advance {steps}")

    def advance_sampled(self, steps: int) -> tuple[dict[str, Any], ...]:
        """Advance once while retaining every fixed-step truth snapshot.

        The line protocol still uses one request/response round trip.  The
        intermediate snapshots preserve exact sensor deadlines inside a
        realtime batch without forcing one pipe transaction per MuJoCo step.
        """

        count = self._uint_range(
            steps,
            "steps",
            minimum=1,
            maximum=4096,
        )
        if self._sample_stride_steps == 1:
            request = f"advance-sampled {count}"
        else:
            request = (
                "advance-sampled-realtime "
                f"{count} {self._sample_stride_steps}"
            )
        event = self._request(request)
        if event.get("event") != "snapshot_batch":
            raise CoordinatorError(
                "MuJoCo host returned an invalid sampled-advance event"
            )
        return self._snapshot_batch(event, maximum_count=count)

    def advance_sampled_with_actuator(
        self,
        command: ActuatorCommand,
        steps: int,
    ) -> tuple[dict[str, Any], ...]:
        """Apply one command and advance sampled truth in one host transaction.

        Physics validates and applies the command before advancing. A rejected
        command therefore produces no physics step and fails closed here.
        """

        count = self._uint_range(steps, "steps", minimum=1, maximum=4096)
        fields = self._actuator_command_fields(command)
        event = self._request(
            " ".join(
                (
                    "actuate-advance-sampled-realtime",
                    str(count),
                    str(self._sample_stride_steps),
                    *fields,
                )
            )
        )
        if (
            event.get("event") != "actuator_snapshot_batch"
            or event.get("source_id") != command.controller_id
            or event.get("sequence") != command.sequence
            or not isinstance(event.get("result"), str)
        ):
            raise CoordinatorError(
                "MuJoCo host returned an invalid actuator sampled-advance event"
            )
        result = event["result"]
        if result != "applied":
            raise CoordinatorError(
                f"Physics Runtime rejected actuator command: {result}"
            )
        return self._snapshot_batch(event, maximum_count=count)

    def pause(self) -> dict[str, Any]:
        """Tell the host to enter PAUSED."""

        return self._request("pause")

    def reset(self) -> dict[str, Any]:
        """Reset the hosted runtime."""

        return self._request("reset")

    def snapshot(self) -> dict[str, Any]:
        """Return the current immutable physics snapshot without advancing."""

        return self._request("snapshot")

    def raycast(
        self,
        *,
        sensor_frame_id: str,
        directions_sensor: tuple[tuple[float, float, float], ...],
        session_id: str,
        model_generation: int,
        reset_generation: int,
        sequence: int,
        sim_time_ns: int,
        offsets_time_ns: tuple[int, ...] | None = None,
        range_min_m: float = 0.1,
        range_max_m: float = 40.0,
        reflectivity_proxy: int = 15,
        unknown_line: int = 0,
    ) -> dict[str, Any]:
        """Raycast a batch of sensor-frame rays through the hosted MuJoCo model."""

        sensor_frame_id = self._protocol_token(
            sensor_frame_id, "sensor_frame_id", maximum_bytes=127
        )
        session_id = self._session_id(session_id, "raycast session_id")
        range_min = self._finite_float(range_min_m, "range_min_m")
        range_max = self._finite_float(range_max_m, "range_max_m")
        if range_min < 0.0 or range_max <= range_min:
            raise CoordinatorError("raycast range must be finite and increasing")
        reflectivity = self._uint_range(
            reflectivity_proxy, "reflectivity_proxy", minimum=1, maximum=255
        )
        line = self._uint_range(unknown_line, "unknown_line", minimum=0, maximum=255)
        expected_model_generation = self._uint_range(
            model_generation, "model_generation", minimum=0
        )
        expected_reset_generation = self._uint_range(
            reset_generation, "reset_generation", minimum=0, maximum=0xFFFFFFFF
        )
        expected_sequence = self._uint_range(
            sequence, "sequence", minimum=0, maximum=0xFFFFFFFFFFFFFFFF
        )
        expected_sim_time_ns = self._uint_range(
            sim_time_ns, "sim_time_ns", minimum=0, maximum=0xFFFFFFFFFFFFFFFF
        )
        if not isinstance(directions_sensor, tuple) or not directions_sensor:
            raise CoordinatorError("directions_sensor must be a non-empty tuple")
        if len(directions_sensor) > 200000:
            raise CoordinatorError("directions_sensor contains more than 200000 rays")
        if offsets_time_ns is None:
            offsets = (0,) * len(directions_sensor)
        else:
            if not isinstance(offsets_time_ns, tuple):
                raise CoordinatorError("offsets_time_ns must be a tuple")
            if len(offsets_time_ns) != len(directions_sensor):
                raise CoordinatorError("offsets_time_ns must match directions_sensor length")
            offsets = offsets_time_ns

        ray_fields: list[str] = []
        for index, direction in enumerate(directions_sensor):
            if not isinstance(direction, tuple) or len(direction) != 3:
                raise CoordinatorError(f"directions_sensor[{index}] must contain 3 values")
            values = tuple(
                self._finite_float(value, f"directions_sensor[{index}]")
                for value in direction
            )
            if values[0] == 0.0 and values[1] == 0.0 and values[2] == 0.0:
                raise CoordinatorError(f"directions_sensor[{index}] must be non-zero")
            offset = self._uint_range(
                offsets[index], f"offsets_time_ns[{index}]", minimum=0, maximum=0xFFFFFFFF
            )
            ray_fields.extend((*(format(value, ".17g") for value in values), str(offset)))

        line_text = " ".join(
            (
                "raycast",
                sensor_frame_id,
                session_id,
                str(expected_model_generation),
                str(expected_reset_generation),
                str(expected_sequence),
                str(expected_sim_time_ns),
                format(range_min, ".17g"),
                format(range_max, ".17g"),
                str(reflectivity),
                str(line),
                str(len(directions_sensor)),
                *ray_fields,
            )
        )
        event = self._request(line_text)
        self._validate_raycast_event(
            event,
            sensor_frame_id=sensor_frame_id,
            session_id=session_id,
            model_generation=expected_model_generation,
            reset_generation=expected_reset_generation,
            sequence=expected_sequence,
            sim_time_ns=expected_sim_time_ns,
        )
        return event

    def bind_actuators(
        self,
        *,
        source_id: str,
        instance_id: str,
        command_type: str,
        stale_timeout_ns: int,
        channels: tuple[str, ...],
    ) -> dict[str, Any]:
        """Resolve one stable actuator layout before physics starts advancing."""

        source_id = self._protocol_token(source_id, "source_id", maximum_bytes=127)
        instance_id = self._protocol_token(
            instance_id, "instance_id", maximum_bytes=127
        )
        command_type = self._protocol_token(
            command_type, "command_type", maximum_bytes=63
        )
        if (
            isinstance(stale_timeout_ns, bool)
            or not isinstance(stale_timeout_ns, int)
            or stale_timeout_ns <= 0
        ):
            raise CoordinatorError("stale_timeout_ns must be a positive integer")
        if not isinstance(channels, tuple) or not channels or len(channels) > 128:
            raise CoordinatorError("actuator channels must be a tuple containing 1..128 items")
        validated_channels = tuple(
            self._protocol_token(channel, "actuator channel", maximum_bytes=127)
            for channel in channels
        )
        if len(set(validated_channels)) != len(validated_channels):
            raise CoordinatorError("actuator channels must be unique")
        if source_id in self._actuator_bindings:
            raise CoordinatorError(f"actuator source {source_id!r} is already bound")

        line = " ".join(
            (
                "bind-actuators",
                source_id,
                instance_id,
                command_type,
                str(stale_timeout_ns),
                str(len(validated_channels)),
                *validated_channels,
            )
        )
        event = self._request(line)
        expected = {
            "event": "actuator_bound",
            "source_id": source_id,
            "instance_id": instance_id,
            "command_type": command_type,
            "channel_count": len(validated_channels),
        }
        if any(event.get(key) != value for key, value in expected.items()):
            raise CoordinatorError("MuJoCo host returned an invalid actuator binding event")
        self._actuator_bindings[source_id] = _ActuatorBinding(
            instance_id=instance_id,
            command_type=command_type,
            channels=validated_channels,
        )
        return event

    def apply_actuator_command(self, command: ActuatorCommand) -> dict[str, Any]:
        """Send dense values using the stable layout established during binding."""

        fields = self._actuator_command_fields(command)
        event = self._request(" ".join(("actuate", *fields)))
        if (
            event.get("event") != "actuator_command"
            or event.get("source_id") != command.controller_id
            or event.get("sequence") != command.sequence
            or not isinstance(event.get("result"), str)
        ):
            raise CoordinatorError("MuJoCo host returned an invalid actuator command event")
        return event

    def _actuator_command_fields(self, command: ActuatorCommand) -> tuple[str, ...]:
        binding = self._actuator_bindings.get(command.controller_id)
        if binding is None:
            raise CoordinatorError(
                f"actuator source {command.controller_id!r} has not been bound"
            )
        if command.instance_id != binding.instance_id:
            raise CoordinatorError("actuator command instance does not match its binding")
        if command.command_type != binding.command_type:
            raise CoordinatorError("actuator command type does not match its binding")
        if command.channels != binding.channels:
            raise CoordinatorError("actuator channel layout does not match its binding")

        values = tuple(format(value, ".17g") for value in command.values)
        return (
            command.controller_id,
            command.instance_id,
            command.command_type,
            command.session_id,
            str(command.generation.model_generation),
            str(command.generation.reset_generation),
            str(command.sequence),
            str(command.apply_time_ns),
            "1" if command.safe_stop else "0",
            str(len(values)),
            *values,
        )

    @staticmethod
    def _snapshot_batch(
        event: dict[str, Any],
        *,
        maximum_count: int,
    ) -> tuple[dict[str, Any], ...]:
        raw_snapshots = event.get("snapshots")
        if (
            not isinstance(raw_snapshots, list)
            or not 1 <= len(raw_snapshots) <= maximum_count
        ):
            raise CoordinatorError(
                "MuJoCo sampled advance returned an invalid snapshot count"
            )
        snapshots: list[dict[str, Any]] = []
        for index, snapshot in enumerate(raw_snapshots):
            if not isinstance(snapshot, dict) or snapshot.get("event") != "snapshot":
                raise CoordinatorError(
                    "MuJoCo sampled advance returned an invalid snapshot "
                    f"at index {index}"
                )
            snapshots.append(snapshot)
        return tuple(snapshots)

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> dict[str, Any]:
        """Apply one complete generation-stamped scenario pose batch atomically."""

        session_id = self._session_id(snapshot.session_id, "scenario session_id")
        model_generation = self._uint_range(
            snapshot.model_generation,
            "scenario model_generation",
            minimum=0,
            maximum=0xFFFFFFFFFFFFFFFF,
        )
        reset_generation = self._uint_range(
            snapshot.reset_generation,
            "scenario reset_generation",
            minimum=0,
            maximum=0xFFFFFFFF,
        )
        sequence = self._uint_range(
            snapshot.sequence,
            "scenario sequence",
            minimum=0,
            maximum=0xFFFFFFFFFFFFFFFF,
        )
        sim_time_ns = self._uint_range(
            snapshot.sim_time_ns,
            "scenario sim_time_ns",
            minimum=0,
            maximum=0xFFFFFFFFFFFFFFFF,
        )
        if not snapshot.entities:
            raise CoordinatorError("kinematic pose batch must contain at least one entity")
        entity_fields: list[str] = []
        for index, entity in enumerate(snapshot.entities):
            if (
                entity.authority != "scenario"
                or entity.physics_proxy_mode != "kinematic"
                or entity.body_stable_id is None
            ):
                raise CoordinatorError(
                    f"scenario entities[{index}] is not a routed MuJoCo kinematic proxy"
                )
            entity_id = self._protocol_token(
                entity.entity_id, f"scenario entities[{index}].entity_id", maximum_bytes=127
            )
            body_stable_id = self._protocol_token(
                entity.body_stable_id,
                f"scenario entities[{index}].body_stable_id",
                maximum_bytes=127,
            )
            position = tuple(
                self._finite_float(value, f"scenario entities[{index}].position_m")
                for value in entity.transform.position_m
            )
            quaternion = tuple(
                self._finite_float(value, f"scenario entities[{index}].quaternion_wxyz")
                for value in entity.transform.quaternion_wxyz
            )
            entity_fields.extend(
                (
                    entity_id,
                    body_stable_id,
                    *(format(value, ".17g") for value in position),
                    *(format(value, ".17g") for value in quaternion),
                )
            )

        line = " ".join(
            (
                "kinematic-poses",
                session_id,
                str(model_generation),
                str(reset_generation),
                str(sequence),
                str(sim_time_ns),
                str(len(snapshot.entities)),
                *entity_fields,
            )
        )
        event = self._request(line)
        expected = {
            "event": "kinematic_poses",
            "session_id": session_id,
            "model_generation": model_generation,
            "reset_generation": reset_generation,
            "sequence": sequence,
            "sim_time_ns": sim_time_ns,
            "entity_count": len(snapshot.entities),
        }
        if any(event.get(key) != value for key, value in expected.items()):
            raise CoordinatorError("MuJoCo host returned an invalid kinematic pose event")
        result = event.get("result")
        if result != "applied":
            raise CoordinatorError(f"MuJoCo host rejected kinematic poses: {result!s}")
        return event

    def stop(self) -> dict[str, Any]:
        """Stop the host and release all process resources."""

        if self._process is None:
            return {
                "event": "stopped",
                "session_id": "",
                "model_generation": 0,
                "reset_generation": 0,
                "sequence": 0,
                "physics_step": 0,
                "sim_time_ns": 0,
            }
        try:
            event = self._request("stop")
            self._process.wait(timeout=self._timeout_s)
            if self._process.returncode != 0:
                raise CoordinatorError(
                    f"MuJoCo host exited with code {self._process.returncode}"
                )
        except BaseException as stop_error:
            try:
                self._terminate()
            except BaseException as cleanup_error:
                _add_exception_note(
                    stop_error,
                    f"MuJoCo interrupted-stop cleanup also failed: {cleanup_error}",
                )
            raise
        else:
            self._close(termination_mode="natural")
            return event

    def _command(self, plan: PhysicsPlan) -> list[str]:
        command = [
            str(self._executable),
            "--session",
            plan.session_id,
            str(plan.model_generation),
            str(plan.world_model_path),
            "--global-policy",
            format(plan.global_policy.timestep_s, ".17g"),
            plan.global_policy.integrator,
            plan.global_policy.solver,
            str(plan.global_policy.iterations),
            *(format(value, ".17g") for value in plan.global_policy.gravity_mps2),
        ]
        for robot in plan.robots:
            command.extend(
                (
                    "--robot",
                    robot.instance_id,
                    str(robot.model_path),
                    robot.attach_root,
                    *(format(value, ".17g") for value in robot.position_m),
                    *(format(value, ".17g") for value in robot.quaternion_wxyz),
                )
            )
            if robot.initial_keyframe is not None:
                command.extend(("--initial-keyframe", robot.initial_keyframe))
        for payload in plan.payloads:
            command.extend(
                (
                    "--payload",
                    payload.instance_id,
                    payload.namespace,
                    payload.robot_instance_id,
                    payload.package["id"],
                    payload.package["version"],
                    payload.package["kind"],
                    payload.package["manifest"],
                    payload.parent_frame,
                    payload.parent_body,
                    *(format(value, ".17g") for value in payload.position_m),
                    *(format(value, ".17g") for value in payload.quaternion_wxyz),
                    str(payload.model_path),
                    payload.attach_root,
                    payload.authority,
                    payload.collision_representation,
                    str(len(payload.frames)),
                )
            )
            for frame in payload.frames:
                command.extend(
                    (
                        frame["name"],
                        frame["role"],
                        frame.get("parent_frame", "-"),
                    )
                )
        for entity in plan.kinematic_entities:
            command.extend(
                (
                    "--kinematic-entity",
                    entity.entity_id,
                    str(entity.model_path),
                    entity.attach_root,
                    *(format(value, ".17g") for value in entity.position_m),
                    *(format(value, ".17g") for value in entity.quaternion_wxyz),
                )
            )
        return command

    def _request(self, command: str) -> dict[str, Any]:
        with self._request_lock:
            process = self._process
            if process is None or process.stdin is None:
                raise CoordinatorError("MuJoCo process is not prepared")
            if process.poll() is not None:
                raise CoordinatorError(
                    f"MuJoCo host exited before command {command!r}"
                )
            try:
                process.stdin.write(command + "\n")
                process.stdin.flush()
            except (BrokenPipeError, OSError) as exc:
                raise CoordinatorError("cannot write to the MuJoCo host") from exc
            return self._read_event()

    @staticmethod
    def _finite_float(value: object, field: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CoordinatorError(f"{field} must be numeric")
        number = float(value)
        if not math.isfinite(number):
            raise CoordinatorError(f"{field} must be finite")
        return number

    @staticmethod
    def _uint_range(
        value: object,
        field: str,
        *,
        minimum: int,
        maximum: int | None = None,
    ) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            raise CoordinatorError(f"{field} must be an integer >= {minimum}")
        if maximum is not None and value > maximum:
            raise CoordinatorError(f"{field} must be <= {maximum}")
        return value

    def _validate_raycast_event(
        self,
        event: dict[str, Any],
        *,
        sensor_frame_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        sequence: int,
        sim_time_ns: int,
    ) -> None:
        if event.get("event") != "raycast":
            raise CoordinatorError("MuJoCo host returned an invalid raycast event")
        if event.get("sensor_frame_id") != sensor_frame_id:
            raise CoordinatorError("MuJoCo host returned raycast data for the wrong stable frame")
        if event.get("session_id") != session_id:
            raise CoordinatorError("MuJoCo raycast session_id does not match")
        if event.get("model_generation") != model_generation:
            raise CoordinatorError("MuJoCo raycast model_generation does not match")
        if event.get("reset_generation") != reset_generation:
            raise CoordinatorError("MuJoCo raycast reset_generation does not match")
        if event.get("sequence") != sequence:
            raise CoordinatorError("MuJoCo raycast sequence does not match")
        if event.get("sim_time_ns") != sim_time_ns:
            raise CoordinatorError("MuJoCo raycast sim_time_ns does not match")
        hits = event.get("hits")
        if type(hits) is not list or event.get("hit_count") != len(hits):
            raise CoordinatorError("MuJoCo raycast hits must match hit_count")
        for index, hit in enumerate(hits):
            if type(hit) is not dict:
                raise CoordinatorError(f"MuJoCo raycast hits[{index}] must be an object")
            xyz = hit.get("xyz_sensor")
            if type(xyz) is not list or len(xyz) != 3:
                raise CoordinatorError(f"MuJoCo raycast hits[{index}].xyz_sensor must contain 3 values")
            for axis, value in enumerate(xyz):
                self._finite_float(value, f"MuJoCo raycast hits[{index}].xyz_sensor[{axis}]")
            for vector_field in (
                "origin_world_m",
                "direction_world",
                "position_world_m",
            ):
                vector = hit.get(vector_field)
                if type(vector) is not list or len(vector) != 3:
                    raise CoordinatorError(
                        f"MuJoCo raycast hits[{index}].{vector_field} must contain 3 values"
                    )
                for axis, value in enumerate(vector):
                    self._finite_float(
                        value,
                        f"MuJoCo raycast hits[{index}].{vector_field}[{axis}]",
                    )
            distance = self._finite_float(
                hit.get("distance_m"),
                f"MuJoCo raycast hits[{index}].distance_m",
            )
            if distance < 0.0:
                raise CoordinatorError(
                    f"MuJoCo raycast hits[{index}].distance_m must be non-negative"
                )
            for field in ("body_stable_id", "entity_id"):
                value = hit.get(field)
                if not isinstance(value, str) or not value:
                    raise CoordinatorError(
                        f"MuJoCo raycast hits[{index}].{field} must be non-empty text"
                    )
            self._uint_range(
                hit.get("offset_time_ns"),
                f"MuJoCo raycast hits[{index}].offset_time_ns",
                minimum=0,
                maximum=0xFFFFFFFF,
            )
            for field in ("reflectivity", "tag", "line"):
                self._uint_range(
                    hit.get(field),
                    f"MuJoCo raycast hits[{index}].{field}",
                    minimum=0,
                    maximum=255,
                )

    @staticmethod
    def _protocol_token(value: str, field: str, *, maximum_bytes: int) -> str:
        if (
            not isinstance(value, str)
            or not value
            or value != value.strip()
            or any(character.isspace() for character in value)
        ):
            raise CoordinatorError(f"{field} must be one non-empty protocol token")
        if len(value.encode("utf-8")) > maximum_bytes:
            raise CoordinatorError(f"{field} exceeds {maximum_bytes} UTF-8 bytes")
        return value

    @staticmethod
    def _session_id(value: object, field: str) -> str:
        if (
            not isinstance(value, str)
            or not value
            or value != value.strip()
            or any(character.isspace() for character in value)
        ):
            raise CoordinatorError(f"{field} must be one non-empty protocol token")
        if len(value.encode("utf-8")) > 63:
            raise CoordinatorError(f"{field} exceeds 63 UTF-8 bytes")
        return value

    def _read_stdout(self, stream: TextIO) -> None:
        try:
            for line in stream:
                self._events.put(line)
        finally:
            self._events.put(None)

    def _read_event(self) -> dict[str, Any]:
        try:
            line = self._events.get(timeout=self._timeout_s)
        except queue.Empty as exc:
            self._terminate()
            raise CoordinatorError("MuJoCo host response timed out") from exc
        if line is None:
            code = self._process.poll() if self._process is not None else None
            raise CoordinatorError(f"MuJoCo host closed its output (exit={code})")
        try:
            value = json.loads(
                line,
                parse_constant=lambda value: (_ for _ in ()).throw(
                    CoordinatorError(
                        f"MuJoCo host emitted non-finite JSON value {value}"
                    )
                ),
            )
        except CoordinatorError:
            raise
        except (json.JSONDecodeError, UnicodeError) as exc:
            raise CoordinatorError("MuJoCo host emitted invalid JSON") from exc
        if type(value) is not dict:
            raise CoordinatorError("MuJoCo host event must be a JSON object")
        if value.get("event") == "error":
            raise CoordinatorError(
                f"MuJoCo host rejected a command: {value.get('message', 'unknown error')}"
            )
        return value

    def _terminate(self) -> None:
        process = self._process
        natural = process is not None and process.poll() is not None
        termination_error: BaseException | None = None
        try:
            if self._process is not None:
                if self._process_owner is not None:
                    self._process_owner.terminate(self._process, timeout_s=2.0)
                elif self._process.poll() is None:
                    self._process.terminate()
                    try:
                        self._process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        self._process.kill()
                        self._process.wait(timeout=2.0)
            elif self._process_owner is not None:
                self._process_owner.close()
        except BaseException as exc:
            termination_error = exc
        try:
            self._close(
                termination_mode="natural" if natural else "owned_terminate"
            )
        except BaseException as close_error:
            if termination_error is None:
                raise
            _add_exception_note(
                termination_error,
                f"MuJoCo process close also failed: {close_error}",
            )
        if termination_error is not None:
            raise termination_error

    def _close(
        self,
        *,
        termination_mode: Literal["natural", "owned_terminate"],
    ) -> None:
        process = self._process
        pid = process.pid if process is not None else None
        cleanup_errors: list[BaseException] = []
        if process is not None:
            if process.stdin is not None:
                try:
                    process.stdin.close()
                except BaseException as exc:
                    cleanup_errors.append(exc)
            if process.stdout is not None:
                try:
                    process.stdout.close()
                except BaseException as exc:
                    cleanup_errors.append(exc)
        if self._reader is not None:
            try:
                self._reader.join(timeout=1.0)
            except BaseException as exc:
                cleanup_errors.append(exc)
        if self._stderr is not None:
            try:
                self._stderr.close()
            except BaseException as exc:
                cleanup_errors.append(exc)
        owner_closed = False
        owner_error: BaseException | None = None
        if self._process_owner is not None:
            try:
                self._process_owner.close_after_exit()
                owner_closed = True
            except BaseException as exc:
                owner_error = exc
        exit_code = process.poll() if process is not None else None
        if pid is not None:
            self._last_shutdown = ProcessShutdownSnapshot(
                pid=pid,
                exit_code=exit_code,
                direct_child_running_after_close=exit_code is None,
                process_owner_closed=owner_closed,
                termination_mode=termination_mode,
            )
        self._process = None
        self._process_owner = None
        self._reader = None
        self._stderr = None
        self._actuator_bindings.clear()
        if owner_error is not None:
            cleanup_errors.append(owner_error)
        if cleanup_errors:
            error = cleanup_errors[0]
            for extra in cleanup_errors[1:]:
                _add_exception_note(error, f"additional MuJoCo close failure: {extra}")
            raise error


def _add_exception_note(error: BaseException, note: str) -> None:
    add_note = getattr(error, "add_note", None)
    if callable(add_note):
        add_note(note)
        return

    # BaseException.add_note was added in Python 3.11.  Preserve the same
    # diagnostic evidence for the Python 3.10 host used by LingTu.
    notes = list(getattr(error, "__notes__", ()))
    notes.append(note)
    try:
        error.__notes__ = notes
    except (AttributeError, TypeError):
        pass
