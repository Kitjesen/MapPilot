"""ROS-free MuJoCo acceptance for one deterministic lift building mission.

This gate isolates the building-transition function from global/local planner
acceptance. It loads the configured Thunder robot model and a real MuJoCo lift
scene, then drives the building state machine through approach, entry, ride,
floor switch, exit, and the final same-floor goal. Robot waypoint motion and
cabin rider coupling are deliberately kinematic, so the report never claims
physical gait, SLAM, or saved-map relocalization evidence.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from drivers.sim.mujoco.lift import MuJoCoLiftCommandAdapter, MuJoCoLiftConfig
from drivers.sim.mujoco.runtime import build_engine, resolve_world
from nav.building import (
    ActiveFloor,
    BuildingMissionOrchestrator,
    BuildingMissionPhase,
    BuildingMissionRequest,
    GoalProgress,
    LiftTransitionExecutor,
    LiftTransitionPhase,
    LiftTransitionPlan,
    LiftTransitionService,
    PoseTarget,
    StaticLiftTransitionCatalog,
)

SOURCE_FLOOR = ActiveFloor("factory-a", "floor-1", "factory-a-floor-1")
TARGET_FLOOR = ActiveFloor("factory-a", "floor-2", "factory-a-floor-2")
FLOOR_WORLD_Z = {"floor-1": 0.0, "floor-2": 3.5}
FLOORS = (SOURCE_FLOOR, TARGET_FLOOR)


class _SimClock:
    def __init__(self, start_s: float = 100.0) -> None:
        self.now_s = float(start_s)

    def __call__(self) -> float:
        return self.now_s

    def advance(self, dt_s: float) -> None:
        self.now_s += float(dt_s)


class _MuJoCoFloorLocalization:
    """Simulation-only floor identity and relocalization evidence adapter."""

    def __init__(
        self,
        engine: Any,
        lift: MuJoCoLiftCommandAdapter,
        *,
        initial_floor: ActiveFloor,
        known_floors: tuple[ActiveFloor, ...] = FLOORS,
    ) -> None:
        self._engine = engine
        self._lift = lift
        self._known_floors = frozenset(known_floors)
        if initial_floor not in self._known_floors:
            raise ValueError("initial floor must be present in known_floors")
        self._active = initial_floor
        self._localized = initial_floor
        self.switch_count = 0

    def active_floor(self) -> ActiveFloor:
        """Return the simulator's explicit active floor identity."""

        return self._active

    def switch_and_relocalize(
        self,
        floor: ActiveFloor,
        seed: PoseTarget,
    ) -> tuple[bool, str]:
        """Verify target-floor MuJoCo geometry before switching map identity."""

        if floor not in self._known_floors or seed.frame_id != "map":
            return False, "sim_floor_binding_unknown"
        lift_state = self._lift.snapshot("lift-a")
        if (
            lift_state is None
            or lift_state.current_floor_id != floor.floor_id
            or lift_state.motion_state != "stopped"
            or lift_state.door_state != "open"
        ):
            return False, "sim_lift_target_state_unverified"
        position = np.asarray(self._engine.get_robot_state().position, dtype=float)
        expected = np.asarray(
            [seed.x, seed.y, FLOOR_WORLD_Z[floor.floor_id] + seed.z],
            dtype=float,
        )
        if not np.isfinite(position).all() or float(np.linalg.norm(position - expected)) > 0.12:
            return False, "sim_target_cabin_pose_unverified"
        self._active = floor
        self._localized = floor
        self.switch_count += 1
        return True, "sim_floor_relocalization_verified"

    def is_localized(self, floor: ActiveFloor) -> bool:
        """Require the exact floor identity and a finite MuJoCo robot pose."""

        position = np.asarray(self._engine.get_robot_state().position, dtype=float)
        expected_z = FLOOR_WORLD_Z[floor.floor_id] + 0.48
        return (
            floor == self._active == self._localized
            and np.isfinite(position).all()
            and abs(float(position[2]) - expected_z) <= 0.12
        )


class _KinematicMuJoCoNavigation:
    """Request-correlated map goals backed by visible MuJoCo robot poses."""

    def __init__(
        self,
        engine: Any,
        floors: _MuJoCoFloorLocalization,
        *,
        linear_speed_mps: float = 1.2,
        yaw_speed_radps: float = 1.5,
        goal_tolerance_m: float = 0.06,
    ) -> None:
        self._engine = engine
        self._floors = floors
        self._linear_speed_mps = float(linear_speed_mps)
        self._yaw_speed_radps = float(yaw_speed_radps)
        self._goal_tolerance_m = float(goal_tolerance_m)
        self._request_id = ""
        self._map_id = ""
        self._target: PoseTarget | None = None
        self._mode = "autonomy"
        self.goal_request_ids: list[str] = []
        self.cancel_request_ids: list[str] = []
        self.stop_request_ids: list[str] = []
        self.rider_coupled_distance_m = 0.0

    def autonomy_ready(self) -> tuple[bool, str]:
        """Expose the exact mutually exclusive native-mode contract."""

        if self._mode != "autonomy":
            return False, f"native_control_mode_{self._mode}"
        return True, "autonomy_ready"

    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        request_id: str | None = None,
    ) -> None:
        """Start one immutable, correlated map-frame kinematic goal."""

        goal_id = str(request_id or "").strip()
        values = (float(x), float(y), float(z), float(yaw))
        if not goal_id or not all(math.isfinite(value) for value in values):
            raise ValueError("finite target and request_id are required")
        self._request_id = goal_id
        self._map_id = self._floors.active_floor().map_id
        self._target = PoseTarget("map", *values)
        self.goal_request_ids.append(goal_id)

    def observe_goal(
        self,
        *,
        request_id: str,
        map_id: str,
        target: PoseTarget,
    ) -> GoalProgress:
        """Accept completion only for the exact active request/map/target."""

        if (
            self._target is None
            or request_id != self._request_id
            or map_id != self._map_id
            or map_id != self._floors.active_floor().map_id
            or target != self._target
        ):
            return GoalProgress.BLOCKED
        position = np.asarray(self._engine.get_robot_state().position, dtype=float)
        distance = float(np.linalg.norm(position - self._world_target(self._target)))
        return GoalProgress.SUCCEEDED if distance <= self._goal_tolerance_m else GoalProgress.EXECUTING

    def cancel(self, reason: str = "cancel", *, request_id: str | None = None) -> None:
        """Clear active kinematic motion and record the correlated cancel."""

        del reason
        self.cancel_request_ids.append(str(request_id or ""))
        self._target = None

    def stop(self, reason: str = "stop", *, request_id: str | None = None) -> None:
        """Clear active kinematic motion without changing control mode."""

        del reason
        self.stop_request_ids.append(str(request_id or ""))
        self._target = None

    def step(self, dt_s: float) -> None:
        """Advance the visible robot pose toward its active map-frame goal."""

        if self._target is None:
            return
        state = self._engine.get_robot_state()
        position = np.asarray(state.position, dtype=float)
        destination = self._world_target(self._target)
        delta = destination - position
        distance = float(np.linalg.norm(delta))
        if distance > 0.0:
            max_step = self._linear_speed_mps * float(dt_s)
            position = destination if distance <= max_step else position + delta * (max_step / distance)
        yaw = self._yaw_from_xyzw(state.orientation)
        yaw_error = self._wrap_angle(self._target.yaw - yaw)
        yaw_step = min(abs(yaw_error), self._yaw_speed_radps * float(dt_s))
        yaw += math.copysign(yaw_step, yaw_error) if yaw_step else 0.0
        self._set_pose(position, yaw)

    def couple_rider(self, cabin_delta_z: float) -> bool:
        """Move an in-cabin robot with the kinematic lift platform."""

        if abs(float(cabin_delta_z)) <= 1e-12:
            return False
        state = self._engine.get_robot_state()
        position = np.asarray(state.position, dtype=float)
        if abs(float(position[0]) - 6.0) > 0.85 or abs(float(position[1]) - 6.0) > 0.85:
            return False
        position[2] += float(cabin_delta_z)
        self._set_pose(position, self._yaw_from_xyzw(state.orientation))
        self.rider_coupled_distance_m += abs(float(cabin_delta_z))
        return True

    def _world_target(self, target: PoseTarget) -> np.ndarray:
        floor = self._floors.active_floor()
        return np.asarray(
            [target.x, target.y, FLOOR_WORLD_Z[floor.floor_id] + target.z],
            dtype=float,
        )

    def _set_pose(self, position: np.ndarray, yaw: float) -> None:
        orientation_xyzw = np.asarray(
            [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)],
            dtype=float,
        )
        self._engine.set_robot_pose(np.asarray(position, dtype=float), orientation_xyzw)

    @staticmethod
    def _yaw_from_xyzw(orientation: Any) -> float:
        x, y, z, w = (float(value) for value in orientation)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _joint_qpos(engine: Any, joint_name: str) -> float:
    import mujoco

    joint_id = mujoco.mj_name2id(engine._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise RuntimeError(f"MuJoCo joint missing: {joint_name}")
    return float(engine._data.qpos[int(engine._model.jnt_qposadr[joint_id])])


def _set_joint_qpos(engine: Any, joint_name: str, value: float) -> None:
    import mujoco

    joint_id = mujoco.mj_name2id(engine._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise RuntimeError(f"MuJoCo joint missing: {joint_name}")
    qpos_addr = int(engine._model.jnt_qposadr[joint_id])
    dof_addr = int(engine._model.jnt_dofadr[joint_id])
    with engine._data_lock:
        engine._data.qpos[qpos_addr] = float(value)
        if 0 <= dof_addr < len(engine._data.qvel):
            engine._data.qvel[dof_addr] = 0.0
        mujoco.mj_forward(engine._model, engine._data)


def _thunder_model_loaded(engine: Any) -> bool:
    import mujoco

    return (
        mujoco.mj_name2id(engine._model, mujoco.mjtObj.mjOBJ_BODY, "base_link") >= 0
        and mujoco.mj_name2id(engine._model, mujoco.mjtObj.mjOBJ_BODY, "lift_cabin") >= 0
    )


def _render_video(
    engine: Any,
    snapshots: list[dict[str, Any]],
    output: Path,
    *,
    fps: float,
    width: int = 960,
    height: int = 720,
) -> dict[str, Any]:
    """Render balanced-brightness phase evidence from captured MuJoCo states."""

    import cv2

    import mujoco

    if not snapshots:
        return {"requested": True, "ok": False, "reason": "no_snapshots"}
    renderer = None
    writer = None
    try:
        engine._model.vis.global_.offwidth = max(engine._model.vis.global_.offwidth, width)
        engine._model.vis.global_.offheight = max(engine._model.vis.global_.offheight, height)
        renderer = mujoco.Renderer(engine._model, height=height, width=width)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = [6.0, 5.1, 2.0]
        camera.distance = 13.5
        camera.elevation = -27.0
        camera.azimuth = 142.0
        scene_option = mujoco.MjvOption()
        scene_option.geomgroup[:] = 1
        scene_option.geomgroup[3] = 0
        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(output),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(fps),
            (int(width), int(height)),
        )
        if not writer.isOpened():
            raise RuntimeError("video_writer_unavailable")

        for snapshot in snapshots:
            qpos = np.asarray(snapshot["qpos"], dtype=np.float64)
            engine._data.qpos[: len(qpos)] = qpos
            engine._data.qvel[:] = 0.0
            mujoco.mj_forward(engine._model, engine._data)
            scene_option.geomgroup[4] = 0 if snapshot["floor_id"] == "floor-1" else 1
            renderer.update_scene(engine._data, camera, scene_option=scene_option)
            frame = cv2.cvtColor(renderer.render().copy(), cv2.COLOR_RGB2BGR)
            frame = np.clip(frame.astype(np.float32) * 0.78, 0.0, 255.0).astype(np.uint8)
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (width, 104), (22, 25, 29), thickness=-1)
            frame = cv2.addWeighted(overlay, 0.78, frame, 0.22, 0.0)
            cv2.putText(
                frame,
                "LingTu ROS-free MuJoCo lift acceptance",
                (24, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.76,
                (235, 240, 245),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                (
                    f"mission={snapshot['mission_phase']}  lift={snapshot['lift_phase']}  "
                    f"door={snapshot['door_state']}  floor={snapshot['floor_id']}"
                ),
                (24, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (80, 220, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"sim t={snapshot['sim_time_s']:.1f}s  kinematic rider coupling; gait/SLAM not claimed",
                (24, 94),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.49,
                (190, 198, 205),
                1,
                cv2.LINE_AA,
            )
            writer.write(frame)
        return {
            "requested": True,
            "ok": True,
            "reason": "rendered",
            "path": str(output),
            "frames": len(snapshots),
            "fps": float(fps),
            "width": int(width),
            "height": int(height),
            "brightness_profile": "balanced_scene_lighting_with_dark_telemetry_banner",
            "presentation_hidden_groups": ["shaft_guards"],
            "conditional_hidden_groups": {"floor-1": ["floor_2"]},
        }
    except Exception as exc:
        return {
            "requested": True,
            "ok": False,
            "reason": f"{type(exc).__name__}:{exc}",
            "path": str(output),
        }
    finally:
        if writer is not None:
            writer.release()
        if renderer is not None:
            renderer.close()


def _transition_plan(
    source_floor: ActiveFloor = SOURCE_FLOOR,
    target_floor: ActiveFloor = TARGET_FLOOR,
) -> LiftTransitionPlan:
    return LiftTransitionPlan(
        lift_id="lift-a",
        source_floor=source_floor,
        target_floor=target_floor,
        source_lobby=PoseTarget("map", 6.0, 4.2, 0.48, math.pi / 2.0),
        source_cabin=PoseTarget("map", 6.0, 6.0, 0.48, math.pi / 2.0),
        target_cabin=PoseTarget("map", 6.0, 6.0, 0.48, -math.pi / 2.0),
        target_lobby=PoseTarget("map", 6.0, 4.2, 0.48, -math.pi / 2.0),
    )


def _mission_request(
    target_floor: ActiveFloor = TARGET_FLOOR,
    *,
    request_id: str = "mujoco-lift-acceptance",
) -> BuildingMissionRequest:
    return BuildingMissionRequest(
        request_id=request_id,
        source="mujoco_acceptance",
        fleet_name="",
        robot_name="thunder-sim",
        building_id=target_floor.building_id,
        floor_id=target_floor.floor_id,
        map_id=target_floor.map_id,
        target=PoseTarget("map", 2.5, 3.2, 0.48, math.pi),
    )


def run_lift_transition_acceptance(
    *,
    output_dir: str | Path,
    direction: str = "up",
    step_s: float = 0.10,
    timeout_s: float = 45.0,
    record_video: bool = False,
    video_fps: float = 10.0,
) -> dict[str, Any]:
    """Run one complete ROS-free MuJoCo lift mission and persist evidence."""

    normalized_direction = str(direction).strip().lower()
    if normalized_direction == "up":
        source_floor, target_floor = SOURCE_FLOOR, TARGET_FLOOR
        request_id = "mujoco-lift-acceptance"
    elif normalized_direction == "down":
        source_floor, target_floor = TARGET_FLOOR, SOURCE_FLOOR
        request_id = "mujoco-lift-acceptance-down"
    else:
        raise ValueError("direction must be 'up' or 'down'")

    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    clock = _SimClock()
    engine = build_engine(
        world=resolve_world("lift_building"),
        drive_mode="kinematic",
        n_rays=16,
        start=[6.0, 2.6, FLOOR_WORLD_Z[source_floor.floor_id] + 0.48],
        mujoco_memory="",
        mid360_pattern=None,
        lidar_backend="mj_multiray",
        require_product_lidar_backend=False,
        allow_legacy_lidar_fallback=True,
    )
    timeline: list[dict[str, Any]] = []
    mission_phases: list[str] = []
    lift_phases: list[str] = []
    cabin_positions: list[float] = []
    video_snapshots: list[dict[str, Any]] = []
    try:
        _set_joint_qpos(engine, "lift_cabin_z", FLOOR_WORLD_Z[source_floor.floor_id])
        lift = MuJoCoLiftCommandAdapter(
            engine,
            config=MuJoCoLiftConfig(cabin_speed_mps=1.0, door_speed_mps=1.0),
            clock=clock,
        )
        floors = _MuJoCoFloorLocalization(
            engine,
            lift,
            initial_floor=source_floor,
        )
        navigation = _KinematicMuJoCoNavigation(engine, floors)
        transition_executor = LiftTransitionExecutor(
            navigation=navigation,
            lift=lift,
            floor_localization=floors,
            clock=clock,
            max_lift_state_age_s=0.5,
            step_timeout_s=15.0,
            ride_timeout_s=20.0,
        )
        transition = LiftTransitionService(
            catalog=StaticLiftTransitionCatalog([_transition_plan(source_floor, target_floor)]),
            executor=transition_executor,
        )
        orchestrator = BuildingMissionOrchestrator(
            navigation=navigation,
            active_floor=floors.active_floor,
            transition_executor=transition,
        )
        mission_request = _mission_request(target_floor, request_id=request_id)
        accepted, admission_reason = orchestrator.submit(mission_request)
        if not accepted:
            raise RuntimeError(f"building mission rejected: {admission_reason}")

        previous_signature: tuple[str, str, str, str] | None = None
        max_steps = max(1, int(math.ceil(float(timeout_s) / float(step_s))))
        for _ in range(max_steps):
            navigation.step(step_s)
            cabin_before = _joint_qpos(engine, "lift_cabin_z")
            lift.step(step_s)
            cabin_after = _joint_qpos(engine, "lift_cabin_z")
            navigation.couple_rider(cabin_after - cabin_before)
            clock.advance(step_s)
            mission_status = orchestrator.tick()
            lift_status = transition_executor.status()
            lift_state = lift.snapshot("lift-a")
            robot_position = [float(value) for value in engine.get_robot_state().position]
            cabin_positions.append(cabin_after)
            mission_phases.append(mission_status.phase.value)
            lift_phases.append(lift_status.phase.value)
            signature = (
                mission_status.phase.value,
                lift_status.phase.value,
                lift_state.door_state if lift_state is not None else "missing",
                lift_state.motion_state if lift_state is not None else "missing",
            )
            if signature != previous_signature:
                timeline.append(
                    {
                        "sim_time_s": round(clock(), 3),
                        "mission_phase": signature[0],
                        "lift_phase": signature[1],
                        "door_state": signature[2],
                        "motion_state": signature[3],
                        "active_floor": asdict(floors.active_floor()),
                        "robot_position": robot_position,
                        "cabin_z": cabin_after,
                    }
                )
                previous_signature = signature
            if record_video:
                video_snapshots.append(
                    {
                        "qpos": np.asarray(engine._data.qpos, dtype=float).copy(),
                        "sim_time_s": clock() - 100.0,
                        "mission_phase": mission_status.phase.value,
                        "lift_phase": lift_status.phase.value,
                        "door_state": lift_state.door_state if lift_state is not None else "missing",
                        "floor_id": floors.active_floor().floor_id,
                        "robot_position": robot_position,
                    }
                )
            if mission_status.phase in {
                BuildingMissionPhase.SUCCEEDED,
                BuildingMissionPhase.FAILED,
                BuildingMissionPhase.CANCELLED,
            }:
                break

        final_mission = orchestrator.status()
        final_lift = transition_executor.status()
        lift_state = lift.snapshot("lift-a")
        final_position = [float(value) for value in engine.get_robot_state().position]
        final_target_world = np.asarray(
            [
                mission_request.target.x,
                mission_request.target.y,
                FLOOR_WORLD_Z[target_floor.floor_id] + mission_request.target.z,
            ],
            dtype=float,
        )
        reached_target = float(
            np.linalg.norm(
                np.asarray(final_position, dtype=float)
                - final_target_world
            )
        ) <= 0.08
        expected_cabin_z = FLOOR_WORLD_Z[target_floor.floor_id]
        lift_motion_verified = (
            bool(cabin_positions)
            and max(cabin_positions) - min(cabin_positions) >= 3.49
            and abs(cabin_positions[-1] - expected_cabin_z) <= 1e-6
        )
        lift_released = lift_state is not None and lift_state.session_id == ""
        core_passed = (
            final_mission.phase is BuildingMissionPhase.SUCCEEDED
            and final_lift.phase is LiftTransitionPhase.SUCCEEDED
            and floors.active_floor() == target_floor
            and floors.switch_count == 1
            and lift_motion_verified
            and lift_released
            and reached_target
        )
        video = (
            _render_video(
                engine,
                video_snapshots,
                output / "lift_transition.mp4",
                fps=video_fps,
            )
            if record_video
            else {"requested": False, "ok": True, "reason": "not_requested"}
        )
        passed = core_passed and video["ok"]
        report: dict[str, Any] = {
            "schema_version": "lingtu.mujoco_lift_transition_acceptance.v1",
            "passed": passed,
            "direction": normalized_direction,
            "source_floor": asdict(source_floor),
            "target_floor": asdict(target_floor),
            "ros_required": False,
            "open_rmf_required": False,
            "native_dds_contract_preserved": True,
            "native_dds_transport_exercised": False,
            "thunder_robot_model_loaded": _thunder_model_loaded(engine),
            "mujoco_lift_joint_motion_verified": lift_motion_verified,
            "kinematic_rider_coupling": navigation.rider_coupled_distance_m >= 3.49,
            "physical_gait_verified": False,
            "global_planner_verified": False,
            "local_planner_verified": False,
            "real_localization_verified": False,
            "native_saved_map_relocalization_verified": False,
            "admission_reason": admission_reason,
            "final_mission_phase": final_mission.phase.value,
            "final_mission_reason": final_mission.reason,
            "final_lift_phase": final_lift.phase.value,
            "final_lift_reason": final_lift.reason,
            "final_floor": asdict(floors.active_floor()),
            "final_lift_floor_id": lift_state.current_floor_id if lift_state is not None else "",
            "final_robot_position": final_position,
            "goal_reached": reached_target,
            "lift_released": lift_released,
            "floor_switch_count": floors.switch_count,
            "goal_request_ids": navigation.goal_request_ids,
            "cancel_request_ids": navigation.cancel_request_ids,
            "stop_request_ids": navigation.stop_request_ids,
            "rider_coupled_distance_m": navigation.rider_coupled_distance_m,
            "mission_phases_seen": list(dict.fromkeys(mission_phases)),
            "lift_phases_seen": list(dict.fromkeys(lift_phases)),
            "timeline": timeline,
            "simulated_duration_s": round(clock() - 100.0, 3),
            "step_s": float(step_s),
            "timeout_s": float(timeout_s),
            "scene": str(resolve_world("lift_building")),
            "video": video,
        }
    finally:
        engine.close()

    (output / "report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return report


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="artifacts/mujoco_lift_transition_acceptance")
    parser.add_argument("--direction", choices=("up", "down"), default="up")
    parser.add_argument("--step-s", type=float, default=0.10)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--record-video", action="store_true")
    parser.add_argument("--video-fps", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    """Run the CLI acceptance and return a process exit code."""

    args = _parse_args()
    report = run_lift_transition_acceptance(
        output_dir=args.output_dir,
        direction=args.direction,
        step_s=args.step_s,
        timeout_s=args.timeout_s,
        record_video=args.record_video,
        video_fps=args.video_fps,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
