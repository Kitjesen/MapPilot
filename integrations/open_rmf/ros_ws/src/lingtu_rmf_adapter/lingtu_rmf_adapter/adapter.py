"""ROS 2 Jazzy Open-RMF sidecar for one LingTu robot."""

from __future__ import annotations

import argparse
import faulthandler
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


def _install_lingtu_source_path() -> Path:
    configured = os.environ.get("LINGTU_ROOT", "").strip()
    candidates = []
    if configured:
        candidates.append(Path(configured))
    candidates.append(Path(__file__).resolve().parents[6])
    for root in candidates:
        source = root / "src"
        if (source / "nav" / "adapters" / "open_rmf").is_dir():
            source_text = str(source)
            if source_text not in sys.path:
                sys.path.insert(0, source_text)
            return root
    raise RuntimeError("LingTu source was not found; set LINGTU_ROOT to the repository root")


_LINGTU_ROOT = _install_lingtu_source_path()

from nav.adapters.open_rmf import (
    FloorBinding,
    GatewayClientConfig,
    GatewayHttpTransport,
    GatewayMissionPort,
    GatewayRobotStateSource,
    NativeGoalCompletionGate,
    PoseTarget,
    RmfDestination,
    SingleRobotRmfBridge,
)


def _enabled(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class BridgeSettings:
    """Validated settings for one LingTu robot and RMF fleet."""

    fleet_name: str
    robot_name: str
    gateway: GatewayClientConfig
    floor_bindings: dict[str, FloorBinding]
    poll_period_s: float
    goal_tolerance_m: float


def _mapping(value: Any, *, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a mapping")
    return value


def load_bridge_settings(path: str | Path) -> BridgeSettings:
    """Load the ROS-free LingTu sidecar configuration."""

    with Path(path).open("r", encoding="utf-8") as stream:
        document = _mapping(
            yaml.safe_load(stream) or {},
            field_name="bridge config",
        )
    gateway = _mapping(document.get("gateway"), field_name="gateway")
    robot = _mapping(document.get("robot"), field_name="robot")
    levels = _mapping(document.get("levels"), field_name="levels")

    api_key_env = str(gateway.get("api_key_env") or "LINGTU_RMF_API_KEY")
    command_env = str(gateway.get("commands_enabled_env") or "LINGTU_RMF_COMMANDS_ENABLED")
    floor_bindings: dict[str, FloorBinding] = {}
    for level_name, raw_binding in levels.items():
        binding = _mapping(
            raw_binding,
            field_name=f"levels.{level_name}",
        )
        floor_bindings[str(level_name)] = FloorBinding(
            building_id=str(binding["building_id"]),
            floor_id=str(binding["floor_id"]),
            map_id=str(binding["map_id"]),
            frame_id=str(binding.get("frame_id") or "map"),
        )

    fleet_name = str(robot["fleet_name"]).strip()
    robot_name = str(robot["robot_name"]).strip()
    return BridgeSettings(
        fleet_name=fleet_name,
        robot_name=robot_name,
        gateway=GatewayClientConfig(
            base_url=str(gateway["base_url"]),
            api_key=os.environ.get(api_key_env, ""),
            client_id=f"rmf:{fleet_name}:{robot_name}",
            commands_enabled=_enabled(os.environ.get(command_env)),
            lease_ttl_s=float(gateway.get("lease_ttl_s", 30.0)),
            timeout_s=float(gateway.get("timeout_s", 3.0)),
            battery_soc_default=float(robot.get("battery_soc_default", 1.0)),
            max_pose_age_s=float(gateway.get("max_pose_age_s", 2.0)),
            max_native_status_age_s=float(gateway.get("max_native_status_age_s", 2.0)),
            allow_insecure_http=bool(gateway.get("allow_insecure_http", False)),
        ),
        floor_bindings=floor_bindings,
        poll_period_s=max(0.1, float(document.get("poll_period_s", 0.5))),
        goal_tolerance_m=max(
            0.01,
            float(document.get("goal_tolerance_m", 0.10)),
        ),
    )


def _fleet_dispatch_enabled(path: str | Path) -> bool:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = _mapping(yaml.safe_load(stream) or {}, field_name="fleet config")
    fleet = _mapping(document.get("rmf_fleet"), field_name="rmf_fleet")
    capabilities = _mapping(
        fleet.get("task_capabilities") or {},
        field_name="rmf_fleet.task_capabilities",
    )
    actions = fleet.get("actions") or []
    return any(bool(value) for value in capabilities.values()) or bool(actions)


def _fleet_transform_levels(path: str | Path) -> frozenset[str]:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = _mapping(yaml.safe_load(stream) or {}, field_name="fleet config")
    fleet = _mapping(document.get("rmf_fleet"), field_name="rmf_fleet")
    raw_transforms = fleet.get("transforms")
    if raw_transforms is None:
        return frozenset()
    transforms = _mapping(raw_transforms, field_name="rmf_fleet.transforms")
    validated: set[str] = set()
    for level, raw_transform in transforms.items():
        field_name = f"rmf_fleet.transforms.{level}"
        transform = _mapping(raw_transform, field_name=field_name)
        missing = {"translation", "rotation", "scale"}.difference(transform)
        if missing:
            raise ValueError(f"{field_name} is missing: {', '.join(sorted(missing))}")
        translation = transform["translation"]
        if not isinstance(translation, (list, tuple)) or len(translation) != 2:
            raise ValueError(f"{field_name}.translation must contain exactly two numbers")
        try:
            translation_values = tuple(float(value) for value in translation)
            rotation = float(transform["rotation"])
            scale = float(transform["scale"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name} must contain numeric transform values") from exc
        if not all(math.isfinite(value) for value in (*translation_values, rotation, scale)):
            raise ValueError(f"{field_name} must contain finite transform values")
        if scale <= 0.0:
            raise ValueError(f"{field_name}.scale must be positive")
        validated.add(str(level))
    return frozenset(validated)


def validate_dispatch_mode(
    settings: BridgeSettings,
    fleet_config_path: str | Path,
) -> None:
    """Validate shadow/live dispatch and per-level coordinate safety gates."""

    if not settings.gateway.commands_enabled and _fleet_dispatch_enabled(fleet_config_path):
        raise RuntimeError(
            "RMF task dispatch is enabled while LingTu commands are in shadow mode; use lingtu_fleet_shadow.yaml"
        )

    if settings.gateway.commands_enabled:
        missing = sorted(set(settings.floor_bindings).difference(_fleet_transform_levels(fleet_config_path)))
        if missing:
            raise RuntimeError(
                "Live RMF commands require an explicit coordinate transform for every level; missing: "
                + ", ".join(missing)
            )


class LingTuRobotAdapter:
    """Own one EasyFullControl robot without exposing velocity control."""

    def __init__(
        self,
        *,
        node: Any,
        rmf_easy: Any,
        fleet_handle: Any,
        robot_configuration: Any,
        settings: BridgeSettings,
    ) -> None:
        self._node = node
        self._rmf_easy = rmf_easy
        self._fleet_handle = fleet_handle
        self._robot_configuration = robot_configuration
        self._settings = settings
        self._transport = GatewayHttpTransport(
            base_url=settings.gateway.base_url,
            timeout_s=settings.gateway.timeout_s,
            allow_insecure_http=settings.gateway.allow_insecure_http,
        )
        self._mission_port = GatewayMissionPort(
            config=settings.gateway,
            transport=self._transport,
        )
        self._state_source = GatewayRobotStateSource(
            config=settings.gateway,
            transport=self._transport,
            floor_bindings=settings.floor_bindings,
        )
        self._bridge = SingleRobotRmfBridge(
            fleet_name=settings.fleet_name,
            robot_name=settings.robot_name,
            mission_port=self._mission_port,
            floor_bindings=settings.floor_bindings,
        )
        self._lock = threading.RLock()
        self._update_handle = None
        self._execution = None
        self._request_id = ""
        self._goal_submitted = False
        self._completion_gate = None
        self._release_pending = False
        self._command_sequence = 0
        self._renewal_sequence = 0
        self._last_renew_monotonic = 0.0

    def callbacks(self):
        """Build the EasyFullControl callbacks for this robot."""

        return self._rmf_easy.RobotCallbacks(
            self.navigate,
            self.stop,
            self.execute_action,
        )

    def _next_request_id(self) -> str:
        self._command_sequence += 1
        return f"rmf:{self._settings.robot_name}:{self._command_sequence}:{time.time_ns()}"

    def navigate(self, destination, execution) -> None:
        """Translate one RMF destination into a LingTu building mission."""

        with self._lock:
            if not self._cancel_active("rmf_command_replaced"):
                self._node.get_logger().error("LingTu could not cancel the previous RMF command; rejecting replacement")
                if not self._release_pending:
                    self._reassign_tasks()
                return
            position = [float(value) for value in destination.position]
            target = PoseTarget(
                frame_id="map",
                x=position[0],
                y=position[1],
                z=0.0,
                yaw=position[2],
            )
            request_id = self._next_request_id()
            self._execution = execution
            self._request_id = request_id
            result = self._bridge.navigate(
                RmfDestination(
                    map_name=str(destination.map),
                    x=target.x,
                    y=target.y,
                    yaw=target.yaw,
                ),
                request_id=request_id,
            )
            if not result.accepted:
                release_pending = result.reason == "gateway_goal_rejected_release_pending"
                cancel_pending = result.reason == "gateway_goal_outcome_unknown_cancel_pending"
                self._goal_submitted = cancel_pending
                self._release_pending = release_pending
                self._node.get_logger().error(f"LingTu rejected RMF navigation [{request_id}]: {result.reason}")
                if release_pending:
                    return
                self._reassign_tasks()
                if not cancel_pending:
                    self._clear_active()
                return
            binding = self._settings.floor_bindings[str(destination.map)]
            self._goal_submitted = True
            self._completion_gate = NativeGoalCompletionGate(
                request_id=request_id,
                map_id=binding.map_id,
                target=target,
                goal_tolerance_m=self._settings.goal_tolerance_m,
            )
            self._renewal_sequence = 0
            self._last_renew_monotonic = time.monotonic()
            self._node.get_logger().info(
                f"LingTu accepted RMF navigation [{request_id}] to {destination.map} {position}"
            )

    def stop(self, activity) -> None:
        """Cancel the matching active LingTu navigation command."""

        with self._lock:
            if self._execution is None:
                return
            identifier = getattr(self._execution, "identifier", None)
            if identifier is None or not identifier.is_same(activity):
                return
            self._cancel_active("rmf_stop_request")

    def execute_action(self, category, description, execution) -> None:
        """Reject action categories that the LingTu fleet did not advertise."""

        del execution
        self._node.get_logger().error(f"Unsupported RMF action rejected: {category} {description}")
        self._reassign_tasks()

    def _activity_identifier(self):
        if self._execution is None:
            return None
        okay = getattr(self._execution, "okay", True)
        if callable(okay):
            okay = okay()
        if not bool(okay):
            self._cancel_active("rmf_execution_stopped")
            return None
        return getattr(self._execution, "identifier", None)

    def _renew_if_due(self) -> bool:
        if not self._goal_submitted or not self._request_id:
            return True
        interval = max(1.0, self._settings.gateway.lease_ttl_s / 3.0)
        now = time.monotonic()
        if now - self._last_renew_monotonic < interval:
            return True
        self._renewal_sequence += 1
        renewed, reason = self._mission_port.renew_lease(
            self._request_id,
            renewal_sequence=self._renewal_sequence,
        )
        if not renewed:
            self._node.get_logger().error(f"RMF command lease renewal failed: {reason}")
            return False
        self._last_renew_monotonic = now
        return True

    def update(self) -> None:
        """Poll LingTu state, update RMF, and advance command lifecycle."""

        try:
            snapshot = self._state_source.snapshot()
            state = self._bridge.report_state(snapshot)
        except Exception as exc:
            self._node.get_logger().warning(f"LingTu RMF state update unavailable: {exc}")
            return
        if not state.connected:
            self._node.get_logger().warning("LingTu state has no verified Open-RMF level binding")
            return

        rmf_state = self._rmf_easy.RobotState(
            state.map_name,
            list(state.position),
            state.battery_soc,
        )
        with self._lock:
            if self._update_handle is None:
                self._update_handle = self._fleet_handle.add_robot(
                    self._settings.robot_name,
                    rmf_state,
                    self._robot_configuration,
                    self.callbacks(),
                )
                if self._update_handle is None:
                    raise RuntimeError("Open-RMF failed to add the LingTu robot")
                return

            activity = self._activity_identifier()
            self._update_handle.update(rmf_state, activity)
            if self._release_pending:
                if self._release_active_lease():
                    self._clear_active()
                    self._reassign_tasks()
                return

            if self._execution is None or not self._goal_submitted:
                return
            if self._completion_gate is None:
                if self._cancel_active("open_rmf_goal_outcome_unknown"):
                    self._reassign_tasks()
                return
            if not self._renew_if_due():
                self._cancel_active("rmf_lease_lost")
                self._reassign_tasks()
                return
            outcome = self._completion_gate.observe(snapshot)
            if outcome == "success":
                if not self._release_active_lease():
                    self._node.get_logger().error(
                        "LingTu reached the RMF goal but command lease release "
                        "is not confirmed; completion will be retried"
                    )
                    return
                self._execution.finished()
                self._node.get_logger().info(f"LingTu completed RMF navigation [{self._request_id}]")
                self._clear_active()
            elif outcome in {"failed", "blocked"}:
                self._node.get_logger().error(f"LingTu RMF navigation [{self._request_id}] entered {outcome}")
                self._cancel_active(f"rmf_native_{outcome}")
                self._reassign_tasks()

    def _release_active_lease(self) -> bool:
        if not self._request_id:
            return True
        released, reason = self._mission_port.release_lease(self._request_id)
        if not released:
            self._node.get_logger().error(f"RMF command lease release failed: {reason}")
        return released

    def _cancel_active(self, reason: str) -> bool:
        if self._goal_submitted and self._request_id:
            cancelled, cancel_reason = self._mission_port.cancel(
                self._request_id,
                reason=reason,
            )
            if not cancelled:
                self._node.get_logger().error(f"LingTu native cancel failed: {cancel_reason}")
                return False
        else:
            if not self._release_active_lease():
                return False
        self._clear_active()
        return True

    def _clear_active(self) -> None:
        self._execution = None
        self._request_id = ""
        self._goal_submitted = False
        self._completion_gate = None
        self._release_pending = False
        self._renewal_sequence = 0
        self._last_renew_monotonic = 0.0

    def _reassign_tasks(self) -> None:
        try:
            self._fleet_handle.more().reassign_dispatched_tasks()
        except Exception as exc:
            self._node.get_logger().error(f"Open-RMF task reassignment failed: {exc}")

    def shutdown(self) -> None:
        """Cancel active navigation and release command ownership."""

        with self._lock:
            self._cancel_active("rmf_sidecar_shutdown")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lingtu_rmf_adapter",
        description="Open-RMF EasyFullControl sidecar for one LingTu robot",
    )
    parser.add_argument("--bridge-config", required=True)
    parser.add_argument("--fleet-config", required=True)
    parser.add_argument("--nav-graph", required=True)
    parser.add_argument("--server-uri", default="")
    parser.add_argument("--use-sim-time", action="store_true")
    return parser


def main(argv=None) -> None:
    """Run the ROS 2 Jazzy EasyFullControl sidecar."""

    faulthandler.enable()

    import rclpy
    import rclpy.node
    import rmf_adapter
    import rmf_adapter.easy_full_control as rmf_easy
    from rclpy.parameter import Parameter
    from rmf_adapter import Adapter

    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    args = _parser().parse_args(args_without_ros[1:])

    settings = load_bridge_settings(args.bridge_config)
    validate_dispatch_mode(settings, args.fleet_config)

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        args.fleet_config,
        args.nav_graph,
    )
    if fleet_config is None:
        raise RuntimeError("Open-RMF failed to parse the fleet configuration")
    if str(fleet_config.fleet_name) != settings.fleet_name:
        raise RuntimeError("RMF fleet name does not match LingTu bridge config")
    robot_configuration = fleet_config.get_known_robot_configuration(settings.robot_name)
    if robot_configuration is None:
        raise RuntimeError("RMF fleet config has no LingTu robot configuration")

    node = rclpy.node.Node(f"{settings.fleet_name}_lingtu_command_handle")
    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    adapter = Adapter.make(f"{settings.fleet_name}_lingtu_fleet_adapter")
    if adapter is None:
        raise RuntimeError("Open-RMF adapter initialization failed; check the schedule node")
    if args.use_sim_time:
        adapter.node.use_sim_time()
    fleet_config.server_uri = args.server_uri or None
    adapter.start()
    time.sleep(1.0)
    fleet_handle = adapter.add_easy_fleet(fleet_config)
    if fleet_handle is None:
        raise RuntimeError("Open-RMF failed to add the LingTu fleet")

    controller = LingTuRobotAdapter(
        node=node,
        rmf_easy=rmf_easy,
        fleet_handle=fleet_handle,
        robot_configuration=robot_configuration,
        settings=settings,
    )
    timer = node.create_timer(settings.poll_period_s, controller.update)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        controller.shutdown()
        node.destroy_timer(timer)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
