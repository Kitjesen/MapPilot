#!/usr/bin/env python3
"""Bridge the external CMU Unity/TARE simulation contract into LingTu.

This module is the explicit boundary between the CMU
``autonomy_stack_mecanum_wheel_platform`` ROS graph and LingTu's canonical
``/nav/*`` plus ``/exploration/*`` topics. It intentionally lives in ``sim/``
and does not import LingTu driver modules, so it can be used only as a
simulation adapter.
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np

from runtime.runtime_interface import FRAMES, MESSAGE_FORMATS, TOPICS, adapter_relay_aliases


@dataclass(frozen=True)
class RelaySpec:
    source_topic: str
    target_topic: str
    msg_type: str
    direction: str
    required: bool = True
    note: str = ""


def _relay_msg_type(msg_format: str) -> str:
    if msg_format in MESSAGE_FORMATS:
        return MESSAGE_FORMATS[msg_format].ros_type
    return msg_format


CMU_CONTRACT_RELAY_TOPICS: tuple[RelaySpec, ...] = tuple(
    RelaySpec(
        alias.source,
        alias.target,
        _relay_msg_type(alias.msg_format),
        "cmu_to_lingtu",
        note=alias.note,
    )
    for alias in adapter_relay_aliases("cmu_unity")
    if not (alias.source == TOPICS.cmd_vel and alias.target == "/cmd_vel")
)


CMU_TO_LINGTU_RELAY_TOPICS: tuple[RelaySpec, ...] = (
    *CMU_CONTRACT_RELAY_TOPICS,
    RelaySpec(
        "/registered_scan",
        TOPICS.registered_cloud,
        "sensor_msgs/msg/PointCloud2",
        "cmu_to_lingtu",
        note=(
            "Legacy full-cloud relay. The default runtime replaces this with "
            "a robot-local body-frame crop on /slam/registered_cloud."
        ),
    ),
    RelaySpec(
        "/way_point",
        TOPICS.exploration_way_point,
        "geometry_msgs/msg/PointStamped",
        "cmu_to_lingtu",
        note="TARE/FAR waypoint output into TAREExplorerModule.",
    ),
    RelaySpec(
        "/global_path_full",
        "/exploration/global_path_full",
        "nav_msgs/msg/Path",
        "cmu_to_lingtu",
        required=False,
        note="TARE full global exploration strategy path for RViz/runtime evidence.",
    ),
    RelaySpec(
        "/global_path",
        "/exploration/global_path",
        "nav_msgs/msg/Path",
        "cmu_to_lingtu",
        required=False,
        note="TARE trimmed global exploration strategy path.",
    ),
    RelaySpec(
        "/local_path",
        "/exploration/local_path",
        "nav_msgs/msg/Path",
        "cmu_to_lingtu",
        required=False,
        note="TARE local exploration strategy path.",
    ),
    RelaySpec(
        "/path",
        TOPICS.exploration_cmu_local_planner_path,
        "nav_msgs/msg/Path",
        "cmu_to_lingtu",
        required=False,
        note="Optional CMU native local planner path, separate from TARE strategy.",
    ),
    RelaySpec(
        "/exploration_path",
        "/exploration/path",
        "nav_msgs/msg/Path",
        "cmu_to_lingtu",
        required=False,
        note="TARE exploration path evidence.",
    ),
    RelaySpec(
        "/runtime",
        "/exploration/runtime",
        "std_msgs/msg/Float32",
        "cmu_to_lingtu",
        required=False,
        note="TARE runtime telemetry.",
    ),
    RelaySpec(
        "/runtime_breakdown",
        "/exploration/runtime_breakdown",
        "std_msgs/msg/Int32MultiArray",
        "cmu_to_lingtu",
        required=False,
        note="TARE runtime breakdown telemetry.",
    ),
    RelaySpec(
        "/exploration_finish",
        "/exploration/finish",
        "std_msgs/msg/Bool",
        "cmu_to_lingtu",
        required=False,
        note="TARE completion flag.",
    ),
)


LINGTU_TO_CMU_RELAY_TOPICS: tuple[RelaySpec, ...] = (
    RelaySpec(
        TOPICS.exploration_start,
        "/start_exploration",
        "std_msgs/msg/Bool",
        "lingtu_to_cmu",
        note="LingTu-controlled TARE start trigger.",
    ),
    RelaySpec(
        TOPICS.goal_point,
        "/goal_point",
        "geometry_msgs/msg/PointStamped",
        "lingtu_to_cmu",
        required=False,
        note="Optional FAR goal input.",
    ),
    RelaySpec(
        TOPICS.navigation_boundary,
        "/navigation_boundary",
        "geometry_msgs/msg/PolygonStamped",
        "lingtu_to_cmu",
        required=False,
        note="Optional exploration/navigation boundary.",
    ),
)


_SIM_CMD_ALIAS = next(
    alias
    for alias in adapter_relay_aliases("cmu_unity")
    if alias.source == TOPICS.cmd_vel and alias.target == "/cmd_vel"
)

SIM_CMD_VEL_RELAY = RelaySpec(
    _SIM_CMD_ALIAS.source,
    _SIM_CMD_ALIAS.target,
    _relay_msg_type(_SIM_CMD_ALIAS.msg_format),
    "lingtu_to_cmu_sim",
    note=(
        _SIM_CMD_ALIAS.note
        or "Simulation-only LingTu command relay. Do not enable on a hardware ROS domain."
    ),
)


def build_relay_specs(
    *,
    relay_cmd_vel_to_sim: bool = False,
    include_optional: bool = True,
    local_registered_cloud: bool = True,
) -> tuple[RelaySpec, ...]:
    specs: list[RelaySpec] = []
    for spec in (*CMU_TO_LINGTU_RELAY_TOPICS, *LINGTU_TO_CMU_RELAY_TOPICS):
        if (
            local_registered_cloud
            and spec.source_topic == "/registered_scan"
            and spec.target_topic == TOPICS.registered_cloud
        ):
            continue
        if include_optional or spec.required:
            specs.append(spec)
    if relay_cmd_vel_to_sim:
        specs.append(SIM_CMD_VEL_RELAY)
    return tuple(specs)


def relay_contract(
    *,
    relay_cmd_vel_to_sim: bool = False,
    include_optional: bool = True,
    local_registered_cloud: bool = True,
) -> dict[str, str]:
    return {
        f"{spec.source_topic}->{spec.target_topic}": spec.msg_type
        for spec in build_relay_specs(
            relay_cmd_vel_to_sim=relay_cmd_vel_to_sim,
            include_optional=include_optional,
            local_registered_cloud=local_registered_cloud,
        )
    }


def required_relay_contract(*, relay_cmd_vel_to_sim: bool = True) -> dict[str, str]:
    return relay_contract(
        relay_cmd_vel_to_sim=relay_cmd_vel_to_sim,
        include_optional=False,
    )


def _resolve_msg_class(msg_type: str) -> type[Any]:
    try:
        package, typename = msg_type.split("/msg/", maxsplit=1)
    except ValueError as exc:
        raise ValueError(f"Unsupported ROS message type string: {msg_type}") from exc
    module = __import__(f"{package}.msg", fromlist=[typename])
    return getattr(module, typename)


def _qos_for_msg(msg_type: str):
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    if msg_type == "sensor_msgs/msg/PointCloud2":
        return QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def _copy_twist_stamped_for_cmu(msg: Any, *, frame_id: str) -> Any:
    from geometry_msgs.msg import TwistStamped

    out = TwistStamped()
    out.header = msg.header
    out.header.frame_id = frame_id or msg.header.frame_id or "vehicle"
    out.twist = msg.twist
    return out


def _odom_pose2d_from_msg(msg: Any) -> tuple[float, float, float, float] | None:
    try:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = float(position.x)
        y = float(position.y)
        z = float(position.z)
        qx = float(orientation.x)
        qy = float(orientation.y)
        qz = float(orientation.z)
        qw = float(orientation.w)
    except Exception:
        return None
    values = (x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(v) for v in values):
        return None
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return (x, y, z, yaw)


def _odom_xy_from_msg(msg: Any) -> tuple[float, float] | None:
    pose = _odom_pose2d_from_msg(msg)
    if pose is None:
        return None
    return (pose[0], pose[1])


def _filter_xyz_points_by_radius(
    points: np.ndarray,
    *,
    center_xy: tuple[float, float],
    radius_m: float,
) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) == 0:
        return np.empty((0, 3), dtype=np.float32)
    xyz = pts[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    if not finite.any():
        return np.empty((0, 3), dtype=np.float32)
    xyz = xyz[finite]
    radius = float(radius_m)
    if radius <= 0:
        return xyz.astype(np.float32, copy=False)
    dx = xyz[:, 0] - float(center_xy[0])
    dy = xyz[:, 1] - float(center_xy[1])
    keep = (dx * dx + dy * dy) <= radius * radius
    return xyz[keep].astype(np.float32, copy=False)


def _filter_xyz_points_by_height(
    points: np.ndarray,
    *,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) == 0:
        return np.empty((0, 3), dtype=np.float32)
    xyz = pts[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    z = xyz[:, 2]
    keep = finite & (z > float(z_min)) & (z < float(z_max))
    return xyz[keep].astype(np.float32, copy=False)


def _global_xyz_points_to_body(
    points: np.ndarray,
    *,
    pose_xyzyaw: tuple[float, float, float, float],
) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) == 0:
        return np.empty((0, 3), dtype=np.float32)
    x, y, z, yaw = pose_xyzyaw
    dx = pts[:, 0].astype(np.float64) - float(x)
    dy = pts[:, 1].astype(np.float64) - float(y)
    dz = pts[:, 2].astype(np.float64) - float(z)
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    body = np.empty((len(pts), 3), dtype=np.float32)
    body[:, 0] = (c * dx + s * dy).astype(np.float32)
    body[:, 1] = (-s * dx + c * dy).astype(np.float32)
    body[:, 2] = dz.astype(np.float32)
    return body


def _pointcloud_xyz_array(msg: Any) -> np.ndarray:
    from sensor_msgs_py import point_cloud2

    points: list[tuple[float, float, float]] = []
    for raw in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        names = getattr(getattr(raw, "dtype", None), "names", None)
        if names and {"x", "y", "z"}.issubset(set(names)):
            x, y, z = float(raw["x"]), float(raw["y"]), float(raw["z"])
        else:
            x, y, z = raw[:3]
            x, y, z = float(x), float(y), float(z)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def _make_xyz_pointcloud(header: Any, points: np.ndarray) -> Any:
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, np.asarray(points, dtype=np.float32))


def _default_ros_domain_is_unsafe() -> bool:
    value = os.environ.get("ROS_DOMAIN_ID", "")
    return value in {"", "0"}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--relay-cmd-vel-to-sim",
        action="store_true",
        help=(
            "Relay LingTu /nav/cmd_vel TwistStamped to CMU /cmd_vel. This is "
            "disabled by default and must only be used in an isolated "
            "simulation ROS domain."
        ),
    )
    parser.add_argument(
        "--allow-default-ros-domain",
        action="store_true",
        help="Allow command relay when ROS_DOMAIN_ID is unset or 0.",
    )
    parser.add_argument(
        "--required-only",
        action="store_true",
        help="Relay only required topics, omitting optional evidence/control topics.",
    )
    parser.add_argument(
        "--cmd-vel-frame-id",
        default="vehicle",
        help="Frame id used when relaying /nav/cmd_vel to CMU /cmd_vel.",
    )
    parser.add_argument(
        "--local-registered-scan-topic",
        default="",
        help=(
            "Publish a robot-centered crop of CMU /registered_scan for TARE. "
            "Use /lingtu/registered_scan_local in LingTu-owned CMU profiles."
        ),
    )
    parser.add_argument(
        "--local-scan-odom-topic",
        default="/state_estimation",
        help="Odometry topic used to center the local registered scan crop.",
    )
    parser.add_argument(
        "--local-scan-radius",
        type=float,
        default=8.0,
        help="XY radius in meters for the robot-centered registered scan crop.",
    )
    parser.add_argument(
        "--local-scan-republish-hz",
        type=float,
        default=2.0,
        help=(
            "Republish the cached CMU registered scan as a robot-centered local "
            "scan at this rate. CMU Unity may publish /registered_scan sparsely; "
            "periodic recropping keeps LingTu/TARE local sensing live as odom moves."
        ),
    )
    parser.add_argument(
        "--local-registered-cloud",
        action="store_true",
        default=True,
        help=(
            "Publish the local registered scan crop to /slam/registered_cloud "
            "instead of relaying the full CMU /registered_scan map."
        ),
    )
    parser.add_argument(
        "--no-local-registered-cloud",
        dest="local_registered_cloud",
        action="store_false",
        help="Disable the local crop to /slam/registered_cloud for compatibility checks.",
    )
    parser.add_argument(
        "--nav-cloud-z-min",
        type=float,
        default=0.30,
        help="Minimum z for the /slam/registered_cloud obstacle-height crop.",
    )
    parser.add_argument(
        "--nav-cloud-z-max",
        type=float,
        default=2.00,
        help="Maximum z for the /slam/registered_cloud obstacle-height crop.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if (
        args.relay_cmd_vel_to_sim
        and _default_ros_domain_is_unsafe()
        and not args.allow_default_ros_domain
    ):
        print(
            "Refusing to relay /nav/cmd_vel -> /cmd_vel on default ROS_DOMAIN_ID. "
            "Set an isolated ROS_DOMAIN_ID or pass --allow-default-ros-domain "
            "for a controlled local test.",
            file=sys.stderr,
        )
        return 2

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        print(f"CMU Unity LingTu adapter requires ROS 2 Python packages: {exc}", file=sys.stderr)
        return 2

    specs = build_relay_specs(
        relay_cmd_vel_to_sim=args.relay_cmd_vel_to_sim,
        include_optional=not args.required_only,
        local_registered_cloud=args.local_registered_cloud,
    )

    class CmuUnityLingtuAdapter(Node):
        def __init__(self) -> None:
            super().__init__("lingtu_cmu_unity_adapter")
            self._publishers = []
            self._subscriptions = []
            self._latest_odom_pose: tuple[float, float, float, float] | None = None
            self._latest_registered_scan_header = None
            self._latest_registered_scan_points: np.ndarray | None = None
            self._local_registered_scan_pubs: tuple[Any | None, Any | None] = (None, None)
            self._local_scan_timer = None
            for spec in specs:
                msg_cls = _resolve_msg_class(spec.msg_type)
                qos = _qos_for_msg(spec.msg_type)
                pub = self.create_publisher(msg_cls, spec.target_topic, qos)
                self._publishers.append(pub)

                def relay(msg, *, relay_spec: RelaySpec = spec, publisher=pub) -> None:
                    out = msg
                    if relay_spec is SIM_CMD_VEL_RELAY:
                        out = _copy_twist_stamped_for_cmu(
                            msg,
                            frame_id=str(args.cmd_vel_frame_id),
                        )
                    publisher.publish(out)

                sub = self.create_subscription(msg_cls, spec.source_topic, relay, qos)
                self._subscriptions.append(sub)
                self.get_logger().info(
                    f"relay {spec.source_topic} -> {spec.target_topic} "
                    f"[{spec.msg_type}] direction={spec.direction} required={spec.required}"
                )
            if not args.relay_cmd_vel_to_sim:
                self.get_logger().info(
                    "simulation command relay disabled: /nav/cmd_vel will not be "
                    "published to CMU /cmd_vel"
                )
            self._configure_local_registered_scan()

        def _configure_local_registered_scan(self) -> None:
            local_topic = str(args.local_registered_scan_topic or "").strip()
            publish_to_nav = bool(args.local_registered_cloud)
            if not local_topic and not publish_to_nav:
                return

            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import PointCloud2

            cloud_qos = _qos_for_msg("sensor_msgs/msg/PointCloud2")
            odom_qos = _qos_for_msg("nav_msgs/msg/Odometry")
            local_pub = (
                self.create_publisher(PointCloud2, local_topic, cloud_qos)
                if local_topic
                else None
            )
            nav_pub = (
                self.create_publisher(PointCloud2, TOPICS.registered_cloud, cloud_qos)
                if publish_to_nav
                else None
            )
            if local_pub is not None:
                self._publishers.append(local_pub)
            if nav_pub is not None:
                self._publishers.append(nav_pub)
            self._local_registered_scan_pubs = (local_pub, nav_pub)

            def on_odom(msg: Odometry) -> None:
                pose = _odom_pose2d_from_msg(msg)
                if pose is not None:
                    self._latest_odom_pose = pose

            def on_registered_scan(msg: PointCloud2) -> None:
                self._latest_registered_scan_header = msg.header
                self._latest_registered_scan_points = _pointcloud_xyz_array(msg)
                self._publish_local_registered_scan()

            self._subscriptions.append(
                self.create_subscription(
                    Odometry,
                    str(args.local_scan_odom_topic),
                    on_odom,
                    odom_qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    "/registered_scan",
                    on_registered_scan,
                    cloud_qos,
                )
            )
            targets = [
                topic
                for topic in (
                    local_topic,
                    TOPICS.registered_cloud if publish_to_nav else "",
                )
                if topic
            ]
            self.get_logger().info(
                f"local registered scan crop enabled: /registered_scan -> {','.join(targets)} "
                f"center={args.local_scan_odom_topic} radius={float(args.local_scan_radius):.2f}m"
            )
            hz = max(0.0, float(args.local_scan_republish_hz))
            if hz > 0.0:
                self._local_scan_timer = self.create_timer(
                    1.0 / hz,
                    self._publish_local_registered_scan,
                )
                self.get_logger().info(
                    f"local registered scan recrop timer enabled: {hz:.2f} Hz"
                )

        def _publish_local_registered_scan(self) -> None:
            local_pub, nav_pub = self._local_registered_scan_pubs
            pose = self._latest_odom_pose
            points = self._latest_registered_scan_points
            header = self._latest_registered_scan_header
            if pose is None or points is None or header is None:
                return
            out_header = copy.copy(header)
            try:
                out_header.stamp = self.get_clock().now().to_msg()
            except Exception:
                pass
            filtered = _filter_xyz_points_by_radius(
                points,
                center_xy=(pose[0], pose[1]),
                radius_m=float(args.local_scan_radius),
            )
            if local_pub is not None:
                local_pub.publish(_make_xyz_pointcloud(out_header, filtered))
            if nav_pub is not None:
                nav_header = copy.copy(out_header)
                nav_header.frame_id = FRAMES.body
                body_points = _global_xyz_points_to_body(filtered, pose_xyzyaw=pose)
                nav_points = _filter_xyz_points_by_height(
                    body_points,
                    z_min=float(args.nav_cloud_z_min),
                    z_max=float(args.nav_cloud_z_max),
                )
                nav_pub.publish(_make_xyz_pointcloud(nav_header, nav_points))

    rclpy.init(args=None)
    node = CmuUnityLingtuAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
