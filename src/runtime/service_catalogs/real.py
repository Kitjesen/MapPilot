"""Real service installation and readiness metadata.

Product roles and their systemd targets come from the real environment declaration.
ProductControl owns process selection and lifecycle.
"""

from __future__ import annotations

import sys
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from message.topics import TOPICS, dds_topic_name
from runtime.yaml_helpers import load_yaml

_REAL_UNITS = {
    name: (process["target"],)
    for name, process in load_yaml(
        Path(__file__).resolve().parents[3] / "config" / "system" / "envs" / "real.yaml"
    )["processes"].items()
}


@dataclass(frozen=True)
class ServiceSpec:
    name: str
    role: str
    units: tuple[str, ...]
    group: str
    retired_units: tuple[str, ...] = ()
    start_units: tuple[str, ...] = ()
    product_default: bool = False
    install_enable_default: bool = False
    ros2_compat: bool = False
    experimental: bool = False
    optional: bool = False
    catalog_alias: bool = False
    checks: tuple[str, ...] = ("systemd",)
    topics: tuple[str, ...] = ()
    dds_topics: tuple[str, ...] = ()
    shm_topics: tuple[str, ...] = ()
    shm_channel_env: tuple[str, ...] = ()
    files: tuple[str, ...] = ()
    status_max_age_s: float | None = None
    binaries: tuple[tuple[str, str, str], ...] = ()
    description: str = ""

    def shm_channels(self, environment: Mapping[str, str]) -> list[str]:
        return [environment[key] for key in self.shm_channel_env if key in environment]

    def metadata(self, environment: Mapping[str, str]) -> dict[str, Any]:
        return {
            "role": self.role,
            "group": self.group,
            "retired_units": list(self.retired_units),
            "product_default": self.product_default,
            "install_enable_default": self.install_enable_default,
            "ros2_compat": self.ros2_compat,
            "experimental": self.experimental,
            "optional": self.optional,
            "catalog_alias": self.catalog_alias,
            "checks": list(self.checks),
            "topics": list(self.topics),
            "dds_topics": list(self.dds_topics),
            "shm_topics": list(self.shm_topics),
            "shm_channels": self.shm_channels(environment),
            "files": list(self.files),
            "status_max_age_s": self.status_max_age_s,
            "binaries": [{"name": name, "env": env, "path": path} for name, env, path in self.binaries],
            "description": self.description,
        }


def _dds(*topics: str) -> tuple[str, ...]:
    return tuple(dds_topic_name(topic) for topic in topics)


SERVICE_SPECS: tuple[ServiceSpec, ...] = (
    ServiceSpec(
        name="lidar",
        role="sensor_input",
        units=_REAL_UNITS["lidar"],
        start_units=_REAL_UNITS["lidar"],
        group="native_dds",
        retired_units=(
            "lingtu-livox-dds.service",
            "robot-lidar.service",
            "lidar.service",
        ),
        product_default=True,
        checks=("systemd", "native_binary", "dds"),
        topics=(TOPICS.lidar_scan, TOPICS.imu),
        dds_topics=_dds(TOPICS.lidar_scan, TOPICS.imu),
        binaries=(
            (
                "livox_dds",
                "LINGTU_LIVOX_BIN",
                "/opt/lingtu/current/bin/livox_sdk2_stream",
            ),
        ),
        description="Native Livox DDS LiDAR input.",
    ),
    ServiceSpec(
        name="driver",
        role="motion_output",
        units=_REAL_UNITS["driver"],
        start_units=_REAL_UNITS["driver"],
        group="native_dds",
        retired_units=("lingtu-driver.service",),
        product_default=True,
        install_enable_default=False,
        checks=("systemd", "native_binary", "status_file"),
        topics=(TOPICS.cmd_vel, "/driver/control_state"),
        dds_topics=(_dds(TOPICS.cmd_vel)[0], "rt/driver/control_state"),
        files=("/dev/shm/lingtu/driver_status.json",),
        status_max_age_s=3.0,
        binaries=(
            (
                "driver",
                "LINGTU_DRIVER_BIN",
                "/opt/lingtu/current/bin/lingtu_driver",
            ),
        ),
        description="Native DDS motion driver using the RobotConfig-selected backend.",
    ),
    ServiceSpec(
        name="camera",
        role="sensor_input",
        units=_REAL_UNITS["camera"],
        start_units=_REAL_UNITS["camera"],
        group="hardware",
        retired_units=("lingtu-camera-dds.service",),
        optional=True,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(TOPICS.camera_info,),
        dds_topics=_dds(TOPICS.camera_info),
        shm_topics=(TOPICS.camera_color, TOPICS.camera_depth, TOPICS.camera_info),
        shm_channel_env=(
            "LINGTU_CAMERA_COLOR_SHM",
            "LINGTU_CAMERA_DEPTH_SHM",
            "LINGTU_CAMERA_INFO_SHM",
        ),
        files=("/dev/shm/lingtu/camera_status.json",),
        binaries=(
            (
                "camera_dds",
                "LINGTU_CAMERA_DDS_BIN",
                "/opt/lingtu/current/bin/lingtu_camera_dds",
            ),
            (
                "orbbec_capture",
                "LINGTU_ORBBEC_CAPTURE_BIN",
                "/opt/lingtu/current/bin/orbbec_capture",
            ),
        ),
        description="Optional native camera service using SHM for frames and DDS for camera metadata.",
    ),
    ServiceSpec(
        name="video",
        role="operator_video",
        units=_REAL_UNITS["video"],
        start_units=_REAL_UNITS["video"],
        group="hardware",
        optional=True,
        checks=("systemd", "native_binary", "status_file", "whep"),
        shm_topics=(TOPICS.camera_color,),
        shm_channel_env=("LINGTU_VIDEO_COLOR_SHM",),
        files=("/dev/shm/lingtu/video_status.json",),
        status_max_age_s=3.0,
        binaries=(
            (
                "camera_webrtc",
                "LINGTU_VIDEO_BIN",
                "/opt/lingtu/current/bin/lingtu_camera_webrtc",
            ),
        ),
        description="Optional native WebRTC preview consuming the existing color SHM.",
    ),
    ServiceSpec(
        name="brainstem",
        role="hardware_bridge",
        units=("robot-brainstem.service", "brainstem.service", "brainstem"),
        group="hardware",
        description="Thunder brainstem bridge.",
    ),
    ServiceSpec(
        name="gnss",
        role="sensor_input",
        units=("lt-gnss.service",),
        start_units=("lt-gnss.service",),
        group="hardware",
        retired_units=("lingtu-gnss-dds.service",),
        optional=True,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(TOPICS.gnss_fix, TOPICS.gnss_status),
        dds_topics=_dds(TOPICS.gnss_fix, TOPICS.gnss_status),
        files=("/dev/shm/lingtu/gnss_status.json",),
        binaries=(
            (
                "gnss_dds",
                "LINGTU_GNSS_DDS_BIN",
                "/opt/lingtu/current/bin/lingtu_gnss_dds",
            ),
        ),
        description="Optional native WTRTK-980 GNSS stream service.",
    ),
    ServiceSpec(
        name="nav",
        role="navigation_runtime",
        units=_REAL_UNITS["nav"],
        start_units=_REAL_UNITS["nav"],
        group="native_dds",
        retired_units=(
            "lingtu-nav-dds.service",
            "nav-dds.service",
            "nav.service",
        ),
        product_default=True,
        checks=("systemd", "native_binary", "status_file"),
        files=("/dev/shm/lingtu/nav_endpoint_status.json",),
        binaries=(
            (
                "nav_dds",
                "LINGTU_NAV_DDS_BIN",
                "/opt/lingtu/current/bin/navd",
            ),
        ),
        description="Native DDS navigation runtime.",
    ),
    ServiceSpec(
        name="traversability",
        role="terrain_runtime",
        units=_REAL_UNITS["traversability"],
        start_units=_REAL_UNITS["traversability"],
        group="native_dds",
        retired_units=(
            "lingtu-traversability-dds.service",
            "traversability-dds.service",
            "traversability.service",
        ),
        product_default=True,
        install_enable_default=False,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(
            TOPICS.traversability,
            TOPICS.terrain_map,
            TOPICS.terrain_map_ext,
        ),
        dds_topics=_dds(
            TOPICS.traversability,
            TOPICS.terrain_map,
            TOPICS.terrain_map_ext,
        ),
        files=("/dev/shm/lingtu/traversability_status.json",),
        binaries=(
            (
                "traversability_dds",
                "LINGTU_TRAVERSABILITY_DDS_BIN",
                "/opt/lingtu/current/bin/lingtu_traversability_dds",
            ),
        ),
        description="Native DDS traversability producer.",
    ),
    ServiceSpec(
        name="explore",
        role="exploration_runtime",
        units=_REAL_UNITS["explore"],
        start_units=_REAL_UNITS["explore"],
        group="native_dds",
        retired_units=(
            "lingtu-explore-dds.service",
            "explore-dds.service",
            "explore.service",
        ),
        optional=True,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(
            TOPICS.odometry,
            TOPICS.exploration_snapshot,
            TOPICS.exploration_command,
            TOPICS.exploration_ack,
            TOPICS.nav_command_request,
            TOPICS.nav_command_ack,
            TOPICS.nav_goal_status,
            TOPICS.exploration_segment_request,
            TOPICS.exploration_segment_ack,
            TOPICS.exploration_segment_status,
        ),
        # Startup readiness observes only periodic inputs. Command/ack traffic begins
        # after START and must not create a circular launcher dependency.
        dds_topics=_dds(TOPICS.odometry, TOPICS.exploration_snapshot),
        files=("/dev/shm/lingtu/explore_status.json",),
        status_max_age_s=6.0,
        binaries=(
            (
                "explore_dds",
                "LINGTU_EXPLORE_DDS_BIN",
                "/opt/lingtu/current/bin/lingtu_explore_dds",
            ),
        ),
        description=(
            "Native hierarchical exploration endpoint over identity-versioned "
            "rolling occupancy snapshots and typed navigation commands."
        ),
    ),
    ServiceSpec(
        name="control",
        role="product_control_transport",
        units=("lt-control.service",),
        start_units=("lt-control.service",),
        group="control",
        optional=True,
        description="Host-independent loopback transport for ProductControl.",
    ),
    ServiceSpec(
        name="gateway",
        role="interface_runtime",
        units=_REAL_UNITS["host"],
        start_units=_REAL_UNITS["host"],
        group="host",
        retired_units=("lingtu.service",),
        product_default=True,
        checks=("systemd", "http"),
        description="LingTu Python runtime and Gateway process.",
    ),
    ServiceSpec(
        name="lingtu",
        role="orchestrator_runtime",
        units=_REAL_UNITS["host"],
        start_units=_REAL_UNITS["host"],
        group="host",
        retired_units=("lingtu.service",),
        product_default=True,
        checks=("systemd", "http"),
        description="LingTu Python runtime service.",
    ),
    ServiceSpec(
        name="slam",
        role="localization_runtime",
        units=_REAL_UNITS["slam"],
        start_units=_REAL_UNITS["slam"],
        group="native_dds",
        retired_units=(
            "lingtu-slam-dds.service",
            "robot-localizer.service",
            "robot-fastlio2.service",
            "localization.service",
            "slam.service",
        ),
        product_default=True,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(
            TOPICS.odometry,
            TOPICS.map_cloud,
            TOPICS.localization_health,
        ),
        dds_topics=_dds(
            TOPICS.odometry,
            TOPICS.map_cloud,
            TOPICS.localization_health,
        ),
        files=("/tmp/lingtu_slam_status.json",),
        binaries=(
            (
                "slam_dds",
                "LINGTU_SLAM_BIN",
                "/opt/lingtu/current/bin/slamd",
            ),
        ),
        description="Native DDS SLAM/localization owner.",
    ),
    ServiceSpec(
        name="maps",
        role="maps_runtime",
        units=_REAL_UNITS["maps"],
        start_units=_REAL_UNITS["maps"],
        group="native_dds",
        retired_units=("mapd.service",),
        product_default=True,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(
            TOPICS.maps_state,
            TOPICS.maps_live_cloud,
            TOPICS.maps_voxel_cloud,
            TOPICS.maps_local_collision,
            TOPICS.maps_accumulated_cloud,
            TOPICS.maps_occupancy,
            TOPICS.maps_elevation,
            TOPICS.maps_esdf,
            TOPICS.maps_scene,
        ),
        dds_topics=_dds(
            TOPICS.maps_state,
            TOPICS.maps_live_cloud,
            TOPICS.maps_voxel_cloud,
            TOPICS.maps_local_collision,
            TOPICS.maps_accumulated_cloud,
            TOPICS.maps_occupancy,
            TOPICS.maps_elevation,
            TOPICS.maps_esdf,
            TOPICS.maps_scene,
        ),
        files=("/dev/shm/lingtu/mapd_status.json",),
        status_max_age_s=3.0,
        binaries=(
            (
                "mapd",
                "LINGTU_MAPD_BIN",
                "/opt/lingtu/current/bin/mapd",
            ),
        ),
        description="Native live maps, bounded scene, and map service runtime.",
    ),
)

GROUP_ORDER: dict[str, tuple[str, ...]] = {
    "control": ("control",),
    "native_dds": (
        "lidar",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "explore",
    ),
    "host": ("gateway", "lingtu"),
    "hardware": ("camera", "video", "gnss", "brainstem"),
}

RUNTIME_INSTALL_ORDER: tuple[str, ...] = (
    "lidar",
    "camera",
    "video",
    "slam",
    "maps",
    "traversability",
    "nav",
    "driver",
    "explore",
    "lingtu",
)

INSTALL_MODE_ALIASES: dict[str, str] = {
    "lidar-dds": "lidar",
    "livox": "lidar",
    "livox-dds": "lidar",
    "driver-dds": "driver",
    "native-driver": "driver",
    "thunder-driver": "driver",
    "camera": "camera",
    "camera-dds": "camera",
    "native-camera": "camera",
    "video": "video",
    "camera-video": "video",
    "native-video": "video",
    "gnss": "gnss",
    "gnss-dds": "gnss",
    "native-gnss": "gnss",
    "slam-dds": "slam",
    "cpp-slam": "slam",
    "mapd": "maps",
    "maps-dds": "maps",
    "nav-dds": "nav",
    "cpp-nav": "nav",
    "traversability-dds": "traversability",
    "terrain-dds": "traversability",
    "explore-dds": "explore",
    "cpp-explore": "explore",
    "exploration-dds": "explore",
    "lingtu": "lingtu",
    "app": "lingtu",
    "host": "lingtu",
}

RUNTIME_MODE_ALIASES: frozenset[str] = frozenset({"field", "nav", "thunder-nav", "field-cpp", "dds-cpp"})


def service_metadata(
    *, camera_driver: str = "orbbec_native", environment: Mapping[str, str] | None = None
) -> dict[str, dict[str, Any]]:
    environment = environment or {}
    return {
        spec.name: (camera_service_spec(spec, camera_driver) if spec.name == "camera" else spec).metadata(environment)
        for spec in SERVICE_SPECS
    }


def service_spec(name: str) -> ServiceSpec | None:
    service = INSTALL_MODE_ALIASES.get(name, name)
    for spec in SERVICE_SPECS:
        if spec.name == service:
            return spec
    return None


def camera_service_spec(spec: ServiceSpec, driver: str) -> ServiceSpec:
    if driver not in {"orbbec_native", "realsense_native"}:
        raise ValueError(f"unsupported camera driver in readiness: {driver}")
    if driver == "orbbec_native":
        return spec
    return replace(spec, binaries=tuple(
        ("realsense_capture", "LINGTU_REALSENSE_CAPTURE_BIN", "/opt/lingtu/current/bin/realsense_capture")
        if binary[0] == "orbbec_capture" else binary
        for binary in spec.binaries
    ))


def service_install_unit(name: str) -> str:
    spec = service_spec(name)
    if spec is None or not spec.start_units:
        return ""
    return spec.start_units[0]


def service_install_enable_default(name: str) -> str:
    spec = service_spec(name)
    if spec is None:
        return ""
    return "1" if spec.install_enable_default else "0"


def install_services(mode: str = "field-cpp") -> tuple[str, ...]:
    """Logical service names to install for a product install mode."""
    mode = (mode or "field-cpp").strip()
    if mode in RUNTIME_MODE_ALIASES:
        return RUNTIME_INSTALL_ORDER
    service = INSTALL_MODE_ALIASES.get(mode, mode)
    spec = service_spec(service)
    if spec is None or not spec.start_units or spec.catalog_alias:
        return ()
    return (spec.name,)


def runtime_services() -> tuple[str, ...]:
    """Logical short service names expected in the native field stack."""
    return RUNTIME_INSTALL_ORDER


def runtime_units() -> tuple[str, ...]:
    """Concrete systemd units checked by read-only field evidence collectors."""
    return tuple(unit for name in runtime_services() if (unit := service_install_unit(name)))


def runtime_status_files() -> dict[str, str]:
    """Status files that prove product service readiness beyond systemd."""
    files: dict[str, str] = {}
    for name in runtime_services():
        spec = service_spec(name)
        if spec is None:
            continue
        for index, path in enumerate(spec.files):
            key = name if index == 0 else f"{name}_{index + 1}"
            files[key] = path
    return files


def runtime_native_binaries(*, camera_driver: str = "orbbec_native") -> dict[str, dict[str, str]]:
    """Native binaries that must exist for product field services."""
    binaries: dict[str, dict[str, str]] = {}
    for service in runtime_services():
        spec = service_spec(service)
        if spec is None:
            continue
        if service == "camera":
            spec = camera_service_spec(spec, camera_driver)
        for name, env, path in spec.binaries:
            binaries[name] = {
                "service": service,
                "env": env,
                "path": path,
            }
    return binaries


def runtime_dds_topics() -> dict[str, dict[str, tuple[str, ...]]]:
    """DDS topic contracts expected from field product services."""
    topics: dict[str, dict[str, tuple[str, ...]]] = {}
    for name in runtime_services():
        spec = service_spec(name)
        if spec is None or not spec.dds_topics:
            continue
        topics[name] = {
            "topics": spec.topics,
            "dds_topics": spec.dds_topics,
        }
    return topics


def service_groups() -> dict[str, list[str]]:
    collected: dict[str, list[str]] = {
        "native_dds": [],
        "host": [],
        "hardware": [],
    }
    for spec in SERVICE_SPECS:
        if spec.name.startswith("legacy_") or spec.catalog_alias:
            continue
        collected.setdefault(spec.group, []).append(spec.name)
    groups: dict[str, list[str]] = {}
    for group, preferred_order in GROUP_ORDER.items():
        present = set(collected.get(group, []))
        groups[group] = [name for name in preferred_order if name in present]
        groups[group].extend(name for name in collected.get(group, []) if name not in preferred_order)
    return groups


def slam_status_services() -> tuple[str, ...]:
    return (
        "lidar",
        "slam",
        "maps",
        "traversability",
        "nav",
        "explore",
    )


def _main(argv: list[str]) -> int:
    command = argv[1] if len(argv) > 1 else "install-services"
    mode = argv[2] if len(argv) > 2 else "field-cpp"
    if command == "install-services":
        services = install_services(mode)
        if not services:
            return 2
        for service in services:
            print(service)
        return 0
    if command == "install-modes":
        for mode_name in sorted(INSTALL_MODE_ALIASES):
            print(mode_name)
        for mode_name in sorted(RUNTIME_MODE_ALIASES):
            print(mode_name)
        return 0
    if command == "install-unit":
        unit = service_install_unit(mode)
        if not unit:
            return 2
        print(unit)
        return 0
    if command == "install-enable-default":
        default = service_install_enable_default(mode)
        if default == "":
            return 2
        print(default)
        return 0
    if command == "retired-units":
        spec = service_spec(mode)
        if spec is None:
            return 2
        for unit in spec.retired_units:
            print(unit)
        return 0
    if command == "readiness-services":
        for service in runtime_services():
            print(service)
        return 0
    if command == "readiness-units":
        for unit in runtime_units():
            print(unit)
        return 0
    if command == "status-files":
        for name, path in runtime_status_files().items():
            print(f"{name}={path}")
        return 0
    if command == "readiness-binaries":
        for name, item in runtime_native_binaries().items():
            print(f"{name}={item['service']}|{item['env']}|{item['path']}")
        return 0
    if command == "readiness-dds-topics":
        for name, contract in runtime_dds_topics().items():
            print(f"{name}={','.join(contract['dds_topics'])}")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv))
