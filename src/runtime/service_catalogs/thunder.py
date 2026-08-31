"""Thunder product service catalog.

This file is data-only on purpose. Service orchestration stays in runtime, while
driver packages can re-export this catalog for legacy import paths.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from typing import Any

from message.topics import dds_topic_name
from runtime.runtime_interface import TOPICS


@dataclass(frozen=True)
class ThunderServiceSpec:
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
    shm_channels: tuple[str, ...] = ()
    files: tuple[str, ...] = ()
    status_max_age_s: float | None = None
    binaries: tuple[tuple[str, str, str], ...] = ()
    description: str = ""

    def metadata(self) -> dict[str, Any]:
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
            "shm_channels": list(self.shm_channels),
            "files": list(self.files),
            "status_max_age_s": self.status_max_age_s,
            "binaries": [{"name": name, "env": env, "path": path} for name, env, path in self.binaries],
            "description": self.description,
        }


def _dds(*topics: str) -> tuple[str, ...]:
    return tuple(dds_topic_name(topic) for topic in topics)


THUNDER_SERVICE_SPECS: tuple[ThunderServiceSpec, ...] = (
    ThunderServiceSpec(
        name="lidar",
        role="sensor_input",
        units=("lt-lidar.service",),
        start_units=("lt-lidar.service",),
        group="native_dds",
        retired_units=(
            "lingtu-livox-dds.service",
            "robot-lidar.service",
            "lidar.service",
        ),
        product_default=True,
        checks=("systemd", "native_binary", "dds"),
        topics=(TOPICS.raw_lidar_points, TOPICS.raw_imu),
        dds_topics=_dds(TOPICS.raw_lidar_points, TOPICS.raw_imu),
        binaries=(
            (
                "livox_dds",
                "LINGTU_LIVOX_BIN",
                "/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream",
            ),
        ),
        description="Native Livox DDS LiDAR input.",
    ),
    ThunderServiceSpec(
        name="driver",
        role="motion_output",
        units=("lt-driver.service",),
        start_units=("lt-driver.service",),
        group="native_dds",
        retired_units=("lingtu-driver.service",),
        product_default=True,
        install_enable_default=True,
        checks=("systemd", "native_binary", "status_file"),
        topics=(TOPICS.cmd_vel, "/driver/control_state"),
        dds_topics=(_dds(TOPICS.cmd_vel)[0], "rt/driver/control_state"),
        files=("/dev/shm/lingtu/driver_status.json",),
        status_max_age_s=3.0,
        binaries=(
            (
                "driver",
                "LINGTU_DRIVER_BIN",
                "/opt/lingtu/current/build/driver/lingtu_driver",
            ),
        ),
        description="Native DDS motion driver using the RobotConfig-selected backend.",
    ),
    ThunderServiceSpec(
        name="camera",
        role="sensor_input",
        units=("lt-camera.service",),
        start_units=("lt-camera.service",),
        group="hardware",
        retired_units=("lingtu-camera-dds.service",),
        optional=True,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(TOPICS.camera_info,),
        dds_topics=_dds(TOPICS.camera_info),
        shm_topics=(TOPICS.camera_color, TOPICS.camera_depth, TOPICS.camera_info),
        shm_channels=(
            "/lingtu_camera_color",
            "/lingtu_camera_depth",
            "/lingtu_camera_info",
        ),
        files=("/dev/shm/lingtu/camera_status.json",),
        binaries=(
            (
                "camera_dds",
                "LINGTU_CAMERA_DDS_BIN",
                "/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
            ),
            (
                "orbbec_capture",
                "LINGTU_ORBBEC_CAPTURE_BIN",
                "/opt/lingtu/current/build/orbbec_native/orbbec_capture",
            ),
        ),
        description="Optional native camera service using SHM for frames and DDS for camera metadata.",
    ),
    ThunderServiceSpec(
        name="brainstem",
        role="hardware_bridge",
        units=("robot-brainstem.service", "brainstem.service", "brainstem"),
        group="hardware",
        description="Thunder brainstem bridge.",
    ),
    ThunderServiceSpec(
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
                "/opt/lingtu/current/build/gnss_dds/lingtu_gnss_dds",
            ),
        ),
        description="Optional native WTRTK-980 GNSS stream service.",
    ),
    ThunderServiceSpec(
        name="nav",
        role="navigation_runtime",
        units=("lt-nav.service",),
        start_units=("lt-nav.service",),
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
                "/opt/lingtu/current/build/nav_endpoint/navd",
            ),
        ),
        description="Native DDS navigation runtime.",
    ),
    ThunderServiceSpec(
        name="traversability",
        role="terrain_runtime",
        units=("lt-terrain.service",),
        start_units=("lt-terrain.service",),
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
                "/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds",
            ),
        ),
        description="Native DDS traversability producer.",
    ),
    ThunderServiceSpec(
        name="explore",
        role="exploration_runtime",
        units=("lt-explore.service",),
        start_units=("lt-explore.service",),
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
                "/opt/lingtu/current/build/nav_endpoint/lingtu_explore_dds",
            ),
        ),
        description=(
            "Native hierarchical exploration endpoint over identity-versioned "
            "rolling occupancy snapshots and typed navigation commands."
        ),
    ),
    ThunderServiceSpec(
        name="gateway",
        role="interface_runtime",
        units=("lt-host.service",),
        start_units=("lt-host.service",),
        group="host",
        retired_units=("lingtu.service",),
        product_default=True,
        checks=("systemd", "http"),
        description="LingTu Python runtime and Gateway process.",
    ),
    ThunderServiceSpec(
        name="lingtu",
        role="orchestrator_runtime",
        units=("lt-host.service",),
        start_units=("lt-host.service",),
        group="host",
        retired_units=("lingtu.service",),
        product_default=True,
        checks=("systemd", "http"),
        description="LingTu Python runtime service.",
    ),
    ThunderServiceSpec(
        name="slam",
        role="localization_runtime",
        units=("lt-slam.service",),
        start_units=("lt-slam.service",),
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
                "/opt/lingtu/current/build/slam_core/slamd",
            ),
        ),
        description="Native DDS SLAM/localization owner.",
    ),
    ThunderServiceSpec(
        name="maps",
        role="maps_runtime",
        units=("lt-maps.service",),
        start_units=("lt-maps.service",),
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
                "/opt/lingtu/current/build/maps/mapd",
            ),
        ),
        description="Native live maps, bounded scene, and map service runtime.",
    ),
)

THUNDER_GROUP_ORDER: dict[str, tuple[str, ...]] = {
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
    "hardware": ("camera", "gnss", "brainstem"),
}

THUNDER_RUNTIME_INSTALL_ORDER: tuple[str, ...] = (
    "lidar",
    "camera",
    "slam",
    "maps",
    "traversability",
    "nav",
    "driver",
    "explore",
    "lingtu",
)

THUNDER_INSTALL_MODE_ALIASES: dict[str, str] = {
    "lidar-dds": "lidar",
    "livox": "lidar",
    "livox-dds": "lidar",
    "driver-dds": "driver",
    "native-driver": "driver",
    "thunder-driver": "driver",
    "camera": "camera",
    "camera-dds": "camera",
    "native-camera": "camera",
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

THUNDER_RUNTIME_MODE_ALIASES: frozenset[str] = frozenset({"field", "nav", "thunder-nav", "field-cpp", "dds-cpp"})


def thunder_service_metadata() -> dict[str, dict[str, Any]]:
    return {spec.name: spec.metadata() for spec in THUNDER_SERVICE_SPECS}


def thunder_service_spec(name: str) -> ThunderServiceSpec | None:
    service = THUNDER_INSTALL_MODE_ALIASES.get(name, name)
    for spec in THUNDER_SERVICE_SPECS:
        if spec.name == service:
            return spec
    return None


def thunder_service_install_unit(name: str) -> str:
    spec = thunder_service_spec(name)
    if spec is None or not spec.start_units:
        return ""
    return spec.start_units[0]


def thunder_service_install_enable_default(name: str) -> str:
    spec = thunder_service_spec(name)
    if spec is None:
        return ""
    return "1" if spec.install_enable_default else "0"


def thunder_install_services(mode: str = "field-cpp") -> tuple[str, ...]:
    """Logical service names to install for a product install mode."""
    mode = (mode or "field-cpp").strip()
    if mode in THUNDER_RUNTIME_MODE_ALIASES:
        return THUNDER_RUNTIME_INSTALL_ORDER
    service = THUNDER_INSTALL_MODE_ALIASES.get(mode, mode)
    spec = thunder_service_spec(service)
    if spec is None or not spec.start_units or spec.catalog_alias:
        return ()
    return (spec.name,)


def thunder_runtime_services() -> tuple[str, ...]:
    """Logical short service names expected in the native field stack."""
    return THUNDER_RUNTIME_INSTALL_ORDER


def thunder_runtime_units() -> tuple[str, ...]:
    """Concrete systemd units checked by read-only field evidence collectors."""
    return tuple(unit for name in thunder_runtime_services() if (unit := thunder_service_install_unit(name)))


def thunder_runtime_status_files() -> dict[str, str]:
    """Status files that prove product service readiness beyond systemd."""
    files: dict[str, str] = {}
    for name in thunder_runtime_services():
        spec = thunder_service_spec(name)
        if spec is None:
            continue
        for index, path in enumerate(spec.files):
            key = name if index == 0 else f"{name}_{index + 1}"
            files[key] = path
    return files


def thunder_runtime_native_binaries() -> dict[str, dict[str, str]]:
    """Native binaries that must exist for product field services."""
    binaries: dict[str, dict[str, str]] = {}
    for service in thunder_runtime_services():
        spec = thunder_service_spec(service)
        if spec is None:
            continue
        for name, env, path in spec.binaries:
            binaries[name] = {
                "service": service,
                "env": env,
                "path": path,
            }
    return binaries


def thunder_runtime_dds_topics() -> dict[str, dict[str, tuple[str, ...]]]:
    """DDS topic contracts expected from field product services."""
    topics: dict[str, dict[str, tuple[str, ...]]] = {}
    for name in thunder_runtime_services():
        spec = thunder_service_spec(name)
        if spec is None or not spec.dds_topics:
            continue
        topics[name] = {
            "topics": spec.topics,
            "dds_topics": spec.dds_topics,
        }
    return topics


def thunder_service_groups() -> dict[str, list[str]]:
    collected: dict[str, list[str]] = {
        "native_dds": [],
        "host": [],
        "hardware": [],
    }
    for spec in THUNDER_SERVICE_SPECS:
        if spec.name.startswith("legacy_") or spec.catalog_alias:
            continue
        collected.setdefault(spec.group, []).append(spec.name)
    groups: dict[str, list[str]] = {}
    for group, preferred_order in THUNDER_GROUP_ORDER.items():
        present = set(collected.get(group, []))
        groups[group] = [name for name in preferred_order if name in present]
        groups[group].extend(name for name in collected.get(group, []) if name not in preferred_order)
    return groups


def thunder_slam_status_services() -> tuple[str, ...]:
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
        services = thunder_install_services(mode)
        if not services:
            return 2
        for service in services:
            print(service)
        return 0
    if command == "install-modes":
        for mode_name in sorted(THUNDER_INSTALL_MODE_ALIASES):
            print(mode_name)
        for mode_name in sorted(THUNDER_RUNTIME_MODE_ALIASES):
            print(mode_name)
        return 0
    if command == "install-unit":
        unit = thunder_service_install_unit(mode)
        if not unit:
            return 2
        print(unit)
        return 0
    if command == "install-enable-default":
        default = thunder_service_install_enable_default(mode)
        if default == "":
            return 2
        print(default)
        return 0
    if command == "retired-units":
        spec = thunder_service_spec(mode)
        if spec is None:
            return 2
        for unit in spec.retired_units:
            print(unit)
        return 0
    if command == "readiness-services":
        for service in thunder_runtime_services():
            print(service)
        return 0
    if command == "readiness-units":
        for unit in thunder_runtime_units():
            print(unit)
        return 0
    if command == "status-files":
        for name, path in thunder_runtime_status_files().items():
            print(f"{name}={path}")
        return 0
    if command == "readiness-binaries":
        for name, item in thunder_runtime_native_binaries().items():
            print(f"{name}={item['service']}|{item['env']}|{item['path']}")
        return 0
    if command == "readiness-dds-topics":
        for name, contract in thunder_runtime_dds_topics().items():
            print(f"{name}={','.join(contract['dds_topics'])}")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv))
