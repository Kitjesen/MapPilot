"""Thunder product service catalog.

This file is data-only on purpose. Service orchestration stays in runtime, while
driver packages can re-export this catalog for legacy import paths.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from typing import Any

from message.dds import dds_topic_name
from runtime.runtime_interface import TOPICS


@dataclass(frozen=True)
class ThunderServiceSpec:
    name: str
    role: str
    units: tuple[str, ...]
    group: str
    start_units: tuple[str, ...] = ()
    installer: str = ""
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
            "product_default": self.product_default,
            "install_enable_default": self.install_enable_default,
            "installer": self.installer,
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
    return tuple(dds_topic_name(topic, typed=True) for topic in topics)


THUNDER_SERVICE_SPECS: tuple[ThunderServiceSpec, ...] = (
    ThunderServiceSpec(
        name="lidar",
        role="sensor_input",
        units=(
            "lingtu-livox-dds.service",
            "robot-lidar.service",
            "lidar.service",
            "lidar",
        ),
        start_units=("lingtu-livox-dds.service",),
        installer="install_livox_dds_service.sh",
        group="native_dds",
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
        units=(
            "lingtu-driver.service",
            "driver.service",
            "driver",
        ),
        start_units=("lingtu-driver.service",),
        installer="install_driver_service.sh",
        group="native_dds",
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
        description="Native DDS to Brainstem motion driver.",
    ),
    ThunderServiceSpec(
        name="endpoint",
        role="hardware_endpoint",
        units=(
            "lingtu-thunder-dds-endpoint.service",
            "thunder-dds-endpoint.service",
            "dds-endpoint",
        ),
        start_units=("lingtu-thunder-dds-endpoint.service",),
        installer="install_dds_endpoint_service.sh",
        group="compatibility",
        optional=True,
        checks=("systemd",),
        description="Compatibility Python DDS endpoint bridge.",
    ),
    ThunderServiceSpec(
        name="camera",
        role="sensor_input",
        units=(
            "lingtu-camera-dds.service",
            "robot-camera.service",
            "camera.service",
            "camera",
        ),
        start_units=("lingtu-camera-dds.service",),
        installer="install_camera_dds_service.sh",
        group="hardware",
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
        units=(
            "lingtu-gnss-dds.service",
            "robot-gnss.service",
            "gnss.service",
            "gnss",
        ),
        start_units=("lingtu-gnss-dds.service",),
        installer="install_gnss_dds_service.sh",
        group="hardware",
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
        units=("lingtu-nav-dds.service", "nav-dds.service", "nav.service", "nav"),
        start_units=("lingtu-nav-dds.service",),
        installer="install_nav_dds_service.sh",
        group="native_dds",
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
        name="nav_dds",
        role="navigation_runtime",
        units=("lingtu-nav-dds.service", "nav-dds.service", "nav_dds"),
        start_units=("lingtu-nav-dds.service",),
        installer="install_nav_dds_service.sh",
        group="native_dds",
        catalog_alias=True,
        checks=("systemd", "native_binary", "status_file"),
        files=("/dev/shm/lingtu/nav_endpoint_status.json",),
        binaries=(
            (
                "nav_dds",
                "LINGTU_NAV_DDS_BIN",
                "/opt/lingtu/current/build/nav_endpoint/navd",
            ),
        ),
        description="Compatibility alias for the short nav service.",
    ),
    ThunderServiceSpec(
        name="traversability",
        role="terrain_runtime",
        units=(
            "lingtu-traversability-dds.service",
            "traversability-dds.service",
            "traversability.service",
            "traversability",
        ),
        start_units=("lingtu-traversability-dds.service",),
        installer="install_traversability_dds_service.sh",
        group="native_dds",
        product_default=True,
        install_enable_default=False,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(TOPICS.traversability, TOPICS.terrain_map, TOPICS.terrain_map_ext),
        dds_topics=_dds(TOPICS.traversability, TOPICS.terrain_map, TOPICS.terrain_map_ext),
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
        units=(
            "lingtu-explore-dds.service",
            "explore-dds.service",
            "explore.service",
            "explore",
        ),
        start_units=("lingtu-explore-dds.service",),
        installer="install_explore_dds_service.sh",
        group="native_dds",
        optional=True,
        checks=("systemd", "native_binary", "dds", "status_file"),
        topics=(
            TOPICS.odometry,
            TOPICS.exploration_snapshot,
            TOPICS.exploration_command,
            TOPICS.exploration_ack,
            TOPICS.nav_command_request,
            TOPICS.nav_command_ack,
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
        units=("lingtu.service", "gateway.service", "gateway"),
        start_units=("lingtu.service",),
        installer="install_lingtu_service.sh",
        group="runtime",
        product_default=True,
        checks=("systemd", "http"),
        description="LingTu Python runtime and Gateway process.",
    ),
    ThunderServiceSpec(
        name="lingtu",
        role="orchestrator_runtime",
        units=("lingtu.service", "lingtu"),
        start_units=("lingtu.service",),
        installer="install_lingtu_service.sh",
        group="runtime",
        product_default=True,
        checks=("systemd", "http"),
        description="LingTu Python runtime service.",
    ),
    ThunderServiceSpec(
        name="slam",
        role="localization_runtime",
        units=(
            "lingtu-slam-dds.service",
            "robot-fastlio2.service",
            "localization.service",
            "slam.service",
            "slam",
        ),
        start_units=("lingtu-slam-dds.service",),
        installer="install_slam_dds_service.sh",
        group="native_dds",
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
                "/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime",
            ),
        ),
        description="Native DDS SLAM/localization owner.",
    ),
    ThunderServiceSpec(
        name="slam_pgo",
        role="map_optimization",
        units=("robot-pgo.service", "slam_pgo.service", "slam_pgo"),
        group="legacy_ros2_compat",
        ros2_compat=True,
        description="Legacy ROS 2 pose-graph optimizer. Not part of native live localization.",
    ),
    ThunderServiceSpec(
        name="localizer",
        role="saved_map_relocalization",
        units=("robot-localizer.service", "localizer.service", "localizer"),
        start_units=("lingtu-slam-dds.service",),
        group="legacy_ros2_compat",
        ros2_compat=True,
        description="Compatibility saved-map localizer entry.",
    ),
    ThunderServiceSpec(
        name="legacy_lidar",
        role="sensor_input",
        units=("robot-lidar.service", "lidar.service", "lidar"),
        group="legacy_ros2_compat",
        ros2_compat=True,
        description="Explicit ROS 2 compatibility LiDAR owner.",
    ),
    ThunderServiceSpec(
        name="legacy_slam",
        role="localization_runtime",
        units=(
            "robot-fastlio2.service",
            "localization.service",
            "slam.service",
            "slam",
        ),
        group="legacy_ros2_compat",
        ros2_compat=True,
        description="Explicit ROS 2 compatibility Fast-LIO owner.",
    ),
    ThunderServiceSpec(
        name="legacy_localizer",
        role="saved_map_relocalization",
        units=("robot-localizer.service", "localizer.service", "localizer"),
        group="legacy_ros2_compat",
        ros2_compat=True,
        description="Explicit ROS 2 compatibility localizer owner.",
    ),
    ThunderServiceSpec(
        name="genz_icp",
        role="experimental_localization",
        units=(
            "robot-genz-icp.service",
            "genz_icp.service",
            "genz-icp.service",
            "genz_icp",
        ),
        group="experimental",
        experimental=True,
        description="Experimental GenZ ICP odometry/localization service.",
    ),
    ThunderServiceSpec(
        name="hba",
        role="map_optimization",
        units=("robot-hba.service", "hba.service", "hba"),
        group="experimental",
        experimental=True,
        description="Experimental HBA offline map optimizer. Not part of native live localization.",
    ),
    ThunderServiceSpec(
        name="super_lio",
        role="experimental_localization",
        units=(
            "robot-super-lio.service",
            "super_lio.service",
            "super-lio.service",
            "super_lio",
        ),
        group="experimental",
        experimental=True,
        description="Experimental Super-LIO localization service.",
    ),
    ThunderServiceSpec(
        name="super_lio_relocation",
        role="experimental_relocalization",
        units=(
            "robot-super-lio-relocation.service",
            "robot-super-lio-reloc.service",
            "super_lio_relocation.service",
            "super_lio_reloc.service",
            "super-lio-relocation.service",
            "super_lio_relocation",
            "super_lio_reloc",
        ),
        group="experimental",
        experimental=True,
        description="Experimental Super-LIO relocation service.",
    ),
)

THUNDER_GROUP_ORDER: dict[str, tuple[str, ...]] = {
    "native_dds": (
        "lidar",
        "slam",
        "traversability",
        "nav",
        "driver",
        "explore",
    ),
    "runtime": ("gateway", "lingtu"),
    "experimental": ("genz_icp", "hba", "super_lio", "super_lio_relocation"),
    "legacy_ros2_compat": ("slam_pgo", "localizer"),
    "hardware": ("camera", "gnss", "brainstem"),
}

THUNDER_FIELD_INSTALL_ORDER: tuple[str, ...] = (
    "lidar",
    "camera",
    "slam",
    "traversability",
    "nav",
    "driver",
    "lingtu",
)

THUNDER_INSTALL_MODE_ALIASES: dict[str, str] = {
    "lidar-dds": "lidar",
    "livox": "lidar",
    "livox-dds": "lidar",
    "dds": "endpoint",
    "dds-endpoint": "endpoint",
    "endpoint-only": "endpoint",
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
    "nav-dds": "nav",
    "cpp-nav": "nav",
    "traversability-dds": "traversability",
    "terrain-dds": "traversability",
    "explore-dds": "explore",
    "cpp-explore": "explore",
    "exploration-dds": "explore",
    "lingtu": "lingtu",
    "app": "lingtu",
    "runtime": "lingtu",
}

THUNDER_FIELD_MODE_ALIASES: frozenset[str] = frozenset({"field", "nav", "thunder-nav", "field-cpp", "dds-cpp"})


def thunder_service_specs() -> tuple[ThunderServiceSpec, ...]:
    return THUNDER_SERVICE_SPECS


def thunder_service_aliases() -> dict[str, tuple[str, ...]]:
    return {spec.name: spec.units for spec in THUNDER_SERVICE_SPECS}


def thunder_service_start_aliases() -> dict[str, tuple[str, ...]]:
    return {spec.name: spec.start_units for spec in THUNDER_SERVICE_SPECS if spec.start_units}


def thunder_service_metadata() -> dict[str, dict[str, Any]]:
    return {spec.name: spec.metadata() for spec in THUNDER_SERVICE_SPECS}


def thunder_service_installers() -> dict[str, str]:
    return {spec.name: spec.installer for spec in THUNDER_SERVICE_SPECS if spec.installer}


def thunder_service_spec(name: str) -> ThunderServiceSpec | None:
    service = THUNDER_INSTALL_MODE_ALIASES.get(name, name)
    for spec in THUNDER_SERVICE_SPECS:
        if spec.name == service:
            return spec
    return None


def thunder_service_install_unit(name: str) -> str:
    spec = thunder_service_spec(name)
    if spec is None:
        return ""
    units = spec.start_units or spec.units
    return units[0] if units else ""


def thunder_service_install_enable_default(name: str) -> str:
    spec = thunder_service_spec(name)
    if spec is None:
        return ""
    return "1" if spec.install_enable_default else "0"


def thunder_install_plan(mode: str = "field-cpp") -> tuple[str, ...]:
    mode = (mode or "field-cpp").strip()
    installers = thunder_service_installers()
    if mode in THUNDER_FIELD_MODE_ALIASES:
        return tuple(installers[name] for name in THUNDER_FIELD_INSTALL_ORDER)
    service = THUNDER_INSTALL_MODE_ALIASES.get(mode, mode)
    installer = installers.get(service)
    return (installer,) if installer else ()


def thunder_install_services(mode: str = "field-cpp") -> tuple[str, ...]:
    """Logical service names to install for a product install mode."""
    mode = (mode or "field-cpp").strip()
    if mode in THUNDER_FIELD_MODE_ALIASES:
        return THUNDER_FIELD_INSTALL_ORDER
    service = THUNDER_INSTALL_MODE_ALIASES.get(mode, mode)
    spec = thunder_service_spec(service)
    if spec is None or not spec.installer:
        return ()
    return (spec.name,)


def thunder_field_readiness_services() -> tuple[str, ...]:
    """Logical short service names expected in the native field stack."""
    return THUNDER_FIELD_INSTALL_ORDER


def thunder_field_readiness_units() -> tuple[str, ...]:
    """Concrete systemd units checked by read-only field evidence collectors."""
    return tuple(unit for name in thunder_field_readiness_services() if (unit := thunder_service_install_unit(name)))


def thunder_field_status_files() -> dict[str, str]:
    """Status files that prove product service readiness beyond systemd."""
    files: dict[str, str] = {}
    for name in thunder_field_readiness_services():
        spec = thunder_service_spec(name)
        if spec is None:
            continue
        for index, path in enumerate(spec.files):
            key = name if index == 0 else f"{name}_{index + 1}"
            files[key] = path
    return files


def thunder_field_native_binaries() -> dict[str, dict[str, str]]:
    """Native binaries that must exist for product field services."""
    binaries: dict[str, dict[str, str]] = {}
    for service in thunder_field_readiness_services():
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


def thunder_field_dds_topics() -> dict[str, dict[str, tuple[str, ...]]]:
    """DDS topic contracts expected from field product services."""
    topics: dict[str, dict[str, tuple[str, ...]]] = {}
    for name in thunder_field_readiness_services():
        spec = thunder_service_spec(name)
        if spec is None or not spec.dds_topics:
            continue
        topics[name] = {
            "topics": spec.topics,
            "dds_topics": spec.dds_topics,
        }
    return topics


def thunder_field_shm_channels() -> dict[str, dict[str, tuple[str, ...]]]:
    """Shared-memory channels expected from field product services."""
    channels: dict[str, dict[str, tuple[str, ...]]] = {}
    for name in thunder_field_readiness_services():
        spec = thunder_service_spec(name)
        if spec is None or not spec.shm_channels:
            continue
        channels[name] = {
            "topics": spec.shm_topics,
            "shm_channels": spec.shm_channels,
        }
    return channels


def thunder_service_groups() -> dict[str, list[str]]:
    collected: dict[str, list[str]] = {
        "native_dds": [],
        "runtime": [],
        "experimental": [],
        "legacy_ros2_compat": [],
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
        "traversability",
        "nav",
        "explore",
        "slam_pgo",
        "localizer",
        "genz_icp",
        "hba",
        "super_lio",
        "super_lio_relocation",
    )


def thunder_optimization_services() -> tuple[str, ...]:
    return tuple(spec.name for spec in THUNDER_SERVICE_SPECS if spec.role == "map_optimization")


def _main(argv: list[str]) -> int:
    command = argv[1] if len(argv) > 1 else "install-plan"
    mode = argv[2] if len(argv) > 2 else "field-cpp"
    if command == "install-plan":
        plan = thunder_install_plan(mode)
        if not plan:
            return 2
        for installer in plan:
            print(installer)
        return 0
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
        for mode_name in sorted(THUNDER_FIELD_MODE_ALIASES):
            print(mode_name)
        return 0
    if command == "install-unit":
        unit = thunder_service_install_unit(mode)
        if not unit:
            return 2
        print(unit)
        return 0
    if command == "installer":
        spec = thunder_service_spec(mode)
        if spec is None or not spec.installer:
            return 2
        print(spec.installer)
        return 0
    if command == "install-enable-default":
        default = thunder_service_install_enable_default(mode)
        if default == "":
            return 2
        print(default)
        return 0
    if command == "readiness-services":
        for service in thunder_field_readiness_services():
            print(service)
        return 0
    if command == "readiness-units":
        for unit in thunder_field_readiness_units():
            print(unit)
        return 0
    if command == "status-files":
        for name, path in thunder_field_status_files().items():
            print(f"{name}={path}")
        return 0
    if command == "readiness-binaries":
        for name, item in thunder_field_native_binaries().items():
            print(f"{name}={item['service']}|{item['env']}|{item['path']}")
        return 0
    if command == "readiness-dds-topics":
        for name, contract in thunder_field_dds_topics().items():
            print(f"{name}={','.join(contract['dds_topics'])}")
        return 0
    if command == "readiness-shm-channels":
        for name, contract in thunder_field_shm_channels().items():
            print(f"{name}={','.join(contract['shm_channels'])}")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv))
