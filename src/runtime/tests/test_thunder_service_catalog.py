from __future__ import annotations


def test_thunder_catalog_exposes_only_the_native_slam_runtime():
    from runtime.service_catalogs.thunder import (
        thunder_service_groups,
        thunder_service_metadata,
        thunder_slam_status_services,
    )

    retired_slam_services = {
        "slam_pgo",
        "localizer",
        "legacy_slam",
        "legacy_localizer",
        "genz_icp",
        "hba",
    }
    assert retired_slam_services.isdisjoint(thunder_service_metadata())
    assert retired_slam_services.isdisjoint(thunder_slam_status_services())

    groups = thunder_service_groups()
    assert groups["native_dds"] == [
        "lidar",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "explore",
    ]
    assert groups["host"] == ["gateway", "lingtu"]
    assert groups["hardware"][:1] == ["camera"]
    assert "nav_dds" not in groups["native_dds"]

    metadata = thunder_service_metadata()
    assert metadata["nav"]["role"] == "navigation_runtime"
    assert metadata["nav"]["product_default"] is True
    assert metadata["nav"]["install_enable_default"] is False
    assert "nav_dds" not in metadata
    assert "host" not in metadata
    assert "legacy_lidar" not in metadata
    assert metadata["driver"]["role"] == "motion_output"
    assert metadata["driver"]["product_default"] is True
    assert metadata["driver"]["install_enable_default"] is True
    assert metadata["driver"]["checks"] == [
        "systemd",
        "native_binary",
        "status_file",
    ]
    assert metadata["driver"]["status_max_age_s"] == 3.0
    assert metadata["camera"]["optional"] is True
    assert metadata["camera"]["install_enable_default"] is False
    assert metadata["gnss"]["optional"] is True
    assert metadata["gnss"]["role"] == "sensor_input"
    assert metadata["gnss"]["ros2_compat"] is False
    assert metadata["traversability"]["install_enable_default"] is False
    assert retired_slam_services.isdisjoint(metadata)


def test_optional_super_lio_integration_is_not_in_the_active_service_catalog():
    from runtime.service_catalogs.thunder import (
        thunder_service_groups,
        thunder_service_metadata,
        thunder_service_spec,
        thunder_slam_status_services,
    )

    integration_names = {"super_lio", "super_lio_relocation"}
    metadata = thunder_service_metadata()

    assert integration_names.isdisjoint(metadata)
    assert "experimental" not in thunder_service_groups()
    assert integration_names.isdisjoint(thunder_slam_status_services())
    for name in metadata:
        spec = thunder_service_spec(name)
        assert spec is not None
        assert all("super_lio" not in unit and "super-lio" not in unit for unit in spec.units)



def test_thunder_catalog_declares_product_readiness_contracts():
    from runtime.runtime_interface import TOPICS
    from runtime.service_catalogs.thunder import thunder_service_metadata

    metadata = thunder_service_metadata()

    assert metadata["nav"]["retired_units"] == [
        "lingtu-nav-dds.service",
        "nav-dds.service",
        "nav.service",
    ]
    assert metadata["driver"]["retired_units"] == ["lingtu-driver.service"]
    assert metadata["camera"]["checks"] == ["systemd", "native_binary", "dds", "status_file"]
    assert metadata["camera"]["topics"] == [TOPICS.camera_info]
    assert metadata["camera"]["dds_topics"] == ["rt/camera/info"]
    assert metadata["camera"]["shm_topics"] == [
        TOPICS.camera_color,
        TOPICS.camera_depth,
        TOPICS.camera_info,
    ]
    assert metadata["camera"]["shm_channels"] == [
        "/lingtu_camera_color",
        "/lingtu_camera_depth",
        "/lingtu_camera_info",
    ]
    assert metadata["camera"]["files"] == ["/dev/shm/lingtu/camera_status.json"]
    assert metadata["camera"]["binaries"] == [
        {
            "name": "camera_dds",
            "env": "LINGTU_CAMERA_DDS_BIN",
            "path": "/opt/lingtu/current/bin/lingtu_camera_dds",
        },
        {
            "name": "orbbec_capture",
            "env": "LINGTU_ORBBEC_CAPTURE_BIN",
            "path": "/opt/lingtu/current/bin/orbbec_capture",
        },
    ]

    assert metadata["lidar"]["checks"] == ["systemd", "native_binary", "dds"]
    assert metadata["lidar"]["topics"] == [TOPICS.raw_lidar_points, TOPICS.raw_imu]
    assert metadata["lidar"]["dds_topics"] == ["rt/lidar/raw_frame", "rt/imu/raw"]
    assert metadata["lidar"]["binaries"] == [
        {
            "name": "livox_dds",
            "env": "LINGTU_LIVOX_BIN",
            "path": "/opt/lingtu/current/bin/livox_sdk2_stream",
        }
    ]

    assert metadata["slam"]["checks"] == ["systemd", "native_binary", "dds", "status_file"]
    assert metadata["slam"]["files"] == ["/tmp/lingtu_slam_status.json"]
    assert metadata["slam"]["binaries"] == [
        {
            "name": "slam_dds",
            "env": "LINGTU_SLAM_BIN",
            "path": "/opt/lingtu/current/bin/slamd",
        }
    ]

    assert metadata["maps"]["checks"] == [
        "systemd",
        "native_binary",
        "dds",
        "status_file",
    ]
    assert metadata["maps"]["topics"] == [
        TOPICS.maps_state,
        TOPICS.maps_live_cloud,
        TOPICS.maps_voxel_cloud,
        TOPICS.maps_local_collision,
        TOPICS.maps_accumulated_cloud,
        TOPICS.maps_occupancy,
        TOPICS.maps_elevation,
        TOPICS.maps_esdf,
        TOPICS.maps_scene,
    ]
    assert metadata["maps"]["dds_topics"] == [
        "rt/maps/state",
        "rt/maps/live_cloud",
        "rt/maps/voxel_cloud",
        "rt/maps/local_collision",
        "rt/maps/accumulated_cloud",
        "rt/maps/occupancy",
        "rt/maps/elevation",
        "rt/maps/esdf",
        "rt/maps/scene",
    ]
    assert metadata["maps"]["files"] == ["/dev/shm/lingtu/mapd_status.json"]
    assert metadata["maps"]["binaries"] == [
        {
            "name": "mapd",
            "env": "LINGTU_MAPD_BIN",
            "path": "/opt/lingtu/current/bin/mapd",
        }
    ]

    assert metadata["traversability"]["checks"] == [
        "systemd",
        "native_binary",
        "dds",
        "status_file",
    ]
    assert metadata["traversability"]["topics"] == [
        TOPICS.traversability,
        TOPICS.terrain_map,
        TOPICS.terrain_map_ext,
    ]
    assert metadata["traversability"]["dds_topics"] == [
        "rt/nav/traversability",
        "rt/nav/terrain_map",
        "rt/nav/terrain_map_ext",
    ]
    assert metadata["traversability"]["binaries"] == [
        {
            "name": "traversability_dds",
            "env": "LINGTU_TRAVERSABILITY_DDS_BIN",
            "path": "/opt/lingtu/current/bin/lingtu_traversability_dds",
        }
    ]

    assert metadata["nav"]["checks"] == ["systemd", "native_binary", "status_file"]
    assert metadata["nav"]["topics"] == []
    assert metadata["nav"]["dds_topics"] == []
    assert metadata["nav"]["files"] == ["/dev/shm/lingtu/nav_endpoint_status.json"]
    assert metadata["nav"]["binaries"] == [
        {
            "name": "nav_dds",
            "env": "LINGTU_NAV_DDS_BIN",
            "path": "/opt/lingtu/current/bin/navd",
        }
    ]
    assert metadata["explore"]["checks"] == [
        "systemd",
        "native_binary",
        "dds",
        "status_file",
    ]
    assert metadata["explore"]["topics"] == [
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
    ]
    assert metadata["explore"]["dds_topics"] == [
        "rt/slam/odometry",
        "rt/nav/exploration_snapshot",
    ]
    assert metadata["gateway"]["checks"] == ["systemd", "http"]



def test_thunder_install_services_and_runtime_order():
    from runtime.service_catalogs.thunder import (
        thunder_install_services,
        thunder_runtime_dds_topics,
        thunder_runtime_native_binaries,
        thunder_runtime_services,
        thunder_runtime_status_files,
        thunder_runtime_units,
        thunder_service_install_unit,
        thunder_service_metadata,
    )
    assert thunder_install_services("field-cpp") == (
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
    assert thunder_runtime_services() == (
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
    assert thunder_runtime_units() == (
        "lt-lidar.service",
        "lt-camera.service",
        "lt-slam.service",
        "lt-maps.service",
        "lt-terrain.service",
        "lt-nav.service",
        "lt-driver.service",
        "lt-explore.service",
        "lt-host.service",
    )
    assert thunder_runtime_status_files() == {
        "camera": "/dev/shm/lingtu/camera_status.json",
        "slam": "/tmp/lingtu_slam_status.json",
        "maps": "/dev/shm/lingtu/mapd_status.json",
        "traversability": "/dev/shm/lingtu/traversability_status.json",
        "nav": "/dev/shm/lingtu/nav_endpoint_status.json",
        "driver": "/dev/shm/lingtu/driver_status.json",
        "explore": "/dev/shm/lingtu/explore_status.json",
    }
    assert thunder_runtime_native_binaries() == {
        "livox_dds": {
            "service": "lidar",
            "env": "LINGTU_LIVOX_BIN",
            "path": "/opt/lingtu/current/bin/livox_sdk2_stream",
        },
        "camera_dds": {
            "service": "camera",
            "env": "LINGTU_CAMERA_DDS_BIN",
            "path": "/opt/lingtu/current/bin/lingtu_camera_dds",
        },
        "orbbec_capture": {
            "service": "camera",
            "env": "LINGTU_ORBBEC_CAPTURE_BIN",
            "path": "/opt/lingtu/current/bin/orbbec_capture",
        },
        "slam_dds": {
            "service": "slam",
            "env": "LINGTU_SLAM_BIN",
            "path": "/opt/lingtu/current/bin/slamd",
        },
        "mapd": {
            "service": "maps",
            "env": "LINGTU_MAPD_BIN",
            "path": "/opt/lingtu/current/bin/mapd",
        },
        "traversability_dds": {
            "service": "traversability",
            "env": "LINGTU_TRAVERSABILITY_DDS_BIN",
            "path": "/opt/lingtu/current/bin/lingtu_traversability_dds",
        },
        "nav_dds": {
            "service": "nav",
            "env": "LINGTU_NAV_DDS_BIN",
            "path": "/opt/lingtu/current/bin/navd",
        },
        "driver": {
            "service": "driver",
            "env": "LINGTU_DRIVER_BIN",
            "path": "/opt/lingtu/current/bin/lingtu_driver",
        },
        "explore_dds": {
            "service": "explore",
            "env": "LINGTU_EXPLORE_DDS_BIN",
            "path": "/opt/lingtu/current/bin/lingtu_explore_dds",
        },
    }
    assert thunder_runtime_dds_topics()["lidar"]["dds_topics"] == (
        "rt/lidar/raw_frame",
        "rt/imu/raw",
    )
    assert thunder_runtime_dds_topics()["camera"]["dds_topics"] == ("rt/camera/info",)
    camera = thunder_service_metadata()["camera"]
    assert camera["shm_topics"] == [
        "/camera/color/image_raw",
        "/camera/depth/image_raw",
        "/camera/color/camera_info",
    ]
    assert camera["shm_channels"] == [
        "/lingtu_camera_color",
        "/lingtu_camera_depth",
        "/lingtu_camera_info",
    ]
    assert thunder_runtime_dds_topics()["slam"]["dds_topics"] == (
        "rt/slam/odometry",
        "rt/slam/map_cloud",
        "rt/slam/localization_health",
    )
    assert thunder_runtime_dds_topics()["maps"]["dds_topics"] == (
        "rt/maps/state",
        "rt/maps/live_cloud",
        "rt/maps/voxel_cloud",
        "rt/maps/local_collision",
        "rt/maps/accumulated_cloud",
        "rt/maps/occupancy",
        "rt/maps/elevation",
        "rt/maps/esdf",
        "rt/maps/scene",
    )
    assert thunder_runtime_dds_topics()["traversability"]["dds_topics"] == (
        "rt/nav/traversability",
        "rt/nav/terrain_map",
        "rt/nav/terrain_map_ext",
    )
    assert "nav" not in thunder_runtime_dds_topics()
    assert thunder_runtime_dds_topics()["driver"]["dds_topics"] == (
        "rt/nav/cmd_vel",
        "rt/driver/control_state",
    )
    for retired_mode in ("dds", "dds-endpoint", "endpoint-only", "endpoint"):
        assert thunder_install_services(retired_mode) == ()
        assert thunder_service_install_unit(retired_mode) == ""
    assert thunder_install_services("lidar-dds") == ("lidar",)
    assert thunder_install_services("native-driver") == ("driver",)
    assert thunder_install_services("camera-dds") == ("camera",)
    assert thunder_install_services("gnss-dds") == ("gnss",)
    assert thunder_install_services("mapd") == ("maps",)
    assert thunder_install_services("explore-dds") == ("explore",)
    assert thunder_install_services("unknown") == ()

    assert thunder_service_install_unit("lidar") == "lt-lidar.service"
    assert thunder_service_install_unit("camera-dds") == "lt-camera.service"
    assert thunder_service_install_unit("gnss-dds") == "lt-gnss.service"
    assert thunder_service_install_unit("slam") == "lt-slam.service"
    assert thunder_service_install_unit("maps-dds") == "lt-maps.service"
    assert thunder_service_install_unit("traversability") == ("lt-terrain.service")
    assert thunder_service_install_unit("nav-dds") == "lt-nav.service"
    assert thunder_service_install_unit("driver") == "lt-driver.service"
    assert thunder_service_install_unit("lingtu") == "lt-host.service"
    assert thunder_service_install_unit("unknown") == ""

    from runtime.service_catalogs.thunder import thunder_service_install_enable_default

    assert thunder_service_install_enable_default("traversability") == "0"
    assert thunder_service_install_enable_default("camera-dds") == "0"
    assert thunder_service_install_enable_default("gnss-dds") == "0"
    assert thunder_service_install_enable_default("nav") == "0"
    assert thunder_service_install_enable_default("mapd") == "0"
    assert thunder_service_install_enable_default("driver") == "1"
    assert thunder_service_install_enable_default("unknown") == ""


def test_thunder_catalog_cli_exports_field_readiness_targets(capsys):
    from runtime.service_catalogs.thunder import _main

    assert _main(["thunder", "install-services", "field-cpp"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lidar",
        "camera",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "explore",
        "lingtu",
    ]

    assert _main(["thunder", "install-services", "camera-dds"]) == 0
    assert capsys.readouterr().out.splitlines() == ["camera"]

    assert _main(["thunder", "install-services", "unknown"]) == 2
    assert capsys.readouterr().out == ""

    assert _main(["thunder", "install-modes"]) == 0
    install_modes = capsys.readouterr().out.splitlines()
    for mode in (
        "camera-dds",
        "field-cpp",
        "gnss-dds",
        "lidar-dds",
        "native-driver",
        "nav-dds",
        "slam-dds",
        "traversability-dds",
    ):
        assert mode in install_modes

    assert _main(["thunder", "readiness-services"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lidar",
        "camera",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "explore",
        "lingtu",
    ]

    assert _main(["thunder", "readiness-units"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lt-lidar.service",
        "lt-camera.service",
        "lt-slam.service",
        "lt-maps.service",
        "lt-terrain.service",
        "lt-nav.service",
        "lt-driver.service",
        "lt-explore.service",
        "lt-host.service",
    ]

    assert _main(["thunder", "retired-units", "nav"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lingtu-nav-dds.service",
        "nav-dds.service",
        "nav.service",
    ]

    assert _main(["thunder", "status-files"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "camera=/dev/shm/lingtu/camera_status.json",
        "slam=/tmp/lingtu_slam_status.json",
        "maps=/dev/shm/lingtu/mapd_status.json",
        "traversability=/dev/shm/lingtu/traversability_status.json",
        "nav=/dev/shm/lingtu/nav_endpoint_status.json",
        "driver=/dev/shm/lingtu/driver_status.json",
        "explore=/dev/shm/lingtu/explore_status.json",
    ]

    assert _main(["thunder", "readiness-dds-topics"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lidar=rt/lidar/raw_frame,rt/imu/raw",
        "camera=rt/camera/info",
        "slam=rt/slam/odometry,rt/slam/map_cloud,rt/slam/localization_health",
        "maps=rt/maps/state,rt/maps/live_cloud,rt/maps/voxel_cloud,rt/maps/local_collision,rt/maps/accumulated_cloud,rt/maps/occupancy,rt/maps/elevation,rt/maps/esdf,rt/maps/scene",
        "traversability=rt/nav/traversability,rt/nav/terrain_map,rt/nav/terrain_map_ext",
        "driver=rt/nav/cmd_vel,rt/driver/control_state",
        "explore=rt/slam/odometry,rt/nav/exploration_snapshot",
    ]

    assert _main(["thunder", "readiness-binaries"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "livox_dds=lidar|LINGTU_LIVOX_BIN|/opt/lingtu/current/bin/livox_sdk2_stream",
        "camera_dds=camera|LINGTU_CAMERA_DDS_BIN|/opt/lingtu/current/bin/lingtu_camera_dds",
        "orbbec_capture=camera|LINGTU_ORBBEC_CAPTURE_BIN|/opt/lingtu/current/bin/orbbec_capture",
        "slam_dds=slam|LINGTU_SLAM_BIN|/opt/lingtu/current/bin/slamd",
        "mapd=maps|LINGTU_MAPD_BIN|/opt/lingtu/current/bin/mapd",
        "traversability_dds=traversability|LINGTU_TRAVERSABILITY_DDS_BIN|/opt/lingtu/current/bin/lingtu_traversability_dds",
        "nav_dds=nav|LINGTU_NAV_DDS_BIN|/opt/lingtu/current/bin/navd",
        "driver=driver|LINGTU_DRIVER_BIN|/opt/lingtu/current/bin/lingtu_driver",
        "explore_dds=explore|LINGTU_EXPLORE_DDS_BIN|/opt/lingtu/current/bin/lingtu_explore_dds",
    ]
