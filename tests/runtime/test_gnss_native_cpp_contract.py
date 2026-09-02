from __future__ import annotations

from pathlib import Path

from runtime.service_catalogs.thunder import thunder_service_spec

GNSS_ROOT = Path("src/drivers/real/gnss")


def _read(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def test_gnss_real_tree_uses_native_impl_deps_layout() -> None:
    assert (GNSS_ROOT / "native" / "sdk.hpp").is_file()
    assert (GNSS_ROOT / "native" / "module.hpp").is_file()
    assert (GNSS_ROOT / "native" / "module.cpp").is_file()
    assert (GNSS_ROOT / "native" / "dds_module.hpp").is_file()
    assert (GNSS_ROOT / "native" / "dds_module.cpp").is_file()
    assert (GNSS_ROOT / "native" / "gnss_dds.cpp").is_file()
    assert (GNSS_ROOT / "impl" / "wtrtk980" / "nmea.cpp").is_file()
    assert (GNSS_ROOT / "impl" / "wtrtk980" / "serial.cpp").is_file()
    assert not (GNSS_ROOT / "deps").exists()
    assert not Path("src/drivers/adapters/ros2/gnss").exists()


def test_gnss_native_cpp_path_is_ros_free() -> None:
    blocked = (
        "rclcpp",
        "sensor_msgs",
        "nav_msgs",
        "geometry_msgs",
        "ament_",
        "package.xml",
        "ros2_node",
        "ros::",
    )
    checked = [
        GNSS_ROOT / "native" / "sdk.hpp",
        GNSS_ROOT / "native" / "module.hpp",
        GNSS_ROOT / "native" / "module.cpp",
        GNSS_ROOT / "native" / "dds_module.hpp",
        GNSS_ROOT / "native" / "dds_module.cpp",
        GNSS_ROOT / "native" / "gnss_dds.cpp",
        GNSS_ROOT / "impl" / "wtrtk980" / "nmea.hpp",
        GNSS_ROOT / "impl" / "wtrtk980" / "nmea.cpp",
        GNSS_ROOT / "impl" / "wtrtk980" / "serial.hpp",
        GNSS_ROOT / "impl" / "wtrtk980" / "serial.cpp",
        GNSS_ROOT / "CMakeLists.txt",
    ]

    offenders: list[str] = []
    for path in checked:
        text = path.read_text(encoding="utf-8")
        for token in blocked:
            if token in text:
                offenders.append(f"{path}:{token}")

    assert offenders == []


def test_gnss_native_dds_entrypoint_is_product_service() -> None:
    entry = _read("src/drivers/real/gnss/native/gnss_dds.cpp")
    module = _read("src/drivers/real/gnss/native/module.cpp")
    dds_module = _read("src/drivers/real/gnss/native/dds_module.cpp")
    cmake = _read("src/drivers/real/gnss/CMakeLists.txt")
    build = _read("scripts/build/build_gnss_dds.sh")
    run = _read("scripts/deploy/thunder/run_gnss_dds.sh")
    service = _read("scripts/deploy/thunder/lt-gnss.service")
    installer = _read("scripts/deploy/thunder/install_catalog_service.sh")
    catalog_entry = thunder_service_spec("gnss")

    assert '#include "native/dds_module.hpp"' in entry
    assert '#include "native/module.hpp"' in entry
    assert '#include "dds/dds.h"' not in entry
    assert '#include "message/cpp/topics.hpp"' in module
    assert '#include "dds/dds.h"' in dds_module
    assert '#include "message/cpp/qos.hpp"' in dds_module
    assert "lingtu_dds_GnssFix_desc" in dds_module
    assert "lingtu_dds_GnssStatus_desc" in dds_module
    assert "add_executable(lingtu_gnss_dds" in cmake
    assert "native/module.cpp" in cmake
    assert "native/dds_module.cpp" in cmake
    assert "CycloneDDS::ddsc" in cmake
    assert "lingtu_dds_contracts" in cmake
    assert "build/gnss_dds" in build
    assert "lingtu_gnss_dds" in build
    assert "LINGTU_GNSS_DDS_BIN" in run
    assert "LINGTU_GNSS_DEVICE" in service
    assert "LINGTU_GNSS_PUBLISH_ODOM=0" in service
    assert catalog_entry is not None
    assert catalog_entry.start_units == ("lt-gnss.service",)
    assert catalog_entry.retired_units == ("lingtu-gnss-dds.service",)
    assert 'catalog install-unit "${SERVICE}"' in installer
    assert 'catalog retired-units "${SERVICE}"' in installer


def test_native_service_is_the_only_gnss_runtime_implementation() -> None:
    retired_python_paths = (
        "src/localization/gnss_module.py",
        "src/localization/gnss_bridge.py",
        "src/localization/gnss_serial_driver.py",
        "src/localization/ntrip_client_module.py",
    )

    assert [path for path in retired_python_paths if Path(path).exists()] == []
