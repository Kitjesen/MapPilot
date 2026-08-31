from __future__ import annotations

from pathlib import Path

import yaml

from lingtu.assembly.compiler import compile_run_plan
from runtime.config import load_config
from runtime.utils.calibration_check import run_calibration_check

ROOT = Path(__file__).resolve().parents[2]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8-sig")


def test_real_robot_config_selects_go2_by_network_interface() -> None:
    config = load_config(str(ROOT / "config" / "robots" / "unitree" / "go2" / "robot.yaml"))

    assert config.driver.backend == "go2"
    assert config.driver.network_interface
    assert config.driver.network_address == "192.168.123.18/24"
    assert config.driver.probe_ip == "192.168.123.161"
    assert config.driver.target == ""


def test_go2_sensor_products_accept_verified_slam_mount() -> None:
    config = load_config(str(ROOT / "config" / "robots" / "unitree" / "go2" / "robot.yaml"))

    report = run_calibration_check(config=config, require_slam=True)

    assert report.errors == []
    assert any("Camera calibration is unverified" in warning for warning in report.warnings)


def test_real_run_plan_owns_driver_backend_configuration() -> None:
    plan = compile_run_plan("teleop", "real", robot="unitree/go2")

    assert plan.env == "real"
    assert plan.native_process_environment["LINGTU_DRIVER_BACKEND"] == "go2"
    assert plan.native_process_environment["LINGTU_DRIVER_NETWORK_INTERFACE"]
    assert plan.native_process_environment["LINGTU_DRIVER_NETWORK_ADDRESS"] == "192.168.123.18/24"
    assert plan.native_process_environment["LINGTU_DRIVER_PROBE_IP"] == "192.168.123.161"
    assert plan.native_process_environment["LINGTU_DDS_NETWORK_INTERFACE"] == "eth0"
    assert (
        "<NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>" in (plan.native_process_environment["CYCLONEDDS_URI"])
    )

    driver = next(process for process in plan.processes if process.name == "driver")
    assert driver.target == "lt-driver.service"
    assert driver.lifecycle == "mode"


def test_generic_deployment_entrypoints_are_target_parameterized() -> None:
    deploy = _read("scripts/deploy/deploy_robot.sh")
    sync = _read("tools/deploy/sync_robot.ps1")

    assert "LINGTU_DRIVER_BACKEND" in deploy
    assert 'scripts/lingtu" --robot "${ROBOT}" --env real switch' in deploy
    assert 'if [ -z "${PRODUCT}" ]; then' in deploy
    assert "specify the Product as the first argument" in deploy
    assert deploy.index('if [ -z "${PRODUCT}" ]; then') < deploy.index('PYTHON_BIN="')
    assert "all_native_build_scripts" not in deploy
    assert "LINGTU_DEPLOY_PLAN_ONLY" in deploy
    assert "LINGTU_TARGET_HOST" in sync
    assert "LINGTU_TARGET_USER" in sync
    assert "192.168.123.18" not in deploy + sync
    assert "sunrise" not in (deploy + sync).lower()
    assert not (ROOT / "scripts" / "deploy" / "sync_sunrise.ps1").exists()


def test_native_driver_has_one_robot_interface_and_two_implementations() -> None:
    native = ROOT / "src" / "drivers" / "real" / "motion"
    interface = (native / "body.hpp").read_text(encoding="utf-8")
    go2 = (native / "robots" / "unitree" / "go2" / "go2.cpp").read_text(encoding="utf-8")
    doso = (native / "robots" / "doso" / "doso.cpp").read_text(encoding="utf-8")
    doso_header = (native / "robots" / "doso" / "doso.hpp").read_text(encoding="utf-8")
    cmake = (native / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "class Body" in interface
    assert "Result move(const Velocity &velocity)" in interface
    assert "Result stop() noexcept" in interface
    assert "Result act(BodyAction action)" in interface
    assert "Capabilities capabilities() const noexcept" in interface
    assert "bool confirmsStop() const noexcept" in interface
    assert "AdapterDiagnostics diagnostics() const" in interface
    assert "struct RpcResult" not in interface
    assert "makeBody" in go2
    assert "makeBody" in doso
    assert "class Doso final" in doso_header
    assert "class Go2 final" in go2
    assert "DosoBackend" not in doso_header + doso
    assert "Go2Backend" not in go2
    assert "ChannelFactory::Instance()->Init" in go2
    assert "SportClient" in go2
    assert "StopMove" in go2
    assert "Move(" in go2
    assert "client_->Move(static_cast<float>(velocity.vx_mps)" in go2
    assert "velocity.vx_mps *" not in go2
    assert "client_->StandUp()" in go2
    assert "client_->Sit()" in go2
    assert "client_->RecoveryStand()" in go2
    assert "client_->Damp()" in go2
    assert "brainstem::Client" in doso
    assert "client.standUp()" in doso
    assert "client.sitDown()" in doso
    assert "return {true, true, false, false};" in doso
    assert "mode == 8" not in go2
    assert 'set(LINGTU_DRIVER_BACKEND "" CACHE STRING' in cmake
    assert 'LINGTU_DRIVER_BACKEND STREQUAL "doso"' in cmake
    assert 'LINGTU_DRIVER_BACKEND STREQUAL "go2"' in cmake
    assert "find_package(unitree_sdk2" in cmake
    assert "motion_backend_factory.cpp" not in cmake


def test_driver_common_velocity_is_physical_and_never_normalized() -> None:
    core = _read("src/drivers/real/motion/core.cpp")
    core_header = _read("src/drivers/real/motion/core.hpp")
    interface = _read("src/drivers/real/motion/body.hpp")
    readme = _read("src/drivers/real/motion/README.md")

    assert "struct Velocity" in core_header
    assert "vx_mps" in core_header
    assert "vy_mps" in core_header
    assert "yaw_rps" in core_header
    assert "Result move(const Velocity &velocity)" in interface
    assert "value / maximum" not in core
    assert "normalize(" not in core
    assert "no normalized `[-1, 1]` representation" in readme


def test_fault_transition_stops_the_selected_backend_before_resetting_state() -> None:
    main = _read("src/drivers/real/motion/main.cpp")
    go2 = _read("src/drivers/real/motion/robots/unitree/go2/go2.cpp")
    fail_closed = main[main.index("auto fail_closed") : main.index("auto send")]
    go2_release = go2[go2.index("Result stop()") : go2.index("Result act(BodyAction action)")]

    assert fail_closed.index("stop_and_release") < fail_closed.index("core.reset()")
    assert "client_->StopMove()" in go2_release
    assert 'stopped ? "stop_confirmed"' in go2_release
    assert "response.stop_confirmed = stopped" in go2_release
    assert 'stats.control.reason = "released"' not in main


def test_driver_public_state_is_vendor_neutral() -> None:
    interface = _read("src/drivers/real/motion/body.hpp")
    status = _read("src/drivers/real/motion/status.cpp")
    dds = _read("src/drivers/real/motion/dds.cpp")

    control = interface[interface.index("struct ControlState") : interface.index("struct AdapterDiagnostics")]
    assert "grpc" not in control
    assert "sdk2" not in control
    assert "owner" not in control
    assert '\\"adapter\\"' in status
    assert '\\"last_velocity\\"' in status
    assert '\\"last_walk\\"' not in status
    assert '\\"brainstem\\"' not in status
    assert 'kDriverOwner[] = "driver"' in dds


def test_native_driver_tests_live_in_the_repository_test_tree() -> None:
    tests = ROOT / "tests" / "drivers" / "real" / "motion"

    assert (tests / "test_core.cpp").is_file()
    assert (tests / "test_output_ack.cpp").is_file()
    assert (tests / "test_status.cpp").is_file()
    assert (tests / "test_doso_io.cpp").is_file()
    assert not (tests / "test_thunder_io.cpp").exists()
    legacy = ROOT / "src" / "drivers" / "real" / "thunder" / "native"
    assert not any(path.is_file() for path in legacy.rglob("*"))


def test_driver_service_consumes_the_product_session_without_user_identity() -> None:
    unit = _read("scripts/deploy/thunder/lt-driver.service")
    runner = _read("scripts/deploy/thunder/run_driver.sh")

    assert "Description=LingTu native field driver" in unit
    assert "EnvironmentFile=/run/lingtu/session.env" in unit
    assert "require_product_session.sh driver" in unit
    assert "brainstem.env" not in unit
    assert "User=lingtu" in unit
    assert "Group=lingtu" in unit
    assert "LINGTU_DRIVER_BACKEND" in runner
    assert "LINGTU_DRIVER_NETWORK_INTERFACE" in runner
    assert "LINGTU_DRIVER_NETWORK_ADDRESS" in runner
    assert "LINGTU_DRIVER_PROBE_IP" in runner
    assert "ldd" in runner
    assert "sunrise" not in (unit + runner).lower()


def test_driver_readiness_contract_is_backend_neutral() -> None:
    projector = _read("src/nav/cpp/endpoint/nav/input/health.cpp")
    switch = _read("src/lingtu/real/switch.py")

    assert 'owner == "grpc"' not in projector
    assert "lingtu-driver@sunrise" not in projector
    assert 'driver.get("backend") != "thunder"' not in switch
    assert "lingtu-driver@sunrise" not in switch


def test_active_deployment_files_have_no_sunrise_account_or_home() -> None:
    active_files = (
        "scripts/deploy/cut_release.sh",
        "scripts/deploy/deploy_robot.sh",
        "tools/deploy/sync_robot.ps1",
        "scripts/deploy/thunder/configure_gateway_api_key.sh",
        "scripts/deploy/thunder/lt-driver.service",
        "scripts/deploy/thunder/lt-nav.service",
        "scripts/deploy/thunder/lt-slam.service",
        "scripts/deploy/thunder/lingtu-runtime.conf",
        "scripts/deploy/thunder/lt-host.service",
        "scripts/deploy/thunder/runtime-env.sh",
        "src/drivers/real/motion/robots/doso/doso.hpp",
        "src/lingtu/real/switch.py",
    )

    offenders = [relative for relative in active_files if "sunrise" in _read(relative).lower()]
    assert offenders == []


def test_products_do_not_own_robot_backend_or_deployment_host() -> None:
    for path in sorted((ROOT / "config" / "runtime_graph" / "products").glob("*.yaml")):
        product = yaml.safe_load(path.read_text(encoding="utf-8"))
        text = str(product)
        assert "go2" not in text
        assert "192.168.123.18" not in text
