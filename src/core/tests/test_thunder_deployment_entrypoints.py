from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def _read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8-sig")


def test_canonical_thunder_deploy_script_uses_product_profile() -> None:
    text = _read("scripts/deploy/deploy_thunder.sh")

    assert "LINGTU_DEPLOY_PROFILE:-thunder-nav" in text
    assert 'lingtu.py "${PROFILE}" --daemon' in text
    assert "git reset --hard" not in text
    assert "git pull --ff-only" in text
    assert "scripts/deploy/thunder/runtime-env.sh" in text
    assert "LINGTU_ENDPOINT:=thunder_field" in text
    assert "LINGTU_ENDPOINT_TRANSPORT:=lcm" in text
    assert "LINGTU_ENDPOINT_CONTRACT:=thunder_field_lcm_v1" in text
    assert '[ -f "/opt/ros/humble/setup.bash" ]' not in text
    assert "SOURCE_ROS2=0" in text
    assert "ros2|sim_ros2|*-ros2|ros-compat|legacy" in text
    assert "lite|thunder-lite|basic|thunder-basic" in text


def test_legacy_s100p_deploy_script_is_only_a_wrapper() -> None:
    text = _read("scripts/deploy/deploy_s100p.sh")

    assert "deploy_thunder.sh" in text
    assert "exec " in text
    assert "git reset --hard" not in text
    assert "lingtu.py nav" not in text


def test_thunder_service_installer_defaults_to_lcm_endpoint() -> None:
    text = _read("scripts/deploy/thunder/install_services.sh")

    assert 'MODE="${1:-lcm-endpoint}"' in text
    assert "install_lcm_endpoint_service.sh" in text
    assert "install_lite_service.sh" in text
    assert "Usage: $0 [lcm-endpoint|lite|ros-compat]" in text
    assert "../s100p/install_services.sh" in text
    assert "ros2-env.sh" in text
    assert 'exec "${LEGACY_INSTALLER}" "${SCRIPT_DIR}/../s100p"' in text
    assert "ros-compat|legacy" in text


def test_thunder_ros2_env_helper_is_the_deployment_compat_boundary() -> None:
    text = _read("scripts/deploy/thunder/ros2-env.sh")

    assert "/opt/ros/${LINGTU_ROS_DISTRO}/setup.bash" in text
    assert "LINGTU_ROS_OVERLAY_SETUP" in text
    assert "RMW_IMPLEMENTATION" in text
    assert "Product entrypoints should use LingTu profiles" in text


def test_thunder_lite_runtime_env_has_no_ros_setup() -> None:
    text = _read("scripts/deploy/thunder/runtime-env.sh")

    assert "LINGTU_PROFILE:=thunder-lite" in text
    assert "LINGTU_MODULE_TRANSPORT:=local" in text
    assert "LINGTU_ENDPOINT:=thunder_lite" in text
    assert "LINGTU_ENDPOINT_TRANSPORT:=local" in text
    assert "LINGTU_ENDPOINT_CONTRACT:=" in text
    assert "LINGTU_SIMULATION_ONLY:=0" in text
    assert "LINGTU_ENABLE_ROBOT_DRIVER:=1" in text
    assert "LINGTU_COMMAND_OUTPUT_MODE:=local_driver" in text
    assert "LINGTU_HARDWARE_CONTROL_BOUNDARY:=module_graph_driver" in text
    assert "/opt/ros" not in text
    assert "ROS_DOMAIN_ID" not in text
    assert "RMW_IMPLEMENTATION" not in text


def test_thunder_lite_service_is_standalone_product_entrypoint() -> None:
    text = _read("scripts/deploy/thunder/lingtu-thunder-lite.service")

    assert "Description=LingTu Thunder Lite runtime" in text
    assert "thunder-runtime-env.sh" in text
    assert 'lingtu.py "${LINGTU_PROFILE}" --no-repl' in text
    assert "--daemon" not in text
    assert "ros2-env.sh" not in text
    assert "/opt/ros" not in text


def test_thunder_lite_service_installer_does_not_delegate_to_legacy_ros_services() -> None:
    text = _read("scripts/deploy/thunder/install_lite_service.sh")

    assert "thunder-runtime-env.sh" in text
    assert "lingtu-thunder-lite.service" in text
    assert "systemctl daemon-reload" in text
    assert "../s100p/install_services.sh" not in text
    assert "ros2-env.sh" not in text


def test_thunder_lcm_endpoint_service_is_standalone_product_entrypoint() -> None:
    text = _read("scripts/deploy/thunder/lingtu-thunder-lcm-endpoint.service")

    assert "Description=LingTu Thunder LCM endpoint" in text
    assert "thunder-runtime-env.sh" in text
    assert "LINGTU_PROFILE=thunder-nav" in text
    assert "LINGTU_ENDPOINT=thunder_field" in text
    assert "LINGTU_ENDPOINT_TRANSPORT=lcm" in text
    assert "LINGTU_ENDPOINT_CONTRACT=thunder_field_lcm_v1" in text
    assert "LINGTU_ENDPOINT_SOURCES=thunder_field" in text
    assert "LINGTU_ENDPOINT_SOURCE=" in text
    assert "LINGTU_ENDPOINT_JSONL_PATH=" in text
    assert "LINGTU_ENDPOINT_JSONL_COMMAND=" in text
    assert "LINGTU_ENDPOINT_JSONL_RATE_HZ=0" in text
    assert "LINGTU_ENABLE_ROBOT_DRIVER=0" in text
    assert "LINGTU_COMMAND_OUTPUT_MODE=endpoint_only" in text
    assert "LINGTU_HARDWARE_CONTROL_BOUNDARY=lcm_endpoint_source" in text
    assert 'sources="${LINGTU_ENDPOINT_SOURCES:-${LINGTU_ENDPOINT_SOURCE:-}}"' in text
    assert "run_lcm_endpoint_service.py" in text
    assert "--source" in text
    assert "--contract" in text
    assert "ros2-env.sh" not in text
    assert "/opt/ros" not in text


def test_thunder_lcm_endpoint_service_installer_is_no_ros() -> None:
    text = _read("scripts/deploy/thunder/install_lcm_endpoint_service.sh")

    assert "thunder-runtime-env.sh" in text
    assert "lingtu-thunder-lcm-endpoint.service" in text
    assert "systemctl daemon-reload" in text
    assert "../s100p/install_services.sh" not in text
    assert "ros2-env.sh" not in text


def test_legacy_service_templates_have_thunder_descriptions() -> None:
    for rel_path in (
        "scripts/deploy/s100p/lidar.service",
        "scripts/deploy/s100p/slam.service",
        "scripts/deploy/s100p/slam_pgo.service",
        "scripts/deploy/s100p/localizer.service",
        "scripts/deploy/s100p/super_lio.service",
        "scripts/deploy/s100p/super_lio_relocation.service",
    ):
        text = _read(rel_path)

        assert "Description=Thunder " in text
        assert "Description=S100P " not in text


def test_legacy_service_templates_source_deploy_compat_env() -> None:
    for rel_path in (
        "scripts/deploy/s100p/lidar.service",
        "scripts/deploy/s100p/slam.service",
        "scripts/deploy/s100p/slam_pgo.service",
        "scripts/deploy/s100p/localizer.service",
        "scripts/deploy/s100p/super_lio.service",
        "scripts/deploy/s100p/super_lio_relocation.service",
    ):
        text = _read(rel_path)

        assert "source /opt/lingtu/config/ros2-env.sh" in text
        assert "source /opt/ros/humble/setup.bash" not in text


def test_release_script_uses_thunder_product_wording() -> None:
    text = _read("scripts/deploy/cut_release.sh")

    assert "LingTu Thunder release" in text
    assert "S100P" not in text
    assert "鈥" not in text


def test_scripts_readme_uses_thunder_as_product_entrypoint() -> None:
    text = _read("scripts/README.md")

    assert "deploy/deploy_thunder.sh" in text
    assert "python lingtu.py thunder-nav" in text
    assert "S100P" not in text


def test_doctor_uses_compat_ros_and_profile_builder() -> None:
    path = ROOT / "scripts" / "diagnostics" / "doctor.py"
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    assert "core.blueprints.full_stack" not in imports
    assert "core.ros2_context" not in imports
    assert "compat.ros2.context" in imports
    assert "blueprint_for_resolved_profile" in text
    assert "LINGTU_DOCTOR_AUTOSTART_SLAM" in text


def test_rerun_live_uses_compat_ros_context() -> None:
    path = ROOT / "scripts" / "visualization" / "rerun_live.py"
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    assert "core.ros2_context" not in imports
    assert "compat.ros2.context" in imports
