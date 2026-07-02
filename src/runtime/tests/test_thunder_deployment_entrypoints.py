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
    assert "LINGTU_ENDPOINT_TRANSPORT:=dds" in text
    assert "LINGTU_ENDPOINT_CONTRACT:=thunder_field_lcm_v1" not in text
    assert '[ -f "/opt/ros/humble/setup.bash" ]' not in text
    assert "SOURCE_ROS2=0" in text
    assert "ros2|sim_ros2|*-ros2|ros-compat|legacy" in text
    assert "lite|thunder-lite|basic|thunder-basic" in text
    assert "Building production nav kernel" in text
    assert 'bash "${REPO}/scripts/build/build_nav_kernel.sh" --clean' in text
    assert 'require_nav_kernel(context="Thunder deployment")' in text


def test_legacy_s100p_deploy_script_is_only_a_wrapper() -> None:
    text = _read("scripts/deploy/deploy_s100p.sh")

    assert "deploy_thunder.sh" in text
    assert "exec " in text
    assert "git reset --hard" not in text
    assert "lingtu.py nav" not in text


def test_thunder_service_installer_defaults_to_dds_endpoint() -> None:
    text = _read("scripts/deploy/thunder/install_services.sh")

    assert 'MODE="${1:-dds-endpoint}"' in text
    assert "install_dds_endpoint_service.sh" in text
    assert "install_slam_dds_service.sh" in text
    assert "slam-dds|cpp-slam" in text
    assert "field-cpp|dds-cpp" in text
    assert "install_lcm_endpoint_service.sh" in text
    assert "install_lite_service.sh" in text
    assert "Usage: $0 [dds-endpoint|slam-dds|nav-dds|field-cpp|lcm-endpoint|lite|ros-compat]" in text
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


def test_thunder_dds_endpoint_service_is_standalone_product_entrypoint() -> None:
    text = _read("scripts/deploy/thunder/lingtu-thunder-dds-endpoint.service")

    assert "Description=LingTu Thunder typed DDS endpoint" in text
    assert "thunder-runtime-env.sh" in text
    assert "LINGTU_PROFILE=thunder-nav" in text
    assert "LINGTU_ENDPOINT=thunder_field" in text
    assert "LINGTU_ENDPOINT_TRANSPORT=dds" in text
    assert "LINGTU_ENDPOINT_CONTRACT=thunder_field_dds_v1" in text
    assert "LINGTU_ENDPOINT_SOURCES=thunder_field" in text
    assert "LINGTU_ENDPOINT_SOURCE=" in text
    assert "LINGTU_ENDPOINT_JSONL_PATH=" in text
    assert "LINGTU_ENDPOINT_JSONL_COMMAND=" in text
    assert "LINGTU_ENDPOINT_JSONL_RATE_HZ=0" in text
    assert "LINGTU_BRAINSTEM_HOST=127.0.0.1" in text
    assert "LINGTU_BRAINSTEM_PORT=13145" in text
    assert "LINGTU_BRAINSTEM_REQUIRE_SDK=1" in text
    assert "LINGTU_BRAINSTEM_AUTO_ENABLE=0" in text
    assert "LINGTU_BRAINSTEM_AUTO_STANDUP=0" in text
    assert "LINGTU_BRAINSTEM_SAFE_SITDOWN=0" in text
    assert "LINGTU_BRAINSTEM_SAFE_DISABLE=0" in text
    assert "LINGTU_BRAINSTEM_CMD_TIMEOUT_MS=200" in text
    assert "LINGTU_ENABLE_ROBOT_DRIVER=0" in text
    assert "LINGTU_COMMAND_OUTPUT_MODE=endpoint_only" in text
    assert "LINGTU_HARDWARE_CONTROL_BOUNDARY=dds_endpoint_source" in text
    assert 'sources="${LINGTU_ENDPOINT_SOURCES:-${LINGTU_ENDPOINT_SOURCE:-}}"' in text
    assert "run_dds_endpoint_service.py" in text
    assert "--source" in text
    assert "--contract" in text
    assert "ros2-env.sh" not in text
    assert "/opt/ros" not in text


def test_thunder_dds_endpoint_service_installer_is_no_ros() -> None:
    text = _read("scripts/deploy/thunder/install_dds_endpoint_service.sh")

    assert "thunder-runtime-env.sh" in text
    assert "lingtu-thunder-dds-endpoint.service" in text
    assert "systemctl daemon-reload" in text
    assert "../s100p/install_services.sh" not in text
    assert "ros2-env.sh" not in text
    assert "lingtu-livox-driver.service" not in text


def test_thunder_slam_dds_service_runs_cpp_runtime() -> None:
    text = _read("scripts/deploy/thunder/lingtu-slam-dds.service")

    assert "Description=LingTu C++ CycloneDDS SLAM runtime" in text
    assert "lingtu-livox-dds.service" in text
    assert "robot-lidar.service" not in text
    assert "LINGTU_SLAM_BIN=/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime" in text
    assert "LINGTU_SLAM_BACKEND=fastlio2" in text
    assert "LINGTU_SLAM_MODE=mapping" in text
    assert "LINGTU_SLAM_CONFIG=/opt/lingtu/current/src/localization/fastlio2/config/mid360_s100p.yaml" in text
    assert "LINGTU_DDS_DOMAIN_ID=0" in text
    assert "LINGTU_SLAM_STATUS_JSON=/tmp/lingtu_slam_status.json" in text
    assert "--domain-id" in text
    assert "source /opt/lingtu/config/thunder-runtime-env.sh" not in text
    assert "source /opt/lingtu/config/ros2-env.sh" not in text
    assert "python" not in text.lower()
    assert "ros2 run fastlio2" not in text
    assert "rclcpp" not in text


def test_thunder_livox_dds_service_publishes_native_sdk2_stream() -> None:
    text = _read("scripts/deploy/thunder/lingtu-livox-dds.service")

    assert "Description=LingTu native Livox SDK2 DDS publisher" in text
    assert "LINGTU_LIVOX_BIN=/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream" in text
    assert "Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json" in text
    assert "--dds" in text
    assert "--domain-id" in text
    assert "--lidar-frame" in text
    assert "--imu-frame" in text
    assert "source /opt/lingtu/config/thunder-runtime-env.sh" in text
    assert "ros2-env.sh" not in text
    assert "livox_ros_driver2" not in text


def test_thunder_slam_dds_installer_is_explicit_cpp_slam_boundary() -> None:
    text = _read("scripts/deploy/thunder/install_slam_dds_service.sh")

    assert "lingtu-slam-dds.service" in text
    assert "lingtu-livox-dds.service" in text
    assert "thunder-runtime-env.sh" in text
    assert "ros2-env.sh" not in text
    assert "run_dds_endpoint_service.py" not in text
    assert "../s100p/install_services.sh" not in text


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
    assert "\u95c1" not in text


def test_release_script_does_not_gate_on_legacy_ros2_local_autonomy() -> None:
    text = _read("scripts/deploy/cut_release.sh")

    assert "ensure_nav_kernel_artifact" in text
    assert 'bash "$DEV_DIR/scripts/build/build_nav_kernel.sh" --clean' in text
    assert "bash scripts/build/build_octoplanner3d.sh" in text
    assert "LINGTU_RELEASE_REQUIRE_OCTOPLANNER3D:-1" in text
    assert "LINGTU_RELEASE_REQUIRE_ROS2_COMPAT:-0" in text
    assert "ROS 2 compatibility package gate skipped" in text
    assert "Build first: cd $DEV_DIR && colcon build" not in text
    assert "RUNTIME_PKGS=" not in text
    assert "lingtu-livox-driver.service" not in text
    assert "emit_release_services" in text

    compat_line = next(
        line for line in text.splitlines() if line.startswith("ROS2_COMPAT_PKGS=")
    )

    for legacy_pkg in (
        "local_planner",
        "sensor_scan_generation",
        "terrain_analysis",
        "terrain_analysis_ext",
        "pct_adapters",
        "pct_planner",
        "tare_planner",
    ):
        assert legacy_pkg not in compat_line

    assert "nav_kernel" not in compat_line
    assert "lingtu-thunder-dds-endpoint.service" in text
    assert "robot-fastlio2.service robot-localizer.service" in text


def test_ota_and_build_docs_do_not_recommend_legacy_ros2_planning_or_local_autonomy() -> None:
    push_script = _read("scripts/ota/push_to_robot.sh")
    package_script = _read("scripts/ota/build_nav_package.sh")
    ota_readme = _read("scripts/ota/README.md")
    build_guide = _read("docs/01-getting-started/BUILD_GUIDE.md")

    combined = "\n".join((push_script, package_script, ota_readme, build_guide))

    assert "--packages-select fastlio2 local_planner" not in combined
    assert "--packages-select local_planner" not in combined
    assert "--packages-select fastlio2 pct_planner" not in combined
    assert "--packages-select pct_planner" not in combined
    assert "pct_planner pct_adapters" not in combined
    assert "local_planner terrain_analysis terrain_analysis_ext" not in combined
    assert 'grep -E "fastlio2|local_planner|pct_planner' not in combined

    assert "bash scripts/build/build_nav_kernel.sh --clean" in build_guide
    assert "bash scripts/build/build_octoplanner3d.sh" in build_guide
    assert "Product default: native planner kernels, no ROS2" in build_guide
    assert "ROS 2 Humble Desktop is optional" in build_guide
    assert "make build           # source ROS Humble" not in build_guide
    assert "legacy colcon-based OTA helpers" in ota_readme
    assert "Use these OTA scripts only when intentionally" in ota_readme


def test_deployment_runbooks_prefer_gateway_dataflow_over_ros_topic_defaults() -> None:
    deployment_readme = _read("docs/04-deployment/README.md")
    cli_doc = _read("docs/04-deployment/lingtu_cli.md")
    replacement_map = _read("docs/architecture/ROS_ROLE_REPLACEMENT_MAP.md")

    assert "bash scripts/build/build_nav_kernel.sh --clean" in deployment_readme
    assert "bash scripts/build/build_octoplanner3d.sh" in deployment_readme
    assert "bash scripts/build/build_ros_workspace.sh" in deployment_readme
    assert "lingtu dataflow /nav/lidar_scan" in deployment_readme
    assert "lingtu dataflow /nav/odometry" in deployment_readme
    assert "lingtu doctor --ros2" in deployment_readme
    assert "ros2 topic hz /nav/lidar_scan" not in deployment_readme
    assert "ros2 topic hz /nav/odometry" not in deployment_readme
    assert "source /opt/ros/humble/setup.bash && colcon build" not in deployment_readme

    assert "lingtu doctor                   # read-only service/Gateway/dataflow diagnostics" in cli_doc
    assert "lingtu doctor --ros2" in cli_doc
    assert "Gateway readiness, health, localization, navigation, state, and camera snapshot" in cli_doc
    assert "publisher/subscriber counts for `/nav/lidar_scan`" not in cli_doc
    assert "scripts/deploy/cut_release.sh` is native-first" in replacement_map
    assert "Feishu and Telegram monitor bots collect status through Gateway endpoints" in replacement_map
    assert "legacy OTA/perception scripts as explicit" in replacement_map
    assert "compatibility paths rather than product defaults" in replacement_map


def test_scripts_readme_uses_thunder_as_product_entrypoint() -> None:
    text = _read("scripts/README.md")
    perception_readme = _read("scripts/perception/README.md")
    root_readme = _read("README.md")
    repo_layout = _read("docs/REPO_LAYOUT.md")

    assert "deploy/deploy_thunder.sh" in text
    assert "deploy/cut_release.sh" in text
    assert "python lingtu.py thunder-nav" in text
    assert "Legacy ROS OTA package and deploy" in text
    assert "ROS2 compatibility live perception demos" in text
    assert "Gateway camera endpoints" in perception_readme
    assert "S100P" not in text
    assert "Ubuntu 22.04 + ROS2 Humble" not in root_readme
    assert "ROS2 Humble is optional for compatibility services" in root_readme
    assert "colcon build / test / format / lint" not in repo_layout
    assert "native/test wrappers plus ROS workspace compatibility" in repo_layout
    assert "ROS2 compatibility perception demos" in repo_layout


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

    assert "runtime.blueprints.full_stack" not in imports
    assert "runtime.ros2_context" not in imports
    assert "runtime.adapters.ros2.context" in imports
    assert "run_gateway_dataflow_checks" in text
    assert "require_nav_kernel" in text
    assert "LingTu native navigation kernel" in text
    assert "use --ros2 or LINGTU_DOCTOR_ROS2=1" in text
    assert "blueprint_for_resolved_profile" in text
    assert "LINGTU_DOCTOR_AUTOSTART_SLAM" in text


def test_robot_ops_doctor_defaults_to_gateway_first_ros2_explicit() -> None:
    text = _read("scripts/lingtu")
    doctor_body = text.split("cmd_doctor() {", 1)[1].split("\n}\n\n# -- Subcommand: soak --", 1)[0]

    assert "[--ros2]" in text
    assert "--ros2) ros2=1" in text
    assert '[ "$ros2" = "1" ] && source_robot_env' in text
    assert '\n    source_robot_env\n' not in doctor_body
    assert 'cmd_doctor_json "$strict" "$non_motion" "$realtime" "$require_camera" "$ros2"' in text
    assert 'ros2_enabled = sys.argv[6] == "1"' in text
    assert "if ros2_enabled:" in text
    assert "ROS2 compatibility graph checks skipped; use --ros2" in text
    assert "camera.ros2_topics_skipped" in text
    assert "camera.gateway_snapshot" in text
    assert "$GW/api/v1/runtime/dataflow" in text
    assert "$GW/api/v1/camera/snapshot" in text
    assert '[ "$ros2" = "1" ] && command -v ros2' in text


def test_monitor_bots_are_gateway_backed_without_ros2_or_embedded_credentials() -> None:
    bot_paths = [
        "scripts/monitor/telegram_monitor_bot.py",
        "scripts/monitor/feishu_monitor_bot.py",
        "scripts/monitor/gateway_status.py",
    ]
    combined = "\n".join(_read(path) for path in bot_paths)

    for rel_path in bot_paths:
        path = ROOT / rel_path
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        imports: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module)
        assert "rclpy" not in imports
        assert "rclpy.node" not in imports
        assert "std_msgs.msg" not in imports

    assert "collect_gateway_status" in combined
    assert "/api/v1/navigation/status" in combined
    assert "/api/v1/health" in combined
    assert "/api/v1/localization/status" in combined
    assert "LINGTU_GATEWAY_URL" in combined
    assert "TELEGRAM_BOT_TOKEN" in combined
    assert "FEISHU_APP_SECRET" in combined
    assert "STATUS_TOPIC" not in combined
    assert "YOUR_BOT_TOKEN_HERE" not in combined
    assert "F4aHJepltjOioMCyDW0zWfvDwKrpdHeQ" not in combined


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

    assert "runtime.ros2_context" not in imports
    assert "runtime.adapters.ros2.context" in imports


def test_rerun_gateway_live_is_ros_free_gateway_viewer() -> None:
    path = ROOT / "scripts" / "visualization" / "rerun_gateway_live.py"
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    assert "rclpy" not in imports
    assert "runtime.adapters.ros2.context" not in imports
    assert "/api/v1/runtime/dataflow" in text
    assert "Gateway-backed Rerun live viewer" in text
