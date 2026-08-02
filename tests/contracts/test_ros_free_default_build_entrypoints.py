"""Contracts for the default native build and release entrypoints."""

# ruff: noqa: S101 - contracts use asserts.

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]

DEFAULT_BUILD_ENTRYPOINTS = (
    "Makefile",
    ".github/workflows/slam-aarch64-build.yml",
    ".github/workflows/release-navigation.yml",
    "docker/Dockerfile",
    "docker/Dockerfile.dev",
    "docker-compose.yml",
    "docker-compose.dev.yml",
)

FORBIDDEN_DEFAULT_BUILD_MARKERS = (
    "/opt/ros",
    "colcon",
    "rosdep",
    "ros-tooling/setup-ros",
    "from ros:",
    "ros2 topic",
    "rmw_implementation",
    "scripts/deploy/s100p/",
    "scripts/ota/",
)


def test_legacy_lingtu_shell_is_physically_absent() -> None:
    """The retired root shell must not survive as an executable tombstone."""

    canonical_shell = ROOT / "scripts" / "lingtu"
    legacy_shell = ROOT / "scripts" / ("lingtu" + ".sh")

    assert canonical_shell.is_file()
    assert not legacy_shell.exists()

def test_field_doctor_requires_explicit_ros2_compatibility_mode() -> None:
    """Default doctor diagnostics must stay on the native Product path."""

    content = (ROOT / "scripts/lingtu").read_text(encoding="utf-8", errors="replace")
    implementation = (ROOT / "src/diagnostics/field/doctor.py").read_text(
        encoding="utf-8",
        errors="replace",
    )
    doctor = content.split("cmd_doctor() {", maxsplit=1)[1].split(
        "# -- Subcommand: soak --",
        maxsplit=1,
    )[0]

    assert "ros2=0" in doctor
    assert "--ros2)" in doctor
    assert '[ "$ros2" = "1" ] && source_robot_env' in doctor
    assert doctor.count("source_robot_env") == 1
    assert '"$py" -m diagnostics.field.doctor' in doctor
    assert "ROS2 compatibility graph checks skipped; use --ros2" in implementation
    assert content.count("source_robot_env") == 2
    for forbidden in (
        "systemctl",
        "journalctl",
        "RunPlan.load",
        "Runtime" + "Manifest.load",
        "<<'PY'",
        "cmd_doctor_json",
    ):
        assert forbidden not in doctor


def test_real_runtime_evidence_defaults_to_gateway_collector() -> None:
    """ROS2 evidence collection must require an explicit compatibility flag."""

    content = (ROOT / "scripts/gates/real_runtime_evidence_collect.py").read_text(
        encoding="utf-8",
        errors="replace",
    )
    collector_argument = content.split(
        'parser.add_argument(\n        "--collector"',
        maxsplit=1,
    )[1].split("parser.add_argument", maxsplit=1)[0]
    dispatch = content.split("def run_collect(", maxsplit=1)[1].split(
        "def build_unavailable_real_runtime_report(",
        maxsplit=1,
    )[0]

    assert 'choices=["gateway", "ros2"]' in collector_argument
    assert 'default="gateway"' in collector_argument
    assert 'if args.collector == "gateway":' in dispatch
    assert 'if args.collector == "ros2":' in dispatch
    assert "--collector ros2" in content


def test_default_build_release_and_container_entrypoints_are_ros_free() -> None:
    """Developers, CI, releases, and containers must share the native path."""

    violations: list[str] = []
    for relative_path in DEFAULT_BUILD_ENTRYPOINTS:
        content = (ROOT / relative_path).read_text(encoding="utf-8", errors="replace").lower()
        for marker in FORBIDDEN_DEFAULT_BUILD_MARKERS:
            if marker in content:
                violations.append(f"{relative_path}: {marker}")

    assert violations == [], "default build surfaces still expose ROS compatibility:\n" + "\n".join(violations)


def test_makefile_exposes_the_native_product_workflow() -> None:
    """The root developer interface must build and operate the native Product."""

    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")

    for native_builder in (
        "scripts/build/build_native_runtime.sh",
        "scripts/build/build_livox_sdk2_stream.sh",
        "scripts/build/build_slam_core.sh",
        "scripts/build/build_dds_probe.sh",
        "scripts/build/build_nav_endpoint.sh",
        "scripts/build/build_driver.sh",
    ):
        assert native_builder in makefile

    assert "if [ -f scripts/build/build_mapd.sh ]" in makefile
    assert "$(PYTHON) -m pytest" in makefile
    assert "scripts/deploy/thunder/install_services.sh field-cpp" in makefile
    assert "$(PYTHON) -m lingtu.control switch map" in makefile
    assert "$(PYTHON) -m lingtu.control switch nav" in makefile
    assert '--map "$(MAP)"' in makefile


def test_active_ci_and_tag_release_use_native_aarch64_artifacts() -> None:
    """Active automation must validate and publish the native field runtime."""

    slam_workflow = (ROOT / ".github/workflows/slam-aarch64-build.yml").read_text(encoding="utf-8")
    release_workflow = (ROOT / ".github/workflows/release-navigation.yml").read_text(encoding="utf-8")
    package_script = ROOT / "scripts/deploy/package_native_release.sh"
    installer_script = ROOT / "scripts/deploy/install_native_release.sh"

    assert "runs-on: ubuntu-22.04-arm" in slam_workflow
    assert "src/maps/include/**" in slam_workflow
    assert "third_party/research_localization/small_gicp/include/**" in slam_workflow
    assert "bash scripts/build/build_slam_core.sh" in slam_workflow
    assert "runs-on: ubuntu-22.04-arm" in release_workflow
    assert "make build PYTHON=python3" in release_workflow
    assert "bash scripts/deploy/package_native_release.sh --self-test" in release_workflow
    assert "dist/*native-release*" in release_workflow
    assert package_script.is_file()
    assert installer_script.is_file()
    package_script_text = package_script.read_text(encoding="utf-8")
    installer_script_text = installer_script.read_text(encoding="utf-8")
    assert "set -euo pipefail" in package_script_text
    assert "build/dds_probe/lingtu_dds_probe" in package_script_text
    assert "git -C \"${ROOT}\" ls-files --error-unmatch" in package_script_text
    assert "build/maps/mapd" in package_script_text
    assert "build/maps/liblingtu_maps.so" in package_script_text
    assert 'INSTALLER_SOURCE="${SCRIPT_ROOT}/scripts/deploy/install_native_release.sh"' in package_script_text
    assert "Active Product requires maps/mapd" in installer_script_text
    assert "RunPlan.load" in installer_script_text
    assert "plan.processes" in installer_script_text
    assert 'process.get("lifecycle", "")' not in installer_script_text
    assert 'product_control "${TARGET_DIR}" restart "${process}"' in installer_script_text
    assert 'product_control "${TARGET_DIR}" reapply' in installer_script_text
    assert 'product_control "${OLD_TARGET}" reapply' in installer_script_text
    assert '--state-dir "${STATE_DIR}"' in installer_script_text
    assert 'product_control "${TARGET_DIR}" apply' not in installer_script_text
    assert 'product_control "${OLD_TARGET}" apply' not in installer_script_text
    assert "restore_previous_release" in installer_script_text
    assert "python3 -m lingtu.control" in installer_script_text
    assert "systemctl" not in installer_script_text
    assert 'git -C "${ROOT}" ls-files -z' in package_script_text
    assert "src/localization/fastlio2/config/self-test.yaml" in package_script_text
    assert '"install_script": "install_nav.sh"' in package_script_text


def test_default_containers_reuse_the_native_build_seam() -> None:
    """Container adapters must reuse the same native build and Gateway checks."""

    dockerfile = (ROOT / "docker/Dockerfile").read_text(encoding="utf-8")
    dev_dockerfile = (ROOT / "docker/Dockerfile.dev").read_text(encoding="utf-8")
    compose = (ROOT / "docker-compose.yml").read_text(encoding="utf-8")
    compose_dev = (ROOT / "docker-compose.dev.yml").read_text(encoding="utf-8")

    assert "RUN make build PYTHON=python3" in dockerfile
    assert "/app/build/dds_probe" in dockerfile
    assert "'.[gateway]'" in dockerfile
    assert "http://127.0.0.1:5050/health" in dockerfile
    assert "'/tmp/lingtu[dev]'" in dev_dockerfile
    assert "http://127.0.0.1:5050/health" in compose
    assert "network_mode: host" not in compose
    assert "devices:" not in compose
    assert "cap_add:" not in compose
    assert "/dev/ttyUSB0" not in compose
    assert "/dev/ttyACM0" not in compose
    assert "NET_ADMIN" not in compose
    assert "ENABLE_LIDAR" not in compose
    assert "SETUP_LIDAR_NET" not in compose
    compose_dev_model = yaml.safe_load(compose_dev)
    assert compose_dev_model["services"]["build-only"]["command"] == ["make", "build", "PYTHON=python3"]
