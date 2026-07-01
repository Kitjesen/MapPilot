from __future__ import annotations

import ast
import json
import os
import subprocess
import sys
from pathlib import Path

import yaml
from tools import package_thunder_lite as packager
from tools.validate import validate_thunder_lite_package as validator
from runtime.blueprints.profile_builder import blueprint_for_resolved_profile
from runtime.blueprints.stacks.autonomy_chain import autonomy_stack_config
from runtime.profiles.endpoints import resolve_runtime_run_spec
from runtime.profiles.resolver import canonical_profile_name, resolve_profile_config
from lingtu.plugin_seed import install_builtin_plugin_catalog

ROOT = Path(__file__).resolve().parents[3]
INTERNAL_PACKAGE_ROOTS = frozenset(
    {
        "cli",
        "runtime",
        "drivers",
        "gateway",
        "lingtu",
        "memory",
        "nav",
        "slam",
        "webrtc",
    }
)


def _python_module_name(rel_path: str) -> str | None:
    path = validator._normalize_manifest_path(rel_path)
    if not path.endswith(".py"):
        return None
    if path.startswith("src/"):
        path = path.removeprefix("src/")
    module = path[:-3].replace("/", ".")
    if module.endswith(".__init__"):
        module = module[: -len(".__init__")]
    return module or None


def _available_python_modules(copied_files: set[str]) -> set[str]:
    modules: set[str] = set()
    for rel_path in copied_files:
        module = _python_module_name(rel_path)
        if not module:
            continue
        parts = module.split(".")
        for index in range(1, len(parts) + 1):
            modules.add(".".join(parts[:index]))
    return modules


def _top_level_imports(path: Path, module_name: str) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports: set[str] = set()
    if path.name == "__init__.py":
        package_parts = module_name.split(".")
    else:
        package_parts = module_name.split(".")[:-1]

    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                base_parts = package_parts[: max(len(package_parts) - node.level + 1, 0)]
                if node.module:
                    base_parts.extend(node.module.split("."))
                if base_parts:
                    imports.add(".".join(base_parts))
            elif node.module:
                imports.add(node.module)
    return imports


def test_thunder_lite_package_manifest_names_minimal_runtime_surface() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))

    assert manifest["profile"] == "thunder-lite"
    assert manifest["runtime"]["expected_config"]["slam_profile"] == "none"
    assert manifest["runtime"]["expected_config"]["enable_gateway"] is False
    assert manifest["runtime"]["expected_config"]["enable_semantic"] is False
    assert manifest["runtime"]["expected_config"]["enable_map_modules"] is False
    assert manifest["runtime"]["expected_config"]["enable_gnss"] is False
    assert manifest["runtime"]["expected_spec"]["module_transport"] == "local"
    assert manifest["runtime"]["expected_spec"]["endpoint_transport"] == "local"
    assert manifest["runtime"]["expected_spec"]["simulation_only"] is False
    assert (
        manifest["runtime"]["expected_spec"]["command_sink"]
        == "hardware_driver_after_cmd_vel_mux"
    )
    assert "scripts/deploy/thunder/runtime-env.sh" in manifest["deploy"]["required_files"]


def test_thunder_lite_package_manifest_declares_package_boundary() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))
    package = manifest["package"]
    include_paths = {validator._normalize_manifest_path(path) for path in package["include_paths"]}
    exclude_paths = {validator._normalize_manifest_path(path) for path in package["exclude_paths"]}
    omit_paths = {validator._normalize_manifest_path(path) for path in package["omit_paths"]}

    assert {
        validator._normalize_manifest_path(path)
        for path in validator.REQUIRED_PACKAGE_INCLUDE_PATHS
    } <= include_paths
    assert {
        validator._normalize_manifest_path(path)
        for path in validator.REQUIRED_PACKAGE_EXCLUDE_PATHS
    } <= exclude_paths
    assert {
        validator._normalize_manifest_path(path)
        for path in validator.REQUIRED_PACKAGE_OMIT_PATHS
    } <= omit_paths
    assert "src/localization" not in include_paths
    assert "src/*/adapters/ros2" not in include_paths
    assert "config/robots/thunder.yaml" in include_paths


def test_pyproject_keeps_gateway_and_heavy_dependencies_out_of_core_install() -> None:
    base_dependencies, optional_dependencies = validator.load_pyproject_dependencies(ROOT / "pyproject.toml")
    base_names = {validator._dependency_name(dep) for dep in base_dependencies}

    assert "lite" in optional_dependencies
    assert optional_dependencies["lite"] == []
    assert {"fastapi", "uvicorn", "websockets"}.isdisjoint(base_names)
    assert {"fastapi", "uvicorn", "websockets"} <= {
        validator._dependency_name(dep) for dep in optional_dependencies["gateway"]
    }
    assert {"torch", "ultralytics", "chromadb", "openai", "anthropic"}.isdisjoint(base_names)


def test_requirements_lite_contains_only_minimal_python_runtime_dependencies() -> None:
    requirements = validator._read_requirements(ROOT / "requirements-lite.txt")
    names = {validator._dependency_name(dep) for dep in requirements}

    assert {"numpy", "scipy", "pyyaml", "pydantic"} <= names
    assert {"fastapi", "uvicorn", "websockets", "torch", "ultralytics", "chromadb"}.isdisjoint(names)


def test_runtime_env_defaults_match_thunder_lite_runtime_spec() -> None:
    defaults = validator._parse_shell_default_env(
        (ROOT / "scripts/deploy/thunder/runtime-env.sh").read_text(
            encoding="utf-8-sig"
        )
    )

    assert defaults["LINGTU_PROFILE"] == "thunder-lite"
    assert defaults["LINGTU_MODULE_TRANSPORT"] == "local"
    assert defaults["LINGTU_ENDPOINT"] == "thunder_lite"
    assert defaults["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert defaults["LINGTU_ENDPOINT_CONTRACT"] == ""
    assert defaults["LINGTU_SIMULATION_ONLY"] == "0"
    assert defaults["LINGTU_COMMAND_OUTPUT_MODE"] == "local_driver"
    assert defaults["LINGTU_HARDWARE_CONTROL_BOUNDARY"] == "module_graph_driver"


def test_thunder_lite_resolved_runtime_uses_lightweight_local_defaults() -> None:
    install_builtin_plugin_catalog()
    config = resolve_profile_config("thunder-lite")
    spec = resolve_runtime_run_spec(canonical_profile_name("thunder-lite"), config)
    autonomy_config = dict(config)
    enable_native = bool(autonomy_config.pop("enable_native", False))
    effective = autonomy_stack_config(enable_native, **autonomy_config)

    assert config["enable_native"] is False
    assert config["planner"] == "direct"
    assert config.get("local_planner_backend") is None
    assert config.get("path_follower_backend") is None
    assert config["python_autonomy_backend"] == "simple"
    assert config["python_path_follower_backend"] == "pid"
    assert effective["backend"] == "simple"
    assert effective["path_follower_backend"] == "pid"
    assert effective["backend"] not in {"nanobind", "cmu", "cmu_py"}
    assert effective["path_follower_backend"] not in {"nav_kernel", "pure_pursuit"}
    assert config["runtime_mode"] == "lite"
    assert config["slam_profile"] == "none"
    assert config["enable_gateway"] is False
    assert config["enable_semantic"] is False
    assert config["enable_map_modules"] is False
    assert config["enable_gnss"] is False
    assert spec.module_transport == "local"
    assert spec.endpoint_transport == "local"
    assert spec.localization_adapter is None


def test_resolved_thunder_lite_profile_builds_only_local_lite_control_graph() -> None:
    install_builtin_plugin_catalog()
    config = resolve_profile_config("thunder-lite")
    bp = blueprint_for_resolved_profile(canonical_profile_name("thunder-lite"), config)
    names = {entry.name for entry in bp._entries}
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert {
        "ThunderDriver",
        "nav.mission",
        "nav.terrain",
        "nav.local_planner",
        "nav.path_follower",
        "nav.safety",
        "GeofenceManagerModule",
        "nav.velocity_mux",
    } <= names
    assert {
        "SlamBridgeModule",
        "SLAMModule",
        "GnssModule",
        "GatewayModule",
        "MCPServerModule",
        "ExternalServiceManagerModule",
        "DeviceManager",
        "nav.in",
        "nav.out",
        "map.out",
        "SemanticPlannerModule",
        "nav.maps",
    }.isdisjoint(names)
    assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" in wires
    assert "nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel" not in wires


def test_thunder_lite_package_validator_rejects_ros_compat_graph_module() -> None:
    class FakeROSBridge:
        pass

    FakeROSBridge.__module__ = "runtime.adapters.ros2.fake_bridge"
    fake_entry = type(
        "FakeEntry",
        (),
        {"name": "FakeROSBridge", "module_cls": FakeROSBridge},
    )()
    fake_blueprint = type("FakeBlueprint", (), {"_entries": [fake_entry]})()
    blockers: list[str] = []

    validator._validate_product_graph("thunder-lite", fake_blueprint, blockers)

    assert any("contains ROS compatibility module" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_slam_package_graph_module() -> None:
    class FakeGnss:
        pass

    FakeGnss.__module__ = "localization.gnss_module"
    fake_entry = type(
        "FakeEntry",
        (),
        {"name": "GnssModule", "module_cls": FakeGnss},
    )()
    fake_blueprint = type("FakeBlueprint", (), {"_entries": [fake_entry]})()
    blockers: list[str] = []

    validator._validate_product_graph("thunder-lite", fake_blueprint, blockers)

    assert any("contains SLAM package module" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_lite_excluded_graph_module() -> None:
    class FakeGateway:
        pass

    FakeGateway.__module__ = "gateway.gateway_module"
    fake_entry = type(
        "FakeEntry",
        (),
        {"name": "GatewayModule", "module_cls": FakeGateway},
    )()
    fake_blueprint = type("FakeBlueprint", (), {"_entries": [fake_entry]})()
    manifest = yaml.safe_load(
        (ROOT / "config" / "thunder_lite_package.yaml").read_text(
            encoding="utf-8-sig"
        )
    )
    blockers: list[str] = []

    validator._validate_product_graph(
        "thunder-lite",
        fake_blueprint,
        blockers,
        manifest,
    )

    assert any("contains Lite-excluded package" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_missing_required_package_exclusion() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))
    manifest["package"]["exclude_paths"] = [
        path
        for path in manifest["package"]["exclude_paths"]
        if validator._normalize_manifest_path(path) != "src/localization"
    ]
    blockers: list[str] = []

    validator._validate_package_boundary(manifest, blockers, [])

    assert any("missing required Lite exclusions" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_missing_required_package_omit() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))
    manifest["package"]["omit_paths"] = [
        path
        for path in manifest["package"]["omit_paths"]
        if validator._normalize_manifest_path(path) != "src/runtime/dds.py"
    ]
    blockers: list[str] = []

    validator._validate_package_boundary(manifest, blockers, [])

    assert any("missing required Lite omissions" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_native_heavy_runtime_defaults() -> None:
    install_builtin_plugin_catalog()
    config = resolve_profile_config(
        "thunder-lite",
        enable_native=True,
        local_planner_backend="nanobind",
        path_follower_backend="nav_kernel",
    )
    spec = resolve_runtime_run_spec(canonical_profile_name("thunder-lite"), config)
    blockers: list[str] = []

    validator._validate_lite_runtime_defaults("thunder-lite", config, spec, blockers)

    assert any("enable_native must be false" in item for item in blockers)
    assert any("local_planner_backend must not use native/heavy backend" in item for item in blockers)
    assert any("path_follower_backend must not use native/heavy backend" in item for item in blockers)


def test_thunder_lite_packager_dry_run_applies_package_boundary(tmp_path: Path) -> None:
    summary = packager.build_package(
        output_dir=tmp_path / "thunder-lite",
        dry_run=True,
    )
    copied_files = set(summary["copied_files"])

    assert "lingtu.py" in copied_files
    assert "cli/lifecycle.py" in copied_files
    assert "cli/repl.py" not in copied_files
    assert "cli/runtime_audit.py" not in copied_files
    assert "cli/runtime_extra.py" not in copied_files
    assert "src/runtime/dds.py" not in copied_files
    assert "src/runtime/dimos_runtime_dataflow.py" not in copied_files
    assert "src/runtime/dynamic_filter.py" not in copied_files
    assert "src/runtime/external_service_module.py" not in copied_files
    assert "src/runtime/gateway_runtime_acceptance.py" not in copied_files
    assert "src/runtime/inspection_acceptance.py" not in copied_files
    assert "src/runtime/map_save.py" not in copied_files
    assert "src/runtime/native_install.py" not in copied_files
    assert "src/runtime/product_field_check.py" not in copied_files
    assert "src/runtime/runtime_evidence.py" not in copied_files
    assert "src/runtime/runtime_validation_gates.py" not in copied_files
    assert "src/runtime/same_source_map_artifacts.py" not in copied_files
    assert "src/runtime/blueprints/full_stack.py" not in copied_files
    assert "src/runtime/blueprints/full_stack_wiring.py" not in copied_files
    assert "src/runtime/blueprints/adapters/driver_ros2_runtime.py" not in copied_files
    assert "src/runtime/blueprints/adapters/mapping_slam.py" not in copied_files
    assert "src/runtime/blueprints/adapters/navigation_io.py" not in copied_files
    assert "src/runtime/blueprints/adapters/perception_gateway.py" not in copied_files
    assert "src/runtime/blueprints/stacks/slam.py" not in copied_files
    assert "src/runtime/blueprints/stacks/gateway.py" not in copied_files
    assert "src/runtime/blueprints/stacks/navigation.py" in copied_files
    assert "src/runtime/blueprints/stacks/navigation_core.py" in copied_files
    assert "src/runtime/blueprints/stacks/navigation_io.py" in copied_files
    assert "src/runtime/blueprints/stacks/autonomy_chain.py" in copied_files
    assert "src/runtime/blueprints/stacks/exploration_goal_sources.py" in copied_files
    assert "src/runtime/blueprints/wires/slam.py" not in copied_files
    assert "src/lingtu/ros2_plugin_seed.py" not in copied_files
    assert "src/lingtu/ros2_shutdown.py" not in copied_files
    assert "src/drivers/real/thunder/blueprints.py" not in copied_files
    assert "src/drivers/real/thunder/connection.py" not in copied_files
    assert "src/nav/services/plan/global_planner/service.py" not in copied_files
    assert "src/nav/services/plan/global_planner/service.py" not in copied_files
    assert "src/nav/lite_planner_backend.py" not in copied_files
    assert "src/nav/plan_safety.py" not in copied_files
    assert "src/nav/mission/navigation.py" in copied_files
    assert "src/nav/mission/tracking/waypoint_tracker.py" in copied_files
    assert "src/nav/kernel/__init__.py" in copied_files
    assert "src/nav/kernel/loader.py" in copied_files
    assert "src/nav/kernel/paths.py" in copied_files
    assert "src/nav/services/plan/contracts.py" in copied_files
    assert "src/nav/services/plan/factory.py" in copied_files
    assert "src/nav/services/plan/global_planner/direct.py" in copied_files
    assert "src/nav/services/plan/global_planner/algorithm/direct_path.py" in copied_files
    assert "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/Dockerfile.build" not in copied_files
    assert "src/nav/mission/model/frame_contract.py" in copied_files
    assert "src/nav/mission/tracking/waypoint_tracker.py" in copied_files
    assert "src/nav/services/safety/safety_ring.py" in copied_files
    assert "src/nav/services/safety/velocity_mux.py" in copied_files
    assert not any(path.startswith("src/runtime/devices/") for path in copied_files)
    assert not any(path.startswith("src/localization/") for path in copied_files)
    assert not any("/adapters/ros2/" in f"/{path}/" for path in copied_files)
    assert not any("/tests/" in f"/{path}/" for path in copied_files)


def test_thunder_lite_packager_exclude_paths_support_globs() -> None:
    assert packager._is_excluded(
        "src/nav/adapters/ros2/bridge.py",
        ("src/*/adapters/ros2/",),
    )
    assert not packager._is_excluded(
        "src/nav/adapters/local/bridge.py",
        ("src/*/adapters/ros2/",),
    )


def test_thunder_lite_packager_dry_run_top_level_imports_resolve_inside_package(
    tmp_path: Path,
) -> None:
    summary = packager.build_package(
        output_dir=tmp_path / "thunder-lite",
        dry_run=True,
    )
    copied_files = set(summary["copied_files"])
    available_modules = _available_python_modules(copied_files)
    unresolved: list[str] = []

    for rel_path in sorted(copied_files):
        module_name = _python_module_name(rel_path)
        if not module_name:
            continue
        for imported in sorted(_top_level_imports(ROOT / rel_path, module_name)):
            root = imported.split(".", 1)[0]
            if root not in INTERNAL_PACKAGE_ROOTS:
                continue
            if imported not in available_modules:
                unresolved.append(f"{rel_path}: {imported}")

    assert unresolved == []


def test_thunder_lite_packager_dry_run_excludes_ros_fallback_strings(tmp_path: Path) -> None:
    summary = packager.build_package(
        output_dir=tmp_path / "thunder-lite",
        dry_run=True,
    )
    leaked = [
        path
        for path in summary["copied_files"]
        if path.endswith(".py")
        and "runtime.adapters.ros2" in (ROOT / path).read_text(encoding="utf-8-sig", errors="ignore")
    ]

    assert leaked == []


def test_thunder_lite_packager_audit_rejects_omitted_files() -> None:
    rel_test_file = "src/runtime/tests/test_thunder_lite_package_contract.py"
    blockers = packager._audit_package_summary(
        {"copied_files": [rel_test_file]},
        output_dir=ROOT / "artifacts" / "unused",
        exclude_paths=(),
        omit_patterns=("**/tests/",),
        forbidden_markers=(),
        dry_run=True,
    )

    assert blockers == [f"omitted path copied into package: {rel_test_file}"]


def test_thunder_lite_packager_dry_run_audits_forbidden_python_imports(
    tmp_path: Path,
    monkeypatch,
) -> None:
    source = tmp_path / "src" / "lite_entry.py"
    source.parent.mkdir(parents=True)
    source.write_text("import rclpy\n", encoding="utf-8")
    monkeypatch.setattr(packager, "ROOT_DIR", tmp_path)

    blockers = packager._audit_package_summary(
        {"copied_files": ["src/lite_entry.py"]},
        output_dir=tmp_path / "out",
        exclude_paths=(),
        omit_patterns=(),
        forbidden_markers=("rclpy",),
        dry_run=True,
    )

    assert blockers == ["src/lite_entry.py: imports forbidden runtime module rclpy"]


def test_thunder_lite_packager_builds_filtered_package(tmp_path: Path) -> None:
    output_dir = tmp_path / "thunder-lite"
    summary = packager.build_package(output_dir=output_dir)

    assert summary["dry_run"] is False
    assert summary["file_count"] > 0
    assert (output_dir / "lingtu.py").is_file()
    assert (output_dir / "cli" / "lifecycle.py").is_file()
    assert not (output_dir / "cli" / "repl.py").exists()
    assert not (output_dir / "cli" / "runtime_audit.py").exists()
    assert not (output_dir / "cli" / "runtime_extra.py").exists()
    assert (output_dir / "src" / "runtime" / "blueprint.py").is_file()
    assert not (output_dir / "src" / "core" / "gateway_runtime_acceptance.py").exists()
    assert not (output_dir / "src" / "core" / "inspection_acceptance.py").exists()
    assert not (output_dir / "src" / "core" / "product_field_check.py").exists()
    assert (output_dir / "src" / "drivers" / "real" / "thunder").is_dir()
    assert not (output_dir / "src" / "nav" / "global_planner.py").exists()
    assert not (
        output_dir / "src" / "nav" / "planning" / "global_planner.py"
    ).exists()
    assert not (output_dir / "src" / "nav" / "lite_planner_backend.py").exists()
    assert not (output_dir / "src" / "nav" / "plan_safety.py").exists()
    assert (output_dir / "src" / "nav" / "mission" / "navigation.py").is_file()
    assert (
        output_dir / "src" / "nav" / "mission" / "tracking" / "waypoint_tracker.py"
    ).is_file()
    assert (output_dir / "src" / "nav" / "services" / "plan" / "factory.py").is_file()
    assert (
        output_dir / "src" / "nav" / "services" / "plan" / "global_planner" / "direct.py"
    ).is_file()
    assert (
        output_dir
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "direct_path.py"
    ).is_file()
    assert (
        output_dir / "src" / "nav" / "mission" / "model" / "frame_contract.py"
    ).is_file()
    assert (output_dir / "src" / "nav" / "services" / "safety" / "safety_ring.py").is_file()
    assert (output_dir / "src" / "nav" / "services" / "safety" / "velocity_mux.py").is_file()
    assert not (output_dir / "src" / "core" / "blueprints" / "full_stack.py").exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "full_stack_wiring.py").exists()
    assert not (
        output_dir / "src" / "core" / "blueprints" / "adapters" / "driver_ros2_runtime.py"
    ).exists()
    assert not (
        output_dir / "src" / "core" / "blueprints" / "adapters" / "mapping_slam.py"
    ).exists()
    assert not (
        output_dir / "src" / "core" / "blueprints" / "adapters" / "navigation_io.py"
    ).exists()
    assert not (
        output_dir / "src" / "core" / "blueprints" / "adapters" / "perception_gateway.py"
    ).exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "stacks" / "localization.py").exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "stacks" / "gateway.py").exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "wires").exists()
    assert not (output_dir / "src" / "core" / "devices").exists()
    assert not (output_dir / "src" / "lingtu" / "ros2_plugin_seed.py").exists()
    assert not (output_dir / "src" / "lingtu" / "ros2_shutdown.py").exists()
    assert not (output_dir / "src" / "drivers" / "real" / "thunder" / "blueprints.py").exists()
    assert not (output_dir / "src" / "drivers" / "real" / "thunder" / "connection.py").exists()
    assert not (output_dir / "src" / "slam").exists()
    assert not (output_dir / "src" / "adapters" / "ros2").exists()
    assert not (output_dir / "src" / "core" / "tests").exists()
    assert not (output_dir / "src" / "core" / "dds.py").exists()
    assert (output_dir / packager.SUMMARY_FILENAME).is_file()


def test_thunder_lite_filtered_package_can_setup_direct_navigation(tmp_path: Path) -> None:
    output_dir = tmp_path / "thunder-lite"
    packager.build_package(output_dir=output_dir)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(output_dir / "src")
    script = (
        "from nav.mission.navigation import Navigation\n"
        "m = Navigation(planner='direct', tomogram='')\n"
        "m.setup()\n"
        "print(type(m._planner_svc).__name__)\n"
        "print(m._planner_svc.has_map)\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=output_dir,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.splitlines() == ["MaplessDirectPlannerService", "False"]


def test_thunder_lite_filtered_package_strict_driver_seed_uses_canonical_thunder(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "thunder-lite"
    packager.build_package(output_dir=output_dir)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(output_dir / "src")
    script = (
        "import json\n"
        "from runtime.registry import clear, list_plugins\n"
        "from lingtu.plugin_seed import seed_builtin_plugins\n"
        "clear()\n"
        "report = seed_builtin_plugins(groups=('driver',), reload_loaded=True, strict=True)\n"
        "print(json.dumps({'report': report, 'drivers': sorted(list_plugins('driver'))}))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=output_dir,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["report"]["failed"] == {}
    assert {"stub", "thunder"} <= set(payload["drivers"])
    assert "nova_dog" not in payload["drivers"]


def test_thunder_lite_filtered_package_setup_does_not_load_excluded_heavy_stacks(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "thunder-lite"
    packager.build_package(output_dir=output_dir)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(output_dir / "src")
    script = (
        "import json\n"
        "import sys\n"
        "from nav.mission.navigation import Navigation\n"
        "m = Navigation(planner='direct', tomogram='')\n"
        "m.setup()\n"
        "excluded = ('nav.services.plan.global_planner.algorithm.pct.vendor', 'runtime.adapters.ros2', 'slam')\n"
        "loaded = sorted(\n"
        "    name for name in sys.modules\n"
        "    if any(name == root or name.startswith(root + '.') for root in excluded)\n"
        ")\n"
        "print(json.dumps(loaded))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=output_dir,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert json.loads(result.stdout) == []


def test_thunder_lite_package_validator_passes_json_mode() -> None:
    result = subprocess.run(
        [sys.executable, "tools/validate/validate_thunder_lite_package.py", "--json"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    assert payload["blockers"] == []
    assert "requirements-lite.txt" in payload["checked_files"]
