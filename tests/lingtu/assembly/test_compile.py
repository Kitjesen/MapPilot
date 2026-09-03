from __future__ import annotations

import json
import sys
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

import pytest

from lingtu.assembly.compiler import (
    _product_roles,
    blueprint_for_resolved_product,
    blueprint_from_run_plan,
)
from lingtu.assembly.compiler import (
    compile_run_plan as _compile_run_plan,
)
from lingtu.assembly.products import resolve_product_host_runtime as _resolve_product_host_runtime
from lingtu.run_plan import RUN_PLAN_SCHEMA, RunPlan
from lingtu.sim.acceptance import load_manifest
from runtime.blueprint import Blueprint
from runtime.graph import (
    ProcessArtifact,
    RuntimeGraph,
    load_runtime_graph,
    resolve_processes,
)

FIELD_PRODUCTS = tuple(sorted(load_runtime_graph().products))
REAL_ROBOT = "unitree/go2"
SIM_ROBOT = "doso/thunder_v4"


def _robot_for_env(env: str) -> str:
    return SIM_ROBOT if env == "sim" else REAL_ROBOT


def resolve_product_host_runtime(product: str, env: str, **kwargs):
    kwargs.setdefault("robot", _robot_for_env(env))
    return _resolve_product_host_runtime(product, env, **kwargs)


def compile_run_plan(product: str, env: str, **kwargs):
    kwargs.setdefault("robot", _robot_for_env(env))
    if env != "sim" or "graph" in kwargs:
        return _compile_run_plan(product, env, **kwargs)
    with patch.object(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    ):
        return _compile_run_plan(product, env, **kwargs)


def test_compile_resolves_env_implementation_once(monkeypatch) -> None:
    import lingtu.assembly.compiler as compiler_module
    import runtime.graph as graph_module
    import runtime.graph.processes as process_module

    original = graph_module.resolve_env_implementation
    calls = 0

    def resolve_once(*args, **kwargs):
        nonlocal calls
        calls += 1
        return original(*args, **kwargs)

    def reject_second_resolution(*_args, **_kwargs):
        raise AssertionError("resolve_processes must reuse the compiler's Env implementation")

    monkeypatch.setattr(graph_module, "resolve_env_implementation", resolve_once)
    monkeypatch.setattr(process_module, "resolve_env_implementation", reject_second_resolution)

    plan = compiler_module.compile_run_plan("nav", "real", robot=REAL_ROBOT)

    assert plan.env == "real"
    assert calls == 1


def test_direct_teleop_does_not_require_robot_local_planner_assets() -> None:
    plan = compile_run_plan("teleop", "real", robot=REAL_ROBOT)

    assert "LINGTU_LOCAL_PLANNER_PATHS" not in plan.native_process_environment


def test_assisted_teleop_cmu_receives_robot_path_library() -> None:
    plan = compile_run_plan(
        "teleop_avoid",
        "sim",
        robot="doso/thunder_v4",
        local_planner="cmu",
        env_config={"backend": "mujoco"},
    )

    assert plan.native_process_environment["LINGTU_LOCAL_PLANNER_PATHS"] == (
        "src/nav/cpp/planning/local/cmu/paths/thunder"
    )
    assert plan.native_process_environment["LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M"] == "0.05"


def test_sim_host_readiness_budget_exceeds_internal_startup_barrier() -> None:
    plan = compile_run_plan(
        "teleop_avoid",
        "sim",
        env_config={"backend": "mujoco"},
    )

    assert plan.process("host").timeout_s > 30.0


def test_sim_map_waits_as_long_as_localization() -> None:
    plan = compile_run_plan(
        "tracking",
        "sim",
        env_config={"backend": "mujoco"},
    )

    assert plan.process("maps").timeout_s >= plan.process("slam").timeout_s


def _subprocess_graph(*, include_support: bool = False) -> RuntimeGraph:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    backend = envs["sim"]["backends"]["mujoco"]
    backend.pop("acceptance", None)
    backend["process_control"] = "subprocess"
    backend["process_manager"] = "direct"
    backend["supported_products"] = list(graph.products)
    backend.pop("support_processes", None)
    backend["presets"] = {
        SIM_ROBOT: {
            "default": "sim/sessions/examples/thunder_omni_contract/session.yaml",
        }
    }
    backend["provided_roles"] = [
        "lidar",
        "imu",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "camera",
        "explore",
        "host",
        "diagnostics",
    ]

    def process(
        owner: str,
        order: int,
        provides: list[str],
        *,
        lifecycle: str = "mode",
    ) -> dict[str, object]:
        artifact = "src/lingtu/run_plan.py"
        return {
            "target": owner,
            "lifecycle": lifecycle,
            "order": order,
            "timeout_s": 10,
            "provides": provides,
            "command": {
                "argv": [artifact, "--owner", owner],
                "cwd": ".",
                "env": {},
                "artifact": {"path": artifact},
                "readiness": {"kind": "process"},
            },
        }

    backend["processes"] = {
        "native_runtime": process(
            "native_runtime",
            10,
            [
                "lidar",
                "imu",
                "slam",
                "maps",
                "traversability",
                "nav",
                "driver",
                "camera",
                "explore",
            ],
        ),
        "host_runtime": process("host_runtime", 20, ["host"]),
        "diagnostics_runtime": process(
            "diagnostics_runtime",
            30,
            ["diagnostics"],
            lifecycle="persistent",
        ),
    }
    backend["stop_before_start"] = ["host_runtime", "native_runtime"]
    if include_support:
        backend["support_processes"] = ["mujoco_feeder"]
        backend["processes"]["mujoco_feeder"] = process(
            "mujoco_feeder",
            5,
            [],
        )
        backend["stop_before_start"].append("mujoco_feeder")
    return RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )


def _materialized_platform_graph(tmp_path: Path) -> RuntimeGraph:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    processes = envs["sim"]["backends"]["mujoco"]["processes"]
    for process_name, process in processes.items():
        commands = process.get("platforms")
        if commands is None:
            commands = {"portable": process["command"]}
        for platform, command in commands.items():
            suffix = ".exe" if platform == "windows" else ""
            relative = f"artifacts/{platform}/{process_name}{suffix}"
            artifact = tmp_path / relative
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_bytes(f"{platform}:{process_name}".encode())
            command["artifact"]["path"] = relative
            entry_index = 1 if str(command["argv"][0]).startswith("python") else 0
            command["argv"][entry_index] = relative
            for index, dependency in enumerate(command.get("dependencies", [])):
                original_dependency = dependency["path"]
                dependency_suffix = Path(original_dependency).suffix
                dependency_relative = f"artifacts/{platform}/{process_name}-dependency-{index}{dependency_suffix}"
                dependency_path = tmp_path / dependency_relative
                dependency_path.write_bytes(f"{platform}:{process_name}:dependency:{index}".encode())
                dependency["path"] = dependency_relative
                nav_client = command.get("env", {}).get("LINGTU_NAV_CLIENT_LIB")
                if nav_client and Path(nav_client).name == Path(original_dependency).name:
                    command["env"]["LINGTU_NAV_CLIENT_LIB"] = dependency_relative
                slam_control = command.get("env", {}).get("LINGTU_SLAM_CONTROL")
                if slam_control and Path(slam_control).name == Path(original_dependency).name:
                    command["env"]["LINGTU_SLAM_CONTROL"] = dependency_relative
    return RuntimeGraph(
        root=tmp_path,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )


def _compile_real(
    product: str,
    *,
    product_variant: str | None = None,
    overrides: dict | None = None,
    graph: RuntimeGraph | None = None,
) -> RunPlan:
    return compile_run_plan(
        product,
        "real",
        product_variant=product_variant,
        overrides=overrides,
        graph=graph,
    )


def _assert_front_rgbd(plan: RunPlan) -> None:
    streams = plan.simulation["sensor_plan"]["streams"]
    assert len(streams["rgb"]) == len(streams["depth"]) == 1
    rgb = streams["rgb"][0]
    depth = streams["depth"][0]
    assert rgb["rate_hz"] == depth["rate_hz"] == 30.0
    assert (rgb["width"], rgb["height"]) == (640, 480)
    assert (depth["width"], depth["height"]) == (640, 480)
    assert (
        rgb["extrinsic"]
        == depth["extrinsic"]
        == {
            "position_m": [0.423358800364963, -0.000496202974186816, 0.11370714960317],
            "quaternion_wxyz": [0.5, -0.5, 0.5, -0.5],
        }
    )


@pytest.mark.parametrize("product", FIELD_PRODUCTS)
def test_every_field_product_declares_one_blueprint_host_contract(product: str) -> None:
    manifest = _compile_real(product)
    blueprint = blueprint_from_run_plan(manifest)

    assert manifest.process_control == "systemd"
    assert manifest.modules == blueprint.module_names
    assert manifest.critical_modules == blueprint.required_module_names
    assert manifest.schema_version == RUN_PLAN_SCHEMA
    assert "acceptance" not in manifest.as_dict()["launch"]
    assert manifest.simulation == {}
    assert manifest.has_process("host")


def test_real_run_plan_selects_go2_mid360_config() -> None:
    plan = _compile_real("teleop_avoid")

    assert "LINGTU_EXPLORE_ROUTE" not in plan.native_process_environment
    slam_config = plan.native_process_environment["LINGTU_SLAM_CONFIG"].replace("\\", "/")
    assert slam_config.endswith("/config/robots/unitree/go2/sensors/mid360_fastlio2.yaml")
    assert "lidar_extrinsic_profile" not in plan.host_config
    assert plan.native_process_environment["LINGTU_LIVOX_LIDAR_IP"] == "192.168.123.20"
    assert plan.native_process_environment["LINGTU_LIVOX_HOST_IP"] == "192.168.123.18"
    assert plan.native_process_environment["LINGTU_LIVOX_NET_IFACE"] == "eth0"
    assert plan.native_process_environment["LINGTU_DRIVER_NETWORK_ADDRESS"] == "192.168.123.18/24"
    assert plan.native_process_environment["LINGTU_DRIVER_PROBE_IP"] == "192.168.123.161"
    assert plan.native_process_environment["LINGTU_DDS_NETWORK_INTERFACE"] == "eth0"
    assert (
        "<NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>" in (plan.native_process_environment["CYCLONEDDS_URI"])
    )
    assert plan.native_process_environment["LINGTU_CONFIG_PATH"] == (
        "config/robots/unitree/go2/robot.yaml"
    )
    assert plan.native_process_environment["LINGTU_NAV_LOCAL_PLANNER_BACKEND"] == "scan"
    assert "LINGTU_LOCAL_PLANNER_PATHS" not in plan.native_process_environment
    assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.05"
    assert plan.native_process_environment["LINGTU_NAV_VEHICLE_LENGTH_M"] == "0.76"
    assert plan.native_process_environment["LINGTU_NAV_VEHICLE_WIDTH_M"] == "0.31"
    assert plan.native_process_environment["LINGTU_TELEOP_OBSTACLE_MARGIN_M"] == "0.1"
    assert plan.native_process_environment["LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M"] == "0.25"
    assert plan.native_process_environment["LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M"] == "0.18"
    assert plan.native_process_environment["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"] == "0.25"
    assert plan.native_process_environment["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"] == "0.35"
    assert plan.native_process_environment["LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS"] == "0.5"
    assert plan.native_process_environment["LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_RATE_RAD_S"] == "1"
    assert plan.native_process_environment["LINGTU_TELEOP_MAX_SPEED_MPS"] == "0.5"
    assert plan.native_process_environment["LINGTU_TELEOP_MAX_YAW_RATE"] == "1"


def test_real_cmu_override_selects_go2_path_library() -> None:
    plan = compile_run_plan("teleop_avoid", "real", local_planner="cmu")

    assert plan.native_process_environment["LINGTU_LOCAL_PLANNER_PATHS"] == (
        "share/lingtu/cmu_paths/go2"
    )
    assert plan.native_process_environment["LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M"] == "0.05"


def test_field_product_compiles_module_graph_and_processes_together() -> None:
    product = _compile_real("nav")
    blueprint = blueprint_from_run_plan(product)

    assert product.product == "nav"
    assert product.env == "real"
    assert product.process_control == "systemd"
    assert product.has_process("slam")
    assert product.has_process("maps")
    assert product.has_process("nav")
    assert product.has_process("driver")
    assert product.native_process_environment["LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS"] == "0.5"
    assert product.native_process_environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.05"
    assert "maps.service" not in product.modules
    assert not {
        "OccupancyGridModule",
        "VoxelGridModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
        "map.out",
    }.intersection(product.modules)
    assert "/slam/odometry" in product.required_topics
    assert "/slam/map_observation" in product.required_topics
    assert "/nav/state" in product.required_topics
    assert "/nav/cmd_vel" in product.required_topics
    assert "final_cmd_vel_single_writer" in product.required_capabilities
    assert "host.bus" in product.modules
    assert "nav.commands" in product.modules
    assert "nav.goals" in product.modules
    assert product.critical_modules == (
        "host.bus",
        "SlamAdapterModule",
        "nav.commands",
        "nav.goals",
        "GatewayModule",
    )
    assert blueprint.required_module_names == product.critical_modules
    wires = {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in blueprint._wires}
    assert "host.bus.navigation_goal_status->nav.goals.navigation_goal_status" in wires


def test_persistent_sim_teleop_preserves_product_host_startup_barrier() -> None:
    resolved = resolve_product_host_runtime(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
    )
    blueprint = blueprint_from_run_plan(plan)

    assert plan.critical_modules == (
        "host.bus",
        "nav.commands",
        "GatewayModule",
    )
    assert blueprint.required_module_names == plan.critical_modules


def test_real_nav_process_payload_keeps_strict_systemd_shape() -> None:
    first = _compile_real("nav")
    second = _compile_real("nav")
    expected_fields = {
        "name",
        "manager",
        "target",
        "order",
        "timeout_s",
        "lifecycle",
    }

    assert first.as_dict() == second.as_dict()
    catalog = first.as_dict()["launch"]["process_catalog"]
    assert catalog["selected"]
    assert catalog["available"]
    for process in (*catalog["selected"], *catalog["available"]):
        fields = set(expected_fields)
        if process["name"] == "lidar":
            fields.add("provides")
        assert set(process) == fields
        if process["name"] == "lidar":
            assert process["provides"] == ["lidar", "imu"]


@pytest.mark.parametrize(
    "roles",
    (
        [123],
        [True],
        [" nav"],
        ["Nav"],
        ["nav-role"],
        [""],
    ),
)
def test_product_roles_reject_coercion_and_invalid_tokens(roles: list[object]) -> None:
    with pytest.raises(ValueError, match="invalid or duplicate process roles"):
        _product_roles({"processes": roles}, product="test")


def test_product_roles_preserve_valid_tokens_without_conversion() -> None:
    assert _product_roles(
        {"processes": ["maps", "nav", "host"]},
        product="test",
    ) == ("maps", "nav", "host")


def test_explore_product_launches_the_native_exploration_contract() -> None:
    product = _compile_real("explore")

    assert product.has_process("explore")
    assert {
        "/nav/exploration/command",
        "/nav/exploration/ack",
        "/nav/exploration_snapshot",
        "/nav/exploration_execution_snapshot",
        "/nav/exploration_segment/request",
        "/nav/exploration_segment/ack",
        "/nav/exploration_segment/status",
    } <= set(product.required_topics)


def test_explore_variants_compile_distinct_complete_run_plans(tmp_path) -> None:
    live = _compile_real("explore", product_variant="live")
    saved_map = _compile_real("explore", product_variant="map")

    assert live.product == saved_map.product == "explore"
    assert live.product_variant == "live"
    assert saved_map.product_variant == "map"
    assert live.contracts == ("lingtu.product.explore.v1",)
    assert saved_map.contracts == ("lingtu.product.explore.map.v1",)
    assert live.lifecycle["product_variant"] == "live"
    assert saved_map.lifecycle["product_variant"] == "map"
    assert live.lifecycle["slam_mode"] == "mapping"
    assert live.lifecycle["requires_map"] is False
    assert saved_map.lifecycle["slam_mode"] == "localization"
    assert saved_map.lifecycle["requires_map"] is True
    assert live.native_process_environment["LINGTU_EXPLORE_ROUTE"] == "live"
    assert saved_map.native_process_environment["LINGTU_EXPLORE_ROUTE"] == "map"
    assert live.native_nav["allow_teleop_takeover"] is False
    assert saved_map.native_nav["allow_teleop_takeover"] is True
    assert live != saved_map

    loaded = RunPlan.load(saved_map.write(tmp_path / "explore-map.json"))

    assert loaded == saved_map
    assert loaded.as_dict()["identity"]["product_variant"] == "map"

    tampered = saved_map.as_dict()
    tampered["identity"]["product_variant"] = "live"
    with pytest.raises(ValueError, match="variant does not match"):
        RunPlan.from_dict(tampered)


@pytest.mark.parametrize("mismatched_surface", ["host", "lifecycle"])
def test_run_plan_create_rejects_mismatched_variant_identity(
    mismatched_surface: str,
) -> None:
    plan = _compile_real("explore", product_variant="map")
    host_config = plan.host_config
    lifecycle = plan.lifecycle
    if mismatched_surface == "host":
        host_config["_product_variant"] = "live"
    else:
        lifecycle["product_variant"] = "live"

    with pytest.raises(ValueError, match="variant does not match"):
        RunPlan.create(
            product=plan.product,
            product_variant=plan.product_variant,
            env=plan.env,
            robot=plan.robot,
            process_control=plan.process_control,
            modules=plan.modules,
            processes=plan.processes,
            available_processes=plan.available_processes,
            stop_before_start=plan.stop_before_start,
            contracts=plan.contracts,
            critical_modules=plan.critical_modules,
            route_contract=plan.route_contract,
            host_config=host_config,
            lifecycle=lifecycle,
            native_process_environment=plan.native_process_environment,
            parameters=plan.parameters,
        )


def test_explore_compile_reads_parameters_from_selected_variant() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["live"]["parameters"] = {
        "segment.max_distance_m": 4.0,
    }
    products["explore"]["variants"]["map"]["parameters"] = {
        "segment.max_distance_m": 3.0,
    }
    variant_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )
    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        product_variant=resolved.product_variant,
        graph=variant_graph,
    )

    assert plan.parameters["segment.max_distance_m"] == 3.0


@pytest.mark.parametrize("env", ("real", "sim"))
def test_all_envs_use_native_mapd_without_python_map_module(env: str) -> None:
    env_config = {"backend": "mujoco"} if env == "sim" else None

    plan = compile_run_plan("nav", env, env_config=env_config)

    assert plan.has_process("maps")
    assert "maps.service" not in plan.modules
    assert "maps.service" not in plan.critical_modules
    assert not {
        "OccupancyGridModule",
        "VoxelGridModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
        "map.out",
    }.intersection(plan.modules)


def test_unknown_product_does_not_compile_to_run_plan() -> None:
    with pytest.raises(ValueError, match="is not a Product"):
        compile_run_plan("unknown", "sim", env_config={"backend": "mujoco"})


def test_go2_sim_compile_reports_missing_session_assets() -> None:
    with pytest.raises(ValueError, match=r"simulation session|unitree/go2"):
        compile_run_plan("teleop", "sim", robot=REAL_ROBOT)


def test_sim_mujoco_process_catalog_declares_exact_native_platform_paths() -> None:
    backend = load_runtime_graph().envs["sim"]["backends"]["mujoco"]
    processes = backend["processes"]

    assert "sensor_publisher" not in processes
    assert backend["stop_before_start"][-4:] == [
        "camera_publisher",
        "imu_publisher",
        "lidar_publisher",
        "driver_bridge",
    ]

    windows_slam = processes["slam_runtime"]["platforms"]["windows"]
    assert windows_slam["artifact"]["path"] == ("build/slam-core-windows-x64/stage/bin/slamd.exe")
    linux_slam = processes["slam_runtime"]["platforms"]["linux"]
    windows_slam_without_domain = list(windows_slam["argv"])
    linux_slam_without_domain = list(linux_slam["argv"])
    for argv in (windows_slam_without_domain, linux_slam_without_domain):
        domain_index = argv.index("--domain-id")
        del argv[domain_index : domain_index + 2]
    assert windows_slam_without_domain[1:] == linux_slam_without_domain[1:]
    assert windows_slam["readiness"] == processes["slam_runtime"]["platforms"]["linux"]["readiness"]
    assert {Path(item["path"]).name for item in windows_slam["dependencies"]} == {
        "boost_filesystem-vc143-mt-x64-1_91.dll",
        "boost_iostreams-vc143-mt-x64-1_91.dll",
        "bz2.dll",
        "ddsc.dll",
        "liblzma.dll",
        "libpng16.dll",
        "lz4.dll",
        "pcl_common.dll",
        "pcl_filters.dll",
        "pcl_io.dll",
        "pcl_io_ply.dll",
        "pcl_kdtree.dll",
        "pcl_octree.dll",
        "pcl_registration.dll",
        "pcl_sample_consensus.dll",
        "pcl_search.dll",
        "yaml-cpp.dll",
        "z.dll",
        "zstd.dll",
    }
    for name in (
        "lidar_publisher",
        "imu_publisher",
        "camera_publisher",
        "driver_bridge",
    ):
        windows = processes[name]["platforms"]["windows"]
        assert windows["artifact"]["path"].startswith("build/windows-native-dds-adapter/Release/")
        assert windows["dependencies"] == [{"path": ("build/windows-native-dds-adapter/Release/ddsc.dll")}]
    for stream in ("lidar", "imu", "camera"):
        process = processes[f"{stream}_publisher"]
        assert process["provides"] == [stream]
        for command in process["platforms"].values():
            stream_index = command["argv"].index("--stream")
            assert command["argv"][stream_index + 1] == stream
            assert command["readiness"] == {
                "kind": "file",
                "target": f"{stream}.ready.json",
            }
    canonical_native_paths = {
        "map_runtime": (
            "build/maps-windows/Release/mapd.exe",
            ("build/maps-windows/Release/ddsc.dll",),
        ),
        "traversability_runtime": (
            "build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_traversability_dds.exe",
            ("build/nav-cpp/windows-x64-nav-endpoint/Release/ddsc.dll",),
        ),
        "nav_runtime": (
            "build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe",
            ("build/nav-cpp/windows-x64-nav-endpoint/Release/ddsc.dll",),
        ),
        "explore_runtime": (
            "build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_explore_dds.exe",
            ("build/nav-cpp/windows-x64-nav-endpoint/Release/ddsc.dll",),
        ),
    }
    for name, (artifact_path, dependency_paths) in canonical_native_paths.items():
        windows = processes[name]["platforms"]["windows"]
        assert windows["artifact"]["path"] == artifact_path
        assert tuple(item["path"] for item in windows["dependencies"]) == (dependency_paths)

    windows_host = processes["host_runtime"]["platforms"]["windows"]
    nav_client_path = "build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_nav_client.dll"
    slam_control_path = "build/slam-core-windows-x64/stage/bin/slamctl.exe"
    assert windows_host["env"]["LINGTU_NAV_CLIENT_LIB"] == nav_client_path
    assert windows_host["env"]["LINGTU_SLAM_CONTROL"] == slam_control_path
    assert windows_host["dependencies"] == [
        {"path": nav_client_path},
        {"path": ("build/nav-cpp/windows-x64-nav-endpoint/Release/ddsc.dll")},
        {"path": slam_control_path},
    ]
    for name in (
        "lidar_publisher",
        "imu_publisher",
        "camera_publisher",
        "driver_bridge",
        "map_runtime",
        "traversability_runtime",
        "nav_runtime",
        "explore_runtime",
    ):
        platforms = processes[name]["platforms"]
        assert platforms["windows"]["artifact"]["path"].endswith(".exe")
        assert not platforms["linux"]["artifact"]["path"].endswith(".exe")


@pytest.mark.parametrize(
    ("product", "product_variant"),
    (
        ("teleop", None),
        ("teleop_avoid", None),
        ("map", None),
        ("nav", None),
        ("tracking", None),
        ("inspection", None),
        ("explore", "live"),
        ("explore", "map"),
    ),
)
def test_sim_mujoco_every_product_compiles_for_windows_with_complete_pe_chain_and_nav_readiness_stage(
    product: str,
    product_variant: str | None,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "windows",
    )
    resolved = resolve_product_host_runtime(
        product,
        "sim",
        product_variant=product_variant,
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        product_variant=resolved.product_variant,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    assert all(
        "/linux/" not in process.command.artifact.path for process in plan.processes if process.command is not None
    )
    assert plan.product == product
    assert plan.product_variant == product_variant
    process_order = {process.name: process.order for process in plan.processes}
    nav_order = process_order["nav_runtime"]
    feeder_order = process_order["mujoco_feeder"]
    assert nav_order == feeder_order
    if "slam_runtime" in process_order:
        assert process_order["slam_runtime"] == feeder_order
    if "traversability_runtime" in process_order:
        assert process_order["traversability_runtime"] == feeder_order
    if product == "teleop":
        assert not plan.has_process("slam")
        return

    slam = plan.process("slam").command
    assert slam.artifact.path.endswith("slam_runtime.exe")
    assert slam.readiness.kind == "file"
    assert slam.readiness.target == "slam.status.json"
    assert "--config" not in slam.argv
    assert plan.native_process_environment["LINGTU_SLAM_CONFIG"].replace("\\", "/") == (
        "src/localization/fastlio2/config/sim_mid360.yaml"
    )
    assert len(slam.dependencies) == 19
    assert all(item.path.endswith(".dll") for item in slam.dependencies)


@pytest.mark.parametrize("process_platform", ("windows", "linux"))
@pytest.mark.parametrize(
    ("product", "expected"),
    (
        ("teleop", {"driver_bridge", "mujoco_feeder", "nav_runtime", "host_runtime"}),
        (
            "teleop_avoid",
            {
                "driver_bridge",
                "lidar_publisher",
                "imu_publisher",
                "mujoco_feeder",
                "slam_runtime",
                "map_runtime",
                "nav_runtime",
                "host_runtime",
            },
        ),
        (
            "map",
            {
                "camera_publisher",
                "driver_bridge",
                "lidar_publisher",
                "imu_publisher",
                "mujoco_feeder",
                "slam_runtime",
                "map_runtime",
                "nav_runtime",
                "host_runtime",
            },
        ),
        (
            "nav",
            {
                "driver_bridge",
                "lidar_publisher",
                "imu_publisher",
                "mujoco_feeder",
                "slam_runtime",
                "map_runtime",
                "nav_runtime",
                "host_runtime",
            },
        ),
        (
            "tracking",
            {
                "camera_publisher",
                "driver_bridge",
                "lidar_publisher",
                "imu_publisher",
                "mujoco_feeder",
                "slam_runtime",
                "map_runtime",
                "nav_runtime",
                "host_runtime",
            },
        ),
        (
            "inspection",
            {
                "camera_publisher",
                "driver_bridge",
                "lidar_publisher",
                "imu_publisher",
                "mujoco_feeder",
                "slam_runtime",
                "map_runtime",
                "nav_runtime",
                "host_runtime",
            },
        ),
        (
            "explore",
            {
                "driver_bridge",
                "lidar_publisher",
                "imu_publisher",
                "mujoco_feeder",
                "slam_runtime",
                "map_runtime",
                "traversability_runtime",
                "nav_runtime",
                "explore_runtime",
                "host_runtime",
            },
        ),
    ),
)
def test_sim_mujoco_products_select_exact_platform_processes(
    process_platform: str,
    product: str,
    expected: set[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: process_platform,
    )
    monkeypatch.setattr(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    )
    selected, available, conflicts, support = resolve_processes(
        product,
        "sim",
        env_config={"backend": "mujoco"},
    )

    assert {process.name for process in selected} == expected
    assert available == selected
    assert conflicts == ()
    assert support == ("mujoco_feeder",)
    assert len({role for process in selected for role in process.provides}) == sum(
        len(process.provides) for process in selected
    )


@pytest.mark.parametrize("process_platform", ("windows", "linux"))
def test_sim_mujoco_slam_is_the_only_navigation_output_owner(
    process_platform: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: process_platform,
    )
    monkeypatch.setattr(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    )

    selected, _, _, _ = resolve_processes(
        "teleop_avoid",
        "sim",
        env_config={"backend": "mujoco"},
    )
    lidar = next(process for process in selected if process.name == "lidar_publisher")

    assert "--navigation-fixture" not in lidar.command.argv


@pytest.mark.parametrize("process_platform", ("windows", "linux"))
def test_sim_mujoco_driver_deadlines_cover_scheduler_jitter(
    process_platform: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: process_platform,
    )
    monkeypatch.setattr(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    )

    selected, _, _, _ = resolve_processes(
        "teleop_avoid",
        "sim",
        env_config={"backend": "mujoco"},
    )
    driver = next(process for process in selected if process.name == "driver_bridge")
    argv = driver.command.argv

    assert argv[argv.index("--command-timeout-ms") + 1] == "500"
    assert argv[argv.index("--heartbeat-timeout-ms") + 1] == (
        "10000" if process_platform == "windows" else "2000"
    )
    assert argv[argv.index("--apply-timeout-ms") + 1] == "30000"


def test_sim_mujoco_linux_nav_selects_one_complete_elf_chain(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "linux",
    )
    resolved = resolve_product_host_runtime("nav", "sim", env_config={"backend": "mujoco"})
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    platform_processes = {
        name for name, process in graph.envs["sim"]["backends"]["mujoco"]["processes"].items() if "platforms" in process
    }
    selected_platform_artifacts = {
        process.command.artifact.path
        for process in plan.processes
        if process.name in platform_processes and process.command is not None
    }
    assert selected_platform_artifacts
    assert all("/linux/" in path and not path.endswith(".exe") for path in selected_platform_artifacts)
    assert plan.process("slam").name == "slam_runtime"


def test_sim_mujoco_unselected_artifacts_do_not_block_and_selection_is_explicit(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    resolved = resolve_product_host_runtime("teleop", "sim", env_config={"backend": "mujoco"})
    linux_driver = tmp_path / "artifacts/linux/driver_bridge"
    linux_driver.unlink()
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "windows",
    )
    windows_plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )
    windows_map = tmp_path / "artifacts/windows/map_runtime.exe"
    windows_map.unlink()
    processes = graph.envs["sim"]["backends"]["mujoco"]["processes"]
    processes["map_runtime"]["platforms"]["windows"]["artifact"]["path"] = "missing/windows-map.exe"
    windows_explore = tmp_path / "artifacts/windows/explore_runtime.exe"
    windows_explore.write_bytes(b"changed-unselected-explore")
    unchanged_windows_plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    linux_driver.write_bytes(b"linux:driver_bridge")
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "linux",
    )
    linux_plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    assert windows_plan.process("driver").command.artifact.path.endswith(".exe")
    assert unchanged_windows_plan == windows_plan
    assert not linux_plan.process("driver").command.artifact.path.endswith(".exe")
    assert windows_plan != linux_plan


def test_sim_mujoco_windows_slam_selected_artifact_paths_are_required(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "windows",
    )
    resolved = resolve_product_host_runtime("nav", "sim", env_config={"backend": "mujoco"})
    baseline = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )
    processes = graph.envs["sim"]["backends"]["mujoco"]["processes"]

    linux_slam_path = Path(processes["slam_runtime"]["platforms"]["linux"]["artifact"]["path"])
    (tmp_path / linux_slam_path).write_bytes(b"changed-unselected-linux-slam")
    unchanged = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )
    assert unchanged == baseline

    windows_slam = baseline.process("slam").command
    selected_dependency = tmp_path / windows_slam.dependencies[0].path
    selected_dependency.write_bytes(b"changed-selected-windows-slam-dependency")
    changed = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )
    assert changed == baseline

    selected_dependency.unlink()
    with pytest.raises(ValueError, match="direct process artifact does not exist"):
        compile_run_plan(
            resolved.product,
            resolved.env,
            env_config={"backend": "mujoco"},
            graph=graph,
        )

    selected_dependency.write_bytes(b"restored-selected-windows-slam-dependency")
    selected_artifact = tmp_path / windows_slam.artifact.path
    selected_artifact.unlink()
    with pytest.raises(ValueError, match="direct process artifact does not exist"):
        compile_run_plan(
            resolved.product,
            resolved.env,
            env_config={"backend": "mujoco"},
            graph=graph,
        )


def test_sim_mujoco_windows_host_uses_staged_slam_control(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "windows",
    )
    resolved = resolve_product_host_runtime("nav", "sim", env_config={"backend": "mujoco"})

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    host_command = plan.process("host").command
    assert host_command is not None
    slam_control = dict(host_command.env)["LINGTU_SLAM_CONTROL"]
    assert Path(slam_control).suffix == ".exe"
    assert slam_control in {dependency.path for dependency in host_command.dependencies}


def test_sim_host_nav_client_dependency_must_be_authenticated(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    command = graph.envs["sim"]["backends"]["mujoco"]["processes"]["host_runtime"]["platforms"]["windows"]
    command["env"]["LINGTU_NAV_CLIENT_LIB"] = "artifacts/windows/untracked.dll"
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "windows",
    )
    resolved = resolve_product_host_runtime("teleop", "sim", env_config={"backend": "mujoco"})

    with pytest.raises(ValueError, match="LINGTU_NAV_CLIENT_LIB"):
        compile_run_plan(
            resolved.product,
            resolved.env,
            env_config={"backend": "mujoco"},
            graph=graph,
        )


def test_sim_host_slam_control_dependency_must_be_authenticated(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    command = graph.envs["sim"]["backends"]["mujoco"]["processes"]["host_runtime"]["platforms"]["windows"]
    command["env"]["LINGTU_SLAM_CONTROL"] = "artifacts/windows/untracked-slamctl.exe"
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "windows",
    )
    resolved = resolve_product_host_runtime("nav", "sim", env_config={"backend": "mujoco"})

    with pytest.raises(ValueError, match="LINGTU_SLAM_CONTROL"):
        compile_run_plan(
            resolved.product,
            resolved.env,
            env_config={"backend": "mujoco"},
            graph=graph,
        )


def test_sim_mujoco_teleop_compiles_persistent_native_processes_and_host_guards() -> None:
    resolved = resolve_product_host_runtime(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )

    manifest = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
    )

    assert manifest.env == "sim"
    assert manifest.process_control == "subprocess"
    assert "acceptance" not in manifest.as_dict()["launch"]
    assert manifest.simulation["session_source"] == (
        "sim/sessions/products/doso/thunder_v4/default.yaml"
    )
    assert manifest.simulation["session"]["world"] == "industrial_park@1.0.0"
    assert manifest.simulation["physics_plan"]["global_policy"]["timestep_s"] == 0.005
    assert manifest.simulation["physics_plan"]["global_policy"]["integrator"] == "euler"
    assert manifest.simulation["physics_plan"]["robots"][0]["spawn"]["position_m"] == [
        3.0,
        4.0,
        0.0,
    ]
    assert manifest.simulation["session"]["session_id"]
    assert [process.name for process in manifest.processes] == [
        "driver_bridge",
        "mujoco_feeder",
        "nav_runtime",
        "host_runtime",
    ]
    assert manifest.support_processes == ("mujoco_feeder",)
    assert next(process for process in manifest.available_processes if process.name == "mujoco_feeder").timeout_s == 60
    assert manifest.process("driver").name == "driver_bridge"
    assert manifest.process("nav").name == "nav_runtime"
    assert manifest.process("host").name == "host_runtime"
    host_dependencies = manifest.process("host").command.dependencies
    if sys.platform == "win32":
        assert len(host_dependencies) == 3
        assert host_dependencies[0].path == (
            "build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_nav_client.dll"
        )
        assert host_dependencies[1].path.endswith("ddsc.dll")
        assert host_dependencies[2].path == "build/slam-core-windows-x64/stage/bin/slamctl.exe"
    else:
        assert [dependency.path for dependency in host_dependencies] == [
            "build/nav_endpoint/liblingtu_nav_client.so"
        ]
    assert manifest.process("nav").timeout_s == 60
    assert manifest.process("nav").command.argv[-2:] == ("--status-s", "0.1")
    assert not manifest.has_process("camera")
    assert manifest.host_config["enable_camera"] is False
    _assert_front_rgbd(manifest)
    assert manifest.stop_before_start == (
        "host_runtime",
        "nav_runtime",
        "mujoco_feeder",
        "driver_bridge",
    )
    assert "host.bus" in manifest.modules
    assert "SimEndpointDriverModule" not in manifest.modules


def test_sim_mujoco_teleop_avoid_compiles_complete_native_safety_chain(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr("runtime.graph.processes._host_process_platform", lambda: "linux")
    graph = _materialized_platform_graph(tmp_path)
    resolved = resolve_product_host_runtime(
        "teleop_avoid",
        "sim",
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    assert plan.process_control == "subprocess"
    assert plan.simulation["session_source"] == (
        "sim/sessions/products/doso/thunder_v4/teleop_avoid.yaml"
    )
    assert plan.simulation["session"]["world"] == "teleop_avoid_field@1.0.0"
    assert plan.simulation["session"]["runtime"]["mode"] == "preview"
    assert plan.lifecycle["native_control_mode"] == "teleop_avoid"
    assert [process.name for process in plan.processes] == [
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "slam_runtime",
        "host_runtime",
    ]
    process_order = {process.name: process.order for process in plan.available_processes}
    assert process_order == {
        "driver_bridge": 10,
        "imu_publisher": 10,
        "lidar_publisher": 10,
        "map_runtime": 20,
        "mujoco_feeder": 20,
        "nav_runtime": 20,
        "slam_runtime": 20,
        "host_runtime": 50,
    }
    assert plan.process("lidar").name == "lidar_publisher"
    assert plan.process("slam").name == "slam_runtime"
    assert plan.process("slam").command.readiness.target == "slam.status.json"
    assert "--navigation-fixture" not in plan.process("lidar").command.argv
    assert plan.native_process_environment["LINGTU_SLAM_MODE"] == "mapping"
    assert plan.native_process_environment["LINGTU_SLAM_CONFIG"].replace("\\", "/") == (
        "src/localization/fastlio2/config/sim_mid360.yaml"
    )
    assert "LINGTU_EXPLORE_ROUTE" not in plan.native_process_environment
    assert plan.native_process_environment["LINGTU_MAPD_EXTENDED_LAYERS"] == "0"
    mid360_stream = plan.simulation["sensor_plan"]["streams"]["mid360"]
    assert len(mid360_stream) == 1
    assert mid360_stream[0]["navigation_fixture_raw_overlay"] is True
    changed = plan.as_dict()
    changed["launch"]["simulation"]["sensor_plan"]["streams"]["mid360"][0]["navigation_fixture_raw_overlay"] = False
    restored = RunPlan.from_dict(changed)
    assert restored.simulation["sensor_plan"]["streams"]["mid360"][0]["navigation_fixture_raw_overlay"] is False
    assert plan.process("maps").name == "map_runtime"
    assert not plan.has_process("traversability")
    nav_command = plan.process("nav").command
    assert "--path-library" not in nav_command.argv
    nav_environment = dict(nav_command.env)
    assert nav_environment["LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS"] == "600"
    assert nav_environment["LINGTU_NAV_ODOM_MAX_AGE_S"] == "0.60"
    assert nav_environment["LINGTU_NAV_CLOUD_MAX_AGE_S"] == "0.60"
    assert nav_environment["LINGTU_NAV_INPUT_RECOVERY_FRAMES"] == "1"
    assert plan.native_process_environment["LINGTU_NAV_LOCAL_PLANNER_BACKEND"] == "scan"
    assert "LINGTU_LOCAL_PLANNER_PATHS" not in plan.native_process_environment
    assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.05"
    assert plan.stop_before_start == (
        "host_runtime",
        "nav_runtime",
        "map_runtime",
        "slam_runtime",
        "mujoco_feeder",
        "imu_publisher",
        "lidar_publisher",
        "driver_bridge",
    )
    assert "VoxelGridModule" not in plan.modules
    assert "TraversabilityCostModule" not in plan.modules


def test_sim_mujoco_explore_live_compiles_exact_native_process_chain(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr("runtime.graph.processes._host_process_platform", lambda: "linux")
    graph = _materialized_platform_graph(tmp_path)
    resolved = resolve_product_host_runtime(
        "explore",
        "sim",
        product_variant="live",
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        product_variant=resolved.product_variant,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    assert plan.product == "explore"
    assert plan.product_variant == "live"
    assert plan.process_control == "subprocess"
    assert plan.lifecycle["slam_mode"] == "mapping"
    assert plan.lifecycle["requires_map"] is False
    assert plan.lifecycle["native_control_mode"] == "autonomy"
    assert plan.native_process_environment["LINGTU_MAPD_EXTENDED_LAYERS"] == "0"
    assert "acceptance" not in plan.as_dict()["launch"]
    assert {process.name for process in plan.processes} == {
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "mujoco_feeder",
        "slam_runtime",
        "map_runtime",
        "nav_runtime",
        "traversability_runtime",
        "explore_runtime",
        "host_runtime",
    }
    assert {role for process in plan.processes for role in process.provides} == {
        "driver",
        "explore",
        "host",
        "imu",
        "lidar",
        "maps",
        "nav",
        "slam",
        "traversability",
    }
    assert plan.host_config["enable_camera"] is False
    assert "camera_backend" not in plan.host_config
    explore = plan.process("explore")
    assert explore.name == "explore_runtime"
    assert explore.command.readiness.kind == "file"
    assert explore.command.readiness.target == "explore.status.json"
    assert "--route" not in explore.command.argv
    assert "LINGTU_EXPLORE_ROUTE" not in dict(explore.command.env)
    assert plan.native_process_environment["LINGTU_EXPLORE_ROUTE"] == "live"


def test_sim_mujoco_explore_map_compiles_saved_map_route_into_run_plan(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr("runtime.graph.processes._host_process_platform", lambda: "linux")
    graph = _materialized_platform_graph(tmp_path)
    resolved = resolve_product_host_runtime(
        "explore",
        "sim",
        product_variant="map",
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        product_variant=resolved.product_variant,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    assert plan.product_variant == "map"
    assert plan.lifecycle["slam_mode"] == "localization"
    assert plan.lifecycle["requires_map"] is True
    assert plan.native_process_environment["LINGTU_EXPLORE_ROUTE"] == "map"
    assert "acceptance" not in plan.as_dict()["launch"]
    assert {process.name for process in plan.processes} == {
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "slam_runtime",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "traversability_runtime",
        "explore_runtime",
        "host_runtime",
    }
    assert "--route" not in plan.process("explore").command.argv
    assert "LINGTU_EXPLORE_ROUTE" not in dict(plan.process("explore").command.env)
    assert "LINGTU_EXPLORE_ROUTE" not in dict(plan.process("traversability").command.env)


@pytest.mark.parametrize(
    ("product", "local_planner"),
    (
        ("nav", "cmu"),
        ("nav", "scan"),
        ("tracking", None),
        ("inspection", None),
    ),
)
def test_sim_mujoco_saved_map_navigation_products_compile_exact_native_chain(
    product: str,
    local_planner: str | None,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "linux",
    )
    graph = _materialized_platform_graph(tmp_path)
    resolved = resolve_product_host_runtime(
        product,
        "sim",
        local_planner=local_planner,
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        local_planner=local_planner,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    assert plan.lifecycle["slam_mode"] == "localization"
    assert plan.lifecycle["requires_map"] is True
    assert plan.native_process_environment["LINGTU_SLAM_MODE"] == "localization"
    selected_local_planner = local_planner or plan.native_nav["local_planner"]
    assert plan.product == product
    assert plan.native_process_environment["LINGTU_NAV_LOCAL_PLANNER_BACKEND"] == selected_local_planner
    nav_argv = plan.process("nav").command.argv
    assert "--path-library" not in nav_argv
    if selected_local_planner == "scan":
        assert "LINGTU_LOCAL_PLANNER_PATHS" not in plan.native_process_environment
        assert "LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M" not in plan.native_process_environment
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.05"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_SIZE_X"] == "200"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_SIZE_Y"] == "200"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_SIZE_Z"] == "100"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_SLIDE_M"] == "0.2"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_RAY_M"] == "5.0"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_P_HIT"] == "0.85"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_P_MISS"] == "0.30"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_P_MIN"] == "0.12"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_P_MAX"] == "0.98"
        assert plan.native_process_environment["LINGTU_MAPD_OCCUPANCY_P_OCC"] == "0.80"
        assert float(plan.native_process_environment["LINGTU_MAPD_INFLATION_RADIUS_M"]) == pytest.approx(
            float(plan.native_process_environment["LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M"])
        )
        assert plan.native_process_environment["LINGTU_MAPD_INFLATION_Z_UP_M"] == "0.10"
        assert plan.native_process_environment["LINGTU_MAPD_INFLATION_Z_DOWN_M"] == "0.10"
        assert float(plan.native_process_environment["LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M"]) == pytest.approx(
            (0.25**2 + 0.3**2) ** 0.5
        )
        assert plan.native_process_environment["LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M"] == "0.25"
        assert plan.native_process_environment["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"] == "0.25"
        assert plan.native_process_environment["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"] == "0.25"
    else:
        assert plan.native_process_environment["LINGTU_LOCAL_PLANNER_PATHS"] == (
            "src/nav/cpp/planning/local/cmu/paths/thunder"
        )
        assert plan.native_process_environment["LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M"] == "0.05"
        assert "LINGTU_MAPD_OCCUPANCY_RESOLUTION_M" not in plan.native_process_environment
    expected_processes = [
        *(["camera_publisher"] if product in {"inspection", "tracking"} else []),
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "slam_runtime",
        "host_runtime",
    ]
    assert [process.name for process in plan.processes] == expected_processes
    assert "acceptance" not in plan.as_dict()["launch"]
    assert "LINGTU_EXPLORE_ROUTE" not in plan.native_process_environment
    assert plan.native_process_environment["LINGTU_MAPD_EXTENDED_LAYERS"] == "0"
    assert not plan.has_process("traversability")
    assert plan.host_config["enable_camera"] is (
        product in {"inspection", "tracking"}
    )
    if product in {"inspection", "tracking"}:
        assert plan.process("camera").name == "camera_publisher"
        assert plan.process("camera") is not plan.process("lidar")
        assert plan.host_config["camera_backend"] == "dds"
        assert plan.host_config["detector"] == "sim_scene"
        assert "encoder" not in plan.host_config
        assert plan.host_config["world"] == (
            "sim/packages/worlds/industrial_park/physics/industrial_park_scene.xml"
        )
        _assert_front_rgbd(plan)
    elif product == "nav":
        assert not plan.has_process("camera")
        assert "camera_backend" not in plan.host_config
        _assert_front_rgbd(plan)


def test_sim_mujoco_map_compiles_exact_rgbd_mapping_chain(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr("runtime.graph.processes._host_process_platform", lambda: "linux")
    graph = _materialized_platform_graph(tmp_path)
    resolved = resolve_product_host_runtime(
        "map",
        "sim",
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    assert plan.product == "map"
    assert plan.process_control == "subprocess"
    assert plan.lifecycle["slam_mode"] == "mapping"
    assert plan.lifecycle["requires_map"] is False
    assert plan.lifecycle["native_control_mode"] == "teleop"
    assert "acceptance" not in plan.as_dict()["launch"]
    assert [process.name for process in plan.processes] == [
        "camera_publisher",
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "slam_runtime",
        "host_runtime",
    ]
    assert plan.process("camera") is not plan.process("lidar")
    assert plan.process("camera") is not plan.process("slam")
    assert plan.process("slam").name == "slam_runtime"
    assert plan.process("camera").name == "camera_publisher"
    assert plan.process("camera").provides == ("camera",)
    assert plan.native_process_environment["LINGTU_SLAM_MODE"] == "mapping"
    assert plan.native_process_environment["LINGTU_MAPD_EXTENDED_LAYERS"] == "1"
    assert plan.host_config["enable_camera"] is True
    assert plan.host_config["use_driver_camera"] is False
    assert plan.host_config["camera_backend"] == "dds"
    assert "camera" in plan.critical_modules
    _assert_front_rgbd(plan)
    assert not plan.has_process("traversability")
    assert not plan.has_process("explore")


def test_subprocess_product_compiles_grouped_typed_processes_once() -> None:
    resolved = resolve_product_host_runtime(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )
    graph = _subprocess_graph()
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        graph=graph,
        env_config={"backend": "mujoco"},
    )

    assert plan.process_control == "subprocess"
    assert "acceptance" not in plan.as_dict()["launch"]
    assert plan.simulation["schema"] == "lingtu.run_plan.simulation.v1"
    assert plan.simulation["session_source"] == ("sim/sessions/examples/thunder_omni_contract/session.yaml")
    assert plan.simulation["session"]["session_id"]
    assert [process.name for process in plan.processes] == [
        "native_runtime",
        "host_runtime",
    ]
    assert [process.name for process in plan.available_processes] == [
        "native_runtime",
        "host_runtime",
    ]
    assert plan.stop_before_start == ("host_runtime", "native_runtime")
    assert plan.process("maps").name == "native_runtime"
    assert plan.process("host").name == "host_runtime"
    assert plan.processes[0].command is not None
    assert "LINGTU_PRODUCT" not in plan.native_process_environment


def test_subprocess_product_compiles_support_process_into_one_run_plan() -> None:
    resolved = resolve_product_host_runtime(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )
    base_plan = compile_run_plan(
        resolved.product,
        resolved.env,
        graph=_subprocess_graph(),
        env_config={"backend": "mujoco"},
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        graph=_subprocess_graph(include_support=True),
        env_config={"backend": "mujoco"},
    )

    assert [process.name for process in plan.processes] == [
        "mujoco_feeder",
        "native_runtime",
        "host_runtime",
    ]
    assert plan.has_process("mujoco_feeder") is False
    with pytest.raises(KeyError, match="mujoco_feeder"):
        plan.process("mujoco_feeder")
    assert plan.stop_before_start == (
        "host_runtime",
        "native_runtime",
        "mujoco_feeder",
    )
    assert plan != base_plan
    assert RunPlan.from_dict(plan.as_dict()).as_dict() == plan.as_dict()


def test_subprocess_mujoco_compile_requires_declared_simulation_preset() -> None:
    graph = _subprocess_graph()
    graph.envs["sim"]["backends"]["mujoco"].pop("presets")
    resolved = resolve_product_host_runtime(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )

    with pytest.raises(ValueError, match="simulation preset"):
        compile_run_plan(
            resolved.product,
            resolved.env,
            graph=graph,
            env_config={"backend": "mujoco"},
        )


def test_resolved_product_blueprint_rejects_unknown_product() -> None:
    with pytest.raises(ValueError, match="Unknown Product"):
        blueprint_for_resolved_product("sim_nav", {})


def test_teleop_avoid_uses_native_operator_motion_not_python_session_module() -> None:
    product = _compile_real("teleop_avoid")

    assert product.product == "teleop_avoid"
    assert product.process_control == "systemd"
    assert product.native_nav["control_mode"] == "teleop_avoid"
    assert product.native_nav["teleop_local_planner"] is True
    assert product.native_nav["check_obstacle"] is True
    assert product.native_nav["use_traversability_cost"] is False
    assert not product.has_process("traversability")
    assert "/nav/operator_motion/control" in product.required_topics
    assert "/nav/operator_motion/sample" in product.required_topics
    assert "/nav/operator_motion/ack" in product.required_topics
    assert "/nav/operator_motion/status" in product.required_topics
    assert "operator_motion_typed_dds_interface" in product.required_capabilities
    assert "native_operator_motion_authority" in product.required_capabilities
    assert "/nav/command/request" in product.required_topics
    assert "/nav/command/ack" in product.required_topics
    assert "operator.motion" not in product.modules
    assert "operator.motion" not in product.critical_modules
    assert "maps.service" not in product.modules
    assert "maps.service" not in product.critical_modules
    assert product.has_process("maps")


@pytest.mark.parametrize("product_name", ["teleop", "teleop_avoid"])
def test_operator_motion_product_passes_compiled_control_boundary_to_gateway(
    monkeypatch,
    product_name: str,
) -> None:
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.delenv("LINGTU_HARDWARE_CONTROL_BOUNDARY", raising=False)
    monkeypatch.delenv("LINGTU_PRODUCT", raising=False)
    monkeypatch.delenv("LINGTU_PROFILE", raising=False)
    product = _compile_real(product_name)
    blueprint = blueprint_from_run_plan(product)

    gateway = next(entry for entry in blueprint._entries if entry.name == "GatewayModule")
    assert gateway.config["command_output_mode"] == "endpoint_only"
    assert gateway.config["hardware_control_boundary"] == "driver"
    assert gateway.config["run_plan"] is product


@pytest.mark.parametrize(
    ("product_name", "require_map_scene"),
    [("teleop", False), ("teleop_avoid", False)],
)
def test_product_topics_control_host_bus_map_scene_requirement(
    product_name: str,
    require_map_scene: bool,
) -> None:
    product = _compile_real(product_name)
    blueprint = blueprint_from_run_plan(product)

    host_bus = next(entry for entry in blueprint._entries if entry.name == "host.bus")
    assert host_bus.config["require_map_scene"] is require_map_scene


def test_map_run_plan_contains_host_blueprint_contract() -> None:
    product = _compile_real("map")
    blueprint = blueprint_from_run_plan(product)
    payload = product.as_dict()

    assert product.modules == blueprint.module_names
    assert product.critical_modules == blueprint.required_module_names
    assert set(payload) == {"identity", "launch", "host", "checks"}
    assert payload["identity"]["schema"] == RUN_PLAN_SCHEMA
    assert payload["identity"]["product"] == "map"
    assert payload["identity"]["env"] == "real"
    assert "endpoint" not in payload
    assert payload["host"]["expected_modules"] == list(product.modules)
    assert payload["checks"]["critical_modules"] == list(product.critical_modules)
    assert payload["host"]["config"] == json.loads(json.dumps(product.host_config))
    assert [process["name"] for process in payload["launch"]["process_catalog"]["selected"]] == [
        "lidar",
        "slam",
        "maps",
        "nav",
        "driver",
        "camera",
        "host",
    ]
    assert product.process("camera").target == "lt-camera.service"
    assert set(payload["identity"]) == {
        "schema",
        "product",
        "product_variant",
        "env",
        "robot",
    }


def test_map_run_plan_preserves_host_config(
    tmp_path,
) -> None:
    operational_overrides = {
        "localization_adapter": "cpp_slam_status",
        "semantic_taxonomy_path": "/opt/lingtu/config/semantics/taxonomy.json",
        "gateway_port": 5051,
        "mcp_port": 8091,
        "enable_gateway": True,
        "enable_teleop": True,
        "enable_camera": True,
        "camera_backend": "dds",
        "camera_jpeg_quality": 82,
        "camera_fps": 18,
        "startup_timeout_s": 33.0,
        "readiness_poll_interval_s": 0.75,
        "stop_timeout_s": 9.0,
        "runtime_failure_grace_s": 6.0,
        "native_navigation_endpoint": "lingtu-nav-dds",
        "planning_frame_id": "map",
        "command_output_mode": "endpoint_only",
        "hardware_control_boundary": "driver",
    }
    resolved = resolve_product_host_runtime(
        "map",
        "real",
        overrides=operational_overrides,
    )
    product = compile_run_plan("map", "real", overrides=operational_overrides)
    repeated = compile_run_plan("map", "real", overrides=operational_overrides)
    payload = product.as_dict()

    assert payload["identity"]["schema"] == RUN_PLAN_SCHEMA
    assert payload == repeated.as_dict()
    host_config = payload["host"]["config"]
    expected_host_config = {
        key: resolved.config[key]
        for key in (
            "_endpoint_transport",
            "_endpoint_contract",
            "slam_profile",
            "localization_adapter",
            "semantic_taxonomy_path",
            "gateway_port",
            "mcp_port",
            "enable_gateway",
            "enable_teleop",
            "enable_camera",
            "camera_backend",
            "camera_jpeg_quality",
            "camera_fps",
            "startup_timeout_s",
            "readiness_poll_interval_s",
            "stop_timeout_s",
            "runtime_failure_grace_s",
            "native_navigation_endpoint",
            "planning_frame_id",
            "command_output_mode",
            "hardware_control_boundary",
        )
    }
    assert {key: host_config[key] for key in expected_host_config} == expected_host_config
    assert host_config == json.loads(json.dumps(product.host_config))
    staging_keys = (
        "_endpoint_transport",
        "_endpoint_contract",
        "command_output_mode",
        "hardware_control_boundary",
    )
    assert {key: host_config[key] for key in staging_keys} == {key: resolved.config[key] for key in staging_keys}
    path = product.write(tmp_path / "map-product.json")
    loaded = RunPlan.load(path)
    assert loaded == product
    assert dict(loaded.host_config) == host_config

    changed = json.loads(path.read_text(encoding="utf-8"))
    changed["host"]["config"]["gateway_port"] = 5052
    path.write_text(json.dumps(changed), encoding="utf-8")
    assert RunPlan.load(path).host_config["gateway_port"] == 5052


def test_product_contract_is_serializable_without_starting_runtime() -> None:
    product = _compile_real("nav")

    payload = product.as_dict()

    assert payload["identity"]["schema"] == RUN_PLAN_SCHEMA
    assert payload["identity"]["env"] == "real"
    assert "endpoint" not in payload
    assert [process["name"] for process in payload["launch"]["process_catalog"]["selected"]] == [
        "lidar",
        "slam",
        "maps",
        "nav",
        "driver",
        "host",
    ]
    assert set(payload["identity"]) == {
        "schema",
        "product",
        "product_variant",
        "env",
        "robot",
    }
    assert payload["launch"]["process_catalog"]["available"]
    assert not hasattr(product, "plan")
    assert payload["host"]["route_contract"] == "robot"
    assert payload["host"]["config"]["_env"] == "real"
    assert payload["checks"]["critical_modules"] == list(product.critical_modules)
    assert payload["checks"]["contracts"] == ["lingtu.product.nav.v1"]
    assert "required_capabilities" not in payload["checks"]
    assert "required_topics" not in payload["checks"]


def test_compiled_lifecycle_omits_dead_hot_switch_candidates() -> None:
    plan = compile_run_plan("map", "real")

    assert "hot_switch_candidates" not in plan.lifecycle


def test_product_env_and_session_parameters_are_resolved_before_launch() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    envs = deepcopy(graph.envs)
    products["nav"]["parameters"] = {"segment.max_distance_m": 3.0}
    envs["real"]["parameter_overrides"] = {"segment.max_distance_m": 4.0}
    parameter_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=envs,
    )

    product = compile_run_plan(
        "nav",
        "real",
        graph=parameter_graph,
        parameter_overrides={"segment.max_distance_m": 2.0},
    )
    payload = product.as_dict()

    assert product.parameters["segment.max_distance_m"] == 2.0
    assert payload["launch"]["parameters"] == product.parameters
    assert set(payload["checks"]) == {
        "contracts",
        "critical_modules",
    }
    assert payload["launch"]["native_process_environment"][
        "LINGTU_NAV_SEGMENT_MAX_DISTANCE_M"
    ] == "2.0"


def test_parameter_validation_uses_declared_traversability_publish_rate() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["traversability_publish_hz"] = 5.0
    parameter_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    with pytest.raises(ValueError, match="two configured map publication periods"):
        _compile_real("nav", graph=parameter_graph)


def test_sim_backend_inherits_env_parameter_overrides(tmp_path: Path) -> None:
    graph = _materialized_platform_graph(tmp_path)
    envs = deepcopy(graph.envs)
    envs["sim"]["parameter_overrides"] = {"segment.max_waypoints": 11}
    envs["sim"]["backends"]["mujoco"].pop("parameter_overrides", None)
    parameter_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )
    resolved = resolve_product_host_runtime(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        graph=parameter_graph,
        env_config={"backend": "mujoco"},
    )

    assert plan.parameters["segment.max_waypoints"] == 11


def test_sim_backend_parameter_overrides_replace_env_defaults(tmp_path: Path) -> None:
    graph = _materialized_platform_graph(tmp_path)
    envs = deepcopy(graph.envs)
    envs["sim"]["parameter_overrides"] = {"segment.max_waypoints": 11}
    envs["sim"]["backends"]["mujoco"]["parameter_overrides"] = {
        "segment.max_waypoints": 3,
    }
    parameter_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )
    resolved = resolve_product_host_runtime(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        graph=parameter_graph,
        env_config={"backend": "mujoco"},
    )

    assert plan.parameters["segment.max_waypoints"] == 3


def test_compiled_run_plan_returns_defensive_config_copies() -> None:
    product = _compile_real("nav")

    config_copy = product.host_config
    config_copy["gateway_port"] = 9999
    native_copy = product.native_nav
    native_copy["control_mode"] = "teleop"

    assert product.host_config["gateway_port"] != 9999
    assert product.native_nav["control_mode"] == "autonomy"


def test_nav_octoplanner_radius_comes_from_native_nav_contract() -> None:
    plan = compile_run_plan("nav", "sim", env_config={"backend": "mujoco"})

    assert float(plan.native_process_environment["LINGTU_NAV_OCTO_ROBOT_RADIUS_M"]) == pytest.approx(
        float(plan.native_process_environment["LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M"])
        + float(plan.native_process_environment["LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M"])
    )


def test_product_compile_defers_startup_preflight(monkeypatch) -> None:
    import lingtu.assembly.stacks.system as system

    calls: list[str] = []
    monkeypatch.setattr(
        system,
        "run_startup_preflight",
        lambda **_: calls.append("preflight"),
    )
    _compile_real("nav")

    assert calls == []


def test_blueprint_runs_deferred_checks_only_when_building() -> None:
    calls: list[str] = []
    blueprint = Blueprint().before_build(lambda: calls.append("preflight"))

    assert calls == []
    blueprint.build()

    assert calls == ["preflight"]


def test_run_plan_round_trip_and_contract_change_is_explicit(tmp_path) -> None:
    product = _compile_real("nav")
    path = product.write(tmp_path / "product.json")

    loaded = RunPlan.load(path)

    assert loaded == product
    assert loaded.process("host").target == "lt-host.service"
    assert loaded.required_capabilities == product.required_capabilities
    assert json.dumps(dict(loaded.host_config), sort_keys=True) == json.dumps(
        product.host_config,
        sort_keys=True,
    )

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["checks"]["contracts"] = ["lingtu.product.tracking.v1"]
    path.write_text(json.dumps(payload), encoding="utf-8")

    changed = RunPlan.load(path)
    assert changed.contracts == ("lingtu.product.tracking.v1",)
    assert changed != product


def test_run_plan_materializes_the_declared_host_blueprint() -> None:
    plan = _compile_real("nav")
    blueprint = blueprint_from_run_plan(plan)

    assert blueprint.module_names == plan.modules
    assert blueprint.required_module_names == plan.critical_modules
    gateway_entry = next(entry for entry in blueprint._entries if entry.name == "GatewayModule")
    assert gateway_entry.config["run_plan"] is plan


def test_acceptance_manifest_loader_reads_the_complete_extends_closure(
    tmp_path: Path,
) -> None:
    parent = tmp_path / "base.json"
    child = tmp_path / "child.json"
    parent.write_text('{"nested":{"base":1,"kept":true}}', encoding="utf-8")
    child.write_text(
        '{"extends":"base.json","nested":{"base":2}}',
        encoding="utf-8",
    )
    before = load_manifest(child, root=tmp_path)

    parent.write_text('{"nested":{"base":1,"kept":false}}', encoding="utf-8")
    after = load_manifest(child, root=tmp_path)

    assert before == {"nested": {"base": 2, "kept": True}}
    assert after == {"nested": {"base": 2, "kept": False}}


_SIM_MUJOCO_DDS_PROCESS_NAMES = (
    "lidar_publisher",
    "imu_publisher",
    "camera_publisher",
    "driver_bridge",
    "slam_runtime",
    "map_runtime",
    "traversability_runtime",
    "nav_runtime",
    "explore_runtime",
    "host_runtime",
)


@pytest.mark.parametrize(
    ("process_platform", "expected_domain"),
    (("windows", 17), ("linux", 231)),
)
def test_sim_mujoco_platform_catalog_uses_one_safe_dds_domain(
    process_platform: str,
    expected_domain: int,
) -> None:
    processes = load_runtime_graph().envs["sim"]["backends"]["mujoco"]["processes"]

    for name in _SIM_MUJOCO_DDS_PROCESS_NAMES:
        command = processes[name]["platforms"][process_platform]
        assert command["env"]["LINGTU_DDS_DOMAIN_ID"] == str(expected_domain)
        if name == "host_runtime":
            continue
        option = "--domain" if name == "explore_runtime" else "--domain-id"
        option_index = command["argv"].index(option)
        assert command["argv"][option_index + 1] == str(expected_domain)

    if process_platform == "windows":
        highest_default_port = 7400 + 250 * expected_domain + 11
        assert highest_default_port < 49152


@pytest.mark.parametrize(
    ("product", "product_variant"),
    (
        ("teleop", None),
        ("teleop_avoid", None),
        ("map", None),
        ("nav", None),
        ("tracking", None),
        ("inspection", None),
        ("explore", "live"),
        ("explore", "map"),
    ),
)
def test_every_windows_sim_product_run_plan_keeps_selected_processes_on_domain_17(
    product: str,
    product_variant: str | None,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    graph = _materialized_platform_graph(tmp_path)
    monkeypatch.setattr(
        "runtime.graph.processes._host_process_platform",
        lambda: "windows",
    )
    resolved = resolve_product_host_runtime(
        product,
        "sim",
        product_variant=product_variant,
        env_config={"backend": "mujoco"},
    )

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        product_variant=resolved.product_variant,
        env_config={"backend": "mujoco"},
        graph=graph,
    )

    selected_dds_processes = {
        process.name: process.command for process in plan.processes if process.name in _SIM_MUJOCO_DDS_PROCESS_NAMES
    }
    assert selected_dds_processes
    for name, command in selected_dds_processes.items():
        assert command is not None
        assert dict(command.env)["LINGTU_DDS_DOMAIN_ID"] == "17"
        if name == "host_runtime":
            continue
        option = "--domain" if name == "explore_runtime" else "--domain-id"
        option_index = command.argv.index(option)
        assert command.argv[option_index + 1] == "17"
