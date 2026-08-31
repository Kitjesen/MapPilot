from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from lingtu.assembly.compiler import compile_run_plan
from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.validation import validate_product
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph import (
    RuntimeGraph,
    load_runtime_graph,
    render_env_mermaid,
    render_product_markdown,
    resolve_env_implementation,
    resolve_processes,
    validate_runtime_graph,
)
from runtime.runtime_interface import TOPICS

REPO_ROOT = Path(__file__).resolve().parents[3]

REAL_PRODUCTS = tuple(sorted(load_runtime_graph().products))
INSPECTION_DDS_TOPICS = (
    TOPICS.inspection_task_request,
    TOPICS.inspection_task_ack,
    TOPICS.inspection_status,
    TOPICS.inspection_task_event,
    TOPICS.inspection_evidence_request,
    TOPICS.inspection_evidence_result,
)


def _direct_process_definition(
    owner: str,
    order: int,
    provides: list[str],
    *,
    artifact_path: str = "src/lingtu/run_plan.py",
) -> dict[str, object]:
    return {
        "target": owner,
        "lifecycle": "mode",
        "order": order,
        "timeout_s": 10,
        "provides": provides,
        "command": {
            "argv": [artifact_path, "--owner", owner],
            "cwd": ".",
            "env": {"LINGTU_LOG_LEVEL": "info"},
            "artifact": {"path": artifact_path},
            "readiness": {"kind": "process"},
        },
    }


def _subprocess_runtime_graph() -> RuntimeGraph:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    backend = envs["sim"]["backends"]["mujoco"]
    backend.pop("acceptance", None)
    backend["process_control"] = "subprocess"
    backend["process_manager"] = "direct"
    backend["supported_products"] = list(graph.products)
    backend.pop("support_processes", None)
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
    backend["processes"] = {
        "native_runtime": _direct_process_definition(
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
        "host_runtime": _direct_process_definition(
            "host_runtime",
            20,
            ["host"],
        ),
        "diagnostics_runtime": {
            **_direct_process_definition(
                "diagnostics_runtime",
                30,
                ["diagnostics"],
            ),
            "lifecycle": "persistent",
        },
    }
    backend["stop_before_start"] = ["host_runtime", "native_runtime"]
    return RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )


def _endpoint_contract(
    graph: RuntimeGraph,
    env: str,
    *,
    backend: str | None = None,
) -> dict[str, object]:
    env_config = {"backend": backend} if backend else None
    implementation = resolve_env_implementation(
        env,
        graph=graph,
        env_config=env_config,
    )
    return dict(implementation["endpoints"]["contract"])


def _native_endpoint_contracts(graph: RuntimeGraph) -> tuple[dict[str, object], ...]:
    return (
        _endpoint_contract(graph, "real"),
        _endpoint_contract(graph, "sim", backend="mujoco"),
    )


def test_runtime_graph_contracts_are_valid() -> None:
    graph = load_runtime_graph()

    assert set(graph.envs) == {"real", "sim"}
    assert {env["schema_version"] for env in graph.envs.values()} == {"lingtu.runtime_graph.env.v1"}
    assert {product["schema_version"] for product in graph.products.values()} == {"lingtu.runtime_graph.product.v1"}
    assert "robot_config_ref" not in graph.envs["real"]
    assert set(graph.envs["sim"]["backends"]) == {"mujoco"}
    assert "inspection" in graph.envs["sim"]["supported_products"]
    assert "inspection" in graph.envs["sim"]["backends"]["mujoco"]["supported_products"]
    assert "default_backend" not in graph.envs["sim"]
    assert validate_runtime_graph(graph) == []


def test_nav_acceptance_selects_local_planner_without_creating_another_product() -> None:
    graph = load_runtime_graph()
    nav = graph.products["nav"]
    mujoco = graph.envs["sim"]["backends"]["mujoco"]

    assert "nav_scan" not in graph.products
    assert nav["operator_switchable"] is True
    assert nav["default_for_session_mode"] is True
    assert nav["native_nav"]["local_planner"] == "cmu"
    assert nav["native_nav"]["local_planners"] == ["cmu", "scan"]
    assert mujoco["acceptance"]["products"]["nav"]["local_planners"] == {
            "cmu": {
                "runner": "sim/scripts/mujoco/native_navigation_acceptance.py",
                "manifest": "config/runtime_graph/acceptance/mujoco_local_cmu.json",
            },
        "scan": {
            "runner": "sim/scripts/mujoco/native_navigation_acceptance.py",
            "manifest": "config/runtime_graph/acceptance/mujoco_local_scan.json",
        },
    }


@pytest.mark.parametrize("mutation", ("missing", "unknown", "ambiguous"))
def test_sim_process_platform_schema_is_strict(mutation: str) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    process = envs["sim"]["backends"]["mujoco"]["processes"]["driver_bridge"]
    if mutation == "missing":
        process["platforms"] = {}
    elif mutation == "unknown":
        process["platforms"]["darwin"] = deepcopy(process["platforms"]["linux"])
    else:
        process["command"] = deepcopy(process["platforms"]["windows"])
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_process_invalid" for issue in issues)


@pytest.mark.parametrize(
    "mutation",
    ("not_list", "missing_path", "extra_field", "loader_mismatch"),
)
def test_sim_process_dependency_schema_is_strict(mutation: str) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    command = envs["sim"]["backends"]["mujoco"]["processes"]["host_runtime"]["platforms"]["windows"]
    if mutation == "not_list":
        command["dependencies"] = "nav_client.dll"
    elif mutation == "missing_path":
        command["dependencies"] = [{}]
    elif mutation == "extra_field":
        command["dependencies"][0]["sha256"] = "a" * 64
    elif mutation == "loader_mismatch":
        command["env"]["LINGTU_NAV_CLIENT_LIB"] = "build/untracked-nav-client.dll"
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    assert any(issue.code == "env_process_invalid" for issue in validate_runtime_graph(broken))


def test_sim_acceptance_catalog_must_cover_supported_products_exactly() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    products = envs["sim"]["backends"]["mujoco"]["acceptance"]["products"]
    del products["teleop_avoid"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_acceptance_product_coverage_invalid" for issue in issues)


def test_sim_acceptance_catalog_rejects_empty_paths() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["sim"]["backends"]["mujoco"]["acceptance"]["products"]["teleop"]["runner"] = ""
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_acceptance_path_invalid" for issue in issues)


def test_sim_acceptance_catalog_rejects_extra_target_fields() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["sim"]["backends"]["mujoco"]["acceptance"]["products"]["teleop"]["fallback"] = (
        "sim/scripts/mujoco/teleop_native_acceptance.py"
    )
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_acceptance_target_invalid" for issue in issues)


@pytest.mark.parametrize("variant", ("live", "map"))
def test_sim_acceptance_catalog_requires_every_declared_product_variant(
    variant: str,
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    variants = envs["sim"]["backends"]["mujoco"]["acceptance"]["products"]["explore"]["variants"]
    del variants[variant]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_acceptance_variant_coverage_invalid" for issue in issues)


def test_sim_acceptance_catalog_rejects_extra_product_variant() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    variants = envs["sim"]["backends"]["mujoco"]["acceptance"]["products"]["explore"]["variants"]
    variants["missing"] = deepcopy(variants["live"])
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_acceptance_variant_coverage_invalid" for issue in issues)


def test_sim_acceptance_catalog_rejects_flat_and_variant_target_ambiguity() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    target = envs["sim"]["backends"]["mujoco"]["acceptance"]["products"]["explore"]
    target["runner"] = "sim/scripts/mujoco/explore_native_acceptance.py"
    target["manifest"] = "config/runtime_graph/acceptance/mujoco_explore_native_acceptance.json"
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_acceptance_target_invalid" for issue in issues)


def test_sim_acceptance_catalog_rejects_empty_variant_paths() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["sim"]["backends"]["mujoco"]["acceptance"]["products"]["explore"]["variants"]["map"]["manifest"] = ""
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_acceptance_path_invalid" for issue in issues)


@pytest.mark.parametrize(
    ("topic_name", "schema", "nominal_rate_hz"),
    (
        (TOPICS.raw_lidar_points, "lingtu.dds.LivoxFrame", 10.0),
        (TOPICS.raw_imu, "lingtu.dds.Imu", 200.0),
        ("/camera/color/image_raw", "lingtu.dds.Image", 30.0),
        ("/camera/depth/image_raw", "lingtu.dds.Image", 30.0),
    ),
)
def test_sim_sensor_topics_are_the_single_contract_source(
    topic_name: str,
    schema: str,
    nominal_rate_hz: float,
) -> None:
    contract = load_runtime_graph().topic_contracts[topic_name]

    assert contract["schema"] == schema
    assert contract["nominal_rate_hz"] == nominal_rate_hz


def test_camera_info_is_latched_calibration_without_a_claimed_frequency() -> None:
    contract = load_runtime_graph().topic_contracts["/camera/color/camera_info"]

    assert contract["schema"] == "lingtu.dds.CameraInfo"
    assert "nominal_rate_hz" not in contract
    assert contract["qos"] == "reliable_transient_local_keep_last_1"
    assert contract["semantics"] == ("latched_camera_calibration_published_once_per_endpoint_start_for_late_joiners")


def test_raw_imu_uses_the_dedicated_imu_frame() -> None:
    assert load_runtime_graph().topic_contracts[TOPICS.raw_imu]["frame"] == "imu_link"


def test_gnss_fix_is_event_driven_without_a_claimed_frequency() -> None:
    contract = load_runtime_graph().topic_contracts["/gnss/fix"]

    assert contract["schema"] == "lingtu.dds.GnssFix"
    assert "nominal_rate_hz" not in contract


def test_runtime_graph_has_no_second_sensor_catalog() -> None:
    graph = load_runtime_graph()

    assert not hasattr(graph, "sensors")
    assert not (REPO_ROOT / "config/runtime_graph/sensors.yaml").exists()
    assert "sensor_profiles" not in graph.envs["sim"]["backends"]["mujoco"]
    declared = "\n".join(graph.topic_contracts)
    for undeclared_sensor in (
        "infrared",
        "fisheye",
        "panoramargb",
        "airy",
        "actor_lidar",
    ):
        assert undeclared_sensor not in declared


def test_sim_odometry_prior_remains_diagnostic_only() -> None:
    contract = load_runtime_graph().topic_contracts["/slam/odom_prior"]

    assert contract["simulation_only"] is True
    assert contract["real_equivalent_required"] is False
    assert "nominal_rate_hz" not in contract


def test_runtime_graph_rejects_an_unknown_product_schema() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["schema_version"] = "lingtu.runtime_graph.product.v0"
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    assert "product_schema_invalid" in {issue.code for issue in validate_runtime_graph(broken)}


def test_final_velocity_topic_is_identity_bound_and_single_writer() -> None:
    topic = load_runtime_graph().topics["topics"][TOPICS.cmd_vel]

    assert topic["role"] == "final_velocity_command"
    assert topic["schema"] == "final_velocity_command"
    assert topic["single_writer_per_product"] is True
    assert topic["semantics"] == "identity_bound_freshness_bounded_final_velocity_envelope"


def test_real_env_resolves_each_product_role_to_one_process_owner() -> None:
    graph = load_runtime_graph()
    available = resolve_processes("nav", "real", graph=graph)[1]

    assert {role for process in available for role in process.provides} == set(graph.products["nav"]["processes"])
    assert len({process.target for process in available}) == len(available)
    for product_name, product in graph.products.items():
        selected, _available, _conflicts, _support = resolve_processes(
            product_name,
            "real",
            graph=graph,
        )
        assert {role for process in selected for role in process.provides} == set(product["processes"])
        assert len({process.target for process in selected}) == len(selected)

    driver = next(process for process in available if process.name == "driver")
    assert driver.lifecycle == "mode"


@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    (
        ("drop_role", "env_role_ownership_mismatch"),
        ("share_owner", "env_process_owner_duplicate"),
    ),
)
def test_real_env_rejects_incomplete_or_non_unique_role_ownership(
    mutation: str,
    expected_code: str,
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    if mutation == "drop_role":
        envs["real"]["provided_roles"].remove("maps")
    else:
        envs["real"]["processes"]["maps"]["target"] = envs["real"]["processes"]["slam"]["target"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == expected_code for issue in issues)


def test_process_resolution_rejects_unmapped_product_process() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["processes"].append("missing")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    with pytest.raises(ValueError, match="missing"):
        resolve_processes("nav", "real", graph=broken)


@pytest.mark.parametrize(
    "conflicts",
    (
        [Path("retired.service")],
        [" retired.service"],
        ["retired.service "],
    ),
)
def test_process_resolution_does_not_coerce_conflict_targets(
    conflicts: list[object],
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["conflicts"] = conflicts
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    with pytest.raises(ValueError, match="conflict"):
        resolve_processes("nav", "real", graph=broken)


@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    (
        ("not_list", "env_process_conflicts_invalid"),
        ("non_string", "env_process_conflicts_invalid"),
        ("whitespace", "env_process_conflicts_invalid"),
        ("invalid_target", "env_process_conflicts_invalid"),
        ("duplicate", "env_process_conflicts_invalid"),
        ("overlap", "env_process_conflict_overlap"),
    ),
)
def test_runtime_graph_static_validation_rejects_invalid_conflicts(
    mutation: str,
    expected_code: str,
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    env = envs["real"]
    if mutation == "not_list":
        env["conflicts"] = "retired.service"
    elif mutation == "non_string":
        env["conflicts"] = [Path("retired.service")]
    elif mutation == "whitespace":
        env["conflicts"] = [" retired.service"]
    elif mutation == "invalid_target":
        env["conflicts"] = ["--retired.service"]
    elif mutation == "duplicate":
        env["conflicts"] = ["retired.service", "retired.service"]
    elif mutation == "overlap":
        env["conflicts"] = [env["processes"]["nav"]["target"]]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    codes = {issue.code for issue in validate_runtime_graph(broken) if issue.scope == "env:real"}
    assert expected_code in codes


def test_sim_process_resolution_requires_an_explicit_supported_backend(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("runtime.graph.processes._host_process_platform", lambda: "windows")
    with pytest.raises(ValueError, match=r"env_config\.backend"):
        resolve_processes("teleop", "sim")

    selected, available, conflicts, support = resolve_processes(
        "teleop",
        "sim",
        env_config={"backend": "mujoco"},
    )

    assert [process.name for process in selected] == [
        "driver_bridge",
        "mujoco_feeder",
        "nav_runtime",
        "host_runtime",
    ]
    assert available == selected
    assert conflicts == ()
    assert support == ("mujoco_feeder",)
    map_selected, map_available, map_conflicts, map_support = resolve_processes(
        "map", "sim", env_config={"backend": "mujoco"}
    )
    assert [process.name for process in map_selected] == [
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
    assert map_available == map_selected
    assert map_conflicts == ()
    assert map_support == ("mujoco_feeder",)
    slam = next(process for process in map_selected if process.name == "slam_runtime")
    windows_map = next(process for process in map_selected if process.name == "map_runtime")
    assert "--disable-query" in windows_map.command.argv
    assert slam.command.artifact.path == ("build/slam-core-windows-x64/stage/bin/slamd.exe")
    assert len(slam.command.dependencies) == 19

    map_platforms = load_runtime_graph().envs["sim"]["backends"]["mujoco"]["processes"]["map_runtime"]["platforms"]
    assert "--disable-query" in map_platforms["windows"]["argv"]
    assert "--disable-query" not in map_platforms["linux"]["argv"]
    with pytest.raises(ValueError, match="unknown backend 'cmu_unity'"):
        resolve_processes(
            "explore",
            "sim",
            env_config={"backend": "cmu_unity"},
        )


def test_sim_mujoco_teleop_avoid_resolves_one_native_owner_per_required_role(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("runtime.graph.processes._host_process_platform", lambda: "windows")
    selected, available, conflicts, support = resolve_processes(
        "teleop_avoid",
        "sim",
        env_config={"backend": "mujoco"},
    )

    assert [process.name for process in selected] == [
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "slam_runtime",
        "traversability_runtime",
        "host_runtime",
    ]
    assert available == selected
    assert conflicts == ()
    assert support == ("mujoco_feeder",)
    owners = {role: process.name for process in selected for role in process.provides}
    assert owners == {
        "driver": "driver_bridge",
        "host": "host_runtime",
        "imu": "imu_publisher",
        "lidar": "lidar_publisher",
        "maps": "map_runtime",
        "nav": "nav_runtime",
        "slam": "slam_runtime",
        "traversability": "traversability_runtime",
    }


def test_sim_slam_product_compiles_the_dedicated_slam_runtime(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("runtime.graph.processes._host_process_platform", lambda: "windows")

    plan = compile_run_plan(
        "map",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
    )

    assert plan.process("slam").name == "slam_runtime"
    assert plan.process("slam").provides == ("slam",)
    assert plan.process("lidar").provides == ("lidar",)


def test_subprocess_process_resolution_groups_logical_roles_by_physical_owner() -> None:
    selected, available, conflicts, support = resolve_processes(
        "nav",
        "sim",
        graph=_subprocess_runtime_graph(),
        env_config={"backend": "mujoco"},
    )

    assert [process.name for process in selected] == [
        "native_runtime",
        "host_runtime",
    ]
    assert [process.name for process in available] == [
        "native_runtime",
        "host_runtime",
    ]
    assert selected[0].provides == (
        "lidar",
        "imu",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "camera",
        "explore",
    )
    assert conflicts == ()
    assert support == ()


def test_subprocess_process_resolution_selects_explicit_support_process() -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["support_processes"] = ["mujoco_feeder"]
    backend["processes"]["mujoco_feeder"] = _direct_process_definition(
        "mujoco_feeder",
        5,
        [],
    )
    backend["stop_before_start"].append("mujoco_feeder")

    selected, available, _conflicts, support = resolve_processes(
        "nav",
        "sim",
        graph=graph,
        env_config={"backend": "mujoco"},
    )

    assert [process.name for process in selected] == [
        "mujoco_feeder",
        "native_runtime",
        "host_runtime",
    ]
    assert next(process for process in available if process.name == "mujoco_feeder").provides == ()
    assert support == ("mujoco_feeder",)


@pytest.mark.parametrize(
    "mutation",
    (
        "not_list",
        "non_trimmed",
        "invalid_name",
        "duplicate",
        "unknown",
        "provides_role",
        "persistent",
        "provided_role_name",
        "product_role_name",
    ),
)
def test_subprocess_process_resolution_rejects_invalid_support_processes(
    mutation: str,
) -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["support_processes"] = ["mujoco_feeder"]
    backend["processes"]["mujoco_feeder"] = _direct_process_definition(
        "mujoco_feeder",
        5,
        [],
    )
    backend["stop_before_start"].append("mujoco_feeder")
    if mutation == "not_list":
        backend["support_processes"] = "mujoco_feeder"
    elif mutation == "non_trimmed":
        backend["support_processes"] = [" mujoco_feeder"]
    elif mutation == "invalid_name":
        backend["support_processes"] = ["mujoco-feeder"]
    elif mutation == "duplicate":
        backend["support_processes"] = ["mujoco_feeder", "mujoco_feeder"]
    elif mutation == "unknown":
        del backend["processes"]["mujoco_feeder"]
    elif mutation == "provides_role":
        backend["processes"]["mujoco_feeder"]["provides"] = ["feeder"]
        backend["provided_roles"].append("feeder")
    elif mutation == "persistent":
        backend["processes"]["mujoco_feeder"]["lifecycle"] = "persistent"
    elif mutation == "provided_role_name":
        backend["provided_roles"].append("mujoco_feeder")
    elif mutation == "product_role_name":
        products = deepcopy(graph.products)
        products["nav"]["processes"].append("mujoco_feeder")
        graph = RuntimeGraph(
            root=graph.root,
            topics=graph.topics,
            products=products,
            envs=graph.envs,
        )

    with pytest.raises(ValueError, match="support"):
        resolve_processes(
            "nav",
            "sim",
            graph=graph,
            env_config={"backend": "mujoco"},
        )


def test_subprocess_process_resolution_allows_duplicate_order_as_barrier_stage() -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["processes"]["host_runtime"]["order"] = 10
    backend["processes"]["diagnostics_runtime"]["order"] = 10

    selected, available, _conflicts, _support = resolve_processes(
        "nav",
        "sim",
        graph=graph,
        env_config={"backend": "mujoco"},
    )

    assert [process.order for process in available] == [10, 10]
    assert [process.name for process in available] == [
        "host_runtime",
        "native_runtime",
    ]
    assert {role for process in selected for role in process.provides} >= {
        "nav",
        "host",
    }


@pytest.mark.parametrize(
    ("mutation", "error"),
    (
        ("missing_provides", "provides"),
        ("empty_provides", "does not map logical process roles"),
        ("missing_command", "command"),
        ("artifact_sha", "artifact"),
        ("artifact_not_argv", "artifact.*argv"),
        ("unknown_command_field", "command"),
        ("unknown_process_field", "process"),
        ("duplicate_role", "duplicate.*role"),
        ("role_union_mismatch", "does not declare logical process roles"),
        ("manager_mismatch", "process manager"),
    ),
)
def test_subprocess_process_resolution_rejects_invalid_direct_contracts(
    mutation: str,
    error: str,
) -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    native = backend["processes"]["native_runtime"]
    if mutation == "missing_provides":
        native.pop("provides")
    elif mutation == "empty_provides":
        native["provides"] = []
    elif mutation == "missing_command":
        native.pop("command")
    elif mutation == "artifact_sha":
        native["command"]["artifact"]["sha256"] = "a" * 64
    elif mutation == "artifact_not_argv":
        native["command"]["artifact"]["path"] = "src/runtime/graph/processes.py"
    elif mutation == "unknown_command_field":
        native["command"]["shell"] = True
    elif mutation == "unknown_process_field":
        native["shell"] = True
    elif mutation == "duplicate_role":
        backend["processes"]["host_runtime"]["provides"].append("nav")
    elif mutation == "role_union_mismatch":
        backend["provided_roles"].remove("nav")
    elif mutation == "manager_mismatch":
        backend["process_manager"] = "systemd"

    with pytest.raises(ValueError, match=error):
        resolve_processes(
            "nav",
            "sim",
            graph=graph,
            env_config={"backend": "mujoco"},
        )


def test_subprocess_backend_static_validation_accepts_grouped_owner_contract() -> None:
    scoped_issues = [
        issue
        for issue in validate_runtime_graph(_subprocess_runtime_graph())
        if issue.scope == "env:sim backend:mujoco"
        and not (issue.code == "native_endpoint_contract_missing" and "/nav/local_traversability" in issue.message)
    ]

    assert scoped_issues == []


def test_subprocess_backend_static_validation_accepts_explicit_support_process() -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["support_processes"] = ["mujoco_feeder"]
    backend["processes"]["mujoco_feeder"] = _direct_process_definition(
        "mujoco_feeder",
        5,
        [],
    )
    backend["stop_before_start"].append("mujoco_feeder")

    scoped_issues = [issue for issue in validate_runtime_graph(graph) if issue.scope == "env:sim backend:mujoco"]

    assert scoped_issues == []


def test_subprocess_backend_static_validation_accepts_shared_order_stage() -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["support_processes"] = ["mujoco_feeder"]
    backend["processes"]["mujoco_feeder"] = _direct_process_definition(
        "mujoco_feeder",
        10,
        [],
    )
    backend["stop_before_start"].append("mujoco_feeder")

    scoped_codes = {issue.code for issue in validate_runtime_graph(graph) if issue.scope == "env:sim backend:mujoco"}

    assert "env_process_order_duplicate" not in scoped_codes


def test_subprocess_backend_static_validation_rejects_orphan_support_process() -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["processes"]["mujoco_feeder"] = _direct_process_definition(
        "mujoco_feeder",
        5,
        [],
    )

    scoped_codes = {issue.code for issue in validate_runtime_graph(graph) if issue.scope == "env:sim backend:mujoco"}

    assert "env_support_processes_invalid" in scoped_codes


@pytest.mark.parametrize(
    "mutation",
    (
        "not_list",
        "non_trimmed",
        "invalid_name",
        "duplicate",
        "unknown",
        "provides_role",
        "persistent",
        "provided_role_name",
        "product_role_name",
    ),
)
def test_subprocess_backend_static_validation_rejects_invalid_support_processes(
    mutation: str,
) -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["support_processes"] = ["mujoco_feeder"]
    backend["processes"]["mujoco_feeder"] = _direct_process_definition(
        "mujoco_feeder",
        5,
        [],
    )
    if mutation == "not_list":
        backend["support_processes"] = "mujoco_feeder"
    elif mutation == "non_trimmed":
        backend["support_processes"] = [" mujoco_feeder"]
    elif mutation == "invalid_name":
        backend["support_processes"] = ["mujoco-feeder"]
    elif mutation == "duplicate":
        backend["support_processes"] = ["mujoco_feeder", "mujoco_feeder"]
    elif mutation == "unknown":
        backend["support_processes"] = ["missing_support"]
    elif mutation == "provides_role":
        backend["processes"]["mujoco_feeder"]["provides"] = ["feeder"]
        backend["provided_roles"].append("feeder")
    elif mutation == "persistent":
        backend["processes"]["mujoco_feeder"]["lifecycle"] = "persistent"
    elif mutation == "provided_role_name":
        backend["provided_roles"].append("mujoco_feeder")
    elif mutation == "product_role_name":
        products = deepcopy(graph.products)
        products["nav"]["processes"].append("mujoco_feeder")
        graph = RuntimeGraph(
            root=graph.root,
            topics=graph.topics,
            products=products,
            envs=graph.envs,
        )

    scoped_codes = {issue.code for issue in validate_runtime_graph(graph) if issue.scope == "env:sim backend:mujoco"}

    assert "env_support_processes_invalid" in scoped_codes


@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    (
        ("missing_command", "env_process_invalid"),
        ("artifact_sha", "env_process_invalid"),
        ("unknown_command_field", "env_process_invalid"),
        ("unknown_process_field", "env_process_invalid"),
        ("duplicate_role", "env_process_role_duplicate"),
        ("role_union_mismatch", "env_role_ownership_mismatch"),
        ("manager_mismatch", "env_process_manager_invalid"),
    ),
)
def test_subprocess_backend_static_validation_rejects_invalid_contracts(
    mutation: str,
    expected_code: str,
) -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    native = backend["processes"]["native_runtime"]
    if mutation == "missing_command":
        native.pop("command")
    elif mutation == "artifact_sha":
        native["command"]["artifact"]["sha256"] = "a" * 64
    elif mutation == "unknown_command_field":
        native["command"]["shell"] = True
    elif mutation == "unknown_process_field":
        native["shell"] = True
    elif mutation == "duplicate_role":
        backend["processes"]["host_runtime"]["provides"].append("nav")
    elif mutation == "role_union_mismatch":
        backend["provided_roles"].remove("diagnostics")
    elif mutation == "manager_mismatch":
        backend["process_manager"] = "systemd"

    scoped_codes = {issue.code for issue in validate_runtime_graph(graph) if issue.scope == "env:sim backend:mujoco"}
    assert expected_code in scoped_codes


def test_subprocess_backend_requires_every_supported_product_role() -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    backend["processes"]["native_runtime"]["provides"].remove("explore")
    backend["provided_roles"].remove("explore")

    scoped_issues = [issue for issue in validate_runtime_graph(graph) if issue.scope == "env:sim backend:mujoco"]

    assert any(
        issue.code == "env_product_process_missing"
        and "Product explore" in issue.message
        and "explore" in issue.message
        for issue in scoped_issues
    )


def test_subprocess_static_validation_does_not_require_built_artifact() -> None:
    graph = _subprocess_runtime_graph()
    backend = graph.envs["sim"]["backends"]["mujoco"]
    missing = "build/not-yet-built/native-runtime"
    native = backend["processes"]["native_runtime"]
    native["command"]["artifact"]["path"] = missing
    native["command"]["argv"][0] = missing

    scoped_issues = [issue for issue in validate_runtime_graph(graph) if issue.scope == "env:sim backend:mujoco"]
    assert not any(issue.code == "env_process_invalid" for issue in scoped_issues)
    with pytest.raises(ValueError, match=r"artifact.*does not exist"):
        resolve_processes(
            "nav",
            "sim",
            graph=graph,
            env_config={"backend": "mujoco"},
        )


def test_sim_backend_rejects_missing_supported_product_topic() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    contract = envs["sim"]["backends"]["mujoco"]["endpoints"]["contract"]
    contract["exposed_topics"].remove(TOPICS.nav_state)
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_product_topic_missing" for issue in issues)
def test_env_resolution_rejects_legacy_deployment_selector_names() -> None:
    for legacy_name in ("legacy_real_target", "mujoco_native_dds", "sim_mujoco_live"):
        with pytest.raises(ValueError, match="unknown Runtime Graph env"):
            resolve_env_implementation(legacy_name)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("target", "--no-block.service", "target is invalid"),
        ("order", "10", "order is invalid"),
        ("timeout_s", 0, "timeout is invalid"),
    ),
)
def test_process_resolution_rejects_unsafe_process_specs(
    field: str,
    value: object,
    message: str,
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["processes"]["nav"][field] = value
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    with pytest.raises(ValueError, match=message):
        resolve_processes("nav", "real", graph=broken)


def test_runtime_graph_rejects_process_contract_drift() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["processes"].remove("traversability")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_process_missing" and "traversability" in issue.message for issue in issues)


def test_runtime_graph_requires_imu_for_slam_products() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["processes"].remove("imu")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_process_missing" and "process imu" in issue.message for issue in issues)


def test_runtime_graph_requires_native_explorer_for_explore_capability() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["processes"].remove("explore")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_process_missing" and "process explore" in issue.message for issue in issues)


def test_runtime_graph_requires_host_process_not_legacy_runtime_name() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["teleop_avoid"]["processes"].remove("host")
    products["teleop_avoid"]["processes"].append("runtime")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_process_missing" and "process host" in issue.message for issue in issues)


def test_runtime_graph_rejects_legacy_service_lists() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["native_services"] = ["legacy.service"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_legacy_service_list" for issue in issues)


def test_runtime_graph_native_dds_endpoints_share_core_contract() -> None:
    graph = load_runtime_graph()
    required = set(graph.native_contract_topics)

    for endpoint in _native_endpoint_contracts(graph):
        endpoint_topics = set(endpoint["source_topics"]) | set(endpoint["exposed_topics"])
        assert required <= endpoint_topics
        assert endpoint["data_plane"] == "native_dds"
        assert endpoint["real_equivalent"] is True


def test_operator_motion_topics_are_native_endpoint_contract() -> None:
    graph = load_runtime_graph()
    operator_topics = {
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_sample,
        TOPICS.operator_motion_ack,
        TOPICS.operator_motion_status,
    }

    assert operator_topics <= set(graph.native_contract_topics)
    assert graph.topic_contracts[TOPICS.operator_motion_control]["frame"] == "none"
    assert graph.topic_contracts[TOPICS.operator_motion_control]["qos"] == "reliable_volatile_keep_last_32"
    assert graph.topic_contracts[TOPICS.operator_motion_sample]["frame"] == "body"
    assert graph.topic_contracts[TOPICS.operator_motion_sample]["qos"] == "best_effort_volatile_keep_last_1"
    assert graph.topic_contracts[TOPICS.operator_motion_ack]["frame"] == "none"
    assert graph.topic_contracts[TOPICS.operator_motion_ack]["qos"] == "reliable_transient_local_keep_last_64"
    assert graph.topic_contracts[TOPICS.operator_motion_ack]["consumers"] == ["operator_motion_adapter"]
    assert (
        graph.topic_contracts[TOPICS.operator_motion_ack]["semantics"]
        == "native_business_ack_for_claim_hold_release_only"
    )
    assert graph.topic_contracts[TOPICS.operator_motion_status]["frame"] == "map"
    assert graph.topic_contracts[TOPICS.operator_motion_status]["qos"] == "reliable_transient_local_keep_last_1"
    assert graph.topic_contracts[TOPICS.operator_motion_status]["consumers"] == []
    assert graph.topic_contracts[TOPICS.operator_motion_status]["external_diagnostics_subscribable"] is True
    assert (
        graph.topic_contracts[TOPICS.operator_motion_status]["semantics"]
        == "active_source_epoch_last_sample_admission_and_final_output_evidence"
    )

    for endpoint in _native_endpoint_contracts(graph):
        assert TOPICS.operator_motion_control in endpoint["source_topics"]
        assert TOPICS.operator_motion_sample in endpoint["source_topics"]
        assert TOPICS.operator_motion_ack in endpoint["exposed_topics"]
        assert TOPICS.operator_motion_status in endpoint["exposed_topics"]


def test_operator_motion_observability_consumers_are_truthful() -> None:
    graph = load_runtime_graph()
    ack = graph.topic_contracts[TOPICS.operator_motion_ack]
    status = graph.topic_contracts[TOPICS.operator_motion_status]
    client_cpp = (REPO_ROOT / "src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")

    assert ack["consumers"] == ["operator_motion_adapter"]
    assert {(binding["owner"], binding["direction"], binding["port"]) for binding in ack["port_bindings"]} == {
        ("native_nav_runtime", "out", "operator_motion_ack"),
        ("operator_motion_adapter", "in", "operator_motion_ack"),
    }
    assert "operator_motion_ack_reader = createReader" in client_cpp
    assert "lingtu::message::kOperatorMotionAck" in client_cpp

    assert status["consumers"] == []
    assert status["external_diagnostics_subscribable"] is True
    assert {(binding["owner"], binding["direction"], binding["port"]) for binding in status["port_bindings"]} == {
        ("native_nav_runtime", "out", "operator_motion_status")
    }


def test_exploration_dds_consumers_do_not_claim_a_gateway_reader() -> None:
    graph = load_runtime_graph()
    grid = graph.topic_contracts[TOPICS.exploration_grid]
    snapshot = graph.topic_contracts[TOPICS.exploration_snapshot]

    assert "gateway" not in grid["consumers"]
    assert "gateway" not in snapshot["consumers"]
    assert grid["consumers"] == ["TAREExplorerModule"]
    assert snapshot["consumers"] == ["native_explore_endpoint"]


def test_inspection_dds_topics_are_a_native_runtime_contract() -> None:
    graph = load_runtime_graph()

    assert set(INSPECTION_DDS_TOPICS) <= set(graph.native_contract_topics)
    assert "/nav/inspection/command" not in graph.native_contract_topics
    assert "/nav/inspection/ack" not in graph.native_contract_topics
    assert graph.topic_contracts[TOPICS.inspection_task_request] == {
        "role": "native_inspection_task_request",
        "frame": "map",
        "schema": "inspection_task_request",
        "producer": "persistent_cpp_navigation_client",
        "consumers": ["native_nav_runtime"],
        "qos": "reliable_volatile_keep_last_32",
        "semantics": "caller_task_id_and_retryable_request_id",
        "port_bindings": [
            {
                "owner": "persistent_cpp_navigation_client",
                "port": "inspection_task_request",
                "direction": "out",
                "boundary": "native",
            },
            {
                "owner": "native_nav_runtime",
                "port": "inspection_task_request",
                "direction": "in",
                "boundary": "endpoint",
            },
        ],
    }
    assert "persistent_cpp_navigation_client" in graph.topic_contracts[TOPICS.inspection_task_ack]["consumers"]
    assert graph.topic_contracts[TOPICS.inspection_status]["qos"] == ("reliable_transient_local_keep_last_1")


def test_native_ack_and_status_topics_do_not_claim_nonexistent_gateway_readers() -> None:
    graph = load_runtime_graph()

    for topic in (TOPICS.exploration_ack, TOPICS.inspection_task_ack):
        assert graph.topic_contracts[topic]["consumers"] == ["persistent_cpp_navigation_client"]

    assert graph.topic_contracts[TOPICS.nav_command_ack]["consumers"] == [
        "persistent_cpp_navigation_client",
        "native_explore_runtime",
    ]
    assert {
        "owner": "native_explore_runtime",
        "port": "navigation_command_ack",
        "direction": "in",
        "boundary": "endpoint",
    } in graph.topic_contracts[TOPICS.nav_command_ack]["port_bindings"]
    command_request = graph.topic_contracts[TOPICS.nav_command_request]
    assert command_request["producer"] == "native_navigation_command_client"
    assert {
        "owner": "native_explore_runtime",
        "port": "navigation_command_request",
        "direction": "out",
        "boundary": "endpoint",
    } in command_request["port_bindings"]

    inspection_status = graph.topic_contracts[TOPICS.inspection_status]
    assert inspection_status["consumers"] == []
    assert inspection_status["external_diagnostics_subscribable"] is True
    assert graph.topic_contracts[TOPICS.inspection_evidence_result]["consumers"] == ["native_nav_runtime"]


def test_native_endpoints_close_the_inspection_task_ack_status_chain() -> None:
    graph = load_runtime_graph()

    for endpoint in _native_endpoint_contracts(graph):
        assert TOPICS.inspection_task_request in endpoint["source_topics"]
        assert TOPICS.inspection_task_ack in endpoint["exposed_topics"]
        assert TOPICS.inspection_status in endpoint["exposed_topics"]
        assert TOPICS.inspection_evidence_request in endpoint["exposed_topics"]
        assert TOPICS.inspection_evidence_result in endpoint["source_topics"]
        assert endpoint["inspection_task_boundary"] == {
            "request": TOPICS.inspection_task_request,
            "ack": TOPICS.inspection_task_ack,
            "status": TOPICS.inspection_status,
            "client_completion": "business_ack_required",
            "request_identity_fields": ["task_id", "request_id"],
            "response_identity_fields": ["task_id", "request_id"],
        }
        assert endpoint["inspection_evidence_boundary"] == {
            "request": TOPICS.inspection_evidence_request,
            "result": TOPICS.inspection_evidence_result,
            "client_completion": "matching_persisted_result_required",
        }


def test_tare_goal_status_is_a_field_endpoint_lifecycle_contract() -> None:
    graph = load_runtime_graph()
    status = graph.topic_contracts[TOPICS.nav_goal_status]

    assert status["role"] == "native_navigation_goal_status"
    assert status["frame"] == "map"
    assert status["schema"] == "navigation_goal_status"
    assert status["producer"] == "native_nav_runtime"
    assert status["consumers"] == ["native_explore_runtime", "host_bus"]
    assert status["qos"] == "reliable_transient_local_keep_last_64"
    assert status["semantics"] == "request_correlated_goal_lifecycle"
    assert {
        (binding["owner"], binding["port"], binding["direction"], binding["boundary"])
        for binding in status["port_bindings"]
    } == {
        ("native_nav_runtime", "goal_status", "out", "endpoint"),
        ("native_explore_runtime", "goal_status", "in", "endpoint"),
        ("host_bus", "navigation_goal_status", "in", "native"),
    }
    assert TOPICS.nav_goal_status in _endpoint_contract(graph, "real")["exposed_topics"]
    for product_name, product_variant in (
        ("explore", "live"),
        ("explore", "map"),
        ("nav", None),
        ("inspection", None),
        ("tracking", None),
    ):
        contract = resolve_product_spec_contracts(
            product_name,
            graph.products[product_name],
            product_variant=product_variant,
        )
        assert TOPICS.nav_goal_status in contract.topics
    assert TOPICS.nav_goal_status not in graph.native_contract_topics


def test_directed_exploration_intent_reuses_the_existing_command_ack_boundary() -> None:
    graph = load_runtime_graph()
    command = graph.topic_contracts[TOPICS.exploration_command]
    ack = graph.topic_contracts[TOPICS.exploration_ack]

    assert command["qos"] == "reliable_volatile_keep_last_32"
    assert command["semantics"] == "exploration_lifecycle_or_directed_target_set_clear"
    assert ack["qos"] == "reliable_transient_local_keep_last_64"
    assert ack["semantics"] == "exploration_business_ack_with_intent_revision"

    directed_target = {
        "set_kind": 5,
        "clear_kind": 6,
        "request_fields": [
            "has_directed_target",
            "directed_target_x",
            "directed_target_y",
            "directed_target_ttl_s",
        ],
        "ack_field": "intent_revision",
        "policy": "soft_tare_candidate_preference",
        "grants_static_boundary_path_authorization": False,
    }
    for endpoint in _native_endpoint_contracts(graph):
        boundary = endpoint["exploration_command_boundary"]
        assert boundary["request"] == TOPICS.exploration_command
        assert boundary["ack"] == TOPICS.exploration_ack
        assert boundary["directed_target"] == directed_target


def test_inspection_product_requires_its_native_task_and_status_topics() -> None:
    graph = load_runtime_graph()
    inspection = graph.products["inspection"]
    contract = resolve_product_spec_contracts("inspection", inspection)

    assert set(INSPECTION_DDS_TOPICS) <= set(contract.topics)


def test_runtime_graph_rejects_missing_localization_health() -> None:
    graph = load_runtime_graph()
    topics = deepcopy(graph.topics)
    topics["topics"].pop(TOPICS.localization_health)
    broken = RuntimeGraph(
        root=graph.root,
        topics=topics,
        products=graph.products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "native_topic_missing" for issue in issues)
    assert any(issue.code == "product_required_topic_missing" for issue in issues)


def test_runtime_graph_rejects_incomplete_operator_mode_contract() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["teleop"].pop("native_control_mode")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_lifecycle_field_missing" and "native_control_mode" in issue.message for issue in issues
    )


def test_runtime_graph_rejects_field_product_topic_without_endpoint_provider() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["endpoints"]["contract"]["exposed_topics"].remove(TOPICS.exploration_snapshot)
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "field_product_topic_missing"
        and "explore" in issue.message
        and TOPICS.exploration_snapshot in issue.message
        for issue in issues
    )


def test_runtime_graph_rejects_map_and_slam_mode_conflict() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["requires_map"] = False
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_map_slam_conflict" and "nav" in issue.message for issue in issues)


def test_runtime_graph_rejects_map_free_exploration_without_live_route() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["live"]["contracts"] = ["lingtu.product.teleop_avoid.v1"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "explore_live_missing" for issue in issues)


def test_runtime_graph_rejects_static_planner_as_map_free_exploration_requirement() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["live"]["contracts"] = ["lingtu.product.nav.v1"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "explore_map_conflict" for issue in issues)


def test_runtime_graph_validates_non_default_explore_variant_contract() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["map"]["contracts"] = ["lingtu.product.missing.v1"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_contract_invalid"
        and issue.scope == "product:explore:variant:map"
        and "variant 'map'" in issue.message
        for issue in issues
    )


def test_runtime_graph_validates_non_default_explore_variant_lifecycle() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["map"]["slam_mode"] = "mapping"
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_map_slam_conflict"
        and issue.scope == "product:explore:variant:map"
        and "variant 'map'" in issue.message
        for issue in issues
    )


def test_runtime_graph_validates_non_default_explore_variant_processes() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["map"]["processes"] = [
        *products["explore"]["processes"],
        "unmapped_variant_process",
    ]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "field_product_process_unmapped"
        and issue.scope == "product:explore:variant:map"
        and "unmapped_variant_process" in issue.message
        for issue in issues
    )


def test_runtime_graph_checks_env_topic_closure_for_non_default_variant() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["endpoints"]["contract"]["exposed_topics"].remove(TOPICS.operator_motion_status)
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "field_product_topic_missing"
        and issue.scope == "product:explore:variant:map"
        and TOPICS.operator_motion_status in issue.message
        for issue in issues
    )


def test_runtime_graph_rejects_duplicate_session_defaults() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["tracking"]["default_for_session_mode"] = True
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_session_default_invalid" and "navigating" in issue.message for issue in issues)


@pytest.mark.parametrize("product", REAL_PRODUCTS)
def test_real_products_compile_against_runtime_graph(product: str) -> None:
    resolved = resolve_product_host_runtime(product, "real", robot="unitree/go2")
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
        product_variant=resolved.product_variant,
    )

    assert plan.product == product
    assert plan.env == "real"


def test_real_product_rejects_noncanonical_or_duplicate_driver() -> None:
    from lingtu.assembly.products import resolve_product_host_config

    graph = load_runtime_graph()
    broken = dict(
        resolve_product_host_config("nav", "real", robot="unitree/go2")
    )
    broken["hardware_control_boundary"] = "dds_endpoint_source"
    broken["enable_robot_driver"] = True

    issues = validate_product(
        "nav",
        product_spec=graph.products["nav"],
        implementation=resolve_env_implementation("real", graph=graph),
        config=broken,
        module_names=(),
        env_name="real",
    )
    codes = {issue.code for issue in issues}

    assert "real_product_driver_boundary_drift" in codes
    assert "real_product_duplicate_driver" in codes
    assert {issue.scope for issue in issues} == {"product:nav"}


def test_runtime_graph_renderers_emit_human_readable_contracts() -> None:
    mermaid = render_env_mermaid(
        "sim",
        env_config={"backend": "mujoco"},
    )
    markdown = render_product_markdown("nav")

    assert mermaid.startswith("flowchart LR")
    assert 'env["sim"]' in mermaid
    assert 'backend["mujoco"]' in mermaid
    assert TOPICS.raw_lidar_points in mermaid
    assert "# nav" in markdown
    assert f"`{TOPICS.odometry}`" in markdown
    assert "native localization" in markdown.lower()
    assert "## Forbidden Modules" in markdown
    assert "`SlamBridgeModule`" not in markdown


def test_env_renderer_rejects_missing_endpoint_contract() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    del envs["sim"]["backends"]["mujoco"]["endpoints"]["contract"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    with pytest.raises(ValueError, match=r"endpoints\.contract"):
        render_env_mermaid(
            "sim",
            graph=broken,
            env_config={"backend": "mujoco"},
        )
