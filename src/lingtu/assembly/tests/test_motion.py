from __future__ import annotations

from copy import deepcopy

from lingtu.assembly.compiler import blueprint_from_run_plan, compile_run_plan
from lingtu.assembly.native_nav import compile_native_nav_config
from lingtu.assembly.products import resolve_product_host_runtime
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph import RuntimeGraph, load_runtime_graph, validate_runtime_graph
from runtime.runtime_interface import TOPICS

OPERATOR_MOTION_TOPICS = frozenset(
    {
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_sample,
        TOPICS.operator_motion_ack,
        TOPICS.operator_motion_status,
    }
)
OPERATOR_MOTION_CAPABILITIES = frozenset(
    {
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
    }
)
OPERATOR_MOTION_CRITICAL_MODULES = frozenset({"nav.commands", "GatewayModule"})
PRODUCTS_REQUIRING_OPERATOR_MOTION = frozenset(
    {
        "inspection",
        "map",
        "nav",
        "teleop",
        "teleop_avoid",
        "tracking",
    }
)


def _with_products(graph: RuntimeGraph, products: dict) -> RuntimeGraph:
    return RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )


def _contracts(name: str, product: dict):
    return resolve_product_spec_contracts(name, product)


def _real_modules(product: str) -> tuple[str, ...]:
    resolved = resolve_product_host_runtime(product, "real", robot="unitree/go2")
    return compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
        product_variant=resolved.product_variant,
    ).modules


def test_operator_motion_python_module_is_not_mounted_by_field_products() -> None:
    for product in ("teleop_avoid", "teleop", "map", "nav"):
        assert "operator.motion" not in _real_modules(product)


def test_teleop_avoid_compilation_uses_native_operator_motion_without_python_authority() -> None:
    resolved = resolve_product_host_runtime(
        "teleop_avoid",
        "real",
        robot="unitree/go2",
    )

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
    )
    blueprint = blueprint_from_run_plan(plan)

    assert plan.critical_modules == (
        "host.bus",
        "SlamAdapterModule",
        "nav.commands",
        "GatewayModule",
    )
    assert blueprint.required_module_names == plan.critical_modules
    assert "operator.motion" not in plan.modules
    assert "maps.service" not in plan.modules
    assert "nav.commands" in plan.modules
    assert plan.process_control == "systemd"
    assert plan.has_process("maps")
    assert plan.has_process("nav")
    assert plan.has_process("driver")
    assert OPERATOR_MOTION_TOPICS <= set(plan.required_topics)
    assert OPERATOR_MOTION_CAPABILITIES <= set(plan.required_capabilities)


def test_teleop_avoid_compiles_native_motion_settings() -> None:
    product = load_runtime_graph().products["teleop_avoid"]
    contract = _contracts("teleop_avoid", product)
    compiled = compile_native_nav_config("teleop_avoid", product)

    expected_native = {
        "control_mode": "teleop_avoid",
        "global_planner": "octoplanner3d",
        "local_planner": "cmu",
        "publish_cmd_vel": True,
        "check_obstacle": True,
        "use_traversability_cost": False,
        "allow_teleop_takeover": False,
        "teleop_local_planner": True,
    }
    assert {key: compiled.native_nav[key] for key in expected_native} == expected_native
    assert compiled.parameters["teleop_planner_horizon_m"] == 3.5
    assert compiled.parameters["teleop_planner_max_deviation_deg"] == 90.0
    assert compiled.native_nav["smoothing"] is False
    assert "traversability" not in product["processes"]
    assert OPERATOR_MOTION_CAPABILITIES <= set(contract.capabilities)
    assert OPERATOR_MOTION_TOPICS <= set(contract.topics)
    assert "required_capabilities" not in product
    assert "required_topics" not in product


def test_only_raw_operator_motion_uses_native_velocity_smoothing() -> None:
    teleop = compile_run_plan("teleop", "real", robot="unitree/go2")
    assisted = compile_run_plan("teleop_avoid", "real", robot="unitree/go2")

    assert teleop.native_process_environment["LINGTU_NAV_SMOOTHER_ENABLED"] == "1"
    assert assisted.native_process_environment["LINGTU_NAV_SMOOTHER_ENABLED"] == "0"


def test_nav_uses_practical_terminal_tolerances() -> None:
    plan = compile_run_plan("nav", "sim", robot="doso/thunder_v4")

    assert plan.native_process_environment["LINGTU_NAV_GOAL_REACHED_M"] == "0.35"
    assert (
        plan.native_process_environment["LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M"]
        == "0.2"
    )


def test_native_nav_compiles_follower_and_recovery_settings() -> None:
    product = deepcopy(load_runtime_graph().products["nav"])
    product["native_nav"].update(
        {
            "path_follower_max_accel_mps2": 0.30,
            "path_follower_max_yaw_rate_rad_s": 0.70,
            "path_follower_heading_align_enter_rad": 0.80,
            "path_follower_heading_align_exit_rad": 0.30,
            "scan_follower": {
                "time_forward_s": 0.65,
                "heading_error_rad": 0.70,
                "position_gain": 0.90,
                "yaw_gain": 1.40,
                "max_vx_mps": 0.68,
                "max_vy_mps": 0.32,
                "max_yaw_rate_rad_s": 0.95,
            },
            "recovery": {
                "behavior_order": ["rotate", "translate"],
                "blocked_interval_s": 1.25,
                "rotation_timeout_s": 2.75,
                "translation_timeout_s": 1.75,
                "max_attempts": 4,
                "translation_speed_mps": 0.12,
                "rotation_rate_rad_s": 0.40,
                "min_rotation_rad": 0.25,
                "max_rotation_rad": 1.10,
                "rotation_candidate_step_rad": 0.15,
                "rotation_sample_step_rad": 0.05,
            },
        }
    )

    compiled = compile_native_nav_config("nav", product)

    assert compiled.parameters["path_follower_max_accel_mps2"] == 0.30
    assert compiled.parameters["path_follower_max_yaw_rate_rad_s"] == 0.70
    assert compiled.parameters["path_follower_heading_align_enter_rad"] == 0.80
    assert compiled.parameters["path_follower_heading_align_exit_rad"] == 0.30
    assert compiled.parameters["scan_time_forward_s"] == 0.65
    assert compiled.parameters["scan_max_vy_mps"] == 0.32
    assert compiled.native_nav["recovery"]["behavior_order"] == (
        "rotate",
        "translate",
    )
    assert compiled.environment["LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2"] == "0.3"
    assert compiled.environment["LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_RATE_RAD_S"] == "0.7"
    assert compiled.environment["LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_ENTER_RAD"] == "0.8"
    assert compiled.environment["LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_EXIT_RAD"] == "0.3"
    assert compiled.environment["LINGTU_NAV_SCAN_TIME_FORWARD_S"] == "0.65"
    assert compiled.environment["LINGTU_NAV_SCAN_HEADING_ERROR_RAD"] == "0.7"
    assert compiled.environment["LINGTU_NAV_SCAN_POSITION_GAIN"] == "0.9"
    assert compiled.environment["LINGTU_NAV_SCAN_YAW_GAIN"] == "1.4"
    assert compiled.environment["LINGTU_NAV_SCAN_MAX_VX_MPS"] == "0.68"
    assert compiled.environment["LINGTU_NAV_SCAN_MAX_VY_MPS"] == "0.32"
    assert compiled.environment["LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S"] == "0.95"
    assert "LINGTU_NAV_SCAN_FINISH_DISTANCE_M" not in compiled.environment
    assert compiled.environment["LINGTU_NAV_RECOVERY_ORDER"] == "rotate,translate"
    assert compiled.environment["LINGTU_NAV_RECOVERY_BLOCKED_INTERVAL_S"] == "1.25"
    assert compiled.environment["LINGTU_NAV_RECOVERY_ROTATION_TIMEOUT_S"] == "2.75"
    assert compiled.environment["LINGTU_NAV_RECOVERY_TRANSLATION_TIMEOUT_S"] == "1.75"
    assert compiled.environment["LINGTU_NAV_RECOVERY_MAX_ATTEMPTS"] == "4"
    assert compiled.environment["LINGTU_NAV_RECOVERY_TRANSLATION_SPEED_MPS"] == "0.12"
    assert compiled.environment["LINGTU_NAV_RECOVERY_ROTATION_RATE_RAD_S"] == "0.4"
    assert compiled.environment["LINGTU_NAV_RECOVERY_MIN_ROTATION_RAD"] == "0.25"
    assert compiled.environment["LINGTU_NAV_RECOVERY_MAX_ROTATION_RAD"] == "1.1"
    assert compiled.environment["LINGTU_NAV_RECOVERY_ROTATION_CANDIDATE_STEP_RAD"] == "0.15"
    assert compiled.environment["LINGTU_NAV_RECOVERY_ROTATION_SAMPLE_STEP_RAD"] == "0.05"


def test_operator_motion_requirement_is_derived_from_native_policy_for_every_product() -> None:
    products = load_runtime_graph().products

    assert set(products) == PRODUCTS_REQUIRING_OPERATOR_MOTION | {"explore"}
    for name, product in products.items():
        native_nav = product.get("native_nav") or {}
        control_mode = product.get("native_control_mode")
        derived_requirement = control_mode in {"teleop", "teleop_avoid"} or (
            control_mode == "autonomy" and native_nav.get("allow_teleop_takeover") is True
        )

        assert derived_requirement is (name in PRODUCTS_REQUIRING_OPERATOR_MOTION)
        contract = _contracts(name, product)
        topics = set(contract.topics)
        capabilities = set(contract.capabilities)
        if derived_requirement:
            assert OPERATOR_MOTION_TOPICS <= topics
            assert OPERATOR_MOTION_CAPABILITIES <= capabilities
            assert OPERATOR_MOTION_CRITICAL_MODULES <= set(product.get("critical_modules", ()))
        else:
            assert OPERATOR_MOTION_TOPICS.isdisjoint(topics)
            assert OPERATOR_MOTION_CAPABILITIES.isdisjoint(capabilities)


def test_compiled_operator_motion_products_make_host_adapters_startup_critical() -> None:
    for profile in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        resolved = resolve_product_host_runtime(profile, "real", robot="unitree/go2")
        manifest = compile_run_plan(
            resolved.product,
            resolved.env,
            robot="unitree/go2",
        )
        blueprint = blueprint_from_run_plan(manifest)

        assert OPERATOR_MOTION_CRITICAL_MODULES <= set(manifest.modules)
        assert OPERATOR_MOTION_CRITICAL_MODULES <= set(manifest.critical_modules)
        assert blueprint.required_module_names == manifest.critical_modules

    explore = load_runtime_graph().products["explore"]
    assert "operator.motion" not in explore.get("critical_modules", ())


def test_explore_map_variant_declares_operator_motion_takeover_contract() -> None:
    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        robot="unitree/go2",
        product_variant="map",
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
        product_variant=resolved.product_variant,
    )

    assert plan.product == "explore"
    assert plan.product_variant == "map"
    assert OPERATOR_MOTION_TOPICS <= set(plan.required_topics)
    assert OPERATOR_MOTION_CAPABILITIES <= set(plan.required_capabilities)
    assert OPERATOR_MOTION_CRITICAL_MODULES <= set(plan.critical_modules)


def test_validator_rejects_contract_without_operator_motion_topics_for_every_required_product() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        products[product_name]["contracts"] = ["lingtu.product.explore.v1"]

    issues = validate_runtime_graph(_with_products(graph, products))

    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        matching = [
            issue
            for issue in issues
            if issue.code == "product_operator_motion_boundary_incomplete" and issue.scope == f"product:{product_name}"
        ]
        assert len(matching) == 1
        assert all(topic in matching[0].message for topic in OPERATOR_MOTION_TOPICS)


def test_validator_rejects_contract_without_operator_motion_capabilities_for_every_required_product() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        products[product_name]["contracts"] = ["lingtu.product.explore.v1"]

    issues = validate_runtime_graph(_with_products(graph, products))

    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        matching = [
            issue
            for issue in issues
            if issue.code == "product_operator_motion_capability_missing" and issue.scope == f"product:{product_name}"
        ]
        assert len(matching) == 1
        assert all(capability in matching[0].message for capability in OPERATOR_MOTION_CAPABILITIES)


def test_validator_rejects_missing_critical_host_adapters_for_every_required_product() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        products[product_name]["critical_modules"] = []

    issues = validate_runtime_graph(_with_products(graph, products))

    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        matching = [
            issue
            for issue in issues
            if issue.code == "product_operator_motion_critical_adapter_missing"
            and issue.scope == f"product:{product_name}"
        ]
        assert len(matching) == 1
        assert all(module in matching[0].message for module in OPERATOR_MOTION_CRITICAL_MODULES)


def test_native_policy_bootstraps_operator_motion_requirement() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    # explore has variants; modify the variant-level native_nav to
    # trigger operator-motion requirements on a variant that does not
    # already enable teleop takeover.
    products["explore"]["variants"]["live"]["native_nav"]["allow_teleop_takeover"] = True

    issues = validate_runtime_graph(_with_products(graph, products))

    codes = {issue.code for issue in issues if issue.scope == "product:explore:variant:live"}
    assert "product_operator_motion_boundary_incomplete" in codes
    assert "product_operator_motion_capability_missing" in codes


def test_product_contracts_use_typed_operator_motion_topics_without_yaml_chains() -> None:
    products = load_runtime_graph().products

    for name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        contract = _contracts(name, products[name])
        assert OPERATOR_MOTION_TOPICS <= set(contract.topics)
        assert "target_chain" not in products[name]
