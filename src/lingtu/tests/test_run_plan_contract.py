# ruff: noqa: S101

from __future__ import annotations

import json
from copy import deepcopy

import pytest

from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.profile_builder import compile_run_plan
from lingtu.control import ProductControl
from lingtu.run_plan import RunPlan, validate_run_plan_snapshot
from lingtu.run_plan_contract import RunPlanCompatibilityError
from runtime.graph import RuntimeGraph, load_runtime_graph


def test_product_control_resolves_product_inside_fixed_real_env() -> None:
    control = ProductControl(env="real", process_env={})

    plan = control.resolve("nav")

    assert isinstance(plan, RunPlan)
    assert plan.env == "real"
    assert plan.product == "nav"
    assert plan.fingerprint
    assert plan.processes
    payload = plan.as_dict()
    assert set(payload) == {"identity", "launch", "host", "checks"}
    assert payload["identity"]["schema"] == "lingtu.run_plan.v1"
    assert payload["identity"]["env"] == "real"
    assert payload["launch"]["session"]["product"] == "nav"
    assert payload["launch"]["session"]["slam_mode"] == "localization"
    assert set(payload["identity"]["compiled_against"]) >= {
        "schema_version",
        "runtime_graph_sha256",
        "service_catalog_sha256",
        "product_contract_catalog_sha256",
        "host_module_graph_sha256",
        "host_source_sha256",
        "release",
    }
    assert payload["checks"] == {
        "contracts": ["lingtu.product.nav.v1"],
        "critical_modules": list(plan.critical_modules),
    }
    assert "required_topics" not in json.dumps(payload, sort_keys=True)
    assert "required_capabilities" not in json.dumps(payload, sort_keys=True)
    plan.assert_compatible(environment={})
    assert "endpoint" not in payload
    assert "driver_backend" not in payload


def test_run_plan_fingerprint_authenticates_lifecycle_and_compiler_identity() -> None:
    plan = ProductControl(env="real", process_env={}).resolve("nav")

    lifecycle_tamper = plan.as_dict()
    lifecycle_tamper["launch"]["session"]["switch_policy"] = "hot_switch"
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        RunPlan.from_dict(lifecycle_tamper)

    compiler_tamper = plan.as_dict()
    compiler_tamper["identity"]["compiled_against"]["host_source_sha256"] = (
        "0" * 64
    )
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        RunPlan.from_dict(compiler_tamper)


def test_run_plan_output_is_deterministic(tmp_path) -> None:
    left = ProductControl(env="real", process_env={}).resolve("nav")
    right = ProductControl(env="real", process_env={}).resolve("nav")

    assert left.fingerprint == right.fingerprint
    assert left.as_dict() == right.as_dict()
    left_path = left.write(tmp_path / "left.json")
    right_path = right.write(tmp_path / "right.json")
    assert left_path.read_bytes() == right_path.read_bytes()


def test_run_plan_summary_is_compact_and_roundtrips() -> None:
    plan = ProductControl(env="real", process_env={}).resolve("nav")

    summary = plan.summary()

    assert summary["kind"] == "run_plan"
    assert summary["product"] == "nav"
    assert summary["env"] == "real"
    assert summary["run_plan_fingerprint"] == plan.fingerprint
    assert summary["processes"] == [process.name for process in plan.processes]
    assert summary["host_modules"] == list(plan.modules)
    assert "host_config" not in summary
    assert "native_process_environment" not in summary
    assert validate_run_plan_snapshot(plan) == {
        "ok": True,
        "blockers": [],
        "warnings": [],
    }


def test_run_plan_release_identity_uses_control_environment_and_fails_closed() -> None:
    native_sha256 = "a" * 64
    plan = ProductControl(
        env="real",
        process_env={
            "LINGTU_RELEASE_VERSION": "release-a",
            "LINGTU_NATIVE_RELEASE_SHA256": native_sha256,
        },
    ).resolve("nav")

    assert plan.compiled_against["release"]["version"] == "release-a"
    assert plan.compiled_against["release"]["native_sha256"] == native_sha256
    plan.assert_compatible(
        environment={
            "LINGTU_RELEASE_VERSION": "release-a",
            "LINGTU_NATIVE_RELEASE_SHA256": native_sha256,
        }
    )
    with pytest.raises(
        RunPlanCompatibilityError,
        match=r"re-run ProductControl\.switch",
    ):
        plan.assert_compatible(
            environment={
                "LINGTU_RELEASE_VERSION": "release-b",
                "LINGTU_NATIVE_RELEASE_SHA256": native_sha256,
            }
        )


def test_real_release_identity_rejects_missing_native_hash() -> None:
    with pytest.raises(ValueError, match="native"):
        ProductControl(
            env="real",
            process_env={"LINGTU_RELEASE_VERSION": "release-a"},
        ).resolve("nav")


def test_run_plan_rejects_current_product_lifecycle_drift(monkeypatch) -> None:
    import lingtu.run_plan_contract as contract_mod

    plan = ProductControl(env="real", process_env={}).resolve("nav")
    graph = load_runtime_graph()
    products = {name: dict(spec) for name, spec in graph.products.items()}
    products["nav"]["slam_mode"] = "mapping"
    changed = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )
    monkeypatch.setattr(contract_mod, "load_runtime_graph", lambda: changed)

    assert plan.equivalent(environment={}) is False
    with pytest.raises(
        RunPlanCompatibilityError,
        match=r"re-run ProductControl\.switch",
    ):
        plan.assert_compatible(environment={})


def test_explore_map_plan_recomputes_its_selected_variant_contract(
    monkeypatch,
) -> None:
    import lingtu.run_plan_contract as contract_mod

    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
        product_variant=resolved.product_variant,
        environment={},
    )

    plan.assert_compatible(environment={})

    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["map"]["slam_mode"] = "mapping"
    changed = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )
    monkeypatch.setattr(contract_mod, "load_runtime_graph", lambda: changed)

    assert plan.equivalent(environment={}) is False
    with pytest.raises(
        RunPlanCompatibilityError,
        match=r"re-run ProductControl\.switch",
    ):
        plan.assert_compatible(environment={})


def test_run_plan_rejects_mismatched_variant_identity() -> None:
    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
        product_variant=resolved.product_variant,
        environment={},
    )

    tampered = plan.as_dict()
    tampered["host"]["config"]["_product_variant"] = "live"
    with pytest.raises(ValueError, match="Host config variant"):
        RunPlan.from_dict(tampered)


def test_run_plan_rejects_unknown_top_level_interpretation_fields() -> None:
    payload = ProductControl(env="real", process_env={}).resolve("nav").as_dict()
    payload["future_interpreter"] = {}

    with pytest.raises(ValueError, match="unsupported fields"):
        RunPlan.from_dict(payload)


def test_run_plan_rejects_flat_v2_without_a_fallback() -> None:
    with pytest.raises(ValueError, match="top level"):
        RunPlan.from_dict(
            {
                "schema_version": "lingtu.run_plan.legacy",
                "product": "nav",
                "env": "real",
            }
        )
