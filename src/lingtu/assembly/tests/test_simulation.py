from __future__ import annotations

import ast
import json
from pathlib import Path
from typing import Any

import pytest
from sim.catalog import CatalogResolver

from lingtu.assembly.compiler import compile_run_plan
from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.simulation import compile_simulation_snapshot
from lingtu.run_plan import RunPlan

REPO_ROOT = Path(__file__).resolve().parents[4]
SESSION_SOURCE = "sim/scenarios/catalog/thunder_omni_contract/session.yaml"
SCENARIO_SESSION_SOURCE = "sim/scenarios/catalog/open_field_pedestrian_crossing/session.yaml"
SIMULATION_SCHEMA = "lingtu.run_plan.simulation.v1"
REAL_ROBOT = "unitree/go2"
SIM_ROBOT = "doso/thunder_v4"


def test_assembly_does_not_reauthor_catalog_generation_fields() -> None:
    assembly_source = REPO_ROOT / "src" / "lingtu" / "assembly" / "simulation.py"
    tree = ast.parse(assembly_source.read_text(encoding="utf-8"))
    string_literals = {
        node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }

    assert "model_generation" not in string_literals
    assert "reset_generation" not in string_literals


def _catalog_snapshot(
    session_source: str = SESSION_SOURCE,
) -> dict[str, Any]:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(REPO_ROOT / session_source)
    return {
        "schema": SIMULATION_SCHEMA,
        "session_source": session_source,
        "session": resolved.session,
        "physics_plan": dict(resolved.physics_plan),
        "visual_plan": resolved.visual_plan,
        "sensor_plan": resolved.sensor_plan,
        "control_plan": resolved.control_plan,
        "transport_intent": resolved.transport_intent,
        "scenario_plan": resolved.scenario_plan,
    }


def _compiled_plan(product: str, env: str) -> RunPlan:
    env_config = {"backend": "mujoco"} if env == "sim" else None
    robot = SIM_ROBOT if env == "sim" else REAL_ROBOT
    resolved = resolve_product_host_runtime(
        product,
        env,
        robot=robot,
        env_config=env_config,
    )
    return compile_run_plan(
        resolved.product,
        resolved.env,
        robot=robot,
        env_config=env_config,
    )


def _recreate(
    base: RunPlan,
    *,
    simulation: dict[str, Any],
    env: str | None = None,
    process_control: str | None = None,
    host_config: dict[str, Any] | None = None,
) -> RunPlan:
    return RunPlan.create(
        product=base.product,
        product_variant=base.product_variant,
        env=env or base.env,
        robot=base.robot,
        process_control=process_control or base.process_control,
        modules=base.modules,
        processes=base.processes,
        available_processes=base.available_processes,
        support_processes=base.support_processes,
        stop_before_start=base.stop_before_start,
        contracts=base.contracts,
        critical_modules=base.critical_modules,
        route_contract=base.route_contract,
        host_config=host_config or base.host_config,
        lifecycle=base.lifecycle,
        native_process_environment=base.native_process_environment,
        parameters=base.parameters,
        simulation=simulation,
    )


def _snapshot_plan() -> RunPlan:
    return _recreate(_compiled_plan("nav", "sim"), simulation=_catalog_snapshot())


def _without_private_backend(plan: RunPlan) -> dict[str, Any]:
    host_config = plan.host_config
    host_config.pop("_env_backend", None)
    return host_config


def test_actual_catalog_snapshot_roundtrips_through_run_plan() -> None:
    plan = _snapshot_plan()

    restored = RunPlan.from_dict(plan.as_dict())

    assert restored == plan
    assert restored.simulation == _catalog_snapshot()
    assert restored.as_dict()["launch"]["simulation"] == plan.simulation


def test_assembly_compiles_complete_catalog_bundle_without_runtime_generation() -> None:
    snapshot = compile_simulation_snapshot(
        env="sim",
        robot=SIM_ROBOT,
        process_control="subprocess",
        backend="mujoco",
        implementation={
            "presets": {
                SIM_ROBOT: {"default": SESSION_SOURCE},
            }
        },
        repository_root=REPO_ROOT,
    )

    assert snapshot == _catalog_snapshot()
    session_id = snapshot["session"]["session_id"]
    for field in ("physics_plan", "visual_plan", "sensor_plan", "control_plan", "transport_intent"):
        assert snapshot[field]["session_id"] == session_id
    assert "model_generation" not in snapshot["physics_plan"]
    assert "reset_generation" not in snapshot["physics_plan"]
    assert snapshot["scenario_plan"] is None


def test_simulation_viewer_changes_only_the_runtime_mode() -> None:
    snapshot = compile_simulation_snapshot(
        env="sim",
        robot=SIM_ROBOT,
        process_control="subprocess",
        backend="mujoco",
        implementation={
            "presets": {
                SIM_ROBOT: {"default": SESSION_SOURCE},
            }
        },
        repository_root=REPO_ROOT,
        viewer=True,
    )

    expected = _catalog_snapshot()
    expected["session"]["runtime"]["mode"] = "preview"
    assert snapshot == expected


def test_simulation_preset_is_selected_by_robot_then_product() -> None:
    snapshot = compile_simulation_snapshot(
        env="sim",
        robot=SIM_ROBOT,
        process_control="subprocess",
        backend="mujoco",
        implementation={
            "presets": {
                "unitree/go2": {"default": SCENARIO_SESSION_SOURCE},
                SIM_ROBOT: {
                    "default": SCENARIO_SESSION_SOURCE,
                    "teleop_avoid": SESSION_SOURCE,
                },
            }
        },
        repository_root=REPO_ROOT,
        product="teleop_avoid",
    )

    assert snapshot["session_source"] == SESSION_SOURCE


def test_simulation_reports_missing_robot_preset() -> None:
    with pytest.raises(ValueError, match=r"simulation preset|unitree/go2"):
        compile_simulation_snapshot(
            env="sim",
            robot="unitree/go2",
            process_control="subprocess",
            backend="mujoco",
            implementation={
                "presets": {
                    SIM_ROBOT: {"default": SESSION_SOURCE},
                }
            },
            repository_root=REPO_ROOT,
        )


def test_scenario_bundle_generation_identity_roundtrips() -> None:
    snapshot = _catalog_snapshot(SCENARIO_SESSION_SOURCE)
    plan = _recreate(_compiled_plan("nav", "sim"), simulation=snapshot)

    restored = RunPlan.from_dict(plan.as_dict())

    assert restored.simulation == snapshot
    scenario_plan = restored.simulation["scenario_plan"]
    assert scenario_plan["model_generation"] == 0
    assert scenario_plan["reset_generation"] == 0


def test_run_plan_v3_launch_requires_simulation_field() -> None:
    payload = _compiled_plan("nav", "real").as_dict()
    payload["launch"].pop("simulation")

    with pytest.raises(ValueError, match="simulation"):
        RunPlan.from_dict(payload)


def test_real_run_plan_rejects_nonempty_simulation_snapshot() -> None:
    with pytest.raises(ValueError, match="env=sim"):
        _recreate(_compiled_plan("nav", "real"), simulation=_catalog_snapshot())


def test_formal_mujoco_subprocess_requires_simulation_snapshot() -> None:
    base = _compiled_plan("nav", "sim")

    with pytest.raises(ValueError, match="simulation"):
        _recreate(base, process_control="subprocess", simulation={})


def test_sim_subprocess_requires_snapshot_without_private_host_backend() -> None:
    base = _compiled_plan("nav", "sim")

    with pytest.raises(ValueError, match="launch.simulation"):
        _recreate(
            base,
            process_control="subprocess",
            simulation={},
            host_config=_without_private_backend(base),
        )


def test_deserialized_sim_subprocess_cannot_bypass_snapshot_with_host_tamper() -> None:
    base = _compiled_plan("nav", "sim")
    payload = base.as_dict()
    payload["launch"]["controller"] = "subprocess"
    payload["launch"]["simulation"] = {}
    payload["host"]["config"].pop("_env_backend", None)
    with pytest.raises(ValueError, match="launch.simulation"):
        RunPlan.from_dict(payload)


def test_simulation_snapshot_rejects_unknown_fields_and_unsafe_source() -> None:
    base = _compiled_plan("nav", "sim")
    snapshot = _catalog_snapshot()
    snapshot["extra"] = True
    with pytest.raises(ValueError, match="unsupported fields"):
        _recreate(base, simulation=snapshot)

    snapshot = _catalog_snapshot()
    snapshot["session_source"] = "../session.yaml"
    with pytest.raises(ValueError, match="session_source"):
        _recreate(base, simulation=snapshot)


def test_simulation_snapshot_rejects_session_id_mismatch() -> None:
    snapshot = _catalog_snapshot()
    snapshot["physics_plan"]["session_id"] = "other"

    with pytest.raises(ValueError, match="session_id"):
        _recreate(_compiled_plan("nav", "sim"), simulation=snapshot)


def test_session_change_is_preserved_by_run_plan_parser() -> None:
    plan = _snapshot_plan()
    payload = plan.as_dict()
    payload["launch"]["simulation"]["session"]["seed"] += 1

    changed = RunPlan.from_dict(payload)
    assert changed.simulation["session"]["seed"] == plan.simulation["session"]["seed"] + 1


def test_physics_plan_schema_excludes_runtime_generation_fields() -> None:
    schema = json.loads((REPO_ROOT / "schemas/simulation/physics-plan.v1.json").read_text(encoding="utf-8"))

    assert "model_generation" not in schema["required"]
    assert "reset_generation" not in schema["required"]
    assert "model_generation" not in schema["properties"]
    assert "reset_generation" not in schema["properties"]


@pytest.mark.parametrize(
    ("field", "path", "value"),
    (
        ("visual_plan", ("coordinate_system", "position_scale"), 101.0),
        ("sensor_plan", ("backends", "physics"), "other"),
        (
            "control_plan",
            ("stale_stop_authority", "safe_stop_on_stale"),
            False,
        ),
        (
            "transport_intent",
            ("allocation_boundary", "runtime_values_external"),
            False,
        ),
    ),
)
def test_complete_bundle_change_is_preserved_by_run_plan_parser(
    field: str,
    path: tuple[str, ...],
    value: object,
) -> None:
    payload = _snapshot_plan().as_dict()
    target = payload["launch"]["simulation"][field]
    for component in path[:-1]:
        target = target[component]
    target[path[-1]] = value

    changed = RunPlan.from_dict(payload)
    target = changed.simulation[field]
    for component in path:
        target = target[component]
    assert target == value
