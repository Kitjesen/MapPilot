"""Compile one selected simulation preset into a RunPlan."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from lingtu.run_plan import SIMULATION_SCHEMA, repository_relative_path


def compile_simulation_snapshot(
    *,
    env: str,
    process_control: str,
    backend: str,
    robot: str,
    implementation: Mapping[str, Any],
    repository_root: Path,
    product: str | None = None,
    viewer: bool = False,
) -> dict[str, Any]:
    """Resolve the Product's simulation preset exactly once."""

    if env != "sim":
        if "presets" in implementation:
            raise ValueError("presets is only valid for env=sim")
        return {}

    presets = implementation.get("presets")
    if not isinstance(presets, Mapping):
        if process_control == "subprocess" and backend == "mujoco":
            raise ValueError("subprocess MuJoCo requires a simulation preset")
        return {}
    robot_presets = presets.get(robot)
    if not isinstance(robot_presets, Mapping):
        raise ValueError(
            f"sim backend {backend!r} has no presets for Robot {robot!r}"
        )
    raw_source = robot_presets.get(product) if product is not None else None
    if raw_source is None:
        raw_source = robot_presets.get("default")
    if raw_source is None:
        raise ValueError(
            f"sim backend {backend!r} has no preset for Robot {robot!r} "
            f"and Product {product!r}"
        )

    source = repository_relative_path(
        raw_source,
        field=f"presets[{robot}]",
    )
    root = Path(repository_root).resolve()
    source_path = (root / source).resolve()
    try:
        source_path.relative_to(root)
    except ValueError as exc:
        raise ValueError("simulation preset escapes the repository root") from exc
    if not source_path.is_file():
        raise ValueError(f"simulation preset does not exist: {source}")

    from sim.catalog import CatalogResolver

    resolved = CatalogResolver.from_repository(root).resolve(source_path)
    session = dict(resolved.session)
    if viewer:
        runtime = dict(session["runtime"])
        runtime["mode"] = "preview"
        session["runtime"] = runtime
    return {
        "schema": SIMULATION_SCHEMA,
        "session_source": source,
        "session": session,
        "physics_plan": resolved.physics_plan,
        "visual_plan": resolved.visual_plan,
        "sensor_plan": resolved.sensor_plan,
        "control_plan": resolved.control_plan,
        "transport_intent": resolved.transport_intent,
        "scenario_plan": resolved.scenario_plan,
    }


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, set | frozenset):
        return tuple(sorted(str(item) for item in value))
    if isinstance(value, list | tuple):
        return tuple(str(item) for item in value)
    return ()


__all__ = ["compile_simulation_snapshot"]
