"""Shared manifest helpers for simulation acceptance tools."""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path, PurePosixPath
from typing import Any


def resolve_artifact(root: Path, value: Any, *, field: str) -> Path:
    """Resolve one safe repository-relative artifact and require it to exist."""

    text = str(value or "").strip()
    if not text or "\\" in text:
        raise ValueError(f"{field} must be a repository-relative POSIX path")
    relative = PurePosixPath(text)
    if (
        relative.is_absolute()
        or relative.as_posix() != text
        or any(part in {"", ".", ".."} or ":" in part for part in relative.parts)
    ):
        raise ValueError(f"{field} must be a safe repository-relative path")
    resolved_root = root.resolve()
    path = (resolved_root / Path(*relative.parts)).resolve()
    try:
        path.relative_to(resolved_root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes its root: {text}") from exc
    if not path.is_file():
        raise ValueError(f"{field} is missing: {path}")
    return path


def load_manifest(path: Path, *, root: Path) -> dict[str, Any]:
    """Load and merge one manifest inheritance chain."""

    return _load_manifest(path.resolve(), root=root.resolve(), seen=set())


def validate_runner_plan(
    root: Path,
    run_plan_path: Path,
    manifest_path: Path,
    *,
    expected_products: tuple[str, ...],
):
    """Validate a runner's explicit manifest against one simulation RunPlan."""

    from lingtu.run_plan import RunPlan

    plan = RunPlan.load(run_plan_path.expanduser().resolve())
    if plan.env != "sim":
        raise ValueError("acceptance runner requires a sim RunPlan")
    if plan.product not in expected_products:
        raise ValueError(
            f"acceptance runner does not support RunPlan Product {plan.product!r}"
        )
    manifest = manifest_path.expanduser().resolve()
    payload = load_manifest(manifest, root=root)
    contract = payload.get("product_contract")
    manifest_product = (
        str(contract.get("product") or "").strip()
        if isinstance(contract, Mapping)
        else ""
    )
    if manifest_product != plan.product:
        raise ValueError(
            "acceptance manifest Product does not match the RunPlan: "
            f"plan={plan.product!r}, manifest={manifest_product!r}"
        )
    return plan


def _load_manifest(
    path: Path,
    *,
    root: Path,
    seen: set[Path],
) -> dict[str, Any]:
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"acceptance manifest escapes the repository: {path}") from exc
    if path in seen:
        raise ValueError(f"acceptance manifest inheritance cycle: {path}")
    if not path.is_file():
        raise ValueError(f"acceptance manifest is missing: {path}")
    visited = {*seen, path}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid acceptance manifest: {path}") from exc
    if not isinstance(payload, Mapping):
        raise ValueError(f"acceptance manifest must be an object: {path}")
    child = dict(payload)
    parent_value = str(child.pop("extends", "") or "").strip()
    if not parent_value:
        return child
    parent = resolve_artifact(path.parent, parent_value, field="manifest extends")
    try:
        parent.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"acceptance manifest extends outside repository: {parent}") from exc
    base = _load_manifest(parent, root=root, seen=visited)
    return _merge(base, child)


def _merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        existing = result.get(key)
        if isinstance(existing, Mapping) and isinstance(value, Mapping):
            result[key] = _merge(existing, value)
        else:
            result[key] = value
    return result


__all__ = [
    "load_manifest",
    "resolve_artifact",
    "validate_runner_plan",
]
