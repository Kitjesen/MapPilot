"""Read-only RunPlan preview for Gateway clients."""

from __future__ import annotations

import os
import shlex
import time
from collections.abc import Mapping
from typing import Any

from runtime.profiles.product_lifecycle import ProductName, product_name

RUNTIME_SWITCH_PLAN_SCHEMA_VERSION = "lingtu.runtime_switch_plan.v1"
_REQUEST_FIELDS = (
    "current_product",
    "target_product",
    "map_name",
    "relocalize",
    "initial_pose",
)


def _clean_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _request_mapping(request: Any) -> dict[str, Any]:
    if request is None:
        return {}
    if isinstance(request, Mapping):
        raw = dict(request)
    elif hasattr(request, "model_dump"):
        raw = dict(request.model_dump())
    else:
        raw = {key: getattr(request, key) for key in _REQUEST_FIELDS if hasattr(request, key)}
    unsupported = sorted(set(raw).difference(_REQUEST_FIELDS))
    if unsupported:
        raise ValueError("unsupported Product switch field(s): " + ", ".join(unsupported))
    return raw


def _canonical_product(value: Any) -> ProductName:
    product = _clean_text(value)
    if product is None:
        raise ValueError("target_product is required")
    return product_name(product)


def _target_variant(product: ProductName, map_name: str | None) -> str | None:
    """Use the same request rule that ProductControl executes."""

    from lingtu.product_switch import SwitchRequest

    return SwitchRequest(target_product=product, map_name=map_name).product_variant


def build_operator_command(request: Mapping[str, Any], *, env: str) -> str:
    """Return the exact external ProductControl command for a validated request."""

    command = [
        "python",
        "-m",
        "lingtu.control",
        "switch",
        _canonical_product(request.get("target_product")),
        "--env",
        env,
    ]
    current_product = _clean_text(request.get("current_product"))
    if current_product is not None:
        command.extend(["--current", product_name(current_product)])
    map_name = _clean_text(request.get("map_name"))
    if map_name is not None:
        command.extend(["--map", map_name])
    command.append("--relocalize" if bool(request.get("relocalize", True)) else "--no-relocalize")
    initial_pose = request.get("initial_pose")
    if initial_pose is not None:
        if not isinstance(initial_pose, list | tuple) or len(initial_pose) != 3:
            raise ValueError("initial_pose must be [x, y, yaw]")
        command.extend(["--initial-pose", *(str(float(item)) for item in initial_pose)])
    return shlex.join(command)


def _run_plan_payload(plan: Any) -> dict[str, Any]:
    as_dict = getattr(plan, "as_dict", None)
    if not callable(as_dict):
        raise TypeError("RunPlan must provide as_dict()")
    payload = as_dict()
    if not isinstance(payload, Mapping):
        raise TypeError("RunPlan.as_dict() must return a mapping")
    return dict(payload)


def _run_plan_identity(payload: Mapping[str, Any]) -> dict[str, Any]:
    identity = payload.get("identity")
    if not isinstance(identity, Mapping):
        raise ValueError("RunPlan identity is missing")
    return dict(identity)


def _active_run_plan(gw: Any | None) -> Any:
    plan = getattr(gw, "_compiled_run_plan", None) if gw is not None else None
    if plan is None:
        raise RuntimeError("run_plan_missing")
    return plan


def _normalize_initial_pose(raw: Mapping[str, Any]) -> tuple[float, float, float] | None:
    initial_pose = raw.get("initial_pose")
    if initial_pose is None:
        return None
    if not isinstance(initial_pose, list | tuple) or len(initial_pose) != 3:
        raise ValueError("initial_pose must be [x, y, yaw]")
    return tuple(float(item) for item in initial_pose)


def _preview_control(env: str) -> Any:
    """Return a side-effect-free ProductControl bound to the active Env."""

    from lingtu.control import ProductControl

    env_backend = _clean_text(os.environ.get("LINGTU_ENV_BACKEND"))
    env_config = {"backend": env_backend} if env == "sim" and env_backend is not None else None
    return ProductControl(
        env=env,
        env_config=env_config,
        process_env=os.environ,
    )


def _changed_fields(current: Mapping[str, Any], target: Mapping[str, Any]) -> list[str]:
    return sorted(
        key for key in set(current).union(target) if key != "schema_version" and current.get(key) != target.get(key)
    )


def _validation(ok: bool, message: str | None = None) -> dict[str, Any]:
    return {
        "ok": ok,
        "blockers": [] if ok or not message else [message],
        "warnings": [],
    }


def build_runtime_switch_plan(
    request: Any = None,
    *,
    gw: Any | None = None,
    control: Any | None = None,
) -> dict[str, Any]:
    """Compile a side-effect-free target RunPlan in the active Env."""

    base: dict[str, Any] = {
        "schema_version": RUNTIME_SWITCH_PLAN_SCHEMA_VERSION,
        "ts": time.time(),
        "ok": False,
        "read_only": True,
        "dry_run": True,
        "motion": False,
        "publishes": [],
        "inputs": {},
        "from": {},
        "to": {},
        "changed": [],
        "current_validation": _validation(False),
        "target_validation": _validation(False),
        "run_plan": None,
        "control_report": None,
        "operator_command": None,
        "blockers": [],
        "links": {
            "runtime_switch_plan": "/api/v1/runtime/switch-plan",
            "runtime_dataflow": "/api/v1/runtime/dataflow",
            "runtime_contract": "/api/v1/diagnostics/runtime-contract",
            "field_check": "/api/v1/diagnostics/field-check",
        },
        "error": None,
    }
    try:
        raw = _request_mapping(request)
        active_plan = _active_run_plan(gw)
        current_payload = _run_plan_payload(active_plan)
        current_identity = _run_plan_identity(current_payload)
        env = _clean_text(getattr(active_plan, "env", None))
        if env not in {"real", "sim"}:
            raise ValueError("active RunPlan Env is invalid")
        active_product_name = product_name(str(getattr(active_plan, "product", "") or "").strip())
        requested_current = _clean_text(raw.get("current_product"))
        current_product = product_name(requested_current) if requested_current is not None else active_product_name
        target_product = _canonical_product(raw.get("target_product"))
        map_name = _clean_text(raw.get("map_name"))
        initial_pose = _normalize_initial_pose(raw)
        base["inputs"] = {
            "env": env,
            "current_product": current_product,
            "target_product": target_product,
            "current_product_source": ("request" if requested_current is not None else "run_plan"),
            "identity_source": "run_plan",
            "map_name": map_name,
            "relocalize": bool(raw.get("relocalize", True)),
            "initial_pose": list(initial_pose) if initial_pose is not None else None,
        }
        base["from"] = current_payload
        base["operator_command"] = build_operator_command(raw, env=env)

        if current_product != active_product_name:
            message = f"requested current Product {current_product} does not match active RunPlan {active_product_name}"
            base["current_validation"] = _validation(False, message)
            base["blockers"] = [message]
            return base

        active_validation = bool(
            current_identity.get("fingerprint")
            and current_identity.get("product") == active_product_name
            and current_identity.get("env") == env
        )
        base["current_validation"] = _validation(
            active_validation,
            None if active_validation else "active RunPlan identity is incomplete",
        )
        if not active_validation:
            base["blockers"] = list(base["current_validation"]["blockers"])
            return base

        preview_control = control or _preview_control(env)
        if _clean_text(getattr(preview_control, "env", None)) != env:
            raise ValueError("ProductControl Env does not match the active Env")
        target_variant = _target_variant(target_product, map_name)
        target_plan = preview_control.resolve(
            target_product,
            product_variant=target_variant,
        )
        if _clean_text(getattr(target_plan, "env", None)) != env:
            raise ValueError("resolved RunPlan Env does not match the active Env")
        if _clean_text(getattr(target_plan, "product", None)) != target_product:
            raise ValueError("resolved RunPlan Product does not match the request")
        assert_compatible = getattr(target_plan, "assert_compatible", None)
        if callable(assert_compatible):
            assert_compatible(environment=os.environ)
        target_payload = _run_plan_payload(target_plan)
        target_identity = _run_plan_identity(target_payload)
        lifecycle = getattr(target_plan, "lifecycle", {})
        if not isinstance(lifecycle, Mapping):
            raise TypeError("target RunPlan lifecycle must be a mapping")
        map_required = bool(lifecycle.get("requires_map", False))
        map_blocker = f"Product {target_product} requires a map" if map_required and map_name is None else None
        if map_blocker is None and not map_required and map_name is not None:
            map_blocker = f"Product {target_product} does not accept a saved map"
        target_ok = bool(
            map_blocker is None
            and target_identity.get("product") == target_product
            and target_identity.get("env") == env
            and target_identity.get("fingerprint")
        )
        blocker = map_blocker or (None if target_ok else "Assembly did not return a valid target RunPlan")
        base.update(
            {
                "ok": target_ok,
                "to": target_payload,
                "changed": _changed_fields(current_payload, target_payload),
                "target_validation": _validation(target_ok, blocker),
                "run_plan": target_payload,
                "control_report": None,
                "blockers": [] if target_ok else [blocker or "RunPlan preview failed"],
                "error": blocker if not target_ok else None,
            }
        )
    except Exception as exc:
        message = str(exc) or exc.__class__.__name__
        base["error"] = message
        base["blockers"] = [message]
    return base


__all__ = [
    "RUNTIME_SWITCH_PLAN_SCHEMA_VERSION",
    "build_operator_command",
    "build_runtime_switch_plan",
]
