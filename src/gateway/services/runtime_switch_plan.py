"""Read-only runtime switch-plan builder for Gateway/App clients."""

from __future__ import annotations

import os
import time
from collections.abc import Mapping
from typing import Any

from runtime.blueprints.profile_graph import resolve_profile_config
from runtime.profiles.endpoints import resolve_runtime_run_spec
from runtime.profiles.product_mode_contracts import (
    PRODUCT_MODE_CONTRACTS,
    product_mode_switch_plan,
)
from runtime.profiles.resolver import canonical_profile_name
from runtime.runtime_switch import compare_runtime_switch, validate_runtime_switch


RUNTIME_SWITCH_PLAN_SCHEMA_VERSION = "lingtu.runtime_switch_plan.v1"


def _clean_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _request_mapping(request: Any) -> dict[str, Any]:
    if request is None:
        return {}
    if isinstance(request, Mapping):
        return dict(request)
    if hasattr(request, "model_dump"):
        return dict(request.model_dump())
    return {
        key: getattr(request, key)
        for key in (
            "current_profile",
            "target_profile",
            "current_endpoint",
            "target_endpoint",
            "endpoint",
        )
        if hasattr(request, key)
    }


def _validation_payload(validation: Any) -> dict[str, Any]:
    return {
        "ok": bool(validation.ok),
        "blockers": list(validation.blockers),
        "warnings": list(validation.warnings),
    }


def _resolve_profile_inputs(raw: Mapping[str, Any]) -> dict[str, Any]:
    current_profile = _clean_text(raw.get("current_profile"))
    current_profile_requested = current_profile is not None
    target_profile = _clean_text(raw.get("target_profile")) or "explore"
    current_endpoint = _clean_text(raw.get("current_endpoint"))
    target_endpoint = _clean_text(raw.get("target_endpoint")) or _clean_text(
        raw.get("endpoint")
    )

    current_profile_source = "query" if current_profile else "env"
    if not current_profile:
        current_profile = _clean_text(os.environ.get("LINGTU_PROFILE"))
    if not current_profile:
        current_profile = "sim_mujoco_live"
        current_profile_source = "default"
    current_profile = canonical_profile_name(current_profile)
    target_profile = canonical_profile_name(target_profile)

    current_endpoint_source = "query" if current_endpoint else "profile_default"
    if not current_endpoint:
        env_profile = _clean_text(os.environ.get("LINGTU_PROFILE"))
        env_endpoint = _clean_text(os.environ.get("LINGTU_ENDPOINT"))
        if env_endpoint and (not current_profile_requested or current_profile == env_profile):
            current_endpoint = env_endpoint
            current_endpoint_source = "env"

    return {
        "current_profile": current_profile,
        "target_profile": target_profile,
        "current_endpoint": current_endpoint,
        "target_endpoint": target_endpoint,
        "endpoint": target_endpoint,
        "current_profile_source": current_profile_source,
        "current_endpoint_source": current_endpoint_source,
        "target_endpoint_source": "query" if target_endpoint else "profile_default",
    }


def build_runtime_switch_plan(request: Any = None) -> dict[str, Any]:
    """Build a dry-run sim/replay/real switch plan without touching Module ports."""

    raw = _request_mapping(request)
    inputs = _resolve_profile_inputs(raw)
    now = time.time()
    base: dict[str, Any] = {
        "schema_version": RUNTIME_SWITCH_PLAN_SCHEMA_VERSION,
        "ts": now,
        "ok": False,
        "read_only": True,
        "dry_run": True,
        "motion": False,
        "publishes": [],
        "lifecycle": "dry_run_preflight",
        "inputs": inputs,
        "from": {},
        "to": {},
        "changed": [],
        "current_validation": {"ok": False, "blockers": [], "warnings": []},
        "target_validation": {"ok": False, "blockers": [], "warnings": []},
        "blockers": [],
        "links": {
            "runtime_switch_plan": "/api/v1/runtime/switch-plan",
            "runtime_dataflow": "/api/v1/runtime/dataflow",
            "runtime_contract": "/api/v1/diagnostics/runtime-contract",
            "field_check": "/api/v1/diagnostics/field-check",
        },
    }

    try:
        current_config = resolve_profile_config(
            str(inputs["current_profile"]),
            runtime_endpoint=inputs.get("current_endpoint"),
        )
        target_config = resolve_profile_config(
            str(inputs["target_profile"]),
            runtime_endpoint=inputs.get("target_endpoint"),
        )
        current_spec = resolve_runtime_run_spec(
            str(inputs["current_profile"]),
            current_config,
        )
        target_spec = resolve_runtime_run_spec(
            str(inputs["target_profile"]),
            target_config,
        )
        current_validation = validate_runtime_switch(current_spec)
        target_validation = validate_runtime_switch(target_spec)
        payload = compare_runtime_switch(current_spec, target_spec)
        current_payload = _validation_payload(current_validation)
        target_payload = _validation_payload(target_validation)
        blockers = [
            f"current runtime boundary: {blocker}"
            for blocker in current_payload["blockers"]
        ] + [
            f"target runtime boundary: {blocker}"
            for blocker in target_payload["blockers"]
        ]
        base.update(payload)
        product_switch = None
        if (
            str(inputs["current_profile"]) in PRODUCT_MODE_CONTRACTS
            and str(inputs["target_profile"]) in PRODUCT_MODE_CONTRACTS
        ):
            product_switch = product_mode_switch_plan(
                str(inputs["current_profile"]),
                str(inputs["target_profile"]),
            )
        base.update(
            {
                "ok": current_validation.ok and target_validation.ok,
                "current_validation": current_payload,
                "target_validation": target_payload,
                "blockers": blockers,
                "product_mode_switch": product_switch,
            }
        )
        return base
    except Exception as exc:
        message = str(exc) or exc.__class__.__name__
        base["error"] = message
        base["blockers"] = [f"runtime switch plan unavailable: {message}"]
        return base
