"""Shared @skill discovery used by protocol adapters."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any


def discover_skills(
    modules: dict[str, Any],
    *,
    exclude: Iterable[str] = (),
    fallback_owner: Any = None,
    fallback_names: Iterable[str] = (),
) -> tuple[dict[str, Any], list[dict]]:
    """Bind unique skill owners after the caller's name exclusions.

    Only an explicit fallback owner/name pair may yield to one other owner.
    The caller owns exposure policy; runtime knows no domain-specific names.
    """
    excluded = set(exclude)
    candidates: dict[str, dict[int, dict[str, Any]]] = {}
    for module_name, module in sorted(modules.items()):
        if not hasattr(module, "get_skill_infos"):
            continue
        for info in module.get_skill_infos():
            if info.func_name in excluded:
                continue
            method = getattr(module, info.func_name, None)
            if method is None:
                continue
            owner = candidates.setdefault(info.func_name, {}).get(id(module))
            if owner is None:
                candidates[info.func_name][id(module)] = {
                    "module": module,
                    "module_names": {module_name},
                    "method": method,
                    "info": info,
                }
            else:
                owner["module_names"].add(module_name)

    registry: dict[str, Any] = {}
    tools: list[dict] = []
    names = set(fallback_names)
    for name in sorted(candidates):
        entries = list(candidates[name].values())
        if len(entries) == 1:
            selected = entries[0]
        elif (
            fallback_owner is not None
            and name in names
            and len(entries) == 2
            and any(e["module"] is fallback_owner for e in entries)
        ):
            selected = next(e for e in entries if e["module"] is not fallback_owner)
        else:
            owners = sorted("/".join(sorted(e["module_names"])) for e in entries)
            raise ValueError(f"duplicate skill '{name}' from modules: {', '.join(owners)}")
        registry[name] = selected["method"]
        tools.append(selected["info"].to_tool())
    return registry, tools
