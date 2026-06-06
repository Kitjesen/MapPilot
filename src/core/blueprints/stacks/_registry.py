"""Registry helpers for stack factories."""

from __future__ import annotations

import importlib
from typing import Any

from core.plugin_seed import seed_builtin_plugins
from core.registry import get


def stack_module(
    category: str,
    name: str,
    *,
    seed_group: str,
    fallback: str,
) -> type[Any]:
    """Resolve a stack module from the registry with import fallback.

    Tests and product configs can register a replacement before calling a stack
    factory. Built-in modules are seeded only when that registry entry is absent.
    """
    try:
        return get(category, name)
    except KeyError:
        pass

    seed_builtin_plugins(groups=(seed_group,), reload_loaded=False)
    try:
        return get(category, name)
    except KeyError:
        module_name, attr = fallback.rsplit(".", 1)
        module = importlib.import_module(module_name)
        return getattr(module, attr)


def optional_stack_module(
    category: str,
    name: str,
    *,
    seed_group: str,
    fallback: str,
) -> type[Any] | None:
    """Resolve an optional stack module, returning None when unavailable."""
    try:
        return stack_module(
            category,
            name,
            seed_group=seed_group,
            fallback=fallback,
        )
    except (ImportError, AttributeError, KeyError):
        return None


def optional_fallback_module(
    category: str,
    name: str,
    *,
    fallback: str,
) -> type[Any] | None:
    """Resolve an optional module without seeding the whole plugin group."""
    try:
        return get(category, name)
    except KeyError:
        pass

    try:
        module_name, attr = fallback.rsplit(".", 1)
        module = importlib.import_module(module_name)
        return getattr(module, attr)
    except (ImportError, AttributeError, KeyError):
        return None
