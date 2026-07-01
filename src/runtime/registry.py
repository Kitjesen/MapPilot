"""Plugin registry — register implementations by category + name.

Modules self-register via @register decorator. Blueprints look up by name.
Zero if/else, zero hardcoded imports.

Usage:
    # Register (in the driver module file)
    @register("driver", "nova_dog")
    class HanDogModule(Module, layer=1):
        ...

    @register("driver", "unitree_go2")
    class UnitreeGo2Module(Module, layer=1):
        ...

    # Look up (in blueprint)
    DriverCls = get("driver", "nova_dog")
    handle = autoconnect(DriverCls.blueprint(), ...).build()

    # List available
    list_plugins("driver")  # ["nova_dog", "unitree_go2", "sim", "stub"]

    # Auto-select by platform
    best = auto_select("driver", platform="aarch64")
"""

from __future__ import annotations

import logging
from typing import Any, Callable

logger = logging.getLogger(__name__)

# Global: category -> {name -> (cls, metadata)}
_registry: dict[str, dict[str, tuple]] = {}


def register(
    category: str,
    name: str,
    *,
    priority: int = 0,
    platforms: set[str] | None = None,
    description: str = "",
) -> Callable[[type], type]:
    """Decorator: register a class under category/name.

    Args:
        category: "driver", "detector", "encoder", "planner", "llm", ...
        name: "nova_dog", "yolo_world", "kimi", ...
        priority: higher = preferred in auto_select
        platforms: {"x86_64", "aarch64"} or empty = all
        description: human-readable label
    """
    def decorator(cls: type) -> type:
        _registry.setdefault(category, {})[name] = (cls, {
            "priority": priority,
            "platforms": platforms or set(),
            "description": description,
        })
        return cls
    return decorator


def get(category: str, name: str) -> type:
    """Get registered class by category/name. Raises KeyError if missing."""
    if category not in _registry:
        raise KeyError(f"Unknown category '{category}'. Available: {list(_registry)}")
    if name not in _registry[category]:
        raise KeyError(
            f"Unknown plugin '{category}/{name}'. "
            f"Available: {list(_registry[category])}"
        )
    return _registry[category][name][0]


def list_plugins(category: str) -> list[str]:
    """List all registered names in a category."""
    return sorted(_registry.get(category, {}).keys())


def list_categories() -> list[str]:
    """List all categories."""
    return sorted(_registry.keys())


def auto_select(category: str, platform: str = "") -> str | None:
    """Pick highest-priority plugin, optionally filtered by platform."""
    if category not in _registry:
        return None
    candidates = []
    for name, (_cls, meta) in _registry[category].items():
        if platform and meta["platforms"] and platform not in meta["platforms"]:
            continue
        candidates.append((meta["priority"], name))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return candidates[0][1]


def get_metadata(category: str, name: str) -> dict[str, Any]:
    """Get plugin metadata."""
    if category in _registry and name in _registry[category]:
        return _registry[category][name][1]
    return {}


def clear() -> None:
    """Clear all registrations (for testing only)."""
    _registry.clear()


def snapshot() -> dict[str, dict[str, tuple]]:
    """Return a deep copy of the current registry (for test save/restore)."""
    return {cat: dict(plugins) for cat, plugins in _registry.items()}


def restore(state: dict[str, dict[str, tuple]]) -> None:
    """Restore registry to a previously snapshotted state (for testing only)."""
    _registry.clear()
    for cat, plugins in state.items():
        _registry[cat] = dict(plugins)


def restore_entries(state: dict[str, dict[str, tuple]]) -> None:
    """Restore specific entries without removing registrations added meanwhile."""
    for cat, plugins in state.items():
        current = _registry.setdefault(cat, {})
        for name, entry in plugins.items():
            current[name] = entry
