"""Generic registry plugin import seeding utilities.

Core owns the registry mechanics, not LingTu's product plugin catalog. Runtime
or product packages pass their catalog into :func:`seed_plugin_modules`.
"""

from __future__ import annotations

import importlib
import logging
import sys
from collections.abc import Iterable, Mapping

from core.registry import restore_entries, snapshot

logger = logging.getLogger(__name__)


PluginModuleCatalog = Mapping[str, tuple[str, ...]]
_CatalogEntry = tuple[PluginModuleCatalog, tuple[str, ...]]

_REGISTERED_PLUGIN_CATALOGS: dict[str, _CatalogEntry] = {}


def register_plugin_module_catalog(
    name: str,
    plugin_modules: PluginModuleCatalog,
    *,
    default_groups: Iterable[str] = (),
    replace: bool = True,
) -> None:
    """Register a product/runtime plugin catalog with the core seed hook.

    Core keeps the registry mechanics here, while product/runtime packages own
    the concrete module catalog and register it during startup.
    """

    catalog_name = str(name or "").strip()
    if not catalog_name:
        raise ValueError("plugin catalog name must be non-empty")
    if not replace and catalog_name in _REGISTERED_PLUGIN_CATALOGS:
        raise ValueError(f"plugin catalog already registered: {catalog_name}")

    _REGISTERED_PLUGIN_CATALOGS[catalog_name] = (
        plugin_modules,
        tuple(default_groups),
    )


def registered_plugin_catalog_names() -> tuple[str, ...]:
    """Return registered plugin catalog names."""

    return tuple(sorted(_REGISTERED_PLUGIN_CATALOGS))


def seed_registered_plugins(
    groups: Iterable[str] | None = None,
    *,
    catalog_names: Iterable[str] | None = None,
    reload_loaded: bool = False,
    strict: bool = False,
) -> dict[str, dict[str, list[str]]]:
    """Seed plugins from registered product/runtime catalogs.

    Args:
        groups: Seed groups to import. ``None`` imports each catalog's default
            groups.
        catalog_names: Optional registered catalog names to use. ``None`` uses
            all registered catalogs.
        reload_loaded: Re-run decorators for already imported modules.
        strict: Re-raise import errors instead of recording them.
    """

    selected_names = (
        tuple(catalog_names)
        if catalog_names is not None
        else tuple(_REGISTERED_PLUGIN_CATALOGS)
    )
    requested = tuple(groups) if groups is not None else None
    report: dict[str, dict[str, list[str]]] = {"loaded": {}, "failed": {}}
    if not selected_names:
        return report

    unknown_catalogs = [
        name for name in selected_names if name not in _REGISTERED_PLUGIN_CATALOGS
    ]
    if unknown_catalogs:
        available = ", ".join(registered_plugin_catalog_names())
        raise ValueError(
            f"Unknown plugin catalog(s): {unknown_catalogs}. Available: {available}"
        )

    seeded_groups: set[str] = set()

    for catalog_name in selected_names:
        plugin_modules, default_groups = _REGISTERED_PLUGIN_CATALOGS[catalog_name]
        catalog_groups = (
            tuple(group for group in requested if group in plugin_modules)
            if requested is not None
            else default_groups
        )
        if not catalog_groups:
            continue

        seeded_groups.update(catalog_groups)
        catalog_report = seed_plugin_modules(
            plugin_modules,
            catalog_groups,
            default_groups=default_groups,
            reload_loaded=reload_loaded,
            strict=strict,
        )
        for kind in ("loaded", "failed"):
            for group, values in catalog_report[kind].items():
                report[kind].setdefault(group, []).extend(values)

    if requested is not None:
        missing = [group for group in requested if group not in seeded_groups]
        if missing:
            available_groups = sorted(
                {
                    group
                    for catalog_name in selected_names
                    for group in _REGISTERED_PLUGIN_CATALOGS[catalog_name][0]
                }
            )
            available = ", ".join(available_groups)
            raise ValueError(
                f"Unknown plugin seed group(s): {missing}. Available: {available}"
            )

    return report


def seed_plugin_modules(
    plugin_modules: PluginModuleCatalog,
    groups: Iterable[str] | None = None,
    *,
    default_groups: Iterable[str] = (),
    reload_loaded: bool = False,
    strict: bool = False,
) -> dict[str, dict[str, list[str]]]:
    """Import plugin modules so their ``@register`` decorators run.

    Args:
        plugin_modules: Mapping of seed group names to importable module names.
        groups: Groups to import. ``None`` imports ``default_groups``.
        default_groups: Groups used when ``groups`` is ``None``.
        reload_loaded: Re-run decorators for already imported modules. This is
            mainly useful after tests call ``core.registry.clear()``.
        strict: Re-raise import errors instead of recording them.

    Returns:
        ``{"loaded": {group: [module, ...]}, "failed": {group: ["mod: err"]}}``.
    """

    requested = tuple(groups) if groups is not None else tuple(default_groups)
    unknown = [group for group in requested if group not in plugin_modules]
    if unknown:
        available = ", ".join(sorted(plugin_modules))
        raise ValueError(f"Unknown plugin seed group(s): {unknown}. Available: {available}")

    preserved_entries = snapshot()
    report: dict[str, dict[str, list[str]]] = {"loaded": {}, "failed": {}}
    try:
        for group in requested:
            for module_name in plugin_modules[group]:
                try:
                    if reload_loaded and module_name in sys.modules:
                        importlib.reload(sys.modules[module_name])
                    else:
                        importlib.import_module(module_name)
                    report["loaded"].setdefault(group, []).append(module_name)
                except Exception as exc:
                    message = f"{module_name}: {exc.__class__.__name__}: {exc}"
                    report["failed"].setdefault(group, []).append(message)
                    if strict:
                        raise
                    logger.debug("Plugin seed failed for %s", module_name, exc_info=True)
    finally:
        restore_entries(preserved_entries)
    return report
