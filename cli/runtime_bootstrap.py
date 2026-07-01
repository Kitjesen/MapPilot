"""Runtime bootstrap hooks for the CLI entrypoint."""

from __future__ import annotations


def install_runtime_plugin_catalog() -> None:
    """Install built-in plugin catalog definitions for CLI launches."""

    try:
        from lingtu.plugin_seed import install_builtin_plugin_catalog
    except ModuleNotFoundError as exc:
        if exc.name != "lingtu.plugin_seed":
            raise
        return
    install_builtin_plugin_catalog()


def seed_runtime_plugins(*, groups: tuple[str, ...], reload_loaded: bool = False) -> None:
    """Seed registry plugins from the core catalog on demand."""

    from runtime.plugin_seed import seed_registered_plugins

    seed_registered_plugins(groups=groups, reload_loaded=reload_loaded)
