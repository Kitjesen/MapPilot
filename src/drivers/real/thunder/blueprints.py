"""Compatibility shims for old Thunder blueprint import paths.

New code should import product blueprints from
``lingtu.assembly.products.thunder``. Driver packages own hardware modules only;
product composition lives under ``lingtu.assembly.products``.
"""

from __future__ import annotations

import warnings
from typing import Any

from runtime.blueprint import Blueprint
from lingtu.assembly.products.thunder import (
    thunder_basic_blueprint,
    thunder_nav_blueprint,
)


def _legacy_overrides(
    *,
    dog_host: str,
    dog_port: int,
    values: dict[str, Any],
) -> dict[str, Any]:
    overrides = dict(values)
    overrides.setdefault("dog_host", dog_host)
    overrides.setdefault("dog_port", dog_port)
    return overrides


def _warn_legacy(name: str) -> None:
    warnings.warn(
        (
            f"drivers.real.thunder.blueprints.{name} is deprecated; "
            "use lingtu.assembly.products.thunder instead."
        ),
        DeprecationWarning,
        stacklevel=3,
    )


def thunder_basic(
    dog_host: str = "127.0.0.1",
    dog_port: int = 13145,
    **kw: Any,
) -> Blueprint:
    """Return the minimal Thunder product blueprint through the legacy path."""

    _warn_legacy("thunder_basic")
    return thunder_basic_blueprint(
        **_legacy_overrides(dog_host=dog_host, dog_port=dog_port, values=kw)
    )


def thunder_nav(
    dog_host: str = "127.0.0.1",
    dog_port: int = 13145,
    **kw: Any,
) -> Blueprint:
    """Return the Thunder navigation blueprint through the legacy path."""

    _warn_legacy("thunder_nav")
    overrides = _legacy_overrides(dog_host=dog_host, dog_port=dog_port, values=kw)
    overrides.setdefault("enable_gateway", False)
    return thunder_nav_blueprint(**overrides)


def thunder_semantic(
    dog_host: str = "127.0.0.1",
    dog_port: int = 13145,
    **kw: Any,
) -> Blueprint:
    """Return the full Thunder semantic navigation blueprint through the legacy path."""

    _warn_legacy("thunder_semantic")
    overrides = _legacy_overrides(dog_host=dog_host, dog_port=dog_port, values=kw)
    overrides.setdefault("enable_gateway", True)
    return thunder_nav_blueprint(**overrides)


nova_dog_basic = thunder_basic
nova_dog_nav = thunder_nav
nova_dog_semantic = thunder_semantic
