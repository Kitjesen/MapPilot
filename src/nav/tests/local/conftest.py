"""conftest.py for src/nav/tests/local/ -- add repo and src to sys.path."""

import os
import sys
from collections.abc import Iterable
from typing import Any

import pytest

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_src = os.path.join(_repo, "src")

for _p in [_repo, _src]:
    if _p not in sys.path:
        sys.path.insert(0, _p)


_TRUE_VALUES = {"1", "true", "yes", "on"}


def pytest_addoption(parser: pytest.Parser) -> None:
    """Register base_autonomy native-backend validation controls."""
    group = parser.getgroup("lingtu")
    group.addoption(
        "--require-nav-core",
        action="store_true",
        default=False,
        help=(
            "Fail base_autonomy native backend tests when LingTu native navigation kernel is absent "
            "instead of skipping them."
        ),
    )


def pytest_configure(config: pytest.Config) -> None:
    """Document local markers when these tests are collected directly."""
    config.addinivalue_line(
        "markers",
        "native: tests that require LingTu native compiled extensions",
    )


def _requires_native_kernel(config: pytest.Config) -> bool:
    env_value = os.getenv("LINGTU_REQUIRE_NAV_KERNEL", "")
    return bool(config.getoption("--require-nav-core")) or (
        env_value.strip().lower() in _TRUE_VALUES
    )


@pytest.fixture
def require_nav_kernel(request: pytest.FixtureRequest) -> Any:
    """Return LingTu native navigation kernel or skip/fail according to the native validation gate."""

    def _require(symbols: Iterable[str], component: str) -> Any:
        from nav.kernel import try_import_nav_kernel

        nav_kernel = try_import_nav_kernel(tuple(symbols))
        if nav_kernel is not None:
            return nav_kernel

        message = f"LingTu native navigation kernel {component} extension is not available"
        if _requires_native_kernel(request.config):
            pytest.fail(
                f"{message}; required by --require-nav-core or "
                "LINGTU_REQUIRE_NAV_KERNEL=1"
            )
        pytest.skip(message)

    return _require
