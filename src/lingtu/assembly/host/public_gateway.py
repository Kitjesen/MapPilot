"""Composition bridge for the public Gateway entry point.

The ProductControl transport owns lifecycle and listeners, while the Gateway
package owns its application implementation.  Keep that cross-layer import in
the Host assembly composition layer instead of leaking it into the lifecycle
controller.
"""

from __future__ import annotations

from typing import Any


def build_public_gateway_app(gateway_port: int, *, control_routes: Any) -> Any:
    """Build the public Gateway app with ProductControl routes attached."""

    from gateway.entry import build_entry_app

    return build_entry_app(gateway_port, control_routes=control_routes)
