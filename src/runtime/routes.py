"""Public runtime route presets.

Use these with ``Blueprint.route_contract(...)`` for metadata-only contracts.
Reserve ``Blueprint.routed_delivery(...)`` for deliberate routed transport.
The implementation details live under ``runtime.route_contract`` and are only
for contract validation.
"""

from __future__ import annotations

from runtime.route_contract.routes import replay, robot, sim

__all__ = ["replay", "robot", "sim"]
