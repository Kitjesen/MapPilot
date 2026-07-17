"""Internal route validation contracts.

Product code should import route presets from :mod:`runtime.routes`.
"""

from .loader import REPO_ROOT, ROUTE_CONFIG_DIR, load_route, load_route_contract
from .model import PortBinding, RouteBackend, RouteContract, RouteSpec, TopicContract
from .render import render_route_mermaid
from .routes import ROUTE_PRESETS, replay, robot, route_preset, sim
from .validator import RouteIssue, assert_route_contract_valid, validate_route_contract

__all__ = [
    "REPO_ROOT",
    "ROUTE_CONFIG_DIR",
    "ROUTE_PRESETS",
    "PortBinding",
    "RouteBackend",
    "RouteContract",
    "RouteIssue",
    "RouteSpec",
    "TopicContract",
    "assert_route_contract_valid",
    "load_route",
    "load_route_contract",
    "render_route_mermaid",
    "replay",
    "robot",
    "route_preset",
    "sim",
    "validate_route_contract",
]
