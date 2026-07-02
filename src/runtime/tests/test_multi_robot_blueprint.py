"""Tests for multi-robot blueprint namespace isolation.

Each robot gets an independent full navigation stack under a unique
``robot_N/`` namespace, merged into a single Blueprint so they can be
built and started together as one system.

The core assertions verify:

1. No duplicate module names after merge (namespace collision)
2. No cross-robot wires (wires stay within their namespace)
3. Wires are correctly prefixed by namespace after application
4. build() successfully creates a SystemHandle with all modules
"""

from __future__ import annotations

import pytest

from runtime.blueprint import Blueprint
from runtime.blueprints.multi_robot import multi_robot_blueprint
from runtime.module import Module

# Avoid SLAM service-manager delays on non-S100P machines.
_FAST_CFG = dict(
    slam_profile="none",
    manage_external_services=False,
)


# ---------------------------------------------------------------------------
# Smoke tests
# ---------------------------------------------------------------------------


def test_multi_robot_blueprint_builds():
    """Minimal multi-robot blueprint builds without errors."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=False,
        enable_semantic=False,
        **_FAST_CFG,
    )
    assert len(bp._entries) >= 4, (
        f"Expected at least 4 entries for 2-stub system, got {len(bp._entries)}"
    )
    # Check no duplicate aliases
    names = [e.name for e in bp._entries]
    assert len(names) == len(set(names)), f"Duplicate aliases: {names}"
    # Each name should be namespaced
    for n in names:
        assert n.startswith("robot_0/") or n.startswith("robot_1/")


def test_multi_robot_full_semantic():
    """Full semantic stack multi-robot builds without name collisions."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=False,
        **_FAST_CFG,
    )
    names = [e.name for e in bp._entries]
    assert len(names) == len(set(names))
    assert len(bp._wires) >= 2, (
        f"Expected at least 2 wires for 2-stub semantic system, got {len(bp._wires)}"
    )


def test_multi_robot_with_gateway():
    """Multi-robot with gateway enabled (port-sharing OK, names must not collide)."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=True,
        gateway_port=5050,
        **_FAST_CFG,
    )
    names = [e.name for e in bp._entries]
    assert len(names) == len(set(names))


# ---------------------------------------------------------------------------
# Wire isolation
# ---------------------------------------------------------------------------


def _check_wire_namespace(bp):
    """Assert all wires stay within their robot namespace."""
    for w in bp._wires:
        ro = w.out_module
        ri = w.in_module
        assert ro.startswith(("robot_0/", "robot_1/")), f"Out module not namespaced: {ro}"
        assert ri.startswith(("robot_0/", "robot_1/")), f"In module not namespaced: {ri}"
        assert ro.split("/")[0] == ri.split("/")[0], (
            f"Cross-robot wire: {ro}.{w.out_port} -> {ri}.{w.in_port}"
        )


def test_multi_robot_wire_isolation_minimal():
    """Minimal stack: all wires are namespace-scoped, no cross-robot leaks."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=False,
        enable_semantic=False,
        **_FAST_CFG,
    )
    _check_wire_namespace(bp)


def test_multi_robot_wire_isolation_semantic():
    """Semantic stack: all wires are namespace-scoped."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=False,
        **_FAST_CFG,
    )
    _check_wire_namespace(bp)


def test_multi_robot_wire_isolation_full():
    """Full stack (gateway included): all wires are namespace-scoped."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=True,
        gateway_port=5050,
        **_FAST_CFG,
    )
    _check_wire_namespace(bp)


# ---------------------------------------------------------------------------
# Single robot edge case
# ---------------------------------------------------------------------------


def test_single_robot():
    """Multi-robot API with a single robot returns a valid Blueprint."""
    bp = multi_robot_blueprint(
        ["stub"],
        llm="mock",
        enable_gateway=False,
        enable_semantic=False,
        **_FAST_CFG,
    )
    names = [e.name for e in bp._entries]
    assert len(names) == len(set(names))
    assert all(n.startswith("robot_0/") for n in names)
    for w in bp._wires:
        assert w.out_module.startswith("robot_0/")
        assert w.in_module.startswith("robot_0/")


# ---------------------------------------------------------------------------
# Different robot types
# ---------------------------------------------------------------------------


def test_multi_robot_same_type_twice():
    """Two robots of identical type still produce unique namespaced entries."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=False,
        enable_semantic=False,
        **_FAST_CFG,
    )
    names = [e.name for e in bp._entries]
    assert len(names) == len(set(names))
    _check_wire_namespace(bp)


# ---------------------------------------------------------------------------
# Three robots
# ---------------------------------------------------------------------------


def test_three_robots():
    """Three robot namespaces all remain isolated."""
    bp = multi_robot_blueprint(
        ["stub", "stub", "stub"],
        llm="mock",
        enable_gateway=False,
        enable_semantic=False,
        **_FAST_CFG,
    )
    names = [e.name for e in bp._entries]
    assert len(names) == len(set(names))
    for n in names:
        assert any(n.startswith(f"robot_{i}/") for i in range(3))
    for w in bp._wires:
        ro, ri = w.out_module, w.in_module
        assert ro.split("/")[0] == ri.split("/")[0]


# ---------------------------------------------------------------------------
# Merge collision detection
# ---------------------------------------------------------------------------


def test_merge_collision_detection():
    """merge() raises ValueError when two blueprints have the same bare name."""
    bp1 = Blueprint()

    class _TestMod(Module):
        pass

    bp1.add(_TestMod, alias="TestModule")

    bp2 = Blueprint()
    bp2.add(_TestMod, alias="TestModule")

    with pytest.raises(ValueError, match="exists in both"):
        bp1.merge(bp2)


# ---------------------------------------------------------------------------
# Build and instantiate (stub-only, no hardware needed)
# ---------------------------------------------------------------------------


def test_multi_robot_build_and_start():
    """Build a multi-robot system and verify both robots' modules are reachable."""
    bp = multi_robot_blueprint(
        ["stub", "stub"],
        llm="mock",
        enable_gateway=False,
        enable_semantic=False,
        **_FAST_CFG,
    )
    system = bp.build()
    system.start()

    nav_0 = system.get_module("robot_0/nav.mission")
    nav_1 = system.get_module("robot_1/nav.mission")
    assert nav_0 is not None
    assert nav_1 is not None
    assert nav_0 is not nav_1  # different instances

    # Verify all modules are accessible
    for entry in bp._entries:
        mod = system.get_module(entry.name)
        assert mod is not None

    system.stop()
