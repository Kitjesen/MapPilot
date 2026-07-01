from __future__ import annotations

import ast
import sys
import types
import unittest
from pathlib import Path


class _FakeSystem:
    def __init__(self) -> None:
        self.modules = {"nav.mission": object()}
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True


class _FakeBuilder:
    def __init__(self, system: _FakeSystem) -> None:
        self._system = system

    def build(self, transport=None) -> _FakeSystem:
        del transport
        return self._system


def test_robot_nav_uses_product_profile_builder(monkeypatch):
    import runtime.blueprints.products as products_mod
    from lingtu import Robot

    case = unittest.TestCase()
    captured: dict[str, object] = {}
    fake_system = _FakeSystem()

    fake_full_stack = types.ModuleType("runtime.blueprints.full_stack")

    def fail_full_stack_blueprint(**kwargs):
        raise AssertionError("Robot SDK should not use full_stack for product profiles")

    def fake_thunder_blueprint(config=None, **overrides):
        resolved = dict(config or {})
        resolved.update(overrides)
        captured["config"] = resolved
        return _FakeBuilder(fake_system)

    fake_full_stack.full_stack_blueprint = fail_full_stack_blueprint
    monkeypatch.setitem(sys.modules, "runtime.blueprints.full_stack", fake_full_stack)
    monkeypatch.setattr(products_mod, "thunder_blueprint", fake_thunder_blueprint)

    robot = Robot("nav", llm="mock").start()

    case.assertIs(robot.system, fake_system)
    case.assertTrue(fake_system.started)
    case.assertEqual(captured["config"]["robot"], "thunder")
    case.assertEqual(captured["config"]["slam_profile"], "localizer")
    case.assertEqual(captured["config"]["llm"], "mock")


def test_robot_start_uses_local_runtime_boundary(monkeypatch):
    import lingtu.runtime as runtime_mod
    from lingtu import Robot

    case = unittest.TestCase()
    captured: dict[str, object] = {}
    fake_system = _FakeSystem()

    def fake_build_system(profile, *, overrides=None, **kwargs):
        captured["profile"] = profile
        captured["overrides"] = dict(overrides or {})
        captured["kwargs"] = kwargs
        return fake_system

    monkeypatch.setattr(runtime_mod, "build_system", fake_build_system)

    robot = Robot("nav", llm="mock").start()

    case.assertIs(robot.system, fake_system)
    case.assertTrue(fake_system.started)
    case.assertEqual(captured, {
        "profile": "nav",
        "overrides": {"llm": "mock"},
        "kwargs": {},
    })


def test_robot_facade_does_not_import_full_stack_directly():
    case = unittest.TestCase()
    root = Path(__file__).resolve().parents[3]
    path = root / "src" / "lingtu" / "robot.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    case.assertNotIn("runtime.blueprints.full_stack", imports)
    case.assertNotIn("runtime.blueprints.profile_builder", imports)
    case.assertNotIn("runtime.profiles.resolver", imports)
